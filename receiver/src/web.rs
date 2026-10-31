//! Web UI: the page, and a WebSocket that streams live samples to it.
//!
//! Only our own page may use it: the server listens on loopback, but any
//! site open in the browser could otherwise read the signal over a
//! WebSocket (which CORS does not cover) or start a recording with a
//! cross-site POST. See [`check_origin`].

use std::io;
use std::net::SocketAddr;
use std::sync::{Arc, mpsc};

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Request, State};
use axum::http::{HeaderValue, StatusCode, header};
use axum::middleware::{self, Next};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use tokio::net::TcpListener;
use tokio::sync::broadcast::{self, error::RecvError};

use crate::device::Command;
use crate::discovery::{self, Device};
use crate::live::Update;
use crate::recorder;

/// Built into the binary, so the receiver runs from any directory.
const INDEX_HTML: &str = include_str!("../ui/index.html");

/// Shared by all requests; every field is cheap to clone.
#[derive(Clone)]
struct AppState {
    updates: broadcast::Sender<Update>,
    commands: mpsc::Sender<Command>,
    /// Where our own page is served from.
    allowed_origins: Arc<[String]>,
}

pub async fn serve(
    listener: TcpListener,
    updates: broadcast::Sender<Update>,
    commands: mpsc::Sender<Command>,
) -> io::Result<()> {
    let port = listener.local_addr()?.port();
    let state = AppState {
        updates,
        commands,
        allowed_origins: [
            format!("http://127.0.0.1:{port}"),
            format!("http://localhost:{port}"),
        ]
        .into(),
    };
    let app = Router::new()
        .route("/", get(index))
        .route("/ws", get(websocket))
        .route("/api/record/start", post(start_recording))
        .route("/api/record/stop", post(stop_recording))
        .route("/api/mark", post(mark))
        .route("/api/discover", post(discover))
        .route("/api/connect", post(connect))
        .route("/api/disconnect", post(disconnect))
        // On the whole router, so a route added later is covered too.
        .layer(middleware::from_fn_with_state(state.clone(), check_origin))
        .with_state(state);
    axum::serve(listener, app).await
}

/// Rejects requests made by pages of other sites.
///
/// Browsers send `Origin` with every WebSocket handshake and every
/// cross-site POST, and a page cannot forge it. That also covers DNS
/// rebinding: a hostile domain pointed at 127.0.0.1 still carries its own
/// name. Requests without `Origin` do not come from a page (curl, scripts),
/// so no other site can be behind them.
async fn check_origin(State(state): State<AppState>, request: Request, next: Next) -> Response {
    let origin = request.headers().get(header::ORIGIN);
    if origin_allowed(origin, &state.allowed_origins) {
        next.run(request).await
    } else {
        StatusCode::FORBIDDEN.into_response()
    }
}

fn origin_allowed(origin: Option<&HeaderValue>, allowed: &[String]) -> bool {
    match origin {
        None => true,
        Some(origin) => allowed.iter().any(|a| origin.as_bytes() == a.as_bytes()),
    }
}

async fn index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

async fn websocket(upgrade: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    let receiver = state.updates.subscribe();
    upgrade.on_upgrade(move |socket| stream_updates(socket, receiver))
}

async fn start_recording(State(state): State<AppState>) -> StatusCode {
    command(&state, Command::Record(recorder::Command::Start))
}

async fn stop_recording(State(state): State<AppState>) -> StatusCode {
    command(&state, Command::Record(recorder::Command::Stop))
}

/// Answers once the devices had their time to reply (under a second).
async fn discover() -> Result<Json<Vec<Device>>, (StatusCode, String)> {
    match discovery::discover().await {
        Ok(devices) => Ok(Json(devices)),
        Err(e) => Err((StatusCode::INTERNAL_SERVER_ERROR, e.to_string())),
    }
}

#[derive(Deserialize)]
struct ConnectRequest {
    address: SocketAddr,
}

async fn connect(State(state): State<AppState>, Json(request): Json<ConnectRequest>) -> StatusCode {
    command(&state, Command::Connect(request.address))
}

async fn disconnect(State(state): State<AppState>) -> StatusCode {
    command(&state, Command::Disconnect)
}

#[derive(Deserialize)]
struct MarkRequest {
    #[serde(default)]
    label: String,
}

/// `Json` also demands `Content-Type: application/json`, which a page of
/// another site cannot send without a CORS preflight: one more barrier
/// besides the Origin check.
async fn mark(State(state): State<AppState>, Json(request): Json<MarkRequest>) -> StatusCode {
    command(
        &state,
        Command::Record(recorder::Command::Mark(request.label)),
    )
}

/// Accepted, not done: the device thread carries it out within ~50 ms and
/// reports the outcome to every tab as a `recording` update.
fn command(state: &AppState, command: Command) -> StatusCode {
    match state.commands.send(command) {
        Ok(()) => StatusCode::ACCEPTED,
        Err(_) => StatusCode::SERVICE_UNAVAILABLE,
    }
}

/// Runs for one browser tab until it closes.
async fn stream_updates(mut socket: WebSocket, mut updates: broadcast::Receiver<Update>) {
    if send(&mut socket, &Update::hello()).await.is_err() {
        return;
    }
    loop {
        tokio::select! {
            update = updates.recv() => match update {
                Ok(update) => {
                    if send(&mut socket, &update).await.is_err() {
                        return;
                    }
                }
                // This tab fell behind and lost the oldest batches; the
                // device thread and the other tabs are not affected. The
                // page sees the jump in `first_index` and draws a gap.
                Err(RecvError::Lagged(_)) => {}
                Err(RecvError::Closed) => return,
            },
            // Notices a closed tab even while the device sends nothing.
            message = socket.recv() => {
                if !matches!(message, Some(Ok(_))) {
                    return;
                }
            }
        }
    }
}

async fn send(socket: &mut WebSocket, update: &Update) -> Result<(), axum::Error> {
    let json = serde_json::to_string(update).expect("updates always serialize");
    socket.send(Message::Text(json.into())).await
}

#[cfg(test)]
mod tests {
    use super::*;

    fn allowed() -> Vec<String> {
        vec![
            "http://127.0.0.1:8080".into(),
            "http://localhost:8080".into(),
        ]
    }

    #[test]
    fn accepts_our_page_and_non_browser_clients() {
        let ours = HeaderValue::from_static("http://localhost:8080");
        assert!(origin_allowed(Some(&ours), &allowed()));
        assert!(origin_allowed(None, &allowed()));
    }

    #[test]
    fn rejects_other_sites() {
        for origin in [
            "https://example.com",
            // Same host, another port: another origin.
            "http://127.0.0.1:3000",
            // A prefix match would let this one in.
            "http://localhost:8080.evil.example",
            // Sent by sandboxed iframes and some redirects.
            "null",
        ] {
            let origin = HeaderValue::from_static(origin);
            assert!(!origin_allowed(Some(&origin), &allowed()), "{origin:?}");
        }
    }
}
