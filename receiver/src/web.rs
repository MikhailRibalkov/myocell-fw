//! Web UI: the page, and a WebSocket that streams live samples to it.

use std::io;
use std::sync::mpsc;

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::http::StatusCode;
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use tokio::net::TcpListener;
use tokio::sync::broadcast::{self, error::RecvError};

use crate::live::Update;
use crate::recorder::Command;

/// Built into the binary, so the receiver runs from any directory.
const INDEX_HTML: &str = include_str!("../ui/index.html");

/// Shared by all requests; both halves are cheap to clone.
#[derive(Clone)]
struct AppState {
    updates: broadcast::Sender<Update>,
    commands: mpsc::Sender<Command>,
}

pub async fn serve(
    listener: TcpListener,
    updates: broadcast::Sender<Update>,
    commands: mpsc::Sender<Command>,
) -> io::Result<()> {
    let app = Router::new()
        .route("/", get(index))
        .route("/ws", get(websocket))
        .route("/api/record/start", post(start_recording))
        .route("/api/record/stop", post(stop_recording))
        .with_state(AppState { updates, commands });
    axum::serve(listener, app).await
}

async fn index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

async fn websocket(upgrade: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    let receiver = state.updates.subscribe();
    upgrade.on_upgrade(move |socket| stream_updates(socket, receiver))
}

async fn start_recording(State(state): State<AppState>) -> StatusCode {
    command(&state, Command::Start)
}

async fn stop_recording(State(state): State<AppState>) -> StatusCode {
    command(&state, Command::Stop)
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
