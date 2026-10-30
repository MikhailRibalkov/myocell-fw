//! Web UI: the page, and a WebSocket that streams live samples to it.

use std::io;

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::{Html, IntoResponse};
use axum::routing::get;
use tokio::net::TcpListener;
use tokio::sync::broadcast::{self, error::RecvError};

use crate::live::Update;

/// Built into the binary, so the receiver runs from any directory.
const INDEX_HTML: &str = include_str!("../ui/index.html");

pub async fn serve(listener: TcpListener, updates: broadcast::Sender<Update>) -> io::Result<()> {
    let app = Router::new()
        .route("/", get(index))
        .route("/ws", get(websocket))
        .with_state(updates);
    axum::serve(listener, app).await
}

async fn index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

async fn websocket(
    upgrade: WebSocketUpgrade,
    State(updates): State<broadcast::Sender<Update>>,
) -> impl IntoResponse {
    let receiver = updates.subscribe();
    upgrade.on_upgrade(move |socket| stream_updates(socket, receiver))
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
