use socketcan::{CanFrame, Socket, tokio::CanSocket};
use std::sync::mpsc::{Receiver, Sender, channel};
use tokio::sync::CancellationToken;

use crate::protocol::{CanDriveMessage, Commands, ProtocolData};

struct CommManager {
    can_txrx: IPCChannel<CanDriveMessage>,
    socket: CanSocket,
    cancellation_token: CancellationToken,
}

impl CommManager {
    pub fn new(port: String, cancellation_token: CancellationToken) -> CommManager {
        return CommManager {
            can_txrx: IPCChannel::new(),
            socket: CanSocket::open(port).unwrap(),
            cancellation_token,
        };
    }

    pub fn get_channel(&self) -> (Sender<CanDriveMessage>, Receiver<CanDriveMessage>) {
        return self.can_txrx.get_channel();
    }

    pub async fn send(&self, msg: CanDriveMessage) -> Result<(), std::sync::mpsc::SendError<CanDriveMessage>>
    {
        self.can_txrx.send(msg)
    }

    pub async fn broadcast(&self, cmd: Commands, data: ProtocolData) -> Result<(), std::sync::mpsc::SendError<CanDriveMessage>>
    {
        let msg = CanDriveMessage {
            id: 0, // Broadcast ID
            command: cmd,
            data,
        };
        self.can_txrx.send(msg)
    }

    pub async fn start(&mut self) -> Vec<tokio::JoinHandle<()>> {
        let threads = Vec::new();
        threads.push(tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = self.cancellation_token.cancelled() => {
                        break;
                    },
                    data = self.can_txrx.recv() => {
                        if let Some(data) = data {
                            self.socket.send(CanFrame::Data(data.id, data.as_bytes())).await?;
                        }
                    },
                };
            }
        }));
        threads.push(tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = self.cancellation_token.cancelled() => {
                        break;
                    },
                    _ = self.socket.next() => {
                        if let Some(Ok(frame)) = frame {
                            if matches!(frame, CanFrame::Data(_)) {
                                if let Some(msg) = match frame.id {
                                    Standard(id) => protocol::CanDriveMessage::parse_can_frame(id, &frame.data),
                                    _ => None,
                                    // Not supported/not for us
                                } {
                                    self.incoming_tx.send(msg).await?;
                                }
                            }
                        }
                    },
                };
            }
        }));
        threads
    }
}
