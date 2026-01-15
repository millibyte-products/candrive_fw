use socketcan::{NonBlockingCan, EmbeddedFrame, Id, Socket, StandardId};
use socketcan::{CanFrame, CanSocket};
use crate::protocol::{self, DISCOVERY_ASSIGN_ID, DISCOVERY_REQUEST_ID};
use crate::util::Threaded;
use std::io::ErrorKind;
use crate::{protocol::{CanDriveMessage}};
use crossbeam_channel::{unbounded, Receiver, Sender};

pub struct CommManager {
    thread: Option<std::thread::JoinHandle<()>>,
    sink: Sender<CanDriveMessage>,
    source: Receiver<CanDriveMessage>,
}

impl CommManager {
    pub fn new(port: String) -> CommManager {
        let (device_message_sink, can_tx_message_queue) = unbounded();
        let (can_rx_message_queue, device_message_source) = unbounded();
        let handle = std::thread::spawn(move || {
            let mut socket = match CanSocket::open(port.as_str()) {
                Ok(socket) => socket,
                Err(e) => {
                    eprintln!("Error opening socket: {:?}", e);
                    return;
                }
            };
            socket.set_nonblocking(true);

            loop {
                match can_tx_message_queue.recv()
                {
                    Ok(msg) => {
                        let id = match msg {
                            CanDriveMessage::DiscoveryReq { .. } => {
                                DISCOVERY_REQUEST_ID
                            },
                            CanDriveMessage::DiscoveryAssign { .. } => {
                                DISCOVERY_ASSIGN_ID
                            },
                            CanDriveMessage::Control { id, .. } =>{
                                id.to_can_id()
                            }
                            _ => {
                                return;
                            }
                        };
                        let frame = CanFrame::new(StandardId::new(id).unwrap(),
                    &msg.as_bytes()).unwrap();
                        match socket.write_frame(&frame)
                        {
                            Ok(_) => (),
                            Err(e) => {
                                if e.kind() == ErrorKind::WouldBlock {
                                    eprintln!("Error sending to tx queue: {:?}", e);
                                }
                            }
                        }
                    },
                    Err(e) => {
                        eprintln!("Error sending to tx queue: {:?}", e);
                    }
                }
                match socket.receive() {
                    Ok(frame) => {
                        if let Some(id) = match frame.id() {
                            Id::Standard(id) => Some(id.as_raw() as u16),
                            _ => None,
                        } {
                            match protocol::CanDriveMessage::parse_can_frame(id, &frame.data()) {
                                Ok(Some(msg)) => {
                                    match can_rx_message_queue.send(msg) {
                                        Ok(_) => (),
                                        Err(e) => {
                                            eprintln!("Error sending to rx queue: {:?}", e);
                                        }
                                    }
                                }
                                Err(e) => eprintln!("Error parsing frame: {:?}", e),
                                _ => ()
                            }
                        }
                    },
                    Err(nb::Error::WouldBlock) => {
                    },
                    Err(e) => {
                        eprintln!("Error receiving from socket: {:?}", e);
                    }
                }
            };
        });
        return CommManager {
            thread: Some(handle),
            sink: device_message_sink,
            source: device_message_source,
        };
    }

    pub fn get_outgoing_queue(&self) -> Sender<CanDriveMessage> {
        return self.sink.clone();
    }

    pub fn get_incoming_queue(&self) -> Receiver<CanDriveMessage> {
        return self.source.clone();
    }
}

impl Threaded for CommManager {
    fn join(&mut self) {
        self.thread.take().unwrap().join().unwrap();
    }
}