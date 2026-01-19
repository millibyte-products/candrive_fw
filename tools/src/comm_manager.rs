use socketcan::{EmbeddedFrame, Id, Socket, StandardId};
use socketcan::{CanFrame, CanSocket};
use crate::protocol::{self, CONTROLLER_ID, DISCOVERY_ASSIGN_ID, DISCOVERY_REQUEST_ID};
use crate::util::{Threaded, critical_error, is_running};
use std::io::ErrorKind;
use crate::{protocol::{CanDriveMessage}};
use crossbeam_channel::{Receiver, Sender, TryRecvError, unbounded};
use log::{error, info, trace};
pub struct CommManager {
    thread: Option<std::thread::JoinHandle<()>>,
    sink: Sender<CanDriveMessage>,
    source: Receiver<CanDriveMessage>,
}

impl CommManager {
    pub fn new(port: String) -> CommManager {
        trace!("Entering comm manager");
        let (device_message_sink, can_tx_message_queue) = unbounded();
        let (can_rx_message_queue, device_message_source) = unbounded();
        let handle = std::thread::spawn(move || {
            trace!("Starting CAN socket thread");
            let socket = match CanSocket::open(port.as_str()) {
                Ok(socket) => socket,
                Err(e) => {
                    critical_error(format!("Error opening socket: {:?}", e));
                    return;
                }
            };
            match socket.set_nonblocking(true) {
                Ok(_) => (),
                Err(e) => {
                    critical_error(format!("Error setting socket to nonblocking: {:?}", e));
                    return;
                }
            }
            info!("CAN socket thread started");
            while is_running() {
                match can_tx_message_queue.try_recv()
                {
                    Ok(msg) => {
                        trace!("Dequeued can_tx message: {:?}", msg);
                        let id = match msg {
                            CanDriveMessage::DiscoveryReq { .. } => {
                                DISCOVERY_REQUEST_ID
                            },
                            CanDriveMessage::DiscoveryAssign { .. } => {
                                DISCOVERY_ASSIGN_ID
                            },
                            CanDriveMessage::Control { id, .. } =>{
                                id.to_can_id()
                            },
                            CanDriveMessage::Broadcast { ..  } => {
                                CONTROLLER_ID
                            },
                        };
                        let data = match msg.as_bytes() {
                            Ok(data) => data,
                            Err(e) => {
                                critical_error(format!("Error serializing message: {:?}", e));
                                return;
                            }
                        };
                        let id = match StandardId::new(id) {
                            Some(id) => id,
                            None => {
                                critical_error(format!("Error creating CAN ID: {:X}", id));
                                return;
                            }
                        };
                        let frame = match CanFrame::new(id,
                    &data) {
                            Some(frame) => frame,
                            None => {
                                critical_error(format!("Error creating CAN frame: {:?} {:?}", id, data));
                                return;
                            }
                        };
                        trace!("Sending frame: ID{:?} [{:?}]", frame.id(), frame.data());
                        match socket.write_frame(&frame)
                        {
                            Ok(_) => (),
                            Err(e) => {
                                if e.kind() == ErrorKind::WouldBlock {
                                    critical_error(format!("Error sending to tx queue: {:?}", e));
                                }
                            }
                        }
                    },
                    Err(TryRecvError::Empty) => {
                        // Nothing to do
                    },
                    Err(e) => {
                        critical_error(format!("Error dequeing message: {:?}", e));
                    }
                }
                match socket.read_frame() {
                    Ok(frame) => {
                        trace!("Incoming CAN frame ID{:?} [{:?}]", frame.id(), frame.data());
                        if let Some(id) = match frame.id() {
                            Id::Standard(id) => Some(id.as_raw() as u16),
                            _ => None,
                        } {
                            match protocol::CanDriveMessage::parse_can_frame(id, &frame.data()) {
                                Ok(Some(msg)) => {
                                    match can_rx_message_queue.send(msg) {
                                        Ok(_) => (),
                                        Err(e) => {
                                            critical_error(format!("Error sending to rx queue: {:?}", e));
                                        }
                                    }
                                }
                                Err(e) => error!("Error parsing frame: {:?}", e),
                                _ => ()
                            }
                        }
                    },
                    Err(e) => {
                        if e.kind() == ErrorKind::WouldBlock {
                            // Nothing to do
                        } else {
                            critical_error(format!("Error receiving from socket: {:?}", e));
                        }
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