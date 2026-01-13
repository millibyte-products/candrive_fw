use tokio::sync::mpsc::{Receiver, Sender, channel};


pub struct IPCChannel<T>
{
    // EXT -> Self queue
    message_send: Sender<T>,
    message_queue: Receiver<T>,
    // Self -> EXT queue
    broadcast_send: Sender<T>,
    broadcast_endpoint: Receiver<T>,
}

impl IPCChannel<T>
{
    pub fn new() -> IPCChannel<T>
    {
        let (message_send, message_queue) = channel();
        let (broadcast_send, broadcast_endpoint) = channel();
        return IPCChannel
        {
            message_send,
            message_queue,
            broadcast_send,
            broadcast_endpoint,
        };
    }

    // Get internal endpoints for sending/receiving
    pub fn get_channel(&self) -> (Sender<T>, Receiver<T>)
    {
        return (self.message_send.clone(), self.broadcast_endpoint.clone());
    }

    // Get external endpoints for sending/receiving
    pub fn get_endpoints(&self) -> (Sender<T>, Receiver<T>)
    {
        return (self.broadcast_send.clone(), self.message_queue.clone());
    }

    // Internal -> External
    pub fn send(&self, msg: T) -> Result<(), tokio::sync::mpsc::SendError<T>>
    {
        self.broadcast_send.send(msg)
    }

    // External -> Internal
    pub fn recv(&self) -> Result<T, tokio::sync::mpsc::RecvError>
    {
        self.message_queue.recv()
    }
}