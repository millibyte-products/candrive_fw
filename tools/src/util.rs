use core::{panic};
use std::{backtrace};
use crossbeam_channel::{Receiver, Sender, TryRecvError};
use crate::{local_commands::LocalCommands, protocol::CanDriveMessage};
use lazy_static::lazy_static;
use log::{error, trace};

pub trait Threaded {
    fn join(&mut self);
}

// Handles messages until a discriminant matches
pub fn wait_for_message(
    can_rx: &Receiver<CanDriveMessage>,
    predicate: impl Fn(&CanDriveMessage) -> bool) -> Result<CanDriveMessage, String>
{
    loop {
        match can_rx.try_recv() {
            Ok(msg) => {
                // Check for ack integrity
                if predicate(&msg) {
                    return Ok(msg);
                }
            },
            Err(TryRecvError::Empty) => {},
            Err(e) => { return Err(format!("Error receiving message: {}", e))}
        }
    }
}

#[derive(Debug)]
pub struct Shared<T>
{
    inner: std::sync::Arc<std::sync::Mutex<T>>,
}

impl<T> Shared<T>
{
    pub fn new(value: T) -> Shared<T>
    {
        Shared {
            inner: std::sync::Arc::new(std::sync::Mutex::new(value)),
        }
    }

    pub fn get(&self) -> std::sync::MutexGuard<'_, T>
    {
        self.inner.lock().unwrap()
    }
}

impl<T> Clone for Shared<T>
{
    fn clone(&self) -> Self
    {
        Shared {
            inner: self.inner.clone(),
        }
    }
}

lazy_static! {
    static ref PROCESS_STATUS: Shared<bool> = Shared::new(false);
    static ref CAN_TX_QUEUE: Shared<Option<Sender<CanDriveMessage>>> = Shared::new(None);
    static ref CAN_RX_QUEUE: Shared<Option<Receiver<CanDriveMessage>>> = Shared::new(None);
    static ref LOCAL_COMMAND_QUEUE: Shared<Option<Receiver<LocalCommands>>> = Shared::new(None);
}

pub fn is_running() -> bool
{
    *PROCESS_STATUS.get()
}

pub fn set_running(running: bool)
{
    *PROCESS_STATUS.get() = running;
}

pub fn critical_error(msg: String)
{
    let bt = backtrace::Backtrace::capture();
    error!("!!! {msg} !!!");
    error!("Trace Start ===============");
    trace!("{bt:?}");
    error!("Trace End ===============");
    set_running(false);
}

pub fn init_message_queues(can_tx: Sender<CanDriveMessage>, can_rx: Receiver<CanDriveMessage>, local_rx: Receiver<LocalCommands>)
{
    CAN_TX_QUEUE.get().replace(can_tx.clone());
    CAN_RX_QUEUE.get().replace(can_rx.clone());
    LOCAL_COMMAND_QUEUE.get().replace(local_rx.clone());
}

pub fn get_can_tx_queue() -> Sender<CanDriveMessage>
{
    if let Some(tx) = CAN_TX_QUEUE.get().as_ref() {
        return tx.clone();
    }
    panic!("CAN TX queue not initialized");
}

pub fn get_can_rx_queue() -> Receiver<CanDriveMessage>
{
    if let Some(rx) = CAN_RX_QUEUE.get().as_ref(){
        return rx.clone();
    }
    panic!("CAN RX queue not initialized");
}

pub fn get_local_command_queue() -> Receiver<LocalCommands>
{
    if let Some(local) = LOCAL_COMMAND_QUEUE.get().as_ref() {
        return local.clone();
    }
    panic!("Local command queue not initialized");
}

