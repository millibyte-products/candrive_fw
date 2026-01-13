use argparse::{ArgumentParser, Store, StoreOption, StoreTrue};
use futures_util::StreamExt;
use std::{clone, io};
use tokio;
use tokio::sync::mpsc::{Receiver, Sender, channel};
use crate::comm_manager::CommManager;
use crate::device_manager::DeviceManager;
use crate::local_commands::{LocalCommandParser, LocalCommands, ParsedArguments};

#[tokio::main]
async fn main() {
    let mut threads = Vec::new();
    let token = CancellationToken::new();
    let cloned_token = token.clone();
    // Open and manage the CAN socket
    let comm_manager = CommManager::new("can0", token.clone());
    let (can_tx, can_rx) = comm_manager.get_channel();
    // Create device model server
    let device_manager = DeviceManager::new(can_tx.clone(), can_rx, token.clone(), None);
    let (device_tx, _device_rx) = device_manager.get_channel();
    // Create local (stdin) command parser
    let local_can_tx = can_tx.clone();
    let local_device_tx = device_tx.clone();
    let local_command_parser = LocalCommandParser::default();
    // Start threads/servers
    threads.extend(comm_manager.start());
    threads.extend(device_manager.start());
    threads.extend(local_command_parser.start(cloned_token, &local_can_tx, &local_device_tx));
    // Join threads
    threads.iter().for_each(|t| t.join());
}
