use argparse::{ArgumentParser, Store, StoreOption, StoreTrue};
use futures_util::StreamExt;
use std::{clone, io};
use crate::comm_manager::CommManager;
use crate::device_manager::DeviceManager;
use crate::local_commands::{LocalCommandParser, LocalCommands, ParsedArguments};

#[tokio::main]
async fn main() {
    let mut threads = Vec::new();
    // Open and manage the CAN socket
    let comm_manager = CommManager::new("can0");
    let local_command_parser = LocalCommandParser::new();

    let can_tx = comm_manager.get_outgoing_queue();
    let can_rx = comm_manager.get_incoming_queue();
    let local_rx = local_command_parser.get_command_queue();
    let device_manager = DeviceManager::new(&can_tx, &can_rx, &local_rx);
    device_manager.join();
    local_command_parser.join();
    comm_manager.join();
}
