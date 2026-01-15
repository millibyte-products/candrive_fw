
use candrive_api::comm_manager::CommManager;
use candrive_api::device_manager::DeviceManager;
use candrive_api::local_commands::LocalCommandParser;
use candrive_api::util::Threaded;

fn main() {
    // Open and manage the CAN socket
    let mut comm_manager = CommManager::new("can0".to_string());
    let mut local_command_parser = LocalCommandParser::new();

    let can_tx = comm_manager.get_outgoing_queue();
    let can_rx = comm_manager.get_incoming_queue();
    let local_rx = local_command_parser.get_command_queue();
    let device_manager = DeviceManager::new(&can_tx, &can_rx, &local_rx);
    device_manager.lock().unwrap().join();
    local_command_parser.join();
    comm_manager.join();
}
