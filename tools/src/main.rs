
use candrive_api::{comm_manager::CommManager, util::set_running};
use candrive_api::device_manager::DeviceManager;
use candrive_api::local_commands::LocalCommandParser;
use candrive_api::util::{Threaded, init_message_queues, is_running};
use log::{ info, trace};
use simplelog::TermLogger;

fn main() {
    TermLogger::init(
        simplelog::LevelFilter::Info,
        simplelog::Config::default(),
        simplelog::TerminalMode::Mixed,
        simplelog::ColorChoice::Auto).unwrap();
    info!("Starting candrive-cli");
    set_running(true);
    // Open and manage the CAN socket
    let mut comm_manager = CommManager::new("can0".to_string());
    let mut local_command_parser = LocalCommandParser::new();

    let can_tx = comm_manager.get_outgoing_queue();
    let can_rx = comm_manager.get_incoming_queue();
    let local_rx = local_command_parser.get_command_queue();
    init_message_queues(can_tx.clone(), can_rx.clone(), local_rx.clone());
    let mut device_manager = DeviceManager::new(&can_tx, &can_rx, &local_rx);
    trace!("Main thread waiting for completion or exit");
    while is_running() {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    device_manager.join();
    local_command_parser.join();
    comm_manager.join();
}
