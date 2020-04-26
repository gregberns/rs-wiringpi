use crate::model::Command;
use std::io::{BufRead, BufReader};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::thread::sleep;
use std::time::Duration;
mod model;

// mod wiringpi;
// use crate::wiringpi::Vehicle;

fn main() {
    // let (tx, rx): (Sender<i32>, Receiver<i32>) = mpsc::channel();
    let state: Arc<Mutex<i32>> = Arc::new(Mutex::new(0));
    let state_ref = state.clone();

    thread::spawn(move || read_stdin(&state_ref));

    // let mut vehicle = Vehicle::new();

    loop {
        // Stopped
        // Read current value
        let command_int = (*state.lock().unwrap()).clone();
        // calculate decicion
        // if between -5 and 5 go forward
        // if < -5 turn right
        // if > 5 turn left
        let command = next_action(command_int);
        // do command
        // vehicle.send_command(command);
        // sleep 100ms
        sleep_ms(1000);
        // stop
        // vehicle.send_command(Command::Stop);
        // sleep 100ms // let the image stabalize
        sleep_ms(1000);
    }
}

fn sleep_ms(duration_ms: u64) {
    sleep(Duration::from_millis(duration_ms));
}

fn read_stdin(state: &Arc<Mutex<i32>>) {
    use std::io;
    use std::io::prelude::*;

    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let line = line.unwrap();
        match line.parse::<i32>() {
            Ok(value) => {
                let mut data = state.lock().unwrap();
                *data = value;
            }
            Err(_) => println!("Parse error. Value: {}", line),
        };

        // sender.send(i).unwrap();
    }
}

fn next_action(current_value: i32) -> Command {
    if current_value < -5 {
        return Command::TurnLeft;
    }
    if -5 < current_value && current_value < 5 {
        return Command::Forward;
    }
    // if 5 < current_value {
    return Command::TurnRight;
    // }
}

// fn handle_client(stream: Receiver<i32>) {
//     // let stream = BufReader::new(stream);

//     let mut vehicle = Vehicle::new();

//     println!("Start reading from socket:");
//     for line in stream.recv() {
//         // let line = line.unwrap();
//         println!("Message Recieved: {}", line);

//         let command = match parse_command(line) {
//             Ok(command) => command,
//             Err(error) => {
//                 println!("Error: {:?}", error);
//                 continue;
//             }
//         };

//         vehicle.send_command(command);
//     }
// }

// fn parse_command(line: String) -> Result<Command, String> {
//     match line.as_ref() {
//         "f" => Ok(Command::Forward),
//         "b" => Ok(Command::Backward),
//         "r" => Ok(Command::TurnRight),
//         "l" => Ok(Command::TurnLeft),
//         "s" => Ok(Command::Stop),
//         // "kill" => Ok(Command::Kill),
//         _ => Err(format!("Unknown command: {}", line)),
//     }
// }
