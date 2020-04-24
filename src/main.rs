use crate::wiringpi::{Command, Vehicle};
use std::io::{BufRead, BufReader};
use std::os::unix::net::{UnixListener, UnixStream};
use std::thread;

mod wiringpi;

fn main() {
    init_socket();
}

fn init_socket() {
    use std::path::Path;

    fn bind(path: impl AsRef<Path>) -> std::io::Result<UnixListener> {
        let path = path.as_ref();
        // ignore error if it occurs
        let _ = std::fs::remove_file(path);
        UnixListener::bind(path)
    }

    println!("Starting socket");
    let listener = bind("/tmp/motors.sock").unwrap();

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                thread::spawn(|| handle_client(stream));
            }
            Err(err) => {
                println!("Error: {}", err);
                break;
            }
        }
    }
}

fn handle_client(stream: UnixStream) {
    let stream = BufReader::new(stream);

    let mut vehicle = Vehicle::new();

    println!("Start reading from socket:");
    for line in stream.lines() {
        let line = line.unwrap();
        println!("Message Recieved: {}", line);

        let command = match parse_command(line) {
            Ok(command) => command,
            Err(error) => {
                println!("Error: {:?}", error);
                continue;
            }
        };

        vehicle.send_command(command);
    }
}

fn parse_command(line: String) -> Result<Command, String> {
    match line.as_ref() {
        "forward" => Ok(Command::Forward),
        "backward" => Ok(Command::Backward),
        "turn_right" => Ok(Command::TurnRight),
        "turn_left" => Ok(Command::TurnLeft),
        "stop" => Ok(Command::Stop),
        "kill" => Ok(Command::Kill),
        _ => Err(format!("Unknown command: {}", line)),
    }
}
