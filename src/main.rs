use libc::c_int;
use std::thread::sleep;
use std::time::Duration;
use stopwatch::Stopwatch;

fn main() {
    controller_init();
    sleep(Duration::from_secs(2));
    // init_distance_measure();

    // test_drive();

    forward();
    sleep(Duration::from_secs(10));

    backward();
    sleep(Duration::from_secs(10));

    forward();
    sleep(Duration::from_secs(10));

    stop();
}

fn test() {
    // update_wheels(vec![WheelDir::ForwardRearRight]);
    let mut motors: [bool; 8] = [true, false, false, false, false, false, false, false];
    latch_tx(&motors);
}

fn forward() {
    update_wheels(vec![
        WheelDir::ForwardFrontRight,
        WheelDir::ForwardFrontLeft,
        WheelDir::ForwardRearRight,
        WheelDir::ForwardRearLeft,
    ]);
}

fn backward() {
    update_wheels(vec![
        WheelDir::BackwardFrontRight,
        WheelDir::BackwardFrontLeft,
        WheelDir::BackwardRearRight,
        WheelDir::BackwardRearLeft,
    ]);
}

fn stop() {
    update_wheels(vec![]);
}

fn update_wheels(wheels: Vec<WheelDir>) {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];

    for wheel in wheels.iter() {
        let wheel_index: usize = wheel_index(*wheel);
        motors[wheel_index] = true;
    }

    println!("update_wheels: {:?}, motors: {:?}", wheels, motors);

    latch_tx(&motors);
}

#[derive(Debug, Clone, Copy)]
enum WheelDir {
    ForwardRearRight,   // Rear right forward
    ForwardRearLeft,    // Rear left forward
    BackwardRearRight,  // rear right backward
    BackwardFrontLeft,  // front left backward
    BackwardFrontRight, // front right backward
    ForwardFrontRight,  // front right forward
    ForwardFrontLeft,   // front left forward
    BackwardRearLeft,   // rear left backward
}

fn wheel_index(wheel: WheelDir) -> usize {
    match wheel {
        WheelDir::ForwardRearRight => 0,
        WheelDir::ForwardRearLeft => 1,
        WheelDir::BackwardRearRight => 2,
        WheelDir::BackwardFrontLeft => 3,
        WheelDir::BackwardFrontRight => 4,
        WheelDir::ForwardFrontRight => 5,
        WheelDir::ForwardFrontLeft => 6,
        WheelDir::BackwardRearLeft => 7,
    }
}

// const MOTOR1_A: u8 = 3;
// const MOTOR1_B: u8 = 2;
// const MOTOR2_A: u8 = 4;
// const MOTOR2_B: u8 = 1;
// const MOTOR4_A: u8 = 0;
// const MOTOR4_B: u8 = 6;
// const MOTOR3_A: u8 = 5;
// const MOTOR3_B: u8 = 7;

#[link(name = "wiringPi")]
extern "C" {

    pub fn wiringPiSetup();

    pub fn pinMode(pin: c_int, mode: c_int);

    // C signature
    // extern void digitalWrite (int pin, int value) ;
    pub fn digitalWrite(pin: c_int, value: c_int);

    // int  digitalRead         (int pin) ;
    pub fn digitalRead(pin: c_int) -> c_int;

    // extern void delayMicroseconds (unsigned int howLong) ;
    pub fn delayMicroseconds(howLong: u16);

    pub fn pullUpDnControl(pin: c_int, pud: c_int);

    //extern int  wiringPiISR         (int pin, int mode, void (*function)(void)) ;
    pub fn wiringPiISR(pin: c_int, mode: c_int, get_ir: fn()) -> c_int;

}

const OUTPUT: c_int = 1;
const LOW: c_int = 0;
const HIGH: c_int = 1;
const MOTORLATCH: c_int = 29; // GPIO 5
const MOTORCLK: c_int = 28; // GPIO 1 (ID_SC)
const MOTORDATA: c_int = 27; // GPIO 0 (ID_SD)
const IRIN: c_int = 5;
const INPUT: c_int = 0;
const INT_EDGE_FALLING: c_int = 1;
const PUD_OFF: c_int = 0;
const PUD_DOWN: c_int = 1;
const PUD_UP: c_int = 2;
const IR_LIMITS: usize = 64; // bytes buffer = IR_LIMITS x8 bits

fn controller_init() {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];
    unsafe {
        wiringPiSetup();
        pinMode(MOTORLATCH, OUTPUT);
        pinMode(MOTORDATA, OUTPUT);
        pinMode(MOTORCLK, OUTPUT);
    }

    latch_tx(&motors);
}

fn latch_tx(motors: &[bool; 8]) {
    unsafe {
        digitalWrite(MOTORLATCH, LOW);
        digitalWrite(MOTORDATA, LOW);

        for i in motors.iter() {
            delayMicroseconds(1); // 10 micros  delayMicroseconds

            digitalWrite(MOTORCLK, LOW);

            if *i {
                digitalWrite(MOTORDATA, HIGH);
                println!("HIGH");
            } else {
                digitalWrite(MOTORDATA, LOW);
                println!("low");
            }

            delayMicroseconds(1); // 10 micros  delayMicroseconds
            digitalWrite(MOTORCLK, HIGH);
        }

        digitalWrite(MOTORLATCH, HIGH);
    }
}

const TRIG: i32 = 25;
const ECHO: i32 = 4;

fn init_distance_measure() {
    loop {
        let dis = distance_measure();
        println!("Distance: {}", dis);
        sleep(Duration::from_millis(50));
    }
}

fn distance_measure() -> u32 {
    unsafe {
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);
    }

    let mut wait_count: i32 = 0;

    unsafe {
        while !(digitalRead(ECHO) == 1) {
            wait_count = wait_count + 1;
            if wait_count >= 5000 {
                break;
            } else {
                //   sleep(0.001);
                // sleep(Duration::from_millis(1));
                sleep(Duration::from_micros(100));
            }
        }
    }
    let sw = Stopwatch::start_new();
    wait_count = 0;
    unsafe {
        while !(digitalRead(ECHO) == 0) {
            wait_count = wait_count + 1;
            if wait_count >= 5000 {
                break;
            } else {
                //   sleep(0.001);
                //sleep(Duration::from_millis(1));
                sleep(Duration::from_micros(50));
            }
        }
    }

    // Sound travels at approximately 340 meters per second. This corresponds
    // to about 29.412µs (microseconds) per centimeter. To measure the distance
    // the sound has travelled we use the formula:
    //
    // Distance = (Time x SpeedOfSound) / 2
    //
    // The "2" is in the formula because the sound has to travel back and forth.
    // First the sound travels away from the sensor, and then it bounces off of
    // a surface and returns back. The easy way to read the distance as centimeters
    // is to use the formula: Centimeters = ((Microseconds / 2) / 29). For example,
    // if it takes 100µs (microseconds) for the ultrasonic sound to bounce back,
    // then the distance is ((100 / 2) / 29) centimeters or about 1.7 centimeters.

    let usec = sw.elapsed().as_micros();
    let x = convert_ms_to_cm(usec);

    // println!("micros: {:?}, x: {:?}", usec, x);

    x
}

fn convert_ms_to_cm(usec: u128) -> u32 {
    ((1.6 * (usec as f64 / 100.0) as f64) + 0.0).round() as u32
}

#[test]
fn t_convert() {
    assert_eq!(10, convert_ms_to_cm(625));
    assert_eq!(20, convert_ms_to_cm(1245));
    assert_eq!(30, convert_ms_to_cm(1865));
    assert_eq!(40, convert_ms_to_cm(2485));
}

fn irInit() {
    unsafe {
        pinMode(IRIN, INPUT);
        pullUpDnControl(IRIN, PUD_UP);
        // to detect falling edge
        // let get_ir = || getIR();
        wiringPiISR(IRIN, INT_EDGE_FALLING, || get_ir());
        // done = 0;
    }
}

fn get_ir() {
    // let buf: [u8; 64];
    let mut buf: Vec<u8> = Vec::new();
    let i: u8 = count_low();
    //   let mut j: u16 = 0x0; // capable 32768 bits = 4096 bytes
    // let mut k: u8 = 0x0;
    let mut bits: u32 = 0;
    // i = countLow();
    for j in 0..IR_LIMITS {
        // buffer bytes LIMITS
        for i in 0..8 {
            // 8 bits
            let k = count_low();
            if k == 0 {
                buf[j] >>= 8 - i;
                // done = 1;
                println!("get_ir k == 0, buf: {:?}, bits: {:?}", buf, bits);
                return;
            }
            buf[j] >>= 1;
            buf[j] = buf[j] + (if k > 30 { 0x80 } else { 0 });
            bits = bits + 1;
        }
    }
    //   done = 1;
    println!("get_ir buf: {:?}, bits: {:?}", buf, bits);
}

fn count_low() -> u8 {
    unsafe {
        let mut i: u8 = 0;
        while digitalRead(IRIN) == 0 {}
        // ; // wait
        while digitalRead(IRIN) == 1 {
            i = i + 1;
            delayMicroseconds(26);
            if i == 0 {
                println!("count_low i == 0");
                return 0; // timeout
            }
        }
        println!("count_low i");
        return i;
    }
}
