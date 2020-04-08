// mod wiringpi_bindings;
use libc::c_int;
// use std::convert::TryFrom;
use std::thread::sleep;
use std::time::Duration;
use stopwatch::Stopwatch;

// Test:
// * Turn off/on a LED first, then worry about motors

fn main() {
    init_wiringpi_setup();
    controller_init();
    // init_distance_measure();

    // test_drive();

    test();
    sleep(Duration::from_secs(5));
    stop();
}

fn test() {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];
    // motors[0] = true; // Rear right forward
    // motors[1] = true; // Rear left forward
    // motors[2] = true; // rear right back
    // Unknown
    motors[3] = true;
    // motors[4] = true;
    // motors[5] = true;
    // motors[6] = true;
    // motors[7] = true;
    latch_tx(&motors);
}

fn init_wiringpi_setup() {
    unsafe {
        wiringPiSetup();
    }
}

fn test_drive() {
    controller_init();

    forward();
    sleep(Duration::from_secs(2));

    println!("=====");

    stop();
    sleep(Duration::from_secs(2));

    println!("=====");

    forward();
    sleep(Duration::from_secs(2));

    println!("=====");

    stop();
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

fn forward() {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];
    motors[2] = true;
    motors[3] = true;
    motors[4] = true;
    motors[7] = true;
    latch_tx(&motors);
}

fn stop() {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];
    motors[0] = false;
    motors[1] = false;
    motors[2] = false;
    motors[3] = false;
    motors[4] = false;
    motors[5] = false;
    motors[6] = false;
    motors[7] = false;
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
        // latch_state
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

// gcc run-motor.c -lwiringPi -lrt -Wall -pthread
// Latch=00000008  Front Right
// Latch=00000000
// Latch=00000010  Front Left
// Latch=00000000
// Latch=00000020  Back Right
// Latch=00000000
// Latch=00000001  Back Left
// Latch=00000000

// Latch=8
// Latch=18
// Latch=38
// Latch=39
// Latch=31
// Latch=21
// Latch=1
// Latch=0

// 4_A 0
// 2_B 1
// 1_B 2
// 1_A 3
// 2_A 4
// 3_A 5
// 4_B 6
// 3_B 7

// #define MOTOR1_A 3//2
// #define MOTOR1_B 2//3
// #define MOTOR2_A 4//1
// #define MOTOR2_B 1//4
// #define MOTOR4_A 0
// #define MOTOR4_B 6
// #define MOTOR3_A 5
// #define MOTOR3_B 7

// struct Motors ([bool; 8]);

// enum MotorNumber {
//     M1,
//     M2,
//     M3,
//     M4,
// }

// enum MotorDirection {
//     Forward,
//     Backward,
//     Nutral
// }

// // const MOTOR1_A: u8 = 3;
// // const MOTOR1_B: u8 = 2;
// // const MOTOR2_A: u8 = 4;
// // const MOTOR2_B: u8 = 1;
// // const MOTOR4_A: u8 = 0;
// // const MOTOR4_B: u8 = 6;
// // const MOTOR3_A: u8 = 5;
// // const MOTOR3_B: u8 = 7;

// fn forward() {
//     set_motor(MotorNumber::M1, MotorDirection::Forward);
//     set_motor(MotorNumber::M2, MotorDirection::Forward);
//     set_motor(MotorNumber::M3, MotorDirection::Forward);
//     set_motor(MotorNumber::M4, MotorDirection::Forward);
// }

// fn stop() {
//     set_motor(MotorNumber::M1, MotorDirection::Nutral);
//     set_motor(MotorNumber::M2, MotorDirection::Nutral);
//     set_motor(MotorNumber::M3, MotorDirection::Nutral);
//     set_motor(MotorNumber::M4, MotorDirection::Nutral);
// }

// fn set_motor(motor_number: MotorNumber, direction: MotorDirection){
//     let motor = match motor_number {
//         M1 => (MOTOR1_A, MOTOR1_B),
//         M2 => (MOTOR2_A, MOTOR2_B),
//         M3 => (MOTOR3_A, MOTOR3_B),
//         M4 => (MOTOR4_A, MOTOR4_B),
//     };

//     match direction {
//         MotorDirection::Forward => ,
//         MotorDirection::Backward => ,
//         MotorDirection::Nutral => ,
//     }

// }

// fn DCMotorRun(motornum: u8, cmd: u8) {
//     let mut a: u8;
//     let mut b: u8;

//     match motornum {
//         1 => {
//             a = MOTOR1_A;
//             b = MOTOR1_B;
//         }
//         2 => {
//             a = MOTOR2_A;
//             b = MOTOR2_B;
//         }
//         3 => {
//             a = MOTOR3_A;
//             b = MOTOR3_B;
//         }
//         4 => {
//             a = MOTOR4_A;
//             b = MOTOR4_B;
//         }
//         _ => return,
//     }

//     let mut latch_state: u8 = 0;

//     match cmd {
//         FORWARD => {
//             latch_state |= a;
//             latch_state &= !b;
//         }
//         BACKWARD => {
//             latch_state &= !a;
//             latch_state |= b;
//         }
//         RELEASE => {
//             latch_state &= !a;
//             latch_state &= !b;
//         }
//         _ => return,
//     }

//     latch_tx(latch_state);

//     // printf("Latch=%08X\n", latch_state);
//     return;
// }

// fn DCMotorInit(num: u8) {
//     let mut latch_state: u8 = 0;

//     match num {
//         1 => latch_state &= !MOTOR1_A & !MOTOR1_B,
//         2 => latch_state &= !MOTOR2_A & !MOTOR2_B,
//         3 => latch_state &= !MOTOR3_A & !MOTOR3_B,
//         4 => latch_state &= !MOTOR4_A & !MOTOR4_B,
//         _ => panic!("Invalid value supplied DCMotorInit"),
//     }

//     latch_tx(latch_state);

//     println!("Latch={}\n", latch_state);
//     return;
// }
