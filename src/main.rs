// mod wiringpi_bindings;
use libc::c_int;
use std::thread::sleep;
use std::time::Duration;
use stopwatch::Stopwatch;

// Test:
// * Turn off/on a LED first, then worry about motors

fn main() {
    println!("wiringPiSetup()");
    unsafe {
        wiringPiSetup();
    }
    println!("wiringPiSetup()");

    println!("irInit()");
    loop {
        let dis = distance_measure();
        println!("{}", dis);
        sleep(Duration::from_millis(10));
    }
    println!("irInit() - complete");

    sleep(Duration::from_secs(50));
}

const Trig: i32 = 25;
const Echo: i32 = 4;

fn distance_measure() -> f64 {
    //   struct timeval tv1;
    //   struct timeval tv2;
    //   long start, stop;
    //   float dis;
    //   long waitCount = 0;

    unsafe {
        digitalWrite(Trig, LOW);
        delayMicroseconds(2);
        digitalWrite(Trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(Trig, LOW);
    }

    let mut waitCount: i32 = 0;

    unsafe {
        while !(digitalRead(Echo) == 1) {
            waitCount = waitCount + 1;
            if waitCount >= 5000 {
                break;
            } else {
                //   sleep(0.001);
                sleep(Duration::from_millis(1));
            }
        }
    }
    //   gettimeofday(&tv1, NULL);
    let sw = Stopwatch::start_new();
    waitCount = 0;
    unsafe {
        while !(digitalRead(Echo) == 0) {
            waitCount = waitCount + 1;
            if waitCount >= 5000 {
                break;
            } else {
                //   sleep(0.001);
                sleep(Duration::from_millis(1));
            }
        }
    }
    //gettimeofday(&tv2, NULL);
    //microseconds
    let usec = sw.elapsed_ms();

    /*
      int gettimeofday(struct timeval *tv, struct timezone *tz);
      The functions gettimeofday() and settimeofday() can get and set the time as well as a timezone.
      The use of the timezone structure is obsolete; the tz argument should normally be specified as NULL.
    */
    //   start = tv1.tv_sec * 1000000 + tv1.tv_usec;
    //   stop = tv2.tv_sec * 1000000 + tv2.tv_usec;

    let dis = usec / (1000000 * 34000) / 2;
    //   dis = (float)(stop - start) / 1000000 * 34000 / 2;

    dis as f64
}

#[test]
fn test() {
    let sw = Stopwatch::start_new();

    sleep(Duration::from_millis(1));
    sleep(Duration::from_millis(1));
    sleep(Duration::from_millis(1));

    let usec = sw.elapsed_ms();

    let dis = usec / (1000000 * 34000) / 2;

    println!("dis     {}", dis);
    println!("dis i64 {}", dis as i64);
    println!("dis f64 {}", dis as i64);
}

fn drive() {
    let mut motors: [bool; 8] = [false, false, false, false, false, false, false, false];

    controller_init(&mut motors);

    forward(&mut motors);
    latch_tx(&mut motors);
    sleep(Duration::from_secs(2));

    println!("=====");

    stop(&mut motors);
    latch_tx(&mut motors);
    sleep(Duration::from_secs(2));

    println!("=====");

    forward(&mut motors);
    latch_tx(&mut motors);
    sleep(Duration::from_secs(2));

    println!("=====");

    stop(&mut motors);
    latch_tx(&mut motors);
}

const OUTPUT: c_int = 1;

const LOW: c_int = 0;
const HIGH: c_int = 1;

const MOTORLATCH: c_int = 29; // GPIO 5
const MOTORCLK: c_int = 28; // GPIO 1 (ID_SC)
const MOTORDATA: c_int = 27; // GPIO 0 (ID_SD)

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

const IRIN: c_int = 5;

const INPUT: c_int = 0;
const INT_EDGE_FALLING: c_int = 1;
const PUD_OFF: c_int = 0;
const PUD_DOWN: c_int = 1;
const PUD_UP: c_int = 2;

const IR_LIMITS: usize = 64; // bytes buffer = IR_LIMITS x8 bits

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

fn controller_init(motors: &mut [bool; 8]) {
    unsafe {
        wiringPiSetup();
        pinMode(MOTORLATCH, OUTPUT);
        pinMode(MOTORDATA, OUTPUT);
        pinMode(MOTORCLK, OUTPUT);
    }

    latch_tx(motors);
}

fn forward(motors: &mut [bool; 8]) {
    motors[2] = true;
    motors[3] = true;
    motors[4] = true;
    motors[7] = true;
}

fn stop(motors: &mut [bool; 8]) {
    motors[0] = false;
    motors[1] = false;
    motors[2] = false;
    motors[3] = false;
    motors[4] = false;
    motors[5] = false;
    motors[6] = false;
    motors[7] = false;
}

fn latch_tx(motors: &mut [bool; 8]) {
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
