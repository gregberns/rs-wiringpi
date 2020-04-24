# WiringPi bindings in Rust

This is a very incomplete project, to learn C bingings in Rust by binding to the WiringPi C library.

Using this library as a reference point: https://github.com/Ogeon/rust-wiringpi

## Build

### Compiling and Linking on MacOS

**There's an issue now compiling and linking against `wiringPi`. Not sure how to get the linker to work**

```bash
# https://rust.azdevs.org/2019-07-24/
rustup target add armv7-unknown-linux-gnueabihf
cargo install --force --git https://github.com/rust-embedded/cross cross
# This will take a while
cross build --target=armv7-unknown-linux-gnueabihf
cp target/armv7-unknown-linux-gnueabihf/debug/rs-wiringpi bin/rs-wiringpi
```

## Run

* Build on RPi
* Open two terminals
* First terminal: `cargo run`
* Second terminal: `nc -U /tmp/motors1.sock`

## Plan

* Control wheels
* Detect distance


* Create crutch to replace wheel
* Attach Camera and/or Distance Calc (to swivel)
* Capture/analyze camera data

## Problems

* Straight line
  * Get a car to see, find, and drive on a line
    * Requires camera
    * See a colored line
  * Alternative: Drive parallel to a wall (might be harder to do - not sure how that would work)
  * Alternative: Drive through 'hallway' without hitting sides
    * Not sure if ultrasound can detect enough around it
    * Could add it to a swivel, then it could check the sides

* Use Ultrasound on servo to see objects 
  * @6:11 https://www.youtube.com/watch?v=yohYrKCexvM

* Vehicle find a route out of a space
  * Alternative: Navigate course without running over a line
  * Reasonable, but not terribly exciting

* Navigate a cluttered environment - avoid objects
  * Technique: See a light/item (2 of them) in camera to identify "true north" and navigate toward it

* Image rec: Object identification and tracking by color
  * Line detection and path creation then following
  * Stalk the cat
* Take four lights to determine location
* Map house and or map space while autonomously traversing it
