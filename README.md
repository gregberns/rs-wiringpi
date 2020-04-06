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
