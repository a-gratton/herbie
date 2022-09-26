# Herbie
An autonomous S&amp;R robot built for our 3B term project.

This repo consists of two projects- the CAD files for herbies mechanical design and the source code for the embedded application (braincell)

# Braincell
Braincell is the embedded application for herbie, designed to run on a STM32F446 microcontroller.

## Building Braincell
### Rust Installation
Braincell is written in rust and compiled with cargo. To install rust, follow the guide from [the rust book](https://doc.rust-lang.org/book/ch01-01-installation.html).

You will also need to install some embedded-specific applications to successfully build braincell- I recommend following the [embedded rust guide](https://docs.rust-embedded.org/book/intro/install.html). Note that this project targets an STM32F4 procesor, so the command for installing the required target is:

 `$ rustup target add thumbv7em-none-eabihf`


### Building and Flashing Blinky
Once you have the required tooling, test your setup with the blinky example:
```
cargo build --example blinky
cd target\thumbv7em-none-eabihf\debug\examples
arm-none-eabi-objcopy -O binary blinky blinky.bin
```
Then flash blinky.bin using ST-Link.


## Important Dependencies
Braincell uses [rtic.rs (Real-Time Interrupt-driven Concurrency)](https://rtic.rs/1/book/en/) as its RTOS and [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) as its hardware abstraction layer.



