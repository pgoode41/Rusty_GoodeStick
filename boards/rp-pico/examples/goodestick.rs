//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

use core::pin::Pin;
use core::convert::TryInto;

//use alloc::string::ToString;
// The macro for our start-up function
use cortex_m_rt::entry;
use hal::gpio::bank0::Gpio0;
use heapless::String;
use cortex_m::delay::Delay;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// Pull in any important traits
use rp_pico::hal::prelude::*;

// USB Communications Class Device support
use usbd_serial::SerialPort;

use embedded_hal::adc::OneShot;

use rp2040_hal::adc::Adc;

use rp_pico::Pins;

use rp2040_hal::{gpio::{bank0::Gpio25, PushPullOutput}, sio::Sio};



/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
//#####################################################################################################################//
//#####################################################################################################################//
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
        // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let mut pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Dozo")
        .product("GoodeStick")
        .serial_number("goodestick_test_device")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
        //let big_yee = String::from("BIG YEEEE!!!\n");
        let mut gpio_pin_0 = pins.gpio0.into_push_pull_output();
        let mut gpio_pin_1 = pins.gpio1.into_push_pull_output();
//#####################################################################################################################//
    loop {
//#########################################################################//
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    let mut cmd: GoodeStickCommandString = {
                        GoodeStickCommandString { 
                            start_bit: [0],
                            gpio_0_number: [0,0],
                            gpio_1_number: [0,0],
                            gpio_2_number: [0,0],
                            gpio_0_start_state: [0],
                            gpio_1_start_state: [0],
                            gpio_2_start_state: [0],
                            gpio_0_active_state: [0],
                            gpio_1_active_state: [0],
                            gpio_2_active_state: [0],
                            pulse_duration: [0,0,0,0],
                            time_mode: [0],
                            program_mode: [0],
                            stop_bit: [0],
                        }
                    };
                    // Convert to upper case
                    /*
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    */
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];

            while !wr_ptr.is_empty() {

                //match serial.write(wr_ptr) {
                //    Ok(len) => {
                 //       wr_ptr = &wr_ptr[len..];

                    

                    serial.write("\n".as_bytes());
                    //serial.write("COUNT: ".as_bytes());
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    serial.write(&[wr_ptr.len()as u8]);
                    
                    serial.write("\n".as_bytes());

                    
                    
                    for x in 0..19 {
                        //let count:u8 = x.try_into().unwrap();
                        if x == 0 {
                            serial.write("\n".as_bytes());
                            serial.write("0 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
    
                        } else if x == 1 {
                            serial.write("\n".as_bytes());
                            serial.write("1 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 2 {
                            serial.write("\n".as_bytes());
                            serial.write("2 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 3 {
                            serial.write("\n".as_bytes());
                            serial.write("3 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 4 {
                            serial.write("\n".as_bytes());
                            serial.write("4 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 5 {
                            serial.write("\n".as_bytes());
                            serial.write("5 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 6 {
                            serial.write("\n".as_bytes());
                            serial.write("6 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        else if x == 7 {
                            serial.write("\n".as_bytes());
                            serial.write("7 HIT".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("\n".as_bytes());
                            serial.write("Input VAL: ".as_bytes());
                            serial.write(&[wr_ptr[x]]);
                            serial.write("\n".as_bytes());
                        }
                        //serial.write("\n".as_bytes());
                        //serial.write("Input VAL: ".as_bytes());
                        //serial.write(&[wr_ptr[x]]);
                        //serial.write("\n".as_bytes());

                        serial.write("\n".as_bytes());
                        serial.write("Input IDX: ".as_bytes());
                        serial.write(&[x as u8]);
                        serial.write("\n".as_bytes());

                        //let val = ascii_decode_base_10(wr_ptr[x]);
                        let val = wr_ptr[x];
                        if val == 11 {
                            serial.write("\n".as_bytes());
                            serial.write("Bad Val: ".as_bytes());
                            serial.write(&[val]);
                            serial.write("\n".as_bytes());
                            continue;
                        };
                        match x {
                            0 => {
                                cmd.start_bit[0] = val; 
                            },
                            1 => {
                                cmd.gpio_0_number[0] = val; 
                            },
                            2 => {
                                cmd.gpio_0_number[1] = val; 
                            },
                            3 => {
                                cmd.gpio_1_number[0] = val; 
                            },
                            4 => {
                                cmd.gpio_1_number[1] = val; 
                            },
                            5 => {
                                cmd.gpio_2_number[0] = val; 
                            },
                            6 => {
                                cmd.gpio_2_number[1] = val; 
                            },
                            7 => {
                                cmd.gpio_0_start_state[0] = val; 
                            },
                            8 => {
                                cmd.gpio_1_start_state[0] = val; 
                            },
                            9 => {
                                cmd.gpio_2_start_state[0] = val; 
                            },
                            10 => {
                                cmd.gpio_0_active_state[0] = val; 
                            },
                            11 => {
                                cmd.gpio_1_active_state[0] = val; 
                            },
                            12 => {
                                cmd.gpio_2_active_state[0] = val; 
                            },
                            13 => {
                                cmd.pulse_duration[0] = val; 
                            },
                            14 => {
                                cmd.pulse_duration[1] = val; 
                            },
                            15 => {
                                cmd.pulse_duration[2] = val; 
                            },
                            16 => {
                                cmd.pulse_duration[3] = val; 
                            },
                            17 => {
                                cmd.time_mode[0] = val; 
                            },
                            18 => {
                                cmd.program_mode[0] = val; 
                            },
                            19 => {
                                cmd.stop_bit[0] = val; 
                            },
                            _ => ()
                        }
                        }
                    }
                    ///*
                    //serial.write(&[cmd.start_bit[0]].as_bytes());
                    serial.write(&[cmd.start_bit[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_0_number[0], cmd.gpio_0_number[1]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_1_number[0], cmd.gpio_1_number[1]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_2_number[0], cmd.gpio_2_number[1]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_0_start_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_1_start_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_2_start_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_0_active_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_1_active_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.gpio_2_active_state[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.pulse_duration[0], cmd.pulse_duration[1], cmd.pulse_duration[2], cmd.pulse_duration[3]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.time_mode[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.program_mode[0]]);
                    serial.write("\n".as_bytes());
                    serial.write(&[cmd.stop_bit[0]]);
                    serial.write("\n".as_bytes());
                    //*/
                }
                
            }
            
        }
    }
} 
//#####################################################################################################################//
//#####################################################################################################################//
//#####################################################################################################################//
//Pin<Gpio0, Output<PushPull>>
//#[derive(Debug)]

struct GoodeStickCommandString {
    start_bit: [u8;1],
    gpio_0_number: [u8;2],
    gpio_1_number: [u8;2],
    gpio_2_number: [u8;2],
    gpio_0_start_state: [u8;1],
    gpio_1_start_state: [u8;1],
    gpio_2_start_state: [u8;1],
    gpio_0_active_state: [u8;1],
    gpio_1_active_state: [u8;1],
    gpio_2_active_state: [u8;1],
    pulse_duration:[u8;4],
    time_mode:[u8;1],
    program_mode:[u8;1],
    stop_bit: [u8;1],
}

fn ascii_decode_base_10(idx: u8) -> u8 {
    match idx as u8 {
        48 => {
            return 0;
        },
        49 => {
            return 1;
        },
        50 => {
            return 2;
        }
        51 => {
            return 3;
        }
        52 => {
            return 4;
        }
        53 => {
            return 5;
        }
        54 => {
            return 6;
        }
        55 => {
            return 7;
        }
        56 => {
            return 8;
        }
        57 => {
            return 9;
        }
        _ => 11,
    }
}


fn gpio_0_fire_pin(cmd_data: GoodeStickCommandString, delay: &mut Delay, pins: Pins) {
    let mut gpio_pin = pins.gpio0.into_push_pull_output();
    let cmd_0 = cmd_data;
    //let mut pin = gpio_pin;
    gpio_pin.set_high().unwrap();
    delay.delay_ms(500);
    gpio_pin.set_low().unwrap();
    delay.delay_ms(500);
}

fn gpio_1_fire_pin(cmd_data: GoodeStickCommandString, delay: &mut Delay, pins: Pins) {
    let mut gpio_pin = pins.gpio1.into_push_pull_output();
    let cmd_0 = cmd_data;
    //let mut pin = gpio_pin;
    gpio_pin.set_high().unwrap();
    delay.delay_ms(500);
    gpio_pin.set_low().unwrap();
    delay.delay_ms(500);
}

fn gpio_2_fire_pin(cmd_data: GoodeStickCommandString, delay: &mut Delay, pins: Pins) {
    let mut gpio_pin = pins.gpio2.into_push_pull_output();
    let cmd_0 = cmd_data;
    //let mut pin = gpio_pin;
    gpio_pin.set_high().unwrap();
    delay.delay_ms(500);
    gpio_pin.set_low().unwrap();
    delay.delay_ms(500);
}