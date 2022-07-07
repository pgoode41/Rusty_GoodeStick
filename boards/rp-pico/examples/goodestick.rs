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
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    //let mut wr_ptr = &buf[..count];
                    let mut wr_ptr = &buf[0..2];
                    let thing = wr_ptr[0].to_ascii_lowercase();


                    /*
                    if thing == 0 {
                        serial.write("YEEE".as_bytes());
                        serial.write(&[thing]);
                    } else {
                        serial.write("NOOO".as_bytes());
                        serial.write(&[thing]);
                    }

                    serial.write(&[wr_ptr[0]]);
                    serial.write(&[wr_ptr[1]]);
                    */

                    //serial.write(0.as_bytes());
                    serial.write(&[thing]);
                    let zero = 0_u8.to_ascii_lowercase();
                    let one = 1_u8.to_ascii_lowercase();
                    

                    match thing {
                        zero => {
                            serial.write("ZERO".as_bytes());
                            serial.write(&[thing]);
                        }
                        one => {
                            serial.write("ONE".as_bytes());
                            serial.write(&[thing]);
                        }

                        _ => ()
                    }
                    //serial.write(nl.as_bytes());

                    /*

                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => {
                                wr_ptr = &wr_ptr[len..]
                                
                                //wr_ptr = &wr_ptr[0..1]
                                
                            }
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                    */
                }
            }
        }
        /* 
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

                        //let cmd_string = String::from(cmd_data.gpio_0_number+cmd_data.gpio_1_number);
                        //serial.write(&cmd_string.as_bytes());
                        

                        //serial.write(&cmd_data.gpio_0_number.as_bytes());
                        let nl: String<2> = String::from("\n");
                        let yee: String<2> = String::from("YEEE!!!!!!!!!\n");
                        buf.iter_mut().take(count).for_each(|b| {
                            b.make_ascii_uppercase();
                        });
                        // Send back to the host
                        let mut wr_ptr = &buf[..count];
                        let thingy = wr_ptr;
                        serial.write(yee.as_bytes());

                        serial.write(&[thingy[0]]);
                        serial.write(nl.as_bytes());
                        serial.write(nl.as_bytes());
                        serial.write(nl.as_bytes());
                        serial.write(nl.as_bytes());
                        
                        
                        while !wr_ptr.is_empty() {
                            match serial.write(wr_ptr) {
                                Ok(len) => wr_ptr = &wr_ptr[0..1],
                                // On error, just drop unwritten data.
                                // One possible error is Err(WouldBlock), meaning the USB
                                // write buffer is full.
                                Err(_) => break,
                            };
                        }
                        

                        //serial.write(&wr_ptr[0..1]);
                        //}
                        //serial.write(wr_ptr);

                        //
                        
                        let cmd_data = GoodeStickCommandString {
                            gpio_0_number: &buf[0..1],
                            gpio_1_number: String::from(&buf[2..3]),
                            gpio_0_start_state: String::from(&buf[4]),
                            gpio_1_start_state: String::from(wr_ptr[5]),
                            gpio_0_active_state: String::from(wr_ptr[6]),
                            gpio_1_active_state: String::from(wr_ptr[7]),
                            pulse_duration: String::from(wr_ptr[8]+wr_ptr[9]+wr_ptr[10]+wr_ptr[11]),
                            time_mode: String::from(wr_ptr[12]),
                            program_mode: String::from(wr_ptr[13]),
                        };
                        //serial.write(&cmd_data.gpio_0_number.as_bytes());

                        match Some(&*cmd_data.gpio_0_number) {
                            Some("00") => {
                                let cmd_0 = &cmd_data;
                                //let mut pin = gpio_pin;
                                gpio_pin_0.set_high().unwrap();
                                delay.delay_ms(500);
                                gpio_pin_0.set_low().unwrap();
                                delay.delay_ms(500);
                            },
                            Some("01") => {
                                let cmd_0 = &cmd_data;
                                //let mut pin = gpio_pin;
                                gpio_pin_1.set_high().unwrap();
                                delay.delay_ms(500);
                                gpio_pin_1.set_low().unwrap();
                                delay.delay_ms(500);
                            },
                            _ => ()
                        }
                        
                        

                        while !wr_ptr.is_empty() {
                            match serial.write(wr_ptr) {
                                Ok(len) => wr_ptr = &wr_ptr[len..],
                                // On error, just drop unwritten data.
                                // One possible error is Err(WouldBlock), meaning the USB
                                // write buffer is full.
                                Err(_) => break,
                            };
                        }
                        //
                    }
                }
            }*/
        }
        
    }
//#####################################################################################################################//
//#####################################################################################################################//
//#####################################################################################################################//
//Pin<Gpio0, Output<PushPull>>
//#[derive(Debug)]

struct GoodeStickCommandString {
    gpio_0_number: String<2>,
    gpio_1_number: String<2>,
    gpio_0_start_state: String<1>,
    gpio_1_start_state: String<1>,
    gpio_0_active_state: String<1>,
    gpio_1_active_state: String<1>,
    pulse_duration:String<1>,
    time_mode:String<1>,
    program_mode:String<1>,
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

/*
#[derive(Debug)]
enum selected_gpio_pin {
    Gpio0(rp2040_hal::gpio::Pin<Gpio25, PushPullOutput>),
}
*/

//fn gpio_25_fire_pin(gpioPing)

//usb_devices=($(ls -lah /dev/ttyACM* | awk {'print$10'}))
//export test_usb=$for i in ${usb_devices[@]};do echo ${i}; done)
