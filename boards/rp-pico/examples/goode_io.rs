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
use core::convert::TryInto;

use cortex_m_rt::entry;
use hal::rom_data::reset_to_usb_boot;
use rp_pico::hal::prelude::*;
use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp_pico::hal::pac;
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::str::FromStr;

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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;
    let mut gpio_pin_0 = pins.gpio0.into_push_pull_output();
    let mut gpio_pin_1 = pins.gpio1.into_push_pull_output();
    let mut gpio_pin_2 = pins.gpio2.into_push_pull_output();
    let mut gpio_pin_3 = pins.gpio3.into_push_pull_output();
    let mut gpio_pin_4 = pins.gpio4.into_push_pull_output();
    let mut gpio_pin_5 = pins.gpio5.into_push_pull_output();
    let mut gpio_pin_6 = pins.gpio6.into_push_pull_output();
    let mut gpio_pin_7 = pins.gpio7.into_push_pull_output();
    let mut gpio_pin_8 = pins.gpio8.into_push_pull_output();
    let mut gpio_pin_9 = pins.gpio9.into_push_pull_output();
    let mut gpio_pin_10 = pins.gpio10.into_push_pull_output();
    let mut gpio_pin_11 = pins.gpio11.into_push_pull_output();
    let mut gpio_pin_12 = pins.gpio12.into_push_pull_output();
    let mut gpio_pin_13 = pins.gpio13.into_push_pull_output();
    let mut gpio_pin_14 = pins.gpio14.into_push_pull_output();
    let mut gpio_pin_15 = pins.gpio15.into_push_pull_output();
    let mut gpio_pin_16 = pins.gpio16.into_push_pull_output();
    let mut gpio_pin_17 = pins.gpio17.into_push_pull_output();
    loop {
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
                    let mut wr_ptr = &buf[..count];
                    let start_bit = u8::from_be_bytes([wr_ptr[0]].try_into().unwrap());
                    let pin_0_state = u8::from_be_bytes([wr_ptr[1]].try_into().unwrap());
                    let pin_1_state = u8::from_be_bytes([wr_ptr[2]].try_into().unwrap());
                    let pin_2_state = u8::from_be_bytes([wr_ptr[3]].try_into().unwrap());
                    let pin_3_state = u8::from_be_bytes([wr_ptr[4]].try_into().unwrap());
                    let pin_4_state = u8::from_be_bytes([wr_ptr[5]].try_into().unwrap());
                    let pin_5_state = u8::from_be_bytes([wr_ptr[6]].try_into().unwrap());
                    let pin_6_state = u8::from_be_bytes([wr_ptr[7]].try_into().unwrap());
                    let pin_7_state = u8::from_be_bytes([wr_ptr[8]].try_into().unwrap());
                    let pin_8_state = u8::from_be_bytes([wr_ptr[9]].try_into().unwrap());
                    let stop_bit = u8::from_be_bytes([wr_ptr[10]].try_into().unwrap());
                    
                    while !wr_ptr.is_empty() {
                        serial.write(wr_ptr);
                        if start_bit == b'7' && stop_bit == b'7'{

                            if pin_0_state == b'0' {
                                gpio_pin_0.set_low().unwrap();
                            } else if pin_0_state == b'1' {
                                gpio_pin_0.set_high().unwrap();
                            }

                            if pin_1_state == b'0' {
                                gpio_pin_1.set_low().unwrap();
                            } else if pin_1_state == b'1' {
                                gpio_pin_1.set_high().unwrap();
                            }

                            if pin_2_state == b'0' {
                                gpio_pin_2.set_low().unwrap();
                            } else if pin_2_state == b'1' {
                                gpio_pin_2.set_high().unwrap();
                            }

                            if pin_3_state == b'0' {
                                gpio_pin_3.set_low().unwrap();
                            } else if pin_3_state == b'1' {
                                gpio_pin_3.set_high().unwrap();
                            }

                            if pin_4_state == b'0' {
                                gpio_pin_4.set_low().unwrap();
                            } else if pin_4_state == b'1' {
                                gpio_pin_4.set_high().unwrap();
                            }

                            if pin_5_state == b'0' {
                                gpio_pin_5.set_low().unwrap();
                            } else if pin_5_state == b'1' {
                                gpio_pin_5.set_high().unwrap();
                            }

                            if pin_6_state == b'0' {
                                gpio_pin_6.set_low().unwrap();
                            } else if pin_6_state == b'1' {
                                gpio_pin_6.set_high().unwrap();
                            }

                            if pin_7_state == b'0' {
                                gpio_pin_7.set_low().unwrap();
                            } else if pin_7_state == b'1' {
                                gpio_pin_7.set_high().unwrap();
                            }

                            if pin_8_state == b'0' {
                                gpio_pin_8.set_low().unwrap();
                            } else if pin_8_state == b'1' {
                                gpio_pin_8.set_high().unwrap();
                            }
                            
                            break
                        } else {
                            serial.flush();
                            reset_to_usb_boot(0,0);
                        }
                    }
                }
            }
        }
        serial.flush();
    }
}


