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

// The macro for our start-up function
use cortex_m_rt::entry;
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
        .serial_number("Dozo_DD_0")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // Configure one of the pins as an ADC input
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    let mut hit_count = 0;
    let mut cycle_count = 0;
    let full_hit_string = ":full_hit:";
    let single_hit_string = "single_hit:";
    let nl: String<2> = String::from("\n");
    let cycle_count_total = 10000;
    let hits_per_cycle_total = 1;
    let hit_distance_adc_value = 876;
    let mut led_pin = pins.led.into_push_pull_output();
    let mut full_hit_count = 0;
//#####################################################################################################################//
    loop {
//#########################################################################//
        if cycle_count >= cycle_count_total {
            hit_count = 0;
        }
        let pin_adc_counts: u32 = adc.read(&mut adc_pin_0).unwrap();
        let data: String<4> = String::from(pin_adc_counts);

        if hit_count >= hits_per_cycle_total && cycle_count <= cycle_count_total {
            full_hit_count+=full_hit_count+1;
            //continue;
            if full_hit_count >= 0 {
                cycle_count = 0;
                serial.write(full_hit_string.as_bytes());
                serial.write(data.as_bytes());
                serial.write(nl.as_bytes());
                hit_count = 0;
                full_hit_count = 0;
                led_pin.set_high().unwrap();
                delay.delay_ms(500);
                led_pin.set_low().unwrap();
                delay.delay_ms(1);
                continue;
            } else {
                hit_count = 0;
            }
        } else if pin_adc_counts >= hit_distance_adc_value {
            serial.write(single_hit_string.as_bytes());
            serial.write(data.as_bytes());
            serial.write(nl.as_bytes());
            hit_count+=hit_count+1;
        }
        cycle_count+=cycle_count+1;
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
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
                /*Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    //let mut firs_char = buf[0] as char;
                    //if firs_char  == 'S' {
                    //    let data_string: String<4> = String::from("First Letter is S \n");
                    //    serial.write(&[buf[0]]);
                    //}
                    serial.write(&[buf[0], buf[1]]);
                    
                }*/
            }
        }
    }
//#####################################################################################################################//
}
//#####################################################################################################################//
//#####################################################################################################################//


//usb_devices=($(ls -lah /dev/ttyACM* | awk {'print$10'}))
//export test_usb=$for i in ${usb_devices[@]};do echo ${i}; done)
