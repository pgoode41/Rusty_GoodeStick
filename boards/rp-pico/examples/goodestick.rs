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
        .serial_number("goodestick_0_device")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

        //let big_yee = String::from("BIG YEEEE!!!\n");

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
     
                        let cmd_data = GoodeStickCommandString {
                            gpio_0_number: String::from(buf[0]+buf[1]),
                            gpio_1_number: String::from(buf[2]+buf[3]),
                            gpio_2_number: String::from(buf[4]+buf[5]),
                            gpio_3_number: String::from(buf[6]+buf[7]),
                            gpio_4_number: String::from(buf[8]+buf[9]),
                            gpio_5_number: String::from(buf[10]+buf[11]),
                            gpio_0_start_state: String::from(buf[12]),
                            gpio_1_start_state: String::from(buf[13]),
                            gpio_2_start_state: String::from(buf[14]),
                            gpio_3_start_state: String::from(buf[15]),
                            gpio_4_start_state: String::from(buf[16]),
                            gpio_5_start_state: String::from(buf[17]),
                            gpio_0_active_state: String::from(buf[19]),
                            gpio_1_active_state: String::from(buf[20]),
                            gpio_2_active_state: String::from(buf[21]),
                            gpio_3_active_state: String::from(buf[22]),
                            gpio_4_active_state: String::from(buf[23]),
                            gpio_5_active_state: String::from(buf[24]),
                            pulse_duration: String::from(buf[25]),
                            time_mode: String::from(buf[26]),
                            program_mode: String::from(buf[27]),

                        };

                        let gpio_num = cmd_data.gpio_0_number;
                        serial.write(gpio_num.as_bytes());

                    }
                }
            }
        }
    }
//#####################################################################################################################//
//#####################################################################################################################//
//#####################################################################################################################//
#[derive(Debug)]
struct GoodeStickCommandString {
    gpio_0_number: String<2>,
    gpio_1_number: String<2>,
    gpio_2_number: String<2>,
    gpio_3_number: String<2>,
    gpio_4_number: String<2>,
    gpio_5_number: String<2>,
    gpio_0_start_state: String<1>,
    gpio_1_start_state: String<1>,
    gpio_2_start_state: String<1>,
    gpio_3_start_state: String<1>,
    gpio_4_start_state: String<1>,
    gpio_5_start_state: String<1>,
    gpio_0_active_state: String<1>,
    gpio_1_active_state: String<1>,
    gpio_2_active_state: String<1>,
    gpio_3_active_state: String<1>,
    gpio_4_active_state: String<1>,
    gpio_5_active_state: String<1>,
    pulse_duration:String<1>,
    time_mode:String<1>,
    program_mode:String<1>,
}
//fn fire_pin(gpioPing)

//usb_devices=($(ls -lah /dev/ttyACM* | awk {'print$10'}))
//export test_usb=$for i in ${usb_devices[@]};do echo ${i}; done)
