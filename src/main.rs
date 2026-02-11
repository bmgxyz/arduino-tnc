#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::RefCell;

use arduino_hal::{
    hal::port::{PD2, PD3, PD4, PD5, PD6},
    port::{mode::Output, Pin},
};
use avr_device::interrupt::{self, Mutex};
use embedded_hal::digital::OutputPin;
use panic_halt as _;

static SINE_TABLE: [u8; 32] = [
    16, 19, 22, 24, 27, 29, 30, 31, 31, 31, 30, 29, 27, 24, 22, 19, 16, 12, 9, 7, 4, 2, 1, 0, 0, 0,
    1, 2, 4, 7, 9, 12,
];

// ASCII letter B with three idle bits, one start bit, eight data bits, and one stop bit (8N1)
static MESSAGE: [bool; 13] = [
    true, true, true, false, false, true, false, false, false, false, true, false, true,
];

static SENDER: Mutex<RefCell<Sender>> = Mutex::new(RefCell::new(Sender {
    counter: 0,
    increment: 0,
    message_index: 0,
    dac_0: None,
    dac_1: None,
    dac_2: None,
    dac_3: None,
    dac_4: None,
}));

const MARK_INCREMENT: u16 = 2100;
const SPACE_INCREMENT: u16 = 3838;

struct Sender {
    counter: u16,
    increment: u16,
    message_index: usize,
    dac_0: Option<Pin<Output, PD2>>,
    dac_1: Option<Pin<Output, PD3>>,
    dac_2: Option<Pin<Output, PD4>>,
    dac_3: Option<Pin<Output, PD5>>,
    dac_4: Option<Pin<Output, PD6>>,
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    interrupt::free(|cs| {
        let mut sender = SENDER.borrow(cs).borrow_mut();
        sender.counter = sender.counter.wrapping_add(sender.increment);
        let sine_index = ((sender.counter >> 11) as usize).clamp(0, SINE_TABLE.len());
        if let Some(sine_value) = SINE_TABLE.get(sine_index) {
            let _ = sender
                .dac_0
                .as_mut()
                .unwrap()
                .set_state((sine_value & 0b10000 != 0).into());
            let _ = sender
                .dac_1
                .as_mut()
                .unwrap()
                .set_state((sine_value & 0b01000 != 0).into());
            let _ = sender
                .dac_2
                .as_mut()
                .unwrap()
                .set_state((sine_value & 0b00100 != 0).into());
            let _ = sender
                .dac_3
                .as_mut()
                .unwrap()
                .set_state((sine_value & 0b00010 != 0).into());
            let _ = sender
                .dac_4
                .as_mut()
                .unwrap()
                .set_state((sine_value & 0b00001 != 0).into());
        }
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    interrupt::free(|cs| {
        let mut sender = SENDER.borrow(cs).borrow_mut();
        match MESSAGE.get(sender.message_index).unwrap() {
            true => sender.increment = MARK_INCREMENT,
            false => sender.increment = SPACE_INCREMENT,
        }
        sender.message_index = (sender.message_index + 1) % MESSAGE.len();
    });
}

#[arduino_hal::entry]
fn main() -> ! {
    unsafe { avr_device::interrupt::enable() };

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut _serial = arduino_hal::default_serial!(dp, pins, 57600);

    interrupt::free(|cs| {
        let mut sender = SENDER.borrow(cs).borrow_mut();
        sender.dac_0 = Some(pins.d2.into_output());
        sender.dac_1 = Some(pins.d3.into_output());
        sender.dac_2 = Some(pins.d4.into_output());
        sender.dac_3 = Some(pins.d5.into_output());
        sender.dac_4 = Some(pins.d6.into_output());
    });

    let dac_timer = dp.TC0;
    dac_timer.tccr0a().write(|w| w.wgm0().ctc());
    dac_timer.tccr0b().write(|w| w.cs0().prescale_8());
    // 52 * 8 = 416 cycles; at 16 MHz this gives a period of 26 µs (about 38460 Hz)
    dac_timer.ocr0a().write(|w| w.set(52));
    dac_timer.timsk0().write(|w| w.ocie0a().set_bit());

    let symbol_timer = dp.TC1;
    symbol_timer
        .tccr1b()
        .write(|w| w.wgm1().set(0b01).cs1().direct());
    // this should theoretically be 13_333 cycles, but in practice we have to go a little faster to
    // account for overhead; this value seems to work well
    symbol_timer.ocr1a().write(|w| w.set(12_750));
    symbol_timer.timsk1().write(|w| w.ocie1a().set_bit());

    loop {}
}
