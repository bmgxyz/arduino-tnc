#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::{
    ptr::{read_volatile, write_volatile},
    sync::atomic::{AtomicU16, Ordering},
};

use embedded_hal::digital::OutputPin;
use heapless::Vec;
use panic_halt as _;

static COUNTER: AtomicU16 = AtomicU16::new(0);
static INCREMENT: AtomicU16 = AtomicU16::new(0);
static SINE_TABLE: [u8; 32] = [
    16, 19, 22, 24, 27, 29, 30, 31, 31, 31, 30, 29, 27, 24, 22, 19, 16, 12, 9, 7, 4, 2, 1, 0, 0, 0,
    1, 2, 4, 7, 9, 12,
];
static mut UPDATE_DAC: bool = false;
static mut NEXT_SYMBOL: bool = false;

const MARK_INCREMENT: u16 = 2100;
const SPACE_INCREMENT: u16 = 3838;

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    let old_counter = COUNTER.load(Ordering::SeqCst);
    let new_counter = old_counter.wrapping_add(INCREMENT.load(Ordering::SeqCst));
    let old_index = old_counter >> 11;
    let new_index = new_counter >> 11;
    if new_index != old_index {
        unsafe { UPDATE_DAC = true };
    }
    COUNTER.store(new_counter, Ordering::SeqCst);
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    unsafe { NEXT_SYMBOL = true };
}

#[arduino_hal::entry]
fn main() -> ! {
    unsafe { avr_device::interrupt::enable() };

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut _serial = arduino_hal::default_serial!(dp, pins, 57600);

    let dac_timer = dp.TC0;
    dac_timer.tccr0a().write(|w| w.wgm0().ctc());
    dac_timer.tccr0b().write(|w| w.cs0().prescale_8());
    // 52 * 8 = 416 cycles; at 16 MHz this gives a period of 26 µs (about 38460 Hz)
    dac_timer.ocr0a().write(|w| w.set(52));
    dac_timer.timsk0().write(|w| w.ocie0a().set_bit());

    let mut dac_0 = pins.d2.into_output();
    let mut dac_1 = pins.d3.into_output();
    let mut dac_2 = pins.d4.into_output();
    let mut dac_3 = pins.d5.into_output();
    let mut dac_4 = pins.d6.into_output();
    let mut led = pins.d13.into_output();

    let symbol_timer = dp.TC1;
    symbol_timer
        .tccr1b()
        .write(|w| w.wgm1().set(0b01).cs1().prescale_256());
    // 52 * 256 = 13_312 cycles; at 16 MHz this gives a period of 832 µs (about 1202 Hz)
    symbol_timer.ocr1a().write(|w| w.set(52));
    symbol_timer.timsk1().write(|w| w.ocie1a().set_bit());

    // ASCII letter B with three idle bits, one start bit, eight data bits, and one stop bit (8N1)
    let message: Vec<bool, 13> = Vec::from_array([
        true, true, true, false, false, true, false, false, false, false, true, false, true,
    ]);
    let mut message_index = 0;

    loop {
        if unsafe { read_volatile(&raw const UPDATE_DAC) } {
            let sine_index =
                ((COUNTER.load(Ordering::SeqCst) >> 11) as usize).clamp(0, SINE_TABLE.len());
            if let Some(sine_value) = SINE_TABLE.get(sine_index) {
                let _ = dac_0.set_state((sine_value & 0b10000 != 0).into());
                let _ = dac_1.set_state((sine_value & 0b01000 != 0).into());
                let _ = dac_2.set_state((sine_value & 0b00100 != 0).into());
                let _ = dac_3.set_state((sine_value & 0b00010 != 0).into());
                let _ = dac_4.set_state((sine_value & 0b00001 != 0).into());
            }
            led.toggle();
            unsafe { write_volatile(&raw mut UPDATE_DAC, false) };
        }
        if unsafe { read_volatile(&raw const NEXT_SYMBOL) } {
            match message.get(message_index).unwrap() {
                true => INCREMENT.store(MARK_INCREMENT, Ordering::SeqCst),
                false => INCREMENT.store(SPACE_INCREMENT, Ordering::SeqCst),
            }
            message_index = (message_index + 1) % message.len();
            unsafe { write_volatile(&raw mut NEXT_SYMBOL, false) };
        }
    }
}
