#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::RefCell;

use arduino_hal::hal::usart::Event;
use arduino_hal::prelude::*;
use arduino_hal::{
    hal::port::{PD0, PD1, PD2, PD3, PD4, PD5, PD6},
    pac::usart0::RegisterBlock,
    port::{
        mode::{Input, Output},
        Pin,
    },
    Usart,
};
use avr_device::{
    generic::Periph,
    interrupt::{self, Mutex},
};
use embedded_hal::digital::OutputPin;
use heapless::Vec;
use panic_halt as _;

static SINE_TABLE: [u8; 32] = [
    16, 19, 22, 24, 27, 29, 30, 31, 31, 31, 30, 29, 27, 24, 22, 19, 16, 12, 9, 7, 4, 2, 1, 0, 0, 0,
    1, 2, 4, 7, 9, 12,
];

static SERIAL: Mutex<
    RefCell<Option<Usart<Periph<RegisterBlock, 192>, Pin<Input, PD0>, Pin<Output, PD1>>>>,
> = Mutex::new(RefCell::new(None));

static MESSAGE: Mutex<RefCell<Vec<u8, 512>>> = Mutex::new(RefCell::new(Vec::new()));

static SENDER: Mutex<RefCell<Sender>> = Mutex::new(RefCell::new(Sender {
    counter: 0,
    increment: 0,
    byte_index: 0,
    state: SenderState::Off,
    dac_0: None,
    dac_1: None,
    dac_2: None,
    dac_3: None,
    dac_4: None,
}));

const MARK_INCREMENT: u16 = 2100;
const SPACE_INCREMENT: u16 = 3838;

const IDLE_DELAY: usize = 128;

enum SenderState {
    Off,
    Idle { index: usize },
    Start,
    Data { bit_index: usize },
    Stop { end_message: bool },
}

struct Sender {
    counter: u16,
    increment: u16,
    byte_index: usize,
    state: SenderState,
    dac_0: Option<Pin<Output, PD2>>,
    dac_1: Option<Pin<Output, PD3>>,
    dac_2: Option<Pin<Output, PD4>>,
    dac_3: Option<Pin<Output, PD5>>,
    dac_4: Option<Pin<Output, PD6>>,
}

impl Sender {
    fn update(&mut self) {
        match self.state {
            SenderState::Off => (),
            SenderState::Idle { ref mut index } => {
                if *index >= IDLE_DELAY {
                    self.state = SenderState::Start;
                } else {
                    *index += 1;
                }
            }
            SenderState::Start => {
                self.state = SenderState::Data { bit_index: 0 };
            }
            SenderState::Data { ref mut bit_index } => {
                *bit_index += 1;
                if *bit_index >= 8 {
                    self.byte_index += 1;
                    let message_length = interrupt::free(|cs| MESSAGE.borrow(cs).borrow().len());
                    self.state = SenderState::Stop {
                        end_message: self.byte_index >= message_length,
                    };
                }
            }
            SenderState::Stop { end_message } => {
                if end_message {
                    self.state = SenderState::Off;
                    self.byte_index = 0;
                    interrupt::free(|cs| MESSAGE.borrow(cs).borrow_mut().clear());
                } else {
                    self.state = SenderState::Start;
                }
            }
        }
        self.increment = match self.state {
            SenderState::Off => 0,
            SenderState::Idle { .. } => MARK_INCREMENT,
            SenderState::Start => SPACE_INCREMENT,
            SenderState::Data { bit_index } => interrupt::free(|cs| {
                if let Some(&byte) = MESSAGE.borrow(cs).borrow().get(self.byte_index) {
                    let bit = ((byte >> bit_index) & 1) != 0;
                    if bit {
                        MARK_INCREMENT
                    } else {
                        SPACE_INCREMENT
                    }
                } else {
                    0
                }
            }),
            SenderState::Stop { .. } => MARK_INCREMENT,
        };
    }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    interrupt::free(|cs| {
        let mut sender = SENDER.borrow(cs).borrow_mut();
        sender.counter = sender.counter.wrapping_add(sender.increment);
        let sine_index = ((sender.counter >> 11) as usize).clamp(0, SINE_TABLE.len());
        if let Some(sine_value) = SINE_TABLE.get(sine_index) {
            if let Some(ref mut dac_0) = sender.dac_0 {
                let _ = dac_0.set_state((sine_value & 0b10000 != 0).into());
            };
            if let Some(ref mut dac_1) = sender.dac_1 {
                let _ = dac_1.set_state((sine_value & 0b01000 != 0).into());
            };
            if let Some(ref mut dac_2) = sender.dac_2 {
                let _ = dac_2.set_state((sine_value & 0b00100 != 0).into());
            };
            if let Some(ref mut dac_3) = sender.dac_3 {
                let _ = dac_3.set_state((sine_value & 0b00010 != 0).into());
            };
            if let Some(ref mut dac_4) = sender.dac_4 {
                let _ = dac_4.set_state((sine_value & 0b00001 != 0).into());
            };
        }
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    interrupt::free(|cs| {
        let mut sender = SENDER.borrow(cs).borrow_mut();
        sender.update();
    });
}

#[avr_device::interrupt(atmega328p)]
fn USART_RX() {
    interrupt::free(|cs| {
        if let Some(ref mut serial) = SERIAL.borrow(cs).borrow_mut().as_mut() {
            if let Ok(b) = serial.read() {
                if b == b'\n' {
                    SENDER.borrow(cs).borrow_mut().state = SenderState::Idle { index: 0 };
                }
                let _ = MESSAGE.borrow(cs).borrow_mut().push(b);
            }
        }
    });
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    serial.listen(Event::RxComplete);
    interrupt::free(|cs| *SERIAL.borrow(cs).borrow_mut() = Some(serial));

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
    symbol_timer.ocr1a().write(|w| w.set(13_250));
    symbol_timer.timsk1().write(|w| w.ocie1a().set_bit());

    unsafe { avr_device::interrupt::enable() };

    loop {}
}
