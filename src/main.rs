#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::RefCell;
use core::hint::spin_loop;

use arduino_hal::hal::port::PD7;
use arduino_hal::hal::usart::Event;
use arduino_hal::port::mode::OpenDrain;
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
use crc::{Crc, CRC_16_IBM_SDLC};
use embedded_hal::digital::OutputPin;
use heapless::Vec;
use panic_halt as _;

static TX_WAVE_TABLE: [u8; 32] = [
    0, 0, 1, 2, 4, 7, 9, 12, 16, 19, 22, 24, 27, 29, 30, 31, 31, 31, 30, 29, 27, 24, 22, 19, 16,
    12, 9, 7, 4, 2, 1, 0,
];

type Usart0 = Usart<Periph<RegisterBlock, 192>, Pin<Input, PD0>, Pin<Output, PD1>>;

static SERIAL: Mutex<RefCell<Option<Usart0>>> = Mutex::new(RefCell::new(None));
static MESSAGE: Mutex<RefCell<Vec<u8, 512>>> = Mutex::new(RefCell::new(Vec::new()));
static KISS: Mutex<RefCell<KissState>> = Mutex::new(RefCell::new(KissState::Idle));
static SENDER: Mutex<RefCell<Sender>> = Mutex::new(RefCell::new(Sender {
    counter: 0,
    increment: 0,
    state: SenderState::Off,
    dac_0: None,
    dac_1: None,
    dac_2: None,
    dac_3: None,
    dac_4: None,
    ptt: None,
}));

const MARK_INCREMENT: u16 = 2100;
const SPACE_INCREMENT: u16 = 3838;

enum KissState {
    Idle,
    Fend { send: bool },
    Command,
    Data(u8),
    Fesc,
}

impl KissState {
    const FEND: u8 = 0xc0;
    const FESC: u8 = 0xdb;
    const TFEND: u8 = 0xdc;
    const TFESC: u8 = 0xdd;
    const COMMAND_DATA: u8 = 0x00;

    fn update(&mut self, byte: u8) {
        *self = match (&self, byte) {
            // FEND after data means the frame is complete and ready to send
            (KissState::Data(_), Self::FEND) => KissState::Fend { send: true },

            // FEND anywhere else means the frame is incomplete and should not be sent, but a new
            // frame is beginning
            (_, Self::FEND) => KissState::Fend { send: false },

            // Idle state waits for FEND
            (KissState::Idle, _) => KissState::Idle,

            // Command byte follows FEND
            (KissState::Fend { .. }, Self::COMMAND_DATA) => KissState::Command,
            (KissState::Fend { .. }, _) => KissState::Command,

            // FESC can follow command or data
            (KissState::Command, Self::FESC) => KissState::Fesc,
            (KissState::Data(_), Self::FESC) => KissState::Fesc,

            // Data follows command byte
            (KissState::Command, b) => KissState::Data(b),

            // Handle escaped bytes
            (KissState::Fesc, Self::TFEND) => KissState::Data(Self::FEND),
            (KissState::Fesc, Self::TFESC) => KissState::Data(Self::FESC),
            (KissState::Fesc, b) => KissState::Data(b),

            // Data follows data
            (KissState::Data(_), b) => KissState::Data(b),
        }
    }
}

enum SenderState {
    Off,
    StartFlag {
        bit_index: u8,
        count: usize,
    },
    Payload {
        byte_index: usize,
        bit_index: u8,
        ones_count: usize,
        stuff_bit: bool,
    },
    EndFlag {
        bit_index: u8,
        count: usize,
    },
}

struct Sender {
    counter: u16,
    increment: u16,
    state: SenderState,
    dac_0: Option<Pin<Output, PD2>>,
    dac_1: Option<Pin<Output, PD3>>,
    dac_2: Option<Pin<Output, PD4>>,
    dac_3: Option<Pin<Output, PD5>>,
    dac_4: Option<Pin<Output, PD6>>,
    ptt: Option<Pin<OpenDrain, PD7>>,
}

impl Sender {
    const START_FLAG_BYTES: usize = 45;
    const END_FLAG_BYTES: usize = 15;

    fn u8_index_msb(byte: u8, bit_index: u8) -> bool {
        (byte >> (7 - bit_index)) & 1 != 0
    }
    fn start(&mut self) {
        self.state = SenderState::StartFlag {
            bit_index: 0,
            count: 0,
        };
        if let Some(ref mut ptt) = self.ptt {
            ptt.set_low();
        }
    }
    fn stop(&mut self) {
        self.state = SenderState::Off;
        if let Some(ref mut ptt) = self.ptt {
            ptt.set_high();
        }
    }
    fn set_dac(&mut self, value: u8) {
        if let Some(ref mut dac_0) = self.dac_0 {
            let _ = dac_0.set_state((value & 0b10000 != 0).into());
        };
        if let Some(ref mut dac_1) = self.dac_1 {
            let _ = dac_1.set_state((value & 0b01000 != 0).into());
        };
        if let Some(ref mut dac_2) = self.dac_2 {
            let _ = dac_2.set_state((value & 0b00100 != 0).into());
        };
        if let Some(ref mut dac_3) = self.dac_3 {
            let _ = dac_3.set_state((value & 0b00010 != 0).into());
        };
        if let Some(ref mut dac_4) = self.dac_4 {
            let _ = dac_4.set_state((value & 0b00001 != 0).into());
        };
    }
    fn sample_update(&mut self) {
        if matches!(self.state, SenderState::Off) {
            self.set_dac(16);
            return;
        }
        self.counter = self.counter.wrapping_add(self.increment);
        let sine_index = ((self.counter >> 11) as usize).clamp(0, TX_WAVE_TABLE.len());
        if let Some(sine_value) = TX_WAVE_TABLE.get(sine_index) {
            self.set_dac(*sine_value);
        }
    }
    fn symbol_update(&mut self) {
        let new_symbol = match self.state {
            SenderState::Off => None,
            SenderState::StartFlag {
                bit_index,
                count: _,
            }
            | SenderState::EndFlag {
                bit_index,
                count: _,
            } => Some(Self::u8_index_msb(0x7e, bit_index)),
            SenderState::Payload {
                byte_index,
                bit_index,
                ref mut ones_count,
                ref mut stuff_bit,
            } => {
                if *stuff_bit {
                    *stuff_bit = false;
                    Some(false)
                } else {
                    let maybe_byte =
                        interrupt::free(|cs| MESSAGE.borrow(cs).borrow().get(byte_index).cloned());
                    if let Some(byte) = maybe_byte {
                        let bit = Self::u8_index_msb(byte, bit_index);
                        if bit {
                            *ones_count += 1;
                        } else {
                            *ones_count = 0;
                        }
                        Some(bit)
                    } else {
                        Some(false)
                    }
                }
            }
        };
        self.increment = match new_symbol {
            Some(true) => self.increment,
            Some(false) => {
                if self.increment == MARK_INCREMENT {
                    SPACE_INCREMENT
                } else {
                    MARK_INCREMENT
                }
            }
            None => 0,
        };
        match self.state {
            SenderState::Off => (),
            SenderState::StartFlag {
                ref mut bit_index,
                ref mut count,
            } => {
                *bit_index += 1;
                if *bit_index >= 8 {
                    *count += 1;
                    *bit_index = 0;
                    if *count > Self::START_FLAG_BYTES {
                        self.state = SenderState::Payload {
                            byte_index: 0,
                            bit_index: 0,
                            ones_count: 0,
                            stuff_bit: false,
                        };
                    }
                }
            }
            SenderState::Payload {
                ref mut byte_index,
                ref mut bit_index,
                ref mut ones_count,
                ref mut stuff_bit,
            } => {
                if *ones_count >= 5 {
                    *stuff_bit = true;
                    *ones_count = 0;
                } else {
                    *bit_index += 1;
                    if *bit_index >= 8 {
                        *byte_index += 1;
                        *bit_index = 0;
                        let message_length =
                            interrupt::free(|cs| MESSAGE.borrow(cs).borrow().len());
                        if *byte_index >= message_length {
                            self.state = SenderState::EndFlag {
                                bit_index: 0,
                                count: 0,
                            };
                        }
                    }
                }
            }
            SenderState::EndFlag {
                ref mut bit_index,
                ref mut count,
            } => {
                *bit_index += 1;
                if *bit_index >= 8 {
                    *count += 1;
                    *bit_index = 0;
                    if *count > Self::END_FLAG_BYTES {
                        self.stop();
                        interrupt::free(|cs| MESSAGE.borrow(cs).borrow_mut().clear());
                    }
                }
            }
        }
    }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    interrupt::free(|cs| SENDER.borrow(cs).borrow_mut().sample_update());
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    interrupt::free(|cs| SENDER.borrow(cs).borrow_mut().symbol_update());
}

#[avr_device::interrupt(atmega328p)]
fn USART_RX() {
    interrupt::free(|cs| {
        if let Some(ref mut serial) = SERIAL.borrow(cs).borrow_mut().as_mut() {
            if let Ok(b) = serial.read() {
                let mut kiss_state = KISS.borrow(cs).borrow_mut();
                kiss_state.update(b);
                match *kiss_state {
                    KissState::Fend { send } if send => {
                        let mut sender = SENDER.borrow(cs).borrow_mut();
                        let mut message = MESSAGE.borrow(cs).borrow_mut();

                        let crc = Crc::<u16>::new(&CRC_16_IBM_SDLC);
                        let mut digest = crc.digest();
                        digest.update(message.as_slice());
                        let crc_bytes = digest.finalize().to_le_bytes();

                        for byte in message.iter_mut() {
                            *byte = byte.reverse_bits();
                        }
                        let _ = message.push(crc_bytes[0].reverse_bits());
                        let _ = message.push(crc_bytes[1].reverse_bits());

                        sender.start();
                    }
                    KissState::Data(d) => {
                        let _ = MESSAGE.borrow(cs).borrow_mut().push(d);
                    }
                    KissState::Idle
                    | KissState::Command
                    | KissState::Fend { .. }
                    | KissState::Fesc => (),
                };
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
        sender.ptt = Some(pins.d7.into_opendrain_high());
        sender.set_dac(0);
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
    symbol_timer.ocr1a().write(|w| w.set(13_300));
    symbol_timer.timsk1().write(|w| w.ocie1a().set_bit());

    unsafe { avr_device::interrupt::enable() };

    loop {
        spin_loop();
    }
}
