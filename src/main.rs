#![no_std]
#![no_main]

#![allow(clippy::manual_range_contains)]

extern crate alloc;

use core::{mem::MaybeUninit, ptr::addr_of};

use alloc::vec;
use device::{gpio::vals::Moder, syscfg::vals::MemMode};
use panic_halt as _;
use wartcl::{empty, Env, FlowChange, OwnedValue, Token, Tokenizer, Value};
use stm32_metapac as device;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        const HEAP_SIZE: usize = 3 * 1024 + 512;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe {
            HEAP.init(addr_of!(HEAP_MEM) as usize, HEAP_SIZE);
        }
    }

    let rcc = device::RCC;
    rcc.apbenr1().write(|w| {
        w.set_usart2en(true);
    });
    rcc.apbenr2().write(|w| {
        w.set_syscfgen(true);
    });
    rcc.gpioenr().modify(|v| {
        v.set_gpioaen(true);
        v.set_gpioben(true);
        v.set_gpiocen(true);
    });
    device::SYSCFG.cfgr1().write(|w| {
        w.0 |= (1 << 4) | (1 << 3);
        w.set_mem_mode(MemMode::MAIN_FLASH);
    });
    device::GPIOA.afr(0).write(|w| {
        w.set_afr(2, 1); // TX
        w.set_afr(3, 1); // RX
    });
    device::GPIOA.moder().modify(|w| {
        w.set_moder(2, Moder::ALTERNATE);
        w.set_moder(3, Moder::ALTERNATE);
    });
    let brr = u16::try_from(16_000_000_u32 / 19_200).unwrap();
    let uart = device::USART2;
    uart.brr().write(|w| w.set_brr(brr));
    uart.cr1().write(|w| {
        w.set_fifoen(true);
        w.set_te(true);
        w.set_re(true);
        w.set_ue(true);
    });

    cortex_m::asm::isb();

    let mut buf = vec![];
    let mut tcl = Env::default();
    tcl.register(b"pinmode", 4, cmd_pinmode);
    tcl.register(b"setpin", 3, cmd_setpin);
    tcl.register(b"clrpin", 3, cmd_clrpin);
    tcl.register(b"delay", 2, cmd_delay);
    tcl.register(b"puts", 0, cmd_puts);
    tcl.register(b"heap", 1, cmd_heap);
    loop {
        match receive() {
            8 | 127 => {
                if buf.pop().is_some() {
                    emit(8);
                    emit(b' ');
                    emit(8);
                } else {
                    emit(7);
                }
                continue;
            }
            c => {
                buf.push(c);
                emit(c);
                if c == b'\r' {
                    emit(b'\n');
                }
                if c != b'\r' && c != b'\n' {
                    // Don't bother tokenizing until end of line.
                    continue;
                }
            }
        }

        let mut p = Tokenizer::new(&buf);
        while let Some(tok) = p.next() {
            if tok == Token::Error && !p.at_end() {
                buf.clear();
                break;
            }

            if matches!(tok, Token::CmdSep(b'\r' | b'\n')) {
                let r = tcl.eval(&buf);
                match r {
                    Err(_) => {
                        emit_s(b"ERROR\r\n");
                    }
                    Ok(result) => {
                        if !result.is_empty() {
                            emit_s(&result);
                            emit_s(b"\r\n");
                        }
                    }
                }
                buf.clear();
                break;
            }
        }
    }
}

fn emit(byte: u8) {
    let uart = device::USART2;
    // Wait for TXE == TXFNF == FIFO not full
    while !uart.isr().read().txe() {}
    uart.tdr().write(|w| w.set_dr(u16::from(byte)));
}

fn emit_s(bytes: &[u8]) {
    for &b in bytes {
        emit(b);
    }
}

fn receive() -> u8 {
    let uart = device::USART2;
    loop {
        let isr = uart.isr().read();
        if isr.ore() || isr.fe() || isr.ne() {
            drain(uart);
            continue;
        }
        if isr.rxne() {
            return uart.rdr().read().0 as u8
        }
    }
}

fn drain(uart: device::usart::Usart) {
    while uart.isr().read().rxne() {
        let _ = uart.rdr().read();
    }
    uart.icr().write(|w| {
        w.set_fe(true);
        w.set_ne(true);
        w.set_ore(true);
    });
}

fn cmd_puts(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    for arg in &args[1..] {
        emit_s(arg);
    }
    emit_s(b"\r\n");
    Ok(empty())
}

fn parse_port(port: &Value) -> Result<device::gpio::Gpio, FlowChange> {
    Ok(match port {
        b"a" | b"A" => device::GPIOA,
        b"b" | b"B" => device::GPIOB,
        b"c" | b"C" => device::GPIOC,
        b"d" | b"D" => device::GPIOD,
        _ => return Err(FlowChange::Error),
    })
}

fn cmd_heap(_interp: &mut Env, _args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let used = HEAP.used();
    let free = HEAP.free();
    let mut text = b"{used ".to_vec();
    text.extend_from_slice(&wartcl::int_value(used as wartcl::Int));
    text.extend_from_slice(b" free ");
    text.extend_from_slice(&wartcl::int_value(free as wartcl::Int));
    text.push(b'}');
    Ok(text.into())
}

fn cmd_pinmode(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = wartcl::int(&args[2]);
    if pin < 0 || pin > 15 {
        return Err(FlowChange::Error);
    }
    let m = match &*args[3] {
        b"output" | b"out" => Moder::OUTPUT,
        b"input" | b"in" => Moder::INPUT,
        _ => return Err(FlowChange::Error),
    };
    bank.moder().modify(|v| v.set_moder(pin as usize, m));
    Ok(empty())
}

fn cmd_setpin(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = wartcl::int(&args[2]);
    if pin < 0 || pin > 15 {
        return Err(FlowChange::Error);
    }
    bank.bsrr().write(|v| v.set_bs(pin as usize, true));
    Ok(empty())
}

fn cmd_clrpin(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = wartcl::int(&args[2]);
    if pin < 0 || pin > 15 {
        return Err(FlowChange::Error);
    }
    bank.bsrr().write(|v| v.set_br(pin as usize, true));
    Ok(empty())
}

fn cmd_delay(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let interval = wartcl::int(&args[1]);
    // Assuming our clock frequency is 64 MHz.
    cortex_m::asm::delay(interval as u32 * 64_000);
    Ok(empty())
}
