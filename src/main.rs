#![no_std]
#![no_main]

extern crate alloc;

use alloc::{vec, vec::Vec};
use core::{mem::MaybeUninit, ptr::addr_of_mut};
use embedded_alloc::Heap;
use panic_halt as _;
use stm32_metapac as device;
use wartcl::{empty, Env, FlowChange, OwnedValue, Token, Tokenizer, Value};

use device::gpio::vals::Moder;

/// Make Rust's alloc crate aware of our allocator implementation.
#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 3 * 1024 + 512; // 3.5 kiB

#[cortex_m_rt::entry]
fn main() -> ! {
    // Initialize the global heap allocator.
    {
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe {
            HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE);
        }
    }

    // Do hardware setup, turning things on and so forth.
    setup_clock_tree();
    let rcc = device::RCC;
    rcc.apbenr1().write(|w| {
        // Make USART2 available.
        w.set_usart2en(true);
    });
    rcc.gpioenr().modify(|v| {
        // Make all three GPIO ports available.
        v.set_gpioaen(true);
        v.set_gpioben(true);
        v.set_gpiocen(true);
    });

    // Configure USART2's TX and RX to connect to the "virtual comm port" on the
    // STLink.
    device::GPIOA.afr(0).write(|w| {
        w.set_afr(2, 1); // TX
        w.set_afr(3, 1); // RX
    });
    device::GPIOA.moder().modify(|w| {
        w.set_moder(2, Moder::ALTERNATE);
        w.set_moder(3, Moder::ALTERNATE);
    });

    // Configure the UART for 19_200 baud.
    let brr = u16::try_from(64_000_000_u32 / 19_200).unwrap();
    let uart = device::USART2;
    uart.brr().write(|w| w.set_brr(brr));
    uart.cr1().write(|w| {
        w.set_fifoen(true);
        w.set_te(true);
        w.set_re(true);
        w.set_ue(true);
    });

    // Set up our Tcl environment with our custom commands.
    let mut tcl = Env::default();
    tcl.register(b"pinmode", 4, cmd_pinmode);
    tcl.register(b"setpin", 3, cmd_setpin);
    tcl.register(b"clrpin", 3, cmd_clrpin);
    tcl.register(b"delay", 2, cmd_delay);
    tcl.register(b"puts", 0, cmd_puts);
    tcl.register(b"heap", 1, cmd_heap);

    // Buffer for accumulating user input:
    let mut buf = vec![];
    loop {
        emit_s(b">>> ");

        'reading:
        loop {
            match receive() {
                8 | 127 => {
                    if buf.pop().is_some() {
                        // Rubout
                        emit(8);
                        emit(b' ');
                        emit(8);
                    } else {
                        // Complain
                        emit(7);
                    }
                    continue 'reading;
                }
                3 => break 'reading,
                c => {
                    buf.push(c);
                    emit(c);
                    if c == b'\r' {
                        // Add newlines, makes terminal emulators happier
                        emit(b'\n');
                    }
                    if c != b'\r' && c != b'\n' {
                        // Don't bother tokenizing until end of line.
                        continue 'reading;
                    }
                }
            }

            // We just got an end-of-line character. The input _might_ be
            // complete... or might span lines. To find out, we hit it with the
            // tokenizer.
            let mut p = Tokenizer::new(&buf);
            while let Some(tok) = p.next() {
                if tok == Token::Error && !p.at_end() {
                    buf.clear();
                    continue 'reading;
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
                    break 'reading;
                }
            }
            // The input spans lines, so give a continuation prompt:
            emit_s(b"... ");
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

#[inline(always)] // expose range of usize to calling function
fn chkpin(pin: i32) -> Result<usize, FlowChange> {
    if (0..=15).contains(&pin) {
        Ok(pin as usize)
    } else {
        Err(FlowChange::Error)
    }
}

/// `heap` - Returns a string describing the used and free bytes in the heap.
fn cmd_heap(_interp: &mut Env, _args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    // Collect heap stats before doing anything that can allocate.
    let used = HEAP.used() as wartcl::Int;
    let free = HEAP.free() as wartcl::Int;

    // We're going to some lengths to avoid reallocating. This is probably not
    // necessary in this command, but provides an example for more complex
    // cases.
    //
    // First, allocate a buffer with enough space for our rendered string. (If
    // we get this math wrong, the code below will wind up having to reallocate
    // the vec behind the scenes.)
    let mut text = Vec::with_capacity(
        b"{used  free }".len()
        + wartcl::int_value_len(used)
        + wartcl::int_value_len(free)
    );
    // Then, render the string in-place:
    text.extend_from_slice(b"{used ");
    wartcl::int_value_into(used, &mut text);
    text.extend_from_slice(b" free ");
    wartcl::int_value_into(free, &mut text);
    text.push(b'}');

    Ok(text.into())
}

/// `pinmode PORT PIN MODE` - sets a GPIO pin to input or output
fn cmd_pinmode(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = chkpin(wartcl::int(&args[2]))?;
    let m = match &*args[3] {
        b"output" | b"out" => Moder::OUTPUT,
        b"input" | b"in" => Moder::INPUT,
        _ => return Err(FlowChange::Error),
    };
    bank.moder().modify(|v| v.set_moder(pin, m));
    Ok(empty())
}

/// `setpin PORT PIN` - sets a pin high
fn cmd_setpin(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = chkpin(wartcl::int(&args[2]))?;
    bank.bsrr().write(|v| v.set_bs(pin, true));
    Ok(empty())
}

/// `clrpin PORT PIN` - sets a pin low
fn cmd_clrpin(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let port = &*args[1];
    let bank = parse_port(port)?;
    let pin = chkpin(wartcl::int(&args[2]))?;
    bank.bsrr().write(|v| v.set_br(pin, true));
    Ok(empty())
}

/// `delay MS` - waits for approximately a certain number of milliseconds.
fn cmd_delay(_interp: &mut Env, args: &mut [OwnedValue]) -> Result<OwnedValue, FlowChange> {
    let interval = wartcl::int(&args[1]);
    // Assuming our clock frequency is 64 MHz.
    cortex_m::asm::delay(interval as u32 * 64_000);
    Ok(empty())
}

fn setup_clock_tree() {
    use device::rcc::vals::{Pllsrc, Sw, Pllm, Plln, Pllr, Pllp, Pllq};
    use device::flash::vals::Latency;

    let flash = device::FLASH;
    let rcc = device::RCC;
    // We come out of reset at 16 MHz on HSI. We would like to be running at 64
    // MHz.
    //
    // This implies that we need to boost the clock speed by 4 using the PLL.
    //
    // Note: to achieve this, we must be in voltage range 1 (required for
    // frequencies above 16 MHz).
    //
    // The PLL's input frequency can go up to 16 MHz, and its internal VCO must
    // be between 64 and 344 MHz (in voltage range 1). Its R-tap is the one that
    // feeds the CPU and buses, so we need the R output to be 64 MHz. The R
    // divisor is limited to integers between 2 and 8 (inclusive), so, we can
    // get what we want by
    //
    // - Setting the PLL input divisor (VCO multiplier) to 8x for an fVCO of 128
    //   MHz.
    // - Setting the R divisor to /2 for a 64 MHz output.
    // - Leaving the P and Q taps off. (Q tap doesn't actually exist on our
    //   part.)

    // The part resets into voltage range 1, delightfully. This means we don't
    // have to adjust it.

    // Adjust our wait states to reflect our target voltage + freq combination.
    // At 64 MHz we'll need 2 wait states. We come out of reset at 0 wait
    // states, so we must override it.
    //
    // Note: the SVD apparently incorrectly models an undocumented reserved bit
    // in this register (bit 18), so DO NOT use `write` or `reset`. If you clear
    // bit 18, bad shit will happen to you -- it interferes with debugger
    // access.
    flash.acr().modify(|w| {
        w.set_latency(Latency::WS2);
        // Prefetch is off at reset, but helps at higher wait states.
        w.set_prften(true);
        // Cache is _on_ at reset, no action required.
    });

    // Fire up the PLL.
    // First, set up our divisors.
    rcc.pllcfgr().write(|w| {
        // Source input from HSI16.
        w.set_pllsrc(Pllsrc::HSI);
        // Leave input undivided.
        w.set_pllm(Pllm::DIV1);
        // Multiply that by 8 in the VCO for 128 MHz.
        w.set_plln(Plln::MUL8);
        // Configure the R-tap to 48 MHz output by dividing by 2. Configure P
        // and Q to valid configurations while we're at it.
        w.set_pllr(Pllr::DIV2);
        w.set_pllp(Pllp::DIV2);
        w.set_pllq(Pllq::DIV2);
        // But we only actually turn the R tap on.
        w.set_pllren(true);
    });
    // Switch it on at the RCC and wait for it to come up. RCC should still be
    // at its reset values, so we don't need to RMW it.
    rcc.cr().write(|w| {
        w.set_pllon(true);
    });
    while !rcc.cr().read().pllrdy() {
        // spin
    }
    // Now switch over to it.
    rcc.cfgr().write(|w| {
        w.set_sw(Sw::PLL1_R);
    });
    while rcc.cfgr().read().sws() != Sw::PLL1_R {
        // spin
    }
}
