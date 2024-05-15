#![no_std]
#![no_main]

#![allow(clippy::manual_range_contains)]

extern crate alloc;

use core::{mem::MaybeUninit, ptr::addr_of};

use alloc::{boxed::Box, vec::Vec};
use panic_halt as _;
use wartcl::{Flow, Tcl};
use stm32_metapac as device;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        const HEAP_SIZE: usize = 16384;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe {
            HEAP.init(addr_of!(HEAP_MEM) as usize, HEAP_SIZE);
        }
    }

    let rcc = device::RCC;
    rcc.ahb4enr().modify(|v| {
        v.set_gpioaen(true);
        v.set_gpioben(true);
        v.set_gpiocen(true);
        v.set_gpioden(true);
        v.set_gpioeen(true);
        // alright, I'm tired of turning on GPIO ports, if we need ports above E
        // I'll fix it later.
    });
    cortex_m::asm::isb();

    device::GPIOB.moder().modify(|v| {
        v.set_moder(0, device::gpio::vals::Moder::OUTPUT);
    });

    let mut tcl = Tcl::init();
    tcl.register(b"setpin", 3, cmd_setpin);
    tcl.register(b"clrpin", 3, cmd_clrpin);
    tcl.register(b"delay", 2, cmd_delay);
    let _ = tcl.eval(br#"
            while {== 0 0} {
                setpin b 0
                delay 500
                clrpin b 0
                delay 500
            }
        "#);
    panic!("bad");
}

fn cmd_setpin(interp: &mut Tcl, args: Vec<Box<[u8]>>) -> wartcl::Flow {
    let port = &*args[1];
    let bank = match port {
        b"b" | b"B" => device::GPIOB,
        _ => return interp.set_result(Flow::Error, Box::new([])),
    };
    let pin = wartcl::int(&args[2]);
    if pin < 0 || pin > 15 {
        return interp.set_result(Flow::Error, (*b"invalid pin index").into());
    }
    bank.bsrr().write(|v| v.set_bs(pin as usize, true));
    Flow::Normal
}

fn cmd_clrpin(interp: &mut Tcl, args: Vec<Box<[u8]>>) -> wartcl::Flow {
    let port = &*args[1];
    let bank = match port {
        b"b" | b"B" => device::GPIOB,
        _ => return interp.set_result(Flow::Error, Box::new([])),
    };
    let pin = wartcl::int(&args[2]);
    if pin < 0 || pin > 15 {
        return interp.set_result(Flow::Error, (*b"invalid pin index").into());
    }
    bank.bsrr().write(|v| v.set_br(pin as usize, true));
    Flow::Normal
}

fn cmd_delay(_interp: &mut Tcl, args: Vec<Box<[u8]>>) -> wartcl::Flow {
    let interval = wartcl::int(&args[1]);
    // Assuming our clock frequency is 64 MHz.
    cortex_m::asm::delay(interval as u32 * 64_000);
    Flow::Normal
}
