#![no_std]
#![no_main]

use core::cell::UnsafeCell;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{
    gpio::{FunctionI2C, Pin},
    pac,
};

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[repr(C, align(4096))]
struct FlashBlock {
    data: UnsafeCell<[u8; 4096]>,
}

use rp2040_flash::flash;

impl FlashBlock {
    #[inline(never)]
    fn addr(&self) -> u32 {
        &self.data as *const _ as u32
    }

    #[inline(never)]
    fn read(&self) -> &[u8; 4096] {
        // Make sure the compiler can't know that
        // we actually access a specific static
        // variable, to avoid unexpected optimizations
        //
        // (Don't try this with strict provenance.)
        let addr = self.addr();

        unsafe { &*(*(addr as *const Self)).data.get() }
    }

    unsafe fn write_flash(&self, data: &[u8; 4096]) {
        let addr = self.addr() - 0x10000000;
        // defmt::assert!(addr & 0xfff == 0);

        cortex_m::interrupt::free(|_cs| {
            flash::flash_range_erase_and_program(addr, data, true);
        });
    }
}

// TODO safety analysis - this is probably not sound
unsafe impl Sync for FlashBlock {}

#[link_section = ".rodata"]
static TEST: FlashBlock = FlashBlock {
    data: UnsafeCell::new([0x55u8; 4096]),
};

#[rp2040_hal::entry]
fn main() -> ! {
    // info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut left_led_pin = pins.gpio25.into_push_pull_output();
    let mut middle_led_pin = pins.gpio24.into_push_pull_output();
    let mut right_led_pin = pins.gpio26.into_push_pull_output();

    middle_led_pin.set_high().unwrap();
    right_led_pin.set_high().unwrap();
    // Do 5 flashes just to make sure everything works
    for _ in 1..5 {
        left_led_pin.set_high().unwrap();
        timer.delay_ms(500);
        left_led_pin.set_low().unwrap();
        timer.delay_ms(500);
    }

    let psm = pac.PSM;

    // // Reset core1 so it's guaranteed to be running
    // // ROM code, waiting for the wakeup sequence
    // psm.frce_off.modify(|_, w| w.proc1().set_bit());
    // while !psm.frce_off.read().proc1().bit_is_set() {
    //     cortex_m::asm::nop();
    // }
    // psm.frce_off.modify(|_, w| w.proc1().clear_bit());

    let mut unique_id = [0u8; 8];
    unsafe { cortex_m::interrupt::free(|_cs| flash::flash_unique_id(&mut unique_id, true)) };
    // info!("Unique ID {:#x}", unique_id);

    let mut data: [u8; 4096] = *TEST.read();
    data[0] = data[0].wrapping_add(1);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    unsafe { TEST.write_flash(&data) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let read_data: [u8; 4096] = *TEST.read();
    // info!("Contents start with {=[u8]:#x}", read_data[0..4]);

    if read_data[0] != 0x56 {
        left_led_pin.set_low().unwrap();
    } else {
        left_led_pin.set_high().unwrap();
    }

    data[0] = data[0].wrapping_sub(1);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    unsafe { TEST.write_flash(&data) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
