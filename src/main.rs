#![no_main]
#![no_std]

use crate::clocks::set_system_clock_exact;
use cortex_m::singleton;
use fugit::RateExtU32;
use panic_reset as _;
use rand::Rng;
use rp2040_hal::{dma::DMAExt, pac, pio::PIOExt, rosc::RingOscillator, Watchdog};
use rp2040_i2s::I2SOutput;

mod clocks;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const EXTERNAL_CRYSTAL_FREQUENCY_HZ: u32 = 12_000_000;

// 1ms of 48kHz, 24-bit audio in bytes.
const NUM_SAMPLES_PER_MILLISECOND: usize = 48;
const PIO_DMA_BUF_SIZE: usize = 2 * NUM_SAMPLES_PER_MILLISECOND; // One PIO state machine writes 2 audio channels to a buffer over DMA.

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Start Clocks
    // Found with: https://github.com/bschwind/rp2040-clock-calculator
    let desired_system_clock = 61440000.Hz();
    let _clocks = set_system_clock_exact(
        desired_system_clock,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    // PIO Globals
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // I2S output
    let dac_output = I2SOutput::new(&mut pio0, sm0, pins.gpio6, pins.gpio7, pins.gpio8).unwrap();
    let (_dac_sm, _dac_fifo_rx, dac_fifo_tx) = dac_output.split();

    // Start double-buffered DMA output to the DAC
    let dma_buf_1 = singleton!(: [u32; PIO_DMA_BUF_SIZE] = [0; PIO_DMA_BUF_SIZE]).unwrap();
    let dma_buf_2 = singleton!(: [u32; PIO_DMA_BUF_SIZE] = [0; PIO_DMA_BUF_SIZE]).unwrap();

    let dma = pac.DMA.split(&mut pac.RESETS);

    let transfer_config =
        rp2040_hal::dma::double_buffer::Config::new((dma.ch0, dma.ch1), dma_buf_1, dac_fifo_tx);

    let tx_transfer = transfer_config.start();
    let mut tx_transfer = tx_transfer.read_next(dma_buf_2);

    // Random noise generator
    let mut rnd = RingOscillator::new(pac.ROSC).initialize();

    loop {
        if tx_transfer.is_done() {
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();

            // Write our next samples to tx_buf
            for sample in tx_buf.iter_mut() {
                let y: f32 = (rnd.gen::<f32>() * 2.0) - 1.0;
                let signed_sample = (y * 2_147_483_648.0) as i32;
                *sample = signed_sample as u32;
            }

            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
    }
}
