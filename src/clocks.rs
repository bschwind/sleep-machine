use crate::EXTERNAL_CRYSTAL_FREQUENCY_HZ;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::{
    clocks::ClocksManager,
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC},
    Watchdog,
};

const CRYSTAL_FREQUENCY_KHZ: u32 = 12_000; // 12 MHz

// Voltage-Controlled Oscillator constants.
const PICO_PLL_VCO_MIN_FREQ_KHZ: u32 = 750_000; // 750 MHz
const PICO_PLL_VCO_MAX_FREQ_KHZ: u32 = 1_600_000; // 1600 MHz

const FEEDBACK_DIVIDER_MIN: u32 = 16;
const FEEDBACK_DIVIDER_MAX: u32 = 320;

const POST_DIVIDER_MIN: u32 = 1;
const POST_DIVIDER_MAX: u32 = 7;

pub fn set_system_clock_exact(
    system_frequency: HertzU32,
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> Option<ClocksManager> {
    let (vco_freq, post_div1, post_div2) = check_sys_clock(system_frequency).unwrap();

    let xosc = rp2040_hal::xosc::setup_xosc_blocking(xosc_dev, EXTERNAL_CRYSTAL_FREQUENCY_HZ.Hz())
        .map_err(|_x| false)
        .unwrap();

    watchdog.enable_tick_generation((EXTERNAL_CRYSTAL_FREQUENCY_HZ / 1_000_000) as u8);

    let mut clocks = rp2040_hal::clocks::ClocksManager::new(clocks_dev);

    let pll_sys = rp2040_hal::pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        rp2040_hal::pll::PLLConfig { vco_freq, refdiv: 1, post_div1, post_div2 },
        // rp2040_hal::pll::PLLConfig { vco_freq: 1584.MHz(), refdiv: 1, post_div1: 6, post_div2: 2 },
        &mut clocks,
        resets,
    )
    .map_err(|_x| false)
    .unwrap();

    let pll_usb = rp2040_hal::pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        rp2040_hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(|_x| false)
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).map_err(|_x| false).unwrap();

    Some(clocks)
}

// TODO(bschwind) - It seems lower jitter can be achieved by optimizing for higher frequencies
//                  and larger dividers.
pub fn check_sys_clock(desired_frequency: HertzU32) -> Option<(HertzU32, u8, u8)> {
    let freq_khz = desired_frequency.to_kHz();
    let reference_divider = 1;
    let reference_freq_khz: u32 = CRYSTAL_FREQUENCY_KHZ / reference_divider;

    for feedback_divide in (FEEDBACK_DIVIDER_MIN..=FEEDBACK_DIVIDER_MAX).rev() {
        let vco_khz = feedback_divide * reference_freq_khz;

        if !(PICO_PLL_VCO_MIN_FREQ_KHZ..=PICO_PLL_VCO_MAX_FREQ_KHZ).contains(&vco_khz) {
            continue;
        }

        for post_div1 in (POST_DIVIDER_MIN..=POST_DIVIDER_MAX).rev() {
            for post_div_2 in (1..=post_div1).rev() {
                let out = vco_khz / (post_div1 * post_div_2);

                if out == freq_khz && (vco_khz % (post_div1 * post_div_2)) == 0 {
                    return Some((
                        vco_khz.kHz(),
                        post_div1.try_into().unwrap(),
                        post_div_2.try_into().unwrap(),
                    ));
                }
            }
        }
    }

    None
}
