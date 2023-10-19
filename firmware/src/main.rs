#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::{bind_interrupts, Config};
use embassy_stm32::can::bxcan::filter::Mask32;
use embassy_stm32::can::bxcan::{Fifo, Frame, StandardId};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull, Output, Level, Speed};
use embassy_stm32::pac::SYSCFG;
use embassy_stm32::peripherals::CAN;
use embassy_stm32::pac::syscfg::vals::Pa11Pa12Rmp;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    CEC_CAN => Rx0InterruptHandler<CAN>, Rx1InterruptHandler<CAN>, SceInterruptHandler<CAN>, TxInterruptHandler<CAN>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config: Config = Default::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(48));

    let p = embassy_stm32::init(config);
    SYSCFG.cfgr1().modify(|w| w.set_pa11_pa12_rmp(Pa11Pa12Rmp::REMAPPED));

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles239_5);

    let mut vrefint = adc.enable_vref(&mut Delay);
    let vrefint_sample = adc.read_internal(&mut vrefint);
    let convert_to_millivolts = |sample| {
        // From https://www.st.com/resource/en/datasheet/stm32f031c6.pdf
        // 6.3.4 Embedded reference voltage
        const VREFINT_MV: u32 = 1230; // mV

        (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
    };

    let mut can = Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    can.as_mut()
        .modify_filters()
        .clear();

    can.as_mut()
        .modify_config()
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .set_automatic_retransmit(false)
        .leave_disabled();

    can.set_bitrate(125_000);

    can.enable().await;

    let _led_red = Output::new(p.PA0, Level::Low, Speed::Low);
    let _led_green = Output::new(p.PA1, Level::Low, Speed::Low);
    let mut ntc = p.PA4;
    let addr = Input::new(p.PA5, Pull::Up);
    let mut pressure_sensor_2 = p.PA6;
    let mut pressure_sensor_1 = p.PA7;

    let can_id = if addr.is_high() { 0x110 } else { 0x111 };

    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        let voltage_sensor_1 = convert_to_millivolts(adc.read(&mut pressure_sensor_1));
        let voltage_sensor_2 = convert_to_millivolts(adc.read(&mut pressure_sensor_2));
        let voltage_ntc = convert_to_millivolts(adc.read(&mut ntc));
        info!("{:03x}, P1: {}mV, P2: {}mV, NTC: {}mV", can_id, voltage_sensor_1, voltage_sensor_2, voltage_ntc);

        let msg = [
            voltage_sensor_1.to_le_bytes()[0],
            voltage_sensor_1.to_le_bytes()[1],
            voltage_sensor_2.to_le_bytes()[0],
            voltage_sensor_2.to_le_bytes()[1],
            voltage_ntc.to_le_bytes()[0],
            voltage_ntc.to_le_bytes()[1],
            0x00,
            0x00
        ];

        let tx_frame = Frame::new_data(unwrap!(StandardId::new(can_id)), msg);
        can.write(&tx_frame).await;

        ticker.next().await;
    }
}
