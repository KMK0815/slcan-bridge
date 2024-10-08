#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::{select, select::Either3};
use embassy_stm32::can::enums::{BusError, TryReadError};
use embassy_stm32::can::{
    filter, Can, Fifo, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler,
    TxInterruptHandler
};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::peripherals::CAN1;
use embassy_stm32::usart::{BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_stm32::{bind_interrupts, peripherals, rcc, time::Hertz, usart, Config};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Instant, Timer};
use embedded_io_async::{Read, Write};
use slcan_bridge as _; // global logger + panicking-behavior + memory layout
use slcan_parser::{FrameByteStreamHandler, SlCanBusSpeed, SlcanIncoming};
use static_cell::StaticCell;

/// The baud rate of the bridge
const BRIDGE_BAUD_RATE: u32 = 115_200;

// connect the interrupts
bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
});

// buffers for serial port
static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();

/// slcan messages channel
static BCAN_CHANNEL: StaticCell<Channel<NoopRawMutex, SlcanIncoming, 10>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("slcan-bridge!");

    //defmt::trace!("trace");
    //defmt::debug!("debug");
    //defmt::info!("info");
    //defmt::warn!("warn");
    //defmt::error!("error");

    // setup rcc config to use external 8MHz clock
    // signal that is provided on the Nucleo board
    // Final sysclk will be 72MHz
    let mut config: Config = Default::default();
    // config.rcc.hsi = true;
    // config.rcc.hse = Some(rcc::Hse {
    //     freq: Hertz(8_000_000),
    //     mode: rcc::HseMode::Bypass,
    // });
    // config.rcc.sys = rcc::Sysclk::PLL1_P;
    // config.rcc.pll = Some(rcc::Pll {
    //     src: rcc::PllSource::HSE,
    //     prediv: rcc::PllPreDiv::DIV1,
    //     mul: rcc::PllMul::MUL9,
    // });
    // config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
    // config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;

    let p = embassy_stm32::init(config);

    // led's
    let mut led1 = Output::new(p.PA9.degrade(), Level::Low, Speed::Low);
    let mut led2 = Output::new(p.PC7.degrade(), Level::Low, Speed::Low);
    let mut heartbeat_led = Output::new(p.PA5.degrade(), Level::Low, Speed::Low);

    // flash the LED's during init
    led1.set_high();
    led2.set_high();
    heartbeat_led.set_high();

    // data channels
    let bcan_channel = BCAN_CHANNEL.init(Channel::new());

    // usart
    let tx_buf = TX_BUF.init([0u8; 32]);
    let rx_buf = RX_BUF.init([0u8; 16]);
    let mut uart_config = usart::Config::default();
    uart_config.baudrate = BRIDGE_BAUD_RATE;
    let uart =
        BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf, rx_buf, uart_config).unwrap();
    let (tx, rx) = uart.split();

    // set alternate pin mapping to B8/B9
    // embassy_stm32::pac::AFIO
    //     .mapr()
    //     .modify(|w| w.set_can1_remap(2));

    // bxcan can device
    let bcan = Can::new(p.CAN1, p.PB8, p.PB9, Irqs);

    Timer::after_millis(800).await;

    led1.set_low();
    led2.set_low();
    heartbeat_led.set_low();

    info!("Starting tasks...");

    unwrap!(spawner.spawn(heartbeat_task(heartbeat_led)));
    unwrap!(spawner.spawn(uart_read_task(rx, bcan_channel.sender())));
    unwrap!(spawner.spawn(bcan_task(tx, bcan, bcan_channel.receiver(), led1, led2)));
}

/// task to flash heartbeat led periodically to signal scheduler is still working
#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after_millis(500).await;
        led.set_high();
        Timer::after_millis(500).await;
    }
}

/// function to write to uart tx
async fn uart_tx(tx: &mut BufferedUartTx<'_>, buf: &[u8]) {
    match tx.write_all(buf).await {
        Ok(()) => {}
        Err(e) => error!("error during tx: {:?}", e),
    }
}

/// task to read uart and extract slcan packets
#[embassy_executor::task]
async fn uart_read_task(
    mut rx: BufferedUartRx<'static>,
    bcan_sender: Sender<'static, NoopRawMutex, SlcanIncoming, 10>,
) {
    info!("uart_read task starting");
    // make a parser
    let mut slcan_parser = FrameByteStreamHandler::new();
    // buffer for incoming data
    let mut buf: [u8; 1] = [0; 1];
    loop {
        match rx.read_exact(&mut buf).await {
            Ok(()) => {
                debug!("rx:0x{:x}", buf[0]);
                match slcan_parser.feed(buf[0]) {
                    // nothing to do
                    Ok(SlcanIncoming::Wait) => {}
                    // packet completed
                    Ok(pkt) => {
                        debug!("received incoming {:?}", pkt);
                        bcan_sender.send(pkt).await;
                    }
                    Err(e) => error!("error parsing: {:?}", e),
                }
            }
            Err(e) => error!("uart rx error: {:?}", e),
        }
    }
}

/// function to alternate led's at 2Hz to signal CAN Bus passive
async fn bus_passive(bcan: &mut Can<'_>, led1: &mut Output<'static>, led2: &mut Output<'static>) {
    loop {
        led1.set_low();
        led2.set_high();
        Timer::after_millis(250).await;
        led1.set_high();
        led2.set_low();
        Timer::after_millis(250).await;
        match bcan.try_read() {
            // if we get a frame, just drop it, the stack
            // should recover in some time
            Ok(_) | Err(TryReadError::Empty) => break,
            Err(TryReadError::BusError(BusError::BusOff)) => {
                // exit this fn, it should enter the bus_off fn
                // from the main loop subsequently
                error!("bus is now off");
                break;
            }
            _ => {}
        }
    }
    led1.set_low();
    led2.set_low();
}

/// function to alternate led's at 4Hz to signal CAN Bus off
async fn bus_off(led1: &mut Output<'static>, led2: &mut Output<'static>) {
    loop {
        led1.set_low();
        led2.set_high();
        Timer::after_millis(125).await;
        led1.set_high();
        led2.set_low();
        Timer::after_millis(125).await;
    }
}

/// task for bcan peripheral
#[embassy_executor::task]
async fn bcan_task(
    mut tx: BufferedUartTx<'static>,
    mut bcan: Can<'static>,
    receiver: Receiver<'static, NoopRawMutex, SlcanIncoming, 10>,
    mut led1: Output<'static>,
    mut led2: Output<'static>,
) {
    info!("bcan task starting");
    let mut can_enabled: bool = false;
    let mut can_silent = false;

    bcan.modify_filters()
        .enable_bank(0, Fifo::Fifo0, filter::Mask32::accept_all());

    bcan.modify_config()
        .set_loopback(false)
        .set_silent(can_silent)
        .set_bitrate(250_000);

    let mut tx_timer_last = None;
    let mut rx_timer_last = None;
    loop {
        // check the channel for slcan packets
        // check CAN device for received frames
        // timeout to turn off rx/tx led's after a wait
        match select::select3(receiver.receive(), bcan.read(), Timer::after_millis(10)).await {
            Either3::First(recv) => {
                debug!("received slcan packet from host");
                match recv {
                    SlcanIncoming::Frame(frame) => {
                        // serial port has passed a frame to transmit on the hardware
                        tx_timer_last.replace(Instant::now());
                        led1.set_high();
                        if !can_enabled || can_silent {
                            warn!("CAN silent, frame discarded");
                        } else {
                            // safety - parser won't allow invalid frames, so this is ok
                            let frame = slcan_bridge::canserial_to_bxcan(&frame)
                                .expect("slcan parser supplied bad frame");
                            let status = bcan.write(&frame).await;
                            debug!("CAN frame {:?} sent", frame);
                            if status.dequeued_frame().is_some() {
                                let lo_priority_frame = status.dequeued_frame().unwrap();
                                bcan.write(lo_priority_frame).await;
                                debug!("low priority frame reinserted into hardware");
                            }
                        }
                    }
                    SlcanIncoming::Open => {
                        // CAN open, get hardware to start tx/rx
                        bcan.enable().await;
                        can_enabled = true;
                        debug!("CAN open");
                    }
                    SlcanIncoming::Close => {
                        // CAN close, disable hardware
                        can_enabled = false;
                        bcan.modify_config();
                        debug!("CAN closed");
                    }
                    SlcanIncoming::Listen => {
                        // CAN Listen only, no tx
                        bcan.modify_config().set_silent(true);
                        can_silent = true;
                        can_enabled = false;
                        debug!("CAN listen");
                    }
                    SlcanIncoming::Speed(spd) => {
                        // set CAN bus speed
                        let bus_spd = match spd {
                            SlCanBusSpeed::C10 => 10_000,
                            SlCanBusSpeed::C20 => 20_000,
                            SlCanBusSpeed::C50 => 50_000,
                            SlCanBusSpeed::C100 => 100_000,
                            SlCanBusSpeed::C125 => 125_000,
                            SlCanBusSpeed::C250 => 250_000,
                            SlCanBusSpeed::C500 => 500_000,
                            SlCanBusSpeed::C800 => 800_000,
                            SlCanBusSpeed::C1000 => 1_000_000,
                        };
                        bcan.modify_config();
                        bcan.set_bitrate(bus_spd);
                        can_enabled = false;
                        debug!("CAN bus speed set to {}", bus_spd);
                    }
                    SlcanIncoming::ReadStatus => {
                        // read the status register, can unstick the hardware
                        warn!("unimplemented");
                    }
                    SlcanIncoming::BitTime(_bt) => {
                        // set the bit timing register
                        warn!("unimplemented");
                    }
                    SlcanIncoming::Wait => {
                        // this shouldn't be passed to this task, it is filtered out
                        // in the other task
                        error!("this shouldn't happen");
                    }
                }
            }
            Either3::Second(e) => match e {
                Ok(env) => {
                    // we have received a frame from the hardware
                    // this can fail via invalid frame, or not enough space
                    // in vector, neither of which should happen
                    info!("CAN frame received {:?}", env.frame);
                    rx_timer_last.replace(Instant::now());
                    led2.set_high();
                    let tx_pkt =
                        slcan_bridge::bxcan_to_vec(&env.frame).expect("bxcan received bad frame");
                    uart_tx(&mut tx, tx_pkt.as_slice()).await;
                }
                Err(e) => match e {
                    BusError::BusPassive => {
                        error!("Bus passive, waiting for it to stabilize");
                        bus_passive(&mut bcan, &mut led1, &mut led2).await;
                    }
                    BusError::BusOff => {
                        bcan.modify_config();
                        can_enabled = false;
                        error!("Bus off, turned off can");
                        bus_off(&mut led1, &mut led2).await;
                    }
                    _ => error!("CAN Bus error, what happened: {:?}", e),
                },
            },
            Either3::Third(_) => {
                let t = Instant::now();

                if tx_timer_last.is_some() && (t - tx_timer_last.unwrap()).as_millis() > 50 {
                    led1.set_low();
                    tx_timer_last = None;
                }

                if rx_timer_last.is_some() && (t - rx_timer_last.unwrap()).as_millis() > 50 {
                    led2.set_low();
                    rx_timer_last = None;
                }
            }
        }
    }
}
