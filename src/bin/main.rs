#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::{select, select::Either3};
use embassy_stm32::can::{
    bxcan::filter::Mask32, bxcan::Fifo, Can, Rx0InterruptHandler, Rx1InterruptHandler,
    SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::peripherals::{CAN, USART2};
use embassy_stm32::usart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Instant, Timer};
use embedded_io_async::{Read, Write};
use slcan_bridge as _; // global logger + panicking-behavior + memory layout
use slcan_parser::{FrameByteStreamHandler, SlCanBusSpeed, SlcanIncoming};
use static_cell::StaticCell;

// connect the interrupts
bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
    CAN1_RX1 => Rx1InterruptHandler<CAN>;
    CAN1_SCE => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
});

// buffers for serial port
static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 4]> = StaticCell::new();

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

    let p = embassy_stm32::init(Default::default());

    // led's
    let mut led1 = Output::new(p.PA9.degrade(), Level::Low, Speed::Low);
    let mut led2 = Output::new(p.PC7.degrade(), Level::Low, Speed::Low);

    led2.set_low();
    led1.set_low();

    // data channels
    let bcan_channel = BCAN_CHANNEL.init(Channel::new());

    // usart
    let tx_buf: &'static mut [u8; 32] = TX_BUF.init([0u8; 32]);
    let rx_buf: &'static mut [u8; 4] = RX_BUF.init([0u8; 4]);
    let mut uart_config = Config::default();
    uart_config.baudrate = 230_400;
    let uart =
        BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf, rx_buf, uart_config).unwrap();
    let (mut tx, rx) = uart.split();

    uart_tx(&mut tx, "Hello, slcan-bridge\r\n".as_bytes()).await;

    // set alternate pin mapping to B8/B9
    embassy_stm32::pac::AFIO
        .mapr()
        .modify(|w| w.set_can1_remap(2));

    // bxcan can device
    let bcan = Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    Timer::after_millis(800).await;

    info!("Starting tasks...");

    unwrap!(spawner.spawn(uart_read_task(rx, bcan_channel.sender())));
    unwrap!(spawner.spawn(bcan_task(tx, bcan, bcan_channel.receiver(), led1, led2)));
}

/// function to write to uart tx
async fn uart_tx(tx: &mut BufferedUartTx<'static, USART2>, buf: &[u8]) {
    match tx.write_all(buf).await {
        Ok(()) => {}
        Err(e) => error!("error during tx: {:?}", e),
    }
}

/// task to read uart and extract slcan packets
#[embassy_executor::task]
async fn uart_read_task(
    mut rx: BufferedUartRx<'static, USART2>,
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
                trace!("RX 0x{:x}", buf[0]);
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

/// task for bcan peripheral
#[embassy_executor::task]
async fn bcan_task(
    mut tx: BufferedUartTx<'static, USART2>,
    mut bcan: Can<'static, CAN>,
    receiver: Receiver<'static, NoopRawMutex, SlcanIncoming, 10>,
    mut led1: Output<'static>,
    mut led2: Output<'static>,
) {
    info!("bcan task starting");
    let mut can_enabled = false;

    bcan.as_mut()
        .modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    bcan.as_mut()
        .modify_config()
        .set_loopback(false)
        .set_silent(false)
        .leave_disabled();

    bcan.set_bitrate(250_000);
    let mut tx_timer_last = None;
    let mut rx_timer_last = None;
    loop {
        // check the channel for slcan packets
        // check CAN device for received frames
        match select::select3(receiver.receive(), bcan.read(), Timer::after_millis(50)).await {
            Either3::First(recv) => {
                debug!("received slcan packet from host");
                match recv {
                    SlcanIncoming::Frame(frame) => {
                        // serial port has passed a frame to transmit on the hardware
                        tx_timer_last.replace(Instant::now());
                        led1.set_low();
                        if !can_enabled {
                            warn!("CAN closed, frame discarded");
                        } else {
                            // safety - parser won't allow invalid frames, so this is ok
                            let frame = slcan_bridge::canserial_to_bxcan(&frame)
                                .expect("slcan parser supplied bad frame");
                            let status = bcan.write(&frame).await;
                            debug!("CAN frame {:?} sent", frame);
                            if status.dequeued_frame().is_some() {
                                let lo_priority_frame = status.dequeued_frame().unwrap();
                                bcan.write(&lo_priority_frame).await;
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
                        bcan.as_mut().modify_config().leave_disabled();
                        debug!("CAN closed");
                    }
                    SlcanIncoming::Listen => {
                        // CAN Listen only, no tx
                        bcan.as_mut()
                            .modify_config()
                            .set_loopback(false)
                            .set_silent(true)
                            .leave_disabled();
                        bcan.set_bitrate(250_000);
                        bcan.enable().await;
                        can_enabled = true;
                        debug!("CAN set to listen/loopback");
                    }
                    SlcanIncoming::Speed(spd) => {
                        // set CAN bus speed
                        bcan.as_mut().modify_config().leave_disabled();
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
                        bcan.set_bitrate(bus_spd);
                        bcan.enable().await;
                        can_enabled = true;
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
                    led2.set_low();
                    let tx_pkt =
                        slcan_bridge::bxcan_to_vec(&env.frame).expect("bxcan received bad frame");
                    uart_tx(&mut tx, tx_pkt.as_slice()).await;
                }
                Err(_) => error!("what happened"),
            },
            Either3::Third(_) => {
                let t = Instant::now();
                if tx_timer_last.is_some() {
                    if (t - tx_timer_last.unwrap()).as_millis() > 50 {
                        led1.set_low();
                        tx_timer_last = None;
                    }
                }
                if rx_timer_last.is_some() {
                    if (t - rx_timer_last.unwrap()).as_millis() > 50 {
                        led2.set_low();
                        rx_timer_last = None;
                    }
                }
            }
        }
    }
}
