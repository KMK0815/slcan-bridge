#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::{select, select::Either};
use embassy_stm32::can::bxcan::filter::Mask32;
use embassy_stm32::can::bxcan::Fifo;
use embassy_stm32::can::{
    Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::peripherals::{CAN, USART2};
use embassy_stm32::usart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::Timer;
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
    let msg = "Hello, slcan-bridge\r\n";
    tx.write_all(msg.as_bytes()).await.unwrap();

    // set alternate pin mapping to B8/B9
    embassy_stm32::pac::AFIO
        .mapr()
        .modify(|w| w.set_can1_remap(2));

    // bxcan can device
    let bcan = Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    Timer::after_millis(800).await;

    info!("Starting tasks...");

    unwrap!(spawner.spawn(uart_read(rx, bcan_channel.sender())));
    unwrap!(spawner.spawn(bcan_task(tx, bcan, bcan_channel.receiver())));
}

/// task to read uart
#[embassy_executor::task]
async fn uart_read(
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
                info!("RX {=[?]}", buf);
                match slcan_parser.feed(buf[0]) {
                    Ok(SlcanIncoming::Wait) => {}
                    Ok(msg) => {
                        info!("received incoming {:?}", msg);
                        bcan_sender.send(msg).await;
                    }
                    Err(_) => error!("error parsing incoming bytes"),
                }
            }
            Err(e) => error!("{:?}", e),
        }
    }
}

/// task for bcan peripheral
#[embassy_executor::task]
async fn bcan_task(
    mut tx: BufferedUartTx<'static, USART2>,
    mut bcan: Can<'static, CAN>,
    receiver: Receiver<'static, NoopRawMutex, SlcanIncoming, 10>,
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

    loop {
        // check the channel for messages to send,
        // can device can signal interrupt
        match select::select(receiver.receive(), bcan.read()).await {
            Either::First(recv) => {
                debug!("received slcan message from host to send on canbus");
                match recv {
                    SlcanIncoming::Frame(frame) => {
                        if !can_enabled {
                            warn!("can disabled, frame discarded");
                        } else {
                            let frame = slcan_bridge::canserial_to_bxcan(&frame).unwrap();
                            bcan.write(&frame).await;
                            debug!("can frame sent");
                        }
                    }
                    SlcanIncoming::Open => {
                        bcan.enable().await;
                        can_enabled = true;
                        debug!("can enabled");
                    }
                    SlcanIncoming::Close => {
                        can_enabled = false;
                        bcan.as_mut().modify_config().leave_disabled();
                        debug!("can disabled");
                    }
                    SlcanIncoming::Listen => {
                        bcan.as_mut()
                            .modify_config()
                            .set_loopback(true)
                            .set_silent(true)
                            .leave_disabled();
                        bcan.set_bitrate(250_000);
                        bcan.enable().await;
                        can_enabled = true;
                        debug!("can set to listen/loopback");
                    }
                    SlcanIncoming::Speed(spd) => {
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
                        debug!("can bus speed set to {}", bus_spd);
                    }
                    SlcanIncoming::ReadStatus => {
                        warn!("unimplemented");
                    }
                    SlcanIncoming::BitTime(_bt) => {
                        warn!("unimplemented");
                    }
                    SlcanIncoming::Wait => {
                        error!("this shouldn't happen");
                    }
                }
            }
            Either::Second(e) => match e {
                Ok(env) => {
                    let tx_pkt = slcan_bridge::bxcan_to_vec(&env.frame).unwrap();
                    info!("got message on bcan {:?}", tx_pkt);
                    unwrap!(tx.write_all(tx_pkt.as_slice()).await);
                }
                Err(_) => error!("what happened"),
            },
        }
    }
}
