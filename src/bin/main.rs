#![no_main]
#![no_std]

//use core::fmt::Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::{DMA1_CH2, DMA1_CH3, SPI1, USART2};
use embassy_stm32::spi::Spi;
use embassy_stm32::usart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config};
use embassy_stm32::{bind_interrupts, gpio::*, peripherals, spi, time::Hertz, usart};
use embassy_time::{Delay, Timer};
use embedded_hal::can::{ExtendedId, Id};
use embedded_io_async::{Read, Write};
use mcp2515::{error::Error, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use slcan_bridge as _; // global logger + panicking-behavior + memory layout
use slcan_parser::{CanserialFrame, FrameByteStreamHandler};
use static_cell::StaticCell;

// connect the interrupts
bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

// buffers for serial port
static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 4]> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("slcan-bridge!");

    //defmt::trace!("trace");
    //defmt::debug!("debug");
    //defmt::info!("info");
    //defmt::warn!("warn");
    //defmt::error!("error");

    let p = embassy_stm32::init(Default::default());

    // usart
    let tx_buf: &'static mut [u8; 32] = TX_BUF.init([0u8; 32]);
    let rx_buf: &'static mut [u8; 4] = RX_BUF.init([0u8; 4]);
    let usart = BufferedUart::new(
        p.USART2,
        Irqs,
        p.PA3,
        p.PA2,
        tx_buf,
        rx_buf,
        Config::default(),
    )
    .unwrap();
    let (mut tx, rx) = usart.split();
    let msg = "Hello, slcan-bridge\r\n";
    tx.write_all(msg.as_bytes()).await.unwrap();

    // spi
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(8_000_000);
    let spi = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config,
    );

    let can_cs = Output::new(p.PB6, Level::High, Speed::Low);
    let mut can = MCP2515::new(spi, can_cs);
    can.init(
        &mut Delay,
        mcp2515::Settings {
            mode: OpMode::Loopback,        // Loopback for testing and example
            can_speed: CanSpeed::Kbps1000, // Many options supported.
            mcp_speed: McpSpeed::MHz16,    // Currently 16MHz and 8MHz chips are supported.
            clkout_en: false,
        },
    )
    .unwrap();

    Timer::after_millis(800).await;

    info!("Starting tasks...");

    unwrap!(spawner.spawn(uart_read(rx,)));
    unwrap!(spawner.spawn(mcp2515_task(tx, can)));
}

/// task to read uart
#[embassy_executor::task]
async fn uart_read(mut rx: BufferedUartRx<'static, USART2>) {
    // make a parser
    let mut slcan_parser = FrameByteStreamHandler::new();
    // buffer for incoming data
    let mut buf: [u8; 1] = [0; 1];
    loop {
        match rx.read_exact(&mut buf).await {
            Ok(()) => {
                info!("RX {=[?]}", buf);
                match slcan_parser.feed(buf[0]) {
                    Ok(f) => {
                        if f.is_some() {
                            let _frame = f.unwrap();
                            info!("received frame");
                        }
                    }
                    Err(_) => error!("error parsing incoming bytes"),
                }
            }
            Err(e) => error!("{:?}", e),
        }
    }
}

/// task for mcp2515
#[embassy_executor::task]
async fn mcp2515_task(
    mut tx: BufferedUartTx<'static, USART2>,
    mut can: MCP2515<Spi<'static, SPI1, DMA1_CH3, DMA1_CH2>, Output<'static>>,
) {
    loop {
        // Send a message
        let frame = CanserialFrame::new_frame(
            Id::Extended(ExtendedId::MAX),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        )
        .unwrap();
        let frame = slcan_bridge::canserial_to_mcp2515(&frame).unwrap();
        can.send_message(frame).unwrap();
        debug!("Sent message!");

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => {
                debug!("Received frame");
                let slcan_frame = slcan_bridge::mcp2515_to_vec(&frame).unwrap();
                unwrap!(tx.write_all(slcan_frame.as_slice()).await);
            }
            Err(Error::NoMessage) => error!("No message to read!"),
            Err(_) => defmt::panic!("Oh no!"),
        }

        Timer::after_millis(1000).await;
    }
}
