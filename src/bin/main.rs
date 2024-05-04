#![no_main]
#![no_std]

use embedded_hal::can::{Frame, Id, StandardId};
use slcan_bridge as _; // global logger + panicking-behavior + memory layout
use slcan_bridge::can::{can_service, BxCan, BxCanBitTiming, BxCanPins};
use slcan_parser::{CanserialFrame, FrameByteStreamHandler, SlCanBusSpeed, SlcanIncoming};

use heapless::Vec;

use core::convert::Infallible;
use core::fmt::Write;
use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::pin;

use device::bdma::vals::{Dir, Pl, Size};
use device::flash::vals::Latency;
use device::gpio::vals::{CnfIn, CnfOut, Mode};
use device::rcc::vals::{Adcpre, Hpre, Pllmul, Pllsrc, Pllxtpre, Ppre, Sw, Usbpre};
use stm32_metapac::{self as device, interrupt};

use lilos::exec::Notify;
use lilos::spsc;
use lilos::time::{Millis, PeriodicGate};

use defmt::*;

///////////////////////////////////////////////////////////////////////////////
// Entry point

#[cortex_m_rt::entry]
fn main() -> ! {
    const CLOCK_HZ: u32 = 72_000_000;
    const BAUD_RATE: u32 = 230_400;

    // Check out peripherals from the runtime. The programming model used by the
    // cortex_m crate expects that there's some common init code where this can
    // be done centrally -- so we pretty much have to do it here.
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Configure sysclk and peripheral clocks
    clock_init();

    info!("Hello");

    // Configure and enable the usart2 device
    // Turn on clock to the USART.
    device::RCC.apb1enr().modify(|w| w.set_usart2en(true));
    // Calculate baud rate divisor for the given peripheral clock. (Using the
    // default 16x oversampling this calculation is pretty straightforward.)
    let cycles_per_bit =
        u16::try_from(CLOCK_HZ / 2 / BAUD_RATE).expect("can't achieve requested baud rate");
    device::USART2.brr().write(|w| w.set_brr(cycles_per_bit));
    // Turn on the USART engine, transmitter, and receiver.
    device::USART2.cr1().write(|w| {
        w.set_ue(true);
        w.set_te(true);
        w.set_re(true);
    });
    // Turn on clock to GPIOA, where our signals emerge.
    device::RCC.apb2enr().modify(|w| w.set_gpioaen(true));
    // Configure our pins as AltFn Output, and Floating Input
    device::GPIOA.cr(0).modify(|w| {
        w.set_cnf_out(2, CnfOut::ALTPUSHPULL);
        w.set_cnf_in(3, CnfIn::FLOATING);
        w.set_mode(2, Mode::OUTPUT10MHZ);
        w.set_mode(3, Mode::INPUT);
    });

    // turn on clock to DMA1
    device::RCC.ahbenr().modify(|w| w.set_dma1en(true));
    // Setup CH7 to transfer to USART2 data register
    device::DMA1.ch(6).par().write_value(0x4000_4404);
    device::DMA1.ch(6).cr().write(|w| {
        w.set_psize(Size::BITS8);
        w.set_msize(Size::BITS8);
        w.set_minc(true);
        w.set_dir(Dir::FROMMEMORY);
        w.set_teie(false);
        w.set_tcie(true);
        w.set_htie(false);
        w.set_circ(false);
        w.set_pl(Pl::VERYHIGH);
        w.set_en(false); // don't start yet
    });

    // configure the bxcan device
    let can_dev = bxcan::Can::builder(BxCan::new(
        device::CAN,
        device::RCC,
        device::AFIO,
        BxCanPins::A12A11,
    ))
    .set_bit_timing(BxCanBitTiming::B1000.into())
    .set_silent(false)
    .leave_disabled();

    // Enable the interrupt's that we'll use to wake tasks.
    // Safety: our ISR (below) is safe to enable at any time -- plus, we're in a
    // future at this point, so interrupts are globally masked.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::DMA1_CHANNEL7);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::USB_HP_CAN1_TX);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::USB_LP_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::CAN1_RX1);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::CAN1_SCE);
    }

    // Create a queue for received usart bytes. The receive task will
    // retrieve those bytes and put them in this queue.
    //
    // First, storage. We'll put this on the stack as part of our async fn;
    // could also be static. The longest packet we can receive is about 28 bytes,
    // so we'll make the queue 32 bytes long. Most likely it will never get near
    // that size as the parsing task will be woken enough to consume it at
    // at about the same speed it is being received. There is other task
    // activity once a packet is received, so that can cause some backup.
    let mut q_storage: [MaybeUninit<u8>; 32] = [MaybeUninit::uninit(); 32];
    // Now, the queue structure,
    let mut q = spsc::Queue::new(&mut q_storage);
    // ...and the two handles to it.
    let (q_push, q_pop) = q.split();

    // Create a queue for CanserialFrame's. The can_service task will
    // receive CAN frames, convert them and put them in this queue.
    //
    // There are 2 FIFO's in the CAN controller, each
    // can hold 3 frames, so we'll make the queue size 6. I know that is simplistic.
    let mut cs_q_storage: [MaybeUninit<CanserialFrame>; 6] = [MaybeUninit::uninit(); 6];
    let mut cs_q = spsc::Queue::new(&mut cs_q_storage);
    let (cs_q_push, cs_q_pop) = cs_q.split();

    // Create a queue for Canserial Packet's. The parsing task will
    // create the packets and pass to the can_service task
    //
    // There are 2 FIFO's in the CAN controller, each
    // can hold 3 frames, so we'll make the queue size 6. I know that is simplistic.
    let mut cspkt_q_storage: [MaybeUninit<SlcanIncoming>; 4] = [MaybeUninit::uninit(); 4];
    let mut cspkt_q = spsc::Queue::new(&mut cspkt_q_storage);
    let (cspkt_q_push, cspkt_q_pop) = cspkt_q.split();

    // Create and pin tasks.
    let heartbeat = pin!(heartbeat(device::RCC, device::GPIOA));
    let receive = pin!(usart_receive(device::USART2, q_push));
    let parsing = pin!(serial_parsing(q_pop, cspkt_q_push));
    let transmit = pin!(serial_frame_transmit(
        device::USART2,
        device::DMA1,
        cs_q_pop,
    ));
    let can = pin!(can_service(can_dev, cs_q_push, cspkt_q_pop));

    // configure LED's A9 & C7
    device::RCC.apb2enr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpiocen(true);
    });
    device::GPIOA.cr(1).modify(|w| {
        w.set_cnf_out(9 - 8, CnfOut::PUSHPULL);
        w.set_mode(9 - 8, Mode::OUTPUT2MHZ);
    });
    device::GPIOC.cr(0).modify(|w| {
        w.set_cnf_out(7, CnfOut::PUSHPULL);
        w.set_mode(7, Mode::OUTPUT2MHZ);
    });
    device::GPIOA.bsrr().write(|w| w.set_br(9 - 8, true));
    device::GPIOC.bsrr().write(|w| w.set_br(7, true));

    // Set up and run the scheduler.
    lilos::time::initialize_sys_tick(&mut cp.SYST, CLOCK_HZ);
    lilos::exec::run_tasks_with_idle(
        &mut [heartbeat, receive, parsing, transmit, can],
        lilos::exec::ALL_TASKS,
        || {
            device::GPIOC.bsrr().write(|w| w.set_br(7, true));
            cortex_m::asm::wfi();
            device::GPIOC.bsrr().write(|w| w.set_bs(7, true));
        },
    )
}

///////////////////////////////////////////////////////////////////////////////
// Clock setup
// Set the clock to use External High Speed oscillator at 8MHz, then use
// the PLL to set sysclk to 72MHz.
fn clock_init() {
    // first enable HSE
    device::RCC.cr().modify(|w| {
        w.set_hsebyp(true);
        w.set_hseon(true);
    });
    while !device::RCC.cr().read().hserdy() {}

    // set PLL configuration
    device::RCC.cfgr().modify(|w| {
        w.set_pllxtpre(Pllxtpre::DIV1);
        w.set_pllmul(Pllmul::MUL9);
        w.set_pllsrc(Pllsrc::HSE_DIV_PREDIV);
    });

    // enable PLL
    device::RCC.cr().modify(|w| w.set_pllon(true));
    while !device::RCC.cr().read().pllrdy() {}

    // set flash
    device::FLASH.acr().modify(|w| {
        w.set_latency(Latency::WS2);
        w.set_prftbe(true);
    });

    // set prescalers
    device::RCC.cfgr().modify(|w| {
        w.set_hpre(Hpre::DIV1); // 72 MHz
        w.set_ppre1(Ppre::DIV2); // 36 MHz
        w.set_ppre2(Ppre::DIV1); // 72 MHz
        w.set_adcpre(Adcpre::DIV6); // 12 MHz
        w.set_usbpre(Usbpre::DIV1_5);
    });

    // Wait for the new prescalars to kick in
    cortex_m::asm::delay(16);

    // switch on PLL
    device::RCC.cfgr().modify(|w| w.set_sw(Sw::PLL1_P));
    while device::RCC.cfgr().read().sws() != Sw::PLL1_P {}
}

///////////////////////////////////////////////////////////////////////////////
// Task implementations

/// Pulses a GPIO pin connected to an LED, to show that the scheduler is still
/// running, etc.
fn heartbeat(rcc: device::rcc::Rcc, gpioa: device::gpio::Gpio) -> impl Future<Output = Infallible> {
    // This is implemented using an explicit `async` block, instead of as an
    // `async fn`, to make it clear which actions occur during setup, and which
    // are ongoing. In particular, we only need to borrow the RCC for *setup*
    // and don't need to retain access to it. This distinction is hard (or
    // impossible?) to express with an `async fn`.

    const PERIOD: Millis = Millis(500);

    // Configure our output pin.
    rcc.apb2enr().modify(|w| w.set_gpioaen(true));
    gpioa.cr(0).modify(|w| {
        w.set_cnf_out(5, CnfOut::PUSHPULL);
        w.set_mode(5, Mode::OUTPUT2MHZ);
    });

    // Set up our timekeeping to capture the current time (not whenever we first
    // get polled). This is usually not important but I'm being picky.
    let mut gate = PeriodicGate::from(PERIOD);

    // Return the task future. We use `move` so that the `gate` is transferred
    // from our stack into the future.
    async move {
        loop {
            gpioa.bsrr().write(|w| w.set_bs(5, true));
            gate.next_time().await;
            gpioa.bsrr().write(|w| w.set_br(5, true));
            gate.next_time().await;
        }
    }
}

/// Serial receive task. Moves bytes from `usart` to `q`.
async fn usart_receive(usart: device::usart::Usart, mut q: spsc::Pusher<'_, u8>) -> Infallible {
    loop {
        debug!("usart_receive");
        q.reserve().await.push(recv(usart).await);
    }
}

/// Serial parsing task. Takes bytes from `q` and assembles into packets
async fn serial_parsing(
    mut q: spsc::Popper<'_, u8>,
    mut pkt_q: spsc::Pusher<'_, SlcanIncoming>,
) -> Infallible {
    // make a parser
    let mut slcan_parser = FrameByteStreamHandler::new();
    loop {
        match slcan_parser.feed(q.pop().await) {
            // nothing to do
            Ok(SlcanIncoming::Wait) => {
                debug!("pkt not complete");
            }
            // packet completed
            Ok(pkt) => {
                debug!("received incoming {:?}", pkt);
                pkt_q.reserve().await.push(pkt);
            }
            Err(e) => error!("error parsing: {:?}", e),
        }
    }
}

/// Frame transmit task. Moves SlcanFrame's from `q` to `usart`.
async fn serial_frame_transmit(
    usart: device::usart::Usart,
    dma_tx: device::bdma::Dma,
    mut q: spsc::Popper<'_, CanserialFrame>,
) -> Infallible {
    loop {
        debug!("serial_frame_transmit");
        send(usart, dma_tx, q.pop().await).await;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Interaction between tasks and ISRs.

/// Notification signal for waking a task from the DMA1 TXC ISR.
static TXE: Notify = Notify::new();

/// Implementation factor of `serial_frame_transmit`: sends `CanserialFrame` out `usart`. This will resolve
/// once the DMA transfer has been completed
///
/// This will only work correctly if DMA1's interrupt is enabled at the NVIC.
async fn send(usart: device::usart::Usart, dma_tx: device::bdma::Dma, frame: CanserialFrame) {
    debug!("send");
    // convert frame to bytes
    let mut buffer: Vec<u8, 32> = Vec::new();
    core::write!(&mut buffer, "{}\r", frame).ok();
    let memptr: *const u8 = buffer.as_ptr();
    // finish the DMA setup with memory size and location, and enable it
    dma_tx.ch(6).mar().write_value(memptr as u32);
    dma_tx
        .ch(6)
        .ndtr()
        .write(|w| w.set_ndt(buffer.len().try_into().unwrap()));
    dma_tx.ch(6).cr().modify(|w| w.set_en(true));
    debug!("mar: {:?}", dma_tx.ch(6).mar().read());
    debug!("ndt: {:?}", dma_tx.ch(6).ndtr().read().ndt());
    // clear the TC bit in usart sr
    usart.sr().modify(|w| w.set_tc(false));
    // Enable the DMA transfer for transmit request by setting the DMA bit in uart CR3
    usart.cr3().write(|w| w.set_dmat(true));
    debug!("send started");
    TXE.until(|| !dma_tx.ch(6).cr().read().en()).await;
}

/// Notification signal for waking a task from the USART RXE ISR.
static RXE: Notify = Notify::new();

/// Implementation factor of `echo_rx`: reads a byte from `usart`. This will
/// resolve once the USART's receive holding register has become non-empty and
/// we've read the value out.
///
/// This will only work correctly if USART2's interrupt is enabled at the NVIC.
async fn recv(usart: device::usart::Usart) -> u8 {
    usart.cr1().modify(|w| w.set_rxneie(true));
    RXE.until(|| usart.sr().read().rxne()).await;
    usart.dr().read().dr() as u8
}

///////////////////////////////////////////////////////////////////////////////
// Interrupt handlers.

/// Interrupt service routine for poking our `Notify` object when
/// usart has received a byte
#[interrupt]
fn USART2() {
    let usart = device::USART2;
    let cr1 = usart.cr1().read();
    let sr = usart.sr().read();

    // Note: we only honor the condition bits when the corresponding interrupt
    // sources are enabled on the USART, because otherwise they didn't cause
    // this interrupt.

    if cr1.rxneie() && sr.rxne() {
        RXE.notify();
        usart.cr1().modify(|w| w.set_rxneie(false));
    }
}

/// Interrupt service routine for poking `Notify` object when DMA
/// has finished with transaction
#[interrupt]
fn DMA1_CHANNEL7() {
    let cr = device::DMA1.ch(6).cr().read();
    let sr = device::DMA1.isr().read();

    // Note: we only honor the condition bits when the corresponding interrupt
    // sources are enabled on the DMA, because otherwise they didn't cause
    // this interrupt.

    // if cr.teie() && sr.teif(7) {
    //     TXE.notify();
    //     device::DMA1.ch(7).cr().modify(|w| w.set_teie(false));
    // }

    if cr.tcie() && sr.tcif(6) {
        TXE.notify();
        // disable DMA1_CH7
        device::DMA1.ch(6).cr().modify(|w| w.set_en(false));
        // clear global interrupts
        device::DMA1.ifcr().write(|w| w.set_gif(6, true));
    }
}

// // connect the interrupts
// bind_interrupts!(struct Irqs {
//     USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
//     USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
//     CAN1_RX1 => Rx1InterruptHandler<CAN>;
//     CAN1_SCE => SceInterruptHandler<CAN>;
//     USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
// });

// // buffers for serial port
// static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
// static RX_BUF: StaticCell<[u8; 4]> = StaticCell::new();

// /// slcan messages channel
// static BCAN_CHANNEL: StaticCell<Channel<NoopRawMutex, SlcanIncoming, 10>> = StaticCell::new();

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     info!("slcan-bridge!");

//     //defmt::trace!("trace");
//     //defmt::debug!("debug");
//     //defmt::info!("info");
//     //defmt::warn!("warn");
//     //defmt::error!("error");

//     // setup rcc config to use external 8MHz clock
//     // signal that is provided on the Nucleo board
//     // Final sysclk will be 72MHz
//     let mut config: Config = Default::default();
//     config.rcc.hsi = true;
//     config.rcc.hse = Some(rcc::Hse {
//         freq: Hertz(8_000_000),
//         mode: rcc::HseMode::Bypass,
//     });
//     config.rcc.sys = rcc::Sysclk::PLL1_P;
//     config.rcc.pll = Some(rcc::Pll {
//         src: rcc::PllSource::HSE,
//         prediv: rcc::PllPreDiv::DIV1,
//         mul: rcc::PllMul::MUL9,
//     });
//     config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
//     config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;

//     let p = embassy_stm32::init(config);

//     // led's
//     let mut led1 = Output::new(p.PA9.degrade(), Level::Low, Speed::Low);
//     let mut led2 = Output::new(p.PC7.degrade(), Level::Low, Speed::Low);

//     // flash the LED's during init
//     led1.set_high();
//     led2.set_high();

//     // data channels
//     let bcan_channel = BCAN_CHANNEL.init(Channel::new());

//     // usart
//     let tx_buf: &'static mut [u8; 32] = TX_BUF.init([0u8; 32]);
//     let rx_buf: &'static mut [u8; 4] = RX_BUF.init([0u8; 4]);
//     let mut uart_config = usart::Config::default();
//     uart_config.baudrate = 230_400;
//     let uart =
//         BufferedUart::new(p.USART2, Irqs, p.PA3, p.PA2, tx_buf, rx_buf, uart_config).unwrap();
//     let (tx, rx) = uart.split();

//     // set alternate pin mapping to B8/B9
//     embassy_stm32::pac::AFIO
//         .mapr()
//         .modify(|w| w.set_can1_remap(2));

//     // bxcan can device
//     let bcan = Can::new(p.CAN, p.PB8, p.PB9, Irqs);

//     Timer::after_millis(800).await;

//     led1.set_low();
//     led2.set_low();

//     info!("Starting tasks...");

//     unwrap!(spawner.spawn(uart_read_task(rx, bcan_channel.sender())));
//     unwrap!(spawner.spawn(bcan_task(tx, bcan, bcan_channel.receiver(), led1, led2)));
// }

// /// function to write to uart tx
// async fn uart_tx(tx: &mut BufferedUartTx<'static, USART2>, buf: &[u8]) {
//     match tx.write_all(buf).await {
//         Ok(()) => {}
//         Err(e) => error!("error during tx: {:?}", e),
//     }
// }

// /// task to read uart and extract slcan packets
// #[embassy_executor::task]
// async fn uart_read_task(
//     mut rx: BufferedUartRx<'static, USART2>,
//     bcan_sender: Sender<'static, NoopRawMutex, SlcanIncoming, 10>,
// ) {
//     info!("uart_read task starting");
//     // make a parser
//     let mut slcan_parser = FrameByteStreamHandler::new();
//     // buffer for incoming data
//     let mut buf: [u8; 1] = [0; 1];
//     loop {
//         match rx.read_exact(&mut buf).await {
//             Ok(()) => {
//                 debug!("rx:0x{:x}", buf[0]);
//                 match slcan_parser.feed(buf[0]) {
//                     // nothing to do
//                     Ok(SlcanIncoming::Wait) => {}
//                     // packet completed
//                     Ok(pkt) => {
//                         debug!("received incoming {:?}", pkt);
//                         bcan_sender.send(pkt).await;
//                     }
//                     Err(e) => error!("error parsing: {:?}", e),
//                 }
//             }
//             Err(e) => error!("uart rx error: {:?}", e),
//         }
//     }
// }

// /// task for bcan peripheral
// #[embassy_executor::task]
// async fn bcan_task(
//     mut tx: BufferedUartTx<'static, USART2>,
//     mut bcan: Can<'static, CAN>,
//     receiver: Receiver<'static, NoopRawMutex, SlcanIncoming, 10>,
//     mut led1: Output<'static>,
//     mut led2: Output<'static>,
// ) {
//     info!("bcan task starting");
//     let mut can_enabled = false;
//     let mut can_silent = false;

//     bcan.as_mut()
//         .modify_filters()
//         .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

//     bcan.as_mut()
//         .modify_config()
//         .set_loopback(false)
//         .set_silent(can_silent)
//         .leave_disabled();

//     bcan.set_bitrate(250_000);
//     let mut tx_timer_last = None;
//     let mut rx_timer_last = None;
//     loop {
//         // check the channel for slcan packets
//         // check CAN device for received frames
//         // timeout to turn off rx/tx led's after a wait
//         match select::select3(receiver.receive(), bcan.read(), Timer::after_millis(5)).await {
//             Either3::First(recv) => {
//                 debug!("received slcan packet from host");
//                 match recv {
//                     SlcanIncoming::Frame(frame) => {
//                         // serial port has passed a frame to transmit on the hardware
//                         tx_timer_last.replace(Instant::now());
//                         led1.set_high();
//                         if !can_enabled {
//                             warn!("CAN closed, frame discarded");
//                         } else if can_silent {
//                             warn!("CAN silent, frame discarded");
//                         } else {
//                             // safety - parser won't allow invalid frames, so this is ok
//                             let frame = slcan_bridge::canserial_to_bxcan(&frame)
//                                 .expect("slcan parser supplied bad frame");
//                             let status = bcan.write(&frame).await;
//                             debug!("CAN frame {:?} sent", frame);
//                             if status.dequeued_frame().is_some() {
//                                 let lo_priority_frame = status.dequeued_frame().unwrap();
//                                 bcan.write(&lo_priority_frame).await;
//                                 debug!("low priority frame reinserted into hardware");
//                             }
//                         }
//                     }
//                     SlcanIncoming::Open => {
//                         // CAN open, get hardware to start tx/rx
//                         bcan.enable().await;
//                         can_enabled = true;
//                         debug!("CAN open");
//                     }
//                     SlcanIncoming::Close => {
//                         // CAN close, disable hardware
//                         can_enabled = false;
//                         bcan.as_mut().modify_config().leave_disabled();
//                         debug!("CAN closed");
//                     }
//                     SlcanIncoming::Listen => {
//                         // CAN Listen only, no tx
//                         bcan.as_mut()
//                             .modify_config()
//                             .set_silent(true)
//                             .leave_disabled();
//                         bcan.enable().await;
//                         can_silent = true;
//                         can_enabled = true;
//                         debug!("CAN listen");
//                     }
//                     SlcanIncoming::Speed(spd) => {
//                         // set CAN bus speed
//                         bcan.as_mut().modify_config().leave_disabled();
//                         let bus_spd = match spd {
//                             SlCanBusSpeed::C10 => 10_000,
//                             SlCanBusSpeed::C20 => 20_000,
//                             SlCanBusSpeed::C50 => 50_000,
//                             SlCanBusSpeed::C100 => 100_000,
//                             SlCanBusSpeed::C125 => 125_000,
//                             SlCanBusSpeed::C250 => 250_000,
//                             SlCanBusSpeed::C500 => 500_000,
//                             SlCanBusSpeed::C800 => 800_000,
//                             SlCanBusSpeed::C1000 => 1_000_000,
//                         };
//                         bcan.set_bitrate(bus_spd);
//                         bcan.enable().await;
//                         can_enabled = true;
//                         debug!("CAN bus speed set to {}", bus_spd);
//                     }
//                     SlcanIncoming::ReadStatus => {
//                         // read the status register, can unstick the hardware
//                         warn!("unimplemented");
//                     }
//                     SlcanIncoming::BitTime(_bt) => {
//                         // set the bit timing register
//                         warn!("unimplemented");
//                     }
//                     SlcanIncoming::Wait => {
//                         // this shouldn't be passed to this task, it is filtered out
//                         // in the other task
//                         error!("this shouldn't happen");
//                     }
//                 }
//             }
//             Either3::Second(e) => match e {
//                 Ok(env) => {
//                     // we have received a frame from the hardware
//                     // this can fail via invalid frame, or not enough space
//                     // in vector, neither of which should happen
//                     info!("CAN frame received {:?}", env.frame);
//                     rx_timer_last.replace(Instant::now());
//                     led2.set_high();
//                     let tx_pkt =
//                         slcan_bridge::bxcan_to_vec(&env.frame).expect("bxcan received bad frame");
//                     uart_tx(&mut tx, tx_pkt.as_slice()).await;
//                 }
//                 Err(e) => match e {
//                     BusError::BusPassive | BusError::BusOff => {
//                         bcan.as_mut().modify_config().leave_disabled();
//                         error!("{:?}, turned off can", e);
//                     }
//                     _ => error!("CAN Bus error, what happened: {:?}", e),
//                 },
//             },
//             Either3::Third(_) => {
//                 let t = Instant::now();
//                 if tx_timer_last.is_some() {
//                     if (t - tx_timer_last.unwrap()).as_millis() > 10 {
//                         led1.set_low();
//                         tx_timer_last = None;
//                     }
//                 }
//                 if rx_timer_last.is_some() {
//                     if (t - rx_timer_last.unwrap()).as_millis() > 10 {
//                         led2.set_low();
//                         rx_timer_last = None;
//                     }
//                 }
//             }
//         }
//     }
// }
