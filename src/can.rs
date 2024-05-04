use crate::{bxcan_to_canserial, canserial_to_bxcan};
use slcan_parser::{CanserialFrame, SlCanBusSpeed, SlcanIncoming};

use core::cell::RefCell;
use core::convert::Infallible;

use device::gpio::vals::{CnfIn, CnfOut, Mode};
use stm32_metapac::{self as device, interrupt};

use lilos::exec::Notify;
use lilos::spsc;
//use lilos::time::{Millis, PeriodicGate};

use defmt::*;

/// Configuration of pins for CAN controller
/// A12A11 is can tx = pin A12, can rx = pin A11
#[derive(Copy, Clone, Format)]
pub enum BxCanPins {
    A12A11,
    B9B8,
    D1D0,
}

/// BitTime for CAN bus, select the bus speed in KHz
#[derive(Copy, Clone, Format)]
pub enum BxCanBitTiming {
    B1000,
    B800,
    B500,
    B250,
    B125,
    B100,
    B50,
    B20,
    B10,
}

/// Get the bit time register values from http://www.bittiming.can-wiki.info/
/// The clock speed is 36MHz, Sample point is 87.5%, SJW=1
impl From<BxCanBitTiming> for u32 {
    fn from(bt: BxCanBitTiming) -> u32 {
        match bt {
            BxCanBitTiming::B1000 => 0x001e0001,
            BxCanBitTiming::B800 => 0x001b0002,
            BxCanBitTiming::B500 => 0x001e0003,
            BxCanBitTiming::B250 => 0x001c0008,
            BxCanBitTiming::B100 => 0x001e0013,
            BxCanBitTiming::B125 => 0x001c0011,
            BxCanBitTiming::B50 => 0x001c002c,
            BxCanBitTiming::B20 => 0x001e0063,
            BxCanBitTiming::B10 => 0x001c00e0,
        }
    }
}

/// Get the bit time register values from http://www.bittiming.can-wiki.info/
/// The clock speed is 36MHz, Sample point is 87.5%, SJW=1
impl From<SlCanBusSpeed> for BxCanBitTiming {
    fn from(bs: SlCanBusSpeed) -> BxCanBitTiming {
        match bs {
            SlCanBusSpeed::C10 => BxCanBitTiming::B10,
            SlCanBusSpeed::C20 => BxCanBitTiming::B20,
            SlCanBusSpeed::C50 => BxCanBitTiming::B50,
            SlCanBusSpeed::C100 => BxCanBitTiming::B100,
            SlCanBusSpeed::C125 => BxCanBitTiming::B125,
            SlCanBusSpeed::C250 => BxCanBitTiming::B250,
            SlCanBusSpeed::C500 => BxCanBitTiming::B500,
            SlCanBusSpeed::C800 => BxCanBitTiming::B800,
            SlCanBusSpeed::C1000 => BxCanBitTiming::B1000,
        }
    }
}

/// BxCan encapsulates CAN controller, implements bxcan Traits `Instance` and `FilterOwner`
/// This is more convenient then poking the hardware registers directly. The bxcan crate
/// is intended to be used by HAL authors
pub struct BxCan {
    _can: device::can::Can,
}

impl BxCan {
    /// Create an instance of `BxCan` given a pin selection
    pub fn new(
        can: device::can::Can,
        rcc: device::rcc::Rcc,
        afio: device::afio::Afio,
        pin_sel: BxCanPins,
    ) -> Self {
        // turn on CAN clock
        rcc.apb1enr().modify(|w| w.set_canen(true));
        let (port, reg, tx, rx, remap) = match pin_sel {
            BxCanPins::A12A11 => {
                // Default alternate configuration, can remap 0b_00
                rcc.apb2enr().modify(|w| w.set_gpioaen(true));
                (device::GPIOA, 1, 12 - 8, 11 - 8, 0)
            }
            BxCanPins::B9B8 => {
                // can remap 0b_10
                rcc.apb2enr().modify(|w| w.set_gpioben(true));
                (device::GPIOB, 1, 9 - 8, 8 - 8, 2)
            }
            BxCanPins::D1D0 => {
                // can remap 0b_11
                rcc.apb2enr().modify(|w| w.set_gpioden(true));
                (device::GPIOD, 0, 1, 0, 3)
            }
        };
        port.cr(reg).modify(|w| {
            w.set_cnf_out(tx, CnfOut::ALTPUSHPULL);
            w.set_cnf_in(rx, CnfIn::FLOATING);
            w.set_mode(tx, Mode::OUTPUT50MHZ);
            w.set_mode(rx, Mode::INPUT);
        });
        afio.mapr().modify(|w| w.set_can1_remap(remap));
        Self { _can: can }
    }
}

/// the stm32f103rb has a single instance of the bxcan block, no slave instances
unsafe impl bxcan::Instance for BxCan {
    const REGISTERS: *mut bxcan::RegisterBlock = 0x4000_6400 as *mut bxcan::RegisterBlock;
}

/// The stm32f103rb has 14 filter banks
unsafe impl bxcan::FilterOwner for BxCan {
    const NUM_FILTER_BANKS: u8 = 14;
}

///////////////////////////////////////////////////////////////////////////////
// Task implementations

/// Can service task. The incoming queue can contain `SlcanIncoming` packets which either contain
/// commands to affect the CAN controller state, or a `CanserialFrame` which is to be sent out on
/// the bus. The controller can also receive frames from the CAN bus that need to be converted to a
/// `CanserialFrame` and then placed in the outgoing queue to be sent on the usart
pub async fn can_service(
    can: bxcan::Can<BxCan>,
    mut q: spsc::Pusher<'_, CanserialFrame>,
    mut pkt: spsc::Popper<'_, SlcanIncoming>,
) -> Infallible {
    let can_dev = RefCell::new(can);
    futures::future::join(can_slcan_pkt(&can_dev, &mut pkt), can_rx(&can_dev, &mut q))
        .await
        .0
}

/// CAN receiver task. When a frame is received from the bus, this will be woken by the `Nofifier`
/// The frame can be retrieved, converted to a `CanserialFrame` and placed on the queue
async fn can_rx(
    can: &RefCell<bxcan::Can<BxCan>>,
    q: &mut spsc::Pusher<'_, CanserialFrame>,
) -> Infallible {
    loop {
        debug!("can_rx");
        if let Some(frame) = recv(can).await {
            let cframe = bxcan_to_canserial(&frame).expect("bxcan received bad frame");
            q.reserve().await.push(cframe);
        }
    }
}

/// CAN transmit task. The serial link can send various packets that need to be acted
/// on. Most modify the state of the CAN controller, and one of them is to transmit
/// a frame onto the CAN bus. This task will do all of those actions
async fn can_slcan_pkt(
    can: &RefCell<bxcan::Can<BxCan>>,
    q: &mut spsc::Popper<'_, SlcanIncoming>,
) -> Infallible {
    let mut can_silent = false;
    loop {
        debug!("can_slcan_pkt");
        match q.pop().await {
            SlcanIncoming::Frame(frame) => {
                // check hardware register to see if tx is enabled
                if !device::CAN.msr().read().txm() {
                    warn!("CAN closed, frame discarded");
                } else if can_silent {
                    warn!("CAN silent, frame discarded");
                } else {
                    debug!("CAN frame {:?} sending", frame);
                    // convert the frame to bxcan frame
                    let frame =
                        canserial_to_bxcan(&frame).expect("slcan parser supplied bad frame");
                    // transmit it
                    if let Ok(status) = can.borrow_mut().transmit(&frame) {
                        // if the frame was queued, but a lower priority frame was kicked out,
                        // put the displace frame back into the hardware
                        if status.dequeued_frame().is_some() {
                            let lo_priority_frame = status.dequeued_frame().unwrap();
                            send_lower_priority_frame(can, lo_priority_frame).await;
                            debug!("low priority frame reinserted into hardware");
                        }
                    }
                }
            }
            SlcanIncoming::Open => {
                // CAN open, get hardware to start tx/rx
                can.borrow_mut().modify_config().enable();
                debug!("CAN open");
            }
            SlcanIncoming::Close => {
                // CAN close, disable hardware
                can.borrow_mut().modify_config().leave_disabled();
                debug!("CAN closed");
            }
            SlcanIncoming::Listen => {
                // CAN Listen only, no tx
                can.borrow_mut().modify_config().set_silent(true).enable();
                can_silent = true;
                debug!("CAN listen");
            }
            SlcanIncoming::Speed(spd) => {
                // set CAN bus speed
                let bus_spd: BxCanBitTiming = spd.into();
                can.borrow_mut()
                    .modify_config()
                    .set_bit_timing(bus_spd.into())
                    .enable();
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
}

///////////////////////////////////////////////////////////////////////////////
// Interaction between tasks and ISRs.

/// Notification signal for waking a task from the CAN TX ISR.
static CANTXE: Notify = Notify::new();

/// Implementation factor of `can_slcan_packet` when the hardware returns a lower priority frame.
/// This will resolve when the interrupt signals an empty mailbox, and the lower priority frame
/// is placed into it
///
/// This will only work correctly if USB_HP_CAN1_TX interrupt is enabled at the NVIC.
async fn send_lower_priority_frame(can: &RefCell<bxcan::Can<BxCan>>, frame: &bxcan::Frame) {
    debug!("send_lower_priority_frame");
    // enable tx interrupt
    device::CAN.ier().modify(|w| w.set_tmeie(true));
    CANTXE
        .until(|| {
            device::CAN.tsr().read().tme(0)
                || device::CAN.tsr().read().tme(1)
                || device::CAN.tsr().read().tme(2)
        })
        .await;
    can.borrow_mut().transmit(frame).ok();
    debug!("lower priority frame sent");
}

/// Notification signal for waking a task from the CAN RX ISR's.
static CANRXE: Notify = Notify::new();

/// Implementation factor of `can_service_rx`: reads a byte from `usart`. This will
/// resolve once the USART's receive holding register has become non-empty and
/// we've read the value out.
///
/// This will only work correctly if USB_LP_CAN1_RX0 and CAN1_RX1 interrupt is enabled at the NVIC.
async fn recv(can: &RefCell<bxcan::Can<BxCan>>) -> Option<bxcan::Frame> {
    debug!("recv");
    // enable recv interrupt
    device::CAN.ier().modify(|w| {
        w.set_fmpie(0, true);
        w.set_fmpie(1, true);
    });
    CANRXE
        .until(|| device::CAN.rfr(0).read().fmp() != 0 || device::CAN.rfr(1).read().fmp() != 0)
        .await;
    let frame = match can.borrow_mut().receive() {
        Ok(frame) => Some(frame),
        Err(_) => {
            error!("Lost a frame due to overrun error");
            None
        }
    };
    frame
}

///////////////////////////////////////////////////////////////////////////////
// Interrupt handlers.

/// Interrupt service routine for poking our `Notify` object when
/// CAN tx has sent a frame
#[interrupt]
fn USB_HP_CAN1_TX() {
    let can = device::CAN;
    let ier = can.ier().read();
    let tsr = can.tsr().read();

    // Note: we only honor the condition bits when the corresponding interrupt
    // sources are enabled on the CAN, because otherwise they didn't cause
    // this interrupt.

    if ier.tmeie() && (tsr.rqcp(0) || tsr.rqcp(1) || tsr.rqcp(2)) {
        CANTXE.notify();
        // turn off the interrupt
        device::CAN.ier().modify(|w| w.set_tmeie(false));
    }
}

/// Interrupt service routine for poking our `Notify` object when
/// CAN rx fifo0 has received a frame
#[interrupt]
fn USB_LP_CAN1_RX0() {
    let can = device::CAN;
    let ier = can.ier().read();
    let rfr = can.rfr(0).read();

    // Note: we only honor the condition bits when the corresponding interrupt
    // sources are enabled on the CAN, because otherwise they didn't cause
    // this interrupt.

    if ier.fmpie(0) && rfr.fmp() > 0 {
        CANRXE.notify();
    }
}

/// Interrupt service routine for poking our `Notify` object when
/// CAN rx fifo1 has received a frame
#[interrupt]
fn CAN1_RX1() {
    let can = device::CAN;
    let ier = can.ier().read();
    let rfr = can.rfr(1).read();

    // Note: we only honor the condition bits when the corresponding interrupt
    // sources are enabled on the CAN, because otherwise they didn't cause
    // this interrupt.

    if ier.fmpie(1) && rfr.fmp() > 0 {
        CANRXE.notify();
    }
}

/// Interrupt service routine for poking our `Notify` object when
/// CAN controller status has changed
#[interrupt]
fn CAN1_SCE() {}
