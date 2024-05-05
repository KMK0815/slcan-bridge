use crate::{bxcan_to_canserial, can_util, canserial_to_bxcan};
use slcan_parser::{CanserialFrame, SlCanBusSpeed, SlcanIncoming};

use core::cell::RefCell;
use core::convert::Infallible;

use stm32_metapac::{self as device, interrupt};

use lilos::exec::Notify;
use lilos::spsc;
//use lilos::time::{Millis, PeriodicGate};

use futures::future;

use defmt::*;

/// BxCan encapsulates CAN controller, implements bxcan Traits `Instance` and `FilterOwner`
/// This is more convenient then poking the hardware registers directly. The bxcan crate
/// is intended to be used by HAL authors
pub struct BxCan(device::can::Can);

impl BxCan {
    /// Create an instance of `BxCan`
    pub fn new(can: device::can::Can) -> Self {
        Self(can)
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
    can_dev
        .borrow_mut()
        .modify_config()
        .set_silent(false)
        .set_loopback(false)
        .leave_disabled();
    future::join(can_slcan_pkt(&can_dev, &mut pkt), can_rx(&can_dev, &mut q))
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
            debug!("received can frame in can_rx");
            let cframe = bxcan_to_canserial(&frame).expect("unable to convert invalid frame");
            q.reserve().await.push(cframe);
        }
    }
}

async fn can_enable(can: &RefCell<bxcan::Can<BxCan>>) {
    let mcr: *const u32 = 0x4000_6400 as *const u32;
    debug!("starting can enable mcr: {}", unsafe { *mcr });
    loop {
        if device::CAN.msr().read().slak() {
            device::CAN.mcr().modify(|w| {
                w.set_abom(true);
                w.set_sleep(false);
            });
        } else {
            break;
        }
        futures::pending!();
    }
    debug!("can is in normal mode");
}

/// CAN transmit task. The serial link can send various packets that need to be acted
/// on. Most modify the state of the CAN controller, and one of them is to transmit
/// a frame onto the CAN bus. This task will do all of those actions
async fn can_slcan_pkt(
    can: &RefCell<bxcan::Can<BxCan>>,
    q: &mut spsc::Popper<'_, SlcanIncoming>,
) -> Infallible {
    let mut can_silent = false;
    let mut can_normal = false;
    loop {
        debug!("can_slcan_pkt");
        match q.pop().await {
            SlcanIncoming::Frame(frame) => {
                if !can_normal {
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
                        // put the displaced frame back into the hardware
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
                can_enable(&can).await;
                //                can.borrow_mut().modify_config().enable();
                can_normal = true;
                debug!("CAN open");
            }
            SlcanIncoming::Close => {
                // CAN close, disable hardware
                can.borrow_mut().modify_config().leave_disabled();
                can_normal = false;
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
                let bit_rate = match spd {
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
                let bit_timing = can_util::calc_can_timings(36_000_000, bit_rate).unwrap();
                let sjw = u8::from(bit_timing.sync_jump_width) as u32;
                let seg1 = u8::from(bit_timing.seg1) as u32;
                let seg2 = u8::from(bit_timing.seg2) as u32;
                let prescaler = u16::from(bit_timing.prescaler) as u32;
                can.borrow_mut()
                    .modify_config()
                    .set_bit_timing(
                        (sjw - 1) << 24 | (seg1 - 1) << 16 | (seg2 - 1) << 20 | (prescaler - 1),
                    )
                    .leave_disabled();
                debug!("CAN bit rate set to {}", bit_rate);
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
                error!("can_sl_pkt got a Wait, this shouldn't happen");
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
    can.borrow_mut()
        .enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);
    can.borrow_mut()
        .enable_interrupt(bxcan::Interrupt::Fifo1MessagePending);
    // enable recv interrupt
    //    device::CAN.ier().modify(|w| {
    //        w.set_fmpie(0, true);
    //        w.set_fmpie(1, true);
    //    });
    debug!(
        "can isr: fmpie0 {} fmpie1 {}",
        device::CAN.ier().read().fmpie(0),
        device::CAN.ier().read().fmpie(1)
    );
    CANRXE
        .until(|| device::CAN.rfr(0).read().fmp() != 0 || device::CAN.rfr(1).read().fmp() != 0)
        .await;
    debug!("can ready for receive");
    match can.borrow_mut().receive() {
        Ok(frame) => Some(frame),
        Err(_) => {
            error!("Lost a frame due to overrun error");
            None
        }
    }
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
