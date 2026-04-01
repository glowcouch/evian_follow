//! Tools for waiting on motor efficiency to drop.
//!
//! This is typically used for detecting when a game object has been intaked.

use core::time::Duration;
use simple_moving_average::{SMA, SingleSumSMA};
use vexide::prelude::sleep;

use vexide::smart::SmartDevice;

/// Things that can measure their efficiency
pub trait Efficiency {
    /// The error returned by measuring.
    type Err;

    /// The minimum interval at which measurement is possible.
    const UPDATE_INTERVAL: Duration;

    /// Returns efficiency on a scale of 0-1 (where 0 means no losses and 1 means full losses)
    #[allow(clippy::missing_errors_doc)]
    fn efficiency(&self) -> Result<f64, Self::Err>;
}

impl Efficiency for vexide_motorgroup::MotorGroup {
    type Err = vexide_motorgroup::MotorGroupError<vexide::smart::PortError, f64>;

    const UPDATE_INTERVAL: Duration = <vexide::smart::motor::Motor as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.efficiency()
    }
}

impl Efficiency for vexide::smart::motor::Motor {
    type Err = vexide::smart::PortError;

    const UPDATE_INTERVAL: Duration = <Self as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.efficiency()
    }
}

/// Controller that can wait for motor efficiency to drop.
pub struct IntakeEfficiency<const SMA_WINDOW: usize> {
    /// The efficiency threshold, above which the intake is considered triggered.
    pub threshold: f64,
}

impl<const SMA_WINDOW: usize> IntakeEfficiency<SMA_WINDOW> {
    /// Will return [`core::task::Poll::Pending`] when the efficiency is above the threshold.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_above<E: Efficiency>(&self, efficiency: &E) -> Result<(), E::Err> {
        let mut sma: SingleSumSMA<f64, f64, SMA_WINDOW> = SingleSumSMA::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            sma.add_sample(efficiency.efficiency()?);

            if sma.get_average() > self.threshold {
                return Ok(());
            }

            sleep(E::UPDATE_INTERVAL).await;
        }
    }

    /// Will return [`core::task::Poll::Pending`] when the efficiency is below the threshold.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_below<E: Efficiency>(&self, efficiency: &E) -> Result<(), E::Err> {
        let mut sma: SingleSumSMA<f64, f64, SMA_WINDOW> = SingleSumSMA::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            sma.add_sample(efficiency.efficiency()?);

            if sma.get_average() < self.threshold {
                return Ok(());
            }

            sleep(E::UPDATE_INTERVAL).await;
        }
    }
}
