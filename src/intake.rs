//! Tools for waiting on motor efficiency to drop.
//!
//! This is typically used for detecting when a game object has been intaked.

use core::time::Duration;

use crate::util::differentiate::Differentiate;
use crate::util::ema::Ema;
use vexide::prelude::{Motor, sleep};
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

impl<M: AsRef<[Motor]> + AsMut<[Motor]>> Efficiency for vexide_motorgroup::MotorGroup<M> {
    type Err = vexide_motorgroup::MotorGroupError<vexide::smart::PortError, f64>;

    const UPDATE_INTERVAL: Duration = <Motor as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.efficiency()
    }
}

impl Efficiency for Motor {
    type Err = vexide::smart::PortError;

    const UPDATE_INTERVAL: Duration = <Self as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.efficiency()
    }
}

/// Controller that can wait for motor efficiency to drop.
pub struct IntakeEfficiency {
    /// The rate of change above which the intake is considered triggered.
    pub threshold: f64,

    /// The EMA smoothness factor (0,1)
    /// Smaller values smooth the measurements more.
    pub smootheness: f64,
}

impl IntakeEfficiency {
    /// Will return [`core::task::Poll::Pending`] when the efficiency is above the threshold.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_above<E: Efficiency>(&self, efficiency: &E) -> Result<(), E::Err> {
        let mut ema = Ema::new(self.smootheness);
        let mut diff = Differentiate::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let next = ema.next(efficiency.efficiency()?);

            // if there isn't enough samples to calculate the rate, wait for the next update
            if let Some(rate) = diff.next(next)
                && rate > self.threshold
            {
                tracing::debug!("rate went above {rate}");
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
        let mut ema = Ema::new(self.smootheness);
        let mut diff = Differentiate::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let next = ema.next(efficiency.efficiency()?);

            // if there isn't enough samples to calculate the rate, wait for the next update
            if let Some(rate) = diff.next(next)
                && rate < self.threshold
            {
                tracing::debug!("rate went below {rate}");
                return Ok(());
            }

            sleep(E::UPDATE_INTERVAL).await;
        }
    }

    /// Will return a vec of `n` samples from the intake.
    ///
    /// This is typically for calibration purposes.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn sample<E: Efficiency>(
        &self,
        efficiency: &E,
        n: usize,
    ) -> Result<Vec<f64>, E::Err> {
        let mut vec = Vec::new();
        let mut ema = Ema::new(self.smootheness);
        let mut diff = Differentiate::new();

        for _ in 0..n {
            let next = ema.next(efficiency.efficiency()?);

            // if there isn't enough samples to calculate the rate, wait for the next update
            if let Some(rate) = diff.next(next) {
                vec.push(rate);
            } else {
                continue;
            }

            sleep(E::UPDATE_INTERVAL).await;
        }

        Ok(vec)
    }
}
