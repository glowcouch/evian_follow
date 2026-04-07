//! Tools for waiting on motor efficiency to drop.
//!
//! This is typically used for detecting when a game object has been intaked.
//!
//! "Efficiency" is becoming more and more of a misnomer.

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

    /// Returns the velocity of the device
    #[allow(clippy::missing_errors_doc)]
    fn velocity(&self) -> Result<f64, Self::Err>;
}

impl<M: AsRef<[Motor]> + AsMut<[Motor]>> Efficiency for vexide_motorgroup::MotorGroup<M> {
    type Err = vexide_motorgroup::MotorGroupError<vexide::smart::PortError, f64>;

    const UPDATE_INTERVAL: Duration = <Motor as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.torque()
    }

    fn velocity(&self) -> Result<f64, Self::Err> {
        self.velocity()
    }
}

impl Efficiency for Motor {
    type Err = vexide::smart::PortError;

    const UPDATE_INTERVAL: Duration = <Self as SmartDevice>::UPDATE_INTERVAL;

    fn efficiency(&self) -> Result<f64, Self::Err> {
        self.torque()
    }

    fn velocity(&self) -> Result<f64, Self::Err> {
        self.velocity()
    }
}

/// Controller that can wait for motor efficiency to drop.
pub struct IntakeEfficiency {
    /// The rate of change above which the intake is considered triggered.
    pub rate_threshold: f64,

    /// The acceleration above which the intake is considered triggered.
    ///
    /// This is to avoid the issue where the motor has a high torque because it is getting up to speed.
    pub accel_threshold: f64,

    /// The EMA smoothness factor (0,1)
    /// Smaller values smooth the measurements more.
    pub smootheness: f64,
}

impl IntakeEfficiency {
    /// Will return [`core::task::Poll::Pending`] when the efficiency measurement is triggered.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_above<E: Efficiency>(&self, efficiency: &E) -> Result<(), E::Err> {
        let mut ema = Ema::new(self.smootheness);
        let mut acceleration = Differentiate::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let value = efficiency.efficiency()?;
            let velocity = efficiency.velocity()?;

            // calculate new smoothed value
            let smooth = ema.next(value);

            // calculate rate of change between smooth value and new value
            //
            // This should have better response times and less false negatives than calculating
            // the rate of change of the smooothed value.
            let rate = value - smooth;

            // calculate next derivative measurement, skip to next measurement if not enough
            // samples are available
            if let Some(accel) = acceleration.next(velocity)
                && rate > self.rate_threshold
                && accel > self.accel_threshold
            {
                tracing::debug!("rate went above {rate}");
                return Ok(());
            }

            sleep(E::UPDATE_INTERVAL).await;
        }
    }

    /// Will return [`core::task::Poll::Pending`] when the efficiency measurement is not triggered.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_below<E: Efficiency>(&self, efficiency: &E) -> Result<(), E::Err> {
        let mut ema = Ema::new(self.smootheness);
        let mut acceleration = Differentiate::new();

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let value = efficiency.efficiency()?;
            let velocity = efficiency.velocity()?;

            // calculate new smoothed value
            let smooth = ema.next(value);

            // calculate rate of change between smooth value and new value
            //
            // This should have better response times and less false negatives than calculating
            // the rate of change of the smooothed value.
            let rate = value - smooth;

            // calculate next derivative measurement, skip to next measurement if not enough
            // samples are available
            if let Some(accel) = acceleration.next(velocity)
                && rate < self.rate_threshold
                && accel < self.accel_threshold
            {
                tracing::debug!("rate went above {rate}");
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
