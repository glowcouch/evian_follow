//! Tools for waiting on motor efficiency to drop.
//!
//! This is typically used for detecting when a game object has been intaked.
//!
//! "Efficiency" is becoming more and more of a misnomer.

use core::error::Error;
use core::fmt::Debug;
use core::time::Duration;

use crate::util::differentiate::Differentiate;
use crate::util::ema::Ema;
use vexide::prelude::{Motor, sleep};
use vexide::smart::SmartDevice;

/// Things that can measure their efficiency
pub trait Efficiency
where
    Self::Err: 'static + Debug,
{
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

/// Signal processing state for [`IntakeEfficiency`].
pub struct EfficiencyState<'a, E: Efficiency> {
    /// The exponential moving average. Used to calculate baseline.
    ema: Ema,

    /// The differentiation. Used to calculate acceleration.
    acceleration: Differentiate,

    /// The efficiency measurement device.
    efficiency: &'a E,
}

impl<E: Efficiency> EfficiencyState<'_, E> {
    /// Returns the next rate of change value.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if `E::efficiency` fails.
    pub fn next_rate(&mut self) -> anyhow::Result<f64> {
        let value = self
            .efficiency
            .efficiency()
            .map_err(|e| anyhow::anyhow!("failed to measure efficiency: {e:?}"))?;
        let baseline = self.ema.next(value);

        Ok(value - baseline)
    }

    /// Returns the next acceleration value.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if `E::velocity` fails.
    ///
    /// Will return [`None`] if there aren't enough samples to calculate the acceleration.
    pub fn next_acceleration(&mut self) -> anyhow::Result<Option<f64>> {
        let velocity = self
            .efficiency
            .velocity()
            .map_err(|e| anyhow::anyhow!("failed to measure velocity: {e:?}"))?;
        let Some(acceleration) = self.acceleration.next(velocity) else {
            return Ok(None);
        };
        Ok(Some(acceleration))
    }
}

/// Controller that can wait for motor efficiency to drop.
pub struct IntakeEfficiency {
    /// The rate of change above which the intake is considered triggered.
    pub rate_threshold: f64,

    /// The acceleration below which the intake is considered triggered.
    ///
    /// This is to avoid the issue where the motor has a high torque because it is getting up to speed.
    pub accel_threshold: f64,

    /// The EMA smoothness factor (0,1)
    /// Smaller values smooth the measurements more.
    pub smootheness: f64,
}

impl IntakeEfficiency {
    /// Returns a new default state.
    pub fn state<'a, E: Efficiency>(&self, efficiency: &'a E) -> EfficiencyState<'a, E> {
        EfficiencyState {
            ema: Ema::new(self.smootheness),
            acceleration: Differentiate::new(),
            efficiency,
        }
    }

    /// Will return [`core::task::Poll::Pending`] when the efficiency measurement is triggered.
    ///
    /// # Errors
    ///
    /// Will return [`Err`] if measurement of efficiency fails.
    pub async fn wait_above<E: Efficiency>(&self, efficiency: &E) -> anyhow::Result<()> {
        let mut state = self.state(efficiency);

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let rate = state
                .next_rate()
                .map_err(|e| anyhow::anyhow!("failed to measure rate: {e:?}"))?;
            let accel = state
                .next_acceleration()
                .map_err(|e| anyhow::anyhow!("failed to measure acceleration: {e:?}"))?;

            if let Some(accel) = accel
                && rate > self.rate_threshold
                && accel < self.accel_threshold
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
    pub async fn wait_below<E: Efficiency>(&self, efficiency: &E) -> anyhow::Result<()> {
        let mut state = self.state(efficiency);

        loop {
            // exit early if measurement fails because we don't want to get stuck in a loop
            let rate = state
                .next_rate()
                .map_err(|e| anyhow::anyhow!("failed to measure rate: {e:?}"))?;
            let accel = state
                .next_acceleration()
                .map_err(|e| anyhow::anyhow!("failed to measure acceleration: {e:?}"))?;

            if let Some(accel) = accel
                && rate < self.rate_threshold
                && accel > self.accel_threshold
            {
                tracing::debug!("rate went above {rate}");
                return Ok(());
            }

            sleep(E::UPDATE_INTERVAL).await;
        }
    }
}
