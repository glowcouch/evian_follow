//! A controller that turns to face a vision sensor object.

use std::time::Instant;

use anyhow::Context;
use evian::{
    control::loops::Feedback,
    drivetrain::model::{Arcade, DrivetrainModel},
    math::Angle,
    prelude::{Drivetrain, Tolerances, TracksForwardTravel, TracksVelocity},
    tracking::Tracking,
};
use simple_moving_average::SMA;
use simple_moving_average::SingleSumSMA;
use vexide::{
    math::Point2,
    prelude::AiVisionSensor,
    smart::{SmartDevice, ai_vision::AiVisionObject},
    time::sleep,
};

pub mod filters;

/// Wrapper around [`AiVisionObject::Color`].
#[allow(clippy::missing_docs_in_private_items, reason = "wrapper")]
pub struct Color {
    id: u8,
    position: Point2<u16>,
    width: u16,
    height: u16,
}

/// Abstraction over vision sensor for better tracking.
///
/// `SMA` is the size of the simple moving average window
struct VisionSensorAbstraction<'a, const SMA_WINDOW: usize> {
    /// The vision sensor
    sensor: &'a AiVisionSensor,
    /// The id of the color object we are tracking.
    object_id: u8,
    /// SMA used to average the angle.
    sma: SingleSumSMA<Angle, f64, SMA_WINDOW>,
}

impl<'a, const SMA_WINDOW: usize> VisionSensorAbstraction<'a, SMA_WINDOW> {
    /// Update the vision sensor and get an angle value back.
    ///
    /// # Errors
    /// - Will return an error if the vision sensor returns an error.
    /// - Will return an error if no objects of `object_id` are detected.
    fn update(&mut self) -> anyhow::Result<Angle> {
        let objects = self.sensor.objects()?;

        // find biggest color object
        let biggest = objects
            .iter()
            .filter_map(|o| match *o {
                AiVisionObject::Color {
                    id,
                    position,
                    width,
                    height,
                } => Some(Color {
                    id,
                    position,
                    width,
                    height,
                }),
                _ => None,
            })
            .filter(|c| c.id == self.object_id)
            .max_by_key(|c| c.width * c.height)
            .context("no objects detected")?;

        // calculate new error using sma
        let error = color_angle(&biggest).wrapped_half();
        self.sma.add_sample(error);

        Ok(self.sma.get_average())
    }

    /// Create a new instance.
    fn new(sensor: &'a AiVisionSensor, object_id: u8) -> Self {
        Self {
            sensor,
            object_id,
            sma: SingleSumSMA::from_zero(Angle::ZERO),
        }
    }
}

/// A controller that turns to face a vision sensor object.
pub struct VisionTrack<
    FA: Feedback<State = Angle, Signal = f64>,
    FL: Feedback<State = f64, Signal = f64>,
> {
    /// The angular controller used for turning.
    ///
    /// Takes the robot's approximate heading relative to the object.
    pub angular_controller: FA,

    /// The linear controller used for driving.
    ///
    /// Takes the robot's distance from the target.
    pub linear_controller: FL,

    /// Tolerances used to determine when the robot has finished driving.
    ///
    /// Takes the robot's distance from the target.
    pub linear_tolerances: Tolerances,
}

impl<
    FA: Feedback<State = Angle, Signal = f64> + Clone,
    FL: Feedback<State = f64, Signal = f64> + Clone,
> VisionTrack<FA, FL>
{
    /// Point to face a vision sensor object.
    ///
    /// `object_id` is the id of the color to track (colors are the only ones supported currently).
    ///
    /// # Errors
    ///
    /// Will return an error if no object is detected by the vision sensor (or it is disconnected).
    pub async fn turn_to_object<M: Arcade, T: Tracking>(
        &self,
        drivetrain: &mut Drivetrain<M, T>,
        vision_sensor: &AiVisionSensor,
        object_id: u8,
    ) {
        // state
        let mut controller = self.angular_controller.clone();
        let mut prev_time = Instant::now();
        let mut sensor = VisionSensorAbstraction::<3>::new(vision_sensor, object_id);

        // TODO: add settling to this
        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            let error = sensor
                .update()
                .inspect_err(|e| tracing::error!("vision sensor error: {e}"))
                .unwrap_or_default();

            // update controller
            let dt = prev_time.elapsed();
            prev_time = Instant::now();
            let steer = controller.update(error, Angle::ZERO, dt);

            // drive robot
            drop(drivetrain.model.drive_arcade(0., steer));
        }
    }

    /// Point to face a vision sensor object while driving some distance.
    ///
    /// `object_id` is the id of the color to track (colors are the only ones supported currently).
    ///
    /// # Errors
    ///
    /// Will return an error if no object is detected by the vision sensor (or it is disconnected).
    pub async fn drive_towards_object<M: Arcade, T: TracksForwardTravel + TracksVelocity>(
        &self,
        drivetrain: &mut Drivetrain<M, T>,
        vision_sensor: &AiVisionSensor,
        object_id: u8,
        distance: f64,
    ) where
        <M as DrivetrainModel>::Error: core::fmt::Debug,
    {
        // state
        let mut angular_controller = self.angular_controller.clone();
        let mut linear_controller = self.linear_controller.clone();
        let mut linear_tolerances = self.linear_tolerances;
        let mut prev_time = Instant::now();
        let mut sensor = VisionSensorAbstraction::<3>::new(vision_sensor, object_id);

        // initial state
        let target_travel = drivetrain.tracking.forward_travel() + distance;

        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            // get values
            let angular_error = sensor
                .update()
                .inspect_err(|e| tracing::error!("vision sensor error: {e}"))
                .unwrap_or_default();
            let dt = prev_time.elapsed();
            prev_time = Instant::now();

            // update angular controller
            let steer = angular_controller
                .update(angular_error, Angle::ZERO, dt)
                .clamp(-1., 1.);

            // update linear controller
            let throttle = linear_controller
                .update(drivetrain.tracking.forward_travel(), target_travel, dt)
                .clamp(-1., 1.)
                * angular_error.cos();

            // check tolerances
            let error = target_travel - drivetrain.tracking.forward_travel();
            if linear_tolerances.check(error, drivetrain.tracking.linear_velocity()) {
                tracing::info!("reached target");
                // stop robot
                drop(
                    drivetrain
                        .model
                        .drive_arcade(0., 0.)
                        .inspect_err(|e| tracing::error!("failed to stop robot: {:?}", e)),
                );
                break;
            }

            // drive robot
            drop(
                drivetrain
                    .model
                    .drive_arcade(throttle, steer)
                    .inspect_err(|e| tracing::error!("failed to drive robot: {:?}", e)),
            );
        }
    }

    /// Future will complete once the vision sensor detects no objects (with id `object_id`).
    ///
    /// `object_id` is the id of the color to track (colors are the only ones supported currently).
    ///
    /// # Errors
    ///
    /// The future will also complete if the vision sensor returns an error.
    pub async fn wait_none(&self, vision_sensor: &AiVisionSensor, object_id: u8) {
        let mut sensor = VisionSensorAbstraction::<3>::new(vision_sensor, object_id);

        while sensor.update().is_ok() {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;
        }
    }
}

/// Returns the angle (from the center of the screen) of a color object.
fn color_angle(object: &Color) -> Angle {
    // find the center of that object (horizontally)
    let center = f64::from(object.width) / 2. + f64::from(object.position.x);

    // the center of the screen
    let target = f64::from(AiVisionSensor::HORIZONTAL_RESOLUTION) / 2.;

    // the distance in pixels
    let distance = center - target;

    // convert to angle
    Angle::from_degrees(
        // negate the answer so that negative values mean the object is to the right and positive
        // values mean the object is to the left
        -(f64::from(AiVisionSensor::HORIZONTAL_FOV) * distance)
            / f64::from(AiVisionSensor::HORIZONTAL_RESOLUTION),
    )
}
