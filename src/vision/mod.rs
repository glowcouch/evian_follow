//! A controller that turns to face a vision sensor object.

use std::time::Instant;

use anyhow::Context;
use evian::{
    control::loops::Feedback,
    drivetrain::model::{Arcade, DrivetrainModel},
    math::Angle,
    prelude::{Drivetrain, Tolerances, TracksForwardTravel, TracksHeading, TracksVelocity},
    tracking::Tracking,
};
use vexide::{
    math::Point2,
    prelude::AiVisionSensor,
    smart::{SmartDevice, ai_vision::AiVisionObject},
    time::sleep,
};

use crate::{
    util::ema::Ema,
    vision::{filters::VisionFilter, sorters::VisionSorter},
};

pub mod filters;
pub mod sorters;

/// Wrapper around [`AiVisionObject::Color`].
#[allow(clippy::missing_docs_in_private_items, reason = "wrapper")]
pub struct Color {
    id: u8,
    position: Point2<u16>,
    width: u16,
    height: u16,
}

/// Abstraction over vision sensor for better tracking.
pub struct VisionSensorAbstraction<'a, 'b, F: VisionFilter, S: VisionSorter, T: TracksHeading> {
    /// The drivetrain tracking.
    ///
    /// Used to calculate absolute heading.
    tracking: &'b T,

    /// The vision sensor
    sensor: &'a AiVisionSensor,

    /// The id of the color object we are tracking.
    object_id: u8,

    /// The smoothing filter used to smooth the absolute heading.
    smoothing: Ema,

    /// Filter used to filter out extra objects.
    filter: F,

    /// Sorter used to pick objects.
    sorter: S,
}

impl<'a, 'b, F: VisionFilter, S: VisionSorter, T: TracksHeading>
    VisionSensorAbstraction<'a, 'b, F, S, T>
{
    /// Update the vision sensor and get an absolute heading back.
    ///
    /// # Errors
    /// - Will return an error if the vision sensor returns an error.
    /// - Will return an error if no objects of `object_id` are detected.
    pub fn update(&mut self) -> anyhow::Result<Angle> {
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
            .filter(|c| self.filter.filter(c))
            .max_by(|a, b| self.sorter.cmp(a, b))
            .context("no objects detected")?;

        // calculate new error
        let error = color_angle(&biggest).wrapped_half();

        // the heading of the object (absolute)
        let heading = (self.tracking.heading() + error).wrapped_half();

        // Smooth the heading
        let smoothed_heading = Angle::from_radians(self.smoothing.next(heading.as_radians()));

        Ok(smoothed_heading)
    }

    /// Create a new instance.
    pub fn new(
        sensor: &'a AiVisionSensor,
        object_id: u8,
        filter: F,
        sorter: S,
        tracking: &'b T,
        ema_smootheness: f64,
    ) -> Self {
        Self {
            tracking,
            sensor,
            object_id,
            filter,
            sorter,
            smoothing: Ema::new(ema_smootheness),
        }
    }
}

/// A controller that turns to face a vision sensor object.
pub struct VisionTrack<
    FA: Feedback<State = Angle, Signal = f64>,
    FL: Feedback<State = f64, Signal = f64>,
    F: VisionFilter,
    S: VisionSorter,
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

    /// Tolerances used to determine when the robot has finished turning.
    ///
    /// Takes the robot's angular error in radians.
    pub angular_tolerances: Tolerances,

    /// Vision sensor filter.
    pub filter: F,

    /// Vision sensor sorter.
    pub sorter: S,

    /// Smoothing factor for the absolute heading.
    pub heading_smootheness: f64,
}

impl<
    FA: Feedback<State = Angle, Signal = f64>,
    FL: Feedback<State = f64, Signal = f64>,
    F: VisionFilter,
    S: VisionSorter,
> VisionTrack<FA, FL, F, S>
{
    /// Returns a new [`VisionTrack`] with the given filter,
    pub fn with_filter<F2: VisionFilter>(self, filter: F2) -> VisionTrack<FA, FL, F2, S> {
        VisionTrack {
            angular_controller: self.angular_controller,
            linear_controller: self.linear_controller,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            filter,
            sorter: self.sorter,
            heading_smootheness: self.heading_smootheness,
        }
    }

    /// Returns a new [`VisionTrack`] with the given sorter
    pub fn with_sorter<S2: VisionSorter>(self, sorter: S2) -> VisionTrack<FA, FL, F, S2> {
        VisionTrack {
            angular_controller: self.angular_controller,
            linear_controller: self.linear_controller,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            filter: self.filter,
            sorter,
            heading_smootheness: self.heading_smootheness,
        }
    }
}

impl<
    FA: Feedback<State = Angle, Signal = f64> + Clone,
    FL: Feedback<State = f64, Signal = f64> + Clone,
    F: VisionFilter + Clone,
    S: VisionSorter + Clone,
> VisionTrack<FA, FL, F, S>
{
    /// Point to face a vision sensor object.
    ///
    /// `object_id` is the id of the color to track (colors are the only ones supported currently).
    ///
    /// # Errors
    ///
    /// Will return an error if no object is detected by the vision sensor (or it is disconnected).
    pub async fn turn_to_object<M: Arcade, T: Tracking + TracksVelocity + TracksHeading>(
        &self,
        drivetrain: &mut Drivetrain<M, T>,
        vision_sensor: &AiVisionSensor,
        object_id: u8,
    ) {
        // state
        let mut controller = self.angular_controller.clone();
        let mut prev_time = Instant::now();
        let mut sensor = VisionSensorAbstraction::new(
            vision_sensor,
            object_id,
            self.filter.clone(),
            self.sorter.clone(),
            &drivetrain.tracking,
            self.heading_smootheness,
        );
        let mut angular_tolerances = self.angular_tolerances;

        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            let object_heading = sensor
                .update()
                .inspect_err(|e| tracing::error!("vision sensor error: {e}"))
                .unwrap_or_default();

            let heading = drivetrain.tracking.heading();

            let angular_error = object_heading - heading;

            // check if within tolerances
            let angular_velocity = drivetrain.tracking.angular_velocity();
            if angular_tolerances.check(angular_error.as_radians(), angular_velocity) {
                break;
            }

            // update controller
            let dt = prev_time.elapsed();
            prev_time = Instant::now();
            let steer = controller.update(heading, object_heading, dt);

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
    pub async fn drive_towards_object<
        M: Arcade,
        T: TracksForwardTravel + TracksVelocity + TracksHeading,
    >(
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
        let mut sensor = VisionSensorAbstraction::new(
            vision_sensor,
            object_id,
            self.filter.clone(),
            self.sorter.clone(),
            &drivetrain.tracking,
            self.heading_smootheness,
        );

        // initial state
        let target_travel = drivetrain.tracking.forward_travel() + distance;

        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            // get values
            let object_heading = sensor
                .update()
                .inspect_err(|e| tracing::error!("vision sensor error: {e}"))
                .unwrap_or_default();
            let dt = prev_time.elapsed();
            prev_time = Instant::now();

            let heading = drivetrain.tracking.heading();

            // find angular error
            let angular_error = object_heading - heading;

            // update angular controller
            let steer = angular_controller
                .update(heading, object_heading, dt)
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
    pub async fn wait_none<T: TracksHeading>(
        &self,
        vision_sensor: &AiVisionSensor,
        object_id: u8,
        // It's kind of silly to have this take a tracking implementation.
        tracking: &T,
    ) {
        let mut sensor = VisionSensorAbstraction::new(
            vision_sensor,
            object_id,
            self.filter.clone(),
            self.sorter.clone(),
            tracking,
            self.heading_smootheness,
        );

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
