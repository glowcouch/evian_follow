//! A controller that turns to face a vision sensor object.

use std::time::Instant;

use evian::{
    control::loops::Feedback,
    drivetrain::model::Arcade,
    math::Angle,
    prelude::{Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};
use simple_moving_average::SMA;
use simple_moving_average::SingleSumSMA;
use vexide::{
    math::Point2,
    prelude::AiVisionSensor,
    smart::{SmartDevice, ai_vision::AiVisionObject, vision},
    time::sleep,
};

/// Wrapper around [`AiVisionObject::Color`].
struct Color {
    #[allow(clippy::missing_docs_in_private_items)]
    id: u8,
    #[allow(clippy::missing_docs_in_private_items)]
    position: Point2<u16>,
    #[allow(clippy::missing_docs_in_private_items)]
    width: u16,
    #[allow(clippy::missing_docs_in_private_items)]
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
    fn update(&mut self) -> Angle {
        if let Ok(objects) = self.sensor.objects() {
            // find biggest color object
            if let Some(biggest) = objects
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
            {
                let error = color_angle(&biggest).wrapped_half();
                self.sma.add_sample(error);
            }
        }
        self.sma.get_average()
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
pub struct VisionTrack<F: Feedback<State = Angle, Signal = f64>> {
    /// The angular controller used for turning.
    ///
    /// Takes the robot's approximate heading relative to the object.
    pub controller: F,

    /// Tolerances used to determine when the robot has finished turning.
    ///
    /// Takes distance in pixels as input.
    //  It might be better to scale distance with the width of the object or something.
    pub tolerances: Tolerances,
}

impl<F: Feedback<State = Angle, Signal = f64> + Clone> VisionTrack<F> {
    /// Point to face a vision sensor object.
    ///
    /// `object_id` is the id of the color to track (colors are the only ones supported currently).
    pub async fn turn_to_object<M: Arcade, T: TracksPosition + TracksHeading + TracksVelocity>(
        &self,
        drivetrain: &mut Drivetrain<M, T>,
        vision_sensor: &AiVisionSensor,
        object_id: u8,
    ) {
        // state
        let mut controller = self.controller.clone();
        let mut prev_time = Instant::now();
        let mut sensor = VisionSensorAbstraction::<3>::new(vision_sensor, object_id);

        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            let error = sensor.update();

            // update controller
            let dt = prev_time.elapsed();
            prev_time = Instant::now();
            let steer = controller.update(error, Angle::ZERO, dt);

            // drive robot
            drop(drivetrain.model.drive_arcade(0., steer));
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
