//! A controller that turns to face a vision sensor object.

use std::time::Instant;

use evian::{
    control::loops::{Feedback, Pid},
    drivetrain::model::Arcade,
    prelude::{Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};
use vexide::{
    math::Point2,
    prelude::AiVisionSensor,
    smart::{SmartDevice, ai_vision::AiVisionObject},
    time::sleep,
};

/// A controller that turns to face a vision sensor object.
pub struct VisionTrack {
    /// The angular controller used for turning.
    ///
    /// Takes the object's distance from the center of the vision sensor as input.
    pub controller: Pid,

    /// Tolerances used to determine when the robot has finished turning.
    ///
    /// Takes distance in pixels as input.
    //  It might be better to scale distance with the width of the object or something.
    pub tolerances: Tolerances,
}

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

impl VisionTrack {
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
        let mut controller = self.controller;
        let mut prev_time = Instant::now();

        loop {
            sleep(AiVisionSensor::UPDATE_INTERVAL).await;

            if let Ok(objects) = vision_sensor.objects() {
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
                    .filter(|c| c.id == object_id)
                    .max_by_key(|c| c.width * c.height)
                {
                    // find the center of that object (horizontally)
                    let center = f64::from(biggest.width) / 2. + f64::from(biggest.position.x);

                    // where we want the object to be
                    let target = f64::from(AiVisionSensor::HORIZONTAL_RESOLUTION) / 2.;

                    // update controller
                    let dt = prev_time.elapsed();
                    prev_time = Instant::now();
                    let steer = controller.update(center, target, dt);

                    // drive robot
                    drop(drivetrain.model.drive_arcade(0., steer));
                }
            }
        }
    }
}
