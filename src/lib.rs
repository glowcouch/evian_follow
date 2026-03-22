//! Alternative following algorithm to [`evian::motion::pursuit::PurePursuit`] that uses
//! closed-loop feedback (such as pid) instead of arc-based turning.
#![feature(iter_map_windows)]
use core::{iter::FusedIterator, pin::Pin, task::Poll, time::Duration};
use std::time::Instant;

use evian::{
    control::loops::{AngularPid, Feedback, Pid},
    drivetrain::model::DrivetrainModel,
    math::{Angle, Vec2},
    motion::pursuit::Waypoint,
    prelude::{Arcade, Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};
use vexide::time::{Sleep, sleep};

/// A path following controller that uses closed-loop feedback for turning and driving.
///
/// This is suitable for dense paths (curves) and sparse paths (waypoints).
#[derive(Debug, Clone)]
pub struct PathFollow {
    /// Linear feedback controller.
    ///
    /// This controls the robot's speed throughout the motion.
    pub linear_controller: Pid,

    /// Angular feedback controller.
    ///
    /// This controls the robot's heading throughout the motion.
    pub angular_controller: AngularPid,

    /// Lookahead radius.
    ///
    /// Points are considered "visited" when they are within this radius. A smaller radius will
    /// result in higher accuracy, but can cause more oscilation.
    pub lookahead: f64,

    /// Throttle angle.
    ///
    /// When the angular error is greater than this angle the robot will not move forwards. Within
    /// this angle, the throttle will be scaled linearly - max speed being when the error is
    /// closest to 0.
    pub throttle_angle: Angle,

    /// The tolerances used for settling.
    ///
    /// The error is the distance to the last point.
    /// The velocity is linear velocity.
    pub tolerances: Tolerances,
}

impl PathFollow {
    /// Follow a set of waypoints.
    ///
    /// Important note: the velocity of waypoints is ignored.
    pub fn follow<
        M: Arcade,
        I: IntoIterator<Item = Waypoint>,
        T: TracksPosition + TracksHeading + TracksVelocity,
    >(
        &self,
        drivetrain: &mut Drivetrain<M, T>,
        waypoints: I,
    ) -> impl Future
    where
        <I as IntoIterator>::IntoIter: Clone + Unpin,
    {
        let Self {
            linear_controller,
            angular_controller,
            lookahead,
            throttle_angle,
            tolerances,
        } = self.clone();
        let waypoints = waypoints.into_iter();
        PathFollowFuture {
            drivetrain,
            // calculate the distance between all waypoints
            path_distance: waypoints
                .clone()
                .map_windows(|window: &[Waypoint; 2]| {
                    window[0].position.distance(window[1].position)
                })
                .sum(),
            waypoints: waypoints.fuse(),
            linear_controller,
            angular_controller,
            lookahead,
            throttle_angle,
            tolerances,
            state: None,
        }
    }
}

/// State of [`PathFollowFuture`].
struct State {
    /// The current target waypoint.
    ///
    /// The robot will try to aim towards this position.
    current: Waypoint,
    /// The current sleep timer.
    ///
    /// The future uses this to sleep in between updates.
    sleep: Sleep,

    /// The previous time at which the controller was updated.
    ///
    /// This is used to calculate delta time.
    prev_time: Instant,
}

/// Future returned by [`PathFollow::follow`].
///
/// The iterator has to implement [`FusedIterator`] because we use it to check if the path is finished.
struct PathFollowFuture<
    'a,
    I: Iterator<Item = Waypoint> + FusedIterator,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading + TracksVelocity,
> {
    /// The drivetrain being moved.
    drivetrain: &'a mut Drivetrain<M, T>,

    /// The set of waypoints that the robot is following.
    ///
    /// A waypoint is consumed each time that the [`State::current`] waypoint is reached.
    waypoints: I,

    /// The total distance between all the waypoints that are left.
    ///
    /// This must be updated whenever a waypoint is taken.
    path_distance: f64,

    /// Linear feedback controller.
    ///
    /// This is copied directly from [`PathFollow`].
    linear_controller: Pid,

    /// Angular feedback controller.
    ///
    /// This is copied directly from [`PathFollow`].
    angular_controller: AngularPid,

    /// The lookahead radius.
    ///
    /// This is copied directly from [`PathFollow`].
    lookahead: f64,

    /// Throttle angle.
    ///
    /// This is copied directly from [`PathFollow`].
    throttle_angle: Angle,

    /// The tolerances used for settling.
    ///
    /// This is copied directly from [`PathFollow`].
    tolerances: Tolerances,

    /// The state of the motion.
    ///
    /// This stores information for use within the future.
    state: Option<State>,
}

impl<
    I: Iterator<Item = Waypoint> + FusedIterator,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading + TracksVelocity,
> PathFollowFuture<'_, I, M, T>
{
    /// returns a heuristic for the total distance left in the motion
    ///
    /// takes the current tracking position
    fn distance_heuristic(&self, position: Vec2<f64>) -> f64 {
        // get the distance to the current waypoint (default to 0 if we don't have a current
        // waypoint selected)
        let distance_to_current = self
            .state
            .as_ref()
            .map(|state| state.current.position.distance(position))
            .unwrap_or_default();
        distance_to_current + self.path_distance
    }

    /// returns the maximum throttle for a given angular error
    fn throttle_limit(&self, error: Angle) -> f64 {
        f64::max(
            0.,
            1. - (error.abs().as_radians() / self.throttle_angle.as_radians()),
        )
    }
}

impl<
    I: Iterator<Item = Waypoint> + Unpin + FusedIterator,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading + TracksVelocity,
> Future for PathFollowFuture<'_, I, M, T>
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        // get state or initialize it
        let state = if let Some(state) = this.state.as_mut() {
            state
        } else {
            // return `Poll::Ready` if there are no more waypoints
            let Some(next_waypoint) = this.waypoints.next() else {
                return Poll::Ready(());
            };

            // initialize state
            this.state.insert(State {
                current: next_waypoint,
                sleep: sleep(Duration::from_millis(5)),
                prev_time: Instant::now(),
            })
        };

        // if the timer is not finished we go back to sleep
        // I have confirmed that calling poll on a `vexide::time::Sleep` will in fact register the
        // waker
        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        // start a new timer
        state.sleep = sleep(Duration::from_millis(5));

        // get tracking data
        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        // find the next point that we have not reached
        while state.current.position.distance(position) <= this.lookahead {
            // find next waypoint (or check settling conditions if there are no more waypoints)
            let Some(new_waypoint) = this.waypoints.next() else {
                // if it is settled
                if this.tolerances.check(
                    position.distance(state.current.position),
                    this.drivetrain.tracking.linear_velocity(),
                ) {
                    // we are done!
                    return Poll::Ready(());
                }
                // if it is not settled, just exit the loop
                break;
            };
            // update the distance that is left on the path
            this.path_distance -= state.current.position.distance(new_waypoint.position);
            // update current waypoint
            state.current = new_waypoint;
        }

        // get delta time
        let dt = state.prev_time.elapsed();
        // update state
        state.prev_time = Instant::now();

        // update angular controller
        let angular_setpoint = Angle::from_radians((state.current.position - position).angle());
        let steer = this
            .angular_controller
            .update(heading, angular_setpoint, dt);

        // update linear controller
        let throttle_limit = this.throttle_limit(angular_setpoint - heading);
        let linear_setpoint = this.distance_heuristic(position);
        let throttle = this.linear_controller.update(0., linear_setpoint, dt);

        // drive the robot
        drop(
            this.drivetrain
                .model
                .drive_arcade(f64::max(throttle, throttle_limit), steer),
        );

        // wake ourselves once so that we can poll the timer (sleep)
        //
        // This doesn't cause a busy loop because we will return `Poll::Pending` once we poll the
        // timer. It just makes sure that the timer has registered our waker
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}
