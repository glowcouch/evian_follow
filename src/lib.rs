#![feature(iter_map_windows)]
use std::{
    pin::Pin,
    task::Poll,
    time::{Duration, Instant},
};

use evian::{
    control::loops::{AngularPid, Feedback, Pid},
    drivetrain::model::DrivetrainModel,
    math::{Angle, Vec2},
    motion::pursuit::Waypoint,
    prelude::{Arcade, Drivetrain, TracksHeading, TracksPosition},
};
use vexide::time::{Sleep, sleep};

#[derive(Debug, Clone)]
pub struct PathFollow {
    pub linear_controller: Pid,
    pub angular_controller: AngularPid,
    pub lookahead: f64,
}

impl PathFollow {
    pub fn follow<M: Arcade, I: IntoIterator<Item = Waypoint>, T: TracksPosition + TracksHeading>(
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
            waypoints,
            linear_controller,
            angular_controller,
            lookahead,
            state: None,
        }
    }
}

struct State {
    current: Waypoint,
    sleep: Sleep,
    prev_time: Instant,
}

struct PathFollowFuture<
    'a,
    I: Iterator<Item = Waypoint>,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading,
> {
    drivetrain: &'a mut Drivetrain<M, T>,
    waypoints: I,
    /// The total distance between all the waypoints that are left
    /// this must be updated whenever a waypoint is taken
    path_distance: f64,
    linear_controller: Pid,
    angular_controller: AngularPid,
    lookahead: f64,
    state: Option<State>,
}

impl<
    'a,
    I: Iterator<Item = Waypoint>,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading,
> PathFollowFuture<'a, I, M, T>
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
}

impl<
    'a,
    I: Iterator<Item = Waypoint> + Unpin,
    M: DrivetrainModel + Arcade,
    T: TracksPosition + TracksHeading,
> Future for PathFollowFuture<'a, I, M, T>
{
    type Output = ();

    fn poll(self: std::pin::Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        // add state
        if this.state.is_none() {
            let next_waypoint = match this.waypoints.next() {
                Some(w) => w,
                None => return Poll::Ready(()),
            };
            this.state = Some(State {
                current: next_waypoint,
                sleep: sleep(Duration::from_millis(5)),
                prev_time: Instant::now(),
            });
        }

        // we know this unwrap is ok because we just initialized state
        let state = this.state.as_mut().unwrap();

        // if the timer is not finished we go back to sleep
        // I have confirmed that calling poll on a `vexide::time::Sleep` will in fact register the
        // waker
        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        } else {
            // start a new timer
            state.sleep = sleep(Duration::from_millis(5));
        }

        // get tracking data
        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        // find the next point that we have not reached
        while state.current.position.distance(position) <= this.lookahead {
            // find next waypoint (or exit if we are finished)
            let new_waypoint = match this.waypoints.next() {
                Some(w) => w,
                None => return Poll::Ready(()),
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
        let linear_setpoint = this.distance_heuristic(position);
        let throttle = this.linear_controller.update(0., linear_setpoint, dt);

        // drive the robot
        drop(this.drivetrain.model.drive_arcade(throttle, steer));

        // wake ourselves once so that we can poll the timer (sleep)
        //
        // This doesn't cause a busy loop because we will return `Poll::Pending` once we poll the
        // timer. It just makes sure that the timer has registered our waker
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}
