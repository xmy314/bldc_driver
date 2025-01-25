use embassy_time::Instant;

pub struct PID {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub windup_deletion: bool,
    pub reversed: bool,

    pid_state: PIDState,
}

/// internal state of PID controller
enum PIDState {
    /// set point isn't given
    UNINITIATED,
    /// set point is given but it doesn't know the current state
    SET { sp: f32 },
    /// in normal operation
    NORM {
        sp: f32,
        p_error: f32,
        i_error: f32,
        d_error: f32, // only used for debugging
        time: embassy_time::Instant,
    },
}

/// PID controller
///
/// No auto tune included
impl PID {
    /// Constructor
    pub fn new(kp: f32, ki: f32, kd: f32, reversed: bool, windup_deletion: bool) -> PID {
        PID {
            kp,
            ki,
            kd,

            reversed,
            windup_deletion,

            pid_state: PIDState::UNINITIATED,
        }
    }

    /// Clear set point and reset accumulated states
    pub fn reset(&mut self) {
        self.pid_state = PIDState::UNINITIATED;
    }

    /// Set set point
    pub fn set(&mut self, sp: f32) {
        self.pid_state = PIDState::SET { sp: sp };
    }

    /// Inspect the error.
    pub fn inspect_p_error(&mut self) -> Option<f32> {
        match self.pid_state {
            PIDState::NORM {
                sp: _,
                p_error,
                i_error: _,
                d_error: _,
                time: _,
            } => Some(p_error),
            PIDState::SET { sp: _ } => None,
            PIDState::UNINITIATED => None,
        }
    }

    /// Step the pid loop,
    ///     update internal state, and
    ///     return a control value
    pub fn update_and_get_throttle(&mut self, value: f32) -> f32 {
        let now = Instant::now();
        let (new_state, throttle) = match self.pid_state {
            PIDState::NORM {
                sp,
                p_error,
                i_error,
                d_error: _,
                time,
            } => {
                let dt = (now - time).as_micros() as f32 / 1_000_000.0;
                let n_p_error = sp - value;

                let mut n_i_error = i_error + p_error * dt;
                if self.windup_deletion && n_i_error * p_error < 0.0 {
                    n_i_error = 0.0;
                }

                let n_d_error = (n_p_error - p_error) / dt;

                (
                    PIDState::NORM {
                        sp: sp,
                        p_error: n_p_error,
                        i_error: n_i_error,
                        d_error: n_d_error,
                        time: now,
                    },
                    self.kp * n_p_error + self.ki * n_i_error + self.kd * n_d_error,
                )
            }
            PIDState::SET { sp } => (
                PIDState::NORM {
                    sp,
                    p_error: 0.0,
                    i_error: 0.0,
                    d_error: 0.0,
                    time: now,
                },
                self.kp * (sp - value),
            ),
            PIDState::UNINITIATED => (PIDState::UNINITIATED, 0.0),
        };
        self.pid_state = new_state;
        if self.reversed {
            -throttle
        } else {
            throttle
        }
    }
}
