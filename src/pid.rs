use embassy_time::Instant;

pub struct PID {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub windup_deletion: bool,
    pub reversed: bool,
    pub output_saturation: Option<f32>,

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

            output_saturation: None,

            pid_state: PIDState::UNINITIATED,
        }
    }

    /// Clear set point and reset accumulated states
    pub fn reset(&mut self) {
        self.pid_state = PIDState::UNINITIATED;
    }

    /// Set set point
    pub fn set(&mut self, sp: f32) {
        self.pid_state = match self.pid_state {
            PIDState::NORM {
                sp: _,
                p_error,
                i_error,
                d_error,
                time,
            } => PIDState::NORM {
                sp,
                p_error,
                i_error,
                d_error,
                time,
            },
            PIDState::SET { sp: _ } | PIDState::UNINITIATED => PIDState::SET { sp: sp },
        }
    }

    pub fn set_with_clear(&mut self, sp: f32) {
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

    pub fn inspect_i_error(&mut self) -> Option<f32> {
        match self.pid_state {
            PIDState::NORM {
                sp: _,
                p_error: _,
                i_error,
                d_error: _,
                time: _,
            } => Some(i_error),
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
                let error = sp - value;

                let n_p_error = self.kp * error;

                let mut n_i_error = i_error + self.ki * (error * dt);

                if self.windup_deletion && n_i_error * n_p_error < 0.0 {
                    n_i_error = 0.0;
                }

                if let Some(max_output) = self.output_saturation {
                    if n_i_error > max_output {
                        n_i_error = max_output;
                    } else if n_i_error < -max_output {
                        n_i_error = -max_output;
                    }
                }

                let n_d_error = self.kd * (error - p_error / self.kp) / dt;

                let mut throttle = n_p_error + n_i_error + n_d_error;
                throttle = if let Some(max_output) = self.output_saturation {
                    if throttle > max_output {
                        max_output
                    } else if throttle < -max_output {
                        -max_output
                    } else {
                        throttle
                    }
                } else {
                    throttle
                };

                (
                    PIDState::NORM {
                        sp: sp,
                        p_error: n_p_error,
                        i_error: n_i_error,
                        d_error: n_d_error,
                        time: now,
                    },
                    throttle,
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
