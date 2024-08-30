use rp2040_hal::Timer;

pub struct PID<'a> {
    pub timer: &'a Timer,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub sp: f32,
    is_new: bool,
    prior_time: rp2040_hal::fugit::Instant<u64, 1, 1000000>,
    prior_error: f32,
    sum: f32,
}
impl<'a> PID<'a> {
    // constructor
    pub fn new(timer: &'a Timer, kp: f32, ki: f32, kd: f32, sp: f32) -> PID {
        PID {
            timer,
            kp,
            ki,
            kd,
            sp,

            is_new: true,
            prior_time: timer.get_counter(),
            prior_error: 0.0,
            sum: 0.0,
        }
    }

    // set a set point
    pub fn set(&mut self, sp: f32) {
        self.reset();
        self.sp = sp;
    }

    // takes in a reading and give out a value.
    pub fn update_and_get_throttle(&mut self, value: f32) -> f32 {
        let now = self.timer.get_counter();
        let dt = (now - self.prior_time).to_micros() as f32 / 1_000_000.0;
        let error = self.sp - value;

        self.sum += error * dt;
        let derror = if !self.is_new {
            (error - self.prior_error) / dt
        } else {
            0.0
        };
        self.is_new = false;
        self.prior_error = error;
        self.prior_time = now;
        self.kp * (error) + self.ki * (self.sum) + self.kd * (derror)
    }

    // reset accumulated states
    pub fn reset(&mut self) {
        self.is_new = true;
        self.prior_error = 0.0;
        self.sum = 0.0;
    }
}
