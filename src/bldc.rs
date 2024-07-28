use defmt::Format;
use embedded_hal::pwm;

/// motor physical parameter description.
#[derive(Debug, PartialEq)]
pub struct BLDCMotor {
    // number of electrical cycles per mechanical cycle
    pub pole_pairs: u8,
    // the following are useful for more sophisticated control
    pub kv: u16,
    pub phase_resistance: f32,
}

// Select a number that is multiple of 6 so the pwm channels are can be equally spaced far apart. 360 is chosen.
// 65535 (u16 max value) is fully on and 0 (u16 min value) is fully off.
const SIN_LUT: [u16; 90] = [
    32768, 33339, 33911, 34482, 35053, 35623, 36193, 36761, 37328, 37893, 38458, 39020, 39580,
    40139, 40695, 41248, 41799, 42348, 42893, 43436, 43975, 44510, 45042, 45571, 46095, 46616,
    47132, 47644, 48151, 48653, 49151, 49644, 50132, 50614, 51091, 51562, 52028, 52487, 52941,
    53389, 53830, 54265, 54693, 55115, 55530, 55938, 56338, 56732, 57118, 57497, 57869, 58233,
    58589, 58937, 59277, 59609, 59933, 60249, 60556, 60855, 61145, 61427, 61699, 61964, 62219,
    62465, 62702, 62930, 63149, 63359, 63559, 63750, 63931, 64103, 64266, 64418, 64562, 64695,
    64819, 64933, 65037, 65132, 65216, 65291, 65355, 65410, 65455, 65490, 65515, 65530,
];

fn pwm_from_elec_phase(elec_phase: u16) -> u16 {
    let sanitized_phase = elec_phase % 360;
    match sanitized_phase {
        0 => 32768,
        90 => 65535,
        180 => 32768,
        270 => 0,
        _ => {
            if sanitized_phase < 90 {
                SIN_LUT[usize::from(sanitized_phase)]
            } else if sanitized_phase < 180 {
                SIN_LUT[usize::from(180 - sanitized_phase)]
            } else if sanitized_phase < 270 {
                65535 - SIN_LUT[usize::from(sanitized_phase - 180)]
            } else {
                65535 - SIN_LUT[usize::from(360 - sanitized_phase)]
            }
        }
    }
}

pub trait BLDCDriver {
    // elec_phase is a number between [0,240).
    fn set_phase(&mut self, elec_phase: u16) -> ();
}

#[derive(Debug)]
pub struct BLDCDriver3PWM<A: pwm::SetDutyCycle, B: pwm::SetDutyCycle, C: pwm::SetDutyCycle> {
    pub motor_specification: BLDCMotor,
    pub a: A,
    pub b: B,
    pub c: C,
}

impl<A: pwm::SetDutyCycle, B: pwm::SetDutyCycle, C: pwm::SetDutyCycle> BLDCDriver
    for BLDCDriver3PWM<A, B, C>
{
    fn set_phase(&mut self, elec_phase: u16) -> () {
        self.a
            .set_duty_cycle_fraction(pwm_from_elec_phase(elec_phase), 65535)
            .unwrap();
        self.b
            .set_duty_cycle_fraction(pwm_from_elec_phase((elec_phase + 120) % 360), 65535)
            .unwrap();
        self.c
            .set_duty_cycle_fraction(pwm_from_elec_phase((elec_phase + 240) % 360), 65535)
            .unwrap();
    }
}
