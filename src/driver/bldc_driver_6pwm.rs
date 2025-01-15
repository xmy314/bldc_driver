#![allow(dead_code)]
use crate::common::em;
use crate::driver::BLDCDriver;

use embedded_hal::pwm;

#[derive(Debug)]
/// Modify the "physical" field voltage in rotor reference frame,
///
/// Some assumptiont that need to be take care of and depend on
/// both the pwm generator and the circuitary leading to the gates.
/// 1. The physical transistor, the gates, close the circuit when the pwm signal is high.
///     Else the dead time would operate in the wrong way and blow up the gates.
/// 2. Pairs of PWM channels have the same period and phase.
///     Else they will short and blow up the gate.
/// 3. Phase correct mode is on.
///     aka: every other pwm cycle goes is reversed in time
///     Else the generated dead time only work half of the time and blow up the gate.
/// 4. The low side pwms are set to inverted mode.
///     eg: when duty cycle is 30%, voltage output is high 70% of the time
///     Else the high side and low side are on at the same time and it blows up.
/// 5. This is really appliable to small motors due to the pwm nature of voltage control and inductance of motors coils.
///     At high current and inductance, switching would cause the voltage to spike.
///     If the said spike is higher than transistor peak voltage, it blows up the gates.
///
/// e.g.:
/// if pwm wraps have a period of 10 clock cycles,
/// dead time of 10% of pwm cycle per switch,
/// and the desired duty cycle is 30%.
///
/// Then, high side would be set to 25% duty cycle which is 30%(desired duty cycle) - 5%(half dead time)
/// Then, low side would be set to 35% duty cycle which is 30%(desired duty cycle) + 5%(half dead time)
/// And these are the duty cycles sent to the pwm channels.
///
/// Then, the assumption #4 above would flip the low side pwm to give and actual duty cycle of 65%.
///
/// Check the source code to see the timing diagram for the above example.
//
// The following are expected timing diagrams with
//     every character other than the vertical bars means one clock cycle,
//     and vertical bars representing start and end of pwm period.
//
// high side pwm
// |━━┓.......|.......┏━━|  5/20 (25%)
// |..┗━━━━━━━|━━━━━━━┛..| 15/20
// low side pwm
// |...┏━━━━━━|━━━━━━┓...| 13/20 (65%)
// |━━━┛......|......┗━━━|  7/20 (35%)
pub struct BLDCDriver6PWM<
    AH: pwm::SetDutyCycle,
    AL: pwm::SetDutyCycle,
    BH: pwm::SetDutyCycle,
    BL: pwm::SetDutyCycle,
    CH: pwm::SetDutyCycle,
    CL: pwm::SetDutyCycle,
> {
    /// supply voltage
    pub vdc: f32,
    /// Ratio out of 65535 of the cycle that is the dead time.
    /// For example, for a 0.1% dead time per switch would be 6.55 cycles.
    /// Which need to be rounded up to 7.
    /// Which correspond to a half dead time of 3.5.
    /// Which need to be rounded up to 4.
    pub half_deadtime: u16,
    pub ah: AH,
    pub al: AL,
    pub bh: BH,
    pub bl: BL,
    pub ch: CH,
    pub cl: CL,
}

impl<
        AH: pwm::SetDutyCycle,
        AL: pwm::SetDutyCycle,
        BH: pwm::SetDutyCycle,
        BL: pwm::SetDutyCycle,
        CH: pwm::SetDutyCycle,
        CL: pwm::SetDutyCycle,
    > BLDCDriver6PWM<AH, AL, BH, BL, CH, CL>
{
    /// Warning, this is not safe, use the safe wrapers instead.
    fn set_srf_voltage_unsafe(&mut self, v_srf: em::Vabc) {
        let minimum_v = if v_srf.a > v_srf.b {
            if v_srf.b > v_srf.c {
                // a>b>c
                v_srf.c
            } else {
                // a>b and c>b
                v_srf.b
            }
        } else if v_srf.a > v_srf.c {
            // b>a>c
            v_srf.c
        } else {
            // b>a and c>a
            v_srf.a
        };

        // This squezes out an extra 15.47% voltage by using the fact
        // the three phases are balanced and 120 degrees apart.
        let duty_a = ((v_srf.a - minimum_v) / self.vdc * 65535.0) as u16;
        let duty_b = ((v_srf.b - minimum_v) / self.vdc * 65535.0) as u16;
        let duty_c = ((v_srf.c - minimum_v) / self.vdc * 65535.0) as u16;

        let (duty_ah, duty_al) = if duty_a < self.half_deadtime {
            (0, duty_a + self.half_deadtime)
        } else if duty_a > 65535 - self.half_deadtime {
            (duty_a - self.half_deadtime, 0)
        } else {
            (duty_a - self.half_deadtime, duty_a + self.half_deadtime)
            // the second is + because they are assumed to be inverted
        };
        let (duty_bh, duty_bl) = if duty_b < self.half_deadtime {
            (0, duty_b + self.half_deadtime)
        } else if duty_b > 65535 - self.half_deadtime {
            (duty_b - self.half_deadtime, 0)
        } else {
            (duty_b - self.half_deadtime, duty_b + self.half_deadtime)
            // the second is + because they are assumed to be inverted
        };
        let (duty_ch, duty_cl) = if duty_c < self.half_deadtime {
            (0, duty_c + self.half_deadtime)
        } else if duty_c > 65535 - self.half_deadtime {
            (duty_c - self.half_deadtime, 0)
        } else {
            (duty_c - self.half_deadtime, duty_c + self.half_deadtime)
            // the second is + because they are assumed to be inverted
        };

        self.ah.set_duty_cycle_fraction(duty_ah, 65535).unwrap();
        self.bh.set_duty_cycle_fraction(duty_bh, 65535).unwrap();
        self.ch.set_duty_cycle_fraction(duty_ch, 65535).unwrap();
        self.al.set_duty_cycle_fraction(duty_al, 65535).unwrap();
        self.bl.set_duty_cycle_fraction(duty_bl, 65535).unwrap();
        self.cl.set_duty_cycle_fraction(duty_cl, 65535).unwrap();
    }

    fn disconnect_all(&mut self) {
        self.ah.set_duty_cycle_fully_off().unwrap();
        self.bh.set_duty_cycle_fully_off().unwrap();
        self.ch.set_duty_cycle_fully_off().unwrap();
        self.al.set_duty_cycle_fully_on().unwrap();
        self.bl.set_duty_cycle_fully_on().unwrap();
        self.cl.set_duty_cycle_fully_on().unwrap();
    }
}

impl<
        AH: pwm::SetDutyCycle,
        AL: pwm::SetDutyCycle,
        BH: pwm::SetDutyCycle,
        BL: pwm::SetDutyCycle,
        CH: pwm::SetDutyCycle,
        CL: pwm::SetDutyCycle,
    > BLDCDriver for BLDCDriver6PWM<AH, AL, BH, BL, CH, CL>
{
    fn get_voltage_limit(&self) -> f32 {
        // 1.5 for 3 phases.
        // 1.5 = 1*cos(0)-2*0.5*cos( 2*pi/3 )
        self.vdc / 1.5
    }

    fn set_srf_voltage(&mut self, v_srf: em::Vabc) {
        let v_srf_limited = v_srf.limit(self.get_voltage_limit());

        self.set_srf_voltage_unsafe(v_srf_limited);
    }

    fn set_rrf_voltage(&mut self, v_rrf: em::Vqd, rotor_angle_rads: em::EAngle) {
        let v_srf_limited = v_rrf
            .limit(self.get_voltage_limit())
            .inverse_parks_transformation(rotor_angle_rads);

        self.set_srf_voltage_unsafe(v_srf_limited);
    }

    /// connect all phases to gnd
    fn off(&mut self) {
        self.ah.set_duty_cycle_fully_off().unwrap();
        self.bh.set_duty_cycle_fully_off().unwrap();
        self.ch.set_duty_cycle_fully_off().unwrap();
        self.al.set_duty_cycle_fully_off().unwrap();
        self.bl.set_duty_cycle_fully_off().unwrap();
        self.cl.set_duty_cycle_fully_off().unwrap();
    }
}
