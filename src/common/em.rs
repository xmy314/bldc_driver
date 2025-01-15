use core::f32::consts;
use micromath::F32;

/// electrical angle
#[derive(Clone, Copy)]
pub struct EAngle(pub f32);

/// mechanical angle
#[derive(Clone, Copy)]
pub struct MAngle(pub f32);

/// abc frame
/// or stator reference frame
/// or srf
#[derive(Clone, Copy)]
pub struct Vabc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// qd frame
/// or rotor reference frame
/// or rrf
#[derive(Clone, Copy)]
pub struct Vqd {
    pub q: f32,
    pub d: f32,
}
/// abc frame
/// or stator reference frame
/// or srf
#[derive(Clone, Copy)]
pub struct Iabc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// qd frame
/// or rotor reference frame
/// or rrf
#[derive(Clone, Copy)]
pub struct Iqd {
    pub q: f32,
    pub d: f32,
}

impl Vabc {
    /// From abc coordinate to qd coordinate
    pub fn parks_transformation(&self, rotor_angle_rads: EAngle) -> Vqd {
        let (sa, ca) = F32(rotor_angle_rads.0).sin_cos();
        let (sb, cb) = F32(rotor_angle_rads.0 - consts::TAU / 3f32).sin_cos();
        let (sc, cc) = F32(rotor_angle_rads.0 + consts::TAU / 3f32).sin_cos();
        Vqd {
            q: (2.0 / 3.0) * ca.0 * self.a
                + (2.0 / 3.0) * cb.0 * self.b
                + (2.0 / 3.0) * cc.0 * self.c,
            d: (2.0 / 3.0) * sa.0 * self.a
                + (2.0 / 3.0) * sb.0 * self.b
                + (2.0 / 3.0) * sc.0 * self.c,
        }
    }

    /// Clamp the voltage based on maximum line to line voltage
    pub fn limit(&self, v_limit: f32) -> Vabc {
        let (sa, ca) = (0.0, 1.0);
        let (sb, cb) = (-0.5, -0.866_025_4);
        let (sc, cc) = (-0.5, 0.866_025_4);
        let vx = (2.0 / 3.0) * (ca * self.a + cb * self.b + cc * self.c);
        let vy = (2.0 / 3.0) * (sa * self.a + sb * self.b + sc * self.c);
        let sqr_magnitude = vx * vx + vy * vy;
        let sqr_limit = v_limit * v_limit;

        if sqr_magnitude > sqr_limit {
            let s = F32(sqr_limit / sqr_magnitude).sqrt().0;
            Vabc {
                a: s * self.a,
                b: s * self.b,
                c: s * self.c,
            }
        } else {
            Vabc {
                a: self.a,
                b: self.b,
                c: self.c,
            }
        }
    }
}

impl Vqd {
    /// From qd coordinate to abc coordinate
    pub fn inverse_parks_transformation(&self, rotor_angle_rads: EAngle) -> Vabc {
        let (sa, ca) = F32(rotor_angle_rads.0).sin_cos();
        let (sb, cb) = F32(rotor_angle_rads.0 - consts::TAU / 3f32).sin_cos();
        let (sc, cc) = F32(rotor_angle_rads.0 + consts::TAU / 3f32).sin_cos();
        Vabc {
            a: ca.0 * self.q + sa.0 * self.d,
            b: cb.0 * self.q + sb.0 * self.d,
            c: cc.0 * self.q + sc.0 * self.d,
        }
    }

    /// Clamp the voltage based on maximum line to line voltage
    pub fn limit(&self, v_limit: f32) -> Vqd {
        let sqr_magnitude = self.d * self.d + self.q * self.q;
        let sqr_limit = v_limit * v_limit;
        if sqr_magnitude > sqr_limit {
            let s = F32(sqr_limit / sqr_magnitude).sqrt().0;
            Vqd {
                q: s * self.q,
                d: s * self.d,
            }
        } else {
            Vqd {
                q: self.q,
                d: self.d,
            }
        }
    }
}

impl Iabc {
    /// From abc coordinate to qd coordinate
    pub fn parks_transformation(&self, rotor_angle_rads: EAngle) -> Iqd {
        let (sa, ca) = F32(rotor_angle_rads.0).sin_cos();
        let (sb, cb) = F32(rotor_angle_rads.0 - consts::TAU / 3f32).sin_cos();
        let (sc, cc) = F32(rotor_angle_rads.0 + consts::TAU / 3f32).sin_cos();
        Iqd {
            q: (2.0 / 3.0) * ca.0 * self.a
                + (2.0 / 3.0) * cb.0 * self.b
                + (2.0 / 3.0) * cc.0 * self.c,
            d: (2.0 / 3.0) * sa.0 * self.a
                + (2.0 / 3.0) * sb.0 * self.b
                + (2.0 / 3.0) * sc.0 * self.c,
        }
    }

    /// Clamp the current based on maximum line current
    pub fn limit(&self, i_limit: f32) -> Iabc {
        let (sa, ca) = (0.0, 1.0);
        let (sb, cb) = (-0.5, -0.866_025_4);
        let (sc, cc) = (-0.5, 0.866_025_4);
        let ix = (2.0 / 3.0) * (ca * self.a + cb * self.b + cc * self.c);
        let iy = (2.0 / 3.0) * (sa * self.a + sb * self.b + sc * self.c);
        let sqr_magnitude = ix * ix + iy * iy;
        let sqr_limit = i_limit * i_limit;
        if sqr_magnitude > sqr_limit {
            let s = F32(sqr_magnitude / sqr_limit).sqrt().0;
            Iabc {
                a: s * self.a,
                b: s * self.b,
                c: s * self.c,
            }
        } else {
            Iabc {
                a: self.a,
                b: self.b,
                c: self.c,
            }
        }
    }
}

impl Iqd {
    /// From qd coordinate to abc coordinate
    pub fn inverse_parks_transformation(&self, rotor_angle_rads: EAngle) -> Iabc {
        let (sa, ca) = F32(rotor_angle_rads.0).sin_cos();
        let (sb, cb) = F32(rotor_angle_rads.0 - consts::TAU / 3f32).sin_cos();
        let (sc, cc) = F32(rotor_angle_rads.0 + consts::TAU / 3f32).sin_cos();
        Iabc {
            a: ca.0 * self.q + sa.0 * self.d,
            b: cb.0 * self.q + sb.0 * self.d,
            c: cc.0 * self.q + sc.0 * self.d,
        }
    }

    /// Clamp the current based on maximum line current
    pub fn limit(&self, i_limit: f32) -> Iqd {
        let sqr_magnitude = self.d * self.d + self.q * self.q;
        let sqr_limit = i_limit * i_limit;
        if sqr_magnitude > sqr_limit {
            let s = F32(sqr_magnitude / sqr_limit).sqrt().0;
            Iqd {
                q: s * self.q,
                d: s * self.d,
            }
        } else {
            Iqd {
                q: self.q,
                d: self.d,
            }
        }
    }
}
