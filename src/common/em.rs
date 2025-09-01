use defmt::*;
use micromath::F32;

/// electrical angle
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct EAngle(pub f32);

/// mechanical angle
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct MAngle(pub f32);

/// abc frame
/// or stator reference frame
/// or srf
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct Vabc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// qd frame
/// or rotor reference frame
/// or rrf
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct Vqd {
    pub q: f32,
    pub d: f32,
}

/// abc frame
/// or stator reference frame
/// or srf
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct Iabc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// qd frame
/// or rotor reference frame
/// or rrf
#[derive(Debug, PartialEq, Clone, Copy, Format)]
pub struct Iqd {
    pub q: f32,
    pub d: f32,
}

impl Vabc {
    /// From abc coordinate to qd coordinate
    pub fn parks_transformation(&self, rotor_angle_rads: f32) -> Vqd {
        let (F32(sa), F32(ca)) = F32(rotor_angle_rads).sin_cos();
        let alpha = self.a - 0.5 * (self.b + self.c);
        let beta = 0.866025403784 * (self.b - self.c);
        Vqd {
            q: 0.6666666666f32 * (ca * alpha + sa * beta),
            d: 0.6666666666f32 * (sa * alpha - ca * beta),
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
    pub fn inverse_parks_transformation(&self, rotor_angle_rads: f32) -> Vabc {
        let (F32(sa), F32(ca)) = F32(rotor_angle_rads).sin_cos();
        let alpha = ca * self.q + sa * self.d;
        let beta = sa * self.q - ca * self.d;
        Vabc {
            a: alpha,
            b: -0.5 * alpha + 0.866025403784 * beta,
            c: -0.5 * alpha - 0.866025403784 * beta,
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
    pub fn parks_transformation(&self, rotor_angle_rads: f32) -> Iqd {
        let (F32(sa), F32(ca)) = F32(rotor_angle_rads).sin_cos();
        let alpha = self.a - 0.5 * (self.b + self.c);
        let beta = 0.866025403784 * (self.b - self.c);
        Iqd {
            q: 0.6666666666f32 * (ca * alpha + sa * beta),
            d: 0.6666666666f32 * (sa * alpha - ca * beta),
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
    pub fn inverse_parks_transformation(&self, rotor_angle_rads: f32) -> Iabc {
        let (F32(sa), F32(ca)) = F32(rotor_angle_rads).sin_cos();
        let alpha = ca * self.q + sa * self.d;
        let beta = sa * self.q - ca * self.d;
        Iabc {
            a: alpha,
            b: -0.5 * alpha + 0.866025403784 * beta,
            c: -0.5 * alpha - 0.866025403784 * beta,
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
