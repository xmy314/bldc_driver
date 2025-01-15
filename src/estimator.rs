// Numerically unstable estimators cannot handle arbitrary amount of data.

/// Check how similiar two normal distributions are.
/// The output range is [0,inf).
pub fn symmetric_divergence(mean1: f32, mean2: f32, var1: f32, var2: f32) -> f32 {
    let d = (mean1 - mean2) * (mean1 - mean2);
    (var1 + d) / (2.0 * var2) + (var2 + d) / (2.0 * var1) - 1.0
}

/// Linear regression between two data
/// numerically unstable, one pass, un-biased estimator
/// finds slope and intercept in a data set
/// linear regression, the formula and explanation can be found here
/// <https://en.wikipedia.org/wiki/Simple_linear_regression#Normality_assumption>
pub struct LinearEstimator {
    n: u32,
    x: f32,
    xx: f32,
    y: f32,
    yy: f32,
    xy: f32,
}

impl LinearEstimator {
    pub fn new() -> LinearEstimator {
        LinearEstimator {
            n: 0,
            x: 0.0,
            xx: 0.0,
            y: 0.0,
            yy: 0.0,
            xy: 0.0,
        }
    }

    pub fn add(&mut self, x: f32, y: f32) {
        self.n += 1;
        self.x += x;
        self.xx += x * x;
        self.y += y;
        self.yy += y * y;
        self.xy += x * y;
    }

    pub fn get_n(&self) -> u32 {
        self.n
    }

    pub fn get_m(&self) -> Option<f32> {
        match self.n {
            0 => None,
            _ => Some(
                (self.n as f32 * self.xy - self.x * self.y)
                    / (self.n as f32 * self.xx - self.x * self.x),
            ),
        }
    }

    pub fn get_m_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 | 2 => None,
            _ => {
                let a = self.get_k().unwrap();
                let b = self.get_m().unwrap();
                let var_b = (self.yy - 2.0 * a * self.y - 2.0 * b * self.xy
                    + 2.0 * self.n as f32 * a * a
                    + 2.0 * a * b * self.x
                    + b * b * self.xx)
                    / ((self.n as f32) * (self.xx - self.x * self.x / self.n as f32));
                let corrected_var_b = var_b * self.n as f32 / (self.n as f32 - 2.0);

                Some(corrected_var_b)
            }
        }
    }

    pub fn get_k(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some(
                (self.xx * self.y - self.x * self.xy) / (self.n as f32 * self.xx - self.x * self.x),
            ),
        }
    }

    pub fn get_k_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 | 2 => None,
            _ => Some(self.get_m_var().unwrap() * self.xx / self.n as f32),
        }
    }
}

/// Linear Regression between two data where latest data have exponentially more weight
/// numerically unstable, one pass, biased estimator
/// finds slope and intercept in a data set
pub struct EMLinearEstimator {
    alpha: f32,
    n: u32,
    w: f32,
    ww: f32,
    x: f32,
    xx: f32,
    y: f32,
    yy: f32,
    xy: f32,
}

impl EMLinearEstimator {
    pub fn new(alpha: f32) -> EMLinearEstimator {
        EMLinearEstimator {
            alpha,
            n: 0,
            w: 0.0,
            ww: 0.0,
            x: 0.0,
            xx: 0.0,
            y: 0.0,
            yy: 0.0,
            xy: 0.0,
        }
    }

    pub fn add(&mut self, x: f32, y: f32) {
        self.w = (1.0 - self.alpha) * self.w + self.alpha;
        self.ww = (1.0 - self.alpha) * self.ww + self.alpha * self.alpha;
        self.x = (1.0 - self.alpha) * self.x + self.alpha * x;
        self.xx = (1.0 - self.alpha) * self.xx + self.alpha * x * x;
        self.y = (1.0 - self.alpha) * self.y + self.alpha * y;
        self.yy = (1.0 - self.alpha) * self.yy + self.alpha * y * y;
        self.xy = (1.0 - self.alpha) * self.xy + self.alpha * x * y;
        if self.n < 3 {
            self.n += 1;
        };
    }

    pub fn get_m(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some((self.w * self.xy - self.x * self.y) / (self.w * self.xx - self.x * self.x)),
        }
    }
    pub fn get_m_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 | 2 => None,
            _ => {
                let a: f32 = self.get_k().unwrap();
                let b = self.get_m().unwrap();
                let var_b = (self.yy - 2.0 * a * self.y - 2.0 * b * self.xy
                    + a * a * self.w
                    + 2.0 * a * b * self.x
                    + b * b * self.xx)
                    / (self.xx - self.x * self.x / self.w);
                Some(var_b)
            }
        }
    }

    pub fn get_k(&self) -> Option<f32> {
        match self.n {
            0 => None,
            _ => Some((self.xx * self.y - self.x * self.xy) / (self.w * self.xx - self.x * self.x)),
        }
    }

    pub fn get_k_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 | 2 => None,
            _ => Some(self.get_m_var().unwrap() * self.xx / self.w),
        }
    }

    pub fn correlation_xx(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some((self.xx - self.x * self.x) / (self.w)),
        }
    }
    pub fn correlation_yy(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some((self.yy - self.y * self.y) / (self.w)),
        }
    }
    pub fn correlation_xy(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some((self.xy - self.x * self.y) / (self.w)),
        }
    }

    pub fn get_square_pearson_correlation(&self) -> Option<f32> {
        match self.n {
            0 | 1 | 2 => None,
            _ => Some(
                self.correlation_xy().unwrap() * self.correlation_xy().unwrap()
                    / (self.correlation_xx().unwrap() * self.correlation_yy().unwrap()),
            ),
        }
    }

    pub fn get_stablized_y(&self, xp: f32) -> Option<f32> {
        // there exists a xy pair such that y=m*x+k
        // and that y=mn*x+kn for updated mn and kn with new data point (xp,yp)
        match self.n {
            0 | 1 => None,
            _ => Some(
                self.get_m().unwrap() * (self.xx - self.x * xp) / (self.x - self.w * xp)
                    + self.get_k().unwrap(),
            ),
        }
    }
}

/// EMLinearEstimator with X reduced to monotonically increasing whole number
/// such that it can be used to check whether Y is plateaued
pub struct ReducedEMLinearEstimator {
    // wrapper of EMLinearEstimator
    // where x is successive whole numbers starting from 0.
    n: f32,
    internal: EMLinearEstimator,
}

impl ReducedEMLinearEstimator {
    pub fn new(alpha: f32) -> ReducedEMLinearEstimator {
        ReducedEMLinearEstimator {
            n: 0.0,
            internal: EMLinearEstimator::new(alpha),
        }
    }

    pub fn get_n(&self) -> u32 {
        self.n as u32
    }

    pub fn add(&mut self, y: f32) {
        self.internal.add(self.n, y);
        self.n += 1.0;
    }

    pub fn get_m(&self) -> Option<f32> {
        self.internal.get_m()
    }

    pub fn get_m_var(&self) -> Option<f32> {
        self.internal.get_m_var()
    }

    pub fn get_k(&self) -> Option<f32> {
        self.internal.get_k()
    }

    pub fn get_k_var(&self) -> Option<f32> {
        self.internal.get_k_var()
    }

    pub fn get_square_pearson_correlation(&self) -> Option<f32> {
        self.internal.get_square_pearson_correlation()
    }

    pub fn get_stablized_y(&self) -> Option<f32> {
        self.internal.get_stablized_y(self.n + 1.0)
    }
}

/// Moving Average
/// numerically stable, one pass, biased, estimator
/// reference: Incremental calculation of weighted mean and variance
/// <https://fanf2.user.srcf.net/hermes/doc/antiforgery/stats.pdf>
pub struct AEstimator {
    n: u32,
    mean: f32,
    var_n: f32,
}

impl AEstimator {
    pub fn new() -> AEstimator {
        AEstimator {
            n: 0,
            mean: 0.0,
            var_n: 0.0,
        }
    }

    pub fn add(&mut self, y: f32) {
        let n = self.n + 1;
        let mean = self.mean + 1.0 / (n as f32) * (y - self.mean);
        let var_n = self.var_n + (y - mean) * (y - self.mean);

        self.n = n;
        self.mean = mean;
        self.var_n = var_n;
    }

    pub fn get_mean(&self) -> Option<f32> {
        match self.n {
            0 => None,
            _ => Some(self.mean),
        }
    }

    pub fn get_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some(self.var_n / self.n as f32),
        }
    }
}

/// Moving average where latest data have exponentially more weight
/// numerically stable, one pass, biased, estimator
/// reference: Incremental calculation of weighted mean and variance
/// <https://fanf2.user.srcf.net/hermes/doc/antiforgery/stats.pdf>
pub struct EMAEstimator {
    alpha: f32, // close to 0
    n: u32,
    w: f32, // approaches 1 as sample count increases
    mean: f32,
    var_w: f32,
}

impl EMAEstimator {
    pub fn new(alpha: f32) -> EMAEstimator {
        EMAEstimator {
            alpha, // close to 0
            n: 0,
            w: 0.0, // approaches 1 with more sampels
            mean: 0.0,
            var_w: 0.0,
        }
    }

    pub fn add(&mut self, y: f32) {
        let w = self.w * (1.0 - self.alpha) + self.alpha;
        let mean = match self.n {
            0 => y,                                            // init
            _ => self.mean + self.alpha / w * (y - self.mean), // recurrent relation
        };
        let var_w: f32 =
            (1.0 - self.alpha) * self.var_w + self.alpha * (y - mean) * (y - self.mean);

        if self.n < 2 {
            // after 2, the behaviour don't change.
            self.n += 1;
        }
        self.w = w;
        self.mean = mean;
        self.var_w = var_w;
    }

    pub fn get_mean(&self) -> Option<f32> {
        match self.n {
            0 => None,
            _ => Some(self.mean),
        }
    }

    pub fn get_var(&self) -> Option<f32> {
        match self.n {
            0 | 1 => None,
            _ => Some(self.var_w / self.w),
        }
    }
}
