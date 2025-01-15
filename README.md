# BLDC driver in Rust

Initially simple foc ported to rust. Except it is less powerful and only works with rp2040 micro controller due to lack of crossplatform timer.

I've now realized my mistake of making this crate much larger than it should be. For example, there was absolutely no reason to have the sensors in this crate when they are already implemented by others in rust community. As such, this crate should only handle the logics of controlling the motors not the hardware part.

Mostly written as a practice to rust and embedded programming.