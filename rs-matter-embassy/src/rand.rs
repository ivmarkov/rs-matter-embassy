/// `rand` function for the esp chips family.
#[cfg(feature = "esp")]
pub mod esp {
    use core::cell::RefCell;

    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

    // ... To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to store the `esp_hal::Rng` in a global variable
    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<esp_hal::rng::Rng>>> =
        Mutex::new(RefCell::new(None));

    /// Initialize the esp-specific `rand` implementation
    /// Need to do this only once
    pub fn esp_init_rand(rng: esp_hal::rng::Rng) {
        RAND.lock(|r| *r.borrow_mut() = Some(rng));
    }

    /// Generate random bytes using the esp-specific `rand` implementation
    pub fn esp_rand(buf: &mut [u8]) {
        RAND.lock(|rng| {
            let mut rng = rng.borrow_mut();

            buf.iter_mut()
                .for_each(|byte| *byte = rng.as_mut().unwrap().random() as _);
        })
    }
}

#[cfg(feature = "rp")]
pub mod rp {
    use embassy_rp::clocks::RoscRng;

    pub fn rp_rand(buf: &mut [u8]) {
        use rand_core::RngCore;

        let mut rng = RoscRng;
        rng.fill_bytes(buf);
    }
}
