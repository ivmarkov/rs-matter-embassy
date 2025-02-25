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
    // TODO: Not cryptographically secure?
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

    // TODO: Not cryptographically secure?
    pub fn rp_rand(buf: &mut [u8]) {
        use rand_core::RngCore;

        let mut rng = RoscRng;
        rng.fill_bytes(buf);
    }
}


#[cfg(feature = "nrf")]
pub mod nrf {
    // use embassy_nrf::clocks::RoscRng;


    use core::cell::RefCell;

    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

    // use embassy_nrf::rng::Rng;
    use embassy_nrf::{bind_interrupts, peripherals, rng};

    bind_interrupts!(struct Irqs {
        RNG => rng::InterruptHandler<peripherals::RNG>;
    });

    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<
        embassy_nrf::rng::Rng<'_, peripherals::RNG>
        // peripherals::RNG
    >>> =
        Mutex::new(RefCell::new(None));

    /// Initialize the nrf-specific `rand` implementation
    /// Need to do this only once
    pub fn nrf_init_rand(rng: peripherals::RNG) {
        RAND.lock(|r| *r.borrow_mut() = Some(
            // embassy_nrf::rng::Rng::new(rng.as_mut().unwrap(), Irqs)
            embassy_nrf::rng::Rng::new(rng, Irqs)
        ));
    }

    // TODO: Not cryptographically secure?
    pub fn nrf_rand(buf: &mut [u8]) {
        // use rand_core::RngCore;

        // let mut rng = RoscRng;
        // let mut p_rng = embassy_nrf::rng::Rng;
        // let mut p_rng =  rand::Rng;

        // let mut rng = Rng::new(rng.borrow_mut();, Irqs);
        // let mut rng = Rng::new(p.RNG, Irqs);

        RAND.lock(|rng| {
            // let mut rng = embassy_nrf::rng::Rng::new(p_rng, Irqs);
            // let mut p_rng = p_rng.borrow_mut();
            // let mut rng = embassy_nrf::rng::Rng::new(p_rng.as_mut().unwrap(), Irqs);
            let mut rng = rng.borrow_mut();

            // buf.iter_mut()
            // .for_each(|byte| *byte = 
            rng.as_mut().unwrap().blocking_fill_bytes(buf);
            //  as _);
            // rng.fill_bytes(buf);
        })
    }
}