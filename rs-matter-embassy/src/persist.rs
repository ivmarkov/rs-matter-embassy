//! Persistence: `EmbassyPersist` - an implementation of the `Persist` trait that uses the `sequential_storage::map` API

use core::cell::RefCell;
use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use log::info;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::persist::{KvBlobStore, MatterPersist};

use sequential_storage::cache::NoCache;
use sequential_storage::map::{SerializationError, Value};

use crate::error::to_persist_error;

pub type EmbassyPersist<'a, S, N> = MatterPersist<'a, EmbassyKvBlobStore<S>, N>;

/// We expect closures, but `sequential_storage::map` operates on `Value` instances
/// (which is less flexible).
///
/// Therefore, cheat and implement a `Value` that stores our closure.
/// Only used during serialization.
///
/// (For deserialization, we take advantage of zero-copy, and pass `&[u8]` as `Value`.)
struct StoreValue<F>(u16, RefCell<Option<F>>);

impl<'d, F> Value<'d> for StoreValue<F>
where
    F: FnOnce(&mut [u8]) -> Result<usize, Error>,
{
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let f = self.1.borrow_mut().take().unwrap();

        let len = f(buffer).map_err(|_| SerializationError::InvalidData)?; // TODO

        info!("Blob {}: stored {} bytes {:?}", self.0, len, &buffer[..len]);

        Ok(len)
    }

    fn deserialize_from(_buffer: &'d [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        // This value is never used for deserialization
        unreachable!()
    }
}

/// A `KvBlobStore`` implementation that uses the `sequential_storage::map` API
/// on top of NOR Flash.
pub struct EmbassyKvBlobStore<S> {
    flash: S,
    flash_range: Range<u32>,
    cache: NoCache,
}

impl<S> EmbassyKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    /// Create a new KV blob store instance.
    pub fn new(flash: S, flash_range: Range<u32>) -> Self {
        Self {
            flash,
            flash_range,
            cache: NoCache::new(),
        }
    }

    async fn load<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), Error>,
    {
        let data: Option<&[u8]> = sequential_storage::map::fetch_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
        )
        .await
        .map_err(to_persist_error)?;

        cb(data)?;

        info!(
            "Blob {key}: loaded {:?} bytes {data:?}",
            data.map(|data| data.len())
        );

        Ok(())
    }

    async fn store<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, Error>,
    {
        let value = StoreValue(key, RefCell::new(Some(cb)));

        sequential_storage::map::store_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
            &value,
        )
        .await
        .map_err(to_persist_error)?;

        Ok(())
    }

    async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        sequential_storage::map::remove_item(
            &mut self.flash,
            self.flash_range.clone(),
            &mut self.cache,
            buf,
            &key,
        )
        .await
        .map_err(to_persist_error)?;

        info!("Blob {key}: removed");

        Ok(())
    }
}

impl<S> KvBlobStore for EmbassyKvBlobStore<S>
where
    S: MultiwriteNorFlash,
{
    async fn load<F>(&mut self, key: u16, buf: &mut [u8], f: F) -> Result<(), Error>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), Error>,
    {
        EmbassyKvBlobStore::load(self, key, buf, f).await
    }

    async fn store<F>(&mut self, key: u16, buf: &mut [u8], f: F) -> Result<(), Error>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, Error>,
    {
        EmbassyKvBlobStore::store(self, key, buf, f).await
    }

    async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error> {
        EmbassyKvBlobStore::remove(self, key, buf).await
    }
}
