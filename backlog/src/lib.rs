//! Lock-free single-producer single-consumer ring for store-and-forward.
//!
//! Unlike a plain queue, items are not freed when they are read but when
//! they are acknowledged, so everything sent but not yet confirmed can be
//! sent again after a reconnect:
//!
//! ```text
//!   acked          sent             written
//!     |  sent,      |   not sent      |
//!     |  not acked  |   yet           |     free
//! ----+-------------+-----------------+--------------
//! ```
//!
//! Only `written` (owned by the producer) and `acked` (owned by the
//! consumer) are shared, as atomics; `sent` is private to the consumer.
//! The storage is borrowed, so it can live anywhere, e.g. in PSRAM, while
//! the atomics stay in internal RAM (atomics do not work in PSRAM on the
//! ESP32-S3).

#![no_std]

use core::marker::PhantomData;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU32, Ordering};

/// Positions run over `0..2 * capacity`, so that a full ring
/// (`written - acked == capacity`) differs from an empty one
/// (`written == acked`) without an extra flag, for any capacity.
pub struct Backlog<'a, T> {
    buf: NonNull<T>,
    capacity: u32,
    written: AtomicU32,
    acked: AtomicU32,
    _storage: PhantomData<&'a mut [T]>,
}

// SAFETY: the slots are only reached through `Producer` and `Consumer`,
// which `split` hands out once and whose positions never overlap; the
// shared positions are atomics.
unsafe impl<T: Send> Sync for Backlog<'_, T> {}
unsafe impl<T: Send> Send for Backlog<'_, T> {}

impl<'a, T: Copy> Backlog<'a, T> {
    /// # Panics
    ///
    /// If `storage` is empty or longer than `u32::MAX / 2`.
    pub fn new(storage: &'a mut [T]) -> Self {
        assert!(!storage.is_empty(), "empty storage");
        let capacity = u32::try_from(storage.len())
            .ok()
            .filter(|&c| c <= u32::MAX / 2)
            .expect("storage too large");
        Self {
            buf: NonNull::from(storage).cast(),
            capacity,
            written: AtomicU32::new(0),
            acked: AtomicU32::new(0),
            _storage: PhantomData,
        }
    }

    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    /// The `&mut` borrow guarantees a single producer and a single consumer
    /// for as long as the handles live.
    pub fn split(&mut self) -> (Producer<'_, 'a, T>, Consumer<'_, 'a, T>) {
        let ring = &*self;
        (Producer { ring }, Consumer { ring, sent: 0 })
    }

    /// Number of items from position `from` to position `to`.
    fn distance(&self, from: u32, to: u32) -> u32 {
        if to >= from {
            to - from
        } else {
            to + 2 * self.capacity - from
        }
    }

    /// Position `n` items after `pos`, `n <= capacity`.
    fn advance(&self, pos: u32, n: u32) -> u32 {
        let next = pos + n;
        if next >= 2 * self.capacity {
            next - 2 * self.capacity
        } else {
            next
        }
    }

    fn slot(&self, pos: u32) -> *mut T {
        let index = if pos >= self.capacity {
            pos - self.capacity
        } else {
            pos
        };
        // SAFETY: `index < capacity`, the length of the storage.
        unsafe { self.buf.as_ptr().add(index as usize) }
    }
}

pub struct Producer<'r, 'a, T> {
    ring: &'r Backlog<'a, T>,
}

impl<T: Copy> Producer<'_, '_, T> {
    /// Items written but not acknowledged yet, seen from the producer side,
    /// which keeps running even when the consumer is blocked.
    pub fn unacked(&self) -> u32 {
        let ring = self.ring;
        let written = ring.written.load(Ordering::Relaxed);
        let acked = ring.acked.load(Ordering::Acquire);
        ring.distance(acked, written)
    }

    /// Appends an item, or gives it back if the ring is full of
    /// unacknowledged items.
    pub fn push(&mut self, item: T) -> Result<(), T> {
        let ring = self.ring;
        // Only this producer stores `written`.
        let written = ring.written.load(Ordering::Relaxed);
        // Acquire: the consumer is done with every slot before `acked`.
        let acked = ring.acked.load(Ordering::Acquire);
        if ring.distance(acked, written) == ring.capacity {
            return Err(item);
        }
        // SAFETY: the slot is free (not between `acked` and `written`), so
        // the consumer does not touch it.
        unsafe { ring.slot(written).write(item) };
        // Release: publish the slot before the new position.
        ring.written
            .store(ring.advance(written, 1), Ordering::Release);
        Ok(())
    }
}

pub struct Consumer<'r, 'a, T> {
    ring: &'r Backlog<'a, T>,
    sent: u32,
}

impl<T: Copy> Consumer<'_, '_, T> {
    /// Items written but not sent yet.
    pub fn unsent(&self) -> u32 {
        let written = self.ring.written.load(Ordering::Acquire);
        self.ring.distance(self.sent, written)
    }

    /// Items written but not acknowledged yet: what a reconnect would cost
    /// if it were lost.
    pub fn unacked(&self) -> u32 {
        let written = self.ring.written.load(Ordering::Acquire);
        let acked = self.ring.acked.load(Ordering::Relaxed);
        self.ring.distance(acked, written)
    }

    /// The next item to send, without consuming it.
    pub fn peek(&self) -> Option<T> {
        // `unsent` loads `written` with Acquire: the slot is fully written.
        // SAFETY: the slot is between `sent` and `written`, which the
        // producer does not touch.
        (self.unsent() > 0).then(|| unsafe { self.ring.slot(self.sent).read() })
    }

    /// Marks the item returned by [`peek`](Self::peek) as sent.
    ///
    /// # Panics
    ///
    /// If there is nothing unsent.
    pub fn mark_sent(&mut self) {
        assert!(self.unsent() > 0, "nothing to mark as sent");
        self.sent = self.ring.advance(self.sent, 1);
    }

    /// Frees sent items from the oldest one for as long as `is_acked`
    /// holds. Returns the number of freed items.
    pub fn ack_while(&mut self, mut is_acked: impl FnMut(&T) -> bool) -> u32 {
        let ring = self.ring;
        let mut acked = ring.acked.load(Ordering::Relaxed);
        let mut freed = 0;
        while acked != self.sent {
            // SAFETY: the slot is between `acked` and `sent`, not reused by
            // the producer until `acked` moves past it.
            let item = unsafe { ring.slot(acked).read() };
            if !is_acked(&item) {
                break;
            }
            acked = ring.advance(acked, 1);
            freed += 1;
        }
        // Release: our reads of the freed slots happen before the producer
        // may overwrite them.
        ring.acked.store(acked, Ordering::Release);
        freed
    }

    /// Sends everything not acknowledged again, e.g. after a reconnect.
    pub fn rewind(&mut self) {
        self.sent = self.ring.acked.load(Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    fn drain_sent(consumer: &mut Consumer<'_, '_, u32>) -> Vec<u32> {
        let mut out = Vec::new();
        while let Some(item) = consumer.peek() {
            out.push(item);
            consumer.mark_sent();
        }
        out
    }

    #[test]
    fn sends_in_order() {
        let mut storage = [0u32; 4];
        let mut ring = Backlog::new(&mut storage);
        let (mut producer, mut consumer) = ring.split();

        assert_eq!(consumer.peek(), None);
        for i in 0..3 {
            producer.push(i).unwrap();
        }
        assert_eq!(consumer.unsent(), 3);
        assert_eq!(drain_sent(&mut consumer), [0, 1, 2]);
        assert_eq!(consumer.unsent(), 0);
        assert_eq!(consumer.unacked(), 3);
    }

    #[test]
    fn full_until_acked() {
        let mut storage = [0u32; 3];
        let mut ring = Backlog::new(&mut storage);
        let (mut producer, mut consumer) = ring.split();

        for i in 0..3 {
            producer.push(i).unwrap();
        }
        assert_eq!(producer.push(3), Err(3));

        // Sending alone frees nothing: the data may still be lost.
        drain_sent(&mut consumer);
        assert_eq!(producer.push(3), Err(3));

        assert_eq!(producer.unacked(), 3);
        assert_eq!(consumer.ack_while(|&x| x < 2), 2);
        assert_eq!(producer.unacked(), 1);
        producer.push(3).unwrap();
        producer.push(4).unwrap();
        assert_eq!(producer.push(5), Err(5));
        assert_eq!(drain_sent(&mut consumer), [3, 4]);
    }

    #[test]
    fn ack_stops_at_unsent() {
        let mut storage = [0u32; 4];
        let mut ring = Backlog::new(&mut storage);
        let (mut producer, mut consumer) = ring.split();

        for i in 0..4 {
            producer.push(i).unwrap();
        }
        consumer.mark_sent();
        consumer.mark_sent();
        // The receiver cannot confirm what was never sent.
        assert_eq!(consumer.ack_while(|_| true), 2);
        assert_eq!(consumer.unacked(), 2);
    }

    #[test]
    fn rewind_resends_unacked() {
        let mut storage = [0u32; 8];
        let mut ring = Backlog::new(&mut storage);
        let (mut producer, mut consumer) = ring.split();

        for i in 0..5 {
            producer.push(i).unwrap();
        }
        drain_sent(&mut consumer);
        consumer.ack_while(|&x| x < 2);

        consumer.rewind();
        assert_eq!(drain_sent(&mut consumer), [2, 3, 4]);
    }

    #[test]
    fn wraps_around_many_times() {
        let mut storage = [0u32; 3];
        let mut ring = Backlog::new(&mut storage);
        let (mut producer, mut consumer) = ring.split();

        let mut expected = 0;
        for i in 0..100 {
            producer.push(i).unwrap();
            if i % 2 == 1 {
                for item in drain_sent(&mut consumer) {
                    assert_eq!(item, expected);
                    expected += 1;
                }
                consumer.ack_while(|_| true);
            }
        }
        assert_eq!(expected, 100);
    }

    #[test]
    fn concurrent_producer_and_consumer() {
        // Miri interprets the code thousands of times slower.
        const ITEMS: u32 = if cfg!(miri) { 200 } else { 200_000 };
        let storage = Vec::leak(std::vec![0u32; if cfg!(miri) { 8 } else { 1000 }]);
        let ring = std::boxed::Box::leak(std::boxed::Box::new(Backlog::new(storage)));
        let (mut producer, mut consumer) = ring.split();

        std::thread::scope(|scope| {
            scope.spawn(move || {
                for i in 0..ITEMS {
                    while producer.push(i).is_err() {
                        std::thread::yield_now();
                    }
                }
            });

            let mut next = 0;
            while next < ITEMS {
                match consumer.peek() {
                    Some(item) => {
                        assert_eq!(item, next, "items arrive complete and in order");
                        consumer.mark_sent();
                        next += 1;
                        if next % 37 == 0 {
                            consumer.ack_while(|_| true);
                        }
                    }
                    None => std::thread::yield_now(),
                }
            }
        });
    }
}
