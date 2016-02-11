/*
 * include/linux/urwlock.h - Upgradeable read/write locks.
 *
 * Copyright (C) 2012 Con Kolivas <kernel@kolivas.org>
 *
 * These are upgradeable variants of read/write locks.
 *
 * When a lock is chosen, one of read, upgradeable or write lock needs to be
 * chosen. Much like read/write locks, a read lock cannot be upgraded to a
 * write lock. However the upgradeable version can be either upgraded to a
 * write lock, or downgraded to a read lock. Unlike read/write locks, these
 * locks favour writers over readers. They are significantly more overhead
 * than either spinlocks or read/write locks as they include one of each,
 * however they are suited to situations where there are clear distinctions
 * between read and write patterns, and where the state may be indeterminate
 * for a period, allowing other readers to continue reading till they need to
 * declare themselves as read or write.
 */

#ifndef __LINUX_URWLOCK_H
#define __LINUX_URWLOCK_H

#include <linux/spinlock.h>

struct urwlock {
	raw_spinlock_t lock;
	rwlock_t rwlock;
};

typedef struct urwlock urwlock_t;

static inline void urwlock_init(urwlock_t *urw)
{
	raw_spin_lock_init(&urw->lock);
	rwlock_init(&urw->rwlock);
}

/* Low level write and read lock/unlock of the rw lock. */
static inline void __urw_write_lock(rwlock_t *rw)
{
	rwlock_acquire(&rw.dep_map, 0, 0, _RET_IP_);
	LOCK_CONTENDED(rw, do_raw_write_trylock, do_raw_write_lock);
}

static inline void __urw_write_unlock(rwlock_t *rw)
{
	rwlock_release(&rw.dep_map, 1, _RET_IP_);
	do_raw_write_unlock(rw);
}

static inline void __urw_read_lock(rwlock_t *rw)
{
	rwlock_acquire_read(&rw.dep_map, 0, 0, _RET_IP_);
	LOCK_CONTENDED(rw, do_raw_read_trylock, do_raw_read_lock);
}

static inline void __urw_read_unlock(rwlock_t *rw)
{
	rwlock_release(&rw.dep_map, 1, _RET_IP_);
	do_raw_read_unlock(rw);
}

/* Write variant of urw lock. Grabs both spinlock and rwlock. */
static inline void urw_wlock(urwlock_t *urw)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
{
	raw_spin_lock(&urw->lock);
	__urw_write_lock(&urw->rwlock);
}

/* Write variant of urw unlock. Releases both spinlock and rwlock. */
static inline void urw_wunlock(urwlock_t *urw)
	__releases(urw->rwlock)
	__releases(urw->lock)
{
	__urw_write_unlock(&urw->rwlock);
	raw_spin_unlock(&urw->lock);
}

/*
 * Read variant of urw lock. Grabs spinlock and rwlock and then releases
 * spinlock.
 */
static inline void urw_rlock(urwlock_t *urw)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
	__releases(urw->lock)
{
	raw_spin_lock(&urw->lock);
	__urw_read_lock(&urw->rwlock);
	spin_release(&urw->lock.dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(&urw->lock);
}

/* Read variant of urw lock. Releases only the rwlock. */
static inline void urw_runlock(urwlock_t *urw)
	__releases(urw->rwlock)
{
	__urw_read_unlock(&urw->rwlock);
}

/* Upgradeable variant of urw lock. Grabs only the spinlock. */
static inline void urw_ulock(urwlock_t *urw)
	__acquires(urw->lock)
{
	raw_spin_lock(&urw->lock);
}

/* Upgrade the upgradeable variant of urwlock. Grabs the write lock. */
static inline void urw_upgrade(urwlock_t *urw)
{
	__urw_write_lock(&urw->rwlock);
}

/*
 * Downgrade the upgradeable variant of urwlock to a read lock. Grabs the
 * read rwlock and releases the spinlock.
 */
static inline void urw_udowngrade(urwlock_t *urw)
	__acquires(urw->rwlock)
	__releases(urw->lock)
{
	__urw_read_lock(&urw->rwlock);
	spin_release(&urw->lock.dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(&urw->lock);
}

/*
 * Downgrade the write variant of urwlock to a read lock. Drops the write
 * rwlock, grabs the read rwlock and releases the spinlock.
 */
static inline void urw_wdowngrade(urwlock_t *urw)
	__releases(urw->rwlock)
	__acquires(urw->rwlock)
	__releases(urw->lock)
{
	__urw_write_unlock(&urw->rwlock);
	__urw_read_lock(&urw->rwlock);
	spin_release(&urw->lock.dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(&urw->lock);
}

/*
 * Unlock the upgradeable variant of urwlock where it has not been up or
 * downgraded.
 */
static inline void urw_uunlock(urwlock_t *urw)
	__releases(urw->lock)
{
	raw_spin_unlock(&urw->lock);
}

/* IRQ variants of urw locks */
static inline void urw_wlock_irq(urwlock_t *urw)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
{
	raw_spin_lock_irq(&urw->lock);
	__urw_write_lock(&urw->rwlock);
}

static inline void urw_wunlock_irq(urwlock_t *urw)
	__releases(urw->rwlock)
	__releases(urw->lock)
{
	__urw_write_unlock(&urw->rwlock);
	raw_spin_unlock_irq(&urw->lock);
}

static inline void urw_rlock_irq(urwlock_t *urw)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
	__releases(urw->lock)
{
	raw_spin_lock_irq(&urw->lock);
	__urw_read_lock(&urw->rwlock);
	spin_release(&urw->lock.dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(&urw->lock);
}

static inline void urw_runlock_irq(urwlock_t *urw)
	__releases(urw->rwlock)
{
	read_unlock_irq(&urw->rwlock);
}

static inline void urw_ulock_irq(urwlock_t *urw)
	__acquires(urw->lock)
{
	raw_spin_lock_irq(&urw->lock);
}

static inline void urw_uunlock_irq(urwlock_t *urw)
	__releases(urw->lock)
{
	raw_spin_unlock_irq(&urw->lock);
}

static inline void urw_wlock_irqsave(urwlock_t *urw, unsigned long *flags)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
{
	raw_spin_lock_irqsave(&urw->lock, *flags);
	__urw_write_lock(&urw->rwlock);
}

static inline void urw_wunlock_irqrestore(urwlock_t *urw, unsigned long *flags)
	__releases(urw->rwlock)
	__releases(urw->lock)
{
	__urw_write_unlock(&urw->rwlock);
	raw_spin_unlock_irqrestore(&urw->lock, *flags);
}

static inline void urw_ulock_irqsave(urwlock_t *urw, unsigned long *flags)
	__acquires(urw->lock)
{
	raw_spin_lock_irqsave(&urw->lock, *flags);
}

static inline void urw_uunlock_irqrestore(urwlock_t *urw, unsigned long *flags)
	__releases(urw->lock)
{
	raw_spin_unlock_irqrestore(&urw->lock, *flags);
}

static inline void urw_rlock_irqsave(urwlock_t *urw, unsigned long *flags)
	__acquires(urw->lock)
	__acquires(urw->rwlock)
	__releases(urw->lock)
{
	raw_spin_lock_irqsave(&urw->lock, *flags);
	__urw_read_lock(&urw->rwlock);
	spin_release(&urw->lock.dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(&urw->lock);
}

static inline void urw_runlock_irqrestore(urwlock_t *urw, unsigned long *flags)
	__releases(urw->rwlock)
{
	read_unlock_irqrestore(&urw->rwlock, *flags);
}

#endif /* __LINUX_URWLOCK_H */
