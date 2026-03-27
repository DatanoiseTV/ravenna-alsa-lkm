/****************************************************************************
*
*  Module Name    : module_timer.c
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Baume Florian
*  Date           : 15/04/2016
*  Modified by    :
*  Date           :
*  Modification   :
*  Known problems : None
*
* Copyright(C) 2017 Merging Technologies
*
* RAVENNA/AES67 ALSA LKM is free software; you can redistribute it and / or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* RAVENNA/AES67 ALSA LKM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with RAVAENNA ALSA LKM ; if not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/version.h>

#include "module_main.h"
#include "module_timer.h"

#define MAX_CATCHUP_ITERATIONS 8

/*
 * base_period_ is used for the initial timer arm and for computing
 * validation bounds.  It is NOT used for periodic scheduling — the
 * PTP manager's next_wakeup drives that adaptively.
 *
 * Accessed from process context (set_base_period, update_base_period)
 * and kthread context (audio_work_fn fallback).  Use WRITE_ONCE /
 * READ_ONCE to prevent torn reads on 32-bit architectures.
 */
static uint64_t base_period_;
static uint64_t max_period_allowed;
static uint64_t min_period_allowed;

/*
 * stop_ controls the shutdown sequence.  Accessed from:
 *   - timer_callback (hardirq context, potentially different CPU)
 *   - audio_work_fn  (kthread context)
 *   - start/stop/kill_clock_timer (process context)
 * Must use atomic_t for SMP visibility guarantees.
 */
static atomic_t stop_ = ATOMIC_INIT(0);

static struct hrtimer my_hrtimer_;
static struct kthread_worker *audio_worker;
static struct kthread_work audio_work;

static int audio_cpu_affinity = -1;
module_param(audio_cpu_affinity, int, 0444);
MODULE_PARM_DESC(audio_cpu_affinity,
    "CPU core to pin audio thread to (-1 = auto)");

/*
 * audio_work_fn — runs in SCHED_FIFO kthread context.
 *
 * Calls t_clock_timer() which does PTP processing and triggers
 * AudioFrameTIC (the audio interrupt callbacks).  The returned
 * next_wakeup is an absolute monotonic nanosecond timestamp
 * (PTP.c:1030: ui64AbsoluteTime * NS_2_REF_UNIT).
 *
 * After processing (with catch-up if behind), re-arms the hrtimer
 * at the exact next_wakeup time for adaptive PTP-synchronized
 * scheduling.  This is the one-shot model: the hrtimer fired once
 * and returned HRTIMER_NORESTART, so it is inactive here and safe
 * to re-arm.
 */
static void audio_work_fn(struct kthread_work *work)
{
    uint64_t next_wakeup, now;
    int64_t period;
    int iterations = 0;

    if (atomic_read(&stop_))
        return;

    do {
        get_clock_time(&now);
        t_clock_timer(&next_wakeup, now);
    } while (!atomic_read(&stop_) && now >= next_wakeup &&
             ++iterations < MAX_CATCHUP_ITERATIONS);

    if (atomic_read(&stop_))
        return;

    /* Compute period for validation (next_wakeup is absolute) */
    period = (int64_t)(next_wakeup - now);

    if (period <= 0) {
        /*
         * Still behind after MAX_CATCHUP_ITERATIONS.
         * Refresh now (stale after catch-up loop) and schedule
         * base_period_ ahead to avoid spinning.
         */
        get_clock_time(&now);
        next_wakeup = now + READ_ONCE(base_period_);
    } else if ((uint64_t)period > 5000000000ULL) {
        /* Safety clamp: period > 5s — schedule 1s ahead */
        get_clock_time(&now);
        next_wakeup = now + 1000000000ULL;
    }
    /* Periods outside [min, max] but within 5s are allowed through —
     * the PTP manager may legitimately request non-standard periods
     * during lock acquisition or rate changes. */

#ifdef CONFIG_PREEMPT_RT
    hrtimer_start(&my_hrtimer_, ns_to_ktime(next_wakeup),
                  HRTIMER_MODE_ABS);
#else
    hrtimer_start(&my_hrtimer_, ns_to_ktime(next_wakeup),
                  HRTIMER_MODE_ABS_HARD);
#endif
}

/*
 * timer_callback — hrtimer callback, one-shot model.
 *
 * Runs in hardirq context (non-RT) or softirq-kthread context (RT).
 * Queues the audio work to the SCHED_FIFO kthread and does NOT
 * restart the timer.  The kthread re-arms the timer after processing
 * with the PTP manager's adaptive next_wakeup time.
 */
static enum hrtimer_restart timer_callback(struct hrtimer *timer)
{
    struct kthread_worker *worker;

    if (atomic_read(&stop_))
        return HRTIMER_NORESTART;

    worker = READ_ONCE(audio_worker);
    if (!worker)
        return HRTIMER_NORESTART;

    kthread_queue_work(worker, &audio_work);
    return HRTIMER_NORESTART;
}

int init_clock_timer(void)
{
    atomic_set(&stop_, 0);

    if (audio_cpu_affinity >= 0)
        audio_worker = kthread_create_worker_on_cpu(
            audio_cpu_affinity, 0, "ravenna-audio");
    else
        audio_worker = kthread_create_worker(0, "ravenna-audio");

    if (IS_ERR(audio_worker)) {
        printk(KERN_ERR "ravenna: failed to create audio worker thread\n");
        return PTR_ERR(audio_worker);
    }

    sched_set_fifo(audio_worker->task);
    kthread_init_work(&audio_work, audio_work_fn);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,15,0)
  #ifdef CONFIG_PREEMPT_RT
    hrtimer_setup(&my_hrtimer_, timer_callback,
                  CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  #else
    hrtimer_setup(&my_hrtimer_, timer_callback,
                  CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
  #endif
#else
  #ifdef CONFIG_PREEMPT_RT
    hrtimer_init(&my_hrtimer_, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  #else
    hrtimer_init(&my_hrtimer_, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
  #endif
    my_hrtimer_.function = timer_callback;
#endif

    set_base_period(1000000); /* 1ms default (AES67 48 frames @ 48kHz) */

    printk(KERN_INFO "ravenna: audio kthread created (cpu=%d)\n",
           audio_cpu_affinity);
    return 0;
}

/*
 * kill_clock_timer — called from module_exit only.
 * Permanently tears down the timer and kthread worker.
 *
 * Ordering: setting stop_ + smp_mb ensures visibility before
 * hrtimer_cancel, which synchronizes with any running timer_callback.
 * kthread_destroy_worker flushes any already-queued work before joining.
 */
void kill_clock_timer(void)
{
    atomic_set(&stop_, 1);
    smp_mb(); /* full barrier: stop_ must be visible before cancel/flush */
    hrtimer_cancel(&my_hrtimer_);
    if (audio_worker) {
        kthread_destroy_worker(audio_worker);
        WRITE_ONCE(audio_worker, NULL);
    }
}

/*
 * start_clock_timer — (re)starts the periodic timer.
 * Called from PTP.c StartAudioFrameTICTimer which always calls
 * StopAudioFrameTICTimer first, so stop_ will be 1.
 * We clear stop_ before arming the timer for the initial kick.
 * After this first firing, the kthread takes over adaptive scheduling.
 */
int start_clock_timer(void)
{
    uint64_t period = READ_ONCE(base_period_);
    ktime_t expiry;

    atomic_set(&stop_, 0);
    smp_mb(); /* full barrier: stop_ must be visible before timer fires */

    expiry = ktime_add(ktime_get(), ns_to_ktime(period));

#ifdef CONFIG_PREEMPT_RT
    hrtimer_start(&my_hrtimer_, expiry, HRTIMER_MODE_ABS);
#else
    hrtimer_start(&my_hrtimer_, expiry, HRTIMER_MODE_ABS_HARD);
#endif
    return 0;
}

/*
 * stop_clock_timer — stops the timer but keeps the kthread alive.
 * Called from StopAudioFrameTICTimer during normal operation
 * (sample rate changes, stream stop).  Timer can be restarted
 * via start_clock_timer.
 *
 * Guard kthread_flush_worker against NULL — kill_clock_timer may
 * have already destroyed the worker during module exit.
 */
void stop_clock_timer(void)
{
    atomic_set(&stop_, 1);
    smp_mb(); /* full barrier: stop_ must be visible before cancel/flush */
    hrtimer_cancel(&my_hrtimer_);
    if (audio_worker)
        kthread_flush_worker(audio_worker);
}

void get_clock_time(uint64_t *clock_time)
{
    *clock_time = (uint64_t)ktime_to_ns(ktime_get());
}

void set_base_period(uint64_t base_period)
{
    WRITE_ONCE(base_period_, base_period);
    WRITE_ONCE(min_period_allowed, base_period / 7);
    WRITE_ONCE(max_period_allowed, (base_period * 10) / 6);
    printk(KERN_INFO "ravenna: base period set to %llu ns\n",
           (unsigned long long)base_period);
}

void update_base_period(uint32_t tic_frame_size, uint32_t sample_rate)
{
    uint64_t period_ns;
    if (sample_rate == 0)
        return;
    period_ns = ((uint64_t)tic_frame_size * 1000000000ULL) / sample_rate;
    set_base_period(period_ns);
}
