/**
 * Memory bandwidth controller for multi-core systems
 *
 * Copyright (C) 2013  Heechul Yun <heechul@illinois.edu>
 *           (C) 2022  Heechul Yun <heechul.yun@ku.edu>
 *
 * This file is distributed under GPL v2 License. 
 * See LICENSE.TXT for details.
 *
 */

/**************************************************************************
 * Conditional Compilation Options
 **************************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DEBUG(x)
#define DEBUG_RECLAIM(x) x
#define DEBUG_USER(x)
#define DEBUG_PROFILE(x) x

/**************************************************************************
 * Included Files
 **************************************************************************/
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/smp.h> /* IPI calls */
#include <linux/irq_work.h>
#include <linux/hardirq.h>
#include <linux/perf_event.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/interrupt.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
#  include <uapi/linux/sched/types.h>
#elif LINUX_VERSION_CODE > KERNEL_VERSION(4, 13, 0)
#  include <linux/sched/types.h>
#elif LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0)
#  include <linux/sched/rt.h>
#endif
#include <linux/sched.h>

/**************************************************************************
 * Public Definitions
 **************************************************************************/
#define CACHE_LINE_SIZE 64
#define BUF_SIZE 256
#define PREDICTOR 1  /* 0 - used, 1 - ewma(a=1/2), 2 - ewma(a=1/4) */

#define DEFAULT_RD_BUDGET_MB 200000
#define DEFAULT_WR_BUDGET_MB 200000

#define DEFALUT_TOR_THRESHOLD 5000
#define DEFAULT_WPQ_LAT_THRESHOLD 3000

#define DEFAULT_RELAX_WRITE_MB 100000
#define DEFAULT_THROTTLE_WRITE_MB 256

#define DEFAULT_QMIN_MB       500

#if defined(__aarch64__) || defined(__arm__)
#  define PMU_LLC_MISS_COUNTER_ID 0x17   // LINE_REFILL
#  define PMU_LLC_WB_COUNTER_ID   0x18   // LINE_WB
#elif defined(__x86_64__) || defined(__i386__)
#  define PMU_LLC_MISS_COUNTER_ID 0x821 // OFFCORE_REQUESTS.ALL_DATA_RD
#  define PMU_LLC_WB_COUNTER_ID   0x40b0 // OFFCORE_REQUESTS.WB
#  define STREAMING_STORES        0x12a   // event=0x2a | (umask=0x1 << 8)
#  define STREAMING_STORES_OFFCORE 0x10800 // offcore_rsp mask -> config1
#  define PMU_TOR_INS_COUNTER_ID  0xc86fff00000135ULL /* unc_cha_tor_inserts.ia_wcil (perf alias config) */
#  define PMU_TOR_OCC_COUNTER_ID  0xc86fff00000136ULL /* unc_cha_tor_occupancy.ia_wcil (perf alias config) */
#  define PMU_TOR_PMU_TYPE_DEFAULT 28      /* from /sys/devices/uncore_cha_0/type */

#  define PMU_WPQ_OCC_PC0_ID 0x082   /* event=0x82, umask=0x00 */
#  define PMU_WPQ_OCC_PC1_ID 0x083   /* event=0x83, umask=0x00 */
#  define PMU_WPQ_INS_PC0_ID 0x120  /* event=0x20, umask=0x01 */
#  define PMU_WPQ_INS_PC1_ID 0x220  /* event=0x20, umask=0x02 */
/* Default HBM PMU index range (matches /sys/devices/uncore_hbm_{8..15}/type). */
#  define PMU_WPQ_PMU_START 159
#  define PMU_WPQ_PMU_END   174

#elif defined(__riscv)
// Note: These performance counters are specific to the T-Head C910.
// 		 They may not function correctly on other RISC-V designs.
#  define PMU_LLC_MISS_COUNTER_ID 0x11   // LL_CACHE_READ_MISS
#  define PMU_LLC_WB_COUNTER_ID   0x13   // LL_CACHE_WRITE_MISS
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0) // somewhere between 4.4-4.10
#  define TM_NS(x) (x)
#else
#  define TM_NS(x) (x).tv64
#endif

/**************************************************************************
 * Public Types
 **************************************************************************/
struct memstat{
	u64 used_read_budget;    /* used read budget */
	u64 used_write_budget;	 /* used write budget */
	u64 assigned_read_budget;  /* assigned read budget */
	u64 assigned_write_budget; /* assigned write budget */
	u64 throttled_time_ns;   
	int throttled;           /* throttled period count */
	u64 throttled_error;     /* throttled & error */
	int throttled_error_dist[10]; /* pct distribution */
	int exclusive;           /* exclusive period count */
	u64 exclusive_ns;        /* exclusive mode real-time duration */
	u64 exclusive_bw;        /* exclusive mode used bandwidth */
};

/* percpu info */
struct core_info {
	/* user configurations */
	int read_limit;          /* read limit mode */
	int write_limit;	 /* write limit mode */

	/* for control logic */
	int read_budget;         /* assigned read budget */
	int write_budget;        /* assigned write budged */

	int cur_read_budget;     /* currently available read budget */
	int cur_write_budget;    /* currently available write budget */

	struct task_struct * throttled_task;
	
	ktime_t throttled_time;  /* absolute time when throttled */

	u64 old_read_val;        /* hold previous read counter value */
	u64 old_write_val;	 /* hold previous write counter value */
	int prev_read_throttle_error;  /* check whether there was throttle error in 
				    the previous period for the read counter */
    	int prev_write_throttle_error; /* check whether there was throttle error in 
				    the previous period for the write counter */

	int exclusive_mode;      /* 1 - if in exclusive (best-effort) mode */
	ktime_t exclusive_time;  /* time when exclusive mode begins */

	struct irq_work	read_pending;  /* delayed work for NMIs */
	struct perf_event *read_event; /* PMC: LLC misses */

	struct irq_work write_pending;   /* delayed work for NMIs */
	struct perf_event *write_event;  /* PMC: LLC writebacks */

	struct perf_event *tor_ins_event;  /* TOR inserts counter */
	struct perf_event *tor_occ_event;  /* TOR occupancy counter */
	struct perf_event *wpq_occ_event;  /* WPQ occupancy counter */
	struct perf_event *wpq_ins_event;  /* WPQ inserts counter */
    
	struct task_struct *throttle_thread;  /* forced throttle idle thread */
	wait_queue_head_t throttle_evt; /* throttle wait queue */

	/* statistics */
	struct memstat overall;  /* stat for overall periods. reset by user */
	int read_used[3];        /* EWMA memory load */
	int write_used[3];	 /* EWMA memory load */
	int64_t period_cnt;      /* active periods count */

	int rtcore;              /* never throttle an rt core */
	/* per-core hr timer */
	struct hrtimer hr_timer;
};

/* global info */
struct memguard_info {
	ktime_t period_in_ktime;
	cpumask_var_t throttle_mask;
	cpumask_var_t active_mask;
};


/**************************************************************************
 * Global Variables
 **************************************************************************/
static struct memguard_info memguard_info;
static struct core_info __percpu *core_info;

static int g_period_us = 1000;

static int g_read_budget_mb = DEFAULT_RD_BUDGET_MB;
static int g_write_budget_mb = DEFAULT_WR_BUDGET_MB;
static int g_relax_write_budget_mb = DEFAULT_RELAX_WRITE_MB;
static int g_throttle_write_budget_mb = DEFAULT_THROTTLE_WRITE_MB;
static int g_tor_lat_threshold = DEFALUT_TOR_THRESHOLD;
static int g_wpq_lat_threshold = DEFAULT_WPQ_LAT_THRESHOLD;
static int g_latency_hot = 0; /* 1 when TOR/WPQ latency above threshold */
static int g_use_wpq = 0;
static u64 g_dynamic_write_limit_events = (u64)-1; /* (u64)-1 => use user limits */

static int g_use_reclaim = 0;   /* 1 - enable reclaim of "guaranteed" bw */
static int g_use_exclusive = 0; /* 2 - spare bw sharing (rtas'13) 
				   5 - propotional sharing (tc'15) */
static int g_qmin = INT_MAX;
static int g_read_counter_id = PMU_LLC_MISS_COUNTER_ID;
/*
 * Default write counter uses streaming stores on Intel SPR (event=0x2a, umask=0x1)
 * with OFFCORE_RSP mask in config1 (0x10800). For other architectures,
 * fall back to PMU_LLC_WB_COUNTER_ID.
 */
#if defined(__x86_64__) || defined(__i386__)
static int g_write_counter_id = STREAMING_STORES;
static u64 g_tor_ins_counter_id = PMU_TOR_INS_COUNTER_ID; /*  raw TOR inserts */
static u64 g_tor_occ_counter_id = PMU_TOR_OCC_COUNTER_ID; /*  raw TOR occupancy */
static int g_tor_pmu_type = PMU_TOR_PMU_TYPE_DEFAULT;
static u64 g_wpq_occ_pc0_counter_id = PMU_WPQ_OCC_PC0_ID;
static u64 g_wpq_occ_pc1_counter_id = PMU_WPQ_OCC_PC1_ID;
static u64 g_wpq_ins_pc0_counter_id = PMU_WPQ_INS_PC0_ID;
static u64 g_wpq_ins_pc1_counter_id = PMU_WPQ_INS_PC1_ID;
#else
static int g_write_counter_id = PMU_LLC_WB_COUNTER_ID; 
static u64 g_tor_ins_counter_id = 0;
static u64 g_tor_occ_counter_id = 0;
static int g_tor_pmu_type = -1;
static u64 g_wpq_occ_pc0_counter_id = 0;
static u64 g_wpq_ins_pc0_counter_id = 0;
static u64 g_wpq_occ_pc1_counter_id = 0;
static u64 g_wpq_ins_pc1_counter_id = 0;
#endif

#if defined(__x86_64__) || defined(__i386__)
/* WPQ PMU type range (defaults map to uncore_hbm_8..15) */
static int g_wpq_pmu_type = PMU_WPQ_PMU_START;
static int g_wpq_pmu_type_last = PMU_WPQ_PMU_END;
#else
static int g_wpq_pmu_type = -1;
static int g_wpq_pmu_type_last = -1;
#endif

static struct dentry *memguard_dir;

/* offcore_rsp / config1 support */
static u64 g_read_config1 = 0;
#if defined(__x86_64__) || defined(__i386__)
static u64 g_write_config1 = STREAMING_STORES_OFFCORE;
#else
static u64 g_write_config1 = 0;
#endif

static u64 __percpu *raw_read_prev;
static u64 __percpu *raw_write_prev;

/* support multiple CHA counters: assume contiguous pmu_type range */
#define MAX_TOR_CHA 64
static u64 tor_ins_prev[MAX_TOR_CHA];
static struct perf_event *tor_ins_event[MAX_TOR_CHA];
static int tor_ins_count;
static u64 tor_occ_prev[MAX_TOR_CHA];
static struct perf_event *tor_occ_event[MAX_TOR_CHA];
static int tor_occ_count;

/* WPQ counters (HBM) */
#define MAX_WPQ_HBM 16
static u64 wpq_ins_pc0_prev[MAX_WPQ_HBM];
static u64 wpq_ins_pc1_prev[MAX_WPQ_HBM];
static u64 wpq_occ_pc0_prev[MAX_WPQ_HBM];
static u64 wpq_occ_pc1_prev[MAX_WPQ_HBM];
static struct perf_event *wpq_ins_pc0_event[MAX_WPQ_HBM];
static struct perf_event *wpq_ins_pc1_event[MAX_WPQ_HBM];
static struct perf_event *wpq_occ_pc0_event[MAX_WPQ_HBM];
static struct perf_event *wpq_occ_pc1_event[MAX_WPQ_HBM];
static int wpq_ins_count;
static int wpq_occ_count;

/**************************************************************************
 * External Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Local Function Prototypes
 **************************************************************************/
static void period_timer_callback_slave(struct core_info *cinfo);
enum hrtimer_restart period_timer_callback_master(struct hrtimer *timer);
static void memguard_read_process_overflow(struct irq_work *entry);
static void memguard_write_process_overflow(struct irq_work *entry);    
static int throttle_thread(void *arg);
static void memguard_on_each_cpu_mask(const struct cpumask *mask,
				      smp_call_func_t func,
				      void *info, bool wait);

/**************************************************************************
 * Module parameters
 **************************************************************************/

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 10, 0)
module_param(g_read_counter_id, hexint,  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
module_param(g_write_counter_id, hexint,  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
#else
module_param(g_read_counter_id, int,  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
module_param(g_write_counter_id, int,  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
#endif

module_param(g_read_config1, ullong, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_read_config1, "raw config1 (e.g., offcore_rsp mask) for read counter");
module_param(g_write_config1, ullong, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_write_config1, "raw config1 (e.g., offcore_rsp mask) for write counter");

module_param(g_period_us, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_period_us, "throttling period in usec");

module_param(g_read_budget_mb, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_read_budget_mb, "default read budget in MB/s");
module_param(g_write_budget_mb, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_write_budget_mb, "default write budget in MB/s");
module_param(g_relax_write_budget_mb, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_relax_write_budget_mb, "write budget (MB/s) to use when latency is below thresholds");
module_param(g_throttle_write_budget_mb, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_throttle_write_budget_mb, "write budget (MB/s) to use when latency is above thresholds");	
module_param(g_tor_lat_threshold, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_tor_lat_threshold, "TOR latency threshold (occupancy/inserts) before throttling");
module_param(g_wpq_lat_threshold, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_wpq_lat_threshold, "WPQ latency threshold (occupancy/inserts) before throttling");
module_param(g_tor_ins_counter_id, ullong, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_tor_ins_counter_id, "raw config for CHA TOR inserts (unc_cha_tor_inserts.ia_wcil)");
module_param(g_tor_occ_counter_id, ullong, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_tor_occ_counter_id, "raw config for CHA TOR occupancy (unc_cha_tor_occupancy.ia_wcil)");
module_param(g_tor_pmu_type, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_tor_pmu_type, "PMU type id for CHA TOR counters (e.g., /sys/devices/uncore_cha_0/type)");
module_param(g_use_wpq, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_use_wpq, "Enable WPQ counters/latency gating (0=off, 1=on)");

/* if last < first, autoprobes upward until failures */
static int g_tor_pmu_type_last = 67;
module_param(g_tor_pmu_type_last, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_tor_pmu_type_last, "Last PMU type id (inclusive) for CHA TOR inserts; -1 to auto-probe contiguous types");
module_param(g_wpq_pmu_type, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_wpq_pmu_type, "PMU type id for uncore_hbm_*/WPQ counters (e.g., /sys/devices/uncore_hbm_0/type)");
module_param(g_wpq_pmu_type_last, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(g_wpq_pmu_type_last, "Last PMU type id (inclusive) for WPQ counters; -1 to auto-probe contiguous types");
/**************************************************************************
 * Module main code
 **************************************************************************/
/* similar to on_each_cpu_mask(), but this must be called with IRQ disabled */
static void memguard_on_each_cpu_mask(const struct cpumask *mask, 
				      smp_call_func_t func,
				      void *info, bool wait)
{
	int cpu = smp_processor_id();
	smp_call_function_many(mask, func, info, wait);
	if (cpumask_test_cpu(cpu, mask)) {
		func(info);
	}
}

/** convert MB/s to #of events (i.e., LLC miss counts) per 1ms */
static inline u64 convert_mb_to_events(int mb)
{
	return div64_u64((u64)mb*1024*1024,
			 CACHE_LINE_SIZE * (1000000/g_period_us));
}
static inline int convert_events_to_mb(u64 events)
{
	int divisor = g_period_us*1024*1024;
	int mb = div64_u64(events*CACHE_LINE_SIZE*1000000 + (divisor-1), divisor);
	return mb;
}

static inline void print_current_context(void)
{
	trace_printk("in_interrupt(%ld)(hard(%ld),softirq(%d)"
		     ",in_nmi(%d)),irqs_disabled(%d)\n",
		     in_interrupt(), in_irq(), (int)in_softirq(),
		     (int)in_nmi(), (int)irqs_disabled());
}

/** read current counter value. */
static inline u64 perf_event_count(struct perf_event *event)
{
	return local64_read(&event->count) + 
		atomic64_read(&event->child_count);
}

/*
 * Best-effort read for debug/telemetry paths:
 * force a PMU read when available, then return the software count snapshot.
 */
static inline u64 perf_event_count_fresh(struct perf_event *event)
{
	if (!event)
		return 0;

	if (event->pmu && event->pmu->read)
		event->pmu->read(event);

	return perf_event_count(event);
}

/** return used event in the current period */
static inline u64 memguard_read_event_used(struct core_info *cinfo)
{
	return perf_event_count(cinfo->read_event) - cinfo->old_read_val;
}

/** return used event in the current period */
static inline u64 memguard_write_event_used(struct core_info *cinfo)
{
	return perf_event_count(cinfo->write_event) - cinfo->old_write_val;
}

static void print_core_info(int cpu, struct core_info *cinfo)
{
	pr_info("CPU%d: budget: %d, cur_budget: %d, period: %ld\n", 
		cpu, cinfo->read_budget, cinfo->cur_read_budget, (long)cinfo->period_cnt);
}

/**
 * update per-core usage statistics
 */
static void update_statistics(struct core_info *cinfo)
{
	/* counter must be stopped by now. */
	s64 read_new, write_new;
	int read_used, write_used;

	read_new = perf_event_count(cinfo->read_event);
	read_used = (int)(read_new - cinfo->old_read_val); 

	cinfo->old_read_val = read_new;
	cinfo->overall.used_read_budget += read_used;
	cinfo->overall.assigned_read_budget += cinfo->read_budget;
	
	write_new = perf_event_count(cinfo->write_event);
	write_used = (int)(write_new - cinfo->old_write_val); 

	cinfo->old_write_val = write_new;
	cinfo->overall.used_write_budget += write_used;
	cinfo->overall.assigned_write_budget += cinfo->write_budget;

	/* EWMA filtered per-core usage statistics */
	cinfo->read_used[0] = read_used;
	cinfo->read_used[1] = (cinfo->read_used[1] * (2-1) + read_used) >> 1; 
	/* used[1]_k = 1/2 used[1]_k-1 + 1/2 used */
	cinfo->read_used[2] = (cinfo->read_used[2] * (4-1) + read_used) >> 2; 
	/* used[2]_k = 3/4 used[2]_k-1 + 1/4 used */
	
	/* EWMA filtered per-core usage statistics */
	cinfo->write_used[0] = write_used;
	cinfo->write_used[1] = (cinfo->write_used[1] * (2-1) + write_used) >> 1; 
	/* used[1]_k = 1/2 used[1]_k-1 + 1/2 used */
	cinfo->write_used[2] = (cinfo->write_used[2] * (4-1) + write_used) >> 2;                                          
	/* used[2]_k = 3/4 used[2]_k-1 + 1/4 used */

	/* core is currently throttled. */
	if (cinfo->throttled_task) {
		cinfo->overall.throttled_time_ns +=
			(TM_NS(ktime_get()) - TM_NS(cinfo->throttled_time));
		cinfo->overall.throttled++;
	}

	/* throttling error condition:
	   I was too aggressive in giving up "unsed" budget */
	if (cinfo->prev_read_throttle_error && read_used < cinfo->read_budget) {
		int diff = cinfo->read_budget - read_used;
		int idx;
		cinfo->overall.throttled_error ++; // += diff;
		BUG_ON(cinfo->read_budget == 0);
		idx = (int)(diff * 10 / cinfo->read_budget);
		cinfo->overall.throttled_error_dist[idx]++;
		trace_printk("ERR: read_throttled_error: %d < %d\n", read_used, cinfo->read_budget);
		/* compensation for error to catch-up*/
		cinfo->read_used[PREDICTOR] = cinfo->read_budget + diff;
	}
	cinfo->prev_read_throttle_error = 0;
    
    	if (cinfo->prev_write_throttle_error && write_used < cinfo->write_budget) {
		int diff = cinfo->write_budget - write_used;
		int idx;
		cinfo->overall.throttled_error ++; // += diff;
		BUG_ON(cinfo->write_budget == 0);
		idx = (int)(diff * 10 / cinfo->write_budget);
		cinfo->overall.throttled_error_dist[idx]++;
		trace_printk("ERR: write_throttled_error: %d < %d\n", write_used, cinfo->write_budget);
		/* compensation for error to catch-up*/
		cinfo->write_used[PREDICTOR] = cinfo->write_budget + diff;
	}
	cinfo->prev_write_throttle_error = 0;           

	/* I was the lucky guy who used the DRAM exclusively */
	if (cinfo->exclusive_mode) {
		u64 exclusive_ns, exclusive_bw;

		/* used time */
		exclusive_ns = (TM_NS(ktime_get()) -
				TM_NS(cinfo->exclusive_time));
		
		/* used bw */
		exclusive_bw = (cinfo->read_used[0] - cinfo->read_budget);

		cinfo->overall.exclusive_ns += exclusive_ns;
		cinfo->overall.exclusive_bw += exclusive_bw;
		cinfo->exclusive_mode = 0;
		cinfo->overall.exclusive++;
	}
	DEBUG_PROFILE(trace_printk("used: %d %d read: %d %d write: %d %d period: %ld\n",
				   read_used, write_used,
				   cinfo->read_budget,
				   cinfo->cur_read_budget,
				   cinfo->write_budget,
				   cinfo->cur_write_budget,
				   (long) cinfo->period_cnt));
}


/**
 * budget is used up. PMU generate an interrupt
 * this run in hardirq, nmi context with irq disabled
 */
static void event_overflow_callback(struct perf_event *event,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
				    int nmi,
#endif
				    struct perf_sample_data *data,
				    struct pt_regs *regs)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	BUG_ON(!cinfo);
	irq_work_queue(&cinfo->read_pending);
}

static void event_write_overflow_callback(struct perf_event *event,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
				    int nmi,
#endif
				    struct perf_sample_data *data,
				    struct pt_regs *regs)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	BUG_ON(!cinfo);
	irq_work_queue(&cinfo->write_pending);
}


/**
 * called by process_overflow
 */
static void __unthrottle_core(void *info)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	if (cinfo->throttled_task) {
		cinfo->exclusive_mode = 1;
		cinfo->exclusive_time = ktime_get();

		cinfo->throttled_task = NULL;
		DEBUG_RECLAIM(trace_printk("exclusive mode begin\n"));
	}
}

static void __newperiod(void *info)
{
	ktime_t start = ktime_get();
	struct memguard_info *global = &memguard_info;
	struct core_info *cinfo = this_cpu_ptr(core_info);
	
	ktime_t new_expire = ktime_add(start, global->period_in_ktime);

	/* arrived before timer interrupt is called */
	hrtimer_start_range_ns(&cinfo->hr_timer, new_expire,
			       0, HRTIMER_MODE_ABS_PINNED);

	DEBUG(trace_printk("begin new period\n"));
	period_timer_callback_slave(cinfo);
}

/**
 * memory overflow handler.
 * must not be executed in NMI context. but in hard irq context
 */
static void memguard_read_process_overflow(struct irq_work *entry)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	struct memguard_info *global = &memguard_info;
	ktime_t start;
	s64 read_budget_used;//, write_budget_used;

	start = ktime_get();

	BUG_ON(in_nmi() || !in_irq());

	read_budget_used = memguard_read_event_used(cinfo);

	/* erroneous overflow, that could have happend before period timer
	   stop the pmu */
	if (read_budget_used < cinfo->cur_read_budget) {
		trace_printk("ERR: used %lld < cur_budget %d. ignore\n",
			     read_budget_used, cinfo->cur_read_budget);
		return;
	}

	if (g_use_reclaim) {
		int amount, i;
		/* reclaim back the budget I donated (if exist) */
		if (cinfo->cur_read_budget < cinfo->read_budget) {
			amount = cinfo->read_budget - cinfo->cur_read_budget;
			cinfo->cur_read_budget += amount;
			local64_set(&cinfo->read_event->hw.period_left, amount);
			DEBUG_RECLAIM(trace_printk("locally reclaimed %d\n",
						   amount));
			return;
		}
		/* try to reclaim from the global pool */
		amount = 0;
		for_each_online_cpu(i) {
			struct core_info *ci = per_cpu_ptr(core_info, i);
			if (i == smp_processor_id())
				continue;
			if (ci->cur_read_budget < ci->read_budget) {
				/* reclaim other core's donated budget */
				int tmp = ci->read_budget - ci->cur_read_budget;
				ci->cur_read_budget += tmp;
				amount += tmp;
			}
			if (amount > g_qmin)
				break;
		}
		if (amount > 0) {
			local64_set(&cinfo->read_event->hw.period_left, amount);
			DEBUG_RECLAIM(trace_printk("globally reclaimed %d\n",
						   amount));
			return;
		}
	}

        /* no more overflow interrupt */
	local64_set(&cinfo->read_event->hw.period_left, 0xfffffff);

	/* check if we donated too much */
	if (read_budget_used < cinfo->read_budget) {
		trace_printk("ERR: throttling error\n");
		cinfo->prev_read_throttle_error = 1;
	}

	if (!cpumask_test_cpu(smp_processor_id(), global->active_mask)) {
		trace_printk("ERR: not active\n");
		cinfo->throttled_task = NULL;
		return;
	}
	/* we are going to be throttled */
	cpumask_set_cpu(smp_processor_id(), global->throttle_mask);
	smp_mb(); // w -> r ordering of the local cpu.
	if (cpumask_equal(global->throttle_mask, global->active_mask)) {
		/* all other cores are alreay throttled */
		if (g_use_exclusive == 2) {
			/* SP: wakeup all (no regulation until next period) */
			memguard_on_each_cpu_mask(global->throttle_mask, __unthrottle_core, NULL, 0);
			return;
		} else if (g_use_exclusive == 5) {
			/* PS: begin a new period */
			memguard_on_each_cpu_mask(global->active_mask, __newperiod, NULL, 0);
			return;
		} else if (g_use_exclusive > 5) {
			trace_printk("ERR: Unsupported exclusive mode %d\n", 
				     g_use_exclusive);
			return;
		} else if (g_use_exclusive != 0 &&
			   cpumask_weight(global->active_mask) == 1) {
			trace_printk("ERR: don't throttle one active core\n");
			return;
		}
	}

	if (cinfo->prev_read_throttle_error)
		return;
	/*
	 * fail to reclaim. now throttle this core
	 */
	DEBUG_RECLAIM(trace_printk("fail to reclaim after %lld nsec. throttle %s\n",
				   TM_NS(ktime_get()) - TM_NS(start), current->comm));

	/* wake-up throttle task */
	cinfo->throttled_task = current;
	cinfo->throttled_time = start;

	// WARN_ON_ONCE(!strncmp(current->comm, "swapper", 7));
	wake_up_interruptible(&cinfo->throttle_evt);
}


static void memguard_write_process_overflow(struct irq_work *entry)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	struct memguard_info *global = &memguard_info;
	ktime_t start;
	s64 write_budget_used;

	start = ktime_get();

	BUG_ON(in_nmi() || !in_irq());
    
	write_budget_used = memguard_write_event_used(cinfo);
    
	if (write_budget_used < cinfo->cur_write_budget)
	{
		trace_printk("ERR: overflow in write timer. used %lld < cur_budget %d. ignore\n",
			     write_budget_used, cinfo->cur_write_budget);
		return;
	}

	if (g_use_reclaim) {
		int amount, i;
		/* reclaim back the budget I donated (if exist) */
		if (cinfo->cur_write_budget < cinfo->write_budget) {
			amount = cinfo->write_budget - cinfo->cur_write_budget;
			cinfo->cur_write_budget += amount;
			local64_set(&cinfo->write_event->hw.period_left, amount);
			DEBUG_RECLAIM(trace_printk("locally reclaimed %d\n",
						   amount));
			return;
		}
		/* try to reclaim from the global pool */
		amount = 0;
		for_each_online_cpu(i) {
			struct core_info *ci = per_cpu_ptr(core_info, i);
			if (i == smp_processor_id())
				continue;
			if (ci->cur_write_budget < ci->write_budget) {
				/* reclaim other core's donated budget */
				int tmp = ci->write_budget - ci->cur_write_budget;
				ci->cur_write_budget += tmp;
				amount += tmp;
			}
			if (amount > g_qmin)
				break;
		}
		if (amount > 0) {
			local64_set(&cinfo->write_event->hw.period_left, amount);
			DEBUG_RECLAIM(trace_printk("globally reclaimed %d\n",
						   amount));
			return;
		}
	}

	local64_set(&cinfo->write_event->hw.period_left, 0xfffffff);

	if (write_budget_used < cinfo->write_budget) {
		trace_printk("ERR: throttling error\n");
		cinfo->prev_write_throttle_error = 1;
	}

	if (!cpumask_test_cpu(smp_processor_id(), global->active_mask)) {
		trace_printk("ERR: not active\n");
		cinfo->throttled_task = NULL;
		return;
	}

	cpumask_set_cpu(smp_processor_id(), global->throttle_mask);
	smp_mb(); // w -> r ordering of the local cpu.
	if (cpumask_equal(global->throttle_mask, global->active_mask)) {
		if (g_use_exclusive == 2) {
			/* SP: wakeup all (no regulation until next period) */
			memguard_on_each_cpu_mask(global->throttle_mask, __unthrottle_core, NULL, 0);
			return;
		} else if (g_use_exclusive == 5) {
			/* PS: begin a new period */
			memguard_on_each_cpu_mask(global->active_mask, __newperiod, NULL, 0);
			return;
		} else if (g_use_exclusive > 5) {
			trace_printk("ERR: Unsupported exclusive mode %d\n", 
				     g_use_exclusive);
			return;
		} else if (g_use_exclusive != 0 &&
			   cpumask_weight(global->active_mask) == 1) {
			trace_printk("ERR: don't throttle one active core\n");
			return;
		}
	}

	if (cinfo->prev_write_throttle_error)
		return;
	
	DEBUG_RECLAIM(trace_printk("fail to reclaim after %lld nsec. throttle %s\n",
				   TM_NS(ktime_get()) - TM_NS(start), current->comm));

	cinfo->throttled_task = current;
	cinfo->throttled_time = start;

	// WARN_ON_ONCE(!strncmp(current->comm, "swapper", 7));
	wake_up_interruptible(&cinfo->throttle_evt);
}

/**
 * per-core period timer callback
 *
 *   called while cpu_base->lock is held by hrtimer_interrupt()
 *
 */
enum hrtimer_restart period_timer_callback_master(struct hrtimer *timer)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	struct memguard_info *global = &memguard_info;
	int orun;

	/* must be irq disabled. hard irq */
	BUG_ON(!irqs_disabled());
	// WARN_ON_ONCE(!in_interrupt());

	/* stop counter */
	cinfo->read_event->pmu->stop(cinfo->read_event, PERF_EF_UPDATE);
	cinfo->write_event->pmu->stop(cinfo->write_event, PERF_EF_UPDATE);

	/* forward timer */
	orun = hrtimer_forward_now(timer, global->period_in_ktime);
	BUG_ON(orun == 0);
	if (orun > 1)
		trace_printk("ERR: timer overrun %d at period %ld\n",
			     orun, (long)cinfo->period_cnt);

	/* assign local period */
	cinfo->period_cnt += orun;

	period_timer_callback_slave(cinfo);

	return HRTIMER_RESTART;
}

/**
 * period_timer algorithm:
 *	excess = 0;
 *	if predict < budget:
 *	   excess = budget - predict;
 *	   global += excess
 *	set interrupt at (budget - excess)
 *
 */
static void period_timer_callback_slave(struct core_info *cinfo)
{
	struct memguard_info *global = &memguard_info;
	int cpu = smp_processor_id();

	/* I'm actively participating */
	cpumask_clear_cpu(cpu, global->throttle_mask);
	smp_mb();
	cpumask_set_cpu(cpu, global->active_mask);

	/* update statistics. */
	update_statistics(cinfo);

	/* new budget assignment from user */
	if (cinfo->read_limit > 0)
		cinfo->read_budget = max(cinfo->read_limit, 1);

	/*
	 * Apply latency-based dynamic write cap as a ceiling on top of the
	 * user-configured per-CPU write_limit.
	 */
	if (cinfo->write_limit > 0) {
		int write_budget = max(cinfo->write_limit, 1);
		u64 dyn = READ_ONCE(g_dynamic_write_limit_events);

		if (dyn != (u64)-1) {
			u64 capped = min_t(u64, dyn, (u64)INT_MAX);
			write_budget = min_t(int, write_budget, (int)capped);
		}
		cinfo->write_budget = write_budget;
	}

	if (cinfo->read_event->hw.sample_period != cinfo->read_budget) {
		/* new budget is assigned */
		trace_printk("MSG: new budget %d is assigned\n", 
				   cinfo->read_budget);
		cinfo->read_event->hw.sample_period = cinfo->read_budget;
	}
	if (cinfo->write_event->hw.sample_period != cinfo->write_budget) {
		/* new budget is assigned */
		trace_printk("MSG: new write budget %d is assigned\n", 
				   cinfo->write_budget);
		cinfo->write_event->hw.sample_period = cinfo->write_budget;
	}

	/* per-task donation policy */
        if (g_use_reclaim) {
		/* donate 'expected surplus' ahead of time. */
		int sur_rd, sur_wr;
		sur_rd = max(cinfo->read_budget - cinfo->read_used[PREDICTOR], 0);
		cinfo->cur_read_budget = cinfo->read_budget - sur_rd;
		sur_wr = max(cinfo->write_budget - cinfo->write_used[PREDICTOR], 0);
		cinfo->cur_write_budget = cinfo->write_budget - sur_wr;
		DEBUG_RECLAIM(trace_printk("donated %d %d\n", sur_rd, sur_wr));
        } else {
		cinfo->cur_read_budget = cinfo->read_budget;
		cinfo->cur_write_budget = cinfo->write_budget;
        }

	/* unthrottle tasks (if any) */
	cinfo->throttled_task = NULL;

        /* setup overflow interrupts except RT cores */
	if (!cinfo->rtcore) {
		/* setup an interrupt */
		local64_set(&cinfo->read_event->hw.period_left, cinfo->cur_read_budget);
		local64_set(&cinfo->write_event->hw.period_left, cinfo->cur_write_budget);
	}

	/* enable performance counter */
	cinfo->read_event->pmu->start(cinfo->read_event, PERF_EF_RELOAD);
	cinfo->write_event->pmu->start(cinfo->write_event, PERF_EF_RELOAD);
}

static struct perf_event *init_counter(int cpu, int budget, u64 counter_id, u64 config1,
				       void *callback, int pmu_type, int exclude_kernel)
{
	struct perf_event *event = NULL;
	u64 sample_period = budget ? (u64)budget : 0; /* counting-only if budget==0 */
	struct perf_event_attr sched_perf_hw_attr = {
		.type		= pmu_type,
		.size		= sizeof(struct perf_event_attr),
		.pinned		= 1,
		.disabled	= 1,
		.config         = counter_id,
		.sample_period  = sample_period,
		.exclude_kernel = exclude_kernel,
	};

	/* NEW: support OFFCORE_RESPONSE / config1 */
	sched_perf_hw_attr.config1 = config1;

	/* Try to register using hardware perf events */
	    event = perf_event_create_kernel_counter(
		&sched_perf_hw_attr,
		cpu, NULL,
		callback
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 2, 0)
		, NULL
#endif
		);

	if (!event)
		return NULL;

	if (IS_ERR(event)) {
		/* vary the KERN level based on the returned errno */
		if (PTR_ERR(event) == -EOPNOTSUPP)
			pr_info("cpu%d. not supported\n", cpu);
		else if (PTR_ERR(event) == -ENOENT)
			pr_info("cpu%d. not h/w event\n", cpu);
		else
			pr_err("cpu%d. unable to create perf event: %ld\n",
			       cpu, PTR_ERR(event));
		return NULL;
	}

	/* success path */
		pr_info("cpu%d enabled counter 0x%llx\n", cpu, (unsigned long long)counter_id);

	return event;
}

static void __start_counter(void *info)
{
	struct memguard_info *global = &memguard_info;
	struct core_info *cinfo = this_cpu_ptr(core_info);
	BUG_ON(!cinfo->read_event);
        BUG_ON(!cinfo->write_event);


	/* initialize hr timer */
	hrtimer_init(&cinfo->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	cinfo->hr_timer.function = &period_timer_callback_master;

	/* start timer */
	hrtimer_start(&cinfo->hr_timer, global->period_in_ktime,
					HRTIMER_MODE_REL_PINNED);

	/* initialize */
	cinfo->throttled_task = NULL;
	cinfo->period_cnt = 0;

	/* start performance counter */
	/* cinfo->event->pmu->start(cinfo->event, PERF_EF_RELOAD); */
}

static void __stop_counter(void *info)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);
	BUG_ON(!cinfo->read_event);
	BUG_ON(!cinfo->write_event);
	
	/* stop the kthrottle/i */
	cinfo->throttled_task = NULL;
	cinfo->period_cnt = -1; // done

	/* stop the counter */
	cinfo->read_event->pmu->stop(cinfo->read_event, PERF_EF_UPDATE);
	cinfo->write_event->pmu->stop(cinfo->write_event, PERF_EF_UPDATE);

	/* stop timer */
	hrtimer_cancel(&cinfo->hr_timer);
}


/**************************************************************************
 * Local Functions
 **************************************************************************/

static ssize_t memguard_control_write(struct file *filp,
				    const char __user *ubuf,
				    size_t cnt, loff_t *ppos)
{
	char buf[BUF_SIZE];
	char *p = buf;

	if (copy_from_user(&buf, ubuf, (cnt > BUF_SIZE) ? BUF_SIZE: cnt) != 0)
		return 0;
	if (!strncmp(p, "rt ", 3)) {
		int i, val;
		sscanf(p+3, "%d %d", &i, &val);
		if (i >=0 && i < num_online_cpus())
			per_cpu_ptr(core_info, i)->rtcore = val;
	} else if (!strncmp(p, "reclaim ", 8))
		sscanf(p+8, "%d", &g_use_reclaim);
	else if (!strncmp(p, "exclusive ", 10))
		sscanf(p+10, "%d", &g_use_exclusive);
	else
		pr_info("ERROR: %s\n", p);
	return cnt;
}

static int memguard_control_show(struct seq_file *m, void *v)
{
	struct memguard_info *global = &memguard_info;
	char buf[BUF_SIZE];
	seq_printf(m, "reclaim: %d\n", g_use_reclaim);
	seq_printf(m, "exclusive: %d\n", g_use_exclusive);
	cpumap_print_to_pagebuf(1, buf, global->active_mask);
	seq_printf(m, "active: %s\n", buf);
	cpumap_print_to_pagebuf(1, buf, global->throttle_mask);	
	seq_printf(m, "throttle: %s\n", buf);
	return 0;
}

static int memguard_control_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, memguard_control_show, NULL);
}

static const struct file_operations memguard_control_fops = {
	.open		= memguard_control_open,
	.write          = memguard_control_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void __update_budget(void *info)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);

	if ((unsigned long)info == 0) {
		pr_info("ERR: Requested budget is zero\n");
		return;
	}
	cinfo->read_limit = (unsigned long)info;
	DEBUG_USER(trace_printk("MSG: New read budget of Core%d is %d\n",
				smp_processor_id(), cinfo->read_budget));

}

static void __update_write_budget(void *info)
{
	struct core_info *cinfo = this_cpu_ptr(core_info);

	if ((unsigned long)info == 0) {
		pr_info("ERR: Requested budget is zero\n");
		return;
	}
	cinfo->write_limit = (unsigned long)info;
	DEBUG_USER(trace_printk("MSG: New write budget of Core%d is %d\n",
				smp_processor_id(), cinfo->write_budget));

}
static ssize_t memguard_read_limit_write(struct file *filp,
				    const char __user *ubuf,
				    size_t cnt, loff_t *ppos)
{
	char buf[BUF_SIZE];
	char *p = buf;
	int i;
	int use_mb = 0;

	if (copy_from_user(&buf, ubuf, (cnt > BUF_SIZE) ? BUF_SIZE: cnt) != 0) 
		return 0;

	if (!strncmp(p, "mb ", 3)) {
		use_mb = 1;
		p+=3;
	}
	for_each_online_cpu(i) {
		int input;
		unsigned long events;
		sscanf(p, "%d", &input);
		if (input == 0) {
			pr_err("ERR: CPU%d: input is zero: %s.\n",i, p);
			continue;
		}
		if (use_mb)
			events = (unsigned long)convert_mb_to_events(input);
		else
			events = input;

		pr_info("CPU%d: New budget=%ld (%d %s)\n", i, 
			events, input, (use_mb)?"MB/s": "events");
		smp_call_function_single(i, __update_budget,
					 (void *)events, 0);
		
		p = strchr(p, ' ');
		if (!p) break;
		p++;
	}
	return cnt;
}

static int memguard_read_limit_show(struct seq_file *m, void *v)
{
	int i, cpu;
	cpu = get_cpu();

	seq_printf(m, "cpu  |budget (MB/s)\t RT?\n");
	seq_printf(m, "-------------------------------\n");

	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		int budget = 0;
		if (cinfo->read_limit > 0)
			budget = cinfo->read_limit;

		WARN_ON_ONCE(budget == 0);
		seq_printf(m, "CPU%d: %d (%dMB/s)\t %s\n", 
			   i, budget, convert_events_to_mb(budget),
			   cinfo->rtcore?"Yes":"No");
	}

	put_cpu();
	return 0;
}

static int memguard_read_limit_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, memguard_read_limit_show, NULL);
}

static const struct file_operations memguard_read_limit_fops = {
	.open		= memguard_read_limit_open,
	.write          = memguard_read_limit_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};



/**
 * Display usage statistics
 *
 * TODO: use IPI
 */
static int memguard_usage_show(struct seq_file *m, void *v)
{
	int i, j;

	/* current utilization */
	for (j = 0; j < 3; j++) {
		seq_printf(m, "EWMA[%d]: ", j);
		for_each_online_cpu(i) {
			struct core_info *cinfo = per_cpu_ptr(core_info, i);
			u64 budget, used, util;

			budget = cinfo->read_budget;
			used = cinfo->read_used[j];
			util = div64_u64(used * 100, (budget) ? budget : 1);
			seq_printf(m, "%llu ", util);
		}
		seq_printf(m, "\n");
	}
	seq_printf(m, "<overall>----\n");

	/* overall utilization
	   WARN: assume budget did not changed */
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		u64 total_budget, total_used, result;

		total_budget = cinfo->overall.assigned_read_budget;
		total_used   = cinfo->overall.used_read_budget;
		result       = div64_u64(total_used * 100, 
					 (total_budget) ? total_budget : 1 );
		seq_printf(m, "%lld ", result);
	}
        seq_printf(m, "\n");
	return 0;
}

static int memguard_usage_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, memguard_usage_show, NULL);
}

static const struct file_operations memguard_usage_fops = {
	.open		= memguard_usage_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* expose raw event counts per CPU for streaming write counter */
static int memguard_raw_events_show(struct seq_file *m, void *v)
{
	int i;

	u64 tor_delta = 0;
	u64 tor_occ_delta = 0;
	u64 wpq_occ_delta = 0;
	u64 wpq_ins_delta = 0;
	/* global TOR inserts observer (sum over CHA range) */
	if (tor_ins_count > 0) {
		int idx;
		for (idx = 0; idx < tor_ins_count; idx++) {
			struct perf_event *ev = tor_ins_event[idx];
			u64 tor_now;

			if (!ev)
				continue;
			tor_now = perf_event_count_fresh(ev);
			tor_delta += tor_now - tor_ins_prev[idx];
			tor_ins_prev[idx] = tor_now;
		}
	}
	/* global TOR occupancy observer (sum over CHA range) */
	if (tor_occ_count > 0) {
		int idx;
		for (idx = 0; idx < tor_occ_count; idx++) {
			struct perf_event *ev = tor_occ_event[idx];
			u64 tor_now;

			if (!ev)
				continue;
			tor_now = perf_event_count_fresh(ev);
			tor_occ_delta += tor_now - tor_occ_prev[idx];
			tor_occ_prev[idx] = tor_now;
		}
	}
	/* WPQ observer (sum over HBM PMUs) */
	if (wpq_occ_count > 0 || wpq_ins_count > 0) {
		int idx;
		for (idx = 0; idx < wpq_occ_count; idx++) {
			struct perf_event *ev0 = wpq_occ_pc0_event[idx];
			struct perf_event *ev1 = wpq_occ_pc1_event[idx];
			u64 now_0, now_1;

			if (!ev0 || !ev1)
				continue;
			now_0 = perf_event_count_fresh(ev0);
			now_1 = perf_event_count_fresh(ev1);
			wpq_occ_delta += (now_0 + now_1) -
					 (wpq_occ_pc0_prev[idx] + wpq_occ_pc1_prev[idx]);
			wpq_occ_pc0_prev[idx] = now_0;
			wpq_occ_pc1_prev[idx] = now_1;
		}
		for (idx = 0; idx < wpq_ins_count; idx++) {
			struct perf_event *ev0 = wpq_ins_pc0_event[idx];
			struct perf_event *ev1 = wpq_ins_pc1_event[idx];
			u64 now_0, now_1;

			if (!ev0 || !ev1)
				continue;
			now_0 = perf_event_count_fresh(ev0);
			now_1 = perf_event_count_fresh(ev1);
			wpq_ins_delta += (now_0 + now_1) -
					 (wpq_ins_pc0_prev[idx] + wpq_ins_pc1_prev[idx]);
			wpq_ins_pc0_prev[idx] = now_0;
			wpq_ins_pc1_prev[idx] = now_1;
		}
	}

	u64 tor_lat = 0, wpq_lat = 0;
	int hot = 0;
	
	if (tor_delta > 0)
		tor_lat = div64_u64(tor_occ_delta, tor_delta);
	
	if (wpq_ins_delta > 0)
		wpq_lat = div64_u64(wpq_occ_delta, wpq_ins_delta);
	if (g_tor_lat_threshold > 0 && tor_delta > 0 &&
	    tor_lat > (u64)g_tor_lat_threshold)
		hot = 1;
	if (g_wpq_lat_threshold > 0 && wpq_ins_delta > 0 &&
	    wpq_lat > (u64)g_wpq_lat_threshold)
		hot = 1;

	/* If we can't compute latency this time, don't change gating state. */
	if (g_tor_lat_threshold <= 0 || tor_delta <= 0)
		goto out_latency_gating;
	if (g_use_wpq && g_wpq_lat_threshold > 0 && wpq_ins_delta <= 0)
		goto out_latency_gating;

	/*
	 * Latency gating: when memory is "hot", cap write bandwidth to
	 * g_write_budget_mb; otherwise relax up to g_relax_write_budget_mb.
	 * The cap is applied in the periodic budget assignment path.
	 */
	int target_mb = hot ? g_throttle_write_budget_mb : g_relax_write_budget_mb;
	u64 target_events = (target_mb > 0) ?
				convert_mb_to_events(target_mb) :
				(u64)-1;
	WRITE_ONCE(g_latency_hot, hot);
	WRITE_ONCE(g_dynamic_write_limit_events, target_events);

out_latency_gating:
	

	seq_printf(m, "cpu  |write/s\n");
	seq_printf(m, "------------------------------\n");

	seq_printf(m, "TOR inserts delta: %llu\n", (unsigned long long)tor_delta);
	seq_printf(m, "TOR occupancy delta: %llu\n",
		   (unsigned long long)tor_occ_delta);
	seq_printf(m, "WPQ inserts delta: %llu\n",
		   (unsigned long long)wpq_ins_delta);
	seq_printf(m, "WPQ occupancy delta: %llu\n",
		   (unsigned long long)wpq_occ_delta);
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);

		u64 w_now = 0;
		u64 w_prev = *per_cpu_ptr(raw_write_prev, i);

		if (cinfo->write_event)
			w_now = perf_event_count(cinfo->write_event);

		u64 w_delta = w_now - w_prev;

		/* update snapshot */
		*per_cpu_ptr(raw_write_prev, i) = w_now;

		seq_printf(m, "CPU%d: %llu\n",
				   i,
				   (unsigned long long)w_delta);
	}

	return 0;
}


static int memguard_raw_events_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, memguard_raw_events_show, NULL);
}

static const struct file_operations memguard_raw_events_fops = {
	.open		= memguard_raw_events_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* modified for easier limit setting */
static ssize_t memguard_write_limit_write(struct file *filp,
				    const char __user *ubuf,
				    size_t cnt, loff_t *ppos)
{
	char buf[BUF_SIZE];
	char *token, *p;
	int use_mb = 0;

	/* copy user input */
	if (copy_from_user(&buf, ubuf, min(cnt, (size_t)BUF_SIZE-1)))
		return -EFAULT;

	buf[min(cnt, (size_t)BUF_SIZE-1)] = '\0';
	p = buf;

	/* optional "mb " prefix */
	if (!strncmp(p, "mb ", 3)) {
		use_mb = 1;
		p += 3;
	}

	/*
	 * New parser:
	 * Accepted syntaxes:
	 *   cpu2=3000
	 *   cpu2:3000
	 *   2=3000
	 *   2:3000
	 *   2 3000     (pair format)
	 */
	while ((token = strsep(&p, " \t\n,")) != NULL) {

		int cpu = -1;
		unsigned long input = 0;
		unsigned long events = 0;

		if (*token == '\0')
			continue;

		/* Case 1: cpuX=Y or cpuX:Y */
		if (sscanf(token, "cpu%d=%lu", &cpu, &input) == 2 ||
		    sscanf(token, "cpu%d:%lu", &cpu, &input) == 2)
			goto apply;

		/* Case 2: X=Y or X:Y */
		if (sscanf(token, "%d=%lu", &cpu, &input) == 2 ||
		    sscanf(token, "%d:%lu", &cpu, &input) == 2)
			goto apply;

		/* Case 3: pair syntax: "<cpu> <value>" */
		if (sscanf(token, "%d", &cpu) == 1) {
			char *next = strsep(&p, " \t\n,");
			if (!next)
				break;
			if (sscanf(next, "%lu", &input) != 1)
				continue;
			goto apply;
		}

		continue;

apply:
		/* validate CPU */
		if (cpu < 0 || cpu >= nr_cpu_ids || !cpu_online(cpu)) {
			pr_warn("memguard: ignoring invalid CPU %d\n", cpu);
			continue;
		}

		/* validate input value */
		if (input == 0) {
			pr_warn("memguard: ignoring zero write_limit for CPU%d\n",
				 cpu);
			continue;
		}

		/* convert MB → events if needed */
		events = use_mb ?
			 (unsigned long)convert_mb_to_events(input) :
			 input;

		pr_info("memguard: CPU%d new write_budget=%lu (%lu %s)\n",
			cpu, events, input, use_mb ? "MB/s" : "events");

		smp_call_function_single(cpu, __update_write_budget,
					 (void *)events, 0);
	}
	return cnt;
}

static int memguard_write_limit_show(struct seq_file *m, void *v)
{
	int i, cpu;
	cpu = get_cpu();

	seq_printf(m, "cpu  |budget (MB/s)\t RT?\n");
	seq_printf(m, "-------------------------------\n");

	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		int budget = 0;
		if (cinfo->write_limit > 0)
			budget = cinfo->write_limit;

		WARN_ON_ONCE(budget == 0);
		seq_printf(m, "CPU%d: %d (%dMB/s) %s\n", 
			   i, budget, convert_events_to_mb(budget),
			   cinfo->rtcore?"Yes":"No");
	}

	put_cpu();
	return 0;
}

static int memguard_write_limit_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, memguard_write_limit_show, NULL);
}

static const struct file_operations memguard_write_limit_fops = {
	.open		= memguard_write_limit_open,
	.write          = memguard_write_limit_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static int memguard_init_debugfs(void)
{

	memguard_dir = debugfs_create_dir("memguard", NULL);
	BUG_ON(!memguard_dir);
	debugfs_create_file("control", 0444, memguard_dir, NULL,
			    &memguard_control_fops);

	debugfs_create_file("read_limit", 0444, memguard_dir, NULL,
			    &memguard_read_limit_fops);
				
	debugfs_create_file("write_limit", 0444, memguard_dir, NULL,
			    &memguard_write_limit_fops);

	debugfs_create_file("usage", 0666, memguard_dir, NULL,
			    &memguard_usage_fops);

	/* raw per-CPU event counters */
	debugfs_create_file("raw_events", 0444, memguard_dir, NULL,
			    &memguard_raw_events_fops);
	return 0;
}

static int throttle_thread(void *arg)
{
	int cpunr = (unsigned long)arg;
	struct core_info *cinfo = per_cpu_ptr(core_info, cpunr);

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 9, 0)
	sched_set_fifo(current);
#else
	static const struct sched_param param = {
		.sched_priority = MAX_USER_RT_PRIO/2,
	};

	sched_setscheduler(current, SCHED_FIFO, &param);
#endif

	while (!kthread_should_stop() && cpu_online(cpunr)) {

		DEBUG(trace_printk("wait an event\n"));
		wait_event_interruptible(cinfo->throttle_evt,
					 cinfo->throttled_task ||
					 kthread_should_stop());

		DEBUG(trace_printk("got an event\n"));

		if (kthread_should_stop())
			break;

		while (cinfo->throttled_task && !kthread_should_stop())
		{
			smp_mb();
			cpu_relax();
			/* TODO: mwait */
		}
	}

	DEBUG(trace_printk("exit\n"));
	return 0;
}

int init_module( void )
{
	int i;

	struct memguard_info *global = &memguard_info;

	/* initialized memguard_info structure */
	memset(global, 0, sizeof(struct memguard_info));
	zalloc_cpumask_var(&global->throttle_mask, GFP_NOWAIT);
	zalloc_cpumask_var(&global->active_mask, GFP_NOWAIT);
	if (g_period_us < 0 || g_period_us > 1000000) {
		printk(KERN_INFO "Must be 0 < period < 1 sec\n");
		return -ENODEV;
	}

	global->period_in_ktime = ktime_set(0, g_period_us * 1000);

	/* initialize all online cpus to be active */
	cpumask_copy(global->active_mask, cpu_online_mask);

	pr_info("NR_CPUS: %d, online: %d\n", NR_CPUS, num_online_cpus());

#if defined(__arm__) || defined(__aarch64__)
	/* check if we are running on ARM */
	u32 cpu_id = (u32)read_cpuid_id();
	u32 cpu_part = (cpu_id >> 4) & 0xFFF;
	if (cpu_part == 0xD0B || cpu_part == 0xD42) { // Cortex-A76/A78
		pr_info("Cortex-A76/A78 detected\n");
		g_read_counter_id = 0x002A;
		g_write_counter_id = 0x002C; // didn't work on cortex-a76
	}
#endif
	if (g_read_counter_id >= 0)
		pr_info("RAW HW READ COUNTER ID: 0x%x\n", g_read_counter_id);
	if (g_write_counter_id >= 0)
		pr_info("RAW HW WRITE COUNTER ID: 0x%x\n", g_write_counter_id);	

	if (g_read_config1)
    pr_info("RAW HW READ CONFIG1 (offcore_rsp): 0x%llx\n",
            (unsigned long long)g_read_config1);
	if (g_write_config1)
		pr_info("RAW HW WRITE CONFIG1 (offcore_rsp): 0x%llx\n",
				(unsigned long long)g_write_config1);
				
	pr_info("HZ=%d, g_period_us=%d\n", HZ, g_period_us);
	pr_info("g_read_budget_mb=%d, g_write_budget_mb=%d\n", g_read_budget_mb, g_write_budget_mb);
	g_qmin = convert_mb_to_events(DEFAULT_QMIN_MB); // default 1000MB/s
	g_dynamic_write_limit_events = convert_mb_to_events(g_relax_write_budget_mb);

	pr_info("Initilizing perf counter\n");
	core_info = alloc_percpu(struct core_info);

	/* allocate per-CPU previous values for raw_events snapshot */
	raw_read_prev  = alloc_percpu(u64);
	raw_write_prev = alloc_percpu(u64);

	/* optional global TOR inserts observers (range of CHA PMUs) */
	for (i = 0; i < MAX_TOR_CHA; i++) {
		tor_ins_prev[i] = 0;
		tor_ins_event[i] = NULL;
		tor_occ_prev[i] = 0;
		tor_occ_event[i] = NULL;
	}
	tor_ins_count = 0;
	tor_occ_count = 0;

	/* clear WPQ state */
	for (i = 0; i < MAX_WPQ_HBM; i++) {
		wpq_ins_pc0_prev[i] = 0;
		wpq_ins_pc1_prev[i] = 0;
		wpq_occ_pc0_prev[i] = 0;
		wpq_occ_pc1_prev[i] = 0;
		wpq_ins_pc0_event[i] = NULL;
		wpq_ins_pc1_event[i] = NULL;
		wpq_occ_pc0_event[i] = NULL;
		wpq_occ_pc1_event[i] = NULL;
	}
	wpq_ins_count = 0;
	wpq_occ_count = 0;

	if (g_tor_ins_counter_id != 0 && g_tor_pmu_type > 0) {
		int type;
		int max_type = (g_tor_pmu_type_last >= g_tor_pmu_type) ?
				g_tor_pmu_type_last :
				g_tor_pmu_type + MAX_TOR_CHA - 1;

		for (type = g_tor_pmu_type;
		     type <= max_type && tor_ins_count < MAX_TOR_CHA;
		     type++) {
			struct perf_event *ev = init_counter(0, 0,
							     g_tor_ins_counter_id, 0,
							     NULL, type, 0);
			if (!ev || IS_ERR(ev)) {
				pr_warn("memguard: failed TOR counter pmu_type=%d (config=0x%llx)\n",
					type,
					(unsigned long long)g_tor_ins_counter_id);
				continue;
			}
			perf_event_enable(ev);
			tor_ins_event[tor_ins_count] = ev;
			tor_ins_prev[tor_ins_count] = 0;
			tor_ins_count++;
		}
	}
	if (g_tor_occ_counter_id != 0 && g_tor_pmu_type > 0) {
		int type;
		int max_type = (g_tor_pmu_type_last >= g_tor_pmu_type) ?
				g_tor_pmu_type_last :
				g_tor_pmu_type + MAX_TOR_CHA - 1;

		for (type = g_tor_pmu_type;
		     type <= max_type && tor_occ_count < MAX_TOR_CHA;
		     type++) {
			struct perf_event *ev = init_counter(0, 0,
							     g_tor_occ_counter_id, 0,
							     NULL, type, 0);
			if (!ev || IS_ERR(ev)) {
				pr_warn("memguard: failed TOR occupancy counter pmu_type=%d (config=0x%llx)\n",
					type,
					(unsigned long long)g_tor_occ_counter_id);
				continue;
			}
			perf_event_enable(ev);
			tor_occ_event[tor_occ_count] = ev;
			tor_occ_prev[tor_occ_count] = 0;
			tor_occ_count++;
		}
	}

	if (g_use_wpq) {
		/*  WPQ counters across HBM PMUs */
		if (g_wpq_pmu_type > 0) {
			int type;
			int max_type = (g_wpq_pmu_type_last >= g_wpq_pmu_type) ?
					g_wpq_pmu_type_last :
					g_wpq_pmu_type + (PMU_WPQ_PMU_END - PMU_WPQ_PMU_START);
			int wpq_count = 0;

			for (type = g_wpq_pmu_type;
				type <= max_type && wpq_count < MAX_WPQ_HBM;
				type++) {

				struct perf_event *occ0 = NULL, *occ1 = NULL;
				struct perf_event *ins0 = NULL, *ins1 = NULL;
				u64 occ0_cfg = g_wpq_occ_pc0_counter_id ? g_wpq_occ_pc0_counter_id : PMU_WPQ_OCC_PC0_ID;
				u64 occ1_cfg = g_wpq_occ_pc1_counter_id ? g_wpq_occ_pc1_counter_id : PMU_WPQ_OCC_PC1_ID;
				u64 ins0_cfg = g_wpq_ins_pc0_counter_id ? g_wpq_ins_pc0_counter_id : PMU_WPQ_INS_PC0_ID;
				u64 ins1_cfg = g_wpq_ins_pc1_counter_id ? g_wpq_ins_pc1_counter_id : PMU_WPQ_INS_PC1_ID;

				occ0 = init_counter(0, 0, occ0_cfg, 0, NULL, type, 0);
				occ1 = init_counter(0, 0, occ1_cfg, 0, NULL, type, 0);
				ins0 = init_counter(0, 0, ins0_cfg, 0, NULL, type, 0);
				ins1 = init_counter(0, 0, ins1_cfg, 0, NULL, type, 0);

				if (!occ0 || !occ1 || !ins0 || !ins1) {
					pr_warn("memguard: failed WPQ counters pmu_type=%d\n", type);
					if (occ0) perf_event_release_kernel(occ0);
					if (occ1) perf_event_release_kernel(occ1);
					if (ins0) perf_event_release_kernel(ins0);
					if (ins1) perf_event_release_kernel(ins1);
					continue;
				}

				perf_event_enable(occ0);
				perf_event_enable(occ1);
				perf_event_enable(ins0);
				perf_event_enable(ins1);

				wpq_occ_pc0_event[wpq_count] = occ0;
				wpq_occ_pc1_event[wpq_count] = occ1;
				wpq_ins_pc0_event[wpq_count] = ins0;
				wpq_ins_pc1_event[wpq_count] = ins1;
				wpq_occ_pc0_prev[wpq_count] = 0;
				wpq_occ_pc1_prev[wpq_count] = 0;
				wpq_ins_pc0_prev[wpq_count] = 0;
				wpq_ins_pc1_prev[wpq_count] = 0;

				wpq_count++;
			}

			/* keep counts aligned across pc0/pc1 for occ/ins */
			wpq_occ_count = wpq_count;
			wpq_ins_count = wpq_count;
		}
	} else {
		wpq_occ_count = 0;
		wpq_ins_count = 0;
	}

	for_each_online_cpu(i) {
		*per_cpu_ptr(raw_read_prev, i)  = 0;
		*per_cpu_ptr(raw_write_prev, i) = 0;
	}

	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		int read_budget, write_budget;

		/* initialize counter h/w & event structure */
		read_budget = convert_mb_to_events(g_read_budget_mb);
		write_budget = convert_mb_to_events(g_write_budget_mb);

		/* initialize per-core data structure */
		memset(cinfo, 0, sizeof(struct core_info));

		/* create performance counter */
		cinfo->read_event = init_counter(i, read_budget,
							(u64)g_read_counter_id, g_read_config1,
							event_overflow_callback, PERF_TYPE_RAW, 1);
		cinfo->write_event = init_counter(i, write_budget,
							(u64)g_write_counter_id, g_write_config1,
							event_write_overflow_callback, PERF_TYPE_RAW, 1);
		if (!cinfo->read_event || !cinfo->write_event)
			break;

		/* initialize budget */
		cinfo->read_budget = cinfo->read_limit = cinfo->read_event->hw.sample_period;
		cinfo->write_budget = cinfo->write_limit = cinfo->write_event->hw.sample_period;

		/* throttled task pointer */
		cinfo->throttled_task = NULL;

		init_waitqueue_head(&cinfo->throttle_evt);

		/* initialize statistics */
		/* update local period information */
		cinfo->period_cnt = 0;
		cinfo->read_used[0] = cinfo->read_used[1] = cinfo->read_used[2] =
			cinfo->read_budget; /* initial condition */
		cinfo->cur_read_budget = cinfo->read_budget;
		cinfo->overall.used_read_budget = 0;
		cinfo->overall.assigned_read_budget = 0;
        
		cinfo->cur_write_budget = cinfo->write_budget;
		cinfo->overall.used_write_budget = 0;
		cinfo->overall.assigned_write_budget = 0;
        
		cinfo->overall.throttled_time_ns = 0;
		cinfo->overall.throttled = 0;
		cinfo->overall.throttled_error = 0;

		memset(cinfo->overall.throttled_error_dist, 0, sizeof(int)*10);
		cinfo->throttled_time = ktime_set(0,0);

		print_core_info(smp_processor_id(), cinfo);

		/* initialize nmi irq_work_queue */
		init_irq_work(&cinfo->read_pending, memguard_read_process_overflow);
        	init_irq_work(&cinfo->write_pending, memguard_write_process_overflow);

		/* create and wake-up throttle threads */
		cinfo->throttle_thread =
			kthread_create_on_node(throttle_thread,
					       (void *)((unsigned long)i),
					       cpu_to_node(i),
					       "kthrottle/%d", i);

		perf_event_enable(cinfo->read_event);
		perf_event_enable(cinfo->write_event);	
		
		BUG_ON(IS_ERR(cinfo->throttle_thread));
		kthread_bind(cinfo->throttle_thread, i);
		wake_up_process(cinfo->throttle_thread);
	}

	memguard_init_debugfs();

	/* start timer and perf counters */
	pr_info("Start period timer (period=%lld us)\n",
		div64_u64(TM_NS(global->period_in_ktime), 1000));
	on_each_cpu(__start_counter, NULL, 0);

	return 0;
}

void cleanup_module( void )
{
	int i;

	struct memguard_info *global = &memguard_info;

	/* stop perf_event counters and timers */
	on_each_cpu(__stop_counter, NULL, 0);
	smp_mb(); /* make exit flag visible to all CPUs */

	/*
	 * Stop per-CPU period timers from process context (not via on_each_cpu),
	 * to avoid deadlocking by interrupting a running hrtimer callback.
	 */
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		if (!cinfo)
			continue;
		cinfo->throttled_task = NULL;
		hrtimer_cancel(&cinfo->hr_timer);
	}

	/* Disable perf events and flush queued overflow work. */
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		if (!cinfo)
			continue;

		irq_work_sync(&cinfo->read_pending);
		irq_work_sync(&cinfo->write_pending);

		if (cinfo->read_event)
			perf_event_disable(cinfo->read_event);
		if (cinfo->write_event)
			perf_event_disable(cinfo->write_event);
	}

	pr_info("LLC bandwidth throttling disabled\n");

	/* stop per-CPU throttle threads */
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		if (!cinfo)
			continue;
		pr_info("Stopping kthrottle/%d\n", i);
		if (cinfo->throttle_thread) {
			kthread_stop(cinfo->throttle_thread);
			cinfo->throttle_thread = NULL;
		}
	}

	/* release per-CPU perf events */
	for_each_online_cpu(i) {
		struct core_info *cinfo = per_cpu_ptr(core_info, i);
		if (!cinfo)
			continue;
		if (cinfo->read_event) {
			perf_event_release_kernel(cinfo->read_event);
			cinfo->read_event = NULL;
		}
		if (cinfo->write_event) {
			perf_event_release_kernel(cinfo->write_event);
			cinfo->write_event = NULL;
		}
	}

	for (i = 0; i < tor_ins_count; i++) {
		if (tor_ins_event[i]) {
			perf_event_disable(tor_ins_event[i]);
			perf_event_release_kernel(tor_ins_event[i]);
			tor_ins_event[i] = NULL;
		}
	}
	for (i = 0; i < tor_occ_count; i++) {
		if (tor_occ_event[i]) {
			perf_event_disable(tor_occ_event[i]);
			perf_event_release_kernel(tor_occ_event[i]);
			tor_occ_event[i] = NULL;
		}
	}
	for (i = 0; i < wpq_occ_count; i++) {
		if (wpq_occ_pc0_event[i]) {
			perf_event_disable(wpq_occ_pc0_event[i]);
			perf_event_release_kernel(wpq_occ_pc0_event[i]);
			wpq_occ_pc0_event[i] = NULL;
		}
		if (wpq_occ_pc1_event[i]) {
			perf_event_disable(wpq_occ_pc1_event[i]);
			perf_event_release_kernel(wpq_occ_pc1_event[i]);
			wpq_occ_pc1_event[i] = NULL;
		}
	}
	for (i = 0; i < wpq_ins_count; i++) {
		if (wpq_ins_pc0_event[i]) {
			perf_event_disable(wpq_ins_pc0_event[i]);
			perf_event_release_kernel(wpq_ins_pc0_event[i]);
			wpq_ins_pc0_event[i] = NULL;
		}
		if (wpq_ins_pc1_event[i]) {
			perf_event_disable(wpq_ins_pc1_event[i]);
			perf_event_release_kernel(wpq_ins_pc1_event[i]);
			wpq_ins_pc1_event[i] = NULL;
		}
	}

	/* free allocated data structure */
	if (raw_read_prev) {
		free_percpu(raw_read_prev);
		raw_read_prev = NULL;
	}
	if (raw_write_prev) {
		free_percpu(raw_write_prev);
		raw_write_prev = NULL;
	}
	free_cpumask_var(global->throttle_mask);
	free_cpumask_var(global->active_mask);
	free_percpu(core_info);
	core_info = NULL;
	tor_ins_count = 0;
	tor_occ_count = 0;
	wpq_ins_count = 0;
	wpq_occ_count = 0;
	pr_info("module uninstalled successfully\n");
	return;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heechul Yun <heechul@illinois.edu>");
