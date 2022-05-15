#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/page.h>
#include <asm/pgtable.h>
#include "internal.h"

void __attribute__((weak)) arch_report_meminfo(struct seq_file *m)
{
}

static int meminfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pagecache;
	unsigned long wmark_low = 0;
	unsigned long pages[NR_LRU_LISTS];
	struct zone *zone;
	int lru;

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_page_state(NR_LRU_BASE + lru);

	for_each_zone(zone)
		wmark_low += zone->watermark[WMARK_LOW];

	/*
	 * Estimate the amount of memory available for userspace allocations,
	 * without causing swapping.
	 */
	available = i.freeram - totalreserve_pages;

	/*
	 * Not all the page cache can be freed, otherwise the system will
	 * start swapping. Assume at least half of the page cache, or the
	 * low watermark worth of cache, needs to stay.
	 */
	pagecache = pages[LRU_ACTIVE_FILE] + pages[LRU_INACTIVE_FILE];
	pagecache -= min(pagecache / 2, wmark_low);
	available += pagecache;

	/*
	 * Part of the reclaimable slab consists of items that are in use,
	 * and cannot be freed. Cap this estimate at the low watermark.
	 */
	available += global_page_state(NR_SLAB_RECLAIMABLE) -
		     min(global_page_state(NR_SLAB_RECLAIMABLE) / 2, wmark_low);

	/*
	 * Part of the kernel memory, which can be released under memory
	 * pressure.
	 */
	available += global_page_state(NR_INDIRECTLY_RECLAIMABLE_BYTES) >>
		PAGE_SHIFT;

	if (available < 0)
		available = 0;

	/*
	 * Tagged format, for easy grepping and expansion.
	 */
	seq_printf(m,
		"MemTotal:       %8lu kB\n"
		"MemFree:        %8lu kB\n"
		"MemAvailable:   %8lu kB\n"
		"Buffers:        %8lu kB\n"
		"Cached:         %8lu kB\n"
		"SwapCached:     %8lu kB\n"
		"Active:         %8lu kB\n"
		"Inactive:       %8lu kB\n"
		"Active(anon):   %8lu kB\n"
		"Inactive(anon): %8lu kB\n"
		"Active(file):   %8lu kB\n"
		"Inactive(file): %8lu kB\n"
		"Unevictable:    %8lu kB\n"
		"Mlocked:        %8lu kB\n"
#ifdef CONFIG_HIGHMEM
		"HighTotal:      %8lu kB\n"
		"HighFree:       %8lu kB\n"
		"LowTotal:       %8lu kB\n"
		"LowFree:        %8lu kB\n"
#endif
#ifndef CONFIG_MMU
		"MmapCopy:       %8lu kB\n"
#endif
		"SwapTotal:      %8lu kB\n"
		"SwapFree:       %8lu kB\n"
		"Dirty:          %8lu kB\n"
		"Writeback:      %8lu kB\n"
		"AnonPages:      %8lu kB\n"
		"Mapped:         %8lu kB\n"
		"Shmem:          %8lu kB\n"
		"Slab:           %8lu kB\n"
		"SReclaimable:   %8lu kB\n"
		"SUnreclaim:     %8lu kB\n"
		"KernelStack:    %8lu kB\n"
		"PageTables:     %8lu kB\n"
#ifdef CONFIG_QUICKLIST
		"Quicklists:     %8lu kB\n"
#endif
		"NFS_Unstable:   %8lu kB\n"
		"Bounce:         %8lu kB\n"
		"WritebackTmp:   %8lu kB\n"
		"CommitLimit:    %8lu kB\n"
		"Committed_AS:   %8lu kB\n"
		"VmallocTotal:   %8lu kB\n"
		"VmallocUsed:    %8lu kB\n"
		"VmallocChunk:   %8lu kB\n"
#ifdef CONFIG_MEMORY_FAILURE
		"HardwareCorrupted: %5lu kB\n"
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		"AnonHugePages:  %8lu kB\n"
#endif
#ifdef CONFIG_CMA
		"CmaTotal:       %8lu kB\n"
		"CmaFree:        %8lu kB\n"
#endif
		,
		K(i.totalram),
		K(i.freeram),
		K(available),
		K(i.bufferram),
		K(cached),
		K(total_swapcache_pages()),
		K(pages[LRU_ACTIVE_ANON]   + pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_ACTIVE_ANON]),
		K(pages[LRU_INACTIVE_ANON]),
		K(pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_UNEVICTABLE]),
		K(global_page_state(NR_MLOCK)),
#ifdef CONFIG_HIGHMEM
		K(i.totalhigh),
		K(i.freehigh),
		K(i.totalram-i.totalhigh),
		K(i.freeram-i.freehigh),
#endif
#ifndef CONFIG_MMU
		K((unsigned long) atomic_long_read(&mmap_pages_allocated)),
#endif
		K(i.totalswap),
		K(i.freeswap),
		K(global_page_state(NR_FILE_DIRTY)),
		K(global_page_state(NR_WRITEBACK)),
		K(global_page_state(NR_ANON_PAGES)),
		K(global_page_state(NR_FILE_MAPPED)),
		K(i.sharedram),
		K(global_page_state(NR_SLAB_RECLAIMABLE) +
				global_page_state(NR_SLAB_UNRECLAIMABLE)),
		K(global_page_state(NR_SLAB_RECLAIMABLE)),
		K(global_page_state(NR_SLAB_UNRECLAIMABLE)),
		global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024,
		K(global_page_state(NR_PAGETABLE)),
#ifdef CONFIG_QUICKLIST
		K(quicklist_total_size()),
#endif
		K(global_page_state(NR_UNSTABLE_NFS)),
		K(global_page_state(NR_BOUNCE)),
		K(global_page_state(NR_WRITEBACK_TEMP)),
		K(vm_commit_limit()),
		K(committed),
		(unsigned long)VMALLOC_TOTAL >> 10,
		0ul, // used to be vmalloc 'used'
		0ul  // used to be vmalloc 'largest_chunk'
#ifdef CONFIG_MEMORY_FAILURE
		, atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10)
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		, K(global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) *
		   HPAGE_PMD_NR)
#endif
#ifdef CONFIG_CMA
		, K(totalcma_pages)
		, K(global_page_state(NR_FREE_CMA_PAGES))
#endif
		);

	hugetlb_report_meminfo(m);

	arch_report_meminfo(m);

	return 0;
#undef K
}

static int meminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_proc_show, NULL);
}

static const struct file_operations meminfo_proc_fops = {
	.open		= meminfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int meminfo_quick_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	long cached;
	unsigned long reserved = 0;
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	/* kernel4.9: cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram; */
	cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;
	seq_printf(m,
		"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n",
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(cached),
		K(i.sharedram),
		/* upper are compatitable for old version */
		/* 5: swap total */
		K(i.totalswap),
		/* 6: swap free */
		K(i.freeswap),
		/* 7: anon pages */
		/* kernel-4.9: K(global_node_page_state(NR_ANON_MAPPED)),*/
		K(global_page_state(NR_ANON_PAGES)),
		/* 8: file mapped pages */
		/* kernel-4.9: K(global_node_page_state(NR_FILE_MAPPED)),*/
		K(global_page_state(NR_FILE_MAPPED)),
		/* 9: slab reclaimable */
		K(global_page_state(NR_SLAB_RECLAIMABLE)),
		/* 10: slab unreclaimable */
		K(global_page_state(NR_SLAB_UNRECLAIMABLE)),
		/* 11: kernel stack */
		/* kernel 4.9: global_page_state(NR_KERNEL_STACK_KB),*/
		global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024,
		/* 12: page tables */
		K(global_page_state(NR_PAGETABLE)),
#ifdef CONFIG_RSC_MEM_STAT
		/* 13: ion free */
		K((unsigned long)atomic_read(&rsc_ion_pool_pages) +
									(rsc_ion_head_free_list_size >> PAGE_SHIFT)),
		/* 14: gpu free */
		K((unsigned long)rsc_kgsl_pool_size_total()),
#ifdef CONFIG_ZCACHE
		/* 15: zcache total */
		K((unsigned long)zcache_pages()),
#else
		/* 15: zcache total */
		reserved,
#endif
#else
		/* 13: ion free */
		reserved,
		/* 14: gpu free */
		reserved,
		/* 15: zcache total */
		reserved,
#endif
		/* 16: reserved */
		reserved,
		/* 17: reserved */
		reserved,
		/* 18: reserved */
		reserved
		);
	return 0;

#undef K
}

static int meminfo_quick_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_quick_proc_show, NULL);
}

static const struct file_operations meminfo_quick_proc_fops = {
	.open		= meminfo_quick_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if defined(CONFIG_RMS_ION_STAT) && defined(CONFIG_RMS_GPU_STAT)
static int rms_version_proc_show(struct seq_file *m, void *v)
{
	int version = 600;
	int other1 = 0;
	int other2 = 0;
	seq_printf(m, "%d %d %d\n", version, other1, other2);
	return 0;
}

static int rms_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rms_version_proc_show, NULL);
}

static const struct file_operations rms_version_proc_fops = {
	.open		= rms_version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int __init proc_meminfo_init(void)
{
	proc_create("meminfo", 0, NULL, &meminfo_proc_fops);
	proc_create("meminfo_quick", S_IRUGO, NULL, &meminfo_quick_proc_fops);
#if defined(CONFIG_RMS_ION_STAT) && defined(CONFIG_RMS_GPU_STAT) && defined(CONFIG_RSC_MEM_STAT)
	proc_create("rms_version", 0444, NULL, &rms_version_proc_fops);
#endif
	return 0;
}
fs_initcall(proc_meminfo_init);
