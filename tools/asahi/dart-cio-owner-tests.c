// SPDX-License-Identifier: GPL-2.0-only
/* Included after the MMIO fixture and before the actual ownership methods. */
#include <limits.h>
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
/* Single-threaded fixture: model publication without CPU barriers. */
#define smp_store_release(p, v) (*(p) = (v))
#define dma_wmb() ((void)0)
#define spin_lock_irqsave(lock, flags) \
	do {                           \
		(void)(lock);          \
		(flags) = 0;           \
	} while (0)
#define spin_unlock_irqrestore(lock, flags) \
	do {                                \
		(void)(lock);               \
		(void)(flags);              \
	} while (0)
#define APPLE_PS_PROT_SLICE(n) (0x100 + 0x40 * (n))
#define PM_SUSPEND_MEM 3
static int pm_suspend_target_state = PM_SUSPEND_MEM;
struct device {
	int refs;
};
struct device_link {};
struct regmap {
	u32 value;
	int fail, fail_read;
	unsigned int offset;
};
struct apple_dart_sleep {
	int result;
};
struct apple_dart_cio;
struct apple_dart {
	struct device *dev;
	void *regs;
	unsigned int num_streams, sleep_writers;
	bool managed_sids, sleep_closed, commands_gated, cio_active,
		cio_violation;
	int managed_lock;
	struct apple_dart_cio *cio;
	struct apple_dart_sleep *sleep_record;
};
static int pm_runtime_resume_and_get(struct device *dev)
{
	dev->refs++;
	return 0;
}
static void pm_runtime_put(struct device *dev)
{
	assert(dev->refs > 0);
	dev->refs--;
}
static int regmap_read(struct regmap *map, unsigned int off, u32 *value)
{
	assert(off == map->offset);
	if (map->fail_read)
		return -EIO;
	*value = map->value;
	return 0;
}
static int regmap_write(struct regmap *map, unsigned int off, u32 value)
{
	assert(off == map->offset);
	if (map->fail)
		return -EIO;
	map->value = value;
	return 0;
}
static int regmap_update_bits(struct regmap *map, unsigned int off, u32 mask,
			      u32 value)
{
	assert(off == map->offset && mask == BIT(4));
	if (map->fail)
		return -EIO;
	map->value = (map->value & ~mask) | value;
	return 0;
}
static struct apple_dart dart;
static struct device provider, owner;
static struct regmap gapf;
static int apple_dart_quiesce_commands(struct device *dev)
{
	assert(dev == &provider && dart.sleep_closed && !dart.sleep_writers);
	dart.commands_gated = true;
	return 0;
}

/* Arguments are evaluated: a diagnostic's register reads still happen. */
static inline void dev_log(void *dev, const char *fmt, ...)
{
	(void)dev;
	(void)fmt;
}
#define dev_err(dev, ...) dev_log(dev, __VA_ARGS__)
#define dev_info(dev, ...) dev_log(dev, __VA_ARGS__)
/* ACTUAL_OWNER */

static struct apple_dart_cio state;
static void reset_owner(unsigned int slice, bool partitions)
{
	reset(partitions ? 28 : 256, partitions);
	provider.refs = owner.refs = 0;
	gapf = (struct regmap){ .offset = APPLE_PS_PROT_SLICE(slice),
				.value = BIT(1) | BIT(7) };
	/* PCIe-C's software allocator covers 64 of the 256 hardware SIDs. */
	dart = (struct apple_dart){ .regs = regs,
				    .num_streams = partitions ? 28 : 64,
				    .dev = &provider,
				    .cio = &state };
	state = (struct apple_dart_cio){ .dart = &dart,
					 .owner = &owner,
					 .gapf = &gapf,
					 .slice = slice,
					 .partitions = partitions,
					 .bank = partitions ?
							 0 :
							 DART_CIO_STREAMS };
}
int main(void)
{
	unsigned int cases = 0;
	const unsigned int slices[] = { 14, 15, 18, 19 };

	for (unsigned int i = 0; i < ARRAY_SIZE(slices); i++) {
		reset_owner(slices[i], i % 2);
		for (unsigned int cycle = 0; cycle < 3; cycle++) {
			assert(!apple_dart_cio_prepare(&state) &&
			       provider.refs == 1 && dart.cio_active);
			assert(apple_dart_cio_prepare(&state) == -EBUSY &&
			       provider.refs == 1);
			assert(!apple_dart_cio_off(&state) && state.changed &&
			       !state.restored);
			assert(state.saved.streams == (i % 2 ? 28 : 256));
			assert(dart.commands_gated && dart.sleep_closed &&
			       gapf.value == (BIT(1) | BIT(7)));
			assert(apple_dart_cio_finish(&state) == -EUCLEAN &&
			       dart.cio_active);
			lose_power();
			assert(!apple_dart_cio_on(&state) && state.restored);
			assert(!dart.commands_gated && !dart.sleep_closed);
			assert(gapf.value == (BIT(1) | BIT(4) | BIT(7)));
			check_restored(&state.saved);
			unsigned int before = writes;

			assert(!apple_dart_cio_on(&state) && writes == before);
			assert(apple_dart_cio_off(&state) == -EINVAL);
			assert(!apple_dart_cio_finish(&state) &&
			       !dart.cio_active && !provider.refs);
			assert(apple_dart_cio_on(&state) == -EINVAL);
			assert(apple_dart_cio_finish(&state) == -EINVAL);
			cases++;
		}
	}
	reset_owner(15, true);
	assert(!apple_dart_cio_prepare(&state));
	assert(!apple_dart_cio_on(&state));
	assert(!apple_dart_cio_finish(&state) && !reads && !writes);
	cases++;
	assert(!apple_dart_cio_prepare(&state));
	assert(!apple_dart_sleep_writer_get(&dart));
	assert(apple_dart_cio_off(&state) == -EBUSY && !reads && !writes);
	apple_dart_sleep_writer_put(&dart);
	assert(!apple_dart_cio_finish(&state));
	cases++;
	for (unsigned int fault = 0; fault < 4; fault++) {
		reset_owner(14, false);
		assert(!apple_dart_cio_prepare(&state));
		if (fault == 0)
			gapf.fail_read = 1;
		if (fault == 1)
			gapf.value &= ~BIT(1);
		if (fault == 2)
			busy = 1;
		if (fault == 3)
			regs[2] = 0x204;
		assert(apple_dart_cio_off(&state) < 0);
		assert(!writes && !dart.commands_gated && !dart.sleep_closed &&
		       !state.changed);
		assert(!apple_dart_cio_finish(&state) && !provider.refs);
		cases++;
	}
	reset_owner(19, true);
	assert(!apple_dart_cio_prepare(&state));
	gapf.fail = 1;
	assert(apple_dart_cio_off(&state) == -EIO);
	assert(state.changed && dart.sleep_closed && dart.commands_gated);
	assert(apple_dart_cio_finish(&state) == -EUCLEAN);
	assert(apple_dart_cio_on(&state) == -EIO && !state.restored);
	gapf.fail = 0;
	ntrace = 0;
	assert(!apple_dart_cio_on(&state));
	check_restored(&state.saved);
	assert(!apple_dart_cio_finish(&state) && !provider.refs);
	cases++;
	reset_owner(18, false);
	assert(!apple_dart_cio_prepare(&state));
	assert(!apple_dart_cio_off(&state));
	lose_power();
	drop = 0x1400;
	assert(apple_dart_cio_on(&state) == -EIO && !state.restored);
	assert(dart.sleep_closed && dart.commands_gated && provider.refs == 1);
	assert(apple_dart_cio_finish(&state) == -EUCLEAN);
	drop = ~0U;
	assert(!apple_dart_cio_on(&state) && !apple_dart_cio_finish(&state));
	cases++;
	reset_owner(15, true);
	assert(!apple_dart_cio_prepare(&state));
	assert(!apple_dart_cio_off(&state));
	assert(apple_dart_sleep_writer_get(&dart) == -EBUSY &&
	       dart.cio_violation);
	unsigned int before = reads + writes;

	assert(apple_dart_cio_on(&state) == -EUCLEAN &&
	       reads + writes == before);
	assert(apple_dart_cio_finish(&state) == -EUCLEAN && dart.sleep_closed &&
	       provider.refs == 1);
	cases++;
	printf("PASS %u CIO DART ownership cases\n", cases);
}
