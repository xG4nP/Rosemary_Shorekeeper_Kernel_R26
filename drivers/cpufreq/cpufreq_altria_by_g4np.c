/*
 *  drivers/cpufreq/cpufreq_altria_by_g4np.c
 *
 *  Copyright (C)  2025 Patrick Ricafrente<patrick.ricafrente27@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include "cpufreq_governor.h"

struct cs_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int down_skip;
	unsigned int requested_freq;
};

static inline struct cs_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
	return container_of(policy_dbs, struct cs_policy_dbs_info, policy_dbs);
}

struct cs_dbs_tuners {
	unsigned int down_threshold;
	unsigned int freq_step;
};

/* altria_by_g4np governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD		(86)
#define DEF_FREQUENCY_DOWN_THRESHOLD		(24)
#define DEF_FREQUENCY_STEP			(7)
#define DEF_SAMPLING_DOWN_FACTOR		(3)
#define MAX_SAMPLING_DOWN_FACTOR		(100)

static inline unsigned int get_freq_step(struct cs_dbs_tuners *cs_tuners,
					 struct cpufreq_policy *policy)
{
	unsigned int freq_step = (cs_tuners->freq_step * policy->max) / 100;


	if (unlikely(freq_step == 0))
		freq_step = DEF_FREQUENCY_STEP;

	return freq_step;
}

static unsigned int cs_dbs_update(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct cs_policy_dbs_info *dbs_info = to_dbs_info(policy_dbs);
	unsigned int requested_freq = dbs_info->requested_freq;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int load = dbs_update(policy);
	unsigned int freq_step;


	if (cs_tuners->freq_step == 0)
		goto out;


	if (requested_freq > policy->max || requested_freq < policy->min) {
		requested_freq = policy->cur;
		dbs_info->requested_freq = requested_freq;
	}

	freq_step = get_freq_step(cs_tuners, policy);


	if (policy_dbs->idle_periods < UINT_MAX) {
		unsigned int freq_steps = policy_dbs->idle_periods * freq_step;

		if (requested_freq > policy->min + freq_steps)
			requested_freq -= freq_steps;
		else
			requested_freq = policy->min;

		policy_dbs->idle_periods = UINT_MAX;
	}


	if (load > dbs_data->up_threshold) {
		dbs_info->down_skip = 0;


		if (requested_freq == policy->max)
			goto out;

		requested_freq += freq_step;
		if (requested_freq > policy->max)
			requested_freq = policy->max;

		__cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_H);
		dbs_info->requested_freq = requested_freq;
		goto out;
	}


	if (++dbs_info->down_skip < dbs_data->sampling_down_factor)
		goto out;
	dbs_info->down_skip = 0;


	if (load < cs_tuners->down_threshold) {

		if (requested_freq == policy->min)
			goto out;

		if (requested_freq > freq_step)
			requested_freq -= freq_step;
		else
			requested_freq = policy->min;

		__cpufreq_driver_target(policy, requested_freq, CPUFREQ_RELATION_L);
		dbs_info->requested_freq = requested_freq;
	}

 out:
	return dbs_data->sampling_rate;
}


static ssize_t store_sampling_down_factor(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_data->sampling_down_factor = input;
	return count;
}

static ssize_t store_up_threshold(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 || input <= cs_tuners->down_threshold)
		return -EINVAL;

	dbs_data->up_threshold = input;
	return count;
}

static ssize_t store_down_threshold(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);


	if (ret != 1 || input < 1 || input > 100 ||
			input >= dbs_data->up_threshold)
		return -EINVAL;

	cs_tuners->down_threshold = input;
	return count;
}

static ssize_t store_ignore_nice_load(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_data->ignore_nice_load)
		return count;

	dbs_data->ignore_nice_load = input;


	gov_update_cpu_data(dbs_data);

	return count;
}

static ssize_t store_freq_step(struct gov_attr_set *attr_set, const char *buf,
			       size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;


	cs_tuners->freq_step = input;
	return count;
}

gov_show_one_common(sampling_rate);
gov_show_one_common(sampling_down_factor);
gov_show_one_common(up_threshold);
gov_show_one_common(ignore_nice_load);
gov_show_one(cs, down_threshold);
gov_show_one(cs, freq_step);

gov_attr_rw(sampling_rate);
gov_attr_rw(sampling_down_factor);
gov_attr_rw(up_threshold);
gov_attr_rw(ignore_nice_load);
gov_attr_rw(down_threshold);
gov_attr_rw(freq_step);

static struct attribute *cs_attributes[] = {
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&up_threshold.attr,
	&down_threshold.attr,
	&ignore_nice_load.attr,
	&freq_step.attr,
	NULL
};



static struct policy_dbs_info *cs_alloc(void)
{
	struct cs_policy_dbs_info *dbs_info;

	dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
	return dbs_info ? &dbs_info->policy_dbs : NULL;
}

static void cs_free(struct policy_dbs_info *policy_dbs)
{
	kfree(to_dbs_info(policy_dbs));
}

static int cs_init(struct dbs_data *dbs_data)
{
	struct cs_dbs_tuners *tuners;

	tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners)
		return -ENOMEM;

	tuners->down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD;
	tuners->freq_step = DEF_FREQUENCY_STEP;
	dbs_data->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
	dbs_data->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	dbs_data->ignore_nice_load = 0;
	dbs_data->tuners = tuners;

	return 0;
}

static void cs_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

static void cs_start(struct cpufreq_policy *policy)
{
	struct cs_policy_dbs_info *dbs_info = to_dbs_info(policy->governor_data);

	dbs_info->down_skip = 0;
	dbs_info->requested_freq = policy->cur;
}

static struct dbs_governor cs_governor = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("altria_by_g4np"),
	.kobj_type = { .default_attrs = cs_attributes },
	.gov_dbs_update = cs_dbs_update,
	.alloc = cs_alloc,
	.free = cs_free,
	.init = cs_init,
	.exit = cs_exit,
	.start = cs_start,
};

#define CPU_FREQ_GOV_ALTRIA_BY_G4NP	(&cs_governor.gov)

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(CPU_FREQ_GOV_ALTRIA_BY_G4NP);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(CPU_FREQ_GOV_ALTRIA_BY_G4NP);
}

MODULE_AUTHOR("Patrick Ricafrente <patrick.ricafrente27@gmail.com>");
MODULE_DESCRIPTION("just a simple gov for better experience");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ALTRIA_BY_G4NP
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return CPU_FREQ_GOV_ALTRIA_BY_G4NP;
}

fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
