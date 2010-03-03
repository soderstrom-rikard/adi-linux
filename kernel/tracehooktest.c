/*
 * Self tests for the misc pieces of the tracehook code.
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ptrace.h>

#include <asm/syscall.h>

static int __init syscall_test(void)
{
	struct pt_regs _regs, *regs = &_regs;
	long _args[8];
	long *args = _args + 1;
	long *sys_args[6], *sa;
	unsigned int i, n, s;

	/*
	 * First find each system register in pt_regs.  We have to assume
	 * syscall_set_arguments() works with very basic arguments.
	 */
	pr_info("TEST: asm/syscall.h: arg offsets: { ");

	for (s = 0; s < 6; ++s)
		args[s] = s;
	memset(regs, 0xad, sizeof(*regs));
	syscall_set_arguments(NULL, regs, 0, 6, args);

	for (s = 0; s < 6; ++s) {
		for (sa = (long *)regs; sa < (long *)(regs + 1); ++sa)
			if (memcmp(&s, sa, sizeof(s)) == 0)
				break;
		if (sa == (long *)(regs + 1)) {
			pr_cont(" FAIL (couldn't locate sys arg %u)\n", s);
			return 1;
		}
		sys_args[s] = sa;
		pr_cont("%li ", (unsigned long)sa - (unsigned long)regs);
	}
	pr_cont("}: PASS\n");

	/* Make sure syscall_get_arguments() works with basic args */
	pr_info("TEST: asm/syscall.h: basic syscall_get_arguments(): ");
	memset(regs, 0xad, sizeof(*regs));
	for (s = 0; s < 6; ++s)
		*sys_args[s] = s;
	syscall_get_arguments(NULL, regs, 0, 6, args);
	for (s = 0; s < 6; ++s)
		if (args[s] != s) {
			pr_cont("FAIL with arg %i (%li)\n", s, args[s]);
			return 1;
		}
	pr_cont("PASS\n");

	/* Now brute force all values of i/n */
	pr_info("TEST: asm/syscall.h: all i/n get/set combos: ");
	for (i = 0; i < 6; ++i) {
		for (n = 0; n < 7 - i; ++n) {

			/* Seed for syscall_get_arguments() test */
			for (s = 0; s < 6; ++s)
				*sys_args[s] = -(s + 1);
			memset(_args, 0, sizeof(_args));
			syscall_get_arguments(NULL, regs, i, n, args);

			/* Check canaries */
			if (_args[0] != 0 || _args[7] != 0)
				goto abort_i_n_get_tests;

			/* Check all system argument values */
			for (s = i; s < i + n; ++s)
				if (*sys_args[s] != args[s - i])
					goto abort_i_n_get_tests;

			/* Check all unused slots */
			for (s = i + n; s < 7; ++s)
				if (args[s] != 0)
					goto abort_i_n_get_tests;

			/* Seed for syscall_set_arguments() test */
			for (s = 0; s < 6; ++s)
				args[s] = -(s + 1);
			memset(regs, 0, sizeof(*regs));
			syscall_set_arguments(NULL, regs, i, n, args);

			/* Check the entire register set */
			for (sa = (long *)regs; sa < (long *)(regs + 1); ++sa) {
				if (*sa == 0)
					continue;
				/* Only some args should be set */
				for (s = i; s < i + n; ++s)
					if (sa == sys_args[s])
						break;
				if (s == i + n)
					goto abort_i_n_set_tests;
			}

			/* Check the valid system argument values */
			for (s = i; s < i + n; ++s)
				if (*sys_args[s] != -(s + 1 - i))
					goto abort_i_n_set_tests;
		}
	}
	pr_cont("PASS\n");

	return 0;

 abort_i_n_get_tests:
	pr_cont("FAIL (get @ i=%u n=%u)\n", i, n);
	return 1;
 abort_i_n_set_tests:
	pr_cont("FAIL (set @ i=%u n=%u)\n", i, n);
	return 1;
}

static int __init tracehooktest_init(void)
{
	return syscall_test();
}
module_init(tracehooktest_init);

static void __exit tracehooktest_exit(void)
{
	/* stub to unload */
}
module_exit(tracehooktest_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Frysinger <vapier@gentoo.org>");
