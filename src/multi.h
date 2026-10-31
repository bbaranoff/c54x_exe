/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MULTI_H
#define MULTI_H
/* Runs the jobs of a plan (one c54x_exe invocation per line) on several CPU
 * cores, one process per job. coeurs <= 0: every CPU of the affinity mask;
 * delai > 0: seconds after which a job is killed. */
int multi_executer(const char *plan, int coeurs, const char *sortie, int delai);
#endif
