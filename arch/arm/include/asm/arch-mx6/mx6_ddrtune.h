#ifndef _MX6_DDRTUNE

int mx6_ddr_init(ulong addr);
int mx6_do_tune_mtest(int iteration_limit);
void mx6_ddr_debug(void);
#endif
