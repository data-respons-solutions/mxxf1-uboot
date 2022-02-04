#include <stdio.h>

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_MX6_DDRCAL)

#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-ddr.h>

void display_calibration(struct mx6_ddr_sysinfo *sysinfo, struct mx6_mmdc_calibration *calib)
{
	printf(".p0_mpdgctrl0\t= 0x%08X\n", calib->p0_mpdgctrl0);
	printf(".p0_mpdgctrl1\t= 0x%08X\n", calib->p0_mpdgctrl1);
	printf(".p0_mprddlctl\t= 0x%08X\n", calib->p0_mprddlctl);
	printf(".p0_mpwrdlctl\t= 0x%08X\n", calib->p0_mpwrdlctl);
	printf(".p0_mpwldectrl0\t= 0x%08X\n", calib->p0_mpwldectrl0);
	printf(".p0_mpwldectrl1\t= 0x%08X\n", calib->p0_mpwldectrl1);
	if (sysinfo->dsize == 2) {
		printf(".p1_mpdgctrl0\t= 0x%08X\n", calib->p1_mpdgctrl0);
		printf(".p1_mpdgctrl1\t= 0x%08X\n", calib->p1_mpdgctrl1);
		printf(".p1_mprddlctl\t= 0x%08X\n", calib->p1_mprddlctl);
		printf(".p1_mpwrdlctl\t= 0x%08X\n", calib->p1_mpwrdlctl);
		printf(".p1_mpwldectrl0\t= 0x%08X\n", calib->p1_mpwldectrl0);
		printf(".p1_mpwldectrl1\t= 0x%08X\n", calib->p1_mpwldectrl1);
	}
#ifdef CONFIG_IMXIMAGE_OUTPUT
	printf("DATA 4 MX6_MMDC_P0_MPDGCTRL0\t= 0x%08X\n", calib->p0_mpdgctrl0);
	printf("DATA 4 MX6_MMDC_P0_MPDGCTRL1\t= 0x%08X\n", calib->p0_mpdgctrl1);
	printf("DATA 4 MX6_MMDC_P0_MPRDDLCTL\t= 0x%08X\n", calib->p0_mprddlctl);
	printf("DATA 4 MX6_MMDC_P0_MPWRDLCTL\t= 0x%08X\n", calib->p0_mpwrdlctl);
	printf("DATA 4 MX6_MMDC_P0_MPWLDECTRL0\t= 0x%08X\n",
	       calib->p0_mpwldectrl0);
	printf("DATA 4 MX6_MMDC_P0_MPWLDECTRL1\t= 0x%08X\n",
	       calib->p0_mpwldectrl1);
	if (sysinfo->dsize == 2) {
		printf("DATA 4 MX6_MMDC_P1_MPDGCTRL0\t= 0x%08X\n",
		       calib->p1_mpdgctrl0);
		printf("DATA 4 MX6_MMDC_P1_MPDGCTRL1\t= 0x%08X\n",
		       calib->p1_mpdgctrl1);
		printf("DATA 4 MX6_MMDC_P1_MPRDDLCTL\t= 0x%08X\n",
		       calib->p1_mprddlctl);
		printf("DATA 4 MX6_MMDC_P1_MPWRDLCTL\t= 0x%08X\n",
		       calib->p1_mpwrdlctl);
		printf("DATA 4 MX6_MMDC_P1_MPWLDECTRL0\t= 0x%08X\n",
		       calib->p1_mpwldectrl0);
		printf("DATA 4 MX6_MMDC_P1_MPWLDECTRL1\t= 0x%08X\n",
		       calib->p1_mpwldectrl1);
	}
#endif
}
#endif
