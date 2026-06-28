/* Route A — M1 standalone bare-metal smoke test.
 * Confirms FSBL -> PL(bitstream) -> app handoff and PS7/DDR/UART init on the Zybo Z7-20,
 * with NO PYNQ/Linux. Prints a banner + heartbeat over the PS UART (115200 8N1).
 * Once this boots from SD, M2 adds SCCB chip-ID over the PL sccb_gpio. */
#include "xil_printf.h"
#include "xil_types.h"
#include "sleep.h"

int main(void)
{
    xil_printf("\r\n");
    xil_printf("============================================================\r\n");
    xil_printf(" Route A  M1: bare-metal boot OK (FSBL -> PL -> app)\r\n");
    xil_printf(" MIPI2HDML standalone, no PYNQ. Heartbeat below.\r\n");
    xil_printf("============================================================\r\n");

    u32 n = 0;
    for (;;) {
        xil_printf("alive %u\r\n", (unsigned)n++);
        sleep(1);   /* BSP sleep() = seconds */
    }
    return 0;
}
