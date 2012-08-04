/*
 *  linux/arch/arm/mach-oxnas/irq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>

#ifdef CONFIG_OXNAS_DDR_MON
static int client = 0;
#endif // CONFIG_OXNAS_DDR_MON

static irqreturn_t OXNAS_timer_interrupt(int irq, void *dev_id)
{
#ifdef CONFIG_OXNAS_DDR_MON
    static const int NUM_MON_CLIENTS = 8;
#endif // CONFIG_OXNAS_DDR_MON

    // Clear the timer interrupt - any write will do
    *((volatile unsigned long*)TIMER1_CLEAR) = 0;

    timer_tick();

#ifdef CONFIG_OXNAS_DDR_MON
    if (!(jiffies % CONFIG_OXNAS_MONITOR_SUBSAMPLE)) {
        // Read the DDR core bus monitors
        u32 diag_reg_contents = readl(DDR_DIAG_REG);
        u32 holdoffs = (diag_reg_contents >> DDR_DIAG_HOLDOFFS_BIT) & ((1UL << DDR_DIAG_HOLDOFFS_NUM_BITS) - 1);
        u32 writes   = (diag_reg_contents >> DDR_DIAG_WRITES_BIT)   & ((1UL << DDR_DIAG_WRITES_NUM_BITS) - 1);
        u32 reads    = (diag_reg_contents >> DDR_DIAG_READS_BIT)    & ((1UL << DDR_DIAG_READS_NUM_BITS) - 1);
    
        printk(KERN_INFO "$WC %d: H=%u, W=%u, R=%u\n", client, holdoffs, writes, reads);
        // Re-arm the DDR core bus monitors
        writel(client << DDR_MON_CLIENT_BIT, DDR_MON_REG);
        if (++client >= NUM_MON_CLIENTS) {
            client = 0;
        }
    }
#endif // CONFIG_OXNAS_DDR_MON

    return IRQ_HANDLED;
}

static struct irqaction oxnas_timer_irq = {
    .name    = "Jiffy tick",
	.flags	 = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .handler = OXNAS_timer_interrupt
};

static void /*__init*/ oxnas_init_time(void)
{
    // Connect the timer interrupt handler
    oxnas_timer_irq.handler = OXNAS_timer_interrupt;
    setup_irq(TIMER_1_INTERRUPT, &oxnas_timer_irq);

    // Stop both timers before programming them
    *((volatile unsigned long*)TIMER1_CONTROL) = 0;
    *((volatile unsigned long*)TIMER2_CONTROL) = 0;

    // Setup timer 1 load value
    *((volatile unsigned long*)TIMER1_LOAD) = TIMER_1_LOAD_VALUE;

    // Setup timer 1 prescaler, periodic operation and start it
    *((volatile unsigned long*)TIMER1_CONTROL) =
        (TIMER_1_PRESCALE_ENUM << TIMER_PRESCALE_BIT) |
        (TIMER_1_MODE          << TIMER_MODE_BIT)     |
        (TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT);

    // Setup timer 2 prescaler, free-running operation and start it
    // This will not be used to generate interrupt, just as a hi-res source of
    // timing information
    *((volatile unsigned long*)TIMER2_CONTROL) =
        (TIMER_2_PRESCALE_ENUM << TIMER_PRESCALE_BIT) |
        (TIMER_2_MODE          << TIMER_MODE_BIT)     |
        (TIMER_ENABLE_ENABLE   << TIMER_ENABLE_BIT);

#ifdef CONFIG_OXNAS_DDR_MON
    // Arm the DDR core bus monitors, start with client zero
    writel(client << DDR_MON_CLIENT_BIT, DDR_MON_REG);
#endif // CONFIG_OXNAS_DDR_MON
}

/*
 * Returns number of microseconds since last clock tick interrupt.
 * Note that interrupts will be disabled when this is called
 * Should take account of any pending timer tick interrupt
 */
static unsigned long oxnas_gettimeoffset(void)
{
	// How long since last timer interrupt?
    unsigned long ticks_since_last_intr =
		(unsigned long)TIMER_1_LOAD_VALUE - *((volatile unsigned long*)TIMER1_VALUE);

    // Is there a timer interrupt pending
    int timer_int_pending =
		*((volatile unsigned long*)RPS_IRQ_RAW_STATUS) & (1UL << TIMER_1_INTERRUPT);

    if (timer_int_pending) {
		// Sample time since last timer interrupt again. Theoretical race between
		// interrupt occuring and ARM reading value before reload has taken
		// effect, but in practice it's not going to happen because it takes
		// multiple clock cycles for the ARM to read the timer value register
		unsigned long ticks2 = (unsigned long)TIMER_1_LOAD_VALUE - *((volatile unsigned long*)TIMER1_VALUE);

		// If the timer interrupt which hasn't yet been serviced, and thus has
		// not yet contributed to the tick count, occured before our initial
		// read of the current timer value then we need to account for a whole
		// timer interrupt period
		if (ticks_since_last_intr <= ticks2) {
			// Add on a whole timer interrupt period, as the tick count will have
			// wrapped around since the previously seen timer interrupt (?)
			ticks_since_last_intr += TIMER_1_LOAD_VALUE;
		}
    }

    return TICKS_TO_US(ticks_since_last_intr);
}

struct sys_timer oxnas_timer = {
    .init   = oxnas_init_time,
    .offset = oxnas_gettimeoffset,
};
