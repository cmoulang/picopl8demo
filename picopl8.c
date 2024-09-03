#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pio_utils.h"
#include "write_sm.pio.h"

#define CLOCK_MHZ 250

#define ACT_RNW_MASK 0x20
#define ACT_REG_MASK 0xF

static_assert(sizeof(void *) == 4);
static_assert(sizeof(int) == 4);

struct
{
    // 16 byte buffer that can be accessed by the 6502 at 0xB400
    _Alignas(16) char buffer[16];
    unsigned int read_bits;
    unsigned int written_bits;
    int read_count;
    int write_count;
    int maxq;
    int irqn;

    PIO pio_activity_handler;
    uint sm_activity_handler;
} pl8;

/*! \brief Obtain the read bitmask - the bitmask is cleared by this operation.
 *
 * \return the read bitmask
 *
 */
unsigned int get_read_bits()
{
    unsigned int result;
    irq_set_enabled(pl8.irqn, false);
    result = pl8.read_bits;
    pl8.read_bits = 0;
    irq_set_enabled(pl8.irqn, true);
    return result;
}

/*! \brief Obtain the write bitmask - the bitmask is cleared by this operation.
 *
 * \return the read bitmask
 *
 */
unsigned int get_written_bits()
{
    unsigned int result;
    irq_set_enabled(pl8.irqn, false);
    result = pl8.written_bits;
    pl8.written_bits = 0;
    irq_set_enabled(pl8.irqn, true);
    return result;
}

/*! \brief Interrupt handler for handing the 6502 activity notifications.
 *
 * Updates read/written bitmasks and updates the counters.
 *
 */
void pl8_act_handler()
{
    int q = 0;
    while (pio_sm_get_rx_fifo_level(pl8.pio_activity_handler, pl8.sm_activity_handler) > 0)
    {
        q++;
        unsigned int x = pio_sm_get(pl8.pio_activity_handler, pl8.sm_activity_handler);
        int reg = x & ACT_REG_MASK;

        if ((x & ACT_RNW_MASK) == ACT_RNW_MASK)
        {
            // 6502 bus read
            pl8.read_bits |= 1u << reg;
            pl8.read_count++;
        }
        else
        {
            // Write op
            pl8.written_bits |= 1u << reg;
            pl8.write_count++;
        }
    }
    if (q > pl8.maxq)
    {
        pl8.maxq = q;
    }
}

int initialise_6502_PIO()
{
    uint offset;

    for (int gpio = PIN_PA0; gpio <= PIN_NB400; gpio++)
    {
        gpio_init(gpio);
        gpio_set_dir(gpio, false);
    }

    for (int gpio = PIN_D0; gpio <= PIN_D7; gpio++)
    {
        gpio_set_pulls(gpio, true, true);
    }

    // Initialise the activity handler SM
    pio_claim_free_sm_and_add_program(
        &activity_handler_program,
        &pl8.pio_activity_handler,
        &pl8.sm_activity_handler,
        &offset);
    activity_handler_init(pl8.pio_activity_handler, pl8.sm_activity_handler, offset);

    // Setup an interrupt handler for the rx fifo not empty event.
    pio_interrupt_source_t pis = pis_smX_rx_fifo_not_empty(pl8.sm_activity_handler);
    int irqn = PIO_IRQ_NUM(pl8.pio_activity_handler, 1);
    pl8.irqn = irqn;
    printf("cpu_irqn = %d  pio_isrc = %d\n", irqn, pis);
    irq_add_shared_handler(irqn, pl8_act_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(irqn, true);
    pio_set_irq1_source_enabled(pl8.pio_activity_handler, pis, true);

    // Initalise the write addr handler
    PIO pio_write_addr_handler;
    uint sm_write_addr_handler;
    pio_claim_free_sm_and_add_program(
        &write_addr_handler_program,
        &pio_write_addr_handler,
        &sm_write_addr_handler,
        &offset);
    write_addr_handler_program_init(pio_write_addr_handler, sm_write_addr_handler, offset);

    // Initalise the write data handler
    PIO pio_write_data_handler;
    uint sm_write_data_handler;
    pio_claim_free_sm_and_add_program(
        &write_data_handler_program,
        &pio_write_data_handler,
        &sm_write_data_handler,
        &offset);
    write_data_handler_program_init(pio_write_data_handler, sm_write_data_handler, offset);

    PIO pio_read_addr_handler;
    uint sm_read_addr_handler;
    pio_claim_free_sm_and_add_program(
        &read_addr_handler_program,
        &pio_read_addr_handler,
        &sm_read_addr_handler,
        &offset);
    read_addr_handler_program_init(pio_read_addr_handler, sm_read_addr_handler, offset);

    PIO pio_read_data_handler;
    uint sm_read_data_handler;
    pio_claim_free_sm_and_add_program(
        &read_data_handler_program,
        &pio_read_data_handler,
        &sm_read_data_handler,
        &offset);
    read_data_handler_program_init(pio_read_data_handler, sm_read_data_handler, offset);

    uint buffer_address = (uint)&pl8.buffer;
    printf("Buffer starts at: %p\n", buffer_address);
    pio_sm_put(pio_write_addr_handler, sm_write_addr_handler, buffer_address >> 4);
    pio_sm_put(pio_read_addr_handler, sm_read_addr_handler, buffer_address >> 4);

    uint write_addr_chan = dma_claim_unused_channel(true);
    uint write_data_chan = dma_claim_unused_channel(true);
    {
        dma_channel_config c = dma_channel_get_default_config(write_addr_chan);
        channel_config_set_high_priority(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio_write_addr_handler, sm_write_addr_handler, false));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, 0);
        channel_config_set_write_increment(&c, 0);
        channel_config_set_chain_to(&c, write_data_chan);
        dma_channel_configure(
            write_addr_chan,
            &c,
            &dma_channel_hw_addr(write_data_chan)->write_addr,
            &pio_write_addr_handler->rxf[sm_write_addr_handler],
            1,
            true);
    }

    {
        dma_channel_config c = dma_channel_get_default_config(write_data_chan);
        channel_config_set_high_priority(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio_write_data_handler, sm_write_data_handler, false));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_read_increment(&c, 0);
        channel_config_set_write_increment(&c, 0);
        channel_config_set_chain_to(&c, write_addr_chan);
        dma_channel_configure(
            write_data_chan,
            &c,
            NULL,
            &pio_write_data_handler->rxf[sm_write_data_handler],
            1,
            true);
    }

    uint read_addr_chan = dma_claim_unused_channel(true);
    uint read_data_chan = dma_claim_unused_channel(true);

    {
        dma_channel_config c = dma_channel_get_default_config(read_addr_chan);
        channel_config_set_high_priority(&c, true);
        channel_config_set_dreq(&c, pio_get_dreq(pio_read_addr_handler, sm_read_addr_handler, false));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, 0);
        channel_config_set_write_increment(&c, 0);
        channel_config_set_chain_to(&c, read_data_chan);
        dma_channel_configure(
            read_addr_chan,
            &c,
            &dma_channel_hw_addr(read_data_chan)->read_addr,
            &pio_read_addr_handler->rxf[sm_read_addr_handler],
            1,
            true);
    }

    {
        dma_channel_config c = dma_channel_get_default_config(read_data_chan);
        channel_config_set_high_priority(&c, true);
        // channel_config_set_dreq(&c, pio_get_dreq(pio_read_data_handler, sm_write_data_handler, false));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_read_increment(&c, 0);
        channel_config_set_write_increment(&c, 0);
        channel_config_set_chain_to(&c, read_addr_chan);
        dma_channel_configure(
            read_data_chan,
            &c,
            &pio_read_data_handler->txf[sm_read_data_handler],
            NULL, // source
            1,
            false // wait to be triggered
        );
    }
}

/*! \brief Measure the frequncy of a digital signal on a gpio input.
 *
 * \param gpio GPIO number
 *
 * Returns the average frequncy in Hz of the signal sampled during the call.
 *
 * \return the frequncy of the signal
 *
 */
int measure_freq(int gpio)
{
    const int sample_count = 10000;
    int count = 0;
    int prev_val = gpio_get(gpio);

    absolute_time_t start_time2 = get_absolute_time();
    for (int i = 0; i < sample_count; i++)
    {
        int current_val = gpio_get(gpio);
        if (current_val != prev_val)
        {
            prev_val = current_val;
            count++;
        }
    }
    absolute_time_t end_time2 = get_absolute_time();
    int elapsed_us = absolute_time_diff_us(start_time2, end_time2);

    int freq = (count * 500000) / elapsed_us;
    return freq;
}

void simple_demo()
{
    int count;
    printf("\nAcorn Atom interface demo " __DATE__ " " __TIME__ "\n");
    int freq = measure_freq(PIN_1MHZ);
    printf("6502 clock = %.2f MHz\n", freq / 1000000.0);


    while (true)
    {
        sleep_ms(1000);
        uint32_t rbits = get_read_bits();
        uint32_t wbits = get_written_bits();
        if (rbits || wbits)
        {
            printf("PL8 Status\nData ");
            for (int i = 15; i >= 0; i--)
            {
                printf("%2x ", pl8.buffer[i]);
            }
            printf("\nRead ");
            for (int i = 15; i >= 0; i--)
            {
                printf("%2x ", !!(rbits & 1u << i));
            }
            printf("\nWrit ");
            for (int i = 15; i >= 0; i--)
            {
                printf("%2x ", !!(wbits & 1u << i));
            }
            printf("\n");
            printf("Read: %d Written: %d Max queue: %d\n", pl8.read_count, pl8.write_count, pl8.maxq);
        }
    }
}

int main()
{
    bool clock_ok = set_sys_clock_khz(CLOCK_MHZ * 1000, true);
    stdio_init_all();
    initialise_6502_PIO();

    // The interface is now running on the PIOs and DMAs 
    // so the cpu is free to do anything it wants.
    simple_demo();
}