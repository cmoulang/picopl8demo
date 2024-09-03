#pragma once

#include "hardware/pio.h"

/*! \brief Lookup the GPIO_FUNC_PIOn constant for the given PIO 
 *
 * \param pio the PIO
 * \return -ve if error or GPIO_FUNC_PIO0, GPIO_FUNC_PIO1 or GPIO_FUNC_PIO2
 */
static const int piou_gpio_func(PIO pio)
{
    int gpio_func;
    if (pio == pio0)
    {
        gpio_func = GPIO_FUNC_PIO0;
    }
    else if (pio == pio1)
    {
        gpio_func = GPIO_FUNC_PIO1;
    }
#ifdef PICO_RP2350
    else if (pio == pio2)
    {
        gpio_func = GPIO_FUNC_PIO2;
    }
#endif
    else
    {
        gpio_func=-1;
    }
    return gpio_func;
}

/*! \brief Lookup the rx fifo not empty pio interrupt source enum for the given state machine
 *
 * \param sm state machine index 0..3
 * \return the appropriate enum: pis_sm0_rx_fifo_not_empty..pis_sm3_rx_fifo_not_empty
*/
static const int pis_smX_rx_fifo_not_empty(int sm)
{
    static_assert(pis_sm3_rx_fifo_not_empty == pis_sm0_rx_fifo_not_empty+3);
    static_assert(pis_sm2_rx_fifo_not_empty == pis_sm0_rx_fifo_not_empty+2);
    static_assert(pis_sm1_rx_fifo_not_empty == pis_sm0_rx_fifo_not_empty+1);
    return pis_sm0_rx_fifo_not_empty + sm;
}
