/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/** @file
@brief Implementation of the ACI transport layer module
*/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Atmel library includes. */
#include "asf.h"

#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include <SPI.h>
#include "hal_platform.h"
#include "hal_aci_tl.h"
#include "SEGGER_RTT.h"
#include "aci_queue.h"
#if ( !defined(__SAM3X8E__) && !defined(__PIC32MX__) )
#include <avr/sleep.h>
#endif
/*
PIC32 supports only MSbit transfer on SPI and the nRF8001 uses LSBit
Use the REVERSE_BITS macro to convert from MSBit to LSBit
The outgoing command and the incoming event needs to be converted
*/
//Board dependent defines
#if defined (__AVR__)
    //For Arduino add nothing
#elif defined(__PIC32MX__)
    //For ChipKit as the transmission has to be reversed, the next definitions have to be added
    #define REVERSE_BITS(byte) (((reverse_lookup[(byte & 0x0F)]) << 4) + reverse_lookup[((byte & 0xF0) >> 4)])
    static const uint8_t reverse_lookup[] = { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };
#elif defined(__SAM3X8E__)
    #define REVERSE_BITS(byte) (((reverse_lookup[(byte & 0x0F)]) << 4) + reverse_lookup[((byte & 0xF0) >> 4)])
    static const uint8_t reverse_lookup[] = { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };

	/* Chip select. */
	#define SPI_CHIP_SEL 0
	#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

	/* Clock polarity. */
	#define SPI_CLK_POLARITY 0

	/* Clock phase. */
	#define SPI_CLK_PHASE 1
#endif

static void m_aci_data_print(hal_aci_data_t *p_data);
static void m_aci_event_check(void);
static void m_aci_isr(void);
static void m_aci_isr_hal(uint32_t id, uint32_t mask);
static void m_aci_pins_set(aci_pins_t *a_pins_ptr);
static inline void m_aci_reqn_disable (void);
static inline void m_aci_reqn_enable (void);
static void m_aci_q_flush(void);
static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data);

static uint8_t        spi_readwrite(uint8_t aci_byte);

static bool           aci_debug_print = false;

aci_queue_t    aci_tx_q;
aci_queue_t    aci_rx_q;

static aci_pins_t	 *a_pins_local_ptr;

static xSemaphoreHandle xSemaphoreIsr;

void m_aci_data_print(hal_aci_data_t *p_data)
{
  const uint8_t length = p_data->buffer[0];
  uint8_t i;
  SEGGER_RTT_printf(0, "%02d: ", length);
  if (length <= 31) {
	  for (i=0; i<=length; i++)
	  {
		SEGGER_RTT_printf(0, "%02x, ", p_data->buffer[i]);
	  }
  }
  SEGGER_RTT_printf(0, "\n");
}

/*
  Interrupt service routine called when the RDYN line goes low. Runs the SPI transfer.
*/
static void m_aci_isr(void)
{
  hal_aci_data_t data_to_send;
  hal_aci_data_t received_data;

  // Receive from queue
  if (!aci_queue_dequeue_from_isr(&aci_tx_q, &data_to_send))
  {
    /* queue was empty, nothing to send */
    data_to_send.status_byte = 0;
    data_to_send.buffer[0] = 0;
  }

  // Receive and/or transmit data
  m_aci_spi_transfer(&data_to_send, &received_data);

  if (!aci_queue_is_full_from_isr(&aci_rx_q) && !aci_queue_is_empty_from_isr(&aci_tx_q))
  {
    m_aci_reqn_enable();
  }

  // Check if we received data
  if (received_data.buffer[0] > 0)
  {
    if (!aci_queue_enqueue_from_isr(&aci_rx_q, &received_data))
    {
      /* Receive Buffer full.
         Should never happen.
         Spin in a while loop.
      */
      while(1);
    }

    // Disable ready line interrupt until we have room to store incoming messages
    if (aci_queue_is_full_from_isr(&aci_rx_q))
    {
      //detachInterrupt(a_pins_local_ptr->interrupt_number);
	  pio_disable_pin_interrupt(a_pins_local_ptr->rdyn_pin);
    }
  }

  return;
}

/*
  Checks the RDYN line and runs the SPI transfer if required.
*/
static void m_aci_event_check(void)
{
  hal_aci_data_t data_to_send;
  hal_aci_data_t received_data;

  // No room to store incoming messages
  if (aci_queue_is_full(&aci_rx_q))
  {
    return;
  }

  // If the ready line is disabled and we have pending messages outgoing we enable the request line
  if (HIGH == ioport_get_pin_level(a_pins_local_ptr->rdyn_pin))
  {
    if (!aci_queue_is_empty(&aci_tx_q))
    {
      m_aci_reqn_enable();
    }

    return;
  }

  // Receive from queue
  if (!aci_queue_dequeue(&aci_tx_q, &data_to_send))
  {
    /* queue was empty, nothing to send */
    data_to_send.status_byte = 0;
    data_to_send.buffer[0] = 0;
  }

  // Receive and/or transmit data
  m_aci_spi_transfer(&data_to_send, &received_data);

  /* If there are messages to transmit, and we can store the reply, we request a new transfer */
  if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
  {
    m_aci_reqn_enable();
  }

  // Check if we received data
  if (received_data.buffer[0] > 0)
  {
    if (!aci_queue_enqueue(&aci_rx_q, &received_data))
    {
      /* Receive Buffer full.
         Should never happen.
         Spin in a while loop.
      */
      while(1);
    }
  }

  return;
}

/** @brief Point the low level library at the ACI pins specified
 *  @details
 *  The ACI pins are specified in the application and a pointer is made available for
 *  the low level library to use
 */
static void m_aci_pins_set(aci_pins_t *a_pins_ptr)
{
  a_pins_local_ptr = a_pins_ptr;
}

static inline void m_aci_reqn_disable (void)
{
  ioport_set_pin_level(a_pins_local_ptr->reqn_pin, 1);
}

static inline void m_aci_reqn_enable (void)
{
  ioport_set_pin_level(a_pins_local_ptr->reqn_pin, 0);
}

static void m_aci_q_flush(void)
{
  noInterrupts();
  /* re-initialize aci cmd queue and aci event queue to flush them*/
  aci_queue_init(&aci_tx_q);
  aci_queue_init(&aci_rx_q);
  interrupts();
}

static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data)
{
  uint8_t byte_cnt;
  uint8_t byte_sent_cnt;
  uint8_t max_bytes;

  m_aci_reqn_enable();

  // Send length, receive header
  byte_sent_cnt = 0;
  received_data->status_byte = spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  // Send first byte, receive length from slave
  received_data->buffer[0] = spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  if (0 == data_to_send->buffer[0])
  {
    max_bytes = received_data->buffer[0];
  }
  else
  {
    // Set the maximum to the biggest size. One command byte is already sent
    max_bytes = (received_data->buffer[0] > (data_to_send->buffer[0] - 1))
                                          ? received_data->buffer[0]
                                          : (data_to_send->buffer[0] - 1);
  }

  if (max_bytes > HAL_ACI_MAX_LENGTH)
  {
    max_bytes = HAL_ACI_MAX_LENGTH;
  }

  // Transmit/receive the rest of the packet
  for (byte_cnt = 0; byte_cnt < max_bytes; byte_cnt++)
  {
    received_data->buffer[byte_cnt+1] =  spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  }

  // RDYN should follow the REQN line in approx 100ns
  m_aci_reqn_disable();

  return (max_bytes > 0);
}

void hal_aci_tl_debug_print(bool enable)
{
	aci_debug_print = enable;
}

void hal_aci_tl_pin_reset(void)
{
    if (PIN_UNUSED != a_pins_local_ptr->reset_pin)
    {
		ioport_set_pin_dir(a_pins_local_ptr->reset_pin, IOPORT_DIR_OUTPUT);

        if ((REDBEARLAB_SHIELD_V1_1     == a_pins_local_ptr->board_name) ||
            (REDBEARLAB_SHIELD_V2012_07 == a_pins_local_ptr->board_name))
        {
            //The reset for the Redbearlab v1.1 and v2012.07 boards are inverted and has a Power On Reset
            //circuit that takes about 100ms to trigger the reset
            ioport_set_pin_level(a_pins_local_ptr->reset_pin, 1);
            delay(100);
            ioport_set_pin_level(a_pins_local_ptr->reset_pin, 0);
        }
        else
        {
            ioport_set_pin_level(a_pins_local_ptr->reset_pin, 1);
            ioport_set_pin_level(a_pins_local_ptr->reset_pin, 0);
            ioport_set_pin_level(a_pins_local_ptr->reset_pin, 1);
        }
    }
}

bool hal_aci_tl_event_peek(hal_aci_data_t *p_aci_data)
{
  if (!a_pins_local_ptr->interface_is_interrupt)
  {
    m_aci_event_check();
  }

  if (aci_queue_peek(&aci_rx_q, p_aci_data))
  {
    return true;
  }

  return false;
}

bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data)
{
  bool was_full;

  if (!a_pins_local_ptr->interface_is_interrupt && !aci_queue_is_full(&aci_rx_q))
  {
    m_aci_event_check();
  }

  was_full = aci_queue_is_full(&aci_rx_q);

  if (aci_queue_dequeue(&aci_rx_q, p_aci_data))
  {
    if (aci_debug_print)
    {
	  taskENTER_CRITICAL();
      SEGGER_RTT_printf(0, "E");
      m_aci_data_print(p_aci_data);
	  taskEXIT_CRITICAL();
    }

    if (was_full && a_pins_local_ptr->interface_is_interrupt)
	  {
      /* Enable RDY line interrupt again */
      //attachInterrupt(a_pins_local_ptr->interrupt_number, m_aci_isr, LOW);
	  //pio_handler_set_pin(PIO_PC22_IDX, PIO_IT_LOW_LEVEL, m_aci_isr_hal);
	  pio_enable_pin_interrupt(a_pins_local_ptr->rdyn_pin);
    }

    /* Attempt to pull REQN LOW since we've made room for new messages */
    if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
    {
      m_aci_reqn_enable();
    }

    return true;
  }

  return false;
}

void hal_aci_tl_init(aci_pins_t *a_pins, bool debug)
{
  aci_debug_print = debug;

  /* Needs to be called as the first thing for proper intialization*/
  m_aci_pins_set(a_pins);
  
  xSemaphoreIsr = xSemaphoreCreateBinary();
  xTaskCreate(hal_aci_tl_isr_handler_task, "HalAciIsrHdl", 240, NULL, 3, NULL);

  /*
  The SPI lines used are mapped directly to the hardware SPI
  MISO MOSI and SCK
  Change here if the pins are mapped differently

  The SPI library assumes that the hardware pins are used
  */
  //SPI.begin();
	spi_enable_clock(SPI0);
	spi_disable(SPI0);
	spi_reset(SPI0);
	spi_set_lastxfer(SPI0);
	spi_set_master_mode(SPI0);
	spi_disable_mode_fault_detect(SPI0);
	spi_set_peripheral_chip_select_value(SPI0, SPI_CHIP_PCS);
	spi_configure_cs_behavior(SPI0, SPI_CHIP_SEL, SPI_CS_KEEP_LOW);
	spi_set_clock_polarity(SPI0, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI0, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI0, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI0, SPI_CHIP_SEL, (sysclk_get_cpu_hz() / 2625000));
	spi_set_transfer_delay(SPI0, SPI_CHIP_SEL, 0, 1);
	spi_enable(SPI0);
  
  //Board dependent defines
  #if defined (__AVR__)
    //For Arduino use the LSB first
    SPI.setBitOrder(LSBFIRST);
  #elif defined(__PIC32MX__)
    //For ChipKit use MSBFIRST and REVERSE the bits on the SPI as LSBFIRST is not supported
    SPI.setBitOrder(MSBFIRST);
  #endif
  //SPI.setClockDivider(a_pins->spi_clock_divider);
  //SPI.setDataMode(SPI_MODE_0);

  /* Initialize the ACI Command queue. This must be called after the delay above. */
  aci_queue_init(&aci_tx_q);
  aci_queue_init(&aci_rx_q);

  //Configure the IO lines
  ioport_set_pin_mode(a_pins->rdyn_pin,		IOPORT_MODE_PULLUP);
  ioport_set_pin_dir(a_pins->rdyn_pin,		IOPORT_DIR_INPUT);
  ioport_set_pin_dir(a_pins->reqn_pin,		IOPORT_DIR_OUTPUT);

  if (PIN_UNUSED != a_pins->active_pin)
  {
	ioport_set_pin_mode(a_pins->active_pin, 0);
	ioport_set_pin_dir(a_pins->active_pin, IOPORT_DIR_INPUT);
  }
  /* Pin reset the nRF8001, required when the nRF8001 setup is being changed */
  hal_aci_tl_pin_reset();

  /* Set the nRF8001 to a known state as required by the datasheet*/
  ioport_set_pin_level(a_pins->miso_pin, 0);
  ioport_set_pin_level(a_pins->mosi_pin, 0);
  ioport_set_pin_level(a_pins->reqn_pin, 1);
  ioport_set_pin_level(a_pins->sck_pin,  0);

  delay(30); //Wait for the nRF8001 to get hold of its lines - the lines float for a few ms after the reset

  /* Attach the interrupt to the RDYN line as requested by the caller */
  if (a_pins->interface_is_interrupt)
  {
    // We use the LOW level of the RDYN line as the atmega328 can wakeup from sleep only on LOW
	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);

	UNUSED(pio_get_interrupt_status(PIOC));

	pio_handler_set_pin(a_pins->rdyn_pin, PIO_IT_FALL_EDGE, m_aci_isr_hal);
	pio_enable_pin_interrupt(a_pins->rdyn_pin);

	NVIC_SetPriority(PIOC_IRQn, 15);
	NVIC_EnableIRQ(PIOC_IRQn);
	
	vTraceSetISRProperties(4, "BleIsrHdl", 15);

    ioport_set_pin_level(a_pins->reqn_pin, 0);
    delay_us(1);
    ioport_set_pin_level(a_pins->reqn_pin, 1);
  }
}

bool hal_aci_tl_send(hal_aci_data_t *p_aci_cmd)
{
  const uint8_t length = p_aci_cmd->buffer[0];
  bool ret_val = false;

  if (length > HAL_ACI_MAX_LENGTH)
  {
    return false;
  }

  ret_val = aci_queue_enqueue(&aci_tx_q, p_aci_cmd);
  if (ret_val)
  {
    if(!aci_queue_is_full(&aci_rx_q))
    {
      // Lower the REQN only when successfully enqueued
      m_aci_reqn_enable();
    }

    if (aci_debug_print)
    {
	  taskENTER_CRITICAL();
      SEGGER_RTT_printf(0, "C"); //ACI Command
      m_aci_data_print(p_aci_cmd);
	  taskEXIT_CRITICAL();
    }
  }

  return ret_val;
}

static uint8_t spi_readwrite(const uint8_t aci_byte)
{
	//Board dependent defines
#if defined (__AVR__)
    //For Arduino the transmission does not have to be reversed
    return SPI.transfer(aci_byte);
#elif defined (__SAM3X8E__)
	//For ARM the transmission has to be reversed
	spi_write(SPI0, REVERSE_BITS(aci_byte), 0, 0);
	while ((spi_read_status(SPI0) & SPI_SR_RDRF) == 0);
	uint16_t data;
	uint8_t pcs;
	spi_read(SPI0, &data, &pcs);
	return REVERSE_BITS((uint8_t)data);
#elif defined(__PIC32MX__)
    //For ChipKit the transmission has to be reversed
    uint8_t tmp_bits;
    tmp_bits = SPI.transfer(REVERSE_BITS(aci_byte));
	return REVERSE_BITS(tmp_bits);
#endif
}

bool hal_aci_tl_rx_q_empty (void)
{
  return aci_queue_is_empty(&aci_rx_q);
}

bool hal_aci_tl_rx_q_full (void)
{
  return aci_queue_is_full(&aci_rx_q);
}

bool hal_aci_tl_tx_q_empty (void)
{
  return aci_queue_is_empty(&aci_tx_q);
}

bool hal_aci_tl_tx_q_full (void)
{
  return aci_queue_is_full(&aci_tx_q);
}

void hal_aci_tl_q_flush (void)
{
  m_aci_q_flush();
}

void m_aci_isr_hal(uint32_t id, uint32_t mask)
{
	//vTraceStoreISRBegin(4);
	
	if (ioport_get_pin_level(a_pins_local_ptr->rdyn_pin))
	{
		SEGGER_RTT_printf(0, "invalid irq\n");
		return;
	}
	
	//m_aci_isr();
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreIsr, &xHigherPriorityTaskWoken);
	
	//vTraceStoreISREnd(1);
}

void hal_aci_tl_isr_handler_task(void *pvParameters)
{
	UNUSED(pvParameters);
	
	xSemaphoreTake(xSemaphoreIsr, 0);
	
	for (;;)
	{
		xSemaphoreTake(xSemaphoreIsr, portMAX_DELAY);
		
		m_aci_isr();
	}
}