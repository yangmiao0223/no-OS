/***************************************************************************//**
 *   @file   iio_demo/src/main.c
 *   @brief  Implementation of Main Function.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "app_config.h"
#include "parameters.h"
#include "error.h"
#include "iio.h"
#include "irq.h"
#include "irq_extra.h"
#include "uart.h"
#include "uart_extra.h"
#include "iio_ad7124.h"
#include "ad7124_regs.h"
#include "spi_extra.h"

#ifdef USE_TCP_SOCKET
#include "wifi.h"
#include "tcp_socket.h"
#endif

#ifdef XILINX_PLATFORM
#include <xparameters.h>
#include <xil_cache.h>
#include <xil_cache.h>
#endif // XILINX_PLATFORM

#ifdef ADUCM_PLATFORM

#include <sys/platform.h>
#include "adi_initialize.h"
#include <drivers/pwr/adi_pwr.h>

#define MAX_SIZE_BASE_ADDR		3000

//static uint8_t in_buff[MAX_SIZE_BASE_ADDR];
//static uint8_t out_buff[MAX_SIZE_BASE_ADDR];

#define DAC_DDR_BASEADDR	((uint32_t)out_buff)
#define ADC_DDR_BASEADDR	((uint32_t)in_buff)

#endif

int32_t platform_init()
{
#ifdef ADUCM_PLATFORM
	if (ADI_PWR_SUCCESS != adi_pwr_Init())
		return FAILURE;

	if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1u))
		return FAILURE;

	if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1u))
		return FAILURE;
	adi_initComponents();
#endif
	return SUCCESS;
}

extern struct iio_device iio_ad7124_device;

/***************************************************************************//**
 * @brief main
*******************************************************************************/
int main(void)
{
	int32_t status;
	//char demo_device_output[] = "demo_device_output";
	//char demo_device_input[] = "demo_device_input";

	/* iio demo configurations. */
	//struct iio_demo_init_param iio_demo_in_init_par;

	/* iio demo configurations. */
	//struct iio_demo_init_param iio_demo_out_init_par;

	/* iio descriptor. */
	struct iio_desc  *iio_device;

	/* iio initialization structure */
	struct iio_init_param iio_inital;

	/* ad7124 iio descriptor instance */
	struct iio_ad7124_desc *iio_ad7124;

	/* ad7124 iio init param */
	struct iio_ad7124_init_param iio_ad7124_initial = {
		.ddr_base_addr = NULL
	};

	/* Initialization for UART. */
	struct uart_init_param uart_init_par;

	/* IRQ initial configuration. */
	struct irq_init_param irq_init_param;

	/* IRQ instance. */
	struct irq_ctrl_desc *irq_desc;

#ifdef USE_TCP_SOCKET
	struct tcp_socket_init_param	socket_param;
	struct wifi_init_param		wifi_param;
	struct wifi_desc		*wifi;
	struct uart_desc		*uart_desc;
#endif

	status = platform_init();
	if (IS_ERR_VALUE(status))
		return status;

#ifdef XILINX_PLATFORM
	/* Xilinx platform dependent initialization for IRQ. */
	struct xil_irq_init_param platform_irq_init_par;

	platform_irq_init_par = (struct xil_irq_init_param ) {
#ifdef XPAR_INTC_SINGLE_DEVICE_ID
		.type = IRQ_PL,
#else
		.type = IRQ_PS,
#endif
	};
#endif // XILINX_PLATFORM

#ifdef ADUCM_PLATFORM
	/* Dummy value for Aducm platform dependent initialization for IRQ. */
	int32_t platform_irq_init_par = 0;
#endif //ADUCM_PLATFORM

	irq_init_param = (struct irq_init_param ) {
		.irq_ctrl_id = INTC_DEVICE_ID,
		.extra = &platform_irq_init_par
	};

	status = irq_ctrl_init(&irq_desc, &irq_init_param);
	if(status < 0)
		return status;

#ifdef XILINX_PLATFORM
	/* Xilinx platform dependent initialization for UART. */
	struct xil_uart_init_param platform_uart_init_par;

	platform_uart_init_par = (struct xil_uart_init_param) {
#ifdef XPAR_XUARTLITE_NUM_INSTANCES
		.type = UART_PL,
#else
		.type = UART_PS,
#endif
		.irq_id = UART_IRQ_ID,
		.irq_desc = irq_desc,
	};
#endif // XILINX_PLATFORM

#ifdef ADUCM_PLATFORM

	/* Aducm platform dependent initialization for UART. */
	struct aducm_uart_init_param platform_uart_init_par = {
		.parity = UART_NO_PARITY,
		.stop_bits = UART_ONE_STOPBIT,
		.word_length = UART_WORDLEN_8BITS
	};
#endif // ADUCM_PLATFORM
	uart_init_par = (struct uart_init_param) {
		.device_id = UART_DEVICE_ID,
		.baud_rate = UART_BAUDRATE,
		.extra = &platform_uart_init_par
	};

	status = irq_global_enable(irq_desc);
	if (status < 0)
		return status;

#ifdef USE_TCP_SOCKET
	status = uart_init(&uart_desc, &uart_init_par);
	if (status < 0)
		return status;



	wifi_param.irq_desc = irq_desc;
	wifi_param.uart_desc = uart_desc;
#ifdef ADUCM_PLATFORM
	wifi_param.uart_irq_conf = uart_desc;
#endif //ADUCM_PLATFORM
	wifi_param.uart_irq_id = UART_IRQ_ID;

	status = wifi_init(&wifi, &wifi_param);
	if (status < 0)
		return status;

	status = wifi_connect(wifi, WIFI_SSID, WIFI_PWD);
	if (status < 0)
		return status;

	char buff[100];
	wifi_get_ip(wifi, buff, 100);
	printf("Tinyiiod ip is: %s\n", buff);

	wifi_get_network_interface(wifi, &socket_param.net);
	socket_param.max_buff_size = 0;

	iio_inital.phy_type = USE_NETWORK;
	iio_inital.tcp_socket_init_param = &socket_param;

#else //USE_TCP_SOCKET
	iio_inital.phy_type = USE_UART;
	iio_inital.uart_init_param = &uart_init_par;
#endif //USE_TCP_SOCKET

	status = iio_init(&iio_device, &iio_inital);
	if(status < 0)
		return status;

	struct ad7124_dev *ad7124_device;
	struct aducm_spi_init_param aducm_spi_ini = {
		.continuous_mode = true,
		.dma = false,
		.half_duplex = false,
		.master_mode = MASTER,
		.spi_channel = SPI1
	};
	struct spi_init_param spi_initial = {
		.chip_select = 0x00,
		.max_speed_hz = 10000000,
		.mode = SPI_MODE_3,
		.extra = &aducm_spi_ini
	};
	struct ad7124_init_param ad7124_initial = {
		.spi_init = &spi_initial,
		.regs = ad7124_regs,
		.spi_rdy_poll_cnt = 1000
	};
	status = ad7124_setup(&ad7124_device, &ad7124_initial);
	if (status < 0)
		return status;

	iio_ad7124_initial.ad7124_driver_handler = ad7124_device;

	status = iio_ad7124_init(&iio_ad7124, &iio_ad7124_initial);
	if (status < 0)
		return status;

	status = iio_register(iio_device, iio_ad7124->iio_ad7124_device,
			      "iio_ad7124", ad7124_device);
	if (status < 0)
		return status;

	do {
		status = iio_step(iio_device);
	} while (true);

	return status;
}
