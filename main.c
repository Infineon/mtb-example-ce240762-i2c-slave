/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Slave. The I2C slave receives the
* command packets from the I2C master to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Packet positions */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)
#define PACKET_STATUS_POS       (1UL)

/* Start and end of packet markers */
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)

/* I2C slave interrupt priority */
#define I2C_SLAVE_IRQ_PRIORITY  (7u)

/* Command valid status */
#define STATUS_CMD_DONE         (0x00UL)
#define STATUS_CMD_FAIL         (0xFFUL)

/* Packet size */
#define PACKET_SIZE             (3UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static uint8_t i2c_write_buffer[PACKET_SIZE] = {0};
static uint8_t i2c_read_buffer[PACKET_SIZE] = {PACKET_SOP,
                                               STATUS_CMD_FAIL,
                                               PACKET_EOP};
static cy_stc_scb_i2c_context_t i2c_context;
const cy_stc_sysint_t IRQ_CFG_I2C =
{
    .intrSrc = (((NvicMux2_IRQn << CY_SYSINT_INTRSRC_MUXIRQ_SHIFT)
                 | scb_11_interrupt_IRQn)),
    .intrPriority = 0u,
};

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    UART_context;           /** UART context */
static mtb_hal_uart_t               UART_hal_obj;           /** Debug UART HAL object  */

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: i2c_slave_callback
********************************************************************************
* Summary:
* This is a callback function for I2C slave events. If a write event occurs,
* the command packet is verified and executed.
*
* Parameters:
*  event        : I2C event
*
* Return:
*  void
*
*******************************************************************************/
void i2c_slave_callback(uint32_t event)
{
    if (0UL == (CY_SCB_I2C_SLAVE_ERR_EVENT & event))
    {
        if (0UL != (CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT & event))
        {
            /* Check start and end of packet markers */
            if ((i2c_write_buffer[PACKET_SOP_POS] == PACKET_SOP) &&
                (i2c_write_buffer[PACKET_EOP_POS] == PACKET_EOP))
            {
                printf("User LED should start blinking \r\n");
                /* Execute command */
                Cy_GPIO_Write(CYBSP_USER_LED1_PORT,
                              CYBSP_USER_LED1_PIN,
                              i2c_write_buffer[PACKET_CMD_POS]);

                /* Update status of received command */
                i2c_read_buffer[PACKET_STATUS_POS] = STATUS_CMD_DONE;
            }

            /* Configure read buffer for the next write */
            i2c_write_buffer[PACKET_SOP_POS] = 0;
            i2c_write_buffer[PACKET_EOP_POS] = 0;
            Cy_SCB_I2C_SlaveConfigWriteBuf(I2C_SLAVE_HW,
                                           i2c_write_buffer,
                                           PACKET_SIZE,
                                           &i2c_context);
        }

        if (0UL != (CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT & event))
        {
            /* Configure write buffer for the next read */
            i2c_read_buffer[PACKET_STATUS_POS] = STATUS_CMD_FAIL;
            Cy_SCB_I2C_SlaveConfigReadBuf(I2C_SLAVE_HW,
                                          i2c_read_buffer,
                                          PACKET_SIZE,
                                          &i2c_context);
        }
    }
}

/*******************************************************************************
* Function Name: i2c_slave_isr
********************************************************************************
* Summary:
* This is a interrupt service routine for I2C slave events. If a write event
* occurs, the command packet is verified and executed.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void i2c_slave_isr(void)
{
    /* i2c_slave_callback is called inside the function */
    Cy_SCB_I2C_SlaveInterrupt(I2C_SLAVE_HW, &i2c_context);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the I2C slave to receive packet from the master
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    handle_error(result);

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&UART_hal_obj, &UART_hal_config, &UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************** "
           "PDL: I2C Slave "
           "************** \r\n\n");

    /* I2C Slave configuration settings */
    printf(">> Configuring I2C Slave..... ");

    /* Deinit I2C before initialization */
    Cy_SCB_I2C_DeInit(I2C_SLAVE_HW);

    /* Initialize I2C as a slave */
    result = Cy_SCB_I2C_Init(I2C_SLAVE_HW, &I2C_SLAVE_config, &i2c_context);

    handle_error(result);

    /* Configures the buffer pointer and the read buffer size */
    Cy_SCB_I2C_SlaveConfigReadBuf(I2C_SLAVE_HW,
                                  i2c_read_buffer,
                                  PACKET_SIZE,
                                  &i2c_context);

    /* Configures the buffer pointer and size of the write buffer */
    Cy_SCB_I2C_SlaveConfigWriteBuf(I2C_SLAVE_HW,
                                   i2c_write_buffer,
                                   PACKET_SIZE,
                                   &i2c_context);

    /* Register cy_stc_scb_i2c_context_t i2c_context -> cbEvents  */
    /* i2c_slave_callback is registered as a callback             */
    Cy_SCB_I2C_RegisterEventCallback(I2C_SLAVE_HW,
                                     i2c_slave_callback,
                                     &i2c_context);

    /* Enable interrupt */
    Cy_SCB_SetSlaveInterruptMask(I2C_SLAVE_HW, CY_SCB_I2C_SLAVE_INTR);
    Cy_SysInt_Init(&IRQ_CFG_I2C, &i2c_slave_isr);
    NVIC_ClearPendingIRQ(NvicMux2_IRQn);
    NVIC_EnableIRQ((IRQn_Type)NvicMux2_IRQn);
    NVIC_SetPriority(NvicMux2_IRQn, I2C_SLAVE_IRQ_PRIORITY);
    Cy_SCB_I2C_Enable(I2C_SLAVE_HW);

    printf("Done\r\n");

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
    }
}

/* [] END OF FILE */