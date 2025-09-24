/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cybsp.h"
#include "retarget_io_init.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* CAN-FD message identifier 1*/
#define CANFD_NODE_1               (1U)
/* CAN-FD message identifier 2 (use different for 2nd device) */
#define CANFD_NODE_2               (2U)
/* message Identifier used for this code */
#define USE_CANFD_NODE             CANFD_NODE_1
/* CAN-FD channel number used */
#define CANFD_HW_CHANNEL           (0U)
/* CAN-FD data buffer index to send data from */
#define CANFD_BUFFER_INDEX         (0U)
/* Maximum incoming data length supported */
#define CANFD_DLC                  (8U)
#define CANFD_CHANNEL_MASK         (0x00000001UL)
#define CANFD_MRAM_DELAY_VAL       (0x0006U)
#define CANFD_INTERRUPT            canfd_0_interrupts0_0_IRQn
/* Priority for GPIO interrupts for USER BTN1 */
#define GPIO_INTERRUPT_PRIORITY    (0U)
/*Port interrupt mask for USER BTN1 */
#define PORT_INTR_MASK             (0x00000001UL << 8U)
/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC   (10U)
/* Debounce delay for USER BTN */
#define BUTTON_DEBOUNCE_DELAY      (300)
#define MSG_IDX_INIT_VAL           (0U)
#define CANFD_INTERRUPT_PRIORITY   (3UL)
/* CANFD Data*/
#define CANFD_USER_DATA_0          (0x04030201U)
#define CANFD_USER_DATA_1          (0x08070605U)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* This is a shared context structure, unique for each can-fd channel */
static cy_stc_canfd_context_t canfd_context;

/* Variable which holds the button pressed status */
volatile bool gpio_intr_flag = false;

/* Array to hold CANFD0 data */
static uint32_t canfd_tx_data_buffer[] = 
{
    [CANFD_DATA_0] = CANFD_USER_DATA_0,
    [CANFD_DATA_1] = CANFD_USER_DATA_1,
};

/* Configuration structure for CANFD0 Interrupt */
static cy_stc_sysint_t canfd_irq_cfg = 
{
    /* Source of interrupt signal */
    .intrSrc = CANFD_INTERRUPT,
    /* Interrupt priority */
    .intrPriority = CANFD_INTERRUPT_PRIORITY,
};

/* Interrupt config structure */
cy_stc_sysint_t intrCfg =
{   /* Source of interrupt signal */
    CYBSP_USER_BTN_IRQ,
    /* Interrupt priority */
    GPIO_INTERRUPT_PRIORITY
};

/*******************************************************************************
 * Function Definitions
 *******************************************************************************/
/*******************************************************************************
 * Function Name: isr_canfd
 *******************************************************************************
 * Summary:
 * This is the interrupt handler function for the can-fd interrupt.
 *
 * Parameters:
 * none
 *
 * Return:
 * void
 ******************************************************************************/
static void isr_canfd(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD0, CANFD_HW_CHANNEL, &canfd_context);
}

/*******************************************************************************
 * Function Name: gpio_interrupt_handler
 *******************************************************************************
 * Summary:
 * GPIO interrupt handler.
 *
 * Parameters:
 * None
 *
 * Return:
 * void
 ******************************************************************************/
void gpio_interrupt_handler()
{
    /* USER_BTN debounce delay */
    Cy_SysLib_Delay(BUTTON_DEBOUNCE_DELAY);

    /* Get interrupt cause */
    uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();

    /* Check if the interrupt was from the user button's port and pin */
    if((PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK)) &&
       (1UL == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN_PORT,
               CYBSP_USER_BTN_PIN)))
    {
        /* Set the interrupt flag */
        gpio_intr_flag = true;

        /* Clear the interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN_IRQ);
    }
}

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * This is the main function. It initializes the CANFD channel and interrupt.
 * User button and User LED are also initialized. The main loop checks
 * for the button pressed interrupt flag and when it is set, a CANFD frame
 * is sent. Whenever a CANFD frame is received from other nodes, the user LED
 * toggles and the received data is logged over serial terminal.
 *
 * Parameters:
 * None
 *
 * Return:
 * Int
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_canfd_status_t status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen. */
    printf("\x1b[2J\x1b[;H");

    printf("===========================================================\r\n");
    printf("PSOC Edge MCU: CANFD                                       \r\n");
    printf("===========================================================\r\n\n");

    printf("===========================================================\r\n");
    printf("CANFD Node-%d\r\n", USE_CANFD_NODE);
    printf("===========================================================\r\n\n");

    /* Enable global interrupts */
    __enable_irq();

     /*Clear GPIO and NVIC interrupt before initializing to avoid false
      * triggering of interrupt.
      */
     Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT,CYBSP_USER_BTN2_PIN );
     Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT,CYBSP_USER_BTN_PIN );
     NVIC_ClearPendingIRQ(CYBSP_USER_BTN_IRQ);

     /* Hook the interrupt service routine for USER BTN interrupt*/
     Cy_SysInt_Init(&intrCfg, &gpio_interrupt_handler);

     /* Enable the USER BTN interrupt */
     NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Enable CANFD Message RAM */
    Cy_CANFD_EnableMRAM(CANFD0, CANFD_CHANNEL_MASK, CANFD_MRAM_DELAY_VAL);

    /*Initialize CANFD */
    status = Cy_CANFD_Init(CANFD0, CANFD_HW_CHANNEL, &CANFD_config,
            &canfd_context);

    /* CAN-FD init failed. Stop program execution */
    if (CY_CANFD_SUCCESS != status)
    {
        handle_app_error();
    }

    /* Hook the interrupt service routine for CANFD interrupt*/
    Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);

    /* Enable the CAN-FD interrupt */
    NVIC_EnableIRQ(CANFD_INTERRUPT);

    /* Enable the configuration changes */
    Cy_CANFD_ConfigChangesEnable(CANFD0, CANFD_HW_CHANNEL);

    /* Sets the Test mode configuration */
    Cy_CANFD_TestModeConfig(CANFD0, CANFD_HW_CHANNEL,
            CY_CANFD_TEST_MODE_DISABLE);

    /* Disables the configuration changes */
    Cy_CANFD_ConfigChangesDisable(CANFD0, CANFD_HW_CHANNEL);

    /* Setting Node Identifier as defined in Macro*/
    CANFD_T0RegisterBuffer_0.id = USE_CANFD_NODE;

    /* Assign the user defined data buffer to CANFD data area */
    CANFD_txBuffer_0.data_area_f = canfd_tx_data_buffer;

    /* CY_CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CY_CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    for (;;)
    {
        /*Check if USER BTN is pressed */
        if (gpio_intr_flag)
        {
            gpio_intr_flag = false;

            /* Sending CAN-FD frame to other node */
            status = Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD0,
                     CANFD_HW_CHANNEL, &CANFD_txBuffer_0, 1U, &canfd_context);

            if (CY_CANFD_SUCCESS == status)
            {
                printf("CAN-FD Frame sent "
                        "with message ID-%d\r\n\r\n",
                        USE_CANFD_NODE);

            }
            else
            {
                printf("Error sending CAN-FD "
                        "Frame with message ID-%d\r\n\r\n",
                        USE_CANFD_NODE);
            }
        }
    }
}

/*******************************************************************************
 * Function Name: canfd_rx_callback
 *******************************************************************************
 * Summary:
 * This is the callback function for can-fd reception
 *
 * Parameters:
 * msg_valid           :          Message received properly or not
 * msg_buf_fifo_num    :          RxFIFO number of the received message
 * canfd_rx_buf        :          Message buffer
 *
 * Return:
 * void

 ******************************************************************************/
void canfd_rx_callback(bool msg_valid, uint8_t msg_buf_fifo_num,
                       cy_stc_canfd_rx_buffer_t *canfd_rx_buf)
{
    /* Array to hold the data bytes of the CAN-FD frame */
    uint8_t canfd_rx_data_buffer[CANFD_DLC];

    /* Variable to hold the data length code of the CAN-FD frame */
    uint32_t canfd_dlc;

    /* Variable to hold the Identifier of the CAN-FD frame */
    uint32_t canfd_id;

    if (msg_valid) {
        /* Checking whether the frame received is a data frame */
        if (CY_CANFD_RTR_DATA_FRAME == canfd_rx_buf->r0_f->rtr)
        {

            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

            canfd_dlc = canfd_rx_buf->r1_f->dlc;
            canfd_id = canfd_rx_buf->r0_f->id;

            printf("%d bytes received with "
                    "message identifier "
                    "%d\r\n\r\n",
                  (int) canfd_dlc, (int) canfd_id);

            memcpy(canfd_rx_data_buffer, canfd_rx_buf->data_area_f,
                    canfd_dlc);

            printf("Rx Data : ");

            for (uint8_t msg_idx = MSG_IDX_INIT_VAL; msg_idx
                 < canfd_dlc; msg_idx++)
            {
                printf(" %d ", canfd_rx_data_buffer[msg_idx]);
            }

            printf("\r\n\r\n");
        }
    }
}
/* [] END OF FILE */
