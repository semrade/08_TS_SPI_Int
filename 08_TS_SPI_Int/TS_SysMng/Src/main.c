/***********************************************************************************
 * File                     :main.c
 *
 * Title                    :
 *
 * Author                   :Tarik SEMRADE
 *
 * Description              :
 *
 * Version                  :
 *
 * Copyright (c) 2020 Tarik SEMRADE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *********************************************************************************/

/**********************************************************************************
 *  Included Files
 *
 *********************************************************************************/

#include "F28x_Project.h"
#include "main.h"

/**********************************************************************************
 *  Defines
 *
 *********************************************************************************/

//
// Globals
//
Uint16 sdata[100];     // Send data buffer
Uint16 rdata[100];     // Receive data buffer
Uint16 rdata_point;   // Keep track of where we are
// in the data stream to check received data

//
// Function Prototypes
//
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);
//void delay_loop(void);
void spi_fifo_init(void);
void error();

/**********************************************************************************
 * \function:       main
 * \brief           main `0` numbers
 * \param[in]       void
 * \return          void
 **********************************************************************************/
void main(void)
{

    Uint16 i;

    /* Set up system flash and turn peripheral clocks */
    InitSysCtrl();

    /* Init all gpio to input */
    InitGpio();


    /*This example function is found in the F2837xD_Gpio.c file and
     * illustrates how to set the GPIO to it's default state.
     * Setup only the GP I/O only for SPI-A functionality
     * */

    InitSpiaGpio();

    /* Globally disable maskable CPU interrupts */
    DINT;

    /* Clear and disable all PIE interrupts */
    InitPieCtrl();

    /* Individually disable maskable CPU interrupts */
    IER = 0x0000;

    /* Clear all CPU interrupt flags */
    IFR = 0x0000;

    /* Populate the PIE interrupt vector table */
    InitPieVectTable();
    /***********************Interrupt linking functions*****************************/
    /* Timer0 ISR function linking */
    /* This is needed to write to EALLOW protected registers*/
    EALLOW;

    PieVectTable.SPIA_RX_INT = &spiRxFifoIsr;
    PieVectTable.SPIA_TX_INT = &spiTxFifoIsr;

    /* This is needed to disable write to EALLOW protected registers */
    EDIS;

    /************************Peripheral Initialization*****************************/

    /* Initialize the Device Peripherals
     * Initialize the SPI only*/

    spi_fifo_init();


    /* Step 5. User specific code, enable interrupts: */
    for (i = 0; i < 2; i++)
    {
        /* Send back the same data*/
        sdata[i] = rdata[i];
    }
    rdata_point = 0;

    /* Enable interrupts required for this example */
    /*
     * Enable the PIE block
     * Enable PIE Group 6, INT 1
     * Enable PIE Group 6, INT 2
     * Enable CPU INT6
     * Enable Global Interrupts
     */

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx2 = 1;
    IER = 0x20;
    EINT;

    while (1)
    {
        asm(" NOP");
    }

}


void error(void)
{
    asm("     ESTOP0");
    //Test failed!! Stop!
    for (;;)
        ;
}
/**********************************************************************************
 * \function:       spi_fifo_init
 * \brief           spi_fifo_init - Initialize SPI FIFO
 * \param[in]       void
 * \return          void
 **********************************************************************************/
void spi_fifo_init()
{
    /* Initialize SPI FIFO registers
     * Enable FIFOs, set TX FIFO level to 2
     * Set RX FIFO level to 2
     * */
    SpiaRegs.SPIFFTX.all = 0xC022;
    SpiaRegs.SPIFFRX.all = 0x0022;
    SpiaRegs.SPIFFCT.all = 0x00;

    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;

    /* Initialize core SPI registers */
    InitSpi();
}
/**********************************************************************************
 * \function:       spiTxFifoIsr
 * \brief           spiTxFifoIsr - ISR for SPI transmit FIFO
 * \param[in]       void
 * \return          void
 **********************************************************************************/
interrupt void spiTxFifoIsr(void)
{
    Uint16 i;
    for (i = 0; i < 2; i++)
    {
        SpiaRegs.SPITXBUF = sdata[i];      // Send data
    }

    for (i = 0; i < 2; i++)                    // Increment data for next cycle
    {
        sdata[i] = sdata[i] + 1;
    }

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= 0x20;       // Issue PIE ACK
}
/**********************************************************************************
 * \function:       spiRxFifoIsr
 * \brief           spiRxFifoIsr - ISR for SPI receive FIFO
 * \param[in]       void
 * \return          void
 **********************************************************************************/
interrupt void spiRxFifoIsr(void)
{
    Uint16 i;

    for (i = 0; i < 2; i++)
    {
        rdata[i] = SpiaRegs.SPIRXBUF;     // Read data
    }

    for (i = 0; i < 2; i++)                  // Check received data
    {
        if (rdata[i] != rdata_point + i)
        {
            error();
        }
    }

    rdata_point++;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= 0x20;       // Issue PIE ack
}
