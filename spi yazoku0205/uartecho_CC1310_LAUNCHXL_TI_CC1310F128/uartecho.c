  /*
   * Copyright (c) 2016, Texas Instruments Incorporated
   * All rights reserved.
   *
   * Redistribution and use in source and binary forms, with or without
   * modification, are permitted provided that the following conditions
   * are met:
   *
   * *  Redistributions of source code must retain the above copyright
   *    notice, this list of conditions and the following disclaimer.
   *
   * *  Redistributions in binary form must reproduce the above copyright
   *    notice, this list of conditions and the following disclaimer in the
   *    documentation and/or other materials provided with the distribution.
   *
   * *  Neither the name of Texas Instruments Incorporated nor the names of
   *    its contributors may be used to endorse or promote products derived
   *    from this software without specific prior written permission.
   *
   * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   */

  //
   //  ======== uartecho.c ========
    //

  // XDCtools Header files //
  #include <xdc/std.h>
  #include <xdc/runtime/System.h>
  #include <xdc/cfg/global.h>

  // BIOS Header files //
  #include <ti/sysbios/BIOS.h>
  #include <ti/sysbios/knl/Task.h>
  #include <ti/sysbios/knl/Clock.h>


  // TI-RTOS Header files //
  #include <ti/drivers/PIN.h>
  #include <ti/drivers/UART.h>
  #include <ti/drivers/I2C.h>
  #include <ti/drivers/SPI.h>

  // Example/Board Header files
  #include "Board.h"


  #include <stdlib.h>
  #include <stdbool.h>
  #include <stdint.h>

  #define TASKSTACKSIZE     768

  Task_Struct task0Struct;
  Char task0Stack[TASKSTACKSIZE];

  // Global memory storage for a PIN_Config table
  static PIN_State ledPinState;
  static SPI_Handle spiHandle;
  static SPI_Params spiParams;

  //
  // Application LED pin configuration table:
  //   - All LEDs board LEDs are off.
   //
  PIN_Config ledPinTable[] = {
      Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_GENERALPURPOSE1   | PIN_GPIO_OUTPUT_EN 	| PIN_GPIO_HIGH	 | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_CP2103UARTRESET   | PIN_GPIO_OUTPUT_EN	| PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
	// Board_SPI0_MOSI | PIN_PULLDOWN,
	// Board_SPI0_MISO | PIN_PULLDOWN,
	// Board_SPI0_CLK | PIN_PULLDOWN,



      PIN_TERMINATE
  };


  PIN_Handle ledPinHandle;




  int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen)
  {
    SPI_Transaction masterTransaction;
    bool success;

    masterTransaction.count  = wlen + rlen;
    masterTransaction.txBuf  = buf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = buf;

    success = SPI_transfer(spiHandle, &masterTransaction);
    if (success)

    	  {
  	  System_printf("data transfered\n");
  	  	  System_flush();
      memcpy(buf,buf+wlen,rlen);
    	  }


    return success ? 0 : -1;
  }
  int bspSpiRead(uint8_t *buf, size_t len)
  {
	  //uint8_t status;
	  bool s;
    SPI_Transaction masterTransaction;

    masterTransaction.count  = len;
    masterTransaction.txBuf  = NULL;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = buf;




			s=	SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
			System_printf("spi read status  = %d\n", masterTransaction.status);

    return s;
  }

  int bspSpiWrite(const uint8_t *buf, size_t len)
  {
	  bool s1;
    SPI_Transaction masterTransaction;

    masterTransaction.count  = len;
    masterTransaction.txBuf  = (void*)buf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    s1= SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
    System_printf("spi write status  = %d\n", masterTransaction.status);

    return s1;
  }

#define EFL_ADDR_RECOVERY   			  0x20000
#define EFL_SIZE_RECOVERY				  0x20000
#define EFL_PAGE_SIZE               0x1000
#define EFL_FLASH_SIZE              0x80000

#define BLS_PROGRAM_PAGE_SIZE     256

#define BLS_CODE_PROGRAM          0x02 /**< Page Program */
#define BLS_CODE_READ             0x03 /**< Read Data */
#define BLS_CODE_ID               0x4B /**< Read Data */

#define BLS_CODE_READ_STATUS      0x05 /**< Read Status Register */
#define BLS_CODE_WRITE_ENABLE     0x06 /**< Write Enable */
#define BLS_CODE_SECTOR_ERASE     0x20 /**< Sector Erase */
#define BLS_CODE_MDID             0x90 /**< Manufacturer Device ID */

#define FLASH_ADDRESS(BLS_PROGRAM_PAGE_SIZE, addr) (((BLS_PROGRAM_PAGE_SIZE) << 12) + (addr))

  bool extFlashWrite(size_t addr, size_t length, const uint8_t *buf)
  {
    uint8_t wbuf[5];
    //uint32_t addr = 0x00000000;
    uint8_t byt = 0xAA;
    while (length > 0)
    {
      /* Wait till previous erase/program operation completes */

      size_t ilen; /* interim length per instruction */

      ilen = BLS_PROGRAM_PAGE_SIZE - (addr % BLS_PROGRAM_PAGE_SIZE);
      if (length < ilen)
      {
        ilen = length;
      }

      wbuf[0] = BLS_CODE_PROGRAM;
      wbuf[1] = (addr>>16);
      wbuf[2] = (addr>>8);
      wbuf[3] = (addr);
      wbuf[4] = (byt);


      addr += ilen;
      length -= ilen;

      /* Up to 100ns CS hold time (which is not clear
       * whether it's application only in between reads)
       * is not imposed here since above instructions
       * should be enough to delay
       * as much. */
  	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);
  	//bspSpiWrite(wbuf, sizeof(wbuf));

      if (bspSpiWrite(wbuf, sizeof(wbuf)))
      {
        /* failure */
      	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);
        return false;
      }

      if (bspSpiWrite(buf,ilen))
      {
        /* failure */
        	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);
        return false;
      }
      buf += ilen;
    	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);
    }

    return true;
  }



  bool extFlashRead(size_t addr, size_t length, uint8_t *buf)
  {
    uint8_t wbuf[5];
   // uint32_t addr = 0x00000000;



    /* SPI is driven with very low frequency (1MHz < 33MHz fR spec)
     * in this temporary implementation.
     * and hence it is not necessary to use fast read. */
    wbuf[0] = BLS_CODE_READ;
    wbuf[1] = (addr>>16);
    wbuf[2] = (addr>>8);
    wbuf[3] = (addr);
    wbuf[4] = NULL;


  	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);

    if (bspSpiWrite(wbuf, sizeof(wbuf)))
    {
      /* failure */
    	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);

      return false;

    }


      int ret = bspSpiRead(buf, length);
	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);

    return ret ;

  }


   //  ======== echoFxn ========
   //  Task for this function is created statically. See the project's .cfg file.
  //
  Void taskFxn(UArg arg0, UArg arg1)
  {
    	PIN_Handle ledPinHandle;

    	ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    	PIN_setOutputValue(ledPinHandle, Board_LED0, 1);

    	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);


    	/*  Configure SPI as master, 1 mHz bit rate*/
    	    SPI_Params_init(&spiParams);
    	    spiParams.bitRate = 1000000;
    	    spiParams.mode         = SPI_MASTER;
    	    spiParams.transferMode = SPI_MODE_BLOCKING;

    	    /* Attempt to open SPI. */
    	    spiHandle = SPI_open(Board_SPI0, &spiParams);

    	    if (spiHandle == NULL)
    	       {
    	    	System_printf("spi is not open \n");
    	    	  System_flush();
 	       }
    	    else{
    	    	System_printf("spi is open \n");
    	    	  System_flush();


	    	        static uint8_t wrbuf[25] = { 3 };
	    	        static uint8_t rdbuf[25] = {   };

	    	       // memset(wrbuf,5, sizeof(wrbuf));
	    	       // memset(rdbuf,4, sizeof(rdbuf));


    	    	  bool success;
    	    	  uint32_t addr ;



                  success = extFlashWrite(FLASH_ADDRESS(BLS_PROGRAM_PAGE_SIZE, addr), sizeof(wrbuf), wrbuf);

                  if (success)
                  {
							   extFlashRead(FLASH_ADDRESS(BLS_PROGRAM_PAGE_SIZE, addr), sizeof(rdbuf), rdbuf);

							   System_printf("read buf %u %u %u %u %u %u \n",  rdbuf[0] ,rdbuf[1] ,rdbuf[2], rdbuf[3] , rdbuf[4], rdbuf[5]);
							   	   	   	   System_flush();
							   System_printf("write buf %u  %u \n",  wrbuf[3], wrbuf[4]);
										   System_flush();

                  }



    	    }


















    }

   //  ======== main ========
   //
  int main(void)
  {
      PIN_Handle ledPinHandle;
      Task_Params taskParams;

      // Call board init functions
      Board_initGeneral();
      Board_initSPI();


      // Construct BIOS objects
      Task_Params_init(&taskParams);
      taskParams.stackSize = TASKSTACKSIZE;
      taskParams.stack = &task0Stack;
      Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


      // Open LED pins
      ledPinHandle = PIN_open(&ledPinState, ledPinTable);

      if(!ledPinHandle) {
    	  System_printf("Error initializing board LED pins\n");
    	  System_flush();
      }
      else{
    	  System_printf("Perfect! initializing board LED pins\n");
    	  System_flush();
      }

     // PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
      PIN_setOutputValue(ledPinHandle, Board_CP2103UARTRESET, 1);
     // PIN_setOutputValue(ledPinHandle, Board_GENERALPURPOSE1, 1);


      // Start BIOS
      BIOS_start();

      return (0);
  }
