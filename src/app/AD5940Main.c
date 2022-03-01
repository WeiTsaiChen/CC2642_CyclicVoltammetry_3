/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Example of ramp test for electro-chemical sensor.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#include "RampTest.h"
#include "AD5940Main.h"

#include "simple_peripheral.h"
#include <ti/sysbios/knl/Task.h>

/**
   User could configure following parameters
**/

// Final results Monitoring 
MeasurementData_Type            PMData;

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

int counter = 0;        // Data counter
int index = 0;          // Data index

/****************************************************************************/

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount, float *Volt, int PointNumber)
{
  // static uint32_t index;
  // uint32_t rd;
    
  /* Print data */
  for(int i=0;i<DataCount;i++)
  { 
    if(PointNumber/2 >= index)
    {
      if(index == 0)
      {
        Volt[3] = Volt[2];
      }
      else
      {
        Volt[3] = Volt[3] + Volt[0];
      }  
    }
    else if(index >= PointNumber/2)
    {
      if(index == PointNumber-1)
      {
        Volt[3] = Volt[2];
      }
      else
      {
        Volt[3] = Volt[3] + Volt[1];
      }
    }
    
    // printf("index:%d, %.3f\n", index++, pData[i]);
    //i += 10;  /* Print though UART consumes too much time. */
    
    PMData.tVolt[i] = -Volt[3];         // Need to invert the result.
    PMData.tmAmpere[i] = -pData[i];     // Need to invert the result.
    
    index++;
      
    if(index == PointNumber) // Re-initialize the parameters
    {
      index = 0;
      Volt[3] = Volt[2];
    }
    
    AD5941_TX_data_transmit(0, 0, 0, Volt[3], pData[i], 0, 2);
    
  }
  
  // rd = AD5940_ReadAfeResult(AFERESULT_SINC3);
  // float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82);
  
  // printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f\n",rd, diff_volt, diff_volt+1.11);
  
  counter++;
  
  return 0;
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  /* Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/

// Initial Settings of CV
void AD5940RampStructInit(void) // COMPLIE FROM THIS SESSION, main function
{
  AppRAMPCfg_Type *pRampCfg;
  
  AppRAMPGetCfg(&pRampCfg);
  
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
  // pRampCfg->RcalVal = 10000.0;                  /* 10kOhm RCAL */
  pRampCfg->RcalVal = 350.0;     
  // Evaluation board: 10000
  // Test circuit: 220, 230
  // PCB Impedance Project: 210
  // PCB CV Project: ? in passive unit combination
  // PCB CV Project: 350, 400 in hexacyanoferrate, Potassium hexacyanoferrate and PBS combination
  // PCB CV Project: 210 in PBS
  
  pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 480;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */ 
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
  /* Configure ramp signal parameters */
  // pRampCfg->RampStartVolt =  -300.0f;           /* -0.3V */
  // pRampCfg->RampPeakVolt = 2300.0f;           /* +2.3V */
  
  // Need to invert the range of voltage you want to measure
  pRampCfg->RampStartVolt = 600.0f;               /* -0.5V */
  pRampCfg->RampPeakVolt = -600.0f;               /* 0.5V */
  
  pRampCfg->VzeroStart = 1300.0f;               /* 1.3V */
  pRampCfg->VzeroPeak =  1300.0f;                /* 1.3V , midscale of the LPDAC*/
  
  pRampCfg->StepNumber = 240;                   /* Total steps. Equals to ADC sample number, originally = 800 */
  // pRampCfg->RampDuration = 24*1000;            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
  pRampCfg->RampDuration = 20*1000;            
  // pRampCfg->SampleDelay = 7.0f;                 /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
  pRampCfg->SampleDelay = 20.0f;
  pRampCfg->LPTIARtiaSel = LPTIARTIA_2K;       /* Maximum current decides RTIA value */
                                                 // 200 -> 200R
  pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT; // Choose the suitable Rload here
  // pRampCfg->LPTIARloadSel = LPTIARLOAD_100R;
  pRampCfg->AdcPgaGain = ADCPGA_1P5;            // We must ensure signal is in range of +-1.5V which is limited by ADC input stage
     
  pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  
  // The postive peak voltage = RampPeakVolt - VzeroStart, 2300-1300=1000(+1V)
  // On the other hand, the negative peak voltage = -(VzeroStart) - RampStartVolt, 300 - 1300 = -1000(-1V)
  // The range of signal = +-1V
  // 0.4
  
}

void AD5940_Main(void) // main
{
  // uint32_t temp; 
  // AppRAMPCfg_Type *pRampCfg;	
  AD5940PlatformCfg();
  AD5940RampStructInit();

  
  AppRAMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
  // AppRAMPCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  // while(1)
  // {
    // AppRAMPGetCfg(&pRampCfg);
    
    // if(AD5940_GetMCUIntFlag())
    // {
    //   AD5940_ClrMCUIntFlag();
    //   temp = APPBUFF_SIZE;
    //   AppRAMPISR(AppBuff, &temp);       // ** //
    //   RampShowResult((float*)AppBuff, temp);
    // }
		/* Repeat Measurement continuously*/
	// 	if(pRampCfg->bTestFinished ==bTRUE)
	// 	{
	// 		AD5940_Delay10us(200000);
	// 		pRampCfg->bTestFinished = bFALSE;
	// 		AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait for trigger */
	// 		AppRAMPCtrl(APPCTRL_START, 0);  // Control the waveform sequence from DAC
	// 	}
  // }
  
}

void AD5940_measurement(int MeasureCycleNum)
{
  uint32_t temp; 
  float V_list[4];
  
  AppRAMPCfg_Type *pRampCfg;	
  
  AppRAMPGetCfg(&pRampCfg);
  V_list[0] = (pRampCfg->RampPeakVolt - pRampCfg->RampStartVolt) / (pRampCfg->StepNumber/2);        // Vstep phase 1
  V_list[1] = -(V_list[0]);     // Vstep phase 2
  V_list[2] = pRampCfg->RampStartVolt;  // Voltage initially
  V_list[3] = pRampCfg->RampStartVolt;  // Voltage recently
  
  AppRAMPCtrl(APPCTRL_START, 0);          /* Control RAMP measurement to start. Second parameter has no meaning with this command. */

  // while(1)
  while(MeasureCycleNum > counter)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppRAMPISR(AppBuff, &temp);       // ** //
      RampShowResult((float*)AppBuff, temp, (float*)V_list, pRampCfg->StepNumber);
    }
		/* Repeat Measurement continuously*/
		if(pRampCfg->bTestFinished == bTRUE)
		{
			AD5940_Delay10us(200000);
			pRampCfg->bTestFinished = bFALSE;
			AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait  for trigger */
			AppRAMPCtrl(APPCTRL_START, 0);  // Control the waveform sequence from DAC
		}
    
    if(MeasureCycleNum == counter)
    {
      counter = 0;
      AppRAMPCtrl(APPCTRL_STOPNOW, 0);
      Task_sleep(10);
    }
    
  }
}
