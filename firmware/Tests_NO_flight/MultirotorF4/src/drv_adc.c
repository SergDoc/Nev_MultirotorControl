#include "board.h"

#define ADC_BATTERY     0
#define ADC_CURRENT     1

// static volatile uint16_t adc1Ch4Value = 0;
static volatile uint16_t adcValues[2];

void adcInit(drv_adc_config_t *init)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    bool multiChannel = init->powerAdcChannel > 0;

    // ADC assumes all the GPIO was already placed in 'AIN' mode
    DMA_DeInit(DMA_Channel_0);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = multiChannel ? 2 : 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = multiChannel ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA_Channel_0, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA_Channel_0, ENABLE);

    ADC_InitStructure.ADC_Resolution = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = multiChannel ? ENABLE : DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = multiChannel ? 2 : 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_28Cycles);
    if (multiChannel)
        ADC_RegularChannelConfig(ADC1, init->powerAdcChannel, 2, ADC_SampleTime_28Cycles);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    //ADC_ResetCalibration(ADC1);
    //while(ADC_GetResetCalibrationStatus(ADC1));
   // ADC_StartCalibration(ADC1);
   // while(ADC_GetCalibrationStatus(ADC1));

    // Fire off ADC
    ADC_SoftwareStartConv(ADC1);
}

uint16_t adcGetChannel(uint8_t channel)
{
    return adcValues[channel];
}
