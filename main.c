#include "F28x_Project.h"

// define macros
#define ACQPS_SETTING 9u
#define ADC_WARMUP_TIME 1000u
#define DAC_WARMUP_TIME 10u
#define DELFINO_MAX_VALUE 4096u
#define MAX_GAIN 10u
#define DT 1.0
#define KP 1.0
#define KI 0.0
#define KD 0.0
#define NUM_INPUTS_PER_OUTPUT 2
#define NUM_OUTPUTS 2
#define HOLD_OUTPUT_GPIO_PIN_A 89
#define LED_GPIO_PIN_A 90
#define HOLD_OUTPUT_GPIO_PIN_B 60
#define LED_GPIO_PIN_B 61
#define LED_THRESHOLD 14

// declare array storing ADC values and store this in ramgs0 for use in DMA
#pragma DATA_SECTION(analog_ins, "ramgs0");
Uint16 analog_ins[NUM_OUTPUTS][NUM_INPUTS_PER_OUTPUT];

// declare array storing output offsets and store this in ramgs0 for use in DMA
#pragma DATA_SECTION(output_offsets, "ramgs0");
Uint16 output_offsets[NUM_OUTPUTS];

// declare array storing gains and store this in ramgs0 for use in DMA
#pragma DATA_SECTION(gains, "ramgs0");
Uint16 gains[NUM_OUTPUTS];

// declare array holding DAC values
Uint16 analog_outs[NUM_OUTPUTS];

// declare variable holding the maximum DAC value that should be outputted
Uint16 dac_max_value;

// move this code to RAM
#pragma CODE_SECTION(dma_ch1_int_isr, ".TI.ramfunc");
__interrupt void dma_ch1_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK1 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch2_int_isr, ".TI.ramfunc");
__interrupt void dma_ch2_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK2 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch3_int_isr, ".TI.ramfunc");
__interrupt void dma_ch3_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK3 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch4_int_isr, ".TI.ramfunc");
__interrupt void dma_ch4_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK4 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch5_int_isr, ".TI.ramfunc");
__interrupt void dma_ch5_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK5 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch6_int_isr, ".TI.ramfunc");
__interrupt void dma_ch6_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK6 = PIEACK_GROUP7;
	EDIS;
}

void configure_adcs(void)
{
	// allow writing to protected registers
	EALLOW;

	// set the ADC clocks to one quarter of the input clock
	AdcaRegs.ADCCTL2.bit.PRESCALE = 0x6;
	AdcbRegs.ADCCTL2.bit.PRESCALE = 0x6;

	// set the ADCs to have twelve bit resolution and put it in single signal mode
	// in single signal mode, the input pin voltage is referenced against VREFLO
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	// generate interrupt pulses at the end of conversions
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	// power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	// give time for the ADCs to power up
	DELAY_US(ADC_WARMUP_TIME);

	// disallow writing to protected registers
	EDIS;
}

void setup_adc_channels(void)
{
	// allow writing to protected registers
	EALLOW;

	// set channels
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5;
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4;
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5;
	AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0;

	// set sample windows
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC4CTL.bit.ACQPS = ACQPS_SETTING;

	// set what SOCs trigger ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
	AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3SEL = 2;
	AdcaRegs.ADCINTSEL3N4.bit.INT4SEL = 3;
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
	AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3SEL = 2;
	AdcbRegs.ADCINTSEL3N4.bit.INT4SEL = 3;

	// enable the ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3E = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT4E = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3E = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT4E = 1;

	// enable continuous mode for the ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3CONT = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT4CONT = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3CONT = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT4CONT = 1;

    // reset the ADC interrupt registers
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;

	// disallow writing to protected registers
	EDIS;
}

void configure_dacs(void)
{
	// allow writing to protected registers
	EALLOW;

	// use the same voltage references as the ADC
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    DaccRegs.DACCTL.bit.DACREFSEL = 1;

    // enable the DACs
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;

    // zero the DACs
    DacaRegs.DACVALS.bit.DACVALS = 0x0;
    DacbRegs.DACVALS.bit.DACVALS = 0x0;
    DaccRegs.DACVALS.bit.DACVALS = 0x0;

    // give time for the DACs to power up
    DELAY_US(DAC_WARMUP_TIME);

	// disallow writing to protected registers
	EDIS;
}

void configure_dma(void)
{
	// disconnect CLA and connect DMA
	EALLOW;
	CpuSysRegs.SECMSEL.bit.PF1SEL = 1;
	EDIS;

	// initialize DMA
	DMAInitialize();

	// setup DMA channel one
	DMACH1AddrConfig((volatile Uint16 *)(&analog_ins[0][0]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT0));

	// send one 16 bit word in a burst
	DMACH1BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH1TransferConfig(1, 0, 0);

	// configure DMA channel one
	DMACH1ModeConfig(1, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel two
	DMACH2AddrConfig((volatile Uint16 *)(&analog_ins[0][1]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT1));

	// send one 16 bit word in a burst
	DMACH2BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH2TransferConfig(1, 0, 0);

	// configure DMA channel two
	DMACH2ModeConfig(2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel three
	DMACH3AddrConfig((volatile Uint16 *)(&output_offsets[0]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT2));

	// send one 16 bit word in a burst
	DMACH3BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH3TransferConfig(1, 0, 0);

	// configure DMA channel three
	DMACH3ModeConfig(3, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel four
	DMACH4AddrConfig((volatile Uint16 *)(&gains[0]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT3));

	// send one 16 bit word in a burst
	DMACH4BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH4TransferConfig(1, 0, 0);

	// configure DMA channel four
	DMACH4ModeConfig(4, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel five
	DMACH5AddrConfig((volatile Uint16 *)(&analog_ins[1][0]), (volatile Uint16 *)(&AdcbResultRegs.ADCRESULT0));

	// send one 16 bit word in a burst
	DMACH5BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH5TransferConfig(1, 0, 0);

	// configure DMA channel five
	DMACH5ModeConfig(6, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel six
	DMACH6AddrConfig((volatile Uint16 *)(&analog_ins[1][1]), (volatile Uint16 *)(&AdcbResultRegs.ADCRESULT1));

	// send one 16 bit word in a burst
	DMACH6BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH6TransferConfig(1, 0, 0);

	// configure DMA channel six
	DMACH6ModeConfig(7, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

}

// move this code to RAM
#pragma CODE_SECTION(pid_loop, ".TI.ramfunc");
void pid_loop(int16 errors[NUM_OUTPUTS],
			  int16 integrals[NUM_OUTPUTS],
			  int16 derivatives[NUM_OUTPUTS],
			  int16 previous_errors[NUM_OUTPUTS])
{
	// start the ADC SOCs
	AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC2 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC3 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC2 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC3 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC4 = 1;

	// wait until the first non-DMA ADC interrupt pulse is sent
	while (AdcbRegs.ADCINTFLG.bit.ADCINT3 == 0);

	// reset the first non-DMA ADC interrupt register
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;

	// store the first non-DMA ADC result
	output_offsets[1] = AdcbResultRegs.ADCRESULT2;

	// wait until the second non-DMA ADC interrupt pulse is sent
	while (AdcbRegs.ADCINTFLG.bit.ADCINT4 == 0);

	// reset the second non-DMA ADC interrupt register
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;

	// store the second non-DMA ADC result
	gains[1] = AdcbResultRegs.ADCRESULT3;

	// store the third non-DMA ADC result
	// there are not enough interrupt pulses to go around,
	// but it should be okay as dac_max_value isn't really
	// sensitive
	dac_max_value = AdcbResultRegs.ADCRESULT4;

	// perform negative PID on all outputs
	size_t i;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		// input minus input offset
		errors[i] = analog_ins[i][1] - analog_ins[i][0];
		integrals[i] += errors[i] * DT;
		derivatives[i] = (errors[i] - previous_errors[i]) / DT;
		analog_outs[i] = -KP * (gains[i] * (MAX_GAIN - 1) / DELFINO_MAX_VALUE + 1) * errors[i]
						- KI * integrals[i] - KD * derivatives[i] + dac_max_value / 2
						+ output_offsets[i] - DELFINO_MAX_VALUE / 2;
		if (analog_outs[i] > dac_max_value)
			analog_outs[i] = dac_max_value;
		previous_errors[i] = errors[i];
	}

	// output on the DACs if the hold signal is off
	if (GPIO_ReadPin(HOLD_OUTPUT_GPIO_PIN_A) == 0)
		DacaRegs.DACVALS.bit.DACVALS = analog_outs[0];
	if (GPIO_ReadPin(HOLD_OUTPUT_GPIO_PIN_B) == 0)
		DacbRegs.DACVALS.bit.DACVALS = analog_outs[1];

	// output the first error signal
	if (errors[0] < 0 && -errors[0] < dac_max_value / 2)
		DaccRegs.DACVALS.bit.DACVALS = errors[0] + dac_max_value / 2;
	else if (errors[0] < 0)
		DaccRegs.DACVALS.bit.DACVALS = 0;
	else if (errors[0] < dac_max_value / 2)
		DaccRegs.DACVALS.bit.DACVALS = errors[0] + dac_max_value / 2;
	else
		DaccRegs.DACVALS.bit.DACVALS = dac_max_value;

	// turn on LEDs if locked
	if (errors[0] < LED_THRESHOLD && -errors[0] < LED_THRESHOLD)
		GPIO_WritePin(LED_GPIO_PIN_A, 1);
	else
		GPIO_WritePin(LED_GPIO_PIN_A, 0);
	if (errors[1] < LED_THRESHOLD && -errors[1] < LED_THRESHOLD)
		GPIO_WritePin(LED_GPIO_PIN_B, 1);
	else
		GPIO_WritePin(LED_GPIO_PIN_B, 0);
}

int main(void)
{
	// declare PID arrays
	int16 errors[NUM_OUTPUTS];
	int16 integrals[NUM_OUTPUTS];
	int16 derivatives[NUM_OUTPUTS];
	int16 previous_errors[NUM_OUTPUTS];

	// initialize system control
	InitSysCtrl();

	// initialize GPIO
	InitGpio();
	GPIO_SetupPinMux(HOLD_OUTPUT_GPIO_PIN_A, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(LED_GPIO_PIN_A, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(HOLD_OUTPUT_GPIO_PIN_B, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(LED_GPIO_PIN_B, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(HOLD_OUTPUT_GPIO_PIN_A, GPIO_INPUT, GPIO_PULLUP);
	GPIO_SetupPinOptions(LED_GPIO_PIN_A, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinOptions(HOLD_OUTPUT_GPIO_PIN_B, GPIO_INPUT, GPIO_PULLUP);
	GPIO_SetupPinOptions(LED_GPIO_PIN_B, GPIO_OUTPUT, GPIO_PUSHPULL);

	// disallow CPU interrupts
	DINT;

	// initialize PIE interrupts
	InitPieCtrl();

	// disable PIE interrupts
	IER = 0x0;

	// clear all PIE interrupts
	IFR = 0x0;

	// populates the PIE vector table for debug purposes
	InitPieVectTable();

	// enable the PIE block
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

	// enable DMA interrupts
	PieCtrlRegs.PIEIER7.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx2 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx3 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx4 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx5 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx6 = 1;
	IER = M_INT7;

	// enable global interrupts
	EINT;

	// enable real time debug interrupts
	ERTM;

	// map DMA interrupts to functions
	EALLOW;
	PieVectTable.DMA_CH1_INT= &dma_ch1_int_isr;
	PieVectTable.DMA_CH2_INT= &dma_ch2_int_isr;
	PieVectTable.DMA_CH3_INT= &dma_ch3_int_isr;
	PieVectTable.DMA_CH4_INT= &dma_ch4_int_isr;
	PieVectTable.DMA_CH5_INT= &dma_ch5_int_isr;
	PieVectTable.DMA_CH6_INT= &dma_ch6_int_isr;
	EDIS;

	// copy the necessary code from flash memory to RAM
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)(&RamfuncsLoadSize));

	// setup flash wait states
    InitFlash_Bank0();
    InitFlash_Bank1();

	// configure the ADCs
	configure_adcs();

	// setup acquisition on ADCINs
	setup_adc_channels();

	// configure the DACs
	configure_dacs();

	// configure DMA
	configure_dma();

	// set all variables to default values
	size_t i;
	size_t j;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		for (j = 0; j < NUM_INPUTS_PER_OUTPUT; j++)
			analog_ins[i][j] = 0;

		analog_outs[i] = 0;
		errors[i] = 0;
		integrals[i] = 0;
		derivatives[i] = 0;
		previous_errors[i] = 0;
	}
	dac_max_value = DELFINO_MAX_VALUE;

	// start the DMA channels
	StartDMACH1();
	StartDMACH2();
	StartDMACH3();
	StartDMACH4();
	StartDMACH5();
	StartDMACH6();

	while (1) {
		// run the PID loop
        pid_loop(errors, integrals, derivatives, previous_errors);

		// delay for a time DT
		DELAY_US(100000*DT);
	}
}
