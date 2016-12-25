
#include "DSP2803x_Device.h"     // DSP280x Headerfile Include File
#include "DSP2803x_Examples.h"     // DSP280x Examples Include File
#include "DSP2803x_I2C_defines.h"
//#include "I2CMaster.h"
//#include "PMBusMaster.h"
//#include "PMBUS.h"
#include <stdio.h>

static unsigned char *I2CMaster_ReceiveField;
static unsigned char *I2CMaster_TransmitField;
unsigned char bus_read;
//unsigned char bus_write;
int mode=0;

unsigned char alert = 0;
// defines para dps n estar a mudar o codigo em baixo
#define I2C_SLAVE_ADDR 0x1B	//configuravel pelos pinos ADDR0 e ADDR1 do TPS40422
//(ref- datasheet do TPS40422) no caso do EVM o address ja está especificado (ref - datasheet do EVM)
#define CLK_PRESCALE	23
//Para uma frequencia de 100kHz (ref - section 3.1.3 SMBus Specification
//master frequency = 60000/[(prescale+1)*25] kHz


void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}

// interrupt do smbus alert
interrupt void xint1_isr(void)
{
	alert = 1;	//received an alert
	XIntruptRegs.XINT1CR.bit.ENABLE = 0;		//Disable XINT1 interrupt
	while(1){
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
		DELAY_US(5000);
	}
}
// interrupt para os 35 ms
interrupt void cpu_timer0_isr(void)
{
	// Timed out. Reset module.
	I2caRegs.I2CMDR.bit.IRS = 0;	//reset
   	// Acknowledge this interrupt to receive more interrupts from group 1
   	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void i2c_master_int1a_isr(void)     // I2C-A
{
	Uint16 I2CMaster_IntSource;

   // Read interrupt source
	I2CMaster_IntSource = I2caRegs.I2CISRC.bit.INTCODE & 0x7;

	switch(I2CMaster_IntSource)
	{
		case I2C_NO_ISRC:   // =0
			break;

		case I2C_ARB_ISRC:  // =1
			break;

		case I2C_NACK_ISRC: // =2
			break;

		case I2C_ARDY_ISRC: // =3
			break;

		case I2C_RX_ISRC:   // =4
			StopCpuTimer0();		//No timeout, so stop the timer
			ReloadCpuTimer0();		//Reload the period value (35 ms timeout)
			//se for para ler um byte
			if(mode==0){
				bus_read = I2caRegs.I2CDRR;
			}


			break;

		case I2C_TX_ISRC:   // =5
			break;

		case I2C_SCD_ISRC:  // =6
			break;

		case I2C_AAS_ISRC:  // =7
			break;

		default:
			asm("   ESTOP0"); // Halt on invalid number.
	}

   // Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

unsigned char is_slave_present(unsigned char SlaveAddress)
{
  unsigned char ACK;
  I2caRegs.I2CSAR = SlaveAddress;
  I2caRegs.I2CCNT = 0x01; 					//prepare to send a dummy byte
  I2caRegs.I2CDXR = 0x00;					//dummy byte to be written
  // I2C TX, start condition
  I2caRegs.I2CMDR.bit.XA=0;
  I2caRegs.I2CMDR.bit.FREE=1;
  I2caRegs.I2CMDR.bit.STT=1;
  I2caRegs.I2CMDR.bit.STP=1;
  I2caRegs.I2CMDR.bit.MST=1;
  I2caRegs.I2CMDR.bit.TRX=1;
  I2caRegs.I2CMDR.bit.IRS=1;
  while (I2caRegs.I2CMDR.bit.STP == 1);     // wait for STOP condition
  ACK = !I2caRegs.I2CSTR.bit.NACK;	// Not aknowledge, é 1 quando o bit n foi recebido pelo slave
  return ACK;                         		// return se o valor do nack
}

void PMBusMaster_Init(unsigned char pmbusmaster_slaveaddr, unsigned char prescaler ){

		unsigned char slave_address;

		// Control Line functionality GPIO0 - Linha 1 para o 1º buck
		GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;	//pullup
		GpioDataRegs.GPASET.bit.GPIO0 = 1;	//Por a 1
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;	//GPIO0 = output

		//Control Line functionality GPIO1 - Linha 2 para o 2º buck
		GpioCtrlRegs.GPAPUD.bit.GPIO1=0;
		GpioDataRegs.GPASET.bit.GPIO1 =1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;
		GpioCtrlRegs.GPADIR.bit.GPIO1= 1;

		//inicialização smbus alert (Gpio2)
		GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;		//Enable pullup
		GpioDataRegs.GPASET.bit.GPIO2 = 1;		//Drive line high
		GpioCtrlRegs.GPAQSEL1.bit.GPIO2 = 0;	//SYNC to SYSCLKOUT
		GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0;	//no qualification (SYNC to SYSCLKOUT)
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;		//GPIO2 = input

		//led do interrupt
		EALLOW;
		GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
		GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
		EDIS;


		//setup interrupt triggered by Alert line
		EALLOW;									// This is needed to write to EALLOW protected registers
		PieVectTable.XINT1 = &xint1_isr;
		EDIS;									// This is needed to disable write to EALLOW protected registers 			GpioIntRegs.GPIOXINT1SEL.all = 2;		//Make GPIO2 input source for XINT1
		XIntruptRegs.XINT1CR.bit.POLARITY = 2;	//XINT1 triggered by falling edge (high-to-low-transition)

		slave_address =pmbusmaster_slaveaddr;


		//inicialização do i2c
		Uint16 I2CMaster_SlaveAddress = slave_address;
		Uint16 I2C_prescaler = prescaler;

		EALLOW;	// This is needed to write to EALLOW protected registers
		PieVectTable.I2CINT1A = &i2c_master_int1a_isr;
		PieVectTable.TINT0 = &cpu_timer0_isr;
		EDIS;   // This is needed to disable write to EALLOW protected registers


		// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
		PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
		// Enable TINT0 in the PIE: Group 1 interrupt 7
	   	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	   	// Enable CPU INT8 which is connected to PIE group 8
		IER |= M_INT8;
		// Enable CPU int1 which is connected to CPU-Timer 0
		IER |= M_INT1;

		InitI2CGpio();
		InitCpuTimers();

		ConfigCpuTimer(&CpuTimer0, 60, 35000);			//CPU Timer 0 interrupt after 35 ms (at 60MHz CPU freq.)

	    // Initialize I2C
		I2caRegs.I2CSAR = I2CMaster_SlaveAddress;		// Slave Address.
		I2caRegs.I2COAR = 0x002D;       				// address as Master.
		I2caRegs.I2CPSC.all = I2C_prescaler;			// Prescaler - need 7-12 Mhz on module clk
		I2caRegs.I2CCLKL = 10;							// NOTE: must be non zero
		I2caRegs.I2CCLKH = 5;							// NOTE: must be non zero
	    I2caRegs.I2CIER.all = 0x2C;						// Enable SCD & ARDY interrupts

	    I2caRegs.I2CMDR.bit.IRS = 1;					// Take I2C out of reset
		   												// Stop I2C when suspended
	    I2caRegs.I2CFFTX.all = 0x6000;					// Enable FIFO mode and TXFIFO


		//verifica se o slave esta na linha se não está prende
		//while(!is_slave_present(pmbusmaster_slaveaddr));

		// Enable Alert line interrupt
		XIntruptRegs.XINT1CR.bit.ENABLE = 1;	//Enable XINT1 interrupt
		// Enable XINT1 interrupt in the PIE: Group 1 interrupt 4
		PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
		// Enable CPU INT1 which is connected to PIE group 1
		IER |= M_INT1;

		// enquanto a linha estiver ocupada  não avança
		while(I2caRegs.I2CSTR.bit.BB);
}

unsigned char master_read_byte(unsigned char slave_address,unsigned char command){

	unsigned char received_byte = 0x01;
	mode=0;
	//Transmissão de dados
	I2caRegs.I2CSAR=slave_address;// address do slave
	I2caRegs.I2CCNT=0x01; //tamanho do comando, so enviamos um byte uma vez que é uma funcão de read
	I2caRegs.I2CDXR=command;//bits a ser enviados

	I2caRegs.I2CMDR.bit.NACKMOD=0;//
	I2caRegs.I2CMDR.bit.FREE=1;// continua a correr mesmo que haja breakpoints
	I2caRegs.I2CMDR.bit.STT=1;// Gera o start condition no clock
	I2caRegs.I2CMDR.bit.STP=1; // Gera o stop conditon
	I2caRegs.I2CMDR.bit.MST=1;// Determina se o modulo é um slave ou um Master
	I2caRegs.I2CMDR.bit.TRX=1;// Diz que está a transmitir
	I2caRegs.I2CMDR.bit.XA=0; // 7 bits (1= 10 bits) de address
	I2caRegs.I2CMDR.bit.RM=0; // Ver tabela 6 do pdf sprufz9d
	I2caRegs.I2CMDR.bit.IRS=1;//bit de reset
	I2caRegs.I2CMDR.bit.STB=0;
	I2caRegs.I2CMDR.bit.FDF=0;

	/**********************tabela para os Bits RM STT e STP***********/
	//RM	STT		STP
	//0		0		0		No activity
	//0		1		1		A trama contém Start, slave address, n bytes (valor declarado no registo I2CCNT) e tem stop condition
	//1		1		0		A trama contêm Start , slave addr, n bytes(enviados) sem stop condition

	StartCpuTimer0(); //35 ms
	while(I2caRegs.I2CMDR.bit.STP==1);
	while(I2caRegs.I2CSTR.bit.BB==1);
	StopCpuTimer0();
	ReloadCpuTimer0();

	//Recepção de dados
	//recebe 1 byte
	I2caRegs.I2CCNT=0x01;
	//received_byte=I2caRegs.I2CDRR;

	I2caRegs.I2CMDR.bit.NACKMOD=0;//
	I2caRegs.I2CMDR.bit.FREE=1;// continua a correr mesmo que haja breakpoints
	I2caRegs.I2CMDR.bit.STT=0;// Gera o start condition no clock
	I2caRegs.I2CMDR.bit.STP=1; // Gera o stop conditon
	I2caRegs.I2CMDR.bit.MST=1;// Determina se o modulo é um slave ou um Master
	I2caRegs.I2CMDR.bit.TRX=0;// Diz que está a transmitir
	I2caRegs.I2CMDR.bit.XA=0; // 7 bits (1= 10 bits) de address
	I2caRegs.I2CMDR.bit.RM=0; // Ver tabela 6 do pdf sprufz9d
	I2caRegs.I2CMDR.bit.RM=0; // Ver tabela 6 do pdf sprufz9d
	I2caRegs.I2CMDR.bit.IRS=1;//bit de reset
	I2caRegs.I2CMDR.bit.STB=0;
	I2caRegs.I2CMDR.bit.FDF=0;
	I2caRegs.I2CMDR.bit.BC=0;

	StartCpuTimer0();
	while(I2caRegs.I2CMDR.bit.STP==1);
	while(I2caRegs.I2CSTR.bit.BB==1);
	return received_byte;
}
void master_write_byte(unsigned char slave_address,unsigned char command,unsigned char master_byte){

	Uint16 byte_count=2;
	//Transmissão de dados
	I2caRegs.I2CSAR=slave_address;// address do slave
	I2caRegs.I2CCNT=0x03; //tamanho do comando, so enviamos um byte uma vez que é uma funcão de read
	while(byte_count>0)
	{
		if(byte_count==2){
			I2caRegs.I2CDXR=command;//bits a ser enviados
			byte_count--;
		}
		if(byte_count==1){
			I2caRegs.I2CDXR=master_byte;
			byte_count--;
		}
	}
	I2caRegs.I2CMDR.bit.NACKMOD=0;//
	I2caRegs.I2CMDR.bit.FREE=1;// continua a correr mesmo que haja breakpoints
	I2caRegs.I2CMDR.bit.STT=1;// Gera o start condition no clock
	I2caRegs.I2CMDR.bit.STP=1; // Gera o stop conditon
	I2caRegs.I2CMDR.bit.MST=1;// Determina se o modulo é um slave ou um Master
	I2caRegs.I2CMDR.bit.TRX=1;// Diz que está a transmitir
	I2caRegs.I2CMDR.bit.XA=0; // 7 bits (1= 10 bits) de address
	I2caRegs.I2CMDR.bit.RM=0; // Ver tabela 6 do pdf sprufz9d
	I2caRegs.I2CMDR.bit.IRS=1;//bit de reset
	I2caRegs.I2CMDR.bit.STB=0;
	I2caRegs.I2CMDR.bit.FDF=0;
	StartCpuTimer0(); //35 ms
	while(I2caRegs.I2CMDR.bit.STP==1);
	while(I2caRegs.I2CSTR.bit.BB==1);
	StopCpuTimer0();
	ReloadCpuTimer0();

}
void master_write_word(unsigned char slave_address,unsigned char command,unsigned char lower, unsigned char high){

	Uint16 byte_count=3;
	//Transmissão de dados
	I2caRegs.I2CSAR=slave_address;// address do slave
	I2caRegs.I2CCNT=0x03; //tamanho do comando, so enviamos um byte uma vez que é uma funcão de read
	while(byte_count>0)
	{
		if(byte_count==3){
			I2caRegs.I2CDXR=command;//bits a ser enviados
			byte_count--;
		}
		if(byte_count==2){
			I2caRegs.I2CDXR=lower;
			byte_count--;
		}
		if(byte_count==1){
			I2caRegs.I2CDXR=high;
			byte_count--;
		}

	}
	I2caRegs.I2CMDR.bit.NACKMOD=0;//
	I2caRegs.I2CMDR.bit.FREE=1;// continua a correr mesmo que haja breakpoints
	I2caRegs.I2CMDR.bit.STT=1;// Gera o start condition no clock
	I2caRegs.I2CMDR.bit.STP=1; // Gera o stop conditon
	I2caRegs.I2CMDR.bit.MST=1;// Determina se o modulo é um slave ou um Master
	I2caRegs.I2CMDR.bit.TRX=1;// Diz que está a transmitir
	I2caRegs.I2CMDR.bit.XA=0; // 7 bits (1= 10 bits) de address
	I2caRegs.I2CMDR.bit.RM=0; // Ver tabela 6 do pdf sprufz9d
	I2caRegs.I2CMDR.bit.IRS=1;//bit de reset
	I2caRegs.I2CMDR.bit.STB=0;
	I2caRegs.I2CMDR.bit.FDF=0;
	StartCpuTimer0(); //35 ms
	while(I2caRegs.I2CMDR.bit.STP==1);
	while(I2caRegs.I2CSTR.bit.BB==1);
	StopCpuTimer0();
	ReloadCpuTimer0();

}
void main(void){

	//Inicialização do controlo do sistema, encontra-se no DSP2803x_SysCtrl.c
	// watchdog etc
	InitSysCtrl();

	// Copy time critical code and Flash setup code to RAM
	// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

	//Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP280x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP280x_DefaultIsr.c.
	// This function is found in DSP280x_PieVect.c.
	InitPieVectTable();

	//init pinos debug
	//init_led();
	// init da biblioteca do PMBus
	PMBusMaster_Init(I2C_SLAVE_ADDR, CLK_PRESCALE);

	EINT;

	//Ler qual é a revisão do pmbus (0x98)
	//verifica que a linha esta ocupada
	//while(I2caRegs.I2CSTR.bit.BB==1);
	unsigned char byte =master_read_byte(I2C_SLAVE_ADDR,0x98);
	//master_write_byte(I2C_SLAVE_ADDR,0xD8,0x20);
	while(1){
		//puts("Hello world\n");
	}
}


