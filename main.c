/*
 * SPVP
 *
 * author: 	Shura Osechkin
 * date: 	23.07.2019
 */

#include "time.h"

#include "Timer/clocker_drv.h"
#include "Timer/timer_drv.h"

#include "upp/upp.h"

#include "proger/proger.h"

#include "GPIO/gpio.h"
#include "uart_hduplex/uart_hduplex.h"

#include "UART/uart_messages.h"
#include "UART/UART_drv.h"

#include "nor/nor.h"

#include "Common/io_containers.h"
#include "Math/data_processing.h"
#include "Math/nmr_math.h"


//#define USE_TIMING

//#define USE_DIELEC_UART
#define USE_TELEMETRIC_UART
#define USE_PRESSURE_UNIT


#define INTC_Timer			4
#define INTC_GPIO			5
#define INTC_UPP			6
#define INTC_UART			7
#ifdef USE_TELEMETRIC_UART
#define INTC_UART_Tele		9
#endif


//#pragma DATA_SECTION(upp_buffer_data, "l2ram_data");
#pragma DATA_SECTION(upp_buffer_page1, "l2ram_data");		// 4096 + 2*PAD
#pragma DATA_SECTION(data_org, "l2ram_data");				// 8192 + 2*PAD
#pragma DATA_SECTION(data1, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data2, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data3, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data4, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data5, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data6, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data7, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(temp_data, "l2ram_data");				// 8192 + 2*PAD
#pragma DATA_SECTION(data_nmr, "sharedram_data");			// 8192 + 2*PAD
#pragma DATA_SECTION(data_sum, "sharedram_data");			// 8192 + 2*PAD
#pragma DATA_SECTION(data_fin, "sharedram_data");
#pragma DATA_SECTION(w, "l2ram_data");						// 8192 + 2*PAD
#pragma DATA_SECTION(brev, "l2ram_data");					// 64


extern void intcVectorTable(void);


unsigned char *upp_buffer;									// указатель на текущую страницу в upp_buffer_data (см. io_containers.c)
unsigned char upp_buffer_page1[UPP_BUFF_SIZE]; 				// буфер, куда передаются данные ЯМР (из АЦП) по быстрому каналу UPP (страница 1)
float data_org[DATA_MAX_LEN + 2 * PAD]; 					// буфер 0 (D0) для хранения и обработки данных
float data1[DATA_MAX_LEN + 2 * PAD]; 						// буфер 1 (D1) для хранения и обработки данных
float data2[DATA_MAX_LEN + 2 * PAD]; 						// буфер 2 (D2) для хранения и обработки данных
float data3[DATA_MAX_LEN + 2 * PAD]; 						// буфер 3 (D3) для хранения и обработки данных
float data4[DATA_MAX_LEN + 2 * PAD]; 						// буфер 4 (D4) для хранения и обработки данных
float data5[DATA_MAX_LEN + 2 * PAD]; 						// буфер 5 (D5) для хранения и обработки данных
float data6[DATA_MAX_LEN + 2 * PAD]; 						// буфер 6 (D6) для хранения и обработки данных
float data7[DATA_MAX_LEN + 2 * PAD]; 						// буфер 7 (D7) для хранения и обработки данных
float temp_data[DATA_MAX_LEN + 2 * PAD]; 					// данные ЯМР после третьего этапа обработки (оконная функция в частотной области)
float w[DATA_MAX_LEN + 2 * PAD]; 							// массив поворачивающих множителей
int rad; 													// параметр, используемый для БПФ

float 	*ptr_data_org,										// указатели на буферы данных data_org, data1, data2, data3, temp_data, w
		*ptr_data1,
		*ptr_data2,
		*ptr_data3,
		*ptr_data4,
		*ptr_data5,
		*ptr_data6,
		*ptr_data7,
		*ptr_temp_data,
		*ptr_w;
int 	len_data_org, 										// длины данных, хранящиеся в буферах data_org, data1, data2, data3, data4, data5, data6, data7
		len_data1,
		len_data2,
		len_data3,
		len_data4,
		len_data5,
		len_data6,
		len_data7;
float *bank[10]; 											// = { ptr_data_org, ptr_data1, ptr_data2, ptr_data3, ptr_data4, ptr_data5, ptr_data6, ptr_data7 };
															// контейнер с указателями на все буферы данных

//volatile Bool input_data_enabled; 							// флаг, равный True, если прием данных разрешен
volatile Bool fpga_prg_started; 							// флаг, равный True, если программа на ПЛИС запущена (если команда NMR_TOOL_START была получена)

volatile NMRToolState tool_state = UNKNOWN_STATE; 			// флаг, устанавливаемый: - в READY, когда заканчивается обработка всех данных (фурье и пр.),
															// - в FREE, когда ЯМК готов к приему команд по UART и к передаче данных по UART к управляющей станции оператора,
															// - в BUSY, когда ЯМК "занят", т.е. производит измерение и/или обработку сигнала ЯМР

unsigned char data_fin[ALLDATA_BUFF_SIZE]; 					// буфер для всех данных, передаваемых в рабочую станцию оператора (данные ЯМР + диэл. данные + мониторинг +...)
int data_fin_counter = 0; 									// счетчик данных (типа unsigned char) в буфере data_fin
float data_nmr[ALLDATA_BUFF_LEN + 2 * PAD]; 				// буфер для накопления данных измерений перед помещением данных в выходной буфер (затем передача по кабелю)
float *ptr_data_nmr; 										// указатель на буфер data_nmr
int data_nmr_counter = 0; 									// счетчик данных (типа float) в буфере data_nmr
float data_sum[ALLDATA_BUFF_LEN + 2 * PAD]; 				// буфер для последовательного (поточечно по мере их измерения) накопления данных перед помещением их в выходной буфер
float *ptr_data_sum; 										// указатель на буфер data_sum

OutBuffer *output_data; 									// контейнер выходных данных для последующей записи в data_fin и передачи в рабочую станцию оператора
SummationBuffer *summ_data; 								// контейнер для хранения данных, получаемых поточечно (т.е. по одной точке с каждого эхо)


// Clocker Objects ------------------------------------------------------------
Clocker **clockers; 										// Array of Clocker objects
Clocker *clocker0; 											// Clocker for Pressure Unit meassurements
Clocker *clocker1; 											// Clocker for incoming UART messages (for header of message)
Clocker *clocker2; 											// Clocker for incoming UART messages (for body of message)
Clocker *clocker3; 											// Clocker for "Data_ready" message
Clocker *clocker4; 											// Clocker for telemetry measurements
Clocker *clocker5; 											// Clocker for SDSP measurements (~200 ms)
// ----------------------------------------------------------------------------

// UART variables -------------------------------------------------------------
//volatile int uartStatus;
volatile Bool dataUnavailable;
volatile Bool transmitterFull;

CSL_UartRegsOvly uartRegs;
UART_Settings uartSettings;
#ifdef USE_TELEMETRIC_UART
CSL_UartRegsOvly uartRegs_Telemetric;
UART_Settings uartSettings_Telemetric;
#endif

QUEUE8 *uart_queue;

//-----------------------------------------------------------------------------

// Timer variables ------------------------------------------------------------
CSL_TmrRegsOvly tmrRegs;
Timer_Settings timerSettings; 								//Timer_Settings timerSettings1;
// ----------------------------------------------------------------------------


// UART Messages --------------------------------------------------------------
GF_Data *gf_data;

uint8_t out_msg_number = 1; 								// номер исходящих сообщений (предполагается сквозная нумерация)
MsgHeader *in_msg_header; 									// заголовок принимаемого сообщения
MsgHeader *out_msg_header; 									// заголовок отправляемого сообщения
MsgCommSettings *msg_settings; 								// параметры для передачи сообщений: длина блоков, количество восстанавливаемых ошибок и т.д.
QUEUE8 *head_q; 											// контейнер для заголовка сообщения, принимаемого из UART
BUFFER8 *body_q;
UART_Message in_msg; 										// принимаемое UART-сообщение
UART_Message out_msg; 										// отправляемое UART-сообщение
GetDataReq data_req; 										// требуемые оператором данные

volatile MultyStateIn msg_header_state = NOT_DEFINED; 		// состояние приема заголовка сообщения (msg_header)
volatile MultyStateIn incom_msg_state = NOT_DEFINED; 		// состояние всего принимаемого сообщения
volatile MultyStateOut outcom_msg_state = NOT_BUILT; 		// состояние всего отправляемого сообщения
volatile int pack_counter = 0; 								// счетчик приходящих пакетов принимаемого сообщения
uint8_t msg_was_treated = 0;								// флаг, указывающий на успех/неуспех приема и обработки многопакетного сообщения (= 0 или = коду ошибки, см. MultiPackMsg_Err)
// ----------------------------------------------------------------------------

// Tool Channels --------------------------------------------------------------
static volatile unsigned int device_serial = 0;
ToolChannel *device_channels = NULL;
int device_channel_count = 0;
Bool device_settings_OK = False;		// = True if ToolChannel data was recieved successfully.
// ----------------------------------------------------------------------------

// UPP variables and functions ------------------------------------------------
volatile unsigned int reg1 = 0, reg2 = 0, reg3 = 0;
unsigned short byte_count = UPP_BUFF_SIZE, line_count = 1; 	// max value for both is 64*1024-1
volatile unsigned int upp_int_status = 0;
volatile Bool uppFull;
//static volatile unsigned int upp_isr_count = 0;
volatile Bool modulesEnabled;
//volatile unsigned int pins_reg = 0, pins_reg_prev = 0;

volatile Bool upp_resetted = False;

volatile uint8_t channel_id = 0;
volatile uint8_t device_id = 0;				// идентификатор устройства, данные которого обрабатываются по сигналу GPIO GP[1]

CSL_UppRegsOvly UPP0Regs = (CSL_UppRegsOvly) (CSL_UPP_0_REGS);
CSL_UppRegsOvly UPPRegs;

volatile uint32_t pins_cmd = 0xFF;
volatile uint32_t pins_reg = 0xFF;
volatile uint8_t pin1_state = 0x00;
volatile uint8_t pin3_state = 0x00;
// ----------------------------------------------------------------------------


// GPIO variables -------------------------------------------------------------
CSL_GpioRegsOvly GPIORegs;
// ----------------------------------------------------------------------------


// Telemetric variables -------------------------------------------------------
#ifdef USE_TELEMETRIC_UART
uint8_t telemetric_data[TELEMETRIC_UART_BUF_LEN]; 			// контейнер для телеметрических данных
volatile unsigned int UART_telemetric_counter = 0; 			// счетчик байт, приходящих от плат телеметрии
volatile unsigned int UART_telemetric_pack_counter = 0; 	// счетчик пакетов длиной TELEMETRIC_DATA_LEN байт, приходящих от плат телеметрии
volatile unsigned int UART_telemetric_local_counter = 0; 	// локальный счетчик для подсчета данных внутри пакета
volatile uint8_t telemetric_board_status = 0;
volatile TelemetryState telemetry_ready = TELE_NOT_READY; 	// флаг готовности телеметрической информации

volatile int temperature_mode = TEMP_MODE_UNKNOWN;			// тип прибора (KMRK, NMKT и SDSP имеют разный способ измерения температуры): 1 - KMRK, NMKT и другие; 2 - SPVP
volatile int temp_request_mode = TEMP_NOT_DEFINED;			// 0 - исходное состояние, 1 - получен ответ на запуск измерения температуры, 2 - получены температуры (только для SPVP)
volatile int temp_sensors = 0;								// количество сенсоров (только для SPVP)
volatile int voltage_request_mode = VOLT_NOT_DEFINED;		// 0 - исходное состояние, 1 - запуск измерений напряжений, 2 - напряжения измерены (только для SPVP)
#endif
// ---------------------------------------------------------------------------


// Pressure Unit variables ---------------------------------------------------
#ifdef USE_PRESSURE_UNIT
unsigned int press_unit_data;								// контейнер для данных прижимного устройства
volatile PressureUnitState press_unit_ready = PRESS_UNIT_NOT_READY;	// флаг готовности измерить данные прижимного устройства
#endif
// ---------------------------------------------------------------------------


// Data Samples --------------------------------------------------------------
//#define DATA_SAMPLES				(1000)

#define SAMPLES_BUFFER_SIZE			(0x800000)
//uint8_t **samples_buffer;									// буфер данных АЦП для их последующей обработки
uint8_t *samples_buffer;									// буфер данных АЦП для их последующей обработки. Массив объектов, содержащий полученные из АЦП данные и описание к ним типа DataSample.
															// Длина эхо может быть разная - поэтому заранее не известно, сколько можно разместить эхо в области памяти samples_buffer.
															// При достижении предела выделенной под samples_buffer памяти, новые эхо записываться не будут
volatile uint32_t samples_buffer_pos;						// указатель на текущую позицию в samples_buffer
volatile uint32_t current_data_sample;						// текущий элемент в data_samples (счетчик готовых, но еще необработанных данных)
DataSample *data_samples[UPP_DATA_COUNT];					// массив объектов, содержащий указатели на полученные из АЦП данные (в samples_buffer) и описание к этим данным (см. DataSample).
															// Длина эхо может быть разная - поэтому заранее не известно, сколько можно разместить эхо в области памяти samples_buffer. Под объекты DataSample памяти выделено с некоторым избытком.

volatile int ds_new_data_enabled = 1;						// 0 - новые данные поступают в данный момент, но еще не инициализированы в буфере data_samples; 1 - данные в данный момент не поступают
//volatile int ds_proc_data_index = 0;						// номер обрабатываемых в данный момент данных в буфере data_samples
volatile int seq_completed = -1; 							// 1 = последовательность завершилась; 0 = не завершилась; -1 = неопределенное состояние
volatile int nmr_data_ready = 0;							// 1 = прием всех эхо ЯМР завершился (можно приступить к обработке); 0 = прием данных ЯМР (эхо) не завершился или измерений ЯМР не производится
// ---------------------------------------------------------------------------


// Processing variables ------------------------------------------------------
Processing_Params *processing_params;
Data_Proc *instr_prg; 										// блоки инструкций для обработки данных
Data_Cmd *instr;
STACKPtrF *data_stack; 										// стек для данных (массивов) типа float: data1, data2, data3, ...
float XX[XX_LEN]; 											// ячейки памяти X0, X1, X2, X3
// ---------------------------------------------------------------------------


volatile int app_finish = 0;

clock_t t_start, t_stop, t_overhead;

int i;

#ifdef USE_TIMING
TimingProc_Buffer timing_buffer;
#endif


void create_Clockers(void);

void onDataAvailable(QUEUE8* bytes);
void init_UART_MsgData(void);

void initDeviceSettings(uint8_t device);
Bool loadDeviceSettings(int *data, int len);
void setDefaultCommSettings();

/*
void initDataHeap();								// инициализация контейнера долговременного хранения данных (в куче)
void resetDataHeap();								// сброс информации по Data heap
*/
void initDataSamples(DataSample *data_samples[]);
//void resetDataSamples(int _from, int _count);

void sendByteArray(uint8_t *arr, uint16_t len, CSL_UartRegsOvly uartRegs);
void sendServiceMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);
void sendShortMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);
void sendMultyPackMsg(UART_Message *uart_msg, CSL_UartRegsOvly uartRegs);
void sendHeader(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);

void executeServiceMsg(MsgHeader *_msg_header);
void executeShortMsg(MsgHeader *_msg_header);
void executeMultypackMsg(UART_Message *uart_msg);
void responseMultypackHeader(MsgHeader *_msg_header);
Bool extractDataFromPacks(UART_Message *uart_msg, uint8_t *arr, uint16_t *len);

void prepareOutputByteArray(OutBuffer *out_buff, SummationBuffer *sum_buff);
#ifdef USE_TELEMETRIC_UART
void telemetryDataToOutput(OutBuffer *out_buff);
#endif
#ifdef USE_PRESSURE_UNIT
void pressureUnitDataToOutput(OutBuffer *out_buff);
#endif
void summationDataToOutput(OutBuffer *out_buff, SummationBuffer *sum_buff);

//void executeProcPack(DataSample **dss, int ds_index, Data_Proc *proc, int index);
void executeProcPack(Data_Proc *proc, int index);
//int newDataAvailable();

#ifdef USE_TELEMETRIC_UART
void toMeasureTemperatures(void);
#endif

// **************************************************************************************
// *************** Main Part of NMR_Tool2 ***********************************************
void main(void)
{
	CacheALL_disable();
	srand(time(NULL));

	shutdown_ARM();
	disableARM();

	_disable_interrupts();


	// ********** Init Devices **********************************************************

	// Timer settings and Initialization ------------------------------------------------
	tmrRegs = tmr0Regs; 									// add Timer0 to application
	timerSettings.freq = 24000u;
	setup_Timer(tmrRegs, timerSettings);					printf("System timer was initialized.\n");
	setup_Timer_INTC(tmrRegs, INTC_Timer);					printf("Timer system interrupt was mapped to DSP INT%d.\n", INTC_Timer);

	create_Clockers();										// Init Clockers
	// ----------------------------------------------------------------------------------


	// GPIO system settings and Initialization ------------------------------------------
	modulesEnabled = FALSE;

	// Ensure previous initiated transitions have finished
	if(check_psc_transition(CSL_PSC_1) == pscTimeout) return;

	// Enable peripherals; Initiate transition
	CSL_FINST(psc1Regs->MDCTL[CSL_PSC_GPIO], PSC_MDCTL_NEXT, ENABLE);
	CSL_FINST(psc1Regs->PTCMD, PSC_PTCMD_GO0, SET);

	// Ensure previous initiated transitions have finished
	if(check_psc_transition(CSL_PSC_1) == pscTimeout) return;

	// Ensure modules enabled
	if(check_psc_MDSTAT(CSL_PSC_1, CSL_PSC_GPIO, CSL_PSC_MDSTAT_STATE_ENABLE) == pscTimeout) return;
	modulesEnabled = TRUE;

	int control_pins = 12;
	int *pins = (int*) calloc(control_pins, sizeof(int));
	pins[0] = GP_1; 										// GPIO[1] Bank0
	pins[1] = GP_2; 										// GPIO[2] Bank0
	pins[2] = GP_3; 										// GPIO[3] Bank0
	pins[3] = GP_4; 										// GPIO[4] Bank0
	pins[4] = GP_5; 										// GPIO[5] Bank0
	pins[5] = GP_6; 										// GPIO[6] Bank0
	pins[6] = GP_7; 										// GPIO[7] Bank0
	pins[7] = GP_8; 										// GPIO[8] Bank0
	pins[8] = GP_9; 										// GPIO[9] Bank0
	pins[9] = GP_10; 										// GPIO[10] Bank0
	pins[10] = GP_11; 										// GPIO[11] Bank0
	pins[11] = GP_12; 										// GPIO[12] Bank0
	enableGPIOPinMux_Bank0(pins, control_pins, sysRegs);

	gpioPowerOn(psc1Regs); 									// Set power on the GPIO module in the power sleep controller

	int *pin_dirs = (int*) calloc(control_pins, sizeof(int));
	for (i = 0; i < control_pins; i++)
	{
		pin_dirs[i] = CSL_GPIO_DIR_DIR_IN;
	}
	configureGPIOPins_Bank0(pins, pin_dirs, control_pins, gpioRegs);	// Configure GPIO pins in Bank0 for output

	int *pin_states = (int*) calloc(3, sizeof(int));
	pin_states[0] = GPIO_FAL_AND_RIS;
	pin_states[1] = GPIO_FAL_AND_RIS;
	pin_states[2] = GPIO_FAL_AND_RIS;
	configureGPIOInterrupts_Bank0(pins, pin_states, 3, gpioRegs); // Enable GPIO interrupt (Bank0) for rising and falling

	mapGPIOInterrupt_Bank0(INTC_GPIO, dspintcRegs); 		// map GPIO events to INTC5
	// ----------------------------------------------------------------------------------


	// UART settings --------------------------------------------------------------------
	uartSettings.BaudRate = 115200;
	uartSettings.DataBits = 8;
	uartSettings.StopBits = 1;
	uartSettings.Parity = NO_PARITY;
	uartSettings.LoopBackMode = False;
	//uartSettings.FIFOMode = False;
	uartSettings.FIFOMode = True;
	uartSettings.FIFOLen = 1;

	uartRegs = uart1Regs; 									// add UART1 to application (for BigGreenBoard #1 - UART1 is for communication board)

	reset_UART(uartRegs);
	setup_UART(uartRegs, uartSettings);						printf("UART1 was initialized.\n");
	setup_UART_INTC(uartRegs, INTC_UART);					printf("UART1 system interrupt was mapped to DSP INT%d.\n", INTC_UART);
	// ----------------------------------------------------------------------------------


	// Telemetric UART settings ---------------------------------------------------------
#ifdef USE_TELEMETRIC_UART
	uartSettings_Telemetric.BaudRate = 19200;
	uartSettings_Telemetric.DataBits = 8;
	uartSettings_Telemetric.StopBits = 1;
	uartSettings_Telemetric.Parity = NO_PARITY;
	uartSettings_Telemetric.LoopBackMode = False;
	uartSettings_Telemetric.FIFOMode = True;
	uartSettings_Telemetric.FIFOLen = 1;

	uartRegs_Telemetric = uart2Regs;

	// Telemetric Board UART initialization
	reset_UART(uartRegs_Telemetric);
	setup_UART(uartRegs_Telemetric, uartSettings_Telemetric);	printf("UART2 was initialized for the Telemetric board.\n");
	setup_UART_INTC(uartRegs_Telemetric, INTC_UART_Tele);		printf("UART2 system interrupt was mapped to DSP INT%d.\n", INTC_UART_Tele);
#endif

	// UPP initialization ---------------------------------------------------------------
	//_disable_interrupts();
	//proger_stop();
	//main_proger_wr_pulseprog_default();
	init_upp();													printf("UPP channel was initialized.\n");
	init_upp_ints(); 											printf("UPP system interrupt was mapped to DSP INT%d.\n", INTC_UPP);
	//_enable_interrupts();
	// ----------------------------------------------------------------------------------


	proger_stop();
	device_serial = proger_rd_device_serial();
	initDeviceSettings(device_serial);
	switch (device_serial)
	{
	case NMKT:
	case KMRK:
	case NMR_KERN: 	temperature_mode = TEMP_MODE_NOT_SPVP; break;
	case SPVP:		temperature_mode = TEMP_MODE_SPVP; break;
	default: break;
	}

	temperature_mode = TEMP_MODE_SPVP;		// temporary !!!
	// ----------------------------------------------------------------------------------


	// NOR Flash initialization *********************************************************
	nor_cfg();
	// ----------------------------------------------------------------------------------


	// Enable INTCs ---------------------------------------------------------------------
	int int_count = 4;
	int int_index = 0;
#ifdef USE_TELEMETRIC_UART
	int_count++;
#endif
	int *INTCs = (int*) calloc(int_count, sizeof(int));
	INTCs[int_index++] = INTC_Timer;						// int 4 added for Timer0
	INTCs[int_index++] = INTC_GPIO;							// int 5 added for GPIO
	INTCs[int_index++] = INTC_UPP; 							// int 6 added for UPP
	INTCs[int_index++] = INTC_UART; 						// int 7 added for Logging UART
#ifdef USE_TELEMETRIC_UART
	INTCs[int_index++] = INTC_UART_Tele; 					// int 9 added for Telemetric UART
#endif
	enable_all_INTC(int_count, INTCs);						printf("All system interrupts were enabled.\n");
	// ----------------------------------------------------------------------------------

	_enable_interrupts();
	// Finish (Initialization of Devices) ***********************************************


	// Enable devices ********************************************************************
	enable_Timer(tmrRegs);									printf("System timer was enabled.\n");
	enable_UART(uartRegs);									printf("UART1 for the cable communication was enabled.\n");


#ifdef USE_TELEMETRIC_UART
	memset(telemetric_data, 0x00, TELEMETRIC_UART_BUF_LEN2);
	enable_UART(uartRegs_Telemetric); 						printf("UART2 for the Telemetric board was enabled.\n");		// start operations on Telemetric UART
#endif
	// **********************************************************************************


	// Init variables and structs *******************************************************
	init_UART_MsgData();									// UART message system

	// Data buffers and containers ------------------------------------------------------
	memset(&upp_buffer_page1[0], 0x0, UPP_BUFF_SIZE);
	upp_buffer = &upp_buffer_page1[0];
	memset(&data_org[0], 0x0, UPP_DATA_SIZE);
	memset(&data1[0], 0x0, UPP_DATA_SIZE);
	memset(&data2[0], 0x0, UPP_DATA_SIZE);
	memset(&data3[0], 0x0, UPP_DATA_SIZE);
	memset(&data4[0], 0x0, UPP_DATA_SIZE);
	memset(&data5[0], 0x0, UPP_DATA_SIZE);
	memset(&data6[0], 0x0, UPP_DATA_SIZE);
	memset(&data7[0], 0x0, UPP_DATA_SIZE);
	memset(&temp_data[0], 0x0, UPP_DATA_SIZE);
	memset(&w[0], 0x0, UPP_DATA_SIZE);
	ptr_data_org = data_org + PAD;
	ptr_data1 = data1 + PAD;
	ptr_data2 = data2 + PAD;
	ptr_data3 = data3 + PAD;
	ptr_data4 = data4 + PAD;
	ptr_data5 = data5 + PAD;
	ptr_data6 = data6 + PAD;
	ptr_data7 = data7 + PAD;
	ptr_temp_data = temp_data + PAD;
	ptr_w = w + PAD;
	len_data1 = 0;
	len_data2 = 0;
	len_data3 = 0;
	len_data4 = 0;
	len_data5 = 0;
	len_data6 = 0;
	len_data7 = 0;

	// контейнер с указателями на все буферы данных
	bank[0] = ptr_data_org;
	bank[1] = ptr_data1;
	bank[2] = ptr_data2;
	bank[3] = ptr_data3;
	bank[4] = ptr_data4;
	bank[5] = ptr_data5;
	bank[6] = ptr_data6;
	bank[7] = ptr_data7;
	bank[8] = ptr_temp_data;
	bank[9] = ptr_w;


	ptr_data_nmr = data_nmr + PAD;
	output_data = (OutBuffer*) malloc(sizeof(OutBuffer));
	OutBuffer_Init(output_data, ptr_data_nmr);

	ptr_data_sum = data_sum + PAD;
	summ_data = (SummationBuffer*) malloc(sizeof(SummationBuffer));
	SummationBuffer_Init(summ_data, ptr_data_sum, ALLDATA_BUFF_LEN, &XX[0]);

	memset(&data_fin[0], 0x0, ALLDATA_BUFF_SIZE * sizeof(unsigned char));
	memset(ptr_data_nmr, 0xFF, ALLDATA_BUFF_LEN * sizeof(float)); // заполнить массив результатов обработки данных ЯМР числом NaN
	memset(ptr_data_sum, 0xFF, ALLDATA_BUFF_LEN * sizeof(float)); // заполнить массив результатов обработки данных ЯМР числом NaN


	// Math functions tabulation --------------------------------------------------------
	initGaussTab();
	initBiGaussTab();

	// Default Parameters for ADC data processing ---------------------------------------
	processing_params = (Processing_Params*) malloc(sizeof(Processing_Params));
	setDefaultProcParams(processing_params);

	instr_prg = (Data_Proc*) malloc(sizeof(Data_Proc));
	//init_DataProc(instr_prg); 								// инициализация структуры для хранения программы обработки данных ЯМР/СДСП и т.п.
	int i;
	for (i = 0; i < MAX_PROCS; i++)
	{
		instr_prg->procs[i] = (uint8_t*)calloc(MAX_PROC_LEN, sizeof(uint8_t));
	    //instr_prg->procs[i] = p + i*MAX_PROC_LEN;
		instr_prg->proc_lens[i] = 0;
		instr_prg->index[i] = 0;
	}

	data_stack = (STACKPtrF*) malloc(sizeof(STACKPtrF));	// инициализация стека данных


	// twiddle factors ------------------------------------------------------------------
	rad = rad_gen(CMPLX_DATA_MAX_LEN);
	tw_gen(ptr_w, CMPLX_DATA_MAX_LEN);

	// Compute overhead of calling clock() twice and init TimingData --------------------
	t_start = clock();
	t_stop = clock();
	t_overhead = t_stop - t_start;

	// Data containers ------------------------------------------------------------------
	samples_buffer = (uint8_t*)malloc(SAMPLES_BUFFER_SIZE*sizeof(uint8_t));
	samples_buffer_pos = 0;
	for (i = 0; i < UPP_DATA_COUNT; i++)
	{
		DataSample *data_sample = (DataSample*) malloc(sizeof(DataSample));
		//data_sample->data_ptr = samples_buffer[i];
		data_sample->data_ptr = 0;
		data_sample->data_len = 0;
		data_sample->echo_number = 0;
		data_sample->proc_id = 0;
		data_sample->tool_id = 0;
		data_sample->channel_id = 0;
		//data_sample->heap_ptr = data_heap[i];
		//data_sample->heap_len = 0;
		data_sample->tag = 0;
		data_samples[i] = data_sample;
	}
	current_data_sample = 0;
	samples_buffer_pos = 0;
	ds_new_data_enabled = 1;
	//initDataSamples(data_samples);
	// ----------------------------------------------------------------------------------


	// ----------------------------------------------------------------------------------
	startClocker(clocker3);
	startClocker(clocker4);
#ifdef USE_TELEMETRIC_UART
	telemetry_ready = TELE_NOT_READY;
#endif
#ifdef USE_PRESSURE_UNIT
	press_unit_ready = PRESS_UNIT_NOT_READY;
	press_unit_data = 0;
#endif

	fpga_prg_started = False;

	instr = (Data_Cmd*) malloc(sizeof(Data_Cmd));
	instr->cmd = 0;
	instr->count = 0;
	instr->type = 0;
	//instr->params = 0;
	instr->params = (float*)calloc(32, sizeof(float));
	//init_DataProcCmd(instr);


#ifdef USE_TIMING
	uint32_t tsch = TSCH;
	uint32_t tscl = TSCL;
	TimingProc_Buffer_Init(&timing_buffer, tsch, tscl);
#endif

	printf("All system variables and constants were initialized.\n");


	// Start program ********************************************************************
	upp_start(byte_count, line_count, upp_buffer);

	printf("Start!\n");

	// Telemetry test -------------------------------------------------------------------
	/*CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
	CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'c');
	CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
	CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
	//proger_restart_time_counter();
	dummyDelay(100);*/
	// ---------------------------------------------------------------------------------

	//volatile int new_data_counter = 0;
	while (app_finish == 0)
	{
		//new_data_counter = newDataAvailable();
		//if (new_data_counter > 0)
		if (seq_completed > 0)
		{
			setupDDR2Cache();
			enableCacheL1();

			int data_samples_count = current_data_sample;
			//if (data_samples_count > 0)
			int i;
			for (i = 0; i <= data_samples_count; i++)
			{
				DataSample *ds = data_samples[i];	// забрать свежие данные
				move_ToFirstDataProcCmd(ds->proc_id-1, instr_prg);
				executeProcPack(instr_prg, ds->proc_id-1);
			}

			current_data_sample = 0;
			samples_buffer_pos = 0;
			seq_completed = -1;

			disableCache();
		}

		if (tool_state == READY) // прибор готов к приему/передаче данных по кабелю (получен сигнал GP0[3] "up")
		{
			QUEUE8_clear(uart_queue);
			QUEUE8_clear(head_q);
			BUFFER8_clear(body_q);

			clearMsgHeader(in_msg_header);

			msg_header_state = NOT_DEFINED;
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;

			timerSettings.enabled = True;
			enable_Timer(tmrRegs);

			startClocker(clocker3);
			startClocker(clocker4);

			//printf("ds_proc_data_index = %d; current_data_sample = %d\n", ds_proc_data_index, current_data_sample);
			//ds_proc_data_index = 0;
			//current_data_sample = 0;


#ifdef USE_TELEMETRIC_UART
			if (telemetry_ready == TELE_NOT_READY)
			{
				temp_request_mode = TEMP_NOT_DEFINED;
				voltage_request_mode = VOLT_NOT_DEFINED;
				toMeasureTemperatures();
				//telemetry_ready = TELE_READY;
				//telemetry_ready = TELE_NOT_READY;
			}
#endif

			unsigned int tele_flag = 0;
#ifdef USE_TELEMETRIC_UART
			if (temperature_mode == TEMP_MODE_NOT_SPVP) if (UART_telemetric_counter % TELEMETRIC_DATA_LEN == 0 && UART_telemetric_counter > 0) tele_flag = 1;
			else if (temperature_mode == TEMP_MODE_SPVP)
			{
				if (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY)
				{
					tele_flag = 1;
				}
			}
#endif
#ifdef USE_PRESSURE_UNIT
			if (press_unit_ready == PRESS_UNIT_READY) tele_flag = 1;
#endif

			uint8_t pg = (uint8_t) proger_rd_pwr_pg();
			unsigned int pp_is_started  = proger_is_started();
			unsigned int pp_is_seq_done = proger_is_seq_done();
			unsigned int out_mask = pg | (tele_flag << 1) | (pp_is_started << 2) | (pp_is_seq_done << 3);
			sendByteArray(NMRTool_Ready[out_mask], SRV_MSG_LEN + 2, uartRegs);

			if (timerSettings.enabled == False)
			{
				timerSettings.enabled = True;
				enable_Timer(tmrRegs);
			}
		}

		if (tool_state == NOT_READY)
		{
			setupDDR2Cache();
			enableCacheL1();

			memset(&data_fin[0], 0x0, ALLDATA_BUFF_SIZE * sizeof(unsigned char));
			data_fin_counter = 0;

			SummationBuffer_ClearAll(summ_data);
			OutBuffer_ClearAll(output_data);

			tool_state = BUSY;
			outcom_msg_state = NOT_BUILT;

			sendByteArray(&NMRTool_NotReady[0], SRV_MSG_LEN + 2, uartRegs);

			memset(upp_buffer, 0x0, UPP_BUFF_SIZE);
			upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

			timerSettings.enabled = False;
			enable_Timer(tmrRegs);

			stopClocker(clocker3);
			stopClocker(clocker4);

			seq_completed = -1;

			disableCache();
		}

		if (tool_state == BUSY)
		{
			if (timerSettings.enabled == True)
			{
				timerSettings.enabled = False;
				enable_Timer(tmrRegs);

				stopClocker(clocker3);
				stopClocker(clocker4);
			}
		}

		if (tool_state == FREE || tool_state == UNKNOWN_STATE)
		{
			if (timerSettings.enabled == False)
			{
				timerSettings.enabled = True;
				enable_Timer(tmrRegs);

				startClocker(clocker3);
				startClocker(clocker4);
			}
		}


		/*if (tool_state == FREE || tool_state == UNKNOWN_STATE)
		{
			unsigned int pins = proger_read_gpio();
			uint8_t pin3 = (pins & (1 << 3)) >> 3;
			if (pin3 == 0 && seq_completed >= 0)
			{
				tool_state = BUSY;
				continue;
			}
		}*/

		// если incom_msg_state == NOT_DEFINED или STARTED
		if (incom_msg_state < FINISHED)
		{
			if (!dataUnavailable)
			{
				//_disable_interrupts();
				dataUnavailable = True;
				onDataAvailable(uart_queue);
				//_enable_interrupts();
			}
		}
		else if (incom_msg_state == FINISHED)
		{
			// если заголовок служебного сообщения (а значит и все служебное сообщение) успешно принят и декодирован
			if (in_msg_header->msg_type == MTYPE_SERVICE)
			{
				//_disable_interrupts();
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();

				executeServiceMsg(in_msg_header);

				clearMsgHeader(in_msg_header);
				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;
			}
			// если заголовок короткого сообщения (а значит и все короткое сообщение) успешно принят и декодирован
			else if (in_msg_header->msg_type == MTYPE_SHORT)
			{
				//_disable_interrupts();
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();

				executeShortMsg(in_msg_header);

				clearMsgHeader(in_msg_header);
				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;
			}
			// если успешно принят и декодирован заголовок многопакетного сообщения
			else if (in_msg_header->msg_type == MTYPE_MULTYPACK)
			{
				//_disable_interrupts();
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();

				executeMultypackMsg(&in_msg);

				clearMsgHeader(in_msg_header);

				int i;
				uint16_t pack_cnt = in_msg.pack_cnt;
				for (i = 0; i < pack_cnt; i++) free(in_msg.msg_packs[i]);
				in_msg.pack_cnt = 0;
				in_msg.msg_header->pack_count = 0;

				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;
			}
		}
		// если принимается многопакетное сообщение (заголовок уже принят)
		else if (incom_msg_state == PACKS_STARTED)
		{
			if (!dataUnavailable)
			{
				//_disable_interrupts();
				dataUnavailable = True;
				onDataAvailable(uart_queue);
				//_enable_interrupts();
			}
		}
		// если incom_msg_state = FAILED или incom_msg_state = TIMED_OUT
		else if (incom_msg_state == FAILED || incom_msg_state == TIMED_OUT)
		{
			//_disable_interrupts();
			clearMsgHeader(in_msg_header);
			QUEUE8_clear(uart_queue);
			QUEUE8_clear(head_q);
			BUFFER8_clear(body_q);
			msg_header_state = NOT_DEFINED;
			incom_msg_state = NOT_DEFINED;
			//_enable_interrupts();
		}
	}
}
// **************************************************************************************


// *************** Cable Communication **************************************************
// Использование START_BYTE и STOP_BYTE
void onDataAvailable(QUEUE8* bytes)
{
	int sz = QUEUE8_count(bytes);
	if (msg_header_state == STARTED && sz < HEADER_LEN) return;

	if (incom_msg_state == STARTED && sz == HEADER_LEN)
	{
		//int tmp_arr[12];
		int i;
		for (i = 0; i < sz; i++)
		{
			uint8_t c = QUEUE8_get(bytes);
			QUEUE8_put(c, head_q);
			//tmp_arr[i] = c;
		}

		int res = findMsgHeader(head_q, in_msg_header, gf_data);
		if (res == E_RS_OK)
		{
			res = checkMsgHeader(in_msg_header);
			if (res == E_MSG_OK)
			{
				msg_header_state = FINISHED;
				if (in_msg_header->msg_type == MTYPE_SERVICE || in_msg_header->msg_type == MTYPE_SHORT) incom_msg_state = FINISHED;
				else if (in_msg_header->msg_type == MTYPE_MULTYPACK)
				{
					stopClocker(clocker1);
					QUEUE8_clear(bytes);
					gf_data->index_body = in_msg_header->rec_errs - 1;
					incom_msg_state = PACKS_STARTED;
					responseMultypackHeader(in_msg_header);
					msg_was_treated = MSG_NO_PACKS;
					int pack_count = (int) in_msg_header->pack_count;
					int pack_len = (int) in_msg_header->pack_len;
					uint64_t packs_delay = 250 + (pack_count * pack_len) / (uartSettings.BaudRate / 8.0) * 1000 * 2; // 2 - двойной запас по времени // it was 50
					initClocker(packs_delay, clocker2_ISR, clocker2);
					startClocker(clocker2);
				}
			}
			else
			{
				msg_header_state = FAILED;
				incom_msg_state = FAILED;
			}
			stopClocker(clocker1);
			return;
		}
		else if (res == E_RS_NOTFOUND)
		{
			msg_header_state = FAILED;
			incom_msg_state = FAILED;
		}
		else if (res == E_RS_LEN)
		{
			msg_header_state = FAILED;
			incom_msg_state = FAILED;
		}

		QUEUE8_clear(head_q);
		//clearMsgHeader(in_msg_header);
		//msg_header_state = NOT_DEFINED;
	}
	else if (msg_header_state == NOT_DEFINED && sz > 1)
	{
		QUEUE8_clear(bytes);
	}


	if (incom_msg_state == PACKS_STARTED)
	{
		msg_was_treated = MSG_DATA_NOT_ALL;
		int sz = QUEUE8_count(bytes);
		int pack_count = (int) in_msg_header->pack_count;
		int pack_len = (int) in_msg_header->pack_len;
		//printf(" sz = %d ;",sz);
		if (sz >= pack_count * pack_len)
		{
			setupDDR2Cache();
			enableCacheL1();
			while (sz-- > 0) BUFFER8_put(QUEUE8_get(bytes), body_q);
			incom_msg_state = PACKS_FINISHED;
			msg_was_treated = MSG_DECODE_ERR;
			disableCache();
		}

		if (incom_msg_state == PACKS_FINISHED)
		{
			stopClocker(clocker2);

			// Enable DDR cache
			setupDDR2Cache();
			enableCacheL1();

			int res = findMsgPackets2(body_q, &in_msg, gf_data);
			if (res == E_RS_OK)
			{
				msg_was_treated = MSG_BAD_PACKETS;
				res = checkMsgPackets(&in_msg);
				if (res == E_MSG_OK)
				{
					msg_was_treated = MSG_EXTRACT_ERR;
					msg_header_state = FINISHED;
					incom_msg_state = FINISHED;
				}
				else
				{
					int i;
					for (i = 0; i < in_msg.pack_cnt; i++) free(in_msg.msg_packs[i]); // added 3.09.2015
					in_msg.pack_cnt = 0;

					msg_header_state = FAILED;
					incom_msg_state = FAILED;
				}
			}
			else
			{
				int i;
				for (i = 0; i < in_msg.pack_cnt; i++) free(in_msg.msg_packs[i]); // added 3.09.2015
				in_msg.pack_cnt = 0;

				msg_was_treated = MSG_DECODE_ERR;
				msg_header_state = FAILED;
				incom_msg_state = FAILED;
			}

			disableCache();
		}
	}
}
/////


void sendByteArray(uint8_t *arr, uint16_t len, CSL_UartRegsOvly uartRegs)
{
	int i;
	for (i = 0; i < len; i++) write_UART(uartRegs, arr[i]);
}

void sendServiceMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	if (!_msg_header) return;

	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->writer & 0x0F) << 4) | (_msg_header->reader & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->data[0];
	arr_poly->data[4] = _msg_header->data[1];
	arr_poly->data[5] = _msg_header->data[2];
	arr_poly->data[6] = _msg_header->data[3];
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);

	int g_num = gf_data->index_hdr;

	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	memcpy(arr_poly->data + HEAD_INFO_LEN, rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = 0; i < HEADER_LEN; i++) write_UART(uartRegs, arr_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}

void sendShortMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->reader & 0x0F) << 4) | (_msg_header->writer & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->data[0];
	arr_poly->data[4] = _msg_header->data[1];
	arr_poly->data[5] = _msg_header->data[2];
	arr_poly->data[6] = _msg_header->data[3];
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);
	arr_poly->power = HEAD_INFO_LEN - 1;

	int g_num = gf_data->index_hdr;

	GFPoly_self_inv(arr_poly);
	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	GFPoly_self_inv(rec_poly);
	memcpy(&_msg_header->rec_data[0], rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = HEAD_INFO_LEN - 1; i >= 0; i--) write_UART(uartRegs, arr_poly->data[i]);
	for (i = 0; i < HEAD_REC_LEN; i++) write_UART(uartRegs, rec_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}

void sendMultyPackMsg(UART_Message *uart_msg, CSL_UartRegsOvly uartRegs)
{
	MsgHeader *_msg_header = uart_msg->msg_header;

	if (_msg_header->msg_type == MTYPE_MULTYPACK)
	{
		uint8_t start_byte = START_BYTE;
		uint8_t stop_byte = STOP_BYTE;
		if (outcom_msg_state == NOT_BUILT || outcom_msg_state == MESSAGE_SENT)
		{
			sendHeader(_msg_header, uartRegs);
			outcom_msg_state = HEADER_SENT;
		}
		else if (outcom_msg_state == HEADER_SENT)
		{
			uint16_t pack_cnt = uart_msg->pack_cnt;
			int i;
			for (i = 0; i < pack_cnt; i++)
			{
				MsgPacket *_msg_packet = uart_msg->msg_packs[i];

				write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
				sendByteArray(&_msg_packet->data[0], _msg_packet->pack_len, uartRegs);
				write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

				if (msg_settings->pack_delay > 0)
				{
					uint32_t dummy_counts = (uint32_t) (10 * msg_settings->pack_delay); // dummyDelay(10) ~= 1 ms
					dummyDelay(dummy_counts);
				}
			}
			outcom_msg_state = MESSAGE_SENT;
		}

	}
}

void sendHeader(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->reader & 0x0F) << 4) | (_msg_header->writer & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->pack_count;
	arr_poly->data[4] = _msg_header->pack_len;
	arr_poly->data[5] = _msg_header->block_len;
	arr_poly->data[6] = _msg_header->rec_errs;
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);
	arr_poly->power = HEAD_INFO_LEN - 1;

	int g_num = gf_data->index_hdr;

	GFPoly_self_inv(arr_poly);
	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	GFPoly_self_inv(rec_poly);
	memcpy(&_msg_header->rec_data[0], rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = HEAD_INFO_LEN - 1; i >= 0; i--) write_UART(uartRegs, arr_poly->data[i]);
	for (i = 0; i < HEAD_REC_LEN; i++) write_UART(uartRegs, rec_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}


Bool extractDataFromPacks(UART_Message *uart_msg, uint8_t *arr, uint16_t *len)
{
	uint32_t i;
	uint32_t pos = 0;
	//uint8_t temp[2000];
	for (i = 0; i < uart_msg->pack_cnt; i++)
	{
		MsgPacket *pack = uart_msg->msg_packs[i];
		uint32_t data_len = pack->data_len;
		if (pos > MAX_BODY_LEN)
		{
			*len = 0;
			return False;
		}
		memcpy(arr + pos, &pack->data[PACK_HEAD_LEN], (data_len - PACK_SRV_LEN) * sizeof(uint8_t));
		//memcpy(&temp[pos], &pack->data[PACK_HEAD_LEN], (data_len-PACK_SRV_LEN)*sizeof(uint8_t));
		pos += data_len - PACK_SRV_LEN;
	}

	if (pos < 3)
	{
		*len = 0;
		return False;
	}

	uint32_t data_pos = 1;
	Bool fin = False;
	while (fin == False)
	{
		uint8_t byte1 = arr[data_pos];
		uint8_t byte2 = arr[data_pos + 1];
		uint16_t data_len = ((uint16_t) byte2 << 8) | (uint16_t) byte1;

		data_pos += data_len + 2; // 1 byte for separator '0xFF' and 1 byte for cmd in data array
		if (data_pos > pos || data_pos > MAX_BODY_LEN)
		{
			*len = 0;
			return False;
		}

		if (arr[data_pos] != 0xFF)
		{
			fin = True;
			*len = data_pos;
		}
		else data_pos += 2;
	}

	return True;
}


void init_UART_MsgData(void)
{
	dataUnavailable = True;
	transmitterFull = True;

	gf_data = (GF_Data*) malloc(sizeof(GF_Data));
	gfdata_init(gf_data, MAX_REC_ERRS);
	gf_data->index = 1; 										// номер порождающего полинома, который равен максимальному количеству корректируемых в данном сообщении ошибок минус 1
	gf_data->index_hdr = 1; 									// номер порождающего полинома, который равен максимальному количеству корректируемых в заголовке ошибок минус 1
	gf_data->index_body = 1; 									// номер порождающего полинома, который равен максимальному количеству корректируемых в теле сообщения ошибок минус 1
	gf_data->index_ftr = 1; 									// номер порождающего полинома, который равен максимальному количеству корректируемых в концовке сообщения ошибок минус 1

	//input_data_enabled = True;

	msg_settings = (MsgCommSettings*) malloc(sizeof(MsgCommSettings));
	msg_settings->block_len = 20;
	msg_settings->rec_errs = gf_data->index_body + 1;
	msg_settings->pack_len = 200;
	msg_settings->antinoise_coding = True;
	msg_settings->packlen_autoadjust = False;
	msg_settings->interleaving = False;

	int i;
	in_msg_header = (MsgHeader*) malloc(sizeof(MsgHeader));
	clearMsgHeader(in_msg_header);
	in_msg.msg_header = in_msg_header;
	/*for (i = 0; i < MAX_PACK_CNT; i++)
	 {
	 MsgPacket *pack = (MsgPacket*)malloc(sizeof(MsgPacket));
	 clearMsgPacket(pack);
	 in_msg.msg_packs[i] = pack;
	 }*/
	for (i = 0; i < MAX_PACK_CNT; i++) in_msg.msg_packs[i] = NULL;
	in_msg.pack_cnt = 0;

	out_msg_header = (MsgHeader*) malloc(sizeof(MsgHeader));
	clearMsgHeader(out_msg_header);
	out_msg.msg_header = out_msg_header;
	for (i = 0; i < MAX_PACK_CNT; i++)
	{
		MsgPacket *pack = (MsgPacket*) malloc(sizeof(MsgPacket));
		clearMsgPacket(pack);
		out_msg.msg_packs[i] = pack;
	}
	out_msg.pack_cnt = 0;

	head_q = (QUEUE8*) malloc(sizeof(QUEUE8));
	body_q = (BUFFER8*) malloc(sizeof(BUFFER8));
	QUEUE8_init(HEADER_LEN, head_q);
	BUFFER8_init(body_q);

	uart_queue = (QUEUE8*) malloc(sizeof(QUEUE8));
	QUEUE8_init(MAX_BODY_LEN, uart_queue);
}

#ifdef USE_TELEMETRIC_UART
void telemetryDataToOutput(OutBuffer *out_buff)
{
	float *dst = out_buff->out_data;
	int dst_pos = out_buff->full_size;
	int data_cnt = out_buff->outdata_counter;
	//memcpy((uint8_t*)dst + dst_pos, &telemetric_data[0], TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));

	if (temperature_mode == TEMP_MODE_NOT_SPVP)
	{
		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[0], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3; // 9 bytes occupy 3 floats (sizeof(float) = 4)
			out_buff->data_id[data_cnt] = DT_DU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3;// = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status >>= 1;

		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[TELEMETRIC_DATA_LEN], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3;
			out_buff->data_id[data_cnt] = DT_TU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3; // = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status >>= 1;

		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[2*TELEMETRIC_DATA_LEN], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3;
			out_buff->data_id[data_cnt] = DT_PU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3; // = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status = 0;

		out_buff->full_size += dst_pos;

		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));
	}
	else if (temperature_mode == TEMP_MODE_SPVP)
	{
		//if ((telemetric_board_status & 0x01) == 1)
		if (temp_request_mode == TEMP_READY)
		{
			uint16_t cnt = 0;
			uint16_t sensors = temp_sensors;
			uint8_t temp[TELEMETRIC_UART_BUF_LEN2+sizeof(uint16_t)];	// let's enable up to 12 temperature meters, also 2 bytes for the number of temperature meters
			memset(&temp[0], 0x00, TELEMETRIC_UART_BUF_LEN2*sizeof(uint8_t));
			memcpy(&temp[cnt], &sensors, sizeof(uint16_t));
			cnt += 1*sizeof(uint16_t);
			memcpy((uint8_t*) (&temp[cnt]), &telemetric_data[0], 2*sensors*sizeof(uint8_t));
			cnt += 2*sensors*sizeof(uint8_t);
			memcpy((uint8_t*) (dst + dst_pos), &temp[0], cnt);
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float)); // the number of float values
			out_buff->data_id[data_cnt] = DT_T;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
			//dst_pos += 5;// = (2*sensors(up to 8) + 4) bytes / sizeof(float)
		}
		if (voltage_request_mode == VOLT_READY)
		{
			uint16_t cnt = 0;
			uint16_t sensors = 4;			// 4 voltage meters
			uint8_t temp[TELEMETRIC_UART_BUF_LEN2+sizeof(uint16_t)]; // 8 bytes of 4 voltage sensors * sizeof(uint16_t) + the number of sensors (4 totally)
			memset(&temp[0], 0x00, (sensors*sizeof(uint16_t)+sizeof(uint16_t)));
			memcpy(&temp[cnt], &sensors, sizeof(uint16_t));
			cnt += 1*sizeof(uint16_t);
			memcpy((uint8_t*) (&temp[cnt]), &telemetric_data[2*temp_sensors], sensors*sizeof(uint16_t));
			cnt += sensors*sizeof(uint16_t);
			memcpy((uint8_t*) (dst + dst_pos), &temp[0], cnt*sizeof(uint8_t));
			out_buff->outdata_counter++;
			//out_buff->outdata_len[data_cnt] = 3; 	// 4 bytes for number '4' (int) and 8 bytes of voltage data occupy 3 elements of type 'float'
			out_buff->outdata_len[data_cnt] = (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
			out_buff->data_id[data_cnt] = DT_U;
			out_buff->group_index[data_cnt++] = 0;
			//dst_pos += 3;							// = (4 + 2*4 bytes)/sizeof(float)
			dst_pos += (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
		}

		telemetric_board_status = 0;

		out_buff->full_size += dst_pos;

		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN2 * sizeof(uint8_t));
	}

	UART_telemetric_counter = 0;
	UART_telemetric_pack_counter = 0;
	telemetry_ready = TELE_NOT_READY;
}
#endif

#ifdef USE_PRESSURE_UNIT
void pressureUnitDataToOutput(OutBuffer *out_buff)
{
	int adc_channel = 0;
	unsigned int mtr_adc_value = proger_read_mtr_adc_value (adc_channel);
	signed int mtr_counter = proger_read_counter_mtr ();
	unsigned int mtr_status = proger_read_mtr_status ();

	/*if ((mtr_status & 0x4) >> 2 && clocker4->max_val < 10000)
	{
		clocker4->max_val = 10000;
		startClocker(clocker4);
	}*/

	float *dst = out_buff->out_data;
	int dst_pos = out_buff->full_size;
	int data_cnt = out_buff->outdata_counter;

	memcpy((uint8_t*) (dst + dst_pos), &mtr_adc_value, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	memcpy((uint8_t*) (dst + dst_pos), &mtr_counter, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	memcpy((uint8_t*) (dst + dst_pos), &mtr_status, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	out_buff->outdata_counter++;
	out_buff->outdata_len[data_cnt] = 3; // 9 bytes occupy 3 floats (sizeof(float) = 4)
	out_buff->data_id[data_cnt] = DT_PRESS_UNIT;
	out_buff->group_index[data_cnt++] = 0;

	out_buff->full_size += dst_pos;

	press_unit_ready = PRESS_UNIT_NOT_READY;

	//uint32_t tmp[1024];
	//memcpy(&tmp[0], out_buff->out_data, dst_pos*sizeof(uint32_t));
	//int tt = 0;
}
#endif

void prepareOutputByteArray(OutBuffer *out_buff, SummationBuffer *sum_buff)
{
	int i;
	int index = 0;

	int outdata_count = out_buff->outdata_counter;
	int pos = 0;
	for (i = 0; i < outdata_count; i++)
	{
		uint8_t data_id = out_buff->data_id[i];
		uint16_t data_len = (uint16_t) out_buff->outdata_len[i];
		uint16_t group_index = (uint16_t) out_buff->group_index[i];

		switch (data_id)
		{
		case DT_SGN_SE_ORG:
		case DT_NS_SE_ORG:
		case DT_SGN_FID_ORG:
		case DT_NS_FID_ORG:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				int16_t min = (int16_t) out_buff->out_data[pos];
				int16_t max = min;
				for (j = 0; j < data_len; j++)
				{
					int16_t x = (int16_t) out_buff->out_data[j + pos];
					if (x < min) min = x;
					if (x > max) max = x;
				}

				float b = -min;
				float a = (max - min) / 255.0;
				if (a == 0) a = 1;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					int16_t x = (int16_t) out_buff->out_data[j + pos];
					uint8_t val = (uint8_t) ((x + b) / a);
					data_fin[j + index] = val;
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

				data_fin_counter = index;
			}

			break;
		}
		case DT_SGN_SE:
		case DT_NS_SE:
		case DT_NS_QUAD_SE_RE:
		case DT_NS_QUAD_SE_IM:
		case DT_NS_QUAD_FID_RE:
		case DT_NS_QUAD_FID_IM:
		case DT_SGN_QUAD_SE_RE:
		case DT_SGN_QUAD_SE_IM:
		case DT_SGN_QUAD_FID_RE:
		case DT_SGN_QUAD_FID_IM:
		case DT_NS_FFT_FID_RE:
		case DT_NS_FFT_SE_RE:
		case DT_SGN_FFT_FID_RE:
		case DT_SGN_FFT_SE_RE:
		case DT_NS_FFT_FID_IM:
		case DT_NS_FFT_SE_IM:
		case DT_SGN_FFT_FID_IM:
		case DT_SGN_FFT_SE_IM:
		case DT_SGN_FFT_FID_AM:
		case DT_NS_FFT_FID_AM:
		case DT_SGN_FFT_SE_AM:
		case DT_NS_FFT_SE_AM:
		case DT_SGN_POWER_SE:
		case DT_SGN_POWER_FID:
		case DT_NS_POWER_SE:
		case DT_NS_POWER_FID:
		case DT_RFP:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				volatile int tt = 0;
				float min = out_buff->out_data[pos];
				float max = min;
				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					if (x < min) min = x;
					if (x > max) max = x;
				}

				float b = -min;
				float a = (max - min) / 255.0;
				if (a == 0) a = 1.0;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint8_t val = (uint8_t) ((x + b) / a);
					data_fin[j + index] = val;
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

				data_fin_counter = index;
			}
			break;
		}
		case DT_SGN_RELAX:
		case DT_AFR1_RX:
		case DT_AFR2_RX:
		case DT_AFR3_RX:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				float min = out_buff->out_data[pos];
				float max = min;
				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint32_t *b = (uint32_t*) &x;
					if (*b != 0xffffffff)
					{
						if (x < min) min = x;
						if (x > max) max = x;
					}
				}

				float b = -min;
				float a = (max - min) / 254.0;
				//if (a == 0) a = 1.0;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint32_t *bb = (uint32_t*) (&(out_buff->out_data[j + pos]));
					if (*bb == 0xffffffff) 	data_fin[j + index] = 0xff;
					else
					{
						if (min == max) data_fin[j + index] = 1;
						else
						{
							uint8_t val = (uint8_t) ((x + b) / a);
							data_fin[j + index] = val;
						}
					}
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

				data_fin_counter = index;
			}
			break;
		}
		case DT_DIEL:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(float));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos), data_in_bytes);
			pos += data_len;
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;
			break;
		}
		case DT_GAMMA:
		case DT_PRESS_UNIT:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(float));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos), data_in_bytes);
			pos += data_len;
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		case DT_DU:
		case DT_PU:
		case DT_TU:
		{
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);

			uint16_t data_in_bytes = (uint16_t) (TELEMETRIC_DATA_LEN);
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos), data_in_bytes);
			pos += 3;

			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		case DT_T:
		{
			uint8_t temp[100];
			memcpy(&temp[0], out_buff->out_data, 100);
			if (temperature_mode != TEMP_MODE_SPVP) return;
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);

			int sensors = *(uint8_t*)(out_buff->out_data+pos);
			//pos++;
			uint16_t data_in_bytes =  2*sensors;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos) + 2, data_in_bytes);
			//pos += 4;			// 8 sensors * 2 bytes / sizeof(float)
			pos += (2*sensors+2+(sizeof(float)-1))/sizeof(float);
			index += data_in_bytes;		// 8 sensors * 2 bytes
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		case DT_U:
		{
			if (temperature_mode != TEMP_MODE_SPVP) return;
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);

			int sensors = *(uint8_t*)(out_buff->out_data+pos);
			uint16_t data_in_bytes = 2*sensors;
			//pos++;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos)+2, data_in_bytes);
			pos += (data_in_bytes + 2 + (sizeof(float)-1))/sizeof(float);
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		default: break;
		}
	}

	SummationBuffer_ClearAll(sum_buff);
	OutBuffer_ClearAll(out_buff);
}

void summationDataToOutput(OutBuffer *out_buff, SummationBuffer *sum_buff)
{
	if (sum_buff->max_size <= 0) return;
	if (sum_buff->pos <= 0) return;

	int outdata_count = out_buff->outdata_counter;
	uint8_t data_id = sum_buff->data_id;
	int data_len = sum_buff->pos;
	int group_index = sum_buff->group_index;

	if (data_len + out_buff->full_size > NMR_DATA_LEN) return;

	memcpy(out_buff->out_data + out_buff->full_size, sum_buff->sum_data, data_len * sizeof(float));
	out_buff->data_id[outdata_count] = data_id;
	out_buff->outdata_len[outdata_count] = data_len;
	out_buff->full_size += data_len;
	out_buff->group_index[outdata_count] = group_index;
	out_buff->outdata_counter++;
}
// **************************************************************************************

// Execute incomming messages ************************************************************
void executeServiceMsg(MsgHeader *_msg_header)
{
	sendServiceMsg(_msg_header, uartRegs);
}

void executeShortMsg(MsgHeader *_msg_header)
{
	uint8_t cmd = _msg_header->data[0];

	switch (cmd)
	{
	case GET_DATA:
	{
		if (tool_state == UNKNOWN_STATE) return;
		//if (fpga_prg_started == False && UART_telemetric_counter != TELEMETRIC_UART_BUF_LEN && sdsp_started == False) return;
		Bool to_out = (fpga_prg_started == False);
#ifdef USE_TELEMETRIC_UART
		if (temperature_mode == TEMP_MODE_NOT_SPVP) to_out = to_out && (UART_telemetric_counter % TELEMETRIC_DATA_LEN > 0);
		else if (temperature_mode == TEMP_MODE_SPVP) to_out = to_out && (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY);
#endif
#ifdef USE_DIELEC_UART
		to_out = to_out && (sdsp_started == False);
#endif
#ifdef USE_PRESSURE_UNIT
		int attempts_cnt = 5;
		if (proger_read_mtr_adc_status())
		{
			proger_mtr_adc_start_conversion();
			while (attempts_cnt > 0)
			{
				dummyDelay(1);
				if (proger_read_mtr_adc_status()) break;
				else --attempts_cnt;
			}
		}

		to_out = to_out && (attempts_cnt == 0);
#endif
		//if (fpga_prg_started == False && UART_telemetric_counter % TELEMETRIC_DATA_LEN > 0 && sdsp_started == False) return;
		if (to_out) return;

		//printf("Get Data !\n");

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_MULTYPACK;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		hdr->pack_count = 0;
		hdr->pack_len = msg_settings->pack_len;
		hdr->block_len = msg_settings->block_len;
		hdr->rec_errs = msg_settings->rec_errs;

		uint16_t *pos = (uint16_t*) malloc(sizeof(uint16_t));
		*pos = 0;
		uint16_t dpos = *pos;
		uint8_t pack_number = 1;

		// Enable DDR cache
		setupDDR2Cache();
		enableCacheL1();

#ifdef USE_TELEMETRIC_UART
		if (telemetry_ready == TELE_READY)
		{
			telemetryDataToOutput(output_data);
			//printf("Telemetry is ready!\n");
		}
#endif
#ifdef USE_PRESSURE_UNIT
		if (press_unit_ready == PRESS_UNIT_READY)
		{
			pressureUnitDataToOutput(output_data);
		}
#endif
		summationDataToOutput(output_data, summ_data);
		prepareOutputByteArray(output_data, summ_data);

		int pack_len = msg_settings->pack_len;
		if (msg_settings->packlen_autoadjust == True)
		{
			int rs_part_len = 2 * hdr->rec_errs;
			pack_len = estimateBestPackLen(data_fin_counter + 3, hdr->block_len, rs_part_len);
		}
		hdr->pack_len = pack_len;

		while (dpos < data_fin_counter + 3)
		{
			//MsgPacket* pack = (MsgPacket*) malloc(sizeof(MsgPacket));
			MsgPacket* pack = out_msg.msg_packs[pack_number - 1];
			clearMsgPacket(pack);
			pack->pack_len = hdr->pack_len;
			pack->block_len = hdr->block_len;
			pack->msg_id = hdr->id;
			pack->packet_number = pack_number;
			pack->rec_errs = gf_data->index_body + 1; // index_body есть степень полинома. Кол-во исправляемых ошибок = index_body+1
			pack->start_marker = MTYPE_PACKET;

			pushDataToMsgPacket(&data_fin[0], data_fin_counter + 3, pos, pack, gf_data);

			out_msg.msg_packs[pack_number - 1] = pack;
			out_msg.pack_cnt = pack_number;
			hdr->pack_count = pack_number;

			pack_number++;
			dpos = *pos;
		}
		free(pos);

		disableCache();
#ifdef USE_DIELEC_UART
		sdsp_started = False;
#endif
		sendMultyPackMsg(&out_msg, uartRegs);
		break;
	}
	case HEADER_OK:
	{
		if (outcom_msg_state == HEADER_SENT)
		{
			sendMultyPackMsg(&out_msg, uartRegs);

			clearMsgHeader(out_msg.msg_header);

			/*if (tool_state == UNKNOWN_STATE)
			{
				startClocker(clocker3);
			}*/
		}
		break;
	}
	case NMRTOOL_CONNECT:
	{
		printf("NMR Tool connect");

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = NMRTOOL_CONNECT;
		hdr->data[1] = (uint8_t)(device_serial & 0xFF);

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;

		clearMsgHeader(out_msg.msg_header);

		timerSettings.enabled = True;
		enable_Timer(tmrRegs);

		proger_stop();

		break;
	}
	case NMRTOOL_CONNECT_DEF:
		{
			printf("NMR Tool connect (default)");

			setDefaultCommSettings();

			MsgHeader *hdr = out_msg.msg_header;
			hdr->msg_type = MTYPE_SHORT;
			hdr->reader = PC_MAIN;
			hdr->writer = LOGGING_TOOL;
			hdr->id = _msg_header->id;
			memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
			hdr->data[0] = NMRTOOL_CONNECT_DEF;
			hdr->data[1] = (uint8_t)(device_serial & 0xFF);

			sendShortMsg(hdr, uartRegs);
			outcom_msg_state = MESSAGE_SENT;

			clearMsgHeader(out_msg.msg_header);

			timerSettings.enabled = True;
			enable_Timer(tmrRegs);

			proger_stop();

			break;
		}
	case NMRTOOL_START:
	{
		proger_stop();

		fpga_prg_started = True;

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		stopClocker(clocker3);
		stopClocker(clocker4);	// added 16.08.2017
		incom_msg_state = NOT_DEFINED;
		//tool_state = FREE; 	// commented 16.03.2016
		//upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
		//upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

		timerSettings.enabled = False;
		disable_Timer(tmrRegs);

		proger_start();

		break;
	}
	case NMRTOOL_STOP:
	{
		fpga_prg_started = False;

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		//tool_state = UNKNOWN_STATE;	// commented 16.03.2016
		proger_stop();

		timerSettings.enabled = True;
		enable_Timer(tmrRegs);

		startClocker(clocker3);
		startClocker(clocker4);		// added 16.08.2017
		tool_state = FREE;
		incom_msg_state = NOT_DEFINED;

		break;
	}
	case PRESS_UNIT_OPEN:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// move up the pressure unit (Open)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_UP);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_UP);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 1000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		printf("OPEN! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	case PRESS_UNIT_CLOSE:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// move down the pressure unit (Close)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_DOWN);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_DOWN);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 1000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		printf("CLOSE! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	case PRESS_UNIT_STOP:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// stop the pressure unit (Stop)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_STOP);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_STOP);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 10000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		printf("STOP! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	default: break;
	}
}

void executeMultypackMsg(UART_Message *uart_msg)
{
	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = uart_msg->msg_header->id;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));

	if (uart_msg->pack_cnt > 0)
	{
		uint8_t cmd = uart_msg->msg_packs[0]->data[PACK_HEAD_LEN];
		if (cmd == DATA_PROC)
		{
			proger_stop();

			enableCacheL1();
			uint8_t *data_arr = (uint8_t*) calloc(MAX_BODY_LEN, sizeof(uint8_t));
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, data_arr, &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;

				uint16_t prg_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = prg_len + PACK_HEAD_LEN;

				uint8_t temp_proger[4096];			// программа для интервального программатора
				memcpy(&temp_proger[0], data_arr + PACK_HEAD_LEN, prg_len*sizeof(uint8_t));

				uint8_t temp_dataproc[4096];		// программа обработки данных интервального программатора

				if (data_arr[pos++] == 0xFF)
				{
					cmd = data_arr[pos++];
					if (cmd == FPGA_PRG)
					{
						uint16_t instr_len = (uint16_t) data_arr[pos] | ((uint16_t) data_arr[pos + 1] << 8);
						clear_AllDataProc(instr_prg);
						//fill_DataProc(instr_prg, data_arr + pos + 2, instr_len);
						memcpy(&temp_dataproc[0], data_arr + pos + 2, instr_len*sizeof(uint8_t));
						fill_DataProc(instr_prg, &temp_dataproc[0], instr_len);

						hdr->data[0] = DATA_OK;
						hdr->data[1] = msg_was_treated;
						sendShortMsg(hdr, uartRegs);

						proger_mem_init();
						//proger_wr_pulseprog(data_arr + PACK_HEAD_LEN, (unsigned int) (prg_len));
						proger_wr_pulseprog(&temp_proger[0], (unsigned int) (prg_len));

						//hdr->data[0] = DATA_OK;
						//sendShortMsg(hdr, uartRegs);

						outcom_msg_state = MESSAGE_SENT;
						clearMsgHeader(out_msg.msg_header);

						fpga_prg_started = True;

						timerSettings.enabled = False;
						enable_Timer(tmrRegs);

						stopClocker(clocker3);
						stopClocker(clocker4);		// added 16.08.2017
						incom_msg_state = NOT_DEFINED;
						tool_state = FREE;
						proger_start();

						free(data_arr);

						return;
					}
				}
			}

			free(data_arr);
			disableCache();

			hdr->data[0] = DATA_FAILED;
			hdr->data[1] = msg_was_treated;
			sendShortMsg(hdr, uartRegs);

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			timerSettings.enabled = True;
			enable_Timer(tmrRegs);

			startClocker(clocker3);
			startClocker(clocker4);		// added 16.08.2017
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;
			proger_start();
		}
		else if (cmd == SET_WIN_PARAMS)
		{
			enableCacheL1();

			uint8_t data_arr[64];
			float fdata_arr[16];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;
				uint16_t prg_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = PACK_HEAD_LEN;

				memcpy(&fdata_arr[0], &data_arr[pos], prg_len * sizeof(uint8_t));

				processing_params->echo_func = (uint8_t) fdata_arr[0];
				processing_params->echo_x0 = (int) fdata_arr[1];
				processing_params->echo_sigma = (int) fdata_arr[2];

				processing_params->spectr_func = (uint8_t) fdata_arr[3];
				processing_params->spectr_x0 = (int) fdata_arr[4];
				processing_params->spectr_sigma = (int) fdata_arr[5];

				hdr->data[0] = DATA_OK;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			else
			{
				hdr->data[0] = DATA_FAILED;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			incom_msg_state = NOT_DEFINED;
			//tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			//startClocker(clocker3);

			//free(data_arr);
			disableCache();
		}
		else if (cmd == SET_COMM_PARAMS)
		{
			enableCacheL1();

			uint8_t data_arr[64];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;
				uint16_t pos = PACK_HEAD_LEN;

				msg_settings->pack_len = (uint8_t) data_arr[pos];
				msg_settings->block_len = (uint8_t) data_arr[pos + 1];
				msg_settings->rec_errs = (uint8_t) data_arr[pos + 2];
				gf_data->index_body = msg_settings->rec_errs - 1;

				uint8_t bools = (uint8_t) data_arr[pos + 3];
				msg_settings->packlen_autoadjust = (0x1 & bools);
				msg_settings->antinoise_coding = (0x2 & bools) >> 1;
				msg_settings->interleaving = (0x4 & bools) >> 2;

				msg_settings->pack_delay = data_arr[pos + 4];

				hdr->data[0] = DATA_OK;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			else
			{
				hdr->data[0] = DATA_FAILED;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			incom_msg_state = NOT_DEFINED;
			//tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			//startClocker(clocker3);

			disableCache();
			//proger_start();
		}
		else if (cmd == LOG_TOOL_SETTINGS)
		{
			enableCacheL1();

			uint8_t data_arr[1000];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;

				uint16_t data_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = PACK_HEAD_LEN;

				uint32_t ui32_data[250];
				int i;
				for (i = 0; i < data_len/sizeof(uint32_t); i++)
				{
					uint32_t v32 = data_arr[pos] | (data_arr[pos+1] << 8) | (data_arr[pos+2] << 16) | (data_arr[pos+3] << 24);
					ui32_data[i] = v32;
					pos += sizeof(uint32_t);
				}
				if (loadDeviceSettings(&ui32_data[0], i))
				{
					hdr->data[0] = DATA_OK;
					hdr->data[1] = msg_was_treated;
					hdr->data[2] = (uint8_t)(device_serial & 0xFF);
					device_settings_OK = True;
				}
				else
				{
					hdr->data[0] = DATA_FAILED;
					hdr->data[1] = msg_was_treated;
					hdr->data[2] = (uint8_t)(device_serial & 0xFF);
					device_settings_OK = False;
				}
			}

			sendShortMsg(hdr, uartRegs);

			incom_msg_state = NOT_DEFINED;
			//tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			disableCache();
		}
	}
}

void responseMultypackHeader(MsgHeader *_msg_header)
{
	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = _msg_header->id;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
	hdr->data[0] = HEADER_OK;

	sendShortMsg(hdr, uartRegs);
	outcom_msg_state = MESSAGE_SENT;
}
// ***************************************************************************************


/*
int newDataAvailable()
{
	//_disable_interrupts();
	int res = 0;
	if ( ds_new_data_enabled > 0 && (current_data_sample > ds_proc_data_index) )
	{
		res = current_data_sample - ds_proc_data_index;
	}
	//_enable_interrupts();

	return res;
}
*/


void executeProcPack(Data_Proc *proc, int index)
{
	/*
	uint8_t _device_id = ds->tool_id;
	uint8_t _channel_id = ds->channel_id;
	uint8_t _proc_index = ds->proc_id;
	uint16_t _data_len = ds->data_len;
	uint16_t _data_number = ds->echo_number;
	*/
	//uint8_t *upp_data_ptr = ds->data_ptr;

	//uint8_t temp[UPP_BUFF_SIZE];
	//memcpy(&temp[0], ds->data_ptr, UPP_BUFF_SIZE);

	//processing_params->channel_id = _channel_id;
	//processing_params->current_echo = _data_number;
	//processing_params->points_count = _data_len;

	if (proc->proc_lens[index] <= 0) return; // если в пакете инструкций не было найдено инструкций, то выход

	while (next_DataProcCmd(index, proc, instr) == True)
	{
		switch (instr->cmd)
		{
		case INS_WR_D0_ST:		STACKPtrF_push(ptr_data_org, data_stack); 		break;
		case INS_WR_D1_ST:		STACKPtrF_push(ptr_data1, data_stack);			break;
		case INS_WR_D2_ST:		STACKPtrF_push(ptr_data2, data_stack);			break;
		case INS_WR_D3_ST:		STACKPtrF_push(ptr_data3, data_stack);			break;
		case INS_WR_DX_ST:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			switch (indexX)
			{
			case 0:	STACKPtrF_push(ptr_data_org, data_stack);	break;
			case 1: STACKPtrF_push(ptr_data1, data_stack);		break;
			case 2: STACKPtrF_push(ptr_data2, data_stack);		break;
			case 3: STACKPtrF_push(ptr_data3, data_stack);		break;
			case 4: STACKPtrF_push(ptr_data4, data_stack);		break;
			case 5: STACKPtrF_push(ptr_data5, data_stack); 		break;
			case 6: STACKPtrF_push(ptr_data6, data_stack);		break;
			case 7: STACKPtrF_push(ptr_data7, data_stack);		break;
			default: break;
			}
			break;
		}
		/*case INS_WR_HX_ST:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			STACKPtrF_push(data_heap[indexX], data_stack);
			break;
		}*/
		case INS_CP_ST_D0:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data_org); break;
		case INS_CP_ST_D1:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data1); break;
		case INS_CP_ST_D2:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data2); break;
		case INS_CP_ST_D3:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data3); break;
		case INS_CP_ST_DX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];

			float *dst = 0;
			switch (indexX)
			{
			case 0: dst = ptr_data_org; break;
			case 1: dst = ptr_data1; break;
			case 2: dst = ptr_data2; break;
			case 3: dst = ptr_data3; break;
			case 4: dst = ptr_data4; break;
			case 5: dst = ptr_data5; break;
			case 6: dst = ptr_data6; break;
			case 7: dst = ptr_data7; break;
			default: break;
			}

			copy_DataTo(data_stack, DATA_MAX_LEN, dst);
			break;
		}
		/*case INS_CP_ST_HX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			copy_DataTo(data_stack, DATA_MAX_LEN, data_heap[indexX]);
			break;
		}*/
		case INS_MV_ST_D0:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data_org); break;
		case INS_MV_ST_D1: 	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data1); break;
		case INS_MV_ST_D2:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data2); break;
		case INS_MV_ST_D3:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data3); break;
		case INS_MV_ST_DX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];

			float *dst = 0;
			switch (indexX)
			{
			case 0: dst = ptr_data_org; break;
			case 1: dst = ptr_data1; break;
			case 2: dst = ptr_data2; break;
			case 3: dst = ptr_data3; break;
			case 4: dst = ptr_data4; break;
			case 5: dst = ptr_data5; break;
			case 6: dst = ptr_data6; break;
			case 7: dst = ptr_data7; break;
			default: break;
			}
			move_FromStack(data_stack, DATA_MAX_LEN, dst);
			break;
		}
		/*case INS_MV_ST_HX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			move_FromStack(data_stack, DATA_MAX_LEN, data_heap[indexX]);
			break;
		}*/
		case INS_ST_SWAP:		STACKPtrF_swap(data_stack); break;
		//case INS_EMUL_NS:		emulate_NoiseData(upp_data_ptr, processing_params); break;
		//case INS_EMUL_FID:		emulate_FIDData(upp_data, processing_params); break;
		//case INS_EMUL_SE:		emulate_EchoData(upp_data_ptr, processing_params); break;
		case INS_EMUL_FID_NS:	emulate_FIDNoiseData(instr, upp_buffer, processing_params); break;
		case INS_EMUL_SE_NS:	emulate_EchoNoiseData(instr, upp_buffer, processing_params); break;
		case INS_OPER_FID:		cast_UPPDataToFID(upp_buffer, processing_params->points_count, ptr_data_org); break;
		case INS_OPER_SE:		cast_UPPDataToSE(upp_buffer, processing_params->points_count,	ptr_data_org); break;
		case INS_OPER_FID_D:	cast_UPPDataToFID2(upp_buffer, processing_params, ptr_data_org); break;
		case INS_OPER_SE_D:		cast_UPPDataToSE2(upp_buffer, processing_params, ptr_data_org); break;
		case INS_DO_OPER1:		do_MathOperationVal(data_stack, DATA_MAX_LEN, summ_data, instr); break;
		case INS_DO_OPER2:		do_MathOperationBin(data_stack, DATA_MAX_LEN, instr); break;
		case INS_DO_XX_OPER:	do_MathOperationXX(summ_data, instr); break;
		case INS_ADD_TO_XX:		add_ValueToXX(summ_data, instr); break;
		case INS_ACC_DAT:		accumulate_Data(data_stack, DATA_MAX_LEN, processing_params); break;
		case INS_SMOOTH_DAT:	accsmooth_Data(data_stack, DATA_MAX_LEN, processing_params, instr);	break;
		case INS_QD_FID:		do_QuadDetect(data_stack); break;
		case INS_QD_SE:			do_QuadDetect(data_stack); break;
		case INS_WIN_TIME:		setWinFuncParamsPro(instr, TIME_DOMAIN_DATA, processing_params); break;
		case INS_WIN_FREQ:		setWinFuncParamsPro(instr, FREQ_DATA, processing_params); break;
		case INS_APP_WIN_TIME:	apply_WinFunc(TIME_DOMAIN_DATA, processing_params, data_stack); break;
		case INS_APP_WIN_FREQ:	apply_WinFunc(FREQ_DATA, processing_params, data_stack); break;
		case INS_FPW:			calc_PowerSpec(data_stack, DATA_MAX_LEN); break;
		case INS_FAMPL:			calc_AmplSpec(data_stack, DATA_MAX_LEN); break;
		case INS_AMP1:			estimate_SignalAmp1(data_stack, DATA_MAX_LEN); break;
		case INS_SPEC_MAX:		estimate_MaxSpectrum1(data_stack, DATA_MAX_LEN / 2, processing_params); break;
		case INS_SUM_DAT:		summarize_Data(data_stack, DATA_MAX_LEN / 2, summ_data, instr); break;
		case INS_SUM_REL_DAT:	summarize_DataForRelax(data_stack, DATA_MAX_LEN / 2, summ_data, processing_params, instr); break;
		case INS_ST_AVER:		average_Data(data_stack, processing_params->points_count, summ_data, instr); break;
		case INS_ZERO_ST:		fill_ByValue(data_stack, DATA_MAX_LEN, 0x00); break;
		case INS_NAN_ST:		fill_ByValue(data_stack, DATA_MAX_LEN, 0xFF); break;
		case INS_CL_ST:			STACKPtrF_clear(data_stack); break;
		case INS_WR_XX_SUMBUF:	write_ValueToSummationBuffer(summ_data, instr); break;
		case INS_MV_ST_OUTBUF:	move_ToOutputBuffer(data_stack, output_data, processing_params, instr->type); break;
		case INS_ACC_TO_OUTBUF:	move_AccToOutputBuffer(data_stack, summ_data, output_data, processing_params); break;
		case INS_XX_TO_OUTBUF:	move_XXToOutBuffer(summ_data, output_data, processing_params, instr); break;
		case INS_ST_DEC_OUTBUF:	decimateDataInOutputbuffer(data_stack, output_data, processing_params, instr); break;
		case INS_GET_GAMMA:
		{
			uint32_t gamma_counts = proger_rd_gamma_count();

			float *XX = 0;
			int num = instr->params[0];
			switch (num)
			{
			case X0: XX = &summ_data->xx[0]; break;
			case X1: XX = &summ_data->xx[1]; break;
			case X2: XX = &summ_data->xx[2]; break;
			case X3: XX = &summ_data->xx[3]; break;
			default: break;
			}

			if (XX != 0)
			{
				*XX = (float) gamma_counts;
			}
			break;
		}
		case INS_FFT:
		{
			if (data_stack->cnt < 2) break; // в стеке должны быть буффер-приемник данных и источник данных (на вершине стека)
			float *src = STACKPtrF_pop(data_stack);
			memcpy(&temp_data[0], src - PAD, UPP_DATA_SIZE); // дублирование данных src, т.к. они разрушаются при выполнении функции DSPF_sp_fftSPxSP(...)
			float *dst = STACKPtrF_first(data_stack);
			DSPF_sp_fftSPxSP(CMPLX_DATA_MAX_LEN, ptr_temp_data, ptr_w, dst, brev, rad, 0, CMPLX_DATA_MAX_LEN);
			break;
		}
		case INS_IF_DATA_NUM:
		{
			if (instr->count != 2) break;
			int data_number = (int) instr->params[0];
			int instr_count = (int) instr->params[1];
			if (processing_params->current_echo != data_number)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_WR_X0:
		{
			if (instr->count != 1) break;
			XX[0] = (float) instr->params[0];
			break;
		}
		case INS_WR_X1:
		{
			if (instr->count != 1) break;
			XX[1] = (float) instr->params[0];
			break;
		}
		case INS_WR_X2:
		{
			if (instr->count != 1) break;
			XX[2] = (float) instr->params[0];
			break;
		}
		case INS_WR_X3:
		{
			if (instr->count != 1) break;
			XX[3] = (float) instr->params[0];
			break;
		}
		case INS_WR_ACC_GRIX:
		{
			if (instr->count != 1) break;
			summ_data->group_index = (int) instr->params[0];
			summ_data->channel_id = processing_params->channel_id; //proger_rd_ch_number();
			//int data_cnt = output_data->outdata_counter;
			//output_data->group_index[data_cnt] = (int) instr->params[0];
			break;
		}
		case INS_IF_COND_X0:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X0, код условия, количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) res = (XX[0] == value); // condition code == 0 - "=="
			else if (cond_code == 1) res = (XX[0] > value); // condition code == 1 - ">"
			else if (cond_code == 2) res = (XX[0] >= value); // condition code == 2 - ">="
			else if (cond_code == 3) res = (XX[0] < value); // condition code == 3 - "<"
			else if (cond_code == 4) res = (XX[0] <= value); // condition code == 4 - "<="
			else if (cond_code == 5) res = (XX[0] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X1:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X1, код условия, количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) res = (XX[1] == value); // condition code == 0 - "=="
			else if (cond_code == 1) res = (XX[1] > value); // condition code == 1 - ">"
			else if (cond_code == 2) res = (XX[1] >= value); // condition code == 2 - ">="
			else if (cond_code == 3) res = (XX[1] < value); // condition code == 3 - "<"
			else if (cond_code == 4) res = (XX[1] <= value); // condition code == 4 - "<="
			else if (cond_code == 5) res = (XX[1] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X2:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X2, код условия, количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) res = (XX[2] == value); // condition code == 0 - "=="
			else if (cond_code == 1) res = (XX[2] > value); // condition code == 1 - ">"
			else if (cond_code == 2) res = (XX[2] >= value); // condition code == 2 - ">="
			else if (cond_code == 3) res = (XX[2] < value); // condition code == 3 - "<"
			else if (cond_code == 4) res = (XX[2] <= value); // condition code == 4 - "<="
			else if (cond_code == 5) res = (XX[2] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X3:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X3, код условия, количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) res = (XX[3] == value); // condition code == 0 - "=="
			else if (cond_code == 1) res = (XX[3] > value); // condition code == 1 - ">"
			else if (cond_code == 2) res = (XX[3] >= value); // condition code == 2 - ">="
			else if (cond_code == 3) res = (XX[3] < value); // condition code == 3 - "<"
			else if (cond_code == 4) res = (XX[3] <= value); // condition code == 4 - "<="
			else if (cond_code == 5) res = (XX[3] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND:
		{
			if (instr->count != 3) break; // инструкция содержит три параметра: два числа, которые сравнивается, количество инструкций, на которое необходимо перейти в случае неравенства чисел
			float value1 = instr->params[0];
			float value2 = instr->params[1];
			int instr_count = (int) instr->params[2];
			if (value1 != value2)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		//case INS_CLEAR_HX:			clearDataHeap(data_heap, &data_heap_len[0], instr); break;
		//case INS_NOISE_UPP_PRE1:	noiseUppDataPreprocessing1(upp_buffer, bank, data_heap, &data_heap_len[0], processing_params, instr); break;
		//case INS_SGN_UPP_PRE1:		signalUppDataPreprocessing1(upp_buffer, bank, data_heap, processing_params, instr); break;
		//case INS_NS_SGN_UPP_PRE3:	signal_noise_UppDataPreprocessing(ds, bank, data_heap, &data_heap_len[0], processing_params, instr); break;
		case INS_NOISE_PROC1:		noiseProcessing1(upp_buffer, bank, rad, instr, processing_params, output_data); break;
		//case INS_SGN_PROC1:			signalProcessing1(upp_data_ptr, bank, rad, instr, processing_params, summ_data, output_data); break;
		//case INS_SGN_PROC1:			NMR_data_Processing(data_samples, current_data_sample, bank, processing_params, instr, output_data); break;
		case INS_SGN_PROC1:			NMR_data_Processing(data_samples, current_data_sample, rad, bank, processing_params, instr, output_data); break;
		//case INS_NOISE_PROC2:		noiseProcessing2(bank, data_heap, rad, instr, processing_params, output_data); break;
		//case INS_SGN_PROC2:			signalProcessing2(bank, data_heap, rad, instr, processing_params, summ_data, output_data); break;
		//case INS_SGN_PROC3:			signalProcessing3(bank, data_heap, rad, instr, processing_params, summ_data, output_data); break;
		case INS_GO_TO:
		{
			if (instr->count != 1) break;
			int instr_count = (int) instr->params[0];
			pass_DataProcCmds(index, proc, instr_count);
			break;
		}
		case INS_NO_OP: break;
		default: break; // выход, если встретилась незнакомая команда
		}

		//free_DataProcCmd(instr);
	}
}

#ifdef USE_TELEMETRIC_UART
void toMeasureTemperatures(void)
{
	UART_telemetric_counter = 0;
	UART_telemetric_pack_counter = 0;
	telemetric_board_status = 0;

	if (temperature_mode == TEMP_MODE_NOT_SPVP)	// KMRK, NMKT и другие кроме SPVP
	{
		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));

		// DU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		//proger_restart_time_counter();
		dummyDelay(100);
		//volatile int tt = proger_read_time_counter();

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// TU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// PU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);
	}
	else if (temperature_mode == TEMP_MODE_SPVP)	// SPVP only
	{
		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN2 * sizeof(uint8_t));

		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'c');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(1000);
	}
}
#endif


void create_Clockers(void)
{
	// *********** create clockers *************
	clockers = (Clocker**) calloc(NUM_CLOCKERS, sizeof(Clocker*));

	// create system application clocker
	clocker0 = (Clocker*) malloc(sizeof(Clocker));
	clockers[0] = clocker0;
	initClocker(10000, clocker0_ISR, clocker0);

	// create UART message clockers (for header trapping)
	clocker1 = (Clocker*) malloc(sizeof(Clocker));
	clockers[1] = clocker1;
	initClocker(2000, clocker1_ISR, clocker1); // it was 20

	// create UART message clockers (for packet trapping)
	clocker2 = (Clocker*) malloc(sizeof(Clocker));
	clockers[2] = clocker2;
	initClocker(1000, clocker2_ISR, clocker2); // it was 120

	// create clocker for repetition time tests (delay between pulse sequences)
	clocker3 = (Clocker*) malloc(sizeof(Clocker));
	clockers[3] = clocker3;
	initClocker(1000, clocker3_ISR, clocker3);

	// create clocker for telemetry measurements (delay between measurements)
	clocker4 = (Clocker*) malloc(sizeof(Clocker));
	clockers[4] = clocker4;
	initClocker(3000, clocker4_ISR, clocker4);

	// create clocker for SDSP measurements (delay for measurements)
	clocker5 = (Clocker*) malloc(sizeof(Clocker));
	clockers[5] = clocker5;
	initClocker(200, clocker5_ISR, clocker5);

	//startClocker(app_clocker);
	// ******************************************
}


void initDeviceSettings(uint8_t device)
{
	switch (device)
	{
	case KMRK:		// KMRK
	{
		device_channel_count = 8;
		device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

		device_channels[0].type = NMR_CHANNEL;
		device_channels[0].channel_id = 0;
		device_channels[0].freq_set_num = 0;
		device_channels[0].frq1 = 0;
		device_channels[0].frq2 = 0;
		device_channels[0].frq3 = 0;
		device_channels[0].frq4 = 0;
		device_channels[0].frq5 = 0;
		device_channels[0].frq6 = 0;
		device_channels[0].addr_rx = 0;
		device_channels[0].addr_tx = 0;

		device_channels[1].type = NMR_CHANNEL;
		device_channels[1].channel_id = 1;
		device_channels[1].freq_set_num = 1;
		device_channels[1].frq1 = 0;
		device_channels[1].frq2 = 0;
		device_channels[1].frq3 = 0;
		device_channels[1].frq4 = 0;
		device_channels[1].frq5 = 0;
		device_channels[1].frq6 = 0;
		device_channels[1].addr_rx = 0;
		device_channels[1].addr_tx = 0;

		device_channels[2].type = GK_CHANNEL;
		device_channels[2].freq_set_num = 0;
		device_channels[2].channel_id = 2;

		device_channels[3].type = SDSP_CHANNEL;
		device_channels[3].freq_set_num = 0;
		device_channels[3].channel_id = 3;

		device_channels[4].type = AFR_CHANNEL;
		device_channels[4].freq_set_num = 1;
		device_channels[4].channel_id = 4;

		device_channels[5].type = AFR_CHANNEL;
		device_channels[5].freq_set_num = 1;
		device_channels[5].channel_id = 5;

		device_channels[6].type = RF_PULSE_CHANNEL;
		device_channels[6].freq_set_num = 1;
		device_channels[6].channel_id = 6;

		device_channels[7].type = RF_PULSE_CHANNEL;
		device_channels[7].freq_set_num = 1;
		device_channels[7].channel_id = 7;

		break;
	}
	default:
	{
		device_channel_count = 8;
		device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

		device_channels[0].type = NMR_CHANNEL;
		device_channels[0].freq_set_num = 0;
		device_channels[0].channel_id = 0;
		device_channels[0].frq1 = 0;
		device_channels[0].frq2 = 0;
		device_channels[0].frq3 = 0;
		device_channels[0].frq4 = 0;
		device_channels[0].frq5 = 0;
		device_channels[0].frq6 = 0;
		device_channels[0].addr_rx = 0;
		device_channels[0].addr_tx = 0;

		device_channels[1].type = NMR_CHANNEL;
		device_channels[1].freq_set_num = 1;
		device_channels[1].channel_id = 1;
		device_channels[1].frq1 = 0;
		device_channels[1].frq2 = 0;
		device_channels[1].frq3 = 0;
		device_channels[1].frq4 = 0;
		device_channels[1].frq5 = 0;
		device_channels[1].frq6 = 0;
		device_channels[1].addr_rx = 1;
		device_channels[1].addr_tx = 1;

		device_channels[2].type = GK_CHANNEL;
		device_channels[2].freq_set_num = 0;
		device_channels[2].channel_id = 2;

		device_channels[3].type = SDSP_CHANNEL;
		device_channels[3].freq_set_num = 0;
		device_channels[3].channel_id = 3;

		device_channels[4].type = AFR_CHANNEL;
		device_channels[4].freq_set_num = 1;
		device_channels[4].channel_id = 4;
		device_channels[4].addr_rx = 0;
		device_channels[4].addr_tx = 2;

		device_channels[5].type = AFR_CHANNEL;
		device_channels[5].freq_set_num = 1;
		device_channels[5].channel_id = 5;
		device_channels[5].addr_rx = 1;
		device_channels[5].addr_tx = 2;

		device_channels[6].type = RF_PULSE_CHANNEL;
		device_channels[6].freq_set_num = 1;
		device_channels[6].channel_id = 6;
		device_channels[6].addr_rx = 2;
		device_channels[6].addr_tx = 0;

		device_channels[7].type = RF_PULSE_CHANNEL;
		device_channels[7].freq_set_num = 1;
		device_channels[7].channel_id = 7;
		device_channels[7].addr_rx = 2;
		device_channels[7].addr_tx = 1;

		break;
	}
	}
}


Bool loadDeviceSettings(int *data, int len)
{
	int struct_size = sizeof(ToolChannel);
	int pos = 0;
	int channels_number = len*sizeof(uint32_t)/struct_size;

	if (channels_number > 0) free(device_channels);
	else return False;

	device_channel_count = channels_number;
	device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

	int index = 0;
	while (index < channels_number)
	{
		uint32_t _data_type = data[pos++];
		uint32_t _channel_id = data[pos++];
		uint32_t _freq_set_num = data[pos++];
		uint32_t _addr_rx = data[pos++];
		uint32_t _addr_tx = data[pos++];
		uint32_t _freq1 = data[pos++];
		uint32_t _freq2 = data[pos++];
		uint32_t _freq3 = data[pos++];
		uint32_t _freq4 = data[pos++];
		uint32_t _freq5 = data[pos++];
		uint32_t _freq6 = data[pos++];
		device_channels[index].type = _data_type;
		device_channels[index].channel_id = _channel_id;
		device_channels[index].freq_set_num = _freq_set_num;
		device_channels[index].addr_rx = _addr_rx;
		device_channels[index].addr_tx = _addr_tx;
		device_channels[index].frq1 = _freq1;
		device_channels[index].frq2 = _freq2;
		device_channels[index].frq3 = _freq3;
		device_channels[index].frq4 = _freq4;
		device_channels[index].frq5 = _freq5;
		device_channels[index].frq6 = _freq6;
		index++;
	}

	return True;
}

void setDefaultCommSettings()
{
	msg_settings->block_len = 20;
	gf_data->index_body = 1;
	msg_settings->rec_errs = gf_data->index_body + 1;
	msg_settings->pack_len = 200;
	msg_settings->antinoise_coding = True;
	msg_settings->packlen_autoadjust = False;
	msg_settings->interleaving = False;
	msg_settings->pack_delay = 0;
}


// DataHeap procedures... -----------------------------------------------------
/*
void initDataHeap()
{
	//data_heap = (float**)calloc(DATA_HEAP_COUNT, sizeof(float*));
	int i;
	for (i = 0; i < DATA_HEAP_COUNT; i++)
	{
		float *heap_arr = (float*)calloc(DATA_MAX_LEN, sizeof(float));
		data_heap[i] = heap_arr;
		data_heap_len[i] = 0;
	}
	//data_heap_counter = 0;
}

void resetDataHeap()
{
	int i;
	for (i = 0; i < DATA_HEAP_COUNT; i++)
	{
		data_heap_len[i] = 0;
	}
	//data_heap_counter = 0;
}
*/
//-----------------------------------------------------------------------------

// Init Samples Buffer --------------------------------------------------------
void initDataSamples(DataSample *data_samples[])
{
	int i;
	for (i = 0; i < UPP_DATA_COUNT; i++)
	{
		DataSample *data_sample = (DataSample*) malloc(sizeof(DataSample));
		//data_sample->data_ptr = samples_buffer[i];
		data_sample->data_ptr = 0;
		data_sample->data_len = 0;
		data_sample->echo_number = 0;
		data_sample->proc_id = 0;
		data_sample->tool_id = 0;
		data_sample->channel_id = 0;
		//data_sample->heap_ptr = data_heap[i];
		//data_sample->heap_len = 0;
		data_sample->tag = 0;
		data_samples[i] = data_sample;
	}

	current_data_sample = 0;
	samples_buffer_pos = 0;
	ds_new_data_enabled = 1;
	//ds_proc_data_index = 0;
}

/*
void resetDataSamples(int _from, int _count)
{
	int i;
	int _to = _from + _count;
	for (i = _from; i < _to; i++)
	{
		volatile DataSample *data_sample = data_samples[i];
		uint8_t *data_arr = data_sample->data_ptr;
		memset(data_arr, 0x00, DATA_MAX_LEN*sizeof(uint8_t));
		float *heap_arr = data_sample->heap_ptr;
		memset(heap_arr, 0x00, DATA_MAX_LEN*sizeof(float));
		data_sample->data_len = 0;
		data_sample->echo_number = 0;
		data_sample->proc_id = 0;
		data_sample->tool_id = 0;
		data_sample->heap_ptr = data_heap[i];
		data_sample->heap_len = 0;
		data_sample->tag = 0;
	}

	current_data_sample = 0;
	ds_new_data_enabled = 1;
	ds_proc_data_index = 0;
}
*/
// ----------------------------------------------------------------------------


/*-----------------------------------------------------------------------------
 * 							Interrupt Functions
 *---------------------------------------------------------------------------*/
interrupt void TIMER0_12_isr(void)
{
	int i;
	for (i = 0; i < NUM_CLOCKERS; i++)
	{
		if (clockers[i]->state == CLR_STARTED)
		{
			if (clockers[i]->counts >= clockers[i]->max_val)
			{
				clockers[i]->state = CLR_FINISHED;
				clockers[i]->ptr_isr();
				clockers[i]->counts = 0;
				clockers[i]->tag++;
			}
			else clockers[i]->counts++;
		}
	}
}


interrupt void UART_isr(void)
{
	uint8_t byte;

	// Determine Prioritized Pending UART Interrupt
	int uartStatus = CSL_FEXT(uartRegs->IIR, UART_IIR_INTID);

	// Set Appropriate Bool
	if (uartStatus == E_DATA_READY)
	{
		uartStatus = read_UART(uartRegs, &byte);
		if (uartStatus == E_OK)
		{
#ifdef LOOPBACK_COMM_UART
			CSL_FINS(uartRegs->THR, UART_THR_DATA, byte);
#endif

			// -------- START and STOP bytes --------
			Bool flag = False;
			//printf("Byte: %d\n", byte);
			if (msg_header_state == NOT_DEFINED)
			{
				if (byte == START_BYTE)
				{
					msg_header_state = STARTED;
					flag = True;
					startClocker(clocker1);
				}
			}
			else if (msg_header_state == STARTED)
			{
				int sz = QUEUE8_count(uart_queue);
				if (sz == HEADER_LEN)
				{
					if (byte == STOP_BYTE)
					{
						incom_msg_state = STARTED;
						flag = True;
					}
				}
			}
			// ------------------------------------

			if (flag == False) QUEUE8_put(byte, uart_queue);
			dataUnavailable = FALSE;
		}
	}
	else if (uartStatus == E_TRAN_BUF_EMPTY) transmitterFull = FALSE;
}

interrupt void UART_Dielec_isr(void)
{
#ifdef USE_DIELEC_UART

#define UART_REC_BUF_FULL_INT 2
#define UART_TRAN_BUF_EMPTY_INT 1

	unsigned char ch;
	volatile unsigned int uart_Dielec_Status;

	// Determine Prioritized Pending UART Interrupt
	uart_Dielec_Status = CSL_FEXT(uartRegs_Dielec->IIR, UART_IIR_INTID);

	// Set Appropriate Bool
	if (uart_Dielec_Status == UART_REC_BUF_FULL_INT)
	{
		ch = CSL_FEXT(uartRegs_Dielec->RBR, UART_RBR_DATA);
#endif
#ifdef LOOPACK_DIELEC_UART
		uart_hduplex_sendchar(uartRegs_Dielec, ch);
#endif
#ifdef USE_DIELEC_UART
		dielec_data[UART_Dielec_counter] = ch;

		UART_Dielec_counter++;

		if (UART_Dielec_counter == (DIELECTR_DATA_LEN * sizeof(uint8_t))) // buffer overflow
				{
			UART_Dielec_counter = 0;
		};
	}
	else if (uart_Dielec_Status == UART_TRAN_BUF_EMPTY_INT)
	{

	}; //transmitterFull = FALSE;
#endif
}

interrupt void UART_Telemitric_isr(void)
{
#ifdef USE_TELEMETRIC_UART
	uint8_t byte;

	int uartStatus = read_UART(uartRegs_Telemetric, &byte);
	if (uartStatus == E_OK)
	{
		if (temperature_mode == TEMP_MODE_NOT_SPVP)
		{
			if (UART_telemetric_counter >= TELEMETRIC_UART_BUF_LEN)
			{
				telemetric_board_status = 0;
				UART_telemetric_counter = 0;
				UART_telemetric_local_counter = 0;
			}
			if (UART_telemetric_local_counter >= TELEMETRIC_DATA_LEN) UART_telemetric_local_counter = 0;

			int pos = TELEMETRIC_DATA_LEN * (UART_telemetric_pack_counter - 1) + UART_telemetric_local_counter;
			UART_telemetric_local_counter++;
			UART_telemetric_counter++;

			switch (UART_telemetric_pack_counter)
			{
			case 1: // DU board
				telemetric_board_status |= 1;
				break;
			case 2: // TU board
				telemetric_board_status |= 2;
				break;
			case 3: // PU board
				telemetric_board_status |= 4;
				break;
			}

			telemetric_data[pos] = byte;
		}
		else if (temperature_mode == TEMP_MODE_SPVP)
		{
			if (temp_request_mode == TEMP_NOT_DEFINED && byte == 1)
			{
				dummyDelay(1000);
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');

				temp_request_mode = TEMP_STARTED;
				temp_sensors = 0;
				UART_telemetric_counter = 0;
				UART_telemetric_local_counter = 0;
			}
			else if (temp_request_mode == TEMP_STARTED)
			{
				temp_sensors = byte;
				temp_request_mode = TEMP_COUNTED;
			}
			else if (temp_request_mode == TEMP_COUNTED && temp_sensors > 0)
			{
				telemetric_data[UART_telemetric_counter++] = byte;
				UART_telemetric_local_counter++;
			}

			if (UART_telemetric_local_counter == 2*temp_sensors && temp_request_mode == TEMP_COUNTED)
			{
				temp_request_mode = TEMP_READY;		// прием температур завершен!
				UART_telemetric_local_counter = 0;

				// измерение напряжений
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'v');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
				voltage_request_mode = VOLT_STARTED;
				return;
			}

			if (voltage_request_mode == VOLT_STARTED && UART_telemetric_local_counter < 8)
			{
				telemetric_data[UART_telemetric_counter++] = byte;
				UART_telemetric_local_counter++;
				//printf("Byte: %d\n", byte);
			}

			if (voltage_request_mode == VOLT_STARTED && UART_telemetric_local_counter == 8)
			{
				voltage_request_mode = VOLT_READY;		// прием напряжений завершен!
				UART_telemetric_local_counter = 0;
				//printf("Bytes totally: %d\n", UART_telemetric_counter);
				telemetry_ready = TELE_READY;
			}
		}
	}
#endif
}


interrupt void GPIO_isr(void)
{
	/* The interrupt handler for the GPIO interrupts                          */

	/* the interrupt could have been because of any one of the pins in the    *
	* bank 0. Hence we will only check if the pin3 or pin2 or pin1 is         *
	* generating the interrupt and then reset it and exit.                    */

	if (gpioRegs->BANK[0].INTSTAT & CSL_GPIO_INTSTAT_STAT1_MASK)
	{
		pins_reg = GPIO_B0_RD();
		pin1_state = (pins_reg & 0x02) >> 1;
		pins_cmd = (pins_reg >> 5) & 0x000000FF;

		if (pin1_state == 0)
		{
			switch (pins_cmd)
			{
			case NMR_TOOL:
			case DUMMY_TOOL:
			case GAMMA_TOOL:
			{
				device_id = pins_cmd;
				channel_id = proger_rd_ch_number();
				break;
			}
			default:
			{
				device_id = 0;
				channel_id = 0;
				break;
			}
			}
		}
		else if (pin1_state == 1)
		{
			switch (device_id)
			{
			case NMR_TOOL:
			{
				upp_resetted = upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки

				ds_new_data_enabled = 0;

				int upp_data_len = proger_rd_adc_points_count();
				int upp_echo_number = proger_rd_echo_count();

				if ((samples_buffer_pos + upp_data_len) < SAMPLES_BUFFER_SIZE || current_data_sample < (UPP_DATA_COUNT - 1) )
				{
					data_samples[current_data_sample]->tool_id = device_id;
					data_samples[current_data_sample]->channel_id = channel_id;
					data_samples[current_data_sample]->data_len = upp_data_len;
					data_samples[current_data_sample]->echo_number = upp_echo_number;
					data_samples[current_data_sample]->proc_id = pins_cmd;
					data_samples[current_data_sample]->data_ptr = samples_buffer + samples_buffer_pos;
					samples_buffer_pos += upp_data_len;

					setupDDR2Cache();
					enableCacheL1();
					memcpy(data_samples[current_data_sample]->data_ptr, upp_buffer, upp_data_len);
					disableCache();

					processing_params->channel_id = channel_id;
					processing_params->current_echo = upp_echo_number;
					processing_params->points_count = upp_data_len;

					current_data_sample++;
				}

				seq_completed = 0;
				ds_new_data_enabled = 1;

				upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

				break;
			}
			case DUMMY_TOOL:
			{
				//processing_params->proc_cmd = pins_cmd;
				data_samples[current_data_sample]->tool_id = device_id;
				data_samples[current_data_sample]->channel_id = channel_id;
				data_samples[current_data_sample]->data_len = 0;
				data_samples[current_data_sample]->echo_number = 0;
				data_samples[current_data_sample]->proc_id = pins_cmd;
				data_samples[current_data_sample]->data_ptr = NULL;

				seq_completed = 1;	// можно приступить к обработке данных NMR

				break;
			}
			case GAMMA_TOOL:
			{
				int proc_index = pins_cmd;
				if (proc_index < MAX_PROCS)
				{
					move_ToFirstDataProcCmd(proc_index - 1, instr_prg);
					executeProcPack(instr_prg, proc_index - 1);
				}

				device_id = 0; // No device
				break;
			}
			default:
			{
				device_id = 0;
				channel_id = 0;
				break;
			}
			}
		}

	    /* reset the interrupt status register                                */
		gpioRegs->BANK[0].INTSTAT |= 0x02;
	}
	if (gpioRegs->BANK[0].INTSTAT & CSL_GPIO_INTSTAT_STAT2_MASK)
	{
		/* reset the interrupt source (so that multiple interrupts dont ccur  */
	    //CSL_FINS(gpioRegs->BANK[0].OUT_DATA,GPIO_OUT_DATA_OUT2,0);

	    /* reset the interrupt status register                                */
	    CSL_FINS(gpioRegs->BANK[0].INTSTAT, GPIO_INTSTAT_STAT2, 0);
	}
	if (gpioRegs->BANK[0].INTSTAT & CSL_GPIO_INTSTAT_STAT3_MASK)
	{
		unsigned int pins_reg = GPIO_B0_RD();
		uint8_t pin3_state = (pins_reg & 0x08) >> 3;

		if (pin3_state == 1)
		{
			tool_state = READY;
			//seq_completed = 1; //перенесено выше. Теперь признаком завершения последовательности является приход DUMMY_TOOL - можно приступить обрабатывать данные
			//printf("Number of last obtained echo: %d\n", current_data_sample);
		}
		else if (pin3_state == 0)
		{
			//ds_proc_data_index = 0;
			current_data_sample = 0;
			samples_buffer_pos = 0;
			seq_completed = 0;

			tool_state = NOT_READY;
		}

	    /* reset the interrupt status register                                */
		gpioRegs->BANK[0].INTSTAT |= 0x08;
	}
}

interrupt void upp_isr(void)
{
	//upp_isr_count++;
	//printf("\tUPP ISR OK!  # \t%d.", ++upp_isr_count);

	// Determine Pending Interrupt
	upp_int_status = UPP0Regs->UPISR;

	if ((upp_int_status & CSL_UPP_UPISR_EOWI_MASK) 	== (1 << CSL_UPP_UPISR_EOWI_SHIFT))
	{
		CSL_FINST(UPP0Regs->UPIER, UPP_UPIER_EOWI, TRUE);
		// clear int-t flag
		uppFull = TRUE;
	};

	if ((upp_int_status & CSL_UPP_UPISR_EOLI_MASK) == (1 << CSL_UPP_UPISR_EOLI_SHIFT))
	{
		CSL_FINST(UPP0Regs->UPIER, UPP_UPIER_EOLI, TRUE);
		// clear int-t flag
		//uppFull = TRUE;
	};
}


void clocker0_ISR(void)
{
	//printf("\nFinished !\n");
}

void clocker1_ISR(void)
{
	incom_msg_state = TIMED_OUT;
}

void clocker2_ISR(void)
{
	incom_msg_state = TIMED_OUT;
}

void clocker3_ISR(void)
{
	if (incom_msg_state == NOT_DEFINED)
	{
		uint8_t pg = (uint8_t) proger_rd_pwr_pg();
		uint8_t tele_flag = 0;
#ifdef USE_TELEMETRIC_UART
		if (temperature_mode == TEMP_MODE_NOT_SPVP)
		{
			if (UART_telemetric_counter % TELEMETRIC_DATA_LEN == 0 && UART_telemetric_counter > 0) tele_flag = 1;
		}
		if (temperature_mode == TEMP_MODE_SPVP)
		{
			if (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY)
			{
				tele_flag = 1;
			}
		}
#endif
#ifdef USE_PRESSURE_UNIT
		if (press_unit_ready == PRESS_UNIT_READY) tele_flag = 1;
		//press_unit_ready = PRESS_UNIT_NOT_READY;
#endif
		uint8_t pp_is_started  = proger_is_started();
		uint8_t pp_is_seq_done = proger_is_seq_done();
		uint8_t out_mask = pg | (tele_flag << 1) | (pp_is_started << 2) | (pp_is_seq_done << 3);
		sendByteArray(NMRTool_Ready[out_mask], SRV_MSG_LEN + 2, uartRegs);
		//pp_is_seq_done = proger_is_seq_done();
	}
	startClocker(clocker3);
}

void clocker4_ISR(void)
{
#ifdef USE_TELEMETRIC_UART
	//if (incom_msg_state == NOT_DEFINED)
	{
		temp_request_mode = TEMP_NOT_DEFINED;
		voltage_request_mode = VOLT_NOT_DEFINED;
		toMeasureTemperatures();
		//telemetry_ready = TELE_READY;
		telemetry_ready = TELE_NOT_READY;
	}
#endif
#ifdef USE_PRESSURE_UNIT
	press_unit_ready = PRESS_UNIT_READY;
#endif
	startClocker(clocker4);
}

void clocker5_ISR(void)
{
#ifdef USE_DIELEC_UART
	sdsp_ready = SDSP_READY;
#endif
}
// **************************************************************************************
