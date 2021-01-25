/*
 * proger.h
 *
 *  Created on: 17.12.2014
 *      Author: aivanov
 *
 *      version number see in proger_defines.h
 */

#ifndef PROGER_H_
#define PROGER_H_

#include "proger_defines.h"

void proger_stop ();
void proger_start ();

unsigned int proger_rd_reg_32 (unsigned int adr);
unsigned int proger_wr_reg_32 (unsigned int adr, unsigned int data);

unsigned int proger_read_time_counter ();
unsigned int proger_restart_time_counter ();

int proger_rd_acquisition_info (unsigned char *array, unsigned int byte_count);

int proger_rd_echo_count (void);
int proger_rd_gamma_count (void);
int proger_rd_adc_points_count (void);
double proger_rd_adc_frq_hz (void);
int proger_rd_pwr_pg (void);
int proger_rd_proger_type (void);
int proger_rd_proger_version (void);
int proger_rd_proger_sub_version (void);
int proger_rd_group_index(void);			 // ������� ������ ������ ������ ������������������� ("���������� �������"), ����������� � ������� ������ ����� ���������
unsigned int proger_read_gpio ();
unsigned int proger_rd_ch_number(void);      // ������� ������ ������ �������� ������ ������, �������������� �������� COM_SET_FRQ
unsigned int proger_rd_device_serial(void);  // ������� ������ ��������� ������ �������
unsigned int proger_rd_connect_speed(void);  // ������� ������ ������� �������� ������ ������� � ��������

unsigned int proger_rd_status(void);         // ������� ������ ������� ������������� (������� (1) ��� ���������� (0) � ���� 0, �������� (1) ��� �� �������� (0) ������������������ �� ������� COM_STOP  � ���� 1)
unsigned int proger_is_started(void);		 // ���������� 1 ���� ������������ ��� ������� �������� proger_start, ���������� 0 ���� ���������� �������� stop
unsigned int proger_is_seq_done(void);       // ���������� 1 ���� ������������ ����� �� ������� COM_STOP � ������� �������� proger_start, ���������� 0 �� ���� ������ �������

int proger_rd_conf_mem ( unsigned char *data );

int proger_wr_pulseprog (unsigned char *array, unsigned int byte_count);

int proger_reset_comm_fifo_counter (void);
int proger_wr_comm_arr_to_fifo (volatile unsigned char *comm);
int proger_wr_comm_to_fifo (unsigned char comm, unsigned char arg1, unsigned char arg2, unsigned char arg3);

int proger_rd_pulseprog (unsigned char *array, unsigned int byte_count);
int proger_compare_pulseprog (unsigned char *array, unsigned int byte_count);

int proger_mem_init ();
int proger_mem_clear ();

unsigned int proger_read_grad_status ();
unsigned int proger_start_grad ();


// Developed by Roman Kochkin -------------------------------------------------
// ��������� ������� "����": ��. enum state
unsigned char proger_read_mtr_status ();
// ���������� "�����": ��. enum motor_cmd_t
unsigned char proger_cmd_mtr ( unsigned char cmd);
// ��������� �������� �������� �������� �����, �������� ���������� ��������� ����� (4 �����)
signed int proger_read_counter_mtr (void);

typedef union {
	struct {
		enum {
				STATE_MOTOR_STOP 		= 0x0,
				STATE_MOTOR_UP			= 0x3,
				STATE_MOTOR_DOWN		= 0x1,
				STATE_MOTOR_BRAKE_UP	= 0x6,
				STATE_MOTOR_BRAKE_DOWN	= 0x4
		} state								:3;
		unsigned int switch_down 			:1;
		unsigned int switch_up 				:1;
		unsigned int						:0;
	} field;
	unsigned char reg;
} motor_status_reg_t;


typedef enum {
	CMD_MOTOR_STOP			= 0x0,
	CMD_MOTOR_UP			= 0x3,
	CMD_MOTOR_DOWN			= 0x1,
	CMD_MOTOR_COUNTER_RESET	= 0x4
} motor_cmd_t;

// ��������� ���, ������� �������� ���������� �� �������������, ������������ ��������� �����
unsigned char proger_mtr_adc_start_conversion(void);
// ������/�������� ��� �� �������������
unsigned char proger_read_mtr_adc_status(void); // 0 - busy, 1 -ready
// ������ �������� �� ��� (12 ���) ������������� (������� ���)
unsigned int proger_read_mtr_adc_value (unsigned char channel);
// ----------------------------------------------------------------------------


//int proger_test_mem_pulseprog ();
//int main_proger_wr_pulseprog_test_GPIO3 ();
//int main_proger_wr_pulseprog_test_GPIO_48f_ADC_Diel();
int main_proger_wr_pulseprog_test_GPIO_48f_ADC_FID ();
int main_proger_wr_pulseprog_test_GPIO_49q_ADC_old_style_windows (
															double       resonance_frq_ofs_hz,
															unsigned int rf_pulse_duration,
															unsigned int nmr_aq_time,
															unsigned int noise_aq_time,
															unsigned int dead_time,
															double       adc_frq_ofs_hz,
															unsigned int damping_pulse_duration_after_rf_pulse,
															unsigned int delay_before_rf_pulses
);

int main_proger_wr_pulseprog_test_FID (
										double       resonance_frq_ofs_hz,
										unsigned int rf_pulse_duration,
										unsigned int nmr_aq_time,
										unsigned int noise_aq_time,
										unsigned int dead_time,
										double       adc_frq_ofs_hz,
										unsigned int damping_pulse_duration_after_rf_pulse,
										unsigned int delay_before_rf_pulses
										);



#endif /* PROGER_H_ */
