/*
 * mpu6050.h
 *
 *  Created on: Mar 22, 2023
 *      Author: valentin
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#define MPU_6500

#ifdef MPU_6500
	#define mpu_6500_selftestXgyro 0x00
	#define mpu_6500_selftestYgyro 0x01
	#define mpu_6500_selftestZgyro 0x02

	#define mpu_6500_selftestXaccl 0x0D
	#define mpu_6500_selftestYaccl 0x0E
	#define mpu_6500_selftestZaccl 0x0F

#define mpu_6500_xg_offset_h 0x13
#define mpu_6500_xg_offset_l 0x14
#define mpu_6500_yg_offset_h 0x15
#define mpu_6500_yg_offset_l 0x16
#define mpu_6500_zg_offset_h 0x17
#define mpu_6500_zg_offset_l 0x18

#define mpu_6500_smprt_div 0x19
#define mpu_6500_config 0x1A
#define mpu_6500_gyro_config 0x1B
#define mpu_6500_accel_config 0x1C
#define mpu_6500_accel_config2 0x1D

#define mpu_6500_lp_accl_odr 0x1E
#define mpu_6500_wom_thr 0x1F
#define mpu_6500_fifo_en 0x23
#define mpu_6500_i2c_mst_ctrl 0x24
#define mpu_6500_i2c_slv0_addr 0x25
#define mpu_6500_i2c_slv0_reg 0x26
#define mpu_6500_i2c_slv0_ctrl 0x27 
#define mpu_6500_i2c_slv1_addr 0x28
#define mpu_6500_i2c_slv1_reg 0x29
#define mpu_6500_i2c_slv1_ctrl 0x2A
#define mpu_6500_i2c_slv2_addr 0x2B
#define mpu_6500_i2c_slv2_reg 0x2C
#define mpu_6500_i2c_slv2_ctrl 0x2D
#define mpu_6500_i2c_slv3_addr 0x2E
#define mpu_6500_i2c_slv3_reg 0x2F
#define mpu_6500_i2c_slv3_ctrl 0x30
#define mpu_6500_i2c_slv4_addr 0x31
#define mpu_6500_i2c_slv4_reg 0x32
#define mpu_6500_i2c_slv4_do 0x33
#define mpu_6500_i2c_slv4_ctrl 0x34
#define mpu_6500_i2c_slv4_di 0x35
#define mpu_6500_i2c_mst_status 0x36
#define mpu_6500_int_pin_cfg 0x37
#define mpu_6500_int_enable 0x38
#define mpu_6500_int_status 0x3A
#define mpu_6500_accel_x_h 0x3B
#define mpu_6500_accel_x_l 0x3C
#define mpu_6500_accel_y_h 0x3D
#define mpu_6500_accel_y_l 0x3E
#define mpu_6500_accel_z_h 0x3F
#define mpu_6500_accel_z_l 0x40

#define mpu_6500_temp_h 0x41
#define mpu_6500_temp_l 0x42

#define mpu_6500_gyro_x_h 0x43
#define mpu_6500_gyro_x_l 0x44
#define mpu_6500_gyro_y_h 0x45
#define mpu_6500_gyro_y_l 0x46
#define mpu_6500_gyro_z_h 0x47
#define mpu_6500_gyro_z_l 0x48

#define mpu_6500_who_am_I 0x75
#define mpu_6500_pwr_mgmt_1 0x6B
#define mpu_6500_pwr_mgmt_2 0x6C
#define mpu_6500_user_ctrl 0x6A
#define mpu_6500_sig_path_rst 0x68

#define mpu_6500_xa_offset_h 0x77
#define mpu_6500_xa_offset_l 0x78
#define mpu_6500_ya_offset_h 0x7A
#define mpu_6500_ya_offset_l 0x7B
#define mpu_6500_za_offset_h 0x7D
#define mpu_6500_za_offset_l 0x7E

#define mpu_6500_address 0x68 // 0xD0 //0b1101000 // 0x68

#define init_byte_107 0b00001000
#define init_byte_106 0b00000001
#define init_byte_104 0b00000111

#elif MPU_6050
	#define mpu_6050_smprt_div 0x19
	#define mpu_6050_config 0x1A
	#define mpu_6050_gyro_config 0x1B
	#define mpu_6050_accel_config 0x1C
	#define mpu_6050_int_enable 0x38
	#define mpu_6050_int_status 0x3A

	#define mpu_6050_accel_x_h 0x3B
	#define mpu_6050_accel_x_l 0x3C
	#define mpu_6050_accel_y_h 0x3D
	#define mpu_6050_accel_y_l 0x3E
	#define mpu_6050_accel_z_h 0x3F
	#define mpu_6050_accel_z_l 0x40

	#define mpu_6050_temp_h 0x41
	#define mpu_6050_temp_l 0x42

	#define mpu_6050_gyro_x_h 0x43
	#define mpu_6050_gyro_x_l 0x44
	#define mpu_6050_gyro_y_h 0x45
	#define mpu_6050_gyro_y_l 0x46
	#define mpu_6050_gyro_z_h 0x47
	#define mpu_6050_gyro_z_l 0x48

	#define mpu_6050_who_am_I 0x75
	#define mpu_6050_pwr_mgmnt_1 0x6B
	#define mpu_6050_user_ctrl 0x6A
	#define mpu_6050_sig_path_rst 0x68

	#define mpu_6050_address 0x68 //0xD0 //0b1101000 // 0x68

	#define init_byte_107 0b00001000
	#define init_byte_106 0b00000001
	#define init_byte_104 0b00000111
#else
	#define mpu_6050_smprt_div 25
	#define MPU_6050_CONFIG 26 //0x1A
	#define MPU_9255_GYRO_CONFIG 27 //0x1B
	#define MPU_9255_ACCEL_CONFIG_1 28
	#define MPU_9255_ACCEL_CONFIG_2 29 //0x1D
	#define MPU_9255_INT_BYPASS_CONFIG 55 //0x37
	#define mpu_6050_int_enable 56
	#define mpu_6050_int_status 58
	#define MPU_9255_ACCEL_X_H 59
	#define mpu_6050_accel_x_l 60
	#define mpu_6050_accel_y_h 61
	#define mpu_6050_accel_y_l 62
	#define mpu_6050_accel_z_h 63
	#define mpu_6050_accel_z_l 64
	#define mpu_6050_temp_h 65
	#define mpu_6050_temp_l 66
	#define mpu_6050_gyro_x_h 67
	#define mpu_6050_gyro_x_l 68
	#define mpu_6050_gyro_y_h 69
	#define mpu_6050_gyro_y_l 70
	#define mpu_6050_gyro_z_h 71
	#define mpu_6050_gyro_z_l 72
	#define mpu_6050_who_am_I 117
	#define MPU_9255_PWR_MGMNT_1 107 //0x6B
	#define MPU_9255_USER_CTRL 106 //0x6A
	#define mpu_6050_sig_path_rst 104
	#define MPU_9255_MAG_CONTROL_CONFIG 10 //0x0A
	#define MPU_9255_MAG_ASAX_CONFIG 16 //0x10
	#define MPU_9255_MAG_STATUS_1_CONFIG 2 //0x02

	#define MPU_9255_ADDRESS 0x68 //0b1101000
	#define MPU_9255_AK8963_DEVICE_ID 0x0C//0x48
	#define MPU_9255_DATA_READY_MASK 0x01
	#define MPU_9255_DATA_READY 0x01
	#define MPU_9255_MAGIC_OVERFLOW_MASK 0x8
	#define MPU_9255_MAG_HXL_AD 0x03


	#define INIT_BYTE_107 0b00000001   //0b00000001
	#define INIT_BYTE_28 0b00001000
	#define init_byte_106 0b00000001
	#define init_byte_104 0b00000111
	#define INIT_BYTE_27 0b00001000
	#define INIT_BYTE_29 0b00000101
	#define INIT_BYTE_26 0b00000101
	#define INIT_BYTE_106 0b00000000
	#define INIT_BYTE_55 0b00000010
	#define INIT_BYTE_MAG_10 0b00011111 //fuse mode
	#define RESET_BYTE_MAG_10 0b00000000 //reset
	#define RESET_BYTE_MAG_MODE2_10 0b00010110 //continuous mode 2 at 100Hz and 16 bit output



#endif /* MPU_6050_H_ */


#endif /* INC_MPU6050_H_ */
