import struct

from I2Cdev import I2Cdev
import time

from helper_3dmath import Quaternion



class MPU6050_Base:

    MPU6050_ADDRESS_AD0_LOW =    0x68 # address pin low (GND), default for InvenSense evaluation board
    MPU6050_ADDRESS_AD0_HIGH =   0x69 # address pin high (VCC)
    MPU6050_DEFAULT_ADDRESS =    MPU6050_ADDRESS_AD0_LOW
    MPU6050_RA_XG_OFFS_TC =      0x00 #[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
    MPU6050_RA_YG_OFFS_TC =      0x01 #[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    MPU6050_RA_ZG_OFFS_TC =      0x02 #[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
    MPU6050_RA_X_FINE_GAIN =     0x03 #[7:0] X_FINE_GAIN
    MPU6050_RA_Y_FINE_GAIN =     0x04 #[7:0] Y_FINE_GAIN
    MPU6050_RA_Z_FINE_GAIN =     0x05 #[7:0] Z_FINE_GAIN
    MPU6050_RA_XA_OFFS_H =       0x06 #[15:0] XA_OFFS
    MPU6050_RA_XA_OFFS_L_TC =    0x07
    MPU6050_RA_YA_OFFS_H =       0x08 #[15:0] YA_OFFS
    MPU6050_RA_YA_OFFS_L_TC =    0x09
    MPU6050_RA_ZA_OFFS_H =       0x0A #[15:0] ZA_OFFS
    MPU6050_RA_ZA_OFFS_L_TC =    0x0B
    MPU6050_RA_SELF_TEST_X =     0x0D #[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
    MPU6050_RA_SELF_TEST_Y =     0x0E #[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
    MPU6050_RA_SELF_TEST_Z =     0x0F #[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
    MPU6050_RA_SELF_TEST_A =     0x10 #[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
    MPU6050_RA_XG_OFFS_USRH =    0x13 #[15:0] XG_OFFS_USR
    MPU6050_RA_XG_OFFS_USRL =    0x14
    MPU6050_RA_YG_OFFS_USRH =    0x15 #[15:0] YG_OFFS_USR
    MPU6050_RA_YG_OFFS_USRL =    0x16
    MPU6050_RA_ZG_OFFS_USRH =    0x17 #[15:0] ZG_OFFS_USR
    MPU6050_RA_ZG_OFFS_USRL =    0x18
    MPU6050_RA_SMPLRT_DIV =      0x19
    MPU6050_RA_CONFIG =          0x1A
    MPU6050_RA_GYRO_CONFIG =     0x1B
    MPU6050_RA_ACCEL_CONFIG =    0x1C
    MPU6050_RA_FF_THR =          0x1D
    MPU6050_RA_FF_DUR =          0x1E
    MPU6050_RA_MOT_THR =         0x1F
    MPU6050_RA_MOT_DUR =         0x20
    MPU6050_RA_ZRMOT_THR =       0x21
    MPU6050_RA_ZRMOT_DUR =       0x22
    MPU6050_RA_FIFO_EN =         0x23
    MPU6050_RA_I2C_MST_CTRL =    0x24
    MPU6050_RA_I2C_SLV0_ADDR =   0x25
    MPU6050_RA_I2C_SLV0_REG =    0x26
    MPU6050_RA_I2C_SLV0_CTRL =   0x27
    MPU6050_RA_I2C_SLV1_ADDR =   0x28
    MPU6050_RA_I2C_SLV1_REG =    0x29
    MPU6050_RA_I2C_SLV1_CTRL =   0x2A
    MPU6050_RA_I2C_SLV2_ADDR =   0x2B
    MPU6050_RA_I2C_SLV2_REG =    0x2C
    MPU6050_RA_I2C_SLV2_CTRL =   0x2D
    MPU6050_RA_I2C_SLV3_ADDR =   0x2E
    MPU6050_RA_I2C_SLV3_REG =    0x2F
    MPU6050_RA_I2C_SLV3_CTRL =   0x30
    MPU6050_RA_I2C_SLV4_ADDR =   0x31
    MPU6050_RA_I2C_SLV4_REG =    0x32
    MPU6050_RA_I2C_SLV4_DO =     0x33
    MPU6050_RA_I2C_SLV4_CTRL =   0x34
    MPU6050_RA_I2C_SLV4_DI =     0x35
    MPU6050_RA_I2C_MST_STATUS =  0x36
    MPU6050_RA_INT_PIN_CFG =     0x37
    MPU6050_RA_INT_ENABLE =      0x38
    MPU6050_RA_DMP_INT_STATUS =  0x39
    MPU6050_RA_INT_STATUS =      0x3A
    MPU6050_RA_ACCEL_XOUT_H =    0x3B
    MPU6050_RA_ACCEL_XOUT_L =    0x3C
    MPU6050_RA_ACCEL_YOUT_H =    0x3D
    MPU6050_RA_ACCEL_YOUT_L =    0x3E
    MPU6050_RA_ACCEL_ZOUT_H =    0x3F
    MPU6050_RA_ACCEL_ZOUT_L =    0x40
    MPU6050_RA_TEMP_OUT_H =      0x41
    MPU6050_RA_TEMP_OUT_L =      0x42
    MPU6050_RA_GYRO_XOUT_H =     0x43
    MPU6050_RA_GYRO_XOUT_L =     0x44
    MPU6050_RA_GYRO_YOUT_H =     0x45
    MPU6050_RA_GYRO_YOUT_L =     0x46
    MPU6050_RA_GYRO_ZOUT_H =     0x47
    MPU6050_RA_GYRO_ZOUT_L =     0x48
    MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    MPU6050_RA_MOT_DETECT_STATUS =   0x61
    MPU6050_RA_I2C_SLV0_DO =     0x63
    MPU6050_RA_I2C_SLV1_DO =     0x64
    MPU6050_RA_I2C_SLV2_DO =     0x65
    MPU6050_RA_I2C_SLV3_DO =     0x66
    MPU6050_RA_I2C_MST_DELAY_CTRL =  0x67
    MPU6050_RA_SIGNAL_PATH_RESET =   0x68
    MPU6050_RA_MOT_DETECT_CTRL =     0x69
    MPU6050_RA_USER_CTRL =       0x6A
    MPU6050_RA_PWR_MGMT_1 =      0x6B
    MPU6050_RA_PWR_MGMT_2 =      0x6C
    MPU6050_RA_BANK_SEL =        0x6D
    MPU6050_RA_MEM_START_ADDR =  0x6E
    MPU6050_RA_MEM_R_W =         0x6F
    MPU6050_RA_DMP_CFG_1 =       0x70
    MPU6050_RA_DMP_CFG_2 =       0x71
    MPU6050_RA_FIFO_COUNTH =     0x72
    MPU6050_RA_FIFO_COUNTL =     0x73
    MPU6050_RA_FIFO_R_W =        0x74
    MPU6050_RA_WHO_AM_I =        0x75
    MPU6050_SELF_TEST_XA_1_BIT =    0x07
    MPU6050_SELF_TEST_XA_1_LENGTH = 0x03
    MPU6050_SELF_TEST_XA_2_BIT =    0x05
    MPU6050_SELF_TEST_XA_2_LENGTH = 0x02
    MPU6050_SELF_TEST_YA_1_BIT =    0x07
    MPU6050_SELF_TEST_YA_1_LENGTH = 0x03
    MPU6050_SELF_TEST_YA_2_BIT =    0x03
    MPU6050_SELF_TEST_YA_2_LENGTH = 0x02
    MPU6050_SELF_TEST_ZA_1_BIT =    0x07
    MPU6050_SELF_TEST_ZA_1_LENGTH = 0x03
    MPU6050_SELF_TEST_ZA_2_BIT =    0x01
    MPU6050_SELF_TEST_ZA_2_LENGTH = 0x02
    MPU6050_SELF_TEST_XG_1_BIT =    0x04
    MPU6050_SELF_TEST_XG_1_LENGTH = 0x05
    MPU6050_SELF_TEST_YG_1_BIT =    0x04
    MPU6050_SELF_TEST_YG_1_LENGTH = 0x05
    MPU6050_SELF_TEST_ZG_1_BIT =    0x04
    MPU6050_SELF_TEST_ZG_1_LENGTH = 0x05
    MPU6050_TC_PWR_MODE_BIT =    7
    MPU6050_TC_OFFSET_BIT =      6
    MPU6050_TC_OFFSET_LENGTH =   6
    MPU6050_TC_OTP_BNK_VLD_BIT = 0
    MPU6050_VDDIO_LEVEL_VLOGIC = 0
    MPU6050_VDDIO_LEVEL_VDD =    1
    MPU6050_CFG_EXT_SYNC_SET_BIT =   5
    MPU6050_CFG_EXT_SYNC_SET_LENGTH = 3
    MPU6050_CFG_DLPF_CFG_BIT =   2
    MPU6050_CFG_DLPF_CFG_LENGTH = 3
    MPU6050_EXT_SYNC_DISABLED =      0x0
    MPU6050_EXT_SYNC_TEMP_OUT_L =    0x1
    MPU6050_EXT_SYNC_GYRO_XOUT_L =   0x2
    MPU6050_EXT_SYNC_GYRO_YOUT_L =   0x3
    MPU6050_EXT_SYNC_GYRO_ZOUT_L =   0x4
    MPU6050_EXT_SYNC_ACCEL_XOUT_L =  0x5
    MPU6050_EXT_SYNC_ACCEL_YOUT_L =  0x6
    MPU6050_EXT_SYNC_ACCEL_ZOUT_L =  0x7
    MPU6050_DLPF_BW_256 =        0x00
    MPU6050_DLPF_BW_188 =        0x01
    MPU6050_DLPF_BW_98 =         0x02
    MPU6050_DLPF_BW_42 =         0x03
    MPU6050_DLPF_BW_20 =         0x04
    MPU6050_DLPF_BW_10 =         0x05
    MPU6050_DLPF_BW_5 =          0x06
    MPU6050_GCONFIG_FS_SEL_BIT =     4
    MPU6050_GCONFIG_FS_SEL_LENGTH =  2
    MPU6050_GYRO_FS_250 =        0x00
    MPU6050_GYRO_FS_500 =        0x01
    MPU6050_GYRO_FS_1000 =       0x02
    MPU6050_GYRO_FS_2000 =       0x03
    MPU6050_ACONFIG_XA_ST_BIT =          7
    MPU6050_ACONFIG_YA_ST_BIT =          6
    MPU6050_ACONFIG_ZA_ST_BIT =          5
    MPU6050_ACONFIG_AFS_SEL_BIT =        4
    MPU6050_ACONFIG_AFS_SEL_LENGTH =     2
    MPU6050_ACONFIG_ACCEL_HPF_BIT =      2
    MPU6050_ACONFIG_ACCEL_HPF_LENGTH =   3
    MPU6050_ACCEL_FS_2 =         0x00
    MPU6050_ACCEL_FS_4 =         0x01
    MPU6050_ACCEL_FS_8 =         0x02
    MPU6050_ACCEL_FS_16 =        0x03
    MPU6050_DHPF_RESET =         0x00
    MPU6050_DHPF_5 =             0x01
    MPU6050_DHPF_2P5 =           0x02
    MPU6050_DHPF_1P25 =          0x03
    MPU6050_DHPF_0P63 =          0x04
    MPU6050_DHPF_HOLD =          0x07
    MPU6050_TEMP_FIFO_EN_BIT =   7
    MPU6050_XG_FIFO_EN_BIT =     6
    MPU6050_YG_FIFO_EN_BIT =     5
    MPU6050_ZG_FIFO_EN_BIT =     4
    MPU6050_ACCEL_FIFO_EN_BIT =  3
    MPU6050_SLV2_FIFO_EN_BIT =   2
    MPU6050_SLV1_FIFO_EN_BIT =   1
    MPU6050_SLV0_FIFO_EN_BIT =   0
    MPU6050_MULT_MST_EN_BIT =    7
    MPU6050_WAIT_FOR_ES_BIT =    6
    MPU6050_SLV_3_FIFO_EN_BIT =  5
    MPU6050_I2C_MST_P_NSR_BIT =  4
    MPU6050_I2C_MST_CLK_BIT =    3
    MPU6050_I2C_MST_CLK_LENGTH = 4
    MPU6050_CLOCK_DIV_348 =      0x0
    MPU6050_CLOCK_DIV_333 =      0x1
    MPU6050_CLOCK_DIV_320 =      0x2
    MPU6050_CLOCK_DIV_308 =      0x3
    MPU6050_CLOCK_DIV_296 =      0x4
    MPU6050_CLOCK_DIV_286 =      0x5
    MPU6050_CLOCK_DIV_276 =      0x6
    MPU6050_CLOCK_DIV_267 =      0x7
    MPU6050_CLOCK_DIV_258 =      0x8
    MPU6050_CLOCK_DIV_500 =      0x9
    MPU6050_CLOCK_DIV_471 =      0xA
    MPU6050_CLOCK_DIV_444 =      0xB
    MPU6050_CLOCK_DIV_421 =      0xC
    MPU6050_CLOCK_DIV_400 =      0xD
    MPU6050_CLOCK_DIV_381 =      0xE
    MPU6050_CLOCK_DIV_364 =      0xF
    MPU6050_I2C_SLV_RW_BIT =      7
    MPU6050_I2C_SLV_ADDR_BIT =    6
    MPU6050_I2C_SLV_ADDR_LENGTH = 7
    MPU6050_I2C_SLV_EN_BIT =      7
    MPU6050_I2C_SLV_BYTE_SW_BIT = 6
    MPU6050_I2C_SLV_REG_DIS_BIT = 5
    MPU6050_I2C_SLV_GRP_BIT =     4
    MPU6050_I2C_SLV_LEN_BIT =     3
    MPU6050_I2C_SLV_LEN_LENGTH =  4
    MPU6050_I2C_SLV4_RW_BIT =         7
    MPU6050_I2C_SLV4_ADDR_BIT =       6
    MPU6050_I2C_SLV4_ADDR_LENGTH =    7
    MPU6050_I2C_SLV4_EN_BIT =         7
    MPU6050_I2C_SLV4_INT_EN_BIT =     6
    MPU6050_I2C_SLV4_REG_DIS_BIT =    5
    MPU6050_I2C_SLV4_MST_DLY_BIT =    4
    MPU6050_I2C_SLV4_MST_DLY_LENGTH = 5
    MPU6050_MST_PASS_THROUGH_BIT =    7
    MPU6050_MST_I2C_SLV4_DONE_BIT =   6
    MPU6050_MST_I2C_LOST_ARB_BIT =    5
    MPU6050_MST_I2C_SLV4_NACK_BIT =   4
    MPU6050_MST_I2C_SLV3_NACK_BIT =   3
    MPU6050_MST_I2C_SLV2_NACK_BIT =   2
    MPU6050_MST_I2C_SLV1_NACK_BIT =   1
    MPU6050_MST_I2C_SLV0_NACK_BIT =   0
    MPU6050_INTCFG_INT_LEVEL_BIT =       7
    MPU6050_INTCFG_INT_OPEN_BIT =        6
    MPU6050_INTCFG_LATCH_INT_EN_BIT =    5
    MPU6050_INTCFG_INT_RD_CLEAR_BIT =    4
    MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT = 3
    MPU6050_INTCFG_FSYNC_INT_EN_BIT =    2
    MPU6050_INTCFG_I2C_BYPASS_EN_BIT =   1
    MPU6050_INTCFG_CLKOUT_EN_BIT =       0
    MPU6050_INTMODE_ACTIVEHIGH =  0x00
    MPU6050_INTMODE_ACTIVELOW =   0x01
    MPU6050_INTDRV_PUSHPULL =     0x00
    MPU6050_INTDRV_OPENDRAIN =    0x01
    MPU6050_INTLATCH_50USPULSE =  0x00
    MPU6050_INTLATCH_WAITCLEAR =  0x01
    MPU6050_INTCLEAR_STATUSREAD = 0x00
    MPU6050_INTCLEAR_ANYREAD =    0x01
    MPU6050_INTERRUPT_FF_BIT =            7
    MPU6050_INTERRUPT_MOT_BIT =           6
    MPU6050_INTERRUPT_ZMOT_BIT =          5
    MPU6050_INTERRUPT_FIFO_OFLOW_BIT =    4
    MPU6050_INTERRUPT_I2C_MST_INT_BIT =   3
    MPU6050_INTERRUPT_PLL_RDY_INT_BIT =   2
    MPU6050_INTERRUPT_DMP_INT_BIT =       1
    MPU6050_INTERRUPT_DATA_RDY_BIT =      0
    MPU6050_DMPINT_5_BIT =           5
    MPU6050_DMPINT_4_BIT =           4
    MPU6050_DMPINT_3_BIT =           3
    MPU6050_DMPINT_2_BIT =           2
    MPU6050_DMPINT_1_BIT =           1
    MPU6050_DMPINT_0_BIT =           0
    MPU6050_MOTION_MOT_XNEG_BIT =    7
    MPU6050_MOTION_MOT_XPOS_BIT =    6
    MPU6050_MOTION_MOT_YNEG_BIT =    5
    MPU6050_MOTION_MOT_YPOS_BIT =    4
    MPU6050_MOTION_MOT_ZNEG_BIT =    3
    MPU6050_MOTION_MOT_ZPOS_BIT =    2
    MPU6050_MOTION_MOT_ZRMOT_BIT =   0
    MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT =  7
    MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT =  4
    MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT =  3
    MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT =  2
    MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT =  1
    MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT =  0
    MPU6050_PATHRESET_GYRO_RESET_BIT =   2
    MPU6050_PATHRESET_ACCEL_RESET_BIT =  1
    MPU6050_PATHRESET_TEMP_RESET_BIT =   0
    MPU6050_DETECT_ACCEL_ON_DELAY_BIT =      5
    MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH =   2
    MPU6050_DETECT_FF_COUNT_BIT =            3
    MPU6050_DETECT_FF_COUNT_LENGTH =         2
    MPU6050_DETECT_MOT_COUNT_BIT =           1
    MPU6050_DETECT_MOT_COUNT_LENGTH =        2
    MPU6050_DETECT_DECREMENT_RESET = 0x0
    MPU6050_DETECT_DECREMENT_1 =     0x1
    MPU6050_DETECT_DECREMENT_2 =     0x2
    MPU6050_DETECT_DECREMENT_4 =     0x3
    MPU6050_USERCTRL_DMP_EN_BIT =            7
    MPU6050_USERCTRL_FIFO_EN_BIT =           6
    MPU6050_USERCTRL_I2C_MST_EN_BIT =        5
    MPU6050_USERCTRL_I2C_IF_DIS_BIT =        4
    MPU6050_USERCTRL_DMP_RESET_BIT =         3
    MPU6050_USERCTRL_FIFO_RESET_BIT =        2
    MPU6050_USERCTRL_I2C_MST_RESET_BIT =     1
    MPU6050_USERCTRL_SIG_COND_RESET_BIT =    0
    MPU6050_PWR1_DEVICE_RESET_BIT =  7
    MPU6050_PWR1_SLEEP_BIT =         6
    MPU6050_PWR1_CYCLE_BIT =         5
    MPU6050_PWR1_TEMP_DIS_BIT =      3
    MPU6050_PWR1_CLKSEL_BIT =        2
    MPU6050_PWR1_CLKSEL_LENGTH =     3
    MPU6050_CLOCK_INTERNAL =         0x00
    MPU6050_CLOCK_PLL_XGYRO =        0x01
    MPU6050_CLOCK_PLL_YGYRO =        0x02
    MPU6050_CLOCK_PLL_ZGYRO =        0x03
    MPU6050_CLOCK_PLL_EXT32K =       0x04
    MPU6050_CLOCK_PLL_EXT19M =       0x05
    MPU6050_CLOCK_KEEP_RESET =       0x07
    MPU6050_PWR2_LP_WAKE_CTRL_BIT =      7
    MPU6050_PWR2_LP_WAKE_CTRL_LENGTH =   2
    MPU6050_PWR2_STBY_XA_BIT =           5
    MPU6050_PWR2_STBY_YA_BIT =           4
    MPU6050_PWR2_STBY_ZA_BIT =           3
    MPU6050_PWR2_STBY_XG_BIT =           2
    MPU6050_PWR2_STBY_YG_BIT =           1
    MPU6050_PWR2_STBY_ZG_BIT =           0
    MPU6050_WAKE_FREQ_1P25 =     0x0
    MPU6050_WAKE_FREQ_2P5 =      0x1
    MPU6050_WAKE_FREQ_5 =        0x2
    MPU6050_WAKE_FREQ_10 =       0x3
    MPU6050_BANKSEL_PRFTCH_EN_BIT =      6
    MPU6050_BANKSEL_CFG_USER_BANK_BIT =  5
    MPU6050_BANKSEL_MEM_SEL_BIT =        4
    MPU6050_BANKSEL_MEM_SEL_LENGTH =     5
    MPU6050_WHO_AM_I_BIT =       6
    MPU6050_WHO_AM_I_LENGTH =    6
    MPU6050_DMP_MEMORY_BANKS  =      8
    MPU6050_DMP_MEMORY_BANK_SIZE =   256
    MPU6050_DMP_MEMORY_CHUNK_SIZE =  16
    MPU6050_FIFO_DEFAULT_TIMEOUT = 11000

    def __init__(self, addr = MPU6050_DEFAULT_ADDRESS):
        self.wireObj = I2Cdev(bus = 0, i2c_addr = addr, debug = False)
        self.start_time = time.perf_counter()  # Record the start time

    def micros(self):
        # Calculate the elapsed time in microseconds
        elapsed_time = time.perf_counter() - self.start_time
        return int(elapsed_time * 1_000_000)  # Convert to microseconds

    def delay(self, milliseconds):
        time.sleep(milliseconds / 1000.0)

    def reset(self):
        self.wireObj.write_bit(self.MPU6050_RA_PWR_MGMT_1, self.MPU6050_PWR1_DEVICE_RESET_BIT, True)

    def setSleepEnabled(self, enabled):
        self.wireObj.write_bit(self.MPU6050_RA_PWR_MGMT_1, self.MPU6050_PWR1_SLEEP_BIT, enabled)

    def setMemoryBank(self, bank, prefetch_enabled, user_bank):
        bank &= 0x1F
        if user_bank:
            bank |= 0x20
        if prefetch_enabled:
            bank |= 0x40
            
        # Assuming `I2Cdev.write_byte` is implemented
        self.wireObj.write_byte(self.MPU6050_RA_BANK_SEL, bank)

    def setMemoryStartAddress(self, address):
        self.wireObj.write_byte(self.MPU6050_RA_MEM_START_ADDR, address)

    def readMemoryByte(self):
        return self.wireObj.read_byte(self.MPU6050_RA_MEM_R_W)

    def getOTPBankValid(self):
        return self.wireObj.read_bit(self.MPU6050_RA_XG_OFFS_TC, self.MPU6050_TC_OTP_BNK_VLD_BIT)

    def setSlaveAddress(self, num, address):
        if num > 3:
            return
        self.wireObj.write_byte(self.MPU6050_RA_I2C_SLV0_ADDR + num * 3, address)

    def setI2CMasterModeEnabled(self, enabled):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled)

    def resetI2CMaster(self):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_I2C_MST_RESET_BIT, True)

    def setClockSource(self, source):
        self.wireObj.write_bits(self.MPU6050_RA_PWR_MGMT_1, self.MPU6050_PWR1_CLKSEL_BIT, self.MPU6050_PWR1_CLKSEL_LENGTH, source)

    def setIntEnabled(self, enabled):
        self.wireObj.write_byte(self.MPU6050_RA_INT_ENABLE, enabled)

    def setRate(self, rate):
        self.wireObj.write_byte(self.MPU6050_RA_SMPLRT_DIV, rate)

    def setExternalFrameSync(self, sync):
        self.wireObj.write_bits(self.MPU6050_RA_CONFIG, self.MPU6050_CFG_EXT_SYNC_SET_BIT, self.MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync)
    
    def setDLPFMode(self, mode):
        self.wireObj.write_bits(self.MPU6050_RA_CONFIG, self.MPU6050_CFG_DLPF_CFG_BIT, self.MPU6050_CFG_DLPF_CFG_LENGTH, mode)
    
    def setFullScaleGyroRange(self, range):
        self.wireObj.write_bits(self.MPU6050_RA_GYRO_CONFIG, self.MPU6050_GCONFIG_FS_SEL_BIT, self.MPU6050_GCONFIG_FS_SEL_LENGTH, range)
    
    def getFullScaleGyroRange(self):
        return self.wireObj.read_bits(self.MPU6050_RA_GYRO_CONFIG, self.MPU6050_GCONFIG_FS_SEL_BIT, self.MPU6050_GCONFIG_FS_SEL_LENGTH)

    def setFullScaleAccelRange(self, range):
        self.wireObj.write_bits(self.MPU6050_RA_ACCEL_CONFIG, self.MPU6050_ACONFIG_AFS_SEL_BIT, self.MPU6050_ACONFIG_AFS_SEL_LENGTH, range)
    
    def getFullScaleAccelRange(self):
        return self.wireObj.read_bits(self.MPU6050_RA_ACCEL_CONFIG, self.MPU6050_ACONFIG_AFS_SEL_BIT, self.MPU6050_ACONFIG_AFS_SEL_LENGTH)
    
    def writeProgMemoryBlock(self, data, data_size, bank = 0, address = 0, verify = True):
        return self.writeMemoryBlock(data, data_size, bank, address, verify, True)
    
    def writeMemoryBlock(self, data, data_size, bank = 0, address = 0, verify = True, use_prog_mem = False):
        self.setMemoryBank(bank, False, False)
        self.setMemoryStartAddress(address)
        i = 0

        # Print out the parameters for debugging purposes
        #print("write_memory_block called with parameters:")
        #print(f"Data: {data}")
        #print(f"Data Size: {data_size}")
        #print(f"Bank: {bank}")
        #print(f"Address: {address}")
        #print(f"Verify: {verify}")
        #print(f"Use Program Memory: {use_prog_mem}")

        verify_buffer = bytearray(self.MPU6050_DMP_MEMORY_CHUNK_SIZE) if verify else None
        prog_buffer = bytearray(self.MPU6050_DMP_MEMORY_CHUNK_SIZE) if use_prog_mem else None

        while i < data_size:
            chunk_size =  self.MPU6050_DMP_MEMORY_CHUNK_SIZE

            if i + chunk_size > data_size:
                chunk_size = data_size - i

            if chunk_size > 256 - address:
                chunk_size = 256 - address
            
            if use_prog_mem:
                for j in range(chunk_size):
                    prog_buffer[j] = data[i + j]  # Assume data comes already in byte format
            else:
                prog_buffer = data[i:i+chunk_size]

            time.sleep(0.01)
            self.wireObj.write_bytes(self.MPU6050_RA_MEM_R_W, prog_buffer[0:chunk_size])

            if verify and verify_buffer is not None:
                self.setMemoryBank(bank, False, False)
                self.setMemoryStartAddress(address)
                verify_buffer = self.wireObj.read_bytes(self.MPU6050_RA_MEM_R_W, chunk_size)

                if bytes(prog_buffer[0:chunk_size]) != bytes(verify_buffer[0:chunk_size]):
                    print(f"Block write verification error, bank {bank}, address {address}!")
                    return False

            i += chunk_size
            address += chunk_size

            if i < data_size:
                if address == 256:
                    bank += 1
                    address = 0
                self.setMemoryBank(bank, False, False)
                self.setMemoryStartAddress(address)

        return True


    def setDMPConfig1(self, config):
        self.wireObj.write_byte(self.MPU6050_RA_DMP_CFG_1, config)

    def setDMPConfig2(self, config):
        self.wireObj.write_byte(self.MPU6050_RA_DMP_CFG_2, config)

    def setOTPBankValid(self, enabled):
        self.wireObj.write_bit(self.MPU6050_RA_XG_OFFS_TC, self.MPU6050_TC_OTP_BNK_VLD_BIT, enabled)

    def setMotionDetectionThreshold(self, threshold):
        self.wireObj.write_byte(self.MPU6050_RA_MOT_THR, threshold)

    def setZeroMotionDetectionThreshold(self, threshold):
        self.wireObj.write_byte(self.MPU6050_RA_ZRMOT_THR, threshold)

    def setMotionDetectionDuration(self, duration):
        self.wireObj.write_byte(self.MPU6050_RA_MOT_DUR, duration)
    
    def setZeroMotionDetectionDuration(self, duration):
        self.wireObj.write_byte(self.MPU6050_RA_ZRMOT_DUR, duration)

    def setFIFOEnabled(self, enabled):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_FIFO_EN_BIT, enabled)

    def resetDMP(self):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_DMP_RESET_BIT, True)

    def setDMPEnabled(self, enabled):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_DMP_EN_BIT, enabled)
    
    def resetFIFO(self):
        self.wireObj.write_bit(self.MPU6050_RA_USER_CTRL, self.MPU6050_USERCTRL_FIFO_RESET_BIT, True)

    def getIntStatus(self):
        return self.wireObj.read_byte(self.MPU6050_RA_INT_STATUS)

    def initialize(self):
        self.setClockSource(self.MPU6050_CLOCK_PLL_XGYRO)
        self.setFullScaleGyroRange(self.MPU6050_GYRO_FS_250)
        self.setFullScaleAccelRange(self.MPU6050_ACCEL_FS_2)
        self.setSleepEnabled(False)  # Thanks to Jack Elston for this tip!

    def testConnection(self):
        #idk it returns false, my bit masking is bad probably
        return self.getDeviceId() == 0x34
        #return self.getDeviceId() == 0x68

    def getDeviceId(self): 
        return self.wireObj.read_bits(self.MPU6050_RA_WHO_AM_I,  self.MPU6050_WHO_AM_I_BIT,  self.MPU6050_WHO_AM_I_LENGTH)
        #return self.wireObj.read_byte(self.MPU6050_RA_WHO_AM_I)

    def GetCurrentFIFOPacket(self,  length):  # overflow proof
        """Get the current FIFO packet from MPU6050."""
        fifoC = None
        BreakTimer = self.micros()  # Assuming self.micros() is defined
        packetReceived = False
        
        while True:
            fifoC = self.getFIFOCount()  # Assuming this method is defined
            
            if fifoC > length:
                if fifoC > 200:  # If FIFO count > 200 bytes
                    self.resetFIFO()  # Fixes any overflow corruption
                    fifoC = 0
                    
                    while (fifoC == 0) and ((self.micros() - BreakTimer) <= self.getFIFOTimeout()):  # Get Next New Packet
                        fifoC = self.getFIFOCount()
                else:  # Less than 200 bytes of data in the FIFO Buffer
                    Trash = [0] * 32  # Assuming I2CDEVLIB_WIRE_BUFFER_LENGTH is defined
                    
                    while fifoC > length:  # Test each time just in case the MPU is writing to the FIFO Buffer
                        fifoC -= length  # Save the last packet
                        
                        while fifoC > 0:  # fifo count will reach zero so this is safe
                            RemoveBytes = min(fifoC, 32)  # Efficiently clear the buffer
                            Trash = self.getFIFOBytes(RemoveBytes)  # Assuming this method is defined
                            fifoC -= RemoveBytes
                        
            if fifoC == 0:
                return 0  # Called too early no data or we timed out after FIFO Reset
            
            # We have 1 packet
            packetReceived = (fifoC == length)
            if not packetReceived and (self.micros() - BreakTimer) > self.getFIFOTimeout():
                return 0
            
            if packetReceived:
                break
        
        return self.getFIFOBytes(length)  # Get 1 packet

    def getFIFOCount(self):
        buffer = self.wireObj.read_bytes(self.MPU6050_RA_FIFO_COUNTH, 2)
        return (buffer[0] << 8) | buffer[1]

    def getFIFOBytes(self, length):
        # Check if the requested length is valid
        if length <= 0:
            return [0]  # Return a list with a single 0

        # Ensure that we don't exceed the maximum block size
        max_block_size = 32
        data = []
        
        while length > 0:
            # Determine how many bytes to read in this iteration
            bytes_to_read = min(length, max_block_size)
            
            # Read the bytes and extend the data list
            data.extend(self.wireObj.read_bytes(self.MPU6050_RA_FIFO_R_W, bytes_to_read))
            
            # Decrement the length by the number of bytes we just read
            length -= bytes_to_read
        
        return data

    def getFIFOTimeout(self):
        return 1000

    def setXGyroOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_XG_OFFS_USRH, struct.pack('>h', offset))

    def setYGyroOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_YG_OFFS_USRH, struct.pack('>h', offset))

    def setZGyroOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_ZG_OFFS_USRH, struct.pack('>h', offset))
    
    def setXAccelOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_XA_OFFS_H, struct.pack('>h', offset))

    def setYAccelOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_YA_OFFS_H, struct.pack('>h', offset))

    def setZAccelOffset(self, offset):
        self.wireObj.write_bytes(self.MPU6050_RA_ZA_OFFS_H, struct.pack('>h', offset))

    def getXGyroOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_XG_OFFS_USRH, 2)
        return struct.unpack('>h', data)[0]

    def getYGyroOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_YG_OFFS_USRH, 2)
        return struct.unpack('>h', data)[0]

    def getZGyroOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_ZG_OFFS_USRH, 2)
        return struct.unpack('>h', data)[0]

    def getXAccelOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_XA_OFFS_H, 2)
        return struct.unpack('>h', data)[0]

    def getYAccelOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_YA_OFFS_H, 2)
        return struct.unpack('>h', data)[0]

    def getZAccelOffset(self):
        data = self.wireObj.read_bytes(self.MPU6050_RA_ZA_OFFS_H, 2)
        return struct.unpack('>h', data)[0]

    def CalibrateGyro(self, loops):
        kP = 0.3
        kI = 90.0
        x = (100 - self.map_value(loops, 1, 5, 20, 0)) * 0.01
        kP *= x
        kI *= x
        
        self.PID(0x43, kP, kI, loops)

    def CalibrateAccel(self, loops):
        kP = 0.3
        kI = 20.0
        x = (100 - self.map_value(loops, 1, 5, 20, 0)) * 0.01
        kP *= x
        kI *= x
        
        self.PID(0x3B, kP, kI, loops)

    def PID(self, read_address, kP, kI, loops):
        save_address = 0x06 if self.getDeviceId() < 0x38 else 0x77 if read_address == 0x3B else 0x13

        bit_zero = [0] * 3
        shift = 3 if save_address == 0x77 else 2
        error, p_term, i_term = [0] * 3, [0] * 3, [0] * 3
        gravity = 8192  # prevent uninitialized warning
        if read_address == 0x3B:
            gravity = 16384 >> self.getFullScaleGyroRange()

        print('>', end='')  # Serial write equivalent
        for i in range(3):
            data = struct.unpack('>h', bytes(self.wireObj.read_bytes(save_address + (i * shift), 2)))[0]
            reading = data
            if save_address != 0x13:
                bit_zero[i] = data & 1  # Capture Bit Zero
                i_term[i] = reading * 8
            else:
                i_term[i] = reading * 4
        
        

        for L in range(loops):
            e_sample = 0
            for c in range(100):  # 100 PI Calculations
                e_sum = 0
                for i in range(3):
                    data = struct.unpack('>h', bytes(self.wireObj.read_bytes(read_address + (i * 2), 2)))[0]
                    reading = data
                    #print(data, end=" ")
                    if (read_address == 0x3B) and (i == 2):
                        reading -= gravity  # Remove Gravity
                    error = -reading
                    e_sum += abs(reading)
                    p_term = kP * error
                    i_term[i] += (error * 0.001) * kI  # Integral term
                    if save_address != 0x13:
                        data = round((p_term + i_term[i]) / 8)  # Compute PID Output
                        data = (data & 0xFFFE) | bit_zero[i]  # Insert Bit0
                    else:
                        data = round((p_term + i_term[i]) / 4)
                    self.wireObj.write_word(save_address + (i * shift), data)
                #print()
                if (c == 99) and (e_sum > 1000):  # Error is still too great
                    c = 0
                    print('*', end='')
                if (e_sum * (0.05 if read_address == 0x3B else 1)) < 5:
                    e_sample += 1  # Successfully found offsets
                if (e_sum < 100) and (c > 10) and (e_sample >= 10):
                    break  # Advance to next Loop
                time.sleep(0.001)
            print('.', end='')
            kP *= 0.75
            kI *= 0.75
            for i in range(3):
                if save_address != 0x13:
                    data = round((i_term[i]) / 8)  # Compute PID Output
                    data = (data & 0xFFFE) | bit_zero[i]  # Insert Bit0
                else:
                    data = round((i_term[i]) / 4)
                self.wireObj.write_word(save_address + (i * shift), data)
        print()
        self.resetFIFO()
        self.resetDMP()

    def map_value(self, value, from_low, from_high, to_low, to_high):
        """ Map a value from one range to another """
        return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


    def getMotion6(self):
        data = self.wireObj.read_bytes(59, 14)

        gravity = 16384 >> self.getFullScaleAccelRange()

        # Extracting the 3 16-bit integers
        ax = struct.unpack('>h', bytes(data[:2]))[0]/gravity
        ay = struct.unpack('>h', bytes(data[2:4]))[0]/gravity
        az = struct.unpack('>h', bytes(data[4:6]))[0]/gravity

        # Extracting the 3 16-bit integers
        gx = struct.unpack('>h', bytes(data[8:10]))[0]/131
        gy = struct.unpack('>h', bytes(data[10:12]))[0]/131
        gz = struct.unpack('>h', bytes(data[12:14]))[0]/131

        return [ax, ay, az, gx, gy, gz]

    def PrintActiveOffsets(self):
        offsets = self.GetActiveOffsets()
        for offset in offsets:
            print(offset)

    def GetActiveOffsets(self):
        return[ self.getXAccelOffset(), self.getYAccelOffset(), self.getZAccelOffset(), self.getXGyroOffset(), self.getYGyroOffset(), self.getZGyroOffset()]


