/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_sc8527.h
** Description: regcp ic
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#ifndef __SOUTHCHIP_8527_H__
#define __SOUTHCHIP_8527_H__

#define UFCS_HARDRESET_RETRY_CNTS	3
#define UFCS_HANDSHAKE_RETRY_CNTS	1

/************************Timer***********************************/
#define UFCS_HANDSHAKE_TIMEOUT		(300)
#define UFCS_PING_TIMEOUT		(300)
#define UFCS_ACK_TIMEOUT		(50)
#define UFCS_MSG_TIMEOUT		(300)
#define UFCS_POWER_RDY_TIMEOUT		(550)
/********************** I2C Slave Addr **************************/
#define SC8527_I2C_ADDR		(0x6A)
#define SC8527_MAX_REG			(0xCA)
#define SC8527_FLAG_NUM		(3)

/********************** SC2201 I2C reg map **********************/
#define SC8527_ENABLE_REG_NUM		(5)

#define SC8527_ADDR_DEVICE_ID		(0x00)
#define SC8527_ADDR_DEVICE_ID1		(0x01)

#define SC8527_ADDR_DPDM_CTRL		(0x20)
#define SC8527_CMD_DPDM_EN		(0x01)

#define SC8527_ADDR_OTG_EN		(0x3B)
#define SC8527_OTG_EN_MASK		0x04

#define STEP_50MV		50
#define STEP_200MV		200
#define STEP_1000MA		1000
#define V1X_CV_OFFSET		3000
#define V1X_l_TH_OFFSET		300
#define V1X_TH_OFFSET		100
#define K_CHANGE_TH_OFFSET	5400
#define K_CHANGE_DEFAULT_TH	5600
#define I2C_ERR_MAX			10
#define IIND_LIM_TH_OFFSET	4000
#define IIND_LIM_DEFAULT_TH	14000

#define SC8527_SNS_RATIO_500_THR	500
#define SC8527_SNS_RATIO_550_THR	550
#define SC8527_SNS_RATIO_575_THR	575
#define SC8527_SNS_RATIO_600_THR	600
#define SC8527_SNS_RATIO_625_THR	625
#define SC8527_SNS_RATIO_550    	(0)
#define SC8527_SNS_RATIO_575	        (0X1)
#define SC8527_SNS_RATIO_600	        (0X2)
#define SC8527_SNS_RATIO_625	        (0X3)
#define SC8527_BCL_CP_MODE		(0X0)
#define SC8527_BCL_AUTO_SNS_RATIO_MODE	(0X1)
#define SC8527_BCL_MTK_PRE_UV_MODE	(0X2)
#define SC8527_BCL_FORCE_SNS_RATIO_MODE	(0X3)
#define SC8527_SNS_CLAMP_ENABLE		(0X1)
#define SC8527_SNS_CLAMP_DISABLE	(0X0)
#define SC8527_SHIPMODE_EN		(0x7)
#define SC8527_OFF_MODE		(0x0)
#define SC8527_CP_MODE		(0x10)
#define SC8527_CV_MODE		(0x20)
#define SC8527_BYP_MODE		(0x30)


#define V2X_OVUV_REG			0x00
#define V2X_OVP_OFFSET			9500
#define V2X_OVP_GAIN			500
#define V2X_OVP_START_BIT		5
#define V2X_OVP_LEN_BIT			2
#define V2X_OVP_DISABLE			BIT(7)
#define V2X_OVP_ENABLE			~V2X_OVP_DISABLE

#define V1X_OVUV_REG			0x01
#define V1X_OVP_OFFSET			4150
#define V1X_OVP_GAIN			25
#define V1X_OVP_START_BIT		2
#define V1X_OVP_LEN_BIT			5
#define V1X_OVP_DISABLE			BIT(7)
#define V1X_OVP_ENABLE			~V1X_OVP_DISABLE
#define V1X_SCP_DISABLE			BIT(1)
#define V1X_SCP_ENABLE			~V1X_SCP_DISABLE

#define VAC_OVUV_REG			0x02
#define VAC_OVP_OFFSET			6500
#define VAC_OVP_GAIN			500
#define VAC_OVP_START_BIT		3
#define VAC_OVP_LEN_BIT			4
#define VAC_OVP_DISABLE			BIT(7)
#define VAC_OVP_ENABLE			~VAC_OVP_DISABLE
#define ACDRV_ENABLE			BIT(0)
#define ACDRV_DISABLE			~ACDRV_ENABLE

#define RVS_OCP_REG				0x03
#define RVS_OCP_OFFSET			300
#define RVS_OCP_GAIN			25
#define RVS_OCP_START_BIT		4
#define RVS_OCP_LEN_BIT			4

#define FWD_OCP_REG				0x03
#define FWD_OCP_OFFSET			400
#define FWD_OCP_GAIN			25
#define FWD_OCP_START_BIT		0
#define FWD_OCP_LEN_BIT			4

#define TIMEOUT_REG		0x04
#define WD_TIMER_START_BIT	4
#define LNC_SS_TIMER_START_BIT	1

#define STATUS_REG		0x09
#define FLAG_REG		0x0c
#define MASK_REG		0x0f

#define FUNCTION_DISABLE_REG	0x13

#define DEVICE_ID_REG		0x14

#define OTG_REG			0x15
#define OTG_ENABLE		0x3
#define OTG_DISABLE		0

#define ENABLE_MOS		0x7B
#define DISENABLE_MOS		0x78
#define I2C_ERR_NUM		10
#define MAIN_I2C_ERROR		(1 << 0)

/* Register 00h */
#define SC8527_REG_00			0x00

/* Register 01h */
#define SC8527_REG_01			0x01

/* Register 02h */
#define SC8527_REG_02			0x2

#define SC8527_CHG_EN_MASK		BIT(0)
#define SC8527_CHG_EN_SHIFT		0
#define SC8527_CHG_ENABLE		1
#define SC8527_CHG_DISABLE		0

#define SC8527_VAC_INRANGE_EN_MASK	BIT(1)
#define SC8527_VAC_INRANGE_EN_SHIFT	1
#define SC8527_VAC_INRANGE_ENABLE	0
#define SC8527_VAC_INRANGE_DISABLE	1

/* Register 03h */
#define SC8527_REG_03		0x03

/* Register 04h */
#define SC8527_REG_04		0x04

/* Register 06h */
#define SC8527_REG_06		0x06

#define SC8527_REG_RESET_MASK		BIT(3)
#define SC8527_REG_RESET_SHIFT		3
#define SC8527_NO_REG_RESET		0
#define SC8527_RESET_REG		1

/* Register 07h */
#define SC8527_REG_07		0x07

/* Register 08h */
#define SC8527_REG_08		0x08

/* Register 09h~0Eh int flag&status */
/* Register 09h: STATUS1 Register */
#define SC8527_REG_09		0x09
#define SC8527_STATUS1_STANDARD_VAL	0x00

/* Register 0Ah: STATUS2 Register */
#define SC8527_REG_0A		0x0A
#define SC8527_STATUS2_SC_EN_STAT		BIT(0)
#define SC8527_STATUS2_REG_EN_STAT		BIT(1)
#define SC8527_STATUS2_CP_SWITCHING_STAT	BIT(2)
#define SC8527_STATUS2_LOW_3BITS_MASK		0x07
#define SC8527_STATUS2_LOW_3BITS_STANDARD_VAL	0x07

/* Register 0Bh: STATUS3 Register */
#define SC8527_REG_0B		0x0B
#define VBUS_INRANGE_STATUS_MASK       (BIT(7) | BIT(6))
#define VBUS_INRANGE_STATUS_SHIFT      6

/* Register 0Ch: FLAG1 Register */
#define SC8527_REG_0C		0x0C
#define SC8527_FLAG1_STANDARD_VAL	0x00

/* Register 0Dh: FLAG2 Register */
#define SC8527_REG_0D		0x0D
#define SC8527_FLAG2_V2X_UVLO_FLG		BIT(7)
#define SC8527_FLAG2_V1X_SCP_FLG		BIT(6)
#define SC8527_FLAG2_CONV_OCP_FLG		BIT(2)
#define SC8527_FLAG2_CHECK_MASK			0xC4	/* bit7, bit6, bit2 */
#define SC8527_FLAG2_CHECK_STANDARD_VAL		0x00

/* Register 0Eh: FLAG3 Register */
#define SC8527_REG_0E		0x0E
#define SC8527_FLAG3_STANDARD_VAL	0x00

/* Register 0Fh */
#define SC8527_REG_0F		0x0F

/* Register 10h */
#define SC8527_REG_10		0x10

/* Register 12h */
#define SC8527_REG_12		0x12

/* Register 14h */
#define SC8527_REG_14		0x14

/* Register 15h */
#define SC8527_REG_15		0x15
#define SC8527_CHG_MODE_MASK	(BIT(1) | BIT(0))
#define SC8527_CHG_FIX_MODE	0
#define SC8527_CHG_AUTO_MODE	3

/* Register 16h */
#define SC8527_REG_16		0x16
#define SC8527_CV_VOLT_MASK	(0XF8)
#define SC8527_CV_VOLT_SHIFT	(0X3)
#define SC8527_V1X_TH_CV2CP_L_MASK	(0X7)
#define SC8527_V1X_TH_CV2CP_L_SHIFT	(0X0)

/* Register 17h */
#define SC8527_REG_17		0x17
#define SC8527_V1X_TH_CV2CP_MASK	(0XF0)
#define SC8527_V1X_TH_CV2CP_SHIFT	(0X4)
#define SC8527_V1X_TH_CV2CP_H_MASK	(0X0F)
#define SC8527_V1X_TH_CV2CP_H_SHIFT	(0X0)

/* Register 1Ah: CONV_STAT1 Register */
#define SC8527_REG_1A		0x1A
#define SC8527_CONV_MODE_STAT_MASK	(0X30)
#define SC8527_CONV_MODE_STAT_SHIFT	(0X4)

/* Register 1Bh: CONV_STAT2 Register */
#define SC8527_REG_1B		0x1B
#define SC8527_CONV_STAT2_IIND_LIM_FLAG		BIT(0)
#define SC8527_CONV_STAT2_IIND_LIM_FLAG_MASK	BIT(0)

/* Register 1Ch */
#define SC8527_REG_1C		0x1C
#define SC8527_IIND_LIM_MASK	(0XF)
#define SC8527_IIND_LIM_SHIFT	(0X0)

/* Register 1Eh */
#define SC8527_REG_1E		0x1E
#define SC8527_SHIPMODE_EN_MASK	(0XE0)
#define SC8527_SHIPMODE_EN_SHIFT	(0X5)
#define SC8527_SHIPMODE_EN_DLY_MASK	(0X8)
#define SC8527_SHIPMODE_EN_DLY_SHIFT	(0X3)

/* Register 1Fh */
#define SC8527_REG_1F		0x1F
#define SC8527_SHIPMODE_EN_ALLOW_MASK	(0X80)
#define SC8527_SHIPMODE_EN_ALLOW_SHIF	(0X7)

/* Register 27h */
#define SC8527_REG_27		0x27
#define SC8527_RX_RDATA_POL_H_MASK		0xFF

/* Register 2Ah */
#define SC8527_REG_2A		0x2A
#define SC8527_PRE_WDATA_POL_H_MASK		0x03

/* Register 2Bh */
#define SC8527_REG_2B		0x2B

/* Register 25h */
#define SC8527_REG_25		0x25
#define SC8527_TX_WDATA_POL_H_MASK		0x03

/* Register 26h */
#define SC8527_REG_26		0x26

/* Register 29h */
#define SC8527_REG_29		0x29
#define SC8527_REG_20		0x20
#define SC8527_REG_21		0x21
#define SC8527_REG_24		0x24
#define SC8527_REG_2A		0x2A

/* Register 24h */
#define SC8527_REG_24		0x24
#define SC8527_VOOC_EN_MASK	0x80
#define SC8527_VOOC_EN_SHIFT	7
#define SC8527_VOOC_ENABLE	1
#define SC8527_VOOC_DISABLE	0

#define SC8527_SOFT_RESET_MASK	0x02
#define SC8527_SOFT_RESET_SHIFT	1
#define SC8527_SOFT_RESET		1

/* Register 2Ch */
#define SC8527_REG_2C		0x2C

/* Register 2Dh */
#define SC8527_REG_2D		0x2D

/* Register 37h */
#define SC8527_REG_37		0x37
#define SC8527_BCL_MODE_MASK	(0XC0)
#define SC8527_BCL_MODE_SHIFT	(0X6)
#define SC8527_SNS_RATIO_MASK	(0X30)
#define SC8527_SNS_RATIO_SHIFT	(0X4)
#define SC8527_K_CHANGE_MASK	(0XF)
#define SC8527_K_CHANGE_SHIFT	(0X0)
/* Register 38h */
#define SC8527_REG_38		0x38
#define SC8527_SNS_CLAMP_MASK	(0X01)
#define SC8527_SNS_CLAMP_SHIFT	(0X0)
/* Register 3ah */
#define SC8527_REG_3A		0x3A
#define SC8527_VAC2V2X_UVP_ENABLE_MASK		BIT(6)
#define SC8527_VAC2V2X_UVP_ENABLE		0
#define SC8527_VAC2V2X_UVP_DISABLE		BIT(6)
#define SC8527_Q2OCP_MASK	(0X02)
#define SC8527_Q2OCP_SHIFT	(0X1)

/*reg 0x40*/
#define SC8527_ADDR_UFCS_CTRL1		(0x40)
#define SEND_SOURCE_HARDRESET		BIT(0)
#define SEND_CABLE_HARDRESET		BIT(1)
#define FLAG_BAUD_RATE_VALUE		(BIT(4) | BIT(3))
#define FLAG_BAUD_NUM_SHIFT		(3)
#define SC8527_CMD_EN_CHIP		(0x80)
#define SC8527_CMD_DIS_CHIP		(0X00)
#define SC8527_MASK_EN_HANDSHAKE	BIT(5)
#define SC8527_CMD_EN_HANDSHAKE	BIT(5)

/*reg 0x41*/
#define SC8527_ADDR_UFCS_CTRL2		(0x41)
#define SC8527_CMD_CLR_TX_RX		(0x30)

/*reg 0x43*/
#define SC8527_ADDR_GENERAL_INT_FLAG1		(0x43)
#define SC8527_FLAG_ACK_RECEIVE_TIMEOUT		BIT(0)
#define SC8527_FLAG_MSG_TRANS_FAIL			BIT(1)
/*#define RX_BUFFER_BUSY_FLAG BIT(2)*/
#define SC8527_FLAG_RX_OVERFLOW			BIT(3)
#define SC8527_FLAG_DATA_READY			BIT(4)
#define SC8527_FLAG_SENT_PACKET_COMPLETE	BIT(5)
#define SC8527_FLAG_HANDSHAKE_SUCCESS		BIT(6)
#define SC8527_FLAG_HANDSHAKE_FAIL		BIT(7)

/*reg 0x44*/
#define SC8527_ADDR_GENERAL_INT_FLAG2		(0x44)
#define SC8527_FLAG_HARD_RESET			BIT(0)
#define SC8527_FLAG_CRC_ERROR			BIT(1)
#define SC8527_FLAG_STOP_ERROR			BIT(2)
#define SC8527_FLAG_START_FAIL			BIT(3)
#define SC8527_FLAG_LENGTH_ERROR		BIT(4)
#define SC8527_FLAG_DATA_BYTE_TIMEOUT		BIT(5)
#define SC8527_FLAG_TRAINING_BYTE_ERROR	BIT(6)
#define SC8527_FLAG_BAUD_RATE_ERROR		BIT(7)

/*reg 0x45*/
#define SC8527_ADDR_GENERAL_INT_FLAG3		(0x45)
#define SC8527_FLAG_BAUD_RATE_CHANGE		BIT(6)
#define SC8527_FLAG_BUS_CONFLICT		BIT(7)

/*reg 0x46*/
#define SC8527_ADDR_UFCS_INT_MASK1		(0x46)
#define SC8527_CMD_MASK_ACK_TIMEOUT		(0x01)

/*reg 0x47*/
#define SC8527_ADDR_UFCS_INT_MASK2		(0x47)
#define SC8527_MASK_TRANING_BYTE_ERROR		(0x40)

/*reg 0x48*/
#define SC8527_ADDR_UFCS_INT_MASK3		(0x48)

/*tx_buffer*/
#define SC8527_ADDR_TX_LENGTH		(0x49)
#define SC8527_ADDR_TX_BUFFER0		(0x4A)
#define SC8527_ADDR_TX_BUFFER63		(0x89)

/*rx_buffer*/
#define SC8527_ADDR_RX_LENGTH		(0x8A)
#define SC8527_ADDR_RX_BUFFER0		(0x8B)
#define SC8527_ADDR_RX_BUFFER63		(0xCA)
#define SC8527_LEN_MAX			64

#define SC8527_DEVICE_ID		0x14
#define SC8527_VENDOR_ID		0x0006

#define SC8527_REG_CC			0xCC

/*********************command buffer**************/
/*avoid to rewrite the baudrate flags*/
#define SC8527_CMD_SND_CMP		BIT(2)
#define SC8527_MASK_SND_CMP		BIT(2)
#endif
