/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   tvp5150yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.0
 *
 * Author:
 * -------
 *  Lu
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2013.03.04  New settings Release
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "tvp5150yuv_Sensor.h"
#include "tvp5150yuv_Camera_Sensor_para.h"
#include "tvp5150yuv_CameraCustomized.h"

//#define TVP5150YUV_DEBUG
#ifdef TVP5150YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 TVP5150_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, TVP5150_WRITE_ID);

}
kal_uint16 TVP5150_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, TVP5150_WRITE_ID);
	
    return get_byte;
}


/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   TVP5150_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 TVP5150_dummy_pixels = 0, TVP5150_dummy_lines = 0;
kal_bool   TVP5150_MODE_CAPTURE = KAL_FALSE;
kal_bool   TVP5150_NIGHT_MODE = KAL_FALSE;

kal_uint32 TVP5150_isp_master_clock;
static kal_uint32 TVP5150_g_fPV_PCLK = 26;

kal_uint8 TVP5150_sensor_write_I2C_address = TVP5150_WRITE_ID;
kal_uint8 TVP5150_sensor_read_I2C_address = TVP5150_READ_ID;

UINT8 TVP5150PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT TVP5150SensorConfigData;

#define TVP5150_SET_PAGE0 	TVP5150_write_cmos_sensor(0xfe, 0x00)
#define TVP5150_SET_PAGE1 	TVP5150_write_cmos_sensor(0xfe, 0x01)


/*************************************************************************
 * FUNCTION
 *	TVP5150_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of TVP5150 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void TVP5150_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_TVP5150_Shutter */


/*************************************************************************
 * FUNCTION
 *	TVP5150_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of TVP5150 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 TVP5150_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = TVP5150_read_cmos_sensor(0x04);
	temp_reg2 = TVP5150_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* TVP5150_read_shutter */


/*************************************************************************
 * FUNCTION
 *	TVP5150_write_reg
 *
 * DESCRIPTION
 *	This function set the register of TVP5150.
 *
 * PARAMETERS
 *	addr : the register index of TVP5150
 *  para : setting parameter of the specified register of TVP5150
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void TVP5150_write_reg(kal_uint32 addr, kal_uint32 para)
{
	TVP5150_write_cmos_sensor(addr, para);
} /* TVP5150_write_reg() */


/*************************************************************************
 * FUNCTION
 *	TVP5150_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from TVP5150.
 *
 * PARAMETERS
 *	addr : the register index of TVP5150
 *
 * RETURNS
 *	the data that read from TVP5150
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 TVP5150_read_reg(kal_uint32 addr)
{
	return TVP5150_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	TVP5150_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void TVP5150_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = TVP5150_read_cmos_sensor(0x42);
	
	if (enalbe)
	{
		TVP5150_write_cmos_sensor(0x42, (temp_AWB_reg |0x02));
	}
	else
	{
		TVP5150_write_cmos_sensor(0x42, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
 * FUNCTION
 *	TVP5150_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of TVP5150 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from TVP5150
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void TVP5150_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* TVP5150_config_window */


/*************************************************************************
 * FUNCTION
 *	TVP5150_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 TVP5150_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
* FUNCTION
*	TVP5150_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


void TVP5150GammaSelect(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
	{
		case TVP5150_RGB_Gamma_m1:						//smallest gamma curve
			TVP5150_write_cmos_sensor(0xfe, 0x00);
			TVP5150_write_cmos_sensor(0xbf, 0x06);
			TVP5150_write_cmos_sensor(0xc0, 0x12);
			TVP5150_write_cmos_sensor(0xc1, 0x22);
			TVP5150_write_cmos_sensor(0xc2, 0x35);
			TVP5150_write_cmos_sensor(0xc3, 0x4b);
			TVP5150_write_cmos_sensor(0xc4, 0x5f);
			TVP5150_write_cmos_sensor(0xc5, 0x72);
			TVP5150_write_cmos_sensor(0xc6, 0x8d);
			TVP5150_write_cmos_sensor(0xc7, 0xa4);
			TVP5150_write_cmos_sensor(0xc8, 0xb8);
			TVP5150_write_cmos_sensor(0xc9, 0xc8);
			TVP5150_write_cmos_sensor(0xca, 0xd4);
			TVP5150_write_cmos_sensor(0xcb, 0xde);
			TVP5150_write_cmos_sensor(0xcc, 0xe6);
			TVP5150_write_cmos_sensor(0xcd, 0xf1);
			TVP5150_write_cmos_sensor(0xce, 0xf8);
			TVP5150_write_cmos_sensor(0xcf, 0xfd);
			break;
		case TVP5150_RGB_Gamma_m2:
			TVP5150_write_cmos_sensor(0xBF, 0x08);
			TVP5150_write_cmos_sensor(0xc0, 0x0F);
			TVP5150_write_cmos_sensor(0xc1, 0x21);
			TVP5150_write_cmos_sensor(0xc2, 0x32);
			TVP5150_write_cmos_sensor(0xc3, 0x43);
			TVP5150_write_cmos_sensor(0xc4, 0x50);
			TVP5150_write_cmos_sensor(0xc5, 0x5E);
			TVP5150_write_cmos_sensor(0xc6, 0x78);
			TVP5150_write_cmos_sensor(0xc7, 0x90);
			TVP5150_write_cmos_sensor(0xc8, 0xA6);
			TVP5150_write_cmos_sensor(0xc9, 0xB9);
			TVP5150_write_cmos_sensor(0xcA, 0xC9);
			TVP5150_write_cmos_sensor(0xcB, 0xD6);
			TVP5150_write_cmos_sensor(0xcC, 0xE0);
			TVP5150_write_cmos_sensor(0xcD, 0xEE);
			TVP5150_write_cmos_sensor(0xcE, 0xF8);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case TVP5150_RGB_Gamma_m3:			
			TVP5150_write_cmos_sensor(0xBF, 0x0B);
			TVP5150_write_cmos_sensor(0xc0, 0x16);
			TVP5150_write_cmos_sensor(0xc1, 0x29);
			TVP5150_write_cmos_sensor(0xc2, 0x3C);
			TVP5150_write_cmos_sensor(0xc3, 0x4F);
			TVP5150_write_cmos_sensor(0xc4, 0x5F);
			TVP5150_write_cmos_sensor(0xc5, 0x6F);
			TVP5150_write_cmos_sensor(0xc6, 0x8A);
			TVP5150_write_cmos_sensor(0xc7, 0x9F);
			TVP5150_write_cmos_sensor(0xc8, 0xB4);
			TVP5150_write_cmos_sensor(0xc9, 0xC6);
			TVP5150_write_cmos_sensor(0xcA, 0xD3);
			TVP5150_write_cmos_sensor(0xcB, 0xDD);
			TVP5150_write_cmos_sensor(0xcC, 0xE5);
			TVP5150_write_cmos_sensor(0xcD, 0xF1);
			TVP5150_write_cmos_sensor(0xcE, 0xFA);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case TVP5150_RGB_Gamma_m4:
			TVP5150_write_cmos_sensor(0xBF, 0x0E);
			TVP5150_write_cmos_sensor(0xc0, 0x1C);
			TVP5150_write_cmos_sensor(0xc1, 0x34);
			TVP5150_write_cmos_sensor(0xc2, 0x48);
			TVP5150_write_cmos_sensor(0xc3, 0x5A);
			TVP5150_write_cmos_sensor(0xc4, 0x6B);
			TVP5150_write_cmos_sensor(0xc5, 0x7B);
			TVP5150_write_cmos_sensor(0xc6, 0x95);
			TVP5150_write_cmos_sensor(0xc7, 0xAB);
			TVP5150_write_cmos_sensor(0xc8, 0xBF);
			TVP5150_write_cmos_sensor(0xc9, 0xCE);
			TVP5150_write_cmos_sensor(0xcA, 0xD9);
			TVP5150_write_cmos_sensor(0xcB, 0xE4);
			TVP5150_write_cmos_sensor(0xcC, 0xEC);
			TVP5150_write_cmos_sensor(0xcD, 0xF7);
			TVP5150_write_cmos_sensor(0xcE, 0xFD);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case TVP5150_RGB_Gamma_m5:
			TVP5150_write_cmos_sensor(0xBF, 0x10);
			TVP5150_write_cmos_sensor(0xc0, 0x20);
			TVP5150_write_cmos_sensor(0xc1, 0x38);
			TVP5150_write_cmos_sensor(0xc2, 0x4E);
			TVP5150_write_cmos_sensor(0xc3, 0x63);
			TVP5150_write_cmos_sensor(0xc4, 0x76);
			TVP5150_write_cmos_sensor(0xc5, 0x87);
			TVP5150_write_cmos_sensor(0xc6, 0xA2);
			TVP5150_write_cmos_sensor(0xc7, 0xB8);
			TVP5150_write_cmos_sensor(0xc8, 0xCA);
			TVP5150_write_cmos_sensor(0xc9, 0xD8);
			TVP5150_write_cmos_sensor(0xcA, 0xE3);
			TVP5150_write_cmos_sensor(0xcB, 0xEB);
			TVP5150_write_cmos_sensor(0xcC, 0xF0);
			TVP5150_write_cmos_sensor(0xcD, 0xF8);
			TVP5150_write_cmos_sensor(0xcE, 0xFD);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case TVP5150_RGB_Gamma_m6:										// largest gamma curve
			TVP5150_write_cmos_sensor(0xBF, 0x14);
			TVP5150_write_cmos_sensor(0xc0, 0x28);
			TVP5150_write_cmos_sensor(0xc1, 0x44);
			TVP5150_write_cmos_sensor(0xc2, 0x5D);
			TVP5150_write_cmos_sensor(0xc3, 0x72);
			TVP5150_write_cmos_sensor(0xc4, 0x86);
			TVP5150_write_cmos_sensor(0xc5, 0x95);
			TVP5150_write_cmos_sensor(0xc6, 0xB1);
			TVP5150_write_cmos_sensor(0xc7, 0xC6);
			TVP5150_write_cmos_sensor(0xc8, 0xD5);
			TVP5150_write_cmos_sensor(0xc9, 0xE1);
			TVP5150_write_cmos_sensor(0xcA, 0xEA);
			TVP5150_write_cmos_sensor(0xcB, 0xF1);
			TVP5150_write_cmos_sensor(0xcC, 0xF5);
			TVP5150_write_cmos_sensor(0xcD, 0xFB);
			TVP5150_write_cmos_sensor(0xcE, 0xFE);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
		case TVP5150_RGB_Gamma_night:									//Gamma for night mode
			TVP5150_write_cmos_sensor(0xBF, 0x0B);
			TVP5150_write_cmos_sensor(0xc0, 0x16);
			TVP5150_write_cmos_sensor(0xc1, 0x29);
			TVP5150_write_cmos_sensor(0xc2, 0x3C);
			TVP5150_write_cmos_sensor(0xc3, 0x4F);
			TVP5150_write_cmos_sensor(0xc4, 0x5F);
			TVP5150_write_cmos_sensor(0xc5, 0x6F);
			TVP5150_write_cmos_sensor(0xc6, 0x8A);
			TVP5150_write_cmos_sensor(0xc7, 0x9F);
			TVP5150_write_cmos_sensor(0xc8, 0xB4);
			TVP5150_write_cmos_sensor(0xc9, 0xC6);
			TVP5150_write_cmos_sensor(0xcA, 0xD3);
			TVP5150_write_cmos_sensor(0xcB, 0xDD);
			TVP5150_write_cmos_sensor(0xcC, 0xE5);
			TVP5150_write_cmos_sensor(0xcD, 0xF1);
			TVP5150_write_cmos_sensor(0xcE, 0xFA);
			TVP5150_write_cmos_sensor(0xcF, 0xFF);
			break;
		default:
			//TVP5150_RGB_Gamma_m1
			TVP5150_write_cmos_sensor(0xfe, 0x00);
			TVP5150_write_cmos_sensor(0xbf, 0x06);
			TVP5150_write_cmos_sensor(0xc0, 0x12);
			TVP5150_write_cmos_sensor(0xc1, 0x22);
			TVP5150_write_cmos_sensor(0xc2, 0x35);
			TVP5150_write_cmos_sensor(0xc3, 0x4b);
			TVP5150_write_cmos_sensor(0xc4, 0x5f);
			TVP5150_write_cmos_sensor(0xc5, 0x72);
			TVP5150_write_cmos_sensor(0xc6, 0x8d);
			TVP5150_write_cmos_sensor(0xc7, 0xa4);
			TVP5150_write_cmos_sensor(0xc8, 0xb8);
			TVP5150_write_cmos_sensor(0xc9, 0xc8);
			TVP5150_write_cmos_sensor(0xca, 0xd4);
			TVP5150_write_cmos_sensor(0xcb, 0xde);
			TVP5150_write_cmos_sensor(0xcc, 0xe6);
			TVP5150_write_cmos_sensor(0xcd, 0xf1);
			TVP5150_write_cmos_sensor(0xce, 0xf8);
			TVP5150_write_cmos_sensor(0xcf, 0xfd);
			break;
	}
}


/*************************************************************************
 * FUNCTION
 *	TVP5150_NightMode
 *
 * DESCRIPTION
 *	This function night mode of TVP5150.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void TVP5150NightMode(kal_bool bEnable)
{
	if (bEnable)
	{	
		//TVP5150_write_cmos_sensor(0xfe, 0x01);
		//if(TVP5150_MPEG4_encode_mode == KAL_TRUE)
			//TVP5150_write_cmos_sensor(0x33, 0x00);
		//else
			//TVP5150_write_cmos_sensor(0x33, 0x30);
		//TVP5150_write_cmos_sensor(0xfe, 0x00);
		//TVP5150GammaSelect(TVP5150_RGB_Gamma_night);
		TVP5150_NIGHT_MODE = KAL_TRUE;
	}
	else 
	{
		//TVP5150_write_cmos_sensor(0xfe, 0x01);
		//if(TVP5150_MPEG4_encode_mode == KAL_TRUE)
			//TVP5150_write_cmos_sensor(0x33, 0x00);
		//else
			//TVP5150_write_cmos_sensor(0x33, 0x20);
		//TVP5150_write_cmos_sensor(0xfe, 0x00);
		//TVP5150GammaSelect(TVP5150_RGB_Gamma_m4);
		TVP5150_NIGHT_MODE = KAL_FALSE;
	}
} /* TVP5150_NightMode */

/*************************************************************************
* FUNCTION
*	TVP5150_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void TVP5150_Sensor_Init(void)
	{

	TVP5150_write_cmos_sensor(0x00,0x00);
	TVP5150_write_cmos_sensor(0x03,0xaf);
	TVP5150_write_cmos_sensor(0x0d,0x00); //(0x0d,0x40)
	TVP5150_write_cmos_sensor(0x0f,0x02);
	
	/*TVP5150_write_cmos_sensor(0xfe , 0x80);    
	TVP5150_write_cmos_sensor(0xfe , 0x80);    
	TVP5150_write_cmos_sensor(0xfc , 0x16);     
	TVP5150_write_cmos_sensor(0xfc , 0x16);     
	TVP5150_write_cmos_sensor(0xfc , 0x16);     
	TVP5150_write_cmos_sensor(0xfc , 0x16);     
	
	TVP5150_write_cmos_sensor(0xfe , 0x00);   
	TVP5150_write_cmos_sensor(0x4f , 0x00);  
	TVP5150_write_cmos_sensor(0x42 , 0x00);  
	TVP5150_write_cmos_sensor(0x03 , 0x00);  
	TVP5150_write_cmos_sensor(0x04 , 0xc0);  
	TVP5150_write_cmos_sensor(0x77 , 0x62);  
	TVP5150_write_cmos_sensor(0x78 , 0x40);  
	TVP5150_write_cmos_sensor(0x79 , 0x4d);  

	TVP5150_write_cmos_sensor(0x05, 0x02); 	
	TVP5150_write_cmos_sensor(0x06, 0x2c); 
	TVP5150_write_cmos_sensor(0x07, 0x00);
	TVP5150_write_cmos_sensor(0x08, 0xb8);

	TVP5150_write_cmos_sensor(0xfe, 0x01);
	TVP5150_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
	TVP5150_write_cmos_sensor(0x2a, 0x60);   //anti-flicker step [7:0]

	TVP5150_write_cmos_sensor(0x2b, 0x02);   //exp level 0  14.28fps
	TVP5150_write_cmos_sensor(0x2c, 0xa0); 
	TVP5150_write_cmos_sensor(0x2d, 0x03);   //exp level 1  12.50fps
	TVP5150_write_cmos_sensor(0x2e, 0x00); 
	TVP5150_write_cmos_sensor(0x2f, 0x03);   //exp level 2  10.00fps
	TVP5150_write_cmos_sensor(0x30, 0xc0); 
	TVP5150_write_cmos_sensor(0x31, 0x05);   //exp level 3  7.14fps
	TVP5150_write_cmos_sensor(0x32, 0x40); 
	TVP5150_write_cmos_sensor(0xfe, 0x00);

	TVP5150_write_cmos_sensor(0xfe , 0x01);    
	TVP5150_write_cmos_sensor(0x4f , 0x00);    
	TVP5150_write_cmos_sensor(0x4c , 0x01);     
	TVP5150_write_cmos_sensor(0xfe , 0x00);    
	//////////////////////////////
	 ///////////AWB///////////
	////////////////////////////////
	TVP5150_write_cmos_sensor(0xfe , 0x01);   
	TVP5150_write_cmos_sensor(0x51 , 0x80);   
	TVP5150_write_cmos_sensor(0x52 , 0x12);   
	TVP5150_write_cmos_sensor(0x53 , 0x80);   
	TVP5150_write_cmos_sensor(0x54 , 0x60);   
	TVP5150_write_cmos_sensor(0x55 , 0x01);   
	TVP5150_write_cmos_sensor(0x56 , 0x06);   
	TVP5150_write_cmos_sensor(0x5b , 0x02);   
	TVP5150_write_cmos_sensor(0x61 , 0xdc);   
	TVP5150_write_cmos_sensor(0x62 , 0xdc);   
	TVP5150_write_cmos_sensor(0x7c , 0x71);   
	TVP5150_write_cmos_sensor(0x7d , 0x00);   
	TVP5150_write_cmos_sensor(0x76 , 0x00);   
	TVP5150_write_cmos_sensor(0x79 , 0x20);   
	TVP5150_write_cmos_sensor(0x7b , 0x00);   
	TVP5150_write_cmos_sensor(0x70 , 0xFF); 
	TVP5150_write_cmos_sensor(0x71 , 0x00);   
	TVP5150_write_cmos_sensor(0x72 , 0x10);   
	TVP5150_write_cmos_sensor(0x73 , 0x40);   
	TVP5150_write_cmos_sensor(0x74 , 0x40);   
	////AWB//
	TVP5150_write_cmos_sensor(0x50 , 0x00);    
	TVP5150_write_cmos_sensor(0xfe , 0x01);    
	TVP5150_write_cmos_sensor(0x4f , 0x00);     
	TVP5150_write_cmos_sensor(0x4c , 0x01);   
	TVP5150_write_cmos_sensor(0x4f , 0x00);   
	TVP5150_write_cmos_sensor(0x4f , 0x00);   
	TVP5150_write_cmos_sensor(0x4f , 0x00);   //wait clr finish   
	TVP5150_write_cmos_sensor(0x4d , 0x36);     ////////////40  
	TVP5150_write_cmos_sensor(0x4e , 0x02);     //SPL D65 
	TVP5150_write_cmos_sensor(0x4e , 0x02);     //SPL D65 
	TVP5150_write_cmos_sensor(0x4d , 0x44);     ////////////40
	TVP5150_write_cmos_sensor(0x4e , 0x02);
	TVP5150_write_cmos_sensor(0x4e , 0x02);
	TVP5150_write_cmos_sensor(0x4e , 0x02);     //SPL D65
	TVP5150_write_cmos_sensor(0x4e , 0x02);      //SPL D65
	TVP5150_write_cmos_sensor(0x4d , 0x53);     /////////////50
	TVP5150_write_cmos_sensor(0x4e , 0x08);    //SPL CWF
	TVP5150_write_cmos_sensor(0x4e , 0x08);      //DNP
	TVP5150_write_cmos_sensor(0x4e , 0x02);   //04//pink      //DNP
	TVP5150_write_cmos_sensor(0x4d , 0x63);     /////////////60
	TVP5150_write_cmos_sensor(0x4e , 0x08);  	//TL84
	TVP5150_write_cmos_sensor(0x4e , 0x08);  	//TL84
	TVP5150_write_cmos_sensor(0x4d , 0x73);     //////////////80
	TVP5150_write_cmos_sensor(0x4e , 0x20);    //SPL   A
	TVP5150_write_cmos_sensor(0x4d , 0x83);     //////////////80
	TVP5150_write_cmos_sensor(0x4e , 0x20);    //SPL   A
	TVP5150_write_cmos_sensor(0x4f , 0x01);      
	
	TVP5150_write_cmos_sensor(0x50 , 0x88);    
	TVP5150_write_cmos_sensor(0xfe , 0x00);     //page0
	
	////////////////////////////////////////////////
	////////////     BLK      //////////////////////
	////////////////////////////////////////////////
	TVP5150_write_cmos_sensor(0x27 , 0x00);   
	TVP5150_write_cmos_sensor(0x2a , 0x40);  
	TVP5150_write_cmos_sensor(0x2b , 0x40);  
	TVP5150_write_cmos_sensor(0x2c , 0x40);  
	TVP5150_write_cmos_sensor(0x2d , 0x40);  
	
	
	//////////////////////////////////////////////
	////////// page  0    ////////////////////////
	//////////////////////////////////////////////
	TVP5150_write_cmos_sensor(0xfe , 0x00);   
	TVP5150_write_cmos_sensor(0x0d , 0x01);    
	TVP5150_write_cmos_sensor(0x0e , 0xe8);    
	TVP5150_write_cmos_sensor(0x0f , 0x02);    
	TVP5150_write_cmos_sensor(0x10 , 0x88);    
	TVP5150_write_cmos_sensor(0x09 , 0x00);    
	TVP5150_write_cmos_sensor(0x0a , 0x00);    
	TVP5150_write_cmos_sensor(0x0b , 0x00);    
	TVP5150_write_cmos_sensor(0x0c , 0x00);    
	TVP5150_write_cmos_sensor(0x16 , 0x00);    
	TVP5150_write_cmos_sensor(0x17 , 0x14);    
	TVP5150_write_cmos_sensor(0x18 , 0x0e);    
	TVP5150_write_cmos_sensor(0x19 , 0x06);    
	
	TVP5150_write_cmos_sensor(0x1b , 0x48);    
	TVP5150_write_cmos_sensor(0x1f , 0xC8);    
	TVP5150_write_cmos_sensor(0x20 , 0x01);    
	TVP5150_write_cmos_sensor(0x21 , 0x78);    
	TVP5150_write_cmos_sensor(0x22 , 0xb0);    
	TVP5150_write_cmos_sensor(0x23 , 0x06);    
	TVP5150_write_cmos_sensor(0x24 , 0x11);    
	TVP5150_write_cmos_sensor(0x26 , 0x00);    
	
	TVP5150_write_cmos_sensor(0x50 , 0x01); //crop mode
	                
	//global gain for range 
	TVP5150_write_cmos_sensor(0x70 , 0x85);   
	
	
	////////////////////////////////////////////////
	////////////     block enable      /////////////
	////////////////////////////////////////////////
	TVP5150_write_cmos_sensor(0x40 , 0x7f);   
	TVP5150_write_cmos_sensor(0x41 , 0x26);   
	TVP5150_write_cmos_sensor(0x42 , 0xff);
	TVP5150_write_cmos_sensor(0x45 , 0x00);   
	TVP5150_write_cmos_sensor(0x44 , 0x02);   
	TVP5150_write_cmos_sensor(0x46 , 0x02);   
	
	TVP5150_write_cmos_sensor(0x4b , 0x01);   
	TVP5150_write_cmos_sensor(0x50 , 0x01);  
	
	//DN & EEINTP
	TVP5150_write_cmos_sensor(0x7e , 0x0a);    
	TVP5150_write_cmos_sensor(0x7f , 0x03);    
	TVP5150_write_cmos_sensor(0x81 , 0x15);    
	TVP5150_write_cmos_sensor(0x82 , 0x85);    
	TVP5150_write_cmos_sensor(0x83 , 0x02);    
	TVP5150_write_cmos_sensor(0x84 , 0xe5);    
	TVP5150_write_cmos_sensor(0x90 , 0xac);    
	TVP5150_write_cmos_sensor(0x92 , 0x02);    
	TVP5150_write_cmos_sensor(0x94 , 0x02);    
	TVP5150_write_cmos_sensor(0x95 , 0x54);    
	
	///////YCP
	TVP5150_write_cmos_sensor(0xd1 , 0x2a);//32
	TVP5150_write_cmos_sensor(0xd2 , 0x2a);//32
	TVP5150_write_cmos_sensor(0xd3 , 0x3d);//add
	TVP5150_write_cmos_sensor(0xdd , 0x14);//18
	TVP5150_write_cmos_sensor(0xde , 0x32);
	TVP5150_write_cmos_sensor(0xe4 , 0x88);
	TVP5150_write_cmos_sensor(0xe5 , 0x40);    
	TVP5150_write_cmos_sensor(0xd7 , 0x0e);    
	                      
	///////////////////////////// 
	//////////////// GAMMA ////// 
	///////////////////////////// 
	//rgb gamma                  
	TVP5150_write_cmos_sensor(0xfe , 0x00);
	TVP5150_write_cmos_sensor(0xbf , 0x10);
	TVP5150_write_cmos_sensor(0xc0 , 0x1c);
	TVP5150_write_cmos_sensor(0xc1 , 0x33);
	TVP5150_write_cmos_sensor(0xc2 , 0x48);
	TVP5150_write_cmos_sensor(0xc3 , 0x5a);
	TVP5150_write_cmos_sensor(0xc4 , 0x6b);
	TVP5150_write_cmos_sensor(0xc5 , 0x7b);
	TVP5150_write_cmos_sensor(0xc6 , 0x95);
	TVP5150_write_cmos_sensor(0xc7 , 0xab);
	TVP5150_write_cmos_sensor(0xc8 , 0xbf);
	TVP5150_write_cmos_sensor(0xc9 , 0xcd);
	TVP5150_write_cmos_sensor(0xca , 0xd9);
	TVP5150_write_cmos_sensor(0xcb , 0xe3);
	TVP5150_write_cmos_sensor(0xcc , 0xeb);
	TVP5150_write_cmos_sensor(0xcd , 0xf7);
	TVP5150_write_cmos_sensor(0xce , 0xfd);
	TVP5150_write_cmos_sensor(0xcf , 0xff);
	         
	///Y gamma           
	TVP5150_write_cmos_sensor(0xfe , 0x00);    
	TVP5150_write_cmos_sensor(0x63 , 0x00);    
	TVP5150_write_cmos_sensor(0x64 , 0x05);    
	TVP5150_write_cmos_sensor(0x65 , 0x0c);    
	TVP5150_write_cmos_sensor(0x66 , 0x1a);    
	TVP5150_write_cmos_sensor(0x67 , 0x29);    
	TVP5150_write_cmos_sensor(0x68 , 0x39);    
	TVP5150_write_cmos_sensor(0x69 , 0x4b);    
	TVP5150_write_cmos_sensor(0x6a , 0x5e);    
	TVP5150_write_cmos_sensor(0x6b , 0x82);    
	TVP5150_write_cmos_sensor(0x6c , 0xa4);    
	TVP5150_write_cmos_sensor(0x6d , 0xc5);    
	TVP5150_write_cmos_sensor(0x6e , 0xe5);    
	TVP5150_write_cmos_sensor(0x6f , 0xFF);
	               
	//////ASDE             
	TVP5150_write_cmos_sensor(0xfe , 0x01);    
	TVP5150_write_cmos_sensor(0x18 , 0x02);    
	TVP5150_write_cmos_sensor(0xfe , 0x00);    
	TVP5150_write_cmos_sensor(0x98 , 0x00);    
	TVP5150_write_cmos_sensor(0x9b , 0x20);    
	TVP5150_write_cmos_sensor(0x9c , 0x80);    
	TVP5150_write_cmos_sensor(0xa4 , 0x10);    
	TVP5150_write_cmos_sensor(0xa8 , 0xB0);    
	TVP5150_write_cmos_sensor(0xaa , 0x40);    
	TVP5150_write_cmos_sensor(0xa2 , 0x23);    
	TVP5150_write_cmos_sensor(0xad , 0x01);    
	
	//////////////////////////////////////////////
	////////// AEC    ////////////////////////
	//////////////////////////////////////////////
	TVP5150_write_cmos_sensor(0xfe , 0x01);   
	TVP5150_write_cmos_sensor(0x9c , 0x00);  //02 
	TVP5150_write_cmos_sensor(0x08 , 0xa0);   
	TVP5150_write_cmos_sensor(0x09 , 0xe8);   
	
	TVP5150_write_cmos_sensor(0x10 , 0x00);  
	TVP5150_write_cmos_sensor(0x11 , 0x11);   
	TVP5150_write_cmos_sensor(0x12 , 0x10);   
	TVP5150_write_cmos_sensor(0x13 , 0x98);   
	TVP5150_write_cmos_sensor(0x15 , 0xfc);   
	TVP5150_write_cmos_sensor(0x18 , 0x03);
	TVP5150_write_cmos_sensor(0x21 , 0xc0);   
	TVP5150_write_cmos_sensor(0x22 , 0x60);   
	TVP5150_write_cmos_sensor(0x23 , 0x30);   
	TVP5150_write_cmos_sensor(0x25 , 0x00);   
	TVP5150_write_cmos_sensor(0x24 , 0x14);   
	
	
	//////////////////////////////////////
	////////////LSC//////////////////////
	//////////////////////////////////////
	//tvp5150 Alight lsc reg setting list
	////Record date: 2013-04-01 15:59:05
	TVP5150_write_cmos_sensor(0xfe , 0x01);
	TVP5150_write_cmos_sensor(0xc0 , 0x10);
	TVP5150_write_cmos_sensor(0xc1 , 0x0c);
	TVP5150_write_cmos_sensor(0xc2 , 0x0a);
	TVP5150_write_cmos_sensor(0xc6 , 0x0e);
	TVP5150_write_cmos_sensor(0xc7 , 0x0b);
	TVP5150_write_cmos_sensor(0xc8 , 0x0a);
	TVP5150_write_cmos_sensor(0xba , 0x26);
	TVP5150_write_cmos_sensor(0xbb , 0x1c);
	TVP5150_write_cmos_sensor(0xbc , 0x1d);
	TVP5150_write_cmos_sensor(0xb4 , 0x23);
	TVP5150_write_cmos_sensor(0xb5 , 0x1c);
	TVP5150_write_cmos_sensor(0xb6 , 0x1a);
	TVP5150_write_cmos_sensor(0xc3 , 0x00);
	TVP5150_write_cmos_sensor(0xc4 , 0x00);
	TVP5150_write_cmos_sensor(0xc5 , 0x00);
	TVP5150_write_cmos_sensor(0xc9 , 0x00);
	TVP5150_write_cmos_sensor(0xca , 0x00);
	TVP5150_write_cmos_sensor(0xcb , 0x00);
	TVP5150_write_cmos_sensor(0xbd , 0x00);
	TVP5150_write_cmos_sensor(0xbe , 0x00);
	TVP5150_write_cmos_sensor(0xbf , 0x00);
	TVP5150_write_cmos_sensor(0xb7 , 0x07);
	TVP5150_write_cmos_sensor(0xb8 , 0x05);
	TVP5150_write_cmos_sensor(0xb9 , 0x05);
	TVP5150_write_cmos_sensor(0xa8 , 0x07);
	TVP5150_write_cmos_sensor(0xa9 , 0x06);
	TVP5150_write_cmos_sensor(0xaa , 0x00);
	TVP5150_write_cmos_sensor(0xab , 0x04);
	TVP5150_write_cmos_sensor(0xac , 0x00);
	TVP5150_write_cmos_sensor(0xad , 0x02);
	TVP5150_write_cmos_sensor(0xae , 0x0d);
	TVP5150_write_cmos_sensor(0xaf , 0x05);
	TVP5150_write_cmos_sensor(0xb0 , 0x00);
	TVP5150_write_cmos_sensor(0xb1 , 0x07);
	TVP5150_write_cmos_sensor(0xb2 , 0x03);
	TVP5150_write_cmos_sensor(0xb3 , 0x00);
	TVP5150_write_cmos_sensor(0xa4 , 0x00);
	TVP5150_write_cmos_sensor(0xa5 , 0x00);
	TVP5150_write_cmos_sensor(0xa6 , 0x00);
	TVP5150_write_cmos_sensor(0xa7 , 0x00);
	TVP5150_write_cmos_sensor(0xa1 , 0x3c);
	TVP5150_write_cmos_sensor(0xa2 , 0x50);
	TVP5150_write_cmos_sensor(0xfe , 0x00);
	              
	///cct       
	TVP5150_write_cmos_sensor(0xB1 , 0x04);   
	TVP5150_write_cmos_sensor(0xB2 , 0xfd);   
	TVP5150_write_cmos_sensor(0xB3 , 0xfc);   
	TVP5150_write_cmos_sensor(0xB4 , 0xf0);   
	TVP5150_write_cmos_sensor(0xB5 , 0x05);   
	TVP5150_write_cmos_sensor(0xB6 , 0xf0);   
	
	Sleep(200);
	
	TVP5150_write_cmos_sensor(0xfe , 0x00);   
	TVP5150_write_cmos_sensor(0x27 , 0xf7);  
	TVP5150_write_cmos_sensor(0x28 , 0x7F);   
	TVP5150_write_cmos_sensor(0x29 , 0x20);   
	TVP5150_write_cmos_sensor(0x33 , 0x20);   
	TVP5150_write_cmos_sensor(0x34 , 0x20);   
	TVP5150_write_cmos_sensor(0x35 , 0x20);   
	TVP5150_write_cmos_sensor(0x36 , 0x20);   
	TVP5150_write_cmos_sensor(0x32 , 0x08);   
	
	TVP5150_write_cmos_sensor(0x47 , 0x00);   
	TVP5150_write_cmos_sensor(0x48 , 0x00);   
	
	TVP5150_write_cmos_sensor(0xfe , 0x01);  
	TVP5150_write_cmos_sensor(0x79 , 0x00);  
	TVP5150_write_cmos_sensor(0x7d , 0x00);   
	TVP5150_write_cmos_sensor(0x50 , 0x88);   
	TVP5150_write_cmos_sensor(0x5b , 0x0c); 
	TVP5150_write_cmos_sensor(0x76 , 0x8f);   
	TVP5150_write_cmos_sensor(0x80 , 0x70);
	TVP5150_write_cmos_sensor(0x81 , 0x70);
	TVP5150_write_cmos_sensor(0x82 , 0xb0);
	TVP5150_write_cmos_sensor(0x70 , 0xff); 
	TVP5150_write_cmos_sensor(0x71 , 0x00); 
	TVP5150_write_cmos_sensor(0x72 , 0x28); 
	TVP5150_write_cmos_sensor(0x73 , 0x0b); 
	TVP5150_write_cmos_sensor(0x74 , 0x0b); 
	
	TVP5150_write_cmos_sensor(0xfe , 0x00);  
	TVP5150_write_cmos_sensor(0x70 , 0x45);  
	TVP5150_write_cmos_sensor(0x4f , 0x01);  
	TVP5150_write_cmos_sensor(0xf1 , 0x07);  
	
	TVP5150_write_cmos_sensor(0xf2 , 0x01);  */
	}



/*************************************************************************
* FUNCTION
*	TVP5150GetSensorID
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate lens parameter.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/



UINT32 TVP5150GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int i;


    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	//sensor_id = TVP5150_read_cmos_sensor(0xf0);
					sensor_id = TVP5150_read_cmos_sensor(0x80);
	            	printk("TVP5150GetSensorID TVP5150 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == TVP5150_SENSOR_ID)   ///TVP5150_SENSOR_ID = 0x9d
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != TVP5150_SENSOR_ID)
    {
        *sensorID = 0xFFFFFFFF;
        SENSORDB("TVP5150GetSensorID TVP5150 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    *sensorID = sensor_id;

    RETAILMSG(1, (TEXT("TVP5150GetSensorID Sensor Read ID OK \r\n")));
	printk("TVP5150GetSensorID----ok\n");
    return ERROR_NONE;
}




/*************************************************************************
* FUNCTION
*	TVP5150_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_TVP5150() directly.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void TVP5150_Write_More_Registers(void)
{
	
	///////////////////////////////////////////////
    	//TVP5150GammaSelect(TVP5150_RGB_Gamma_m4);//0:use default
}


/*************************************************************************
 * FUNCTION
 *	TVP5150Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 TVP5150Open(void)
{
    kal_uint16 sensor_id=0;
    int i;

    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	//sensor_id = TVP5150_read_cmos_sensor(0xf0);
					sensor_id = TVP5150_read_cmos_sensor(0x80);
	            	printk("TVP5150Open TVP5150 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == TVP5150_SENSOR_ID)  //  TVP5150_SENSOR_ID= 0x9d
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != TVP5150_SENSOR_ID)
    {
        SENSORDB("TVP5150 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	
     TVP5150_MPEG4_encode_mode = KAL_FALSE;
    RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
    // initail sequence write in
    TVP5150_Sensor_Init();
    TVP5150_Write_More_Registers();
	printk("wuyb------TVP5150Open ok\n");
    return ERROR_NONE;
} /* TVP5150Open */


/*************************************************************************
 * FUNCTION
 *	TVP5150Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 TVP5150Close(void)
{
    return ERROR_NONE;
} /* TVP5150Close */


/*************************************************************************
 * FUNCTION
 * TVP5150Preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 TVP5150Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        TVP5150_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        TVP5150_MPEG4_encode_mode = KAL_FALSE;
    }

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&TVP5150SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* TVP5150Preview */


/*************************************************************************
 * FUNCTION
 *	TVP5150Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 TVP5150Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    TVP5150_MODE_CAPTURE=KAL_TRUE;

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&TVP5150SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* TVP5150_Capture() */



UINT32 TVP5150GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* TVP5150GetResolution() */


UINT32 TVP5150GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY; //SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH; //SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 4;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->YUVAwbDelayFrame = 2;  // add by lanking
    pSensorInfo->YUVEffectDelayFrame = 2;  // add by lanking
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_PIXELS;

        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        break;
    }
    TVP5150PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &TVP5150SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* TVP5150GetInfo() */


UINT32 TVP5150Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
       // TVP5150Capture(pImageWindow, pSensorConfigData);
         TVP5150Preview(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* TVP5150Control() */

BOOL TVP5150_set_param_wb(UINT16 para)
{

	switch (para)
	{
		case AWB_MODE_OFF:

		break;
		
		case AWB_MODE_AUTO:
			TVP5150_awb_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			TVP5150_awb_enable(KAL_FALSE);
			TVP5150_write_cmos_sensor(0x77, 0x8c); //WB_manual_gain 
			TVP5150_write_cmos_sensor(0x78, 0x50);
			TVP5150_write_cmos_sensor(0x79, 0x40);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			TVP5150_awb_enable(KAL_FALSE);
			TVP5150_write_cmos_sensor(0x77, 0x74); 
			TVP5150_write_cmos_sensor(0x78, 0x52);
			TVP5150_write_cmos_sensor(0x79, 0x40);			
		break;
		
		case AWB_MODE_INCANDESCENT: //office
			TVP5150_awb_enable(KAL_FALSE);
			TVP5150_write_cmos_sensor(0x77, 0x48);
			TVP5150_write_cmos_sensor(0x78, 0x40);
			TVP5150_write_cmos_sensor(0x79, 0x5c);
		break;
		
		case AWB_MODE_TUNGSTEN: //home
			TVP5150_awb_enable(KAL_FALSE);
			TVP5150_write_cmos_sensor(0x77, 0x40);
			TVP5150_write_cmos_sensor(0x78, 0x54);
			TVP5150_write_cmos_sensor(0x79, 0x70);
		break;
		
		case AWB_MODE_FLUORESCENT:
			TVP5150_awb_enable(KAL_FALSE);
			TVP5150_write_cmos_sensor(0x77, 0x40);
			TVP5150_write_cmos_sensor(0x78, 0x42);
			TVP5150_write_cmos_sensor(0x79, 0x50);
		break;
		
		default:
		return FALSE;
	}

	return TRUE;
} /* TVP5150_set_param_wb */


BOOL TVP5150_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;

	switch (para)
	{
		case MEFFECT_OFF:
			TVP5150_write_cmos_sensor(0x43 , 0x00);
		break;
		
		case MEFFECT_SEPIA:
			TVP5150_write_cmos_sensor(0x43 , 0x02);
			TVP5150_write_cmos_sensor(0xda , 0xd0);
			TVP5150_write_cmos_sensor(0xdb , 0x28);
		break;
		
		case MEFFECT_NEGATIVE:
			TVP5150_write_cmos_sensor(0x43 , 0x01);
		break;
		
		case MEFFECT_SEPIAGREEN:
			TVP5150_write_cmos_sensor(0x43 , 0x02);
			TVP5150_write_cmos_sensor(0xda , 0xc0);
			TVP5150_write_cmos_sensor(0xdb , 0xc0);
		break;
		
		case MEFFECT_SEPIABLUE:
			TVP5150_write_cmos_sensor(0x43 , 0x02);
			TVP5150_write_cmos_sensor(0xda , 0x50);
			TVP5150_write_cmos_sensor(0xdb , 0xe0);
		break;

		case MEFFECT_MONO:
			TVP5150_write_cmos_sensor(0x43 , 0x02);
			TVP5150_write_cmos_sensor(0xda , 0x00);
			TVP5150_write_cmos_sensor(0xdb , 0x00);
		break;
		default:
			ret = FALSE;
	}

	return ret;

} /* TVP5150_set_param_effect */


BOOL TVP5150_set_param_banding(UINT16 para)
{
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			TVP5150_write_cmos_sensor(0x05, 0x02); 	
			TVP5150_write_cmos_sensor(0x06, 0x2c); 
			TVP5150_write_cmos_sensor(0x07, 0x00);
			TVP5150_write_cmos_sensor(0x08, 0xb8);
			
			TVP5150_SET_PAGE1;
			TVP5150_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			TVP5150_write_cmos_sensor(0x2a, 0x60);   //anti-flicker step [7:0]
			
			TVP5150_write_cmos_sensor(0x2b, 0x02);   //exp level 0  14.28fps
			TVP5150_write_cmos_sensor(0x2c, 0xa0); 
			TVP5150_write_cmos_sensor(0x2d, 0x03);   //exp level 1  12.50fps
			TVP5150_write_cmos_sensor(0x2e, 0x00); 
			TVP5150_write_cmos_sensor(0x2f, 0x04);   //exp level 2  10.00fps
			TVP5150_write_cmos_sensor(0x30, 0xe0); 
			TVP5150_write_cmos_sensor(0x31, 0x06);   //exp level 3  7.14fps
			TVP5150_write_cmos_sensor(0x32, 0x60); 
			TVP5150_SET_PAGE0;
			break;

		case AE_FLICKER_MODE_60HZ:
			TVP5150_write_cmos_sensor(0x05, 0x02); 	
			TVP5150_write_cmos_sensor(0x06, 0x4c); 
			TVP5150_write_cmos_sensor(0x07, 0x00);
			TVP5150_write_cmos_sensor(0x08, 0x88);
			
			TVP5150_SET_PAGE1;
			TVP5150_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			TVP5150_write_cmos_sensor(0x2a, 0x4e);   //anti-flicker step [7:0]
			
			TVP5150_write_cmos_sensor(0x2b, 0x02);   //exp level 0  15.00fps
			TVP5150_write_cmos_sensor(0x2c, 0x70); 
			TVP5150_write_cmos_sensor(0x2d, 0x03);   //exp level 0  12.00fps
			TVP5150_write_cmos_sensor(0x2e, 0x0c); 
			TVP5150_write_cmos_sensor(0x2f, 0x04);   //exp level 0  10.00fps
			TVP5150_write_cmos_sensor(0x30, 0x92); 
			TVP5150_write_cmos_sensor(0x31, 0x05);   //exp level 0  7.05fps
			TVP5150_write_cmos_sensor(0x32, 0xca); 
			TVP5150_SET_PAGE0;
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* TVP5150_set_param_banding */


BOOL TVP5150_set_param_exposure(UINT16 para)
{
	switch (para)
	{
		case AE_EV_COMP_n13:
			TVP5150_write_cmos_sensor(0xd5, 0xc0);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0x60);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_n10:
			TVP5150_write_cmos_sensor(0xd5, 0xd0);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0x68);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_n07:
			TVP5150_write_cmos_sensor(0xd5, 0xe0);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0x70);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_n03:
			TVP5150_write_cmos_sensor(0xd5, 0xf0);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0x78);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;				
		
		case AE_EV_COMP_00:
			TVP5150_write_cmos_sensor(0xd5, 0x00);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0x98);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;

		case AE_EV_COMP_03:
			TVP5150_write_cmos_sensor(0xd5, 0x10);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0xa0);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_07:
			TVP5150_write_cmos_sensor(0xd5, 0x20);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0xa8);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_10:
			TVP5150_write_cmos_sensor(0xd5, 0x30);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0xb0);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		
		case AE_EV_COMP_13:
			TVP5150_write_cmos_sensor(0xd5, 0x40);
			TVP5150_write_cmos_sensor(0xfe, 0x01);
			TVP5150_write_cmos_sensor(0x13, 0xb8);
			TVP5150_write_cmos_sensor(0xfe, 0x00);
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* TVP5150_set_param_exposure */

UINT32 TVP5150YUVSetVideoMode(UINT16 u2FrameRate)    
{
  
        TVP5150_MPEG4_encode_mode = KAL_TRUE;
     if (u2FrameRate == 30)
   	{
   	
   	    /*********video frame ************/
		
   	}
    else if (u2FrameRate == 15)       
    	{
    	
   	    /*********video frame ************/
		
    	}
    else
   	{
   	
            SENSORDB("Wrong Frame Rate"); 
			
   	}

      return TRUE;

}


UINT32 TVP5150YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    switch (iCmd) {
    case FID_AWB_MODE:
        //TVP5150_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        //TVP5150_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        //TVP5150_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
       // TVP5150_set_param_banding(iPara);
		break;
	case FID_SCENE_MODE:
		//TVP5150NightMode(iPara);
        break;
    default:
        break;
    }
    return TRUE;
} /* TVP5150YUVSensorSetting */


UINT32 TVP5150FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 TVP5150SensorRegNumber;
    UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang TVP5150FeatureControl FeatureId=%d\r\n"), FeatureId));

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+TVP5150_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+TVP5150_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = TVP5150_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        //TVP5150NightMode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        TVP5150_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        TVP5150_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = TVP5150_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &TVP5150SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        TVP5150YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:    //  lanking
	 TVP5150YUVSetVideoMode(*pFeatureData16);
	 break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	TVP5150GetSensorID(pFeatureData32);
	break;
    default:
        break;
	}
return ERROR_NONE;
}	/* TVP5150FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncTVP5150YUV=
{
	TVP5150Open,
	TVP5150GetInfo,
	TVP5150GetResolution,
	TVP5150FeatureControl,
	TVP5150Control,
	TVP5150Close
};


UINT32 TVP5150_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncTVP5150YUV;
	return ERROR_NONE;
} /* SensorInit() */
