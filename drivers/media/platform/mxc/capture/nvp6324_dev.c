#define DEBUG

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/videodev2.h>
#include <linux/delay.h>

#include <linux/fsl_devices.h>
#include <linux/regulator/consumer.h>

#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"


#define FMT_NOT_USE			0
#define FMT_CVBS_H720_NTSC	5
#define FMT_CVBS_H720_PAL	6
#define FMT_CVBS_H960_NTSC	7
#define FMT_CVBS_H960_PAL	8
#define FMT_AHD_1080P_30P 	11
#define FMT_AHD_1080P_25P 	12
#define FMT_AHD_720P_60P	13
#define FMT_AHD_720P_30P	15
#define	FMT_AHD_960P_30P	19

#define PCLK_594MHZ 1
#define PCLK_756MHZ 2
#define PCLK_1242MHZ 3

#define MIPI_LANES_2 2
#define MIPI_LANES_4 4

#define YUV422_TYPE 0
#define YUV420_TYPE 1
#define YUV420_LEGACY_TYPE 2

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p);
static int ioctl_s_power(struct v4l2_int_device *s, int on);
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a);
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a);
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f);
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc);
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc);
static int ioctl_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *fsize);
static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *fival);
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id);
static int ioctl_init(struct v4l2_int_device *s);
static int ioctl_dev_init(struct v4l2_int_device *s);
static int ioctl_dev_exit(struct v4l2_int_device *s);

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc nvp6324_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},
	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)
				ioctl_enum_frameintervals},				
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave nvp6324_slave = {
	.ioctls = nvp6324_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(nvp6324_ioctl_desc),
};

static struct v4l2_int_device nvp6324_int_device = {
	.module = THIS_MODULE,
	.name = "nvp6324",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &nvp6324_slave,
	},
};


struct sensor {
	struct sensor_data 	sen;
	v4l2_std_id       	std_id;
	uint16_t 		curr_mod;
	uint16_t		curr_height;
	uint16_t 		curr_width;
	uint16_t 		curr_fps;
	uint8_t                 lanes;
	uint8_t			pclk;
	uint8_t                 pixformat;
	uint8_t 		arb_enable;
	uint8_t 		arb_scale;
} nvp6324_data;



static inline int nvp6324_read_reg(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(nvp6324_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_err(&nvp6324_data.sen.i2c_client->dev,
			 "%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

static inline int nvp6324_write_reg(u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(nvp6324_data.sen.i2c_client, reg, val);

	if (ret < 0) {
		dev_err(&nvp6324_data.sen.i2c_client->dev,
			 "%s:write reg error:reg=%2x,val=%2x\n", __func__,
			 reg, val);
		return -1;
	}

	return 0;
}

int nvp6324_chanState(uint8_t state, uint8_t ch)
{
    char tmp_reg = 0;
      
    if(!state)
    {      
      nvp6324_write_reg(0xFF, 0x21); /////mipi tx phy disable
      nvp6324_write_reg(0x07, 0x80);
      
      //nvp6324_write_reg(0xFF, 0x21); /////mipi tx clk enable
     // nvp6324_write_reg(0x46, 0x01);
      
      nvp6324_write_reg(0xFF, 0x00);
      nvp6324_write_reg(0x00+ch, 0x01); //Power down VAFE
      nvp6324_write_reg(0xFF, 0x01);
      tmp_reg = nvp6324_read_reg(0x95);
      tmp_reg |= 1<<ch;
      nvp6324_write_reg(0x95, tmp_reg); //Power down description2 PRE_Reset1~4
      nvp6324_write_reg(0x96, tmp_reg); //Power down description2 Post_Reset1~4
      nvp6324_write_reg(0x98, tmp_reg); //Power down description2 Powerdwon_Dec1~4
      nvp6324_write_reg(0x9C, tmp_reg); //Power down description2 Powerdwon_VADC_CLK1~4
      nvp6324_write_reg(0x9F, tmp_reg); //Power down description2 Powerdwon_TimeMultiflex1~4
      tmp_reg ^= 0x0F;
      nvp6324_write_reg(0x97, tmp_reg); //Power down description2 CH_Reset1~4
      nvp6324_data.arb_enable &= ~((1<<ch)|(1<<ch+4));
      dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 channel num:%d down\n", ch);
      //arb	
      if(nvp6324_data.lanes == 4) //4lane
      {
	    nvp6324_write_reg(0xFF, 0x20);
	    nvp6324_write_reg(0x40, 0x01);
	    nvp6324_write_reg(0x00, 0x00);
	    nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	    nvp6324_write_reg(0x0F, 0x00);
	    nvp6324_write_reg(0x0D, 0x01);
	    nvp6324_write_reg(0x1B, 0x00);
	    nvp6324_write_reg(0x1C, 0x00);
	    nvp6324_write_reg(0x40, 0x00);
	    nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
      }
      else if(nvp6324_data.lanes == 2)//2lane
      {
	    nvp6324_write_reg(0xFF, 0x20);
	    nvp6324_write_reg(0x40, 0x01);
	    nvp6324_write_reg(0x00, 0x00);
	    nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	    nvp6324_write_reg(0x0F, 0x00);
	    nvp6324_write_reg(0x0D, 0x00);
	    nvp6324_write_reg(0x1B, 0x00);
	    nvp6324_write_reg(0x1C, 0x00);
	    nvp6324_write_reg(0x40, 0x00);
	    nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
      }
    }
    else
    {
    //  nvp6324_write_reg(0xFF, 0x21); /////mipi tx clk enable
   //   nvp6324_write_reg(0x46, 0x00);
      
      nvp6324_write_reg(0xFF, 0x21); /////mipi tx phy enable
      nvp6324_write_reg(0x07, 0x00); 
      nvp6324_write_reg(0x07, 0x0F);       
       
      
      nvp6324_write_reg(0xFF, 0x00);
      nvp6324_write_reg(0x00+ch, 0x00); //Power down VAFE
      nvp6324_write_reg(0xFF, 0x01);
      tmp_reg = nvp6324_read_reg(0x95);
      tmp_reg &= ~(1<<ch);
      nvp6324_write_reg(0x95, tmp_reg); //Power down description2 PRE_Reset1~4
      nvp6324_write_reg(0x96, tmp_reg); //Power down description2 Post_Reset1~4
      nvp6324_write_reg(0x98, tmp_reg); //Power down description2 Powerdwon_Dec1~4
      nvp6324_write_reg(0x9C, tmp_reg); //Power down description2 Powerdwon_VADC_CLK1~4
      nvp6324_write_reg(0x9F, tmp_reg); //Power down description2 Powerdwon_TimeMultiflex1~4
      tmp_reg ^= 0x0F;
      nvp6324_write_reg(0x97, tmp_reg); //Power down description2 CH_Reset1~4
      nvp6324_data.arb_enable |= ((1<<ch)|(1<<ch+4));
      dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 channel num:%d up\n", ch);
      //arb	
      if(nvp6324_data.lanes == 4) //4lane
      {
	    nvp6324_write_reg(0xFF, 0x20);
	    nvp6324_write_reg(0x40, 0x01);
	    nvp6324_write_reg(0x00, 0x00);
	    nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	    nvp6324_write_reg(0x0F, 0x00);
	    nvp6324_write_reg(0x0D, 0x01);
	    nvp6324_write_reg(0x1B, 0x00);
	    nvp6324_write_reg(0x1C, 0x00);
	    nvp6324_write_reg(0x40, 0x00);
	    nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
      }
      else if(nvp6324_data.lanes == 2)//2lane
      {
	    nvp6324_write_reg(0xFF, 0x20);
	    nvp6324_write_reg(0x40, 0x01);
	    nvp6324_write_reg(0x00, 0x00);
	    nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	    nvp6324_write_reg(0x0F, 0x00);
	    nvp6324_write_reg(0x0D, 0x00);
	    nvp6324_write_reg(0x1B, 0x00);
	    nvp6324_write_reg(0x1C, 0x00);
	    nvp6324_write_reg(0x40, 0x00);
	    nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
      }      
    }  
}

/*******************************************************************************
*	Description		: Initialize nvp6324 device
*	Argurments		:   1). Input: pvifmt[]
*	                       -. Video Input format - 4channel
*				        2). Input: lane(MIPI lane)
*				           -. 4(4lane), 2(2lane)
*				        3). Input: pclk(MIPI p-clk)
*				           -. 1(594Mhz), 2(756Mhz), 3(1242Mhz)
*	Return value	: 0:success, -1:fail to set.
*	Modify			:
*	warning			: You can use below example.(4lane, 756Mhz, AHD 2M30P 2CH)
*						Usage: 
*						unsigned char lane = 4; 	// 4(4lane)-default, 2(2lane)
*						unsigned char pclk = 2; 	// 1(594Mhz), 2(756Mhz), 3(1242Mhz)
*						unsigned char datatype = 0; // 0:YUV422 8bit(default), 1:YUV420 8bit, 2:YUV420 Legacy 8bit
*						unsigned char vifmt[4] = {FMT_AHD_720P_30P, FMT_AHD_720P_30P, FMT_AHD_720P_30P, FMT_AHD_720P_30P};
*						chip_initialize(vifmt, lane, pclk, datatype);
*
*******************************************************************************/
int nvp6324_chip_initialize(uint8_t pvifmt[], uint16_t lane, uint16_t pclk, uint16_t datatype)
{
    int ch;
    int ret;
    char tmp_reg=0;
 	// default setting
    nvp6324_write_reg(0xFF, 0x00);
    nvp6324_write_reg(0x18, 0x03);
    nvp6324_write_reg(0x19, 0x03);
    nvp6324_write_reg(0x1A, 0x03);
    nvp6324_write_reg(0x1B, 0x03);
    nvp6324_write_reg(0xFF, 0x01);
    nvp6324_write_reg(0x7C, 0x00);
    nvp6324_write_reg(0xFF, 0x04);
    nvp6324_write_reg(0x00, 0x01); //Power down description1 Motion CH1 off
    nvp6324_write_reg(0x07, 0x01); //Power down description1 Motion CH2 off
    nvp6324_write_reg(0x0E, 0x01); //Power down description1 Motion CH3 off
    nvp6324_write_reg(0x15, 0x01); //Power down description1 Motion CH4 off
    nvp6324_write_reg(0xFF, 0x13);
    nvp6324_write_reg(0x30, 0x00);
    nvp6324_write_reg(0x31, 0x00);
    nvp6324_write_reg(0x32, 0x00);
    nvp6324_write_reg(0x33, 0x00);
    nvp6324_write_reg(0xFF, 0x0A);
    nvp6324_write_reg(0x77, 0x8F);
    nvp6324_write_reg(0xF7, 0x8F);
    nvp6324_write_reg(0xFF, 0x0B);
    nvp6324_write_reg(0x77, 0x8F);
    nvp6324_write_reg(0xF7, 0x8F);
    nvp6324_write_reg(0xFF, 0x21);
    nvp6324_write_reg(0x07, 0x80);
    nvp6324_write_reg(0x07, 0x00);

    // choose Lane/pclk
    if(lane == 4) //4lane
    {
	if(pclk == 1) //1(594Mhz), 2(756Mhz), 3(1242Mhz)
	{
		//mipi 4lanes, 594Mbps/lanes, UYVY
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x40, 0xAC);
		nvp6324_write_reg(0x41, 0x10);
		nvp6324_write_reg(0x42, 0x03);
		nvp6324_write_reg(0x43, 0x43);
		nvp6324_write_reg(0x10, 0x0A);
		nvp6324_write_reg(0x11, 0x04);
		nvp6324_write_reg(0x12, 0x06);
		nvp6324_write_reg(0x13, 0x09);
		nvp6324_write_reg(0x14, 0x16);
		nvp6324_write_reg(0x15, 0x04);
		nvp6324_write_reg(0x16, 0x05);
		nvp6324_write_reg(0x17, 0x01);
		nvp6324_write_reg(0x18, 0x0D);
		nvp6324_write_reg(0x19, 0x05);
		nvp6324_write_reg(0x1A, 0x0A);
		nvp6324_write_reg(0x1B, 0x08);
		nvp6324_write_reg(0x1C, 0x07);
		nvp6324_write_reg(0x44, 0x00);
		nvp6324_write_reg(0x49, 0xF3);
		nvp6324_write_reg(0x49, 0xF0);
		nvp6324_write_reg(0x44, 0x02);
		nvp6324_write_reg(0x08, 0x40);
		nvp6324_write_reg(0x0F, 0x01);
		nvp6324_write_reg(0x38, 0x1E);
		nvp6324_write_reg(0x39, 0x1E);
		nvp6324_write_reg(0x3A, 0x1E);
		nvp6324_write_reg(0x3B, 0x1E);
		nvp6324_write_reg(0x3C, 0x00);
		nvp6324_write_reg(0x2D, 0x01);
		nvp6324_write_reg(0x45, 0x02);
		nvp6324_write_reg(0x07, 0x0F);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 MIPI configure: 4Lane, 594Mhz\n");			
	}
	else if(pclk == 2) //1(594Mhz), 2(756Mhz), 3(1242Mhz)
	{
		//mipi 4lanes, 756Mbps/lanes, UYVY(422)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x40, 0xDC);
		nvp6324_write_reg(0x41, 0x10);
		nvp6324_write_reg(0x42, 0x03);
		nvp6324_write_reg(0x43, 0x43);
		nvp6324_write_reg(0x10, 0x0C);
		nvp6324_write_reg(0x11, 0x05);
		nvp6324_write_reg(0x12, 0x07);
		nvp6324_write_reg( 0x13, 0x0B);
		nvp6324_write_reg(0x14, 0x1C);
		nvp6324_write_reg(0x15, 0x04);
		nvp6324_write_reg(0x16, 0x07);
		nvp6324_write_reg(0x17, 0x01);
		nvp6324_write_reg(0x18, 0x0E);
		nvp6324_write_reg(0x19, 0x06);
		nvp6324_write_reg(0x1A, 0x0D);
		nvp6324_write_reg(0x1B, 0x0B);
		nvp6324_write_reg(0x1C, 0x09);
		nvp6324_write_reg(0x44, 0x00);
		nvp6324_write_reg(0x49, 0xF3);
		nvp6324_write_reg(0x49, 0xF0);
		nvp6324_write_reg(0x44, 0x02);
		nvp6324_write_reg(0x08, 0x40);
		nvp6324_write_reg(0x0F, 0x01);
		nvp6324_write_reg(0x38, 0x1E);
		nvp6324_write_reg(0x39, 0x1E);
		nvp6324_write_reg(0x3A, 0x1E);
		nvp6324_write_reg(0x3B, 0x1E);
		nvp6324_write_reg(0x3C, 0x00);
		nvp6324_write_reg(0x2D, 0x01);
		nvp6324_write_reg(0x45, 0x02);
		nvp6324_write_reg(0x07, 0x0F);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 MIPI configure: 4Lane, 756\n");		
	}
	else if(pclk == 3) //1(594Mhz), 2(756Mhz), 3(1242Mhz)
	{
		//mipi 4lanes, 1242Mbps/lanes, UYVY(422)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x40, 0xB4);
		nvp6324_write_reg(0x41, 0x00);
		nvp6324_write_reg(0x42, 0x03);
		nvp6324_write_reg(0x43, 0x43);
		nvp6324_write_reg(0x10, 0x13);
		nvp6324_write_reg(0x11, 0x08);
		nvp6324_write_reg(0x12, 0x0B);
		nvp6324_write_reg(0x13, 0x12);
		nvp6324_write_reg(0x14, 0x2D);
		nvp6324_write_reg(0x15, 0x07);
		nvp6324_write_reg(0x16, 0x0B);
		nvp6324_write_reg(0x17, 0x02);
		nvp6324_write_reg(0x18, 0x12);
		nvp6324_write_reg(0x19, 0x09);
		nvp6324_write_reg(0x1A, 0x15);
		nvp6324_write_reg(0x1B, 0x11);
		nvp6324_write_reg(0x1C, 0x0E);
		nvp6324_write_reg(0x44, 0x00);
		nvp6324_write_reg(0x49, 0xF3);
		nvp6324_write_reg(0x49, 0xF0);
		nvp6324_write_reg(0x44, 0x02);
		nvp6324_write_reg(0x08, 0x40);
		nvp6324_write_reg(0x0F, 0x01);
		nvp6324_write_reg(0x38, 0x1E);
		nvp6324_write_reg(0x39, 0x1E);
		nvp6324_write_reg( 0x3A, 0x1E);
		nvp6324_write_reg(0x3B, 0x1E);
		nvp6324_write_reg(0x3C, 0x00);
		nvp6324_write_reg(0x2D, 0x01);
		nvp6324_write_reg(0x45, 0x02);
		nvp6324_write_reg(0x07, 0x0F);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 MIPI configure: 4Lane, 1242\n");		
	}
	else
	{
		dev_info(&nvp6324_data.sen.i2c_client->dev, "Error! You should choose pclk as 1(594Mhz), 2(756Mhz), 3(1242Mhz)");		
		return -1;
	}
    }
    else if(lane == 2) //2lane
    {
    	if(pclk == 1) //1(594Mhz), 2(756Mhz), 3(1242Mhz)
    	{
    		//mipi 2lanes, 594Mbps/lanes, UYVY
    		nvp6324_write_reg(0xFF, 0x21);
    		nvp6324_write_reg(0x40, 0xAC);
    		nvp6324_write_reg(0x41, 0x10);
    		nvp6324_write_reg(0x42, 0x03);
    		nvp6324_write_reg(0x43, 0x43);
    		nvp6324_write_reg(0x10, 0x0A);
    		nvp6324_write_reg(0x11, 0x04);
    		nvp6324_write_reg(0x12, 0x06);
    		nvp6324_write_reg(0x13, 0x09);
    		nvp6324_write_reg(0x14, 0x16);
    		nvp6324_write_reg(0x15, 0x04);
    		nvp6324_write_reg(0x16, 0x05);
    		nvp6324_write_reg(0x17, 0x01);
    		nvp6324_write_reg(0x18, 0x0D);
    		nvp6324_write_reg(0x19, 0x05);
    		nvp6324_write_reg(0x1A, 0x0A);
    		nvp6324_write_reg(0x1B, 0x08);
    		nvp6324_write_reg(0x1C, 0x07);
    		nvp6324_write_reg(0x44, 0x00);
    		nvp6324_write_reg(0x49, 0xF3);
    		nvp6324_write_reg(0x49, 0xF0);
    		nvp6324_write_reg(0x44, 0x02);
    		nvp6324_write_reg(0x08, 0x40);
    		nvp6324_write_reg(0x0F, 0x01);
    		nvp6324_write_reg(0x38, 0x1E);
    		nvp6324_write_reg(0x39, 0x1E);
    		nvp6324_write_reg(0x3A, 0x1E);
    		nvp6324_write_reg(0x3B, 0x1E);
    		nvp6324_write_reg(0x3C, 0x00);
    		nvp6324_write_reg(0x2D, 0x00);
    		nvp6324_write_reg(0x45, 0x02);
    		nvp6324_write_reg(0x07, 0x07);
    		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 MIPI configure: 2Lane, 594\n");	
    	}
    	else if(pclk == 2) //1(594Mhz), 2(756Mhz), 3(1242Mhz)
    	{
		//mipi 2lanes, 756Mbps/lanes, UYVY
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x40, 0xDC);
		nvp6324_write_reg(0x41, 0x10);
		nvp6324_write_reg(0x42, 0x03);
		nvp6324_write_reg(0x43, 0x43);
		nvp6324_write_reg(0x10, 0x0C);
		nvp6324_write_reg(0x11, 0x05);
		nvp6324_write_reg(0x12, 0x07);
		nvp6324_write_reg(0x13, 0x0B);
		nvp6324_write_reg(0x14, 0x1C);
		nvp6324_write_reg(0x15, 0x04);
		nvp6324_write_reg(0x16, 0x07);
		nvp6324_write_reg(0x17, 0x01);
		nvp6324_write_reg(0x18, 0x0E);
		nvp6324_write_reg(0x19, 0x06);
		nvp6324_write_reg(0x1A, 0x0D);
		nvp6324_write_reg(0x1B, 0x0B);
		nvp6324_write_reg(0x1C, 0x09);
		nvp6324_write_reg(0x44, 0x00);
		nvp6324_write_reg(0x49, 0xF3);
		nvp6324_write_reg(0x49, 0xF0);
		nvp6324_write_reg(0x44, 0x02);
		nvp6324_write_reg(0x08, 0x40);
		nvp6324_write_reg(0x0F, 0x01);
		nvp6324_write_reg(0x38, 0x1E);
		nvp6324_write_reg(0x39, 0x1E);
		nvp6324_write_reg(0x3A, 0x1E);
		nvp6324_write_reg(0x3B, 0x1E);
		nvp6324_write_reg(0x3C, 0x00);
		nvp6324_write_reg(0x2D, 0x00);
		nvp6324_write_reg(0x45, 0x02);
		nvp6324_write_reg(0x07, 0x07);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 MIPI configure: 2Lane, 756\n");	
    	}
	else
	{
		dev_info(&nvp6324_data.sen.i2c_client->dev, "Error! You should choose pclk as 1(594Mhz), 2(756Mhz)\n");
		return -1;
	}
    }
    else
    {
		dev_info(&nvp6324_data.sen.i2c_client->dev, "Error! You should choose lane as 4Lane or 2Lane\n");
		return -1;
    }
    
    // set video format
    for(ch = 0; ch < 4 /*4channel*/; ch++)
    {
        nvp6324_write_reg(0xFF, 0x00);
        nvp6324_write_reg(0x14 + ch, ch);
        nvp6324_write_reg(0xFF, 0x05 + ch);
        nvp6324_write_reg(0x00, 0xD0);
        nvp6324_write_reg(0x02, 0x0C);
        nvp6324_write_reg(0x1E, 0x00);
        nvp6324_write_reg(0x26, 0x40);
        nvp6324_write_reg(0x32, 0x10);
        nvp6324_write_reg(0x59, 0x00);
        nvp6324_write_reg(0x5A, 0x00);
        nvp6324_write_reg(0x5B, 0x41);
        nvp6324_write_reg(0x65, 0x80);
        nvp6324_write_reg(0x94, 0x00);
        nvp6324_write_reg(0x95, 0x00);
        nvp6324_write_reg(0xB9, 0xB2);
        nvp6324_write_reg(0x01, 0x22);
        nvp6324_write_reg(0x92, 0x00);

        if(pvifmt[ch] == FMT_AHD_1080P_30P)
        {
		nvp6324_data.arb_scale &= ~(0x3<<(ch*2));
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x00);
		nvp6324_write_reg(0x08 + ch, 0x02); 
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0x20);
		nvp6324_write_reg(0x18 + ch, 0x01);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x00);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x86);//contrast
		nvp6324_write_reg(0x28 + ch, 0x80);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0xFE);//u_offset
		nvp6324_write_reg(0x50 + ch, 0xFB);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x05);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x48);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);       
		nvp6324_write_reg(0x05, 0x24);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xA8);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x13);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x26);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x03);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x08);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0xA0);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 Video Input[FMT_AHD_1080P_30P] ch:%d\n", ch);
        }
        else if(pvifmt[ch] == FMT_AHD_1080P_25P)
        {
		nvp6324_data.arb_scale &= ~(0x3<<(ch*2));
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x00);
		nvp6324_write_reg(0x08 + ch, 0x03); 
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0x20);
		nvp6324_write_reg(0x18 + ch, 0x01);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x00);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x86);//contrast
		nvp6324_write_reg(0x28 + ch, 0x80);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0xFE);//u_offset
		nvp6324_write_reg(0x50 + ch, 0xFB);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x05);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x48);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c); 
		nvp6324_write_reg(0x05, 0x24);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xA8);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x13);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x26);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x03);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x08);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0xA0);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 Video Input[FMT_AHD_1080P_25P] ch:%d\n", ch);
        }
        else if(pvifmt[ch] == FMT_AHD_960P_30P)
        {
		nvp6324_data.arb_scale &= ~(0x3<<(ch*2));
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x00);
		nvp6324_write_reg(0x08 + ch, 0x02);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0x20);
		nvp6324_write_reg(0x18 + ch, 0x01);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x00);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x10);
		nvp6324_write_reg(0x6F, 0x82);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x76, 0x00);
		nvp6324_write_reg(0x77, 0x80);
		nvp6324_write_reg(0x78, 0x00);
		nvp6324_write_reg(0x79, 0x11);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xFF, 0x11);
		nvp6324_write_reg(0x00 + (ch*0x20), 0x0F);
		nvp6324_write_reg(0x01 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x02 + (ch*0x20), 0x9D);
		nvp6324_write_reg(0x03 + (ch*0x20), 0x05);
		nvp6324_write_reg(0x04 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x05 + (ch*0x20), 0x08);
		nvp6324_write_reg(0x06 + (ch*0x20), 0xCA);
		nvp6324_write_reg(0x0A + (ch*0x20), 0x03);
		nvp6324_write_reg(0x0B + (ch*0x20), 0xC0);
		nvp6324_write_reg(0x0C + (ch*0x20), 0x04);
		nvp6324_write_reg(0x0D + (ch*0x20), 0x4B);
		nvp6324_write_reg(0x10 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x11 + (ch*0x20), 0x96);
		nvp6324_write_reg(0x12 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x13 + (ch*0x20), 0x82);
		nvp6324_write_reg(0x14 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x15 + (ch*0x20), 0x30);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x86);//contrast
		nvp6324_write_reg(0x28 + ch, 0x80);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0xFE);//u_offset
		nvp6324_write_reg(0x50 + ch, 0xFB);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x05);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x80);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x24);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xA8);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x13);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x00);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x26);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x03);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x08);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0xA0);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_AHD_960P_30P] ch:%d\n", ch);
        }
        else if(pvifmt[ch] == FMT_AHD_720P_60P)
        {
		nvp6324_data.arb_scale &= ~(0x3<<(ch*2));
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x00);
		nvp6324_write_reg(0x08 + ch, 0x04);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0x20);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0x20);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x00);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x10);
		nvp6324_write_reg(0x6F, 0x1B);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x86);//contrast
		nvp6324_write_reg(0x28 + ch, 0x80);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x03);//hue
		nvp6324_write_reg(0x44 + ch, 0xFC);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0xF8);//u_offset
		nvp6324_write_reg(0x50 + ch, 0xF8);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x05);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x80);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x24);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xA8);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x14);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x19);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x0D);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x03);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x08);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0xA0);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x05);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x80);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x70);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x64);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_AHD_720P_60P] ch:%d\n", ch);
        }
        else if(pvifmt[ch] == FMT_AHD_720P_30P)
        {
		nvp6324_data.arb_scale |= 0x1<<(ch*2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x00);
		nvp6324_write_reg(0x08 + ch, 0x0C);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0x20);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x00);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x10);
		nvp6324_write_reg(0x6F, 0x1C);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x88);//contrast
		nvp6324_write_reg(0x28 + ch, 0x84);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x03);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x0F);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0x00);//u_offset
		nvp6324_write_reg(0x50 + ch, 0x00);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x05);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x4A);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x24);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xA8);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x00);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x30);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x03);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x07);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0xB0);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_AHD_720P_30P] ch:%d\n", ch);
        }
        else if(pvifmt[ch] == FMT_CVBS_H960_NTSC)
        {
		nvp6324_data.arb_scale |= 0x2<<(ch*2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x0E);
		nvp6324_write_reg(0x08 + ch, 0x00);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0xA0);
		tmp_reg = nvp6324_read_reg(0x14 + ch);
		tmp_reg &= ~(1<<4);
		tmp_reg |= 0x10;
		nvp6324_write_reg(0x14 + ch, tmp_reg);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0xC0);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0x04);
		nvp6324_write_reg(0x50, 0x84);
		nvp6324_write_reg(0x53, 0x04);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x01);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0xB8);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x90);//contrast
		nvp6324_write_reg(0x28 + ch, 0x90);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x08);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0x00);//u_offset
		nvp6324_write_reg(0x50 + ch, 0x00);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x18);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x70);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x20);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xC0);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x00);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x48);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x02);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);

	    	// SD field option(fix to 1 for odd field, 2 for even field)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x3E, 0x11);
		nvp6324_write_reg(0x3F, 0x11);
		nvp6324_write_reg(0x3C, 0x01);

		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_CVBS_H960_NTSC] ch:%d\n", ch);
        }
        else if( pvifmt[ch] == FMT_CVBS_H960_PAL )
        {
		nvp6324_data.arb_scale |= 0x2<<(ch*2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x0F);
		nvp6324_write_reg(0x08 + ch, 0x00);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0xDD);
		tmp_reg = nvp6324_read_reg(0x14 + ch);
		tmp_reg &= ~(1<<4);
		tmp_reg |= 0x0;
		nvp6324_write_reg(0x14 + ch, tmp_reg);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x04);
		nvp6324_write_reg(0x88 + ch, 0x01);
		nvp6324_write_reg(0x8C + ch, 0x02);
		nvp6324_write_reg(0xA0 + ch, 0x00);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x47, 0xEE);
		nvp6324_write_reg(0x50, 0xC6);
		nvp6324_write_reg(0x53, 0x04);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x01);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x0D);
		nvp6324_write_reg(0xA9, 0x00);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0xB8);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x90);//contrast
		nvp6324_write_reg(0x28 + ch, 0x90);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x08);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0x00);//u_offset
		nvp6324_write_reg(0x50 + ch, 0x00);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x07);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0x68);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x03);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x20);
		nvp6324_write_reg(0x1D, 0x0c);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xCC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xC0);
		nvp6324_write_reg(0x31, 0x02);
		nvp6324_write_reg(0x38, 0x00);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0xFF, 0x02 + ch/2); //coax
		nvp6324_write_reg(0x00 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x01 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x02 + (ch%2 * 0x80), 0x48);
		nvp6324_write_reg(0x03 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x04 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x05 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x06 + (ch%2 * 0x80), 0x09);
		nvp6324_write_reg(0x07 + (ch%2 * 0x80), 0x0E);
		nvp6324_write_reg(0x08 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x09 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0A + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x0B + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x0C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x0D + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x0E + (ch%2 * 0x80), 0x02);
		nvp6324_write_reg(0x0F + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x60 + (ch%2 * 0x80), 0x55);
		nvp6324_write_reg(0x61 + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x62 + (ch%2 * 0x80), 0x06);
		nvp6324_write_reg(0x63 + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x66 + (ch%2 * 0x80), 0x81);
		nvp6324_write_reg(0x68 + (ch%2 * 0x80), 0x60);
		nvp6324_write_reg(0x69 + (ch%2 * 0x80), 0x2D);
		nvp6324_write_reg(0x7C + (ch%2 * 0x80), 0x01);
		nvp6324_write_reg(0x7D + (ch%2 * 0x80), 0x80);
  
		// SD field option(fix to 1 for odd field, 2 for even field)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x3E, 0x11);
		nvp6324_write_reg(0x3F, 0x11);
		nvp6324_write_reg(0x3C, 0x01);

		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_CVBS_H960_PAL] ch:%d\n", ch);
        }
        else if( pvifmt[ch] == FMT_CVBS_H720_NTSC )
        {
		nvp6324_data.arb_scale |= 0x1<<(ch*2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x04);
		nvp6324_write_reg(0x08 + ch, 0x00);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0xA0);
		tmp_reg = nvp6324_read_reg(0x14 + ch);
		tmp_reg &= ~(1<<4);
		nvp6324_write_reg(0x14 + ch, tmp_reg);
		nvp6324_write_reg(0x5C + ch, 0x82);
		nvp6324_write_reg(0x78 + ch, 0xC0);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x06);
		nvp6324_write_reg(0x88 + ch, 0x07);
		nvp6324_write_reg(0x8C + ch, 0x01);
		nvp6324_write_reg(0xA0 + ch, 0x10);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x21, 0x24);
		nvp6324_write_reg(0x47, 0x04);
		nvp6324_write_reg(0x50, 0x80);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x01);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x01);
		nvp6324_write_reg(0xA9, 0x1C);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x90);//contrast
		nvp6324_write_reg(0x28 + ch, 0x90);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0x00);//u_offset
		nvp6324_write_reg(0x50 + ch, 0x00);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x1C);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0xD4);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x8F);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x20);
		nvp6324_write_reg(0x1D, 0x0C);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xDC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xC0);
		nvp6324_write_reg(0x31, 0x82);
		nvp6324_write_reg(0x38, 0x1D);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);

		// SD field option(fix to 1 for odd field, 2 for even field)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x3E, 0x11);
		nvp6324_write_reg(0x3F, 0x11);
		nvp6324_write_reg(0x3C, 0x01);

		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_CVBS_H720_NTSC] ch:%d\n", ch);
	}
        else if( pvifmt[ch] == FMT_CVBS_H720_PAL )
        {
		nvp6324_data.arb_scale |= 0x1<<(ch*2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x04 + ch, 0x05);
		nvp6324_write_reg(0x08 + ch, 0x00);
		nvp6324_write_reg(0x0C + ch, 0x00);
		nvp6324_write_reg(0x10 + ch, 0xDD);
		tmp_reg = nvp6324_read_reg(0x14 + ch);
		tmp_reg &= ~(1<<4);
		nvp6324_write_reg(0x14 + ch, tmp_reg);
		nvp6324_write_reg(0x5C + ch, 0x02);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x01);
		nvp6324_write_reg(0x84 + ch, 0x06);
		nvp6324_write_reg(0x88 + ch, 0x07);
		nvp6324_write_reg(0x8C + ch, 0x01);
		nvp6324_write_reg(0xA0 + ch, 0x10);
		nvp6324_write_reg(0x78 + ch, 0x21);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x21, 0x24);
		nvp6324_write_reg(0x47, 0x04);
		nvp6324_write_reg(0x50, 0x80);
		nvp6324_write_reg(0x53, 0x00);
		nvp6324_write_reg(0x57, 0x00);
		nvp6324_write_reg(0x62, 0x20);
		nvp6324_write_reg(0x64, 0x01);
		nvp6324_write_reg(0x69, 0x00);
		nvp6324_write_reg(0x6E, 0x00);
		nvp6324_write_reg(0x6F, 0x00);
		nvp6324_write_reg(0x90, 0x0D);
		nvp6324_write_reg(0xA9, 0x0A);
		nvp6324_write_reg(0xB5, 0x80);
		nvp6324_write_reg(0xB7, 0xFC);
		nvp6324_write_reg(0xB8, 0x39);
		nvp6324_write_reg(0xB9, 0x72);
		nvp6324_write_reg(0xBB, 0x0F);
		nvp6324_write_reg(0xFF, 0x09);
		nvp6324_write_reg(0x44 + ch, 0x00);
		nvp6324_write_reg(0x50 + (ch*4), 0x30);
		nvp6324_write_reg(0x51 + (ch*4), 0x6F);
		nvp6324_write_reg(0x52 + (ch*4), 0x67);
		nvp6324_write_reg(0x53 + (ch*4), 0x48);
		nvp6324_write_reg(0x96 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x97 + (ch*0x20), 0x10);
		nvp6324_write_reg(0x98 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x99 + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9A + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9B + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9C + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9D + (ch*0x20), 0x00);
		nvp6324_write_reg(0x9E + (ch*0x20), 0x00);
		nvp6324_write_reg(0xFF, 0x0A + (ch/2));
		nvp6324_write_reg(0x30 + ((ch%2) * 0x80), 0xAC);
		nvp6324_write_reg(0x31 + ((ch%2) * 0x80), 0x78);
		nvp6324_write_reg(0x32 + ((ch%2) * 0x80), 0x17);
		nvp6324_write_reg(0x33 + ((ch%2) * 0x80), 0xC1);
		nvp6324_write_reg(0x34 + ((ch%2) * 0x80), 0x40);
		nvp6324_write_reg(0x35 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x36 + ((ch%2) * 0x80), 0xC3);
		nvp6324_write_reg(0x37 + ((ch%2) * 0x80), 0x0A);
		nvp6324_write_reg(0x38 + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x39 + ((ch%2) * 0x80), 0x02);
		nvp6324_write_reg(0x3A + ((ch%2) * 0x80), 0x00);
		nvp6324_write_reg(0x3B + ((ch%2) * 0x80), 0xB2);
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x20 + ch, 0x00);//brightness
		nvp6324_write_reg(0x24 + ch, 0x90);//contrast
		nvp6324_write_reg(0x28 + ch, 0x90);//black_level
		nvp6324_write_reg(0x2C + ch, 0x90);//h_v_sharpness
		nvp6324_write_reg(0x30 + ch, 0x00);//y_peaking
		nvp6324_write_reg(0x34 + ch, 0x00);//y_fir
		nvp6324_write_reg(0x40 + ch, 0x00);//hue
		nvp6324_write_reg(0x44 + ch, 0x00);//u_gain
		nvp6324_write_reg(0x48 + ch, 0x00);//v_gain
		nvp6324_write_reg(0x4C + ch, 0x00);//u_offset
		nvp6324_write_reg(0x50 + ch, 0x00);//v_offset
		nvp6324_write_reg(0x58 + ch, 0x80);//saturation
		nvp6324_write_reg(0x60 + ch, 0x10);
		nvp6324_write_reg(0x64 + ch, 0x0A);//c_y_delay
		nvp6324_write_reg(0x68 + ch, 0xD8);//h_delay_a
		nvp6324_write_reg(0x6C + ch, 0x00);//h_delay_c
		nvp6324_write_reg(0x7C + ch, 0x8F);
		nvp6324_write_reg(0xFF, 0x05 + ch);
		nvp6324_write_reg(0x01, 0x2c);
		nvp6324_write_reg(0x05, 0x20);
		nvp6324_write_reg(0x1D, 0x0C);
		nvp6324_write_reg(0x24, 0x2A);
		nvp6324_write_reg(0x25, 0xCC);
		nvp6324_write_reg(0x27, 0x57);
		nvp6324_write_reg(0x28, 0x80);
		nvp6324_write_reg(0x2B, 0xC0);
		nvp6324_write_reg(0x31, 0x02);
		nvp6324_write_reg(0x38, 0x1F);
		nvp6324_write_reg(0x58, 0x77);
		nvp6324_write_reg(0x5C, 0x78);
		nvp6324_write_reg(0x5F, 0x00);
		nvp6324_write_reg(0x65, 0x00);
		nvp6324_write_reg(0xD1, 0x30);
		nvp6324_write_reg(0xD5, 0x80);
		nvp6324_write_reg(0xFF, 0x0A + ch/2);
		nvp6324_write_reg(0x25 + (ch%2 * 0x80), 0x10);
		nvp6324_write_reg(0x27 + (ch%2 * 0x80), 0x1E);
		nvp6324_write_reg(0x3C + (ch%2 * 0x80), 0x00);
		nvp6324_write_reg(0x3D + (ch%2 * 0x80), 0x00);

		// SD field option(fix to 1 for odd field, 2 for even field)
		nvp6324_write_reg(0xFF, 0x21);
		nvp6324_write_reg(0x3E, 0x11);
		nvp6324_write_reg(0x3F, 0x11);
		nvp6324_write_reg(0x3C, 0x01);

		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[FMT_CVBS_H720_PAL] ch:%d\n", ch);
        }
        else // FMT_NOT_USE
        {
		nvp6324_write_reg(0xFF, 0x00);
		nvp6324_write_reg(0x00+ch, 0x01); //Power down VAFE
		nvp6324_write_reg(0xFF, 0x01);
		tmp_reg = nvp6324_read_reg(0x95);
		tmp_reg |= 1<<ch;
		nvp6324_write_reg(0x95, tmp_reg); //Power down description2 PRE_Reset1~4
		nvp6324_write_reg(0x96, tmp_reg); //Power down description2 Post_Reset1~4
		nvp6324_write_reg(0x98, tmp_reg); //Power down description2 Powerdwon_Dec1~4
		nvp6324_write_reg(0x9C, tmp_reg); //Power down description2 Powerdwon_VADC_CLK1~4
		nvp6324_write_reg(0x9F, tmp_reg); //Power down description2 Powerdwon_TimeMultiflex1~4
		tmp_reg ^= 0x0F;
		nvp6324_write_reg(0x97, tmp_reg); //Power down description2 CH_Reset1~4
		nvp6324_data.arb_enable &= ~((1<<ch)|(1<<ch+4));
		dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 Video Input[NOT_USED] ch:%d down\n", ch);
        }
    }

    //vout
    nvp6324_write_reg(0xFF, 0x01);
    nvp6324_write_reg(0xC0, 0x00);
    nvp6324_write_reg(0xC1, 0x00);
    nvp6324_write_reg(0xC2, 0x11);
    nvp6324_write_reg(0xC3, 0x11);
    nvp6324_write_reg(0xC4, 0x22);
    nvp6324_write_reg(0xC5, 0x22);
    nvp6324_write_reg(0xC6, 0x33);
    nvp6324_write_reg(0xC7, 0x33);
    nvp6324_write_reg(0xC8, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xC9, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCA, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCB, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCC, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCD, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCE, 0x00); //Power down - Parallel_Disable
    nvp6324_write_reg(0xCF, 0x00); //Power down - Parallel_Disable

    //arb	
    if(lane == 4) //4lane
    {
	  nvp6324_write_reg(0xFF, 0x20);
	  nvp6324_write_reg(0x40, 0x01);
	  nvp6324_write_reg(0x00, 0x00);
	  nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	  nvp6324_write_reg(0x0F, 0x00);
	  nvp6324_write_reg(0x0D, 0x01);
	  nvp6324_write_reg(0x1B, 0x00);
	  nvp6324_write_reg(0x1C, 0x00);
	  nvp6324_write_reg(0x40, 0x00);
	  nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
    }
    else if(lane == 2)//2lane
    {
	  nvp6324_write_reg(0xFF, 0x20);
	  nvp6324_write_reg(0x40, 0x01);
	  nvp6324_write_reg(0x00, 0x00);
	  nvp6324_write_reg(0x01, nvp6324_data.arb_scale);
	  nvp6324_write_reg(0x0F, 0x00);
	  nvp6324_write_reg(0x0D, 0x00);
	  nvp6324_write_reg(0x1B, 0x00);
	  nvp6324_write_reg(0x1C, 0x00);
	  nvp6324_write_reg(0x40, 0x00);
	  nvp6324_write_reg(0x00, nvp6324_data.arb_enable);
    }

    //Data type - 0:YUV422 8bit(0x1E), 1:YUV420 8bit(0x18), 2:YUV420 Legacy 8bit(0x1A)
    int i = 0;	
    unsigned char dphy_data_type = 0x00;
    unsigned char arb_data_type = 0x00;
    for(i = 0; i < 4; i++)
    {		
	    if(datatype == 0)
	    {
		    dphy_data_type = 0x1E;		
	    }
	    else if(datatype == 1)
	    {
		    dphy_data_type = 0x18;
		    arb_data_type |= 0x2<<(i*2);
	    }
	    else if(datatype == 2)
	    {
		    dphy_data_type = 0x1A;
		    arb_data_type |= 0x1<<(i*2);			
	    }
	    nvp6324_write_reg(0xFF, 0x21);
	    nvp6324_write_reg(0x38+i, dphy_data_type);
	    dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 d-phy(ch:%d), data type:Bank21x%x=0x%x\n", i, 0x38+i, dphy_data_type);
    }
    nvp6324_write_reg(0xFF, 0x20);
    nvp6324_write_reg(0x0F, arb_data_type);
    dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 arbiter, data type:Bank20x0f=0x%x\n", arb_data_type);


    /*
     * For only Test
     * pattern(When it is NOVIDEO), the Decoder output pattern image to SoC(output)
     * 	    -. 1280x960 don't support to output pattern.
     */
#if 0
    /* pattern - moving cursor */
    nvp6324_write_reg(0xff, 0x00);
    nvp6324_write_reg(0x18, 0x13);
    nvp6324_write_reg(0x1c, 0x1a);
    nvp6324_write_reg(0x19, 0x13);
    nvp6324_write_reg(0x1d, 0x1b);
    nvp6324_write_reg(0x1a, 0x13);
    nvp6324_write_reg(0x1e, 0x1c);
    nvp6324_write_reg(0x1b, 0x13);
    nvp6324_write_reg(0x1f, 0x1f);
    nvp6324_write_reg(0xff, 0x05);
    nvp6324_write_reg(0x6a, 0x81);
    nvp6324_write_reg(0xff, 0x06);
    nvp6324_write_reg(0x6a, 0x81);
    nvp6324_write_reg(0xff, 0x07);
    nvp6324_write_reg(0x6a, 0x81);
    nvp6324_write_reg(0xff, 0x08);
    nvp6324_write_reg(0x6a, 0x81);
    
    dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 pattern moving cursor\n");
#endif
    /* pattern - output(normal) */
#if 0
    //100% color bar
    nvp6324_write_reg(0xff, 0x00);
    nvp6324_write_reg(0x18, 0x13);
    nvp6324_write_reg(0x1c, 0x1b);
    nvp6324_write_reg(0x19, 0x13);
    nvp6324_write_reg(0x1d, 0x1c);
    nvp6324_write_reg(0x1a, 0x13);
    nvp6324_write_reg(0x1e, 0x1c);
    nvp6324_write_reg(0x1b, 0x13);
    nvp6324_write_reg(0x1f, 0x1b);
    //BackGround Color
    nvp6324_write_reg(0xff, 0x00);
    nvp6324_write_reg(0x1c, 0x02);
    nvp6324_write_reg(0x1d, 0x02);
    nvp6324_write_reg(0x1e, 0x01);
    nvp6324_write_reg(0x1f, 0x02);
    
    dev_info(&nvp6324_data.sen.i2c_client->dev,"nvp6324 pattern normal\n");
#endif

   dev_info(&nvp6324_data.sen.i2c_client->dev, "Init complete\n");

    return 0;
}

/************************************************************************
* End of file
*************************************************************************/

int nvp6324_video_init(void)
{
	int ret = 0;
	
	uint16_t curr_mod;
	uint16_t curr_height;
	uint16_t curr_width;
	uint16_t curr_fps;
	
	
	nvp6324_data.sen.streamcap.capability = 0;
	nvp6324_data.sen.streamcap.capturemode = 0;
	nvp6324_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;    /* YUV422 */
	nvp6324_data.std_id = V4L2_STD_ALL; 
	nvp6324_data.sen.mipi_camera = 1;
	
	nvp6324_data.curr_mod = FMT_CVBS_H720_PAL;
	
	switch(nvp6324_data.curr_mod)
	{
	    case FMT_CVBS_H720_PAL:		  
	      nvp6324_data.sen.pix.width = 720;
	      nvp6324_data.sen.pix.height = 480;
	      nvp6324_data.sen.spix.swidth = 720;
	      nvp6324_data.sen.spix.sheight = 480;
	      nvp6324_data.sen.streamcap.timeperframe.denominator = 25;
	      nvp6324_data.sen.streamcap.timeperframe.numerator = 1;
	      nvp6324_data.sen.spix.top = 0;		
	      nvp6324_data.sen.spix.left = 0;
	      ipu_csi_set_interlaced_mode(1);
	      dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video mode = FMT_CVBS_H720_PAL\n");
	      break;
	    case FMT_AHD_720P_30P:		  
	      nvp6324_data.sen.pix.width = 1280;
	      nvp6324_data.sen.pix.height = 720;
	      nvp6324_data.sen.spix.swidth = 1280;
	      nvp6324_data.sen.spix.sheight = 720;
	      nvp6324_data.sen.streamcap.timeperframe.denominator = 30;
	      nvp6324_data.sen.streamcap.timeperframe.numerator = 1;
	      nvp6324_data.sen.spix.top = 0;		
	      nvp6324_data.sen.spix.left = 0;
	      ipu_csi_set_interlaced_mode(0);
	      dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video mode = FMT_AHD_720P_30P\n");
	      break;
	     case FMT_AHD_1080P_30P:		  
	      nvp6324_data.sen.pix.width = 1920;
	      nvp6324_data.sen.pix.height = 1080;
	      nvp6324_data.sen.spix.swidth = 1920;
	      nvp6324_data.sen.spix.sheight = 1080;
	      nvp6324_data.sen.streamcap.timeperframe.denominator = 30;
	      nvp6324_data.sen.streamcap.timeperframe.numerator = 1;
	      nvp6324_data.sen.spix.top = 0;		
	      nvp6324_data.sen.spix.left = 0;
	      ipu_csi_set_interlaced_mode(0);
	      dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video mode = FMT_AHD_1080P_30P\n");
	      break;
	      case FMT_AHD_1080P_25P:		  
	      nvp6324_data.sen.pix.width = 1920;
	      nvp6324_data.sen.pix.height = 1080;
	      nvp6324_data.sen.spix.swidth = 1920;
	      nvp6324_data.sen.spix.sheight = 1080;
	      nvp6324_data.sen.streamcap.timeperframe.denominator = 25;
	      nvp6324_data.sen.streamcap.timeperframe.numerator = 1;
	      nvp6324_data.sen.spix.top = 0;		
	      nvp6324_data.sen.spix.left = 0;
	      ipu_csi_set_interlaced_mode(0);
	      dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video mode = FMT_AHD_1080P_30P\n");
	      break;
	     default:
	        dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 error WRONG VIDEO INPUT TYPE!\n");
	      break;	     
	}

	nvp6324_data.curr_width = nvp6324_data.sen.pix.width;
	nvp6324_data.curr_height = nvp6324_data.sen.pix.height;
	nvp6324_data.curr_fps = nvp6324_data.sen.streamcap.timeperframe.denominator;	
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324_data.curr_width = %d\n", nvp6324_data.curr_width);
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324_data.curr_height = %d\n", nvp6324_data.curr_height);
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324_data.curr_fps = %d\n", nvp6324_data.curr_fps);
	
	uint8_t vifmt[4] = {0, 0, nvp6324_data.curr_mod, 0};
	
	nvp6324_data.lanes = MIPI_LANES_4;
	nvp6324_data.pixformat = YUV422_TYPE;
	nvp6324_data.pclk = PCLK_756MHZ;
	nvp6324_data.arb_scale = 0;
        nvp6324_data.arb_enable = 0xFF;
    
	nvp6324_chip_initialize(vifmt, MIPI_LANES_4, PCLK_756MHZ, YUV422_TYPE);
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video init done\n");

	return ret;
}

static int nvp6324_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	
	int ret;
	
	dev_info(dev, "Start nvp6324 probe...\n");
	
	memset(&nvp6324_data, 0, sizeof(nvp6324_data));
	nvp6324_data.sen.i2c_client = client;
	nvp6324_data.sen.on = true;
	nvp6324_data.sen.csi = 0;
	nvp6324_data.sen.mipi_camera = 1;
	

	nvp6324_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(nvp6324_data.sen.sensor_clk)) {
		nvp6324_data.sen.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(nvp6324_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
				   &(nvp6324_data.sen.mclk));
	if (ret) {
		dev_err(dev, "mclk missing or invalid\n");
		return ret;
	}
    dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324_data.sen.mclk=%d from devicetree.\n", nvp6324_data.sen.mclk);

	ret = of_property_read_u32(dev->of_node, "mclk_source",
				   (u32*)&(nvp6324_data.sen.sensor_clk));
	if (ret) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return ret;
	}
	
	pr_info("Nvp6324 check ipu\n");
    	ret = of_property_read_u32(dev->of_node, "ipu_id", &nvp6324_data.sen.ipu_id);
	if (ret) {
		dev_err(dev, "ipu_id missing or invalid\n");
		return ret;
	}
    	else dev_info(&nvp6324_data.sen.i2c_client->dev, "Read ipu_id=%d from devicetree.\n", nvp6324_data.sen.ipu_id);

	pr_info("Nvp6324 check csi\n");
    	ret = of_property_read_u32(dev->of_node, "csi_id", &nvp6324_data.sen.csi);
	if (ret) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}
    	else dev_info(&nvp6324_data.sen.i2c_client->dev, "Read csi_id=%d from devicetree.\n", nvp6324_data.sen.csi);

	clk_prepare_enable(nvp6324_data.sen.sensor_clk);
	
	nvp6324_write_reg(0xFF, 0x00);
	
	int rev = nvp6324_read_reg(0xf4);
	
	if(rev != 0xB0){
	  dev_info(dev, "nvp6324 not found! error!", client->addr);
	  return -1;
	}
	
	dev_info(dev, "nvp6324 found, address: 0x%02x, chip id: 0xb0", client->addr);	
	
	nvp6324_int_device.priv = &nvp6324_data;
	
	nvp6324_video_init();
	
	ret = v4l2_int_device_register(&nvp6324_int_device);
	pr_debug("   v4l2 device created, status is %d\n", ret);
	
	pr_debug("%s: init done, ret=%d\n", __func__,ret);

	return 0;
}

static int nvp6324_remove(struct i2c_client *client)
{	
	void *mipi_csi2_info;

	dev_dbg(&nvp6324_data.sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, "nvp6324", client->addr << 1, client->adapter->name);
	
	v4l2_int_device_unregister(&nvp6324_int_device);
	
 	/* disable mipi csi2 */
    	mipi_csi2_info = mipi_csi2_get_info();
    	if (mipi_csi2_info)
        if (mipi_csi2_get_status(mipi_csi2_info)){
            mipi_csi2_disable(mipi_csi2_info);
	    pr_info("mipi csi2 disabled\n");	
	}	

	

	return 0;
}

static const struct i2c_device_id nvp6324_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, nvp6324_id);

static const struct of_device_id nvp6324_of_match[] = {
	{ .compatible = "nextchip,nvp6324_mipi" },
	{ /* sentinel */ }
};

static struct i2c_driver nvp6324_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvp6324_mipi",
		.of_match_table = of_match_ptr(nvp6324_of_match),
	},
	.probe = nvp6324_probe,
	.remove = nvp6324_remove,
	.id_table = nvp6324_id,
};

module_i2c_driver(nvp6324_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("NVP6324 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");


static int nvp6324_init_csi(void)      
{
	void *mipi_csi2_info;
	u32 mipi_reg;     
		
	nvp6324_chanState(0, 2);
	
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return;
	}
  
	mipi_csi2_set_lanes(mipi_csi2_info, 4);  /////     
	
	mipi_csi2_reset_ahd(mipi_csi2_info);

 	mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
	
	nvp6324_chanState(1, 2);
	
	msleep(800);//wait stable
	if (mipi_csi2_info) {
		unsigned int i = 0;
		/* wait for mipi sensor ready */
		while (1) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			if (mipi_reg == 0x300){
				break;
			}
			if (i++ >= 200) {
				dev_info(&nvp6324_data.sen.i2c_client->dev,"mipi csi2 can not receive sensor clk! %x\n", mipi_reg);			
				return -1;				
			}
			msleep(10);
		}
		dev_info(&nvp6324_data.sen.i2c_client->dev,"dphy status is: %x\n", mipi_reg);
		
		i = 0;
		/* wait for mipi stable */
		while (1) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			if (!mipi_reg)
				break;
			if (i++ >= 20) {
				dev_info(&nvp6324_data.sen.i2c_client->dev,"mipi csi2 can not receive data correctly! %x\n", mipi_reg);					
				return -1;				
			}
			msleep(10);
		}	
		dev_info(&nvp6324_data.sen.i2c_client->dev,"mipi csi2 error status is: %x\n", mipi_reg);
	}
	
	uint8_t regD = 0;
	
	nvp6324_write_reg(0xFF, 0x00);
	
	regD = nvp6324_read_reg(0xA4);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg No Vid1: %x\n", regD);	
	regD = nvp6324_read_reg(0xA5);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg No Vid2: %x\n", regD);	
	regD = nvp6324_read_reg(0xA6);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg No Vid3: %x\n", regD);	
	regD = nvp6324_read_reg(0xA7);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg No Vid4: %x\n", regD);	

	regD = nvp6324_read_reg(0xD0);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Agc lock1: %x\n", regD);	
	regD = nvp6324_read_reg(0xD1);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Agc lock2: %x\n", regD);	
	regD = nvp6324_read_reg(0xD2);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Agc lock3: %x\n", regD);	
	regD = nvp6324_read_reg(0xD3);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Agc lock4: %x\n", regD);	
	
	regD = nvp6324_read_reg(0xD4);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Clamp lock1: %x\n", regD);	
	regD = nvp6324_read_reg(0xD5);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Clamp lock2: %x\n", regD);	
	regD = nvp6324_read_reg(0xD6);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Clamp lock3: %x\n", regD);	
	regD = nvp6324_read_reg( 0xD7);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Clamp lock4: %x\n", regD);

	regD = nvp6324_read_reg(0xD8);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Hlock lock1: %x\n", regD);	
	regD = nvp6324_read_reg(0xD9);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Hlock lock2: %x\n", regD);	
	regD = nvp6324_read_reg(0xDA);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Hlock lock3: %x\n", regD);	
	regD = nvp6324_read_reg(0xDB);
	dev_info(&nvp6324_data.sen.i2c_client->dev,"Reg Hlock lock4: %x\n", regD);	
	
	return 0;
}


///////////////////////////////////////////V4L//////////////////////////////////////////


/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
#if 1
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
#else
    p->if_type = V4L2_IF_TYPE_BT656_INTERLACED;
    p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT;
#endif
	p->u.bt656.clock_curr = nvp6324_data.sen.mclk;  //BT656 interlace clock mode 1 - prorgressive 0 - interlaced
	pr_debug("   clock_curr=mclk=%d\n", nvp6324_data.sen.mclk);
	p->u.bt656.clock_min = 27000000;
	p->u.bt656.clock_max  = 27000000;
    //p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ifparm clock = %d\n" , p->u.bt656.clock_curr);	

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for videoc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {		
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 set power on\n");	
	} else if (!on && sensor->on) {
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 set power off\n");	
	}
	sensor->on = on;
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 V4L2_BUF_TYPE_VIDEO_CAPTURE ioctl_g_parm\n");	
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 cparm->capability = %d cparm->timeperframe.denominator = %d cparm->timeperframe.numerator = %d cparm->capturemode = %d\n",
					cparm->capability , cparm->timeperframe.denominator, cparm->timeperframe.numerator , cparm->capturemode);	
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */

static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_parm\n");	
	

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:  
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */

static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_fmt_cap\n");	

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 V4L2_BUF_TYPE_VIDEO_CAPTURE\n");	
		dev_info(&nvp6324_data.sen.i2c_client->dev, " Returning pix of %dx%d\n",sensor->sen.pix.width, sensor->sen.pix.height);		
		f->fmt.pix = sensor->sen.pix;
		f->fmt.pix.priv = 1;
		break;
	
	case V4L2_BUF_TYPE_SENSOR:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 V4L2_BUF_TYPE_SENSOR\n");	
		dev_info(&nvp6324_data.sen.i2c_client->dev, " Returning spix of %dx%d\n",sensor->sen.pix.width, sensor->sen.pix.height);	
		f->fmt.spix = sensor->sen.spix;
		break;
	case V4L2_BUF_TYPE_PRIVATE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 V4L2_BUF_TYPE_PRIVATE\n");	
		v4l2_std_id std = V4L2_STD_PAL;	
		f->fmt.pix.pixelformat = (u32)V4L2_STD_PAL;
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */

static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl brightness\n");	
		break;
	case V4L2_CID_CONTRAST:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl contrast\n");	
		break;
	case V4L2_CID_SATURATION:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl saturation\n");	
		break;
	case V4L2_CID_HUE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid hue\n");		
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl auto white balance\n");	
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl do white brightness\n");	
		break;
	case V4L2_CID_RED_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid red balance\n");	
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid blue balance\n");	
		break;
	case V4L2_CID_GAMMA:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid gamma\n");	
		break;
	case V4L2_CID_EXPOSURE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid exposure\n");	
		break;
	case V4L2_CID_AUTOGAIN:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl autogain\n");	
		break;
	case V4L2_CID_GAIN:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid gain\n");	
		break;
	case V4L2_CID_HFLIP:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid h flip\n");	
		break;
	case V4L2_CID_VFLIP:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl cid v flip\n");	
		break;
	default:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_ctrl default\n");	
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */

static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl brightness\n");	
		break;
	case V4L2_CID_CONTRAST:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl contrast\n");	
		break;
	case V4L2_CID_SATURATION:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl saturation\n");	
		break;
	case V4L2_CID_HUE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid hue\n");	
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl auto white balance\n");	
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl do white balance\n");	
		break;
	case V4L2_CID_RED_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl red balance\n");	
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl blue balance\n");	
		break;
	case V4L2_CID_GAMMA:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid gamma\n");	
		break;
	case V4L2_CID_EXPOSURE:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid exposure\n");	
		break;
	case V4L2_CID_AUTOGAIN:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid autogain\n");	
		break;
	case V4L2_CID_GAIN:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid gain\n");	
		break;
	case V4L2_CID_HFLIP:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid h flip\n");	
		break;
	case V4L2_CID_VFLIP:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl cid v flip\n");	
		break;
	default:
		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_s_ctrl default\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */

static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{	
	
	fsize->discrete.width = nvp6324_data.curr_width;
	fsize->discrete.height  = nvp6324_data.curr_height;
	  
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_enum_framesizes w = %d h = %d\n", fsize->discrete.width, fsize->discrete.height);	

	
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_enum_frameintervals\n");	

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
        fival->discrete.denominator = nvp6324_data.curr_fps;
      
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */

static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{	
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_g_chip_ident\n");	

	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"nvp6324-m_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_NVP6324;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */

static int ioctl_init(struct v4l2_int_device *s)
{	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "ioctl_init\n");
	
	return 0;
}


/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *io
 * Initialise the device when slave attaches to the master.
 */

static int ioctl_dev_init(struct v4l2_int_device *s)
{		
	dev_info(&nvp6324_data.sen.i2c_client->dev, "ioctl_dev_init\n");
	int status = nvp6324_init_csi();
	return status;
}


static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;
	dev_info(&nvp6324_data.sen.i2c_client->dev, "ioctl_dev_exit\n");
	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}


///////////////////////////////////////////V4L//////////////////////////////////////////


