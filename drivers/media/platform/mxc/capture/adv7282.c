/*
 * Copyright 2016 Centralp Automatismes Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file adv7282.c
 *
 * @brief Analog Device ADV7282-m video decoder functions
 *
 * @ingroup Camera
 */

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

// Add DEBUG traces
#define EXTRA_TRACE_DEBUG 1
    
#ifdef EXTRA_TRACE_DEBUG
#undef dev_dbg
#define dev_dbg(dev, format, arg...) {dev_printk(KERN_ERR, dev, format, ##arg);}
#undef pr_debug
#define pr_debug(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#endif

#define ADV7282_VOLTAGE_ANALOG               1800000
#define ADV7282_VOLTAGE_DIGITAL_CORE         1800000
#define ADV7282_VOLTAGE_DIGITAL_IO           3300000
#define ADV7282_VOLTAGE_PLL                  1800000

#define ADV7282_DEFAULT_CSI_I2C_ADDR 0x44
#define ADV7282_DEFAULT_VPP_I2C_ADDR 0x42

#define ADV7282_XCLK_MIN 6000000
#define ADV7282_XCLK_MAX 24000000
#define INIT_FLAG 1
#define WORK_FLAG 0
static int adv7282_downloadFirmware(void);
static void adv7282_initDev(uint8_t flag);
static int adv7282_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int adv7282_detach(struct i2c_client *client);

static const struct i2c_device_id adv7282_id[] = {
	{"adv7282", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7282_id);

static struct i2c_driver adv7282_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adv7282",
		   },
	.probe = adv7282_probe,
	.remove = adv7282_detach,
	.id_table = adv7282_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data sen;
	v4l2_std_id        std_id;
    int                rev_id;
   	struct i2c_client  *csi_client;
	struct i2c_client  *vpp_client;
	int isConf;
	int isAttached;
} adv7282_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	ADV7282_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	ADV7282_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	ADV7282_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define ADV7282_STD_MAX		(ADV7282_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	u16 active_top;		/*!< Active top. */
	u16 active_left;	/*!< Active left. */
	int frame_rate;		/*!< Frame rate. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {     
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,		/* SENS_FRM_WIDTH */
	 .raw_height = 525,		/* SENS_FRM_HEIGHT */
	 .active_width = 720,		/* ACT_FRM_WIDTH */
	 .active_height = 480,		/* ACT_FRM_HEIGHT */
	 .active_top = 0,
	 .active_left = 0,
	.frame_rate = 30,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,     
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .active_top = 0,
	 .active_left = 0,
	.frame_rate = 25,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .active_top = 0,
	 .active_left = 0,
	.frame_rate = 25,
	 },
};

/*!* Standard index of ADV7282. */
static video_fmt_idx video_idx = ADV7282_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME                    "adv7282"
#define ADV7282_INPUT_CTL              0x00	/* Input Control */
#define ADV7282_STATUS_1               0x10	/* Status #1 */
#define ADV7282_BRIGHTNESS             0x0a	/* Brightness */
#define ADV7282_IDENT                  0x11	/* IDENT */
#define ADV7282_VSYNC_FIELD_CTL_1      0x31	/* VSYNC Field Control #1 */
#define ADV7282_MANUAL_WIN_CTL         0x3d	/* Manual Window Control */
#define ADV7282_SD_SATURATION_CB       0xe3	/* SD Saturation Cb */
#define ADV7282_SD_SATURATION_CR       0xe4	/* SD Saturation Cr */
#define ADV7282_PWR_MNG                0x0f /* Power Management */
#define ADV7282_STATUS1                0x10
#define ADV7282_STATUS2                0x12
#define ADV7282_STATUS3                0x13

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl adv7282_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}
};

/***********************************************************************
 * I2C transfer.
 ***********************************************************************/

static inline int adv7282_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(adv7282_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"%s:read reg error: reg=0x%02x\n", __func__, reg);
		return -1;
	}
	return val;
}

static int adv7282_write_reg(u8 reg, u8 val)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(adv7282_data.sen.i2c_client, reg, val);
	if (ret < 0) {
	    dev_dbg(&adv7282_data.sen.i2c_client->dev,
      	        "%s:write reg error:reg=0x%02x,val=0x%02x\n", __func__,reg, val);
		return -1;
	}
	return 0;
}

/* Added functions by Centralp */
static int adv7282_csi_write(unsigned int reg, unsigned int value)
{
	return i2c_smbus_write_byte_data(adv7282_data.csi_client, reg, value);
}
static inline int adv7282_csi_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(adv7282_data.csi_client, reg);
	if (val < 0) {
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"%s:csi_read reg error: reg=0x%02x\n", __func__, reg);
		return -1;
	}
	return val;
}

static int adv7282_vpp_write(unsigned int reg, unsigned int value)
{
	return i2c_smbus_write_byte_data(adv7282_data.vpp_client, reg, value);
}
static inline int adv7282_vpp_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(adv7282_data.vpp_client, reg);
	if (val < 0) {
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"%s:vpp_read reg error: reg=0x%02x\n", __func__, reg);
		return -1;
	}
	return val;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void adv7282_get_std(v4l2_std_id *std)
{
	int status_1, standard, idx;
	bool locked;

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282_get_std\n");

	status_1 = adv7282_read(ADV7282_STATUS_1);
	locked = status_1 & 0x1;
	standard = status_1 & 0x70;

    	pr_debug("adv7282_get_std: status1=0x%02X, locked=%d, standard=0x%02X\n",status_1, locked, 	standard );

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = ADV7282_NOT_LOCKED;

	if (standard == 0x40) {
		/* PAL */
		*std = V4L2_STD_PAL;
		idx = ADV7282_PAL;
		pr_debug("adv7282_get_std: set ADV7282_PAL\n");
	} else if (standard == 0) {
		/*NTSC*/
		*std = V4L2_STD_NTSC;     
		idx = ADV7282_NTSC;
		pr_debug("adv7282_get_std: set ADV7282_NTSC\n");
	} else {
		*std = V4L2_STD_ALL;
		idx = ADV7282_NOT_LOCKED;
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"Got invalid video standard!\n");
	}

	mutex_unlock(&mutex);
	
    /* This assumes autodetect which this device uses. */
	if (*std != adv7282_data.std_id) {
		video_idx = idx;
		adv7282_data.std_id = *std;
		adv7282_data.sen.pix.width = video_fmts[idx].raw_width;
		adv7282_data.sen.pix.height = video_fmts[idx].raw_height;
        	adv7282_data.sen.spix.swidth = video_fmts[idx].raw_width;
		adv7282_data.sen.spix.sheight = video_fmts[idx].raw_height;
		adv7282_data.sen.spix.top = video_fmts[idx].active_top;		
		adv7282_data.sen.spix.left = video_fmts[idx].active_left;
        	pr_debug("adv7282_get_std: set video_idx=%d\n", idx);
	} 


}

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
	dev_dbg(&adv7282_data.sen.i2c_client->dev, "adv7282:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_curr = 24000000;  //BT656 interlace clock mode 1 - prorgressive 0 - interlaced
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

static void adv7282_power(int flag)
{
	if(flag){	
		
	}
	else{			

	}
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {		
		adv7282_power(1);		
	} else if (!on && sensor->on) {
		adv7282_power(0);	
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

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
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
	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282:ioctl_s_parm\n");
	

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

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "adv7282:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		f->fmt.pix.priv = 1;
		break;
	
	case V4L2_BUF_TYPE_SENSOR:
		pr_debug("BUF TYPE SENSOR\n");
		f->fmt.spix = adv7282_data.sen.spix;
		break;
	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		adv7282_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "adv7282:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(adv7282_qctrl); i++)
		if (qc->id && qc->id == adv7282_qctrl[i].id) {
			memcpy(qc, &(adv7282_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
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
	int sat = 0;

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		adv7282_data.sen.brightness = adv7282_read(ADV7282_BRIGHTNESS);
		vc->value = adv7282_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		vc->value = adv7282_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		sat = adv7282_read(ADV7282_SD_SATURATION_CB);
		adv7282_data.sen.saturation = sat;
		vc->value = adv7282_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		vc->value = adv7282_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		vc->value = adv7282_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = adv7282_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = adv7282_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   Default case\n");
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
	u8 tmp;

	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		adv7282_write_reg(ADV7282_BRIGHTNESS, tmp);
		adv7282_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		adv7282_write_reg(ADV7282_SD_SATURATION_CB, tmp);
		adv7282_write_reg(ADV7282_SD_SATURATION_CR, tmp);
		adv7282_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&adv7282_data.sen.i2c_client->dev,
			"   Default case\n");
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
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

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
	video_fmt_t fmt;
	int i;

	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
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
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"adv7282-m_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_ADV7282;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&adv7282_data.sen.i2c_client->dev, "In adv7282:ioctl_init\n");
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
	void *mipi_csi2_info;
	adv7282_initDev(INIT_FLAG);
	return 0;
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;
	//pr_info("IOCTL DEV EXIT!%s:\n", __func__);
	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc adv7282_ioctl_desc[] = {

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
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},
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

static struct v4l2_int_slave adv7282_slave = {
	.ioctls = adv7282_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7282_ioctl_desc),
};

static struct v4l2_int_device adv7282_int_device = {
	.module = THIS_MODULE,
	.name = "adv7282",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &adv7282_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

// Debug function
static void adv7282_reg_status(void)
{
   dev_dbg(&adv7282_data.sen.i2c_client->dev,"ADV7282: Dbg: STATUS1=0x%02X\n",adv7282_read(ADV7282_STATUS1) );
   dev_dbg(&adv7282_data.sen.i2c_client->dev,"ADV7282: Dbg: STATUS2=0x%02X\n",adv7282_read(ADV7282_STATUS2) );
   dev_dbg(&adv7282_data.sen.i2c_client->dev,"ADV7282: Dbg: STATUS3=0x%02X\n",adv7282_read(ADV7282_STATUS3) );
}

static void adv7282_csiTxState(uint8_t flag)
{	
	int ret = 0;
	if(adv7282_data.isConf){
		 if(flag)
		 {
			pr_debug("==== csi_tx before start =0x%02x\n", adv7282_csi_read(0x00));
			ret = adv7282_csi_write(0x00,0x00);
			pr_debug("==== csi_tx after start =0x%02x\n", adv7282_csi_read(0x00));
			msleep(100);
		 }
		 else if(!flag)
		{
			pr_debug("==== csi_tx before stop =0x%02x\n", adv7282_csi_read(0x00));
			ret = adv7282_csi_write(0x00,0x80);
			pr_debug("==== csi_tx after stop =0x%02x\n", adv7282_csi_read(0x00));
			msleep(100);
		}
	}
	return 0;
}

static void adv7282_initDev(uint8_t flag)      
{
	void *mipi_csi2_info;
	u32 mipi_reg;     
	int ret = 0;
	
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	adv7282_downloadFirmware();

	adv7282_csiTxState(0);

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	mipi_csi2_set_lanes(mipi_csi2_info, 1);

	if(flag == INIT_FLAG )
		mipi_csi2_reset_pal(mipi_csi2_info);

 	mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);

	adv7282_csiTxState(1);

	msleep(400);//wait stable
	if (mipi_csi2_info) {
		unsigned int i = 0;
		/* wait for mipi sensor ready */
		while (1) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			if (mipi_reg != 0x200)
				break;
			if (i++ >= 20) {
				pr_err("mipi csi2 can not receive sensor clk! %x\n", mipi_reg);
				return -1;
			}
			msleep(10);
		}
		pr_debug("DPHY STATUS IS: %x\n", mipi_reg);
		
		i = 0;
		/* wait for mipi stable */
		while (1) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			if (!mipi_reg)
				break;
			if (i++ >= 20) {
				pr_err("mipi csi2 can not receive data correctly!\n");
				return -1;
			}
			msleep(10);
		}
		pr_debug("MIPI CSI2 ERROR STATUS IS: %x\n", mipi_reg);
	}
	return;
}

static void adv7282_hard_reset(void)
{
	///My autodetect progressive config
	adv7282_write_reg(0x0F,0x00); //Exit Power Down Mode
	adv7282_write_reg(0x0e,0x00); //ADI Required Write 
	adv7282_write_reg(0x00,0x00); //Force Free-run mode 
	adv7282_write_reg(0x0e,0x80); //Force standard to PAL
	adv7282_write_reg(0x9c,0x00); //Set Free-run pattern to color bars 
	adv7282_write_reg(0x9c,0xFF); //Set Free-run pattern to color bars 
	adv7282_write_reg(0x0e,0x00); //ADI Required Write 
	adv7282_write_reg(0x03,0x4e); //Power-up INTRQ pin
	adv7282_write_reg(0x04,0x57); //Enable INTRQ output driver
	adv7282_write_reg(0x13,0x00); // select SH1
	adv7282_write_reg(0x17,0x41); // select SH1
	adv7282_write_reg(0x1D,0xC0); //Tri-State LLC output driver
	adv7282_write_reg(0x52,0xCD); //ADI Required Write
	adv7282_write_reg(0x80,0x51); //ADI Required Write     
	adv7282_write_reg(0x81,0x51); //ADI Required Write
	adv7282_write_reg(0x82,0x68); //ADI Required Write
	adv7282_write_reg(0xFD,0x84); //Set VPP Map Address
	adv7282_write_reg(0xFE,0x88); //Set CSI Map Address     
}


static int adv7282_downloadFirmware(void)
{
	adv7282_hard_reset();	
	adv7282_csi_write(0x0D,0x80);  // Program Virtual Channel #1 under ADV7282-m
	pr_debug("VC_REF=0x%02x\n", adv7282_csi_read(0x0D)); // Display Virtual Channel #1 register 
	pr_info("csi config done progressive\n");
	adv7282_csi_write(0x01,0x20);
	adv7282_csi_write(0x02,0x28);
	adv7282_csi_write(0x03,0x38);
	adv7282_csi_write(0x04,0x30);
	adv7282_csi_write(0x05,0x30);
	adv7282_csi_write(0x06,0x80); 
	adv7282_csi_write(0x07,0x70); 
	adv7282_csi_write(0x08,0x50);
	 
	adv7282_csi_write(0xDE,0x02); 
	adv7282_csi_write(0xD2,0xF7); 
	adv7282_csi_write(0xD8,0x65); 
	adv7282_csi_write(0xE0,0x09); 
	adv7282_csi_write(0x2C,0x00); 
	adv7282_csi_write(0x1D,0x80); 
	pr_debug("==== csi_tx at config is =0x%02x\n", adv7282_csi_read(0x00));
	
	adv7282_data.isConf = 1;

	adv7282_vpp_write(0xA3,0x00); //ADI Required Write
	adv7282_vpp_write(0x5B,0x00); //I2C_DEINT_ENABLE : Advanced Timing Enabled
	adv7282_vpp_write(0x55,0x80); //ADV_TIMING_ MODE_EN : Enable I2P / Enable the Deinterlacer for I2P
	pr_info("vpp config done progressive\n"); 	
	v4l2_std_id std;
	adv7282_get_std(&std);
}


static int my_open(struct inode *i, struct file *f)
{
	pr_info("ADV7282 open\n");
    return 0;
}
static int my_close(struct inode *i, struct file *f)
{
	pr_info("ADV7282 closed\n");
    return 0;
}

static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{   
 
	switch (cmd)
    {
        case 0xCA:
	    if(adv7282_data.isAttached)	{
	    	v4l2_int_device_unregister(&adv7282_int_device);
	   	 pr_info("ADV7282 detached\n");
		adv7282_data.isAttached = 0;
		}
            break;	
        case 0xCB:	
	   if(!adv7282_data.isAttached)
	   {
	    v4l2_int_device_register(&adv7282_int_device);
	    pr_info("ADV7282 attached\n");
	     adv7282_data.isAttached = 1;
	   }
            break; 	
	case 0xCC:
	    break;    
    }
 
    return 0;
}

static struct file_operations Fops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_close,
	.unlocked_ioctl = my_ioctl
};


/*! ADV7282 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * ADV7282 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */                
static int adv7282_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	u32 cvbs = true;
	void *mipi_csi2_info;
	u32 mipi_reg;
	struct device *dev = &client->dev;

	dev_info(dev, "Start adv7282 probe...\n");

	pr_info("power down\n");

	memset(&adv7282_data, 0, sizeof(adv7282_data));
	adv7282_data.sen.i2c_client = client;
	adv7282_data.sen.streamcap.timeperframe.denominator = 30;
	adv7282_data.sen.streamcap.timeperframe.numerator = 1;
	adv7282_data.std_id = V4L2_STD_ALL;
    	video_idx = ADV7282_NOT_LOCKED;
	adv7282_data.sen.pix.width = video_fmts[video_idx].raw_width;
	adv7282_data.sen.pix.height = video_fmts[video_idx].raw_height;
	adv7282_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;    /* YUV422 */
	adv7282_data.sen.spix.swidth = video_fmts[video_idx].raw_width - 1;
	adv7282_data.sen.spix.sheight = video_fmts[video_idx].raw_height;
	adv7282_data.sen.spix.top = video_fmts[video_idx].active_top;		
	adv7282_data.sen.spix.left = video_fmts[video_idx].active_left;

	adv7282_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	adv7282_data.sen.on = true;
   	adv7282_data.sen.csi = 0;         
    	pr_info("reset i2c agent\n");
    	adv7282_write_reg(ADV7282_PWR_MNG, 0x80); // Reset i2c agent
    	msleep(400);
   	adv7282_write_reg(ADV7282_PWR_MNG, 0x0);  // reset mode : off
	pr_info("reset off\n");
	adv7282_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(adv7282_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(adv7282_data.sen.sensor_clk);
	}

	pr_info("check ipu\n");
    	ret = of_property_read_u32(dev->of_node, "ipu_id", &adv7282_data.sen.ipu_id);
	if (ret) {
		dev_err(dev, "ipu_id missing or invalid\n");
		return ret;
	}
    	else dev_dbg(&adv7282_data.sen.i2c_client->dev, "Read ipu_id=%d from devicetree.\n", 			adv7282_data.sen.ipu_id);

	pr_info("check csi\n");
    	ret = of_property_read_u32(dev->of_node, "csi_id", &adv7282_data.sen.csi);
	if (ret) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}
    	else dev_dbg(&adv7282_data.sen.i2c_client->dev, "Read csi_id=%d from devicetree.\n", 							adv7282_data.sen.csi);

	dev_dbg(&adv7282_data.sen.i2c_client->dev,
		"%s:adv7282 probe i2c address is 0x%02X\n",
		__func__, adv7282_data.sen.i2c_client->addr);

	adv7282_data.rev_id = adv7282_read(ADV7282_IDENT);
	dev_dbg(&adv7282_data.sen.i2c_client->dev,
		"%s:ADV7282_IDENT=%02X\n", __func__, (unsigned char)adv7282_data.rev_id);
	
	
	/* ADV7282 is always MIPI */
	adv7282_data.sen.mipi_camera = 1; 
	
	clk_prepare_enable(adv7282_data.sen.sensor_clk);
	
    	// Create CSI client @44h
    	adv7282_data.csi_client = i2c_new_dummy(client->adapter, ADV7282_DEFAULT_CSI_I2C_ADDR);
	if (!adv7282_data.csi_client){
		i2c_unregister_device(adv7282_data.csi_client);
		return -ENODEV;
	}

	pr_info("create vpp client\n");
    	// Create VPP client @42h
    	adv7282_data.vpp_client = i2c_new_dummy(client->adapter, ADV7282_DEFAULT_VPP_I2C_ADDR);
	if (!adv7282_data.vpp_client){
       		i2c_unregister_device(adv7282_data.vpp_client);
       		return -ENODEV;
    	}

	adv7282_downloadFirmware(); //Download once , it wont change

	adv7282_power(0);

	pr_debug("   type is %d (expect %d)\n",
		 adv7282_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		 adv7282_int_device.u.slave->num_ioctls);
	pr_info("print reg status\n");

	ret = register_chrdev(200, 
		         "adv7282",
		         &Fops);
	pr_debug("   FOPS, status is %d\n", ret);
	adv7282_int_device.priv = &adv7282_data;
	//ret = v4l2_int_device_register(&adv7282_int_device);
	//pr_debug("   v4l2 device created, status is %d\n", ret);
	pr_info("v4l2 dev created\n");
   
	pr_debug("%s: init done, ret=%d\n", __func__,ret);

    	return ret;
}



/*!
 * ADV7282 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7282_detach(struct i2c_client *client)
{
    	void *mipi_csi2_info;

	dev_dbg(&adv7282_data.sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

    	i2c_unregister_device(adv7282_data.csi_client);
    	i2c_unregister_device(adv7282_data.vpp_client);
	if(adv7282_data.isAttached)
		v4l2_int_device_unregister(&adv7282_int_device);

	 /* disable mipi csi2 */
    	mipi_csi2_info = mipi_csi2_get_info();
    	if (mipi_csi2_info)
        if (mipi_csi2_get_status(mipi_csi2_info))
            mipi_csi2_disable(mipi_csi2_info);

	pr_info("MIPI CSI2 DISABLED\n");
	return 0;
}

/*!
 * ADV7282 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int adv7282_init(void)
{
	u8 err = 0;
	
    	pr_debug("In adv7282_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&adv7282_i2c_driver);
	if (err != 0)
	pr_err("%s:driver registration failed, error=%d \n",__func__, err);

     	pr_debug("adv7282_init : done, err=%d\n", err);
    	 return err;
}

/*!
 * ADV7282 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit adv7282_clean(void)
{
	dev_dbg(&adv7282_data.sen.i2c_client->dev, " adv7282_clean\n");
	i2c_del_driver(&adv7282_i2c_driver);
}

module_init(adv7282_init);
module_exit(adv7282_clean);

MODULE_AUTHOR("IgorKurn");
MODULE_DESCRIPTION("Adv7282 suchara");
MODULE_LICENSE("GPL");

