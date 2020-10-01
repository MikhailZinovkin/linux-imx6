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

int nvp6324_video_init(void);
static void nvp6324_hardware_init(void);

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

struct reg_base {
	u8 bank;
	u8 addr;
	u8 mask;
	u8 offset;
};

struct reg_pack {
	struct reg_base *pbase;
	u8 *pval;
	u32 size;
};

enum nvp6324_mode {
	nvp6324_mode_min = 0,
	nvp6324_mode_1080p = 0,
	nvp6324_mode_720p,
	nvp6324_mode_sd,
	nvp6324_mode_max = nvp6324_mode_sd,
};

enum nvp6324_fps {
	nvp6324_fps_min = 0,
	nvp6324_fps_25 = 0,
	nvp6324_fps_30,
	nvp6324_fps_50,
	nvp6324_fps_60,
	nvp6324_fps_max = nvp6324_fps_60
};

enum nvp6324_phy_mclks {
	nvp6324_mclk_min = 0,
	nvp6324_378mhz = 0,
	nvp6424_594mhz,
	nvp6324_756mhz,
	nvp6324_1242mhz,
	nvp6324_mclk_max = nvp6324_1242mhz
};

enum nvp6324_analog_input {
	nvp6324_input_min = 0,
	nvp6324_input_single = 0,
	nvp6324_input_differ,
	nvp6324_input_max = nvp6324_input_differ
};


struct nvp6324_mode_info {
	enum nvp6324_mode mode;
	u32 width;
	u32 height;
	struct reg_pack reg_pack;
};


struct sensor {
	struct sensor_data sen;
	v4l2_std_id        std_id;
	
	int current_bank;
	int isConf;
	int isAttached;

	enum nvp6324_phy_mclks phy_mclks;
	enum nvp6324_analog_input analog_input;
	enum nvp6324_fps current_fps;
	enum nvp6324_mode current_mod_type;

	const struct nvp6324_mode_info *current_mode;
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

static inline int nvp6324_set_reg_bank(u8 bank)
{
	int ret;

	if (nvp6324_data.current_bank == bank)
		return 0;

	ret = nvp6324_write_reg(0xff, bank);

	if (ret < 0)
		return ret;

	nvp6324_data.current_bank = bank;
	return 0;
}

int nvp6324_transfer_regs(const struct reg_pack *reg_pack,
			  u32 channel, u32 counts)
{
	u32 i, j;
	u32 ch = channel;
	u8 bank, addr, val, mask, offset;
	const struct reg_base *pbase = reg_pack->pbase;
	u8 *pval = reg_pack->pval;
	u32 size = reg_pack->size;

if (counts == 0 || counts > 4 || (ch + counts) > 4 || size == 0)
		return -EINVAL;

	for (i = 0; i < counts; i++, ch++) {
		for (j = 0; j < size; j++) {
			bank = pbase[j].bank;
			addr = pbase[j].addr;
			mask = pbase[j].mask;
			offset = pbase[j].offset;
			val = pval[j];

			switch(bank) {
			case 0x5:
				bank += ch;
				break;
			case 0x9:
				if (addr >= 0x96 && addr <=0x9e)
					addr += ch << 5;
				else if (addr >= 0x50 && addr <= 0x53)
					addr += ch << 2;
				else
					addr += ch;
				break;
			case 0xa:
				if ((addr >= 0x30 && addr <=0x3c) ||
				    (addr >= 0x25 && addr <=0x27)) {
					bank += ch / 2;
					addr += ch % 2 * 0x80;
				} else {
					addr += ch;
				}
				break;
			default:
				addr += ch;
				break;
 			}
		  
			val = (val & mask) << offset;
			if((bank == 0x00) && (addr == 0x00)){
			  val = 0x0;
			  dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 we must use only VC0 on bank: 0x%02x, reg: 0x%02x, data: 0x%02x", bank, addr, val);
			}
			else if((bank == 0x00) && (addr == 0x01)){
			  val = 0x1;
			  dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324   VC1 off bank: 0x%02x, reg: 0x%02x, data: 0x%02x", bank, addr, val);
			}
			else if((bank == 0x00) && (addr == 0x02)){
			   val = 0x1;
			   dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324  VC2 off bank: 0x%02x, reg: 0x%02x, data: 0x%02x", bank, addr, val);
			}
			else if((bank == 0x00) && (addr == 0x03)){
			   val = 0x1;
			   dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324  VC3 off bank: 0x%02x, reg: 0x%02x, data: 0x%02x", bank, addr, val);
			}
			nvp6324_set_reg_bank(bank);
			nvp6324_write_reg(addr, val);
			dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 nvp6324_transfer_regs, bank: 0x%02x, reg: 0x%02x, data: 0x%02x", bank, addr, val);
			
		}
	}

	return 0;
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
	nvp6324_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
 

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


	nvp6324_data.current_bank = 0xff;
	nvp6324_set_reg_bank(0);
	ret = nvp6324_read_reg(0xf4);
	if (ret != 0xb0) {
		dev_warn(dev, "nvp6324 not found, chip id reg 0xf4=0x%x.\n", ret);
		clk_disable_unprepare(nvp6324_data.sen.sensor_clk);
		return -ENODEV;
	}

	ret = nvp6324_video_init();
	if (ret < 0) {
		dev_err(dev, "failed to init video: %d\n", ret);
		return ret;
	}

	dev_info(dev, "nvp6324 found, address: 0x%02x, chip id: 0xb0", client->addr);
	
	nvp6324_int_device.priv = &nvp6324_data;

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


/* common */
static struct reg_base nvp6324_regs_base_common[] = {
	{0x05, 0x32, 0xff, 0x0}, // NOVIDEO_DET_A
	{0x05, 0xb9, 0xff, 0x0}, // HAFC_LPF_SEL
	{0x09, 0x44, 0xff, 0x0}, // FSC_EXT_EN
	{0x05, 0x00, 0xff, 0x0}, // A_CMP_PW_MODE
	{0x05, 0x02, 0xff, 0x0}, // A_CMP_TIMEUNIT
	{0x05, 0x1e, 0xff, 0x0}, // VAFEMD
	{0x05, 0x58, 0xff, 0x0}, // VAFE1_EQ_BAND_SEL
	{0x05, 0x59, 0xff, 0x0}, // LPF_BYPASS
	{0x05, 0x5a, 0xff, 0x0}, // VAFE_IMP_CNT
	{0x05, 0x5b, 0xff, 0x0}, // VAFE_DUTY
	{0x05, 0x5c, 0xff, 0x0}, // VAFE_B_LPF_SEL
	{0x05, 0x94, 0xff, 0x0}, // PWM_DELAY_H
	{0x05, 0x95, 0xff, 0x0}, // PWM_DELAY_L
	{0x05, 0x65, 0xff, 0x0}, // VAFE_CML_SPEED

	{0x09, 0x44, 0xff, 0x0}, // FSC_EXT_EN
	{0x09, 0x50, 0xff, 0x0}, // FSC_EXT_VAL_7_0
	{0x09, 0x51, 0xff, 0x0}, // FSC_EXT_VAL_15_8
	{0x09, 0x52, 0xff, 0x0}, // FSC_EXT_VAL_23_16
	{0x09, 0x53, 0xff, 0x0}, // FSC_EXT_VAL_31_24
};

static u8 val_common[ARRAY_SIZE(nvp6324_regs_base_common)] = {
	0x10, // NOVIDEO_DET_A
	0xb2, // HAFC_LPF_SEL
	0x00, // FSC_EXT_EN
	0xd0, // A_CMP_PW_MODE
	0x0c, // A_CMP_TIMEUNIT
	0x00, // VAFEMD
	0x00, // VAFE1_EQ_BAND_SEL
	0x00, // LPF_BYPASS
	0x00, // VAFE_IMP_CNT
	0x41, // VAFE_DUTY
	0x78, // VAFE_B_LPF_SEL
	0x00, // PWM_DELAY_H
	0x00, // PWM_DELAY_L
	0x80, // VAFE_CML_SPEED

	0x00, // FSC_EXT_EN
	0x30, // FSC_EXT_VAL_7_0
	0x6f, // FSC_EXT_VAL_15_8
	0x67, // FSC_EXT_VAL_23_16
	0x48, // FSC_EXT_VAL_31_24
};

/* video mode */
static struct reg_base nvp6324_regs_base_mode[] = {
	// decoder_color_fmtdef
	{0x00, 0x20, 0xff, 0x0}, // brightnees
	{0x00, 0x24, 0xff, 0x0}, // contrast
	{0x00, 0x28, 0xff, 0x0}, // black_level
	{0x00, 0x40, 0xff, 0x0}, // hue
	{0x00, 0x44, 0xff, 0x0}, // u_gain
	{0x00, 0x48, 0xff, 0x0}, // v_gain
	{0x00, 0x4c, 0xff, 0x0}, // u_offset
	{0x00, 0x50, 0xff, 0x0}, // v_offset
	{0x00, 0x58, 0xff, 0x0}, // saturation_a
	{0x05, 0x2b, 0xff, 0x0}, // saturation_b
	{0x05, 0x24, 0xff, 0x0}, // burst_dec_a
	{0x05, 0x5f, 0xff, 0x0}, // burst_dec_b
	{0x05, 0xd1, 0xff, 0x0}, // burst_dec_c
	{0x05, 0x26, 0xff, 0x0}, // FSC_LOCK_SENSE
	{0x05, 0xb8, 0xff, 0x0}, // HPLL_MASK_END
	{0x09, 0x40, 0xff, 0x0}, // FSC_DET_MODE
	{0x05, 0xb5, 0xff, 0x0}, // HPLL Locking

	// decoder_basic_vfmt_fmtdef
	{0x00, 0x10, 0xff, 0x0}, // video_format
	{0x00, 0x04, 0xff, 0x0}, // sd_mode
	{0x00, 0x08, 0xff, 0x0}, // ahd_mode
	{0x00, 0x0c, 0xff, 0x0}, // spl_mode
	{0x05, 0x69, 0x03, 0x0}, // sd_freq_sel

	// decoder_basic_chroma_fmtdef
	{0x00, 0x5c, 0xff, 0x0}, // pal_cm_off
	{0x05, 0x28, 0xff, 0x0}, // s_point
	{0x05, 0x25, 0xff, 0x0}, // fsc_lock_mode
	{0x05, 0x90, 0xff, 0x0}, // comb_mode

	// decoder_basic_timing_fmtdef
	{0x05, 0x47, 0xff, 0x0}, // sync_rs
	{0x00, 0x68, 0xff, 0x0}, // h_delay_lsb
	{0x00, 0x6c, 0xff, 0x0}, // h_dly_msb
	{0x05, 0x38, 0x01, 0x4}, // h_mask_on
	{0x05, 0x38, 0x0f, 0x0}, // h_mask_sel
	{0x05, 0x64, 0xff, 0x0}, // mem_rdp
	{0x00, 0x64, 0xff, 0x0}, // v_blk_end_b
	{0x00, 0x60, 0xff, 0x0}, // y_delay
	{0x00, 0x14, 0x01, 0x4}, // fld_inv
	{0x00, 0x78, 0xff, 0x0}, // v_blk_end_a

	// decoder_basic_hscaler_fmtdef
	{0x09, 0x96, 0xff, 0x0}, // h_down_scaler
	{0x09, 0x97, 0xff, 0x0}, // h_scaler_mode
	{0x09, 0x98, 0xff, 0x0}, // ref_base_lsb
	{0x09, 0x99, 0xff, 0x0}, // ref_base_msb
	{0x05, 0x53, 0x03, 0x2}, // line_mem_mode
	{0x09, 0x93, 0xff, 0x0}, // h_scaler_active

	// decoder_basic_hpll_fmtdef
	{0x05, 0x50, 0xff, 0x0}, // hpll_mask_on
	{0x05, 0xbb, 0xff, 0x0}, // hafc_byp_th_e
	{0x05, 0xb7, 0xff, 0x0}, // hafc_byp_th_s
	{0x05, 0xb8, 0xff, 0x0}, // hafc_op_md

	// clock
	{0x01, 0x84, 0xff, 0x0}, // clk_adc
	{0x01, 0x88, 0xff, 0x0}, // clk_pre
	{0x01, 0x8c, 0xff, 0x0}, // clk_post

	// Reserved
	{0x05, 0x6e, 0xff, 0x0}, // vblk_end_sel
	{0x05, 0x6f, 0xff, 0x0}, // vblk_end_ext
	{0x05, 0x01, 0xff, 0x0}, // cml_mode
	{0x05, 0x05, 0xff, 0x0}, // agc_op
	{0x05, 0x1d, 0xff, 0x0}, // g_sel
	{0x05, 0x62, 0xff, 0x0}, // sync_sel

	{0x00, 0x00, 0xff, 0x0}, // PD_VCH
};

static u8 val_mode_1080p25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x03, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x48, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x03, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};

static u8 val_mode_720p25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x0d, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x05, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};

static u8 val_mode_sdp25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x00, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0xb8, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x00, // HPLL Locking

	0xdd, // video_format
	0x0f, // sd_mode
	0x00, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x0b, // pal_cm_off
	0xd0, // s_point
	0xcc, // fsc_lock_mode
	0x0d, // comb_mode

	0xee, // sync_rs
	0x60, // h_delay_lsb
	0x00, // h_dly_msb
	0x00, // h_mask_on
	0x00, // h_mask_sel
	0x01, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x00, // v_blk_end_a

	0x10, // h_down_scaler
	0x10, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x01, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0xb9, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x20, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};



static u8 val_mode_1080p30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x02, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x48, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x04, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};

static u8 val_mode_720p30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x0c, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x05, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x10, // vblk_end_sel
	0x1c, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};

static u8 val_mode_sdp30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x8c, // contrast
	0x80, // black_level
	0x00, // hue
	0x10, // u_gain
	0x10, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x00, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0xb8, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x00, // HPLL Locking

	0xa0, // video_format
	0x0e, // sd_mode
	0x00, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x8b, // pal_cm_off
	0xd0, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0x04, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x00, // h_mask_on
	0x00, // h_mask_sel
	0x01, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x01, // fld_inv
	0x80, // v_blk_end_a

	0x10, // h_down_scaler
	0x10, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x01, // line_mem_mode
	0x00, // h_scaler_active

	0x84, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0xb9, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x20, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
	
	0x00,
};


struct nvp6324_input_mode {
	enum nvp6324_analog_input mode;
	struct reg_pack reg_pack;
};

static int nvp6324_framerates[] = {
	[nvp6324_fps_25] = 25,
	[nvp6324_fps_30] = 30,
	[nvp6324_fps_50] = 50,
	[nvp6324_fps_60] = 60,
};

/* analog single and differential */
static struct reg_base nvp6324_regs_base_analog_in[] = {
	{0x05, 0x00, 0xff, 0x0},
	{0x05, 0x01, 0xff, 0x0},
	{0x05, 0x1d, 0xff, 0x0},
	{0x05, 0x92, 0xff, 0x0},
};

static u8 val_analog_in_diff[ARRAY_SIZE(nvp6324_regs_base_analog_in)] = {
	0xd0,
	0x2c,
	0x8c,
	0x00,
};

static u8 val_analog_in_single[ARRAY_SIZE(nvp6324_regs_base_analog_in)] = {
	0xd0,
	0xa2,
	0x0c,
	0x00,
};


static struct nvp6324_input_mode nvp6324_input_mode_data[nvp6324_input_max + 1] = {
	{
		nvp6324_input_single,
		{ nvp6324_regs_base_analog_in,
		  val_analog_in_single,
		  ARRAY_SIZE(nvp6324_regs_base_analog_in),
		},
	},

	{
		nvp6324_input_differ,
		{ nvp6324_regs_base_analog_in,
		  val_analog_in_diff,
		  ARRAY_SIZE(nvp6324_regs_base_analog_in),
		},
	}
};

static struct nvp6324_mode_info nvp6324_mode_info_data[nvp6324_fps_max + 1][nvp6324_mode_max + 1] = {
	/* 25 pal fps */
	{
		{ nvp6324_mode_1080p, 1920, 1080,
			{ nvp6324_regs_base_mode,
			  val_mode_1080p25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_720p, 1280, 720,
			{ nvp6324_regs_base_mode,
			  val_mode_720p25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_sd, 720, 480,
			{ nvp6324_regs_base_mode,
			  val_mode_sdp25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		}
	},

	/* 30 ntsc fps */
	{
		{ nvp6324_mode_1080p, 1920, 1080,
			{ nvp6324_regs_base_mode,
			  val_mode_1080p30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_720p, 1280, 720,
			{ nvp6324_regs_base_mode,
			  val_mode_720p30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_sd, 720, 480,
			{ nvp6324_regs_base_mode,
			  val_mode_sdp30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		}
	},

	/* 50 pal fps */
	{
	},

	/* 60 ntsc fps */
	{
	}
};



static struct reg_base nvp6324_regs_base_pattern1[] = {
	{0x00, 0x18, 0xff, 0x0},	
	{0x00, 0x1c, 0xff, 0x0},	
	{0x05, 0x6a, 0xff, 0x0},
};

static struct reg_base nvp6324_regs_base_pattern2[] = {
	{0x00, 0x19, 0xff, 0x0},	
	{0x00, 0x1d, 0xff, 0x0},	
	{0x06, 0x6a, 0xff, 0x0},
};

static struct reg_base nvp6324_regs_base_pattern3[] = {
	{0x00, 0x1a, 0xff, 0x0},	
	{0x00, 0x1e, 0xff, 0x0},	
	{0x07, 0x6a, 0xff, 0x0},
};

static struct reg_base nvp6324_regs_base_pattern4[] = {
	{0x00, 0x1b, 0xff, 0x0},	
	{0x00, 0x1f, 0xff, 0x0},	
	{0x08, 0x6a, 0xff, 0x0},
};

static u8 val_pattern1[ARRAY_SIZE(nvp6324_regs_base_pattern1)] = {
	0x13,	
	0xF1,  //12 - yellow  //16 red  ///18 black ///10-blue	
	0x80,  //80
};

static u8 val_pattern2[ARRAY_SIZE(nvp6324_regs_base_pattern2)] = {
	0x03,	
	0xF2,  //12 - yellow  //16 red  ///18 black ///10-blue	 
	0x80,  //80
};
static u8 val_pattern3[ARRAY_SIZE(nvp6324_regs_base_pattern3)] = {
	0x03,	
	0xF1,  //12 - yellow  //16 red  ///18 black ///10-blue	 5 magenta
	0x80,  //80
};
static u8 val_pattern4[ARRAY_SIZE(nvp6324_regs_base_pattern4)] = {
	0x03,	
	0xF1,  //12 - yellow  //16 red  ///18 black ///10-blue	 4 green
	0x80,  //80
};

static inline void nvp6324_pattern_enable(void)
{
	struct reg_pack reg_pack;

		
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 pattern init start\n");
	
	reg_pack.pbase = nvp6324_regs_base_pattern1;
	reg_pack.pval = val_pattern1;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_pattern1);
	
	nvp6324_transfer_regs(&reg_pack, 0, 1); //was 4
	
	reg_pack.pbase = nvp6324_regs_base_pattern2;
	reg_pack.pval = val_pattern2;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_pattern2);
	
	nvp6324_transfer_regs(&reg_pack, 0, 1); //was 4
	
	reg_pack.pbase = nvp6324_regs_base_pattern3;
	reg_pack.pval = val_pattern3;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_pattern3);
	
	nvp6324_transfer_regs(&reg_pack, 0, 1); //was 4
	
	reg_pack.pbase = nvp6324_regs_base_pattern4;
	reg_pack.pval = val_pattern4;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_pattern4);
	
	nvp6324_transfer_regs(&reg_pack, 0, 1); //was 4
	
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 pattern init done\n");
}

inline enum nvp6324_mode nvp6324_video_current_mode(void)
{
	return nvp6324_data.current_mode->mode;
}



static inline void nvp6324_manual_set(void)
{
	int i;
	u8 reg;

	nvp6324_set_reg_bank(0x01);
	nvp6324_write_reg(0x7c, 0x0);

	/* detect */
	nvp6324_set_reg_bank(0x13);
	for (i = 0; i < 4; i++) {
		reg = nvp6324_read_reg(0x30);
		reg &= (~(1 << (i + 4)) & (~(1 << i)));
		nvp6324_write_reg(0x30, reg);

		reg = nvp6324_read_reg(0x31);
		reg &= (~(1 << (i + 4)) & (~(1 << i)));
		nvp6324_write_reg(0x31, reg);

		reg = nvp6324_read_reg(0x32);
		reg &= (~(1 << i));
		nvp6324_write_reg(0x32, reg);
	}
}


////////////////////////////////MIPI START////////////////////////////////////////////////////////////////


/* phy clk */
static struct reg_base nvp6324_regs_base_mclk[] = {
	{0x21, 0x40, 0xff, 0x0},
	{0x21, 0x41, 0xff, 0x0},
	{0x21, 0x42, 0xff, 0x0},
	{0x21, 0x43, 0xff, 0x0},
	{0x21, 0x11, 0xff, 0x0},
	{0x21, 0x10, 0xff, 0x0},
	{0x21, 0x12, 0xff, 0x0},
	{0x21, 0x13, 0xff, 0x0},
	{0x21, 0x17, 0xff, 0x0},
	{0x21, 0x18, 0xff, 0x0},
	{0x21, 0x15, 0xff, 0x0},
	{0x21, 0x14, 0xff, 0x0},
	{0x21, 0x16, 0xff, 0x0},
	{0x21, 0x19, 0xff, 0x0},
	{0x21, 0x1a, 0xff, 0x0},
	{0x21, 0x1b, 0xff, 0x0},
	{0x21, 0x1c, 0xff, 0x0},
	{0x21, 0x0E, 0xff, 0x0},
};

static u8 val_mclk_378[ARRAY_SIZE(nvp6324_regs_base_mclk)] = {
	0xdc,
	0x20,
	0x03,
	0x43,
	0x03,
	0x07,
	0x04,
	0x06,
	0x01,
	0x0b,
	0x02,
	0x0e,
	0x04,
	0x03,
	0x07,
	0x06,
	0x05,
	0x02,
};

static u8 val_mclk_594[ARRAY_SIZE(nvp6324_regs_base_mclk)] = {
	0xcc,
	0x10,
	0x03,
	0x43,
	0x04,
	0x0a,
	0x06,
	0x09,
	0x01,
	0x0d,
	0x04,
	0x16,
	0x05,
	0x05,
	0x0a,
	0x08,
	0x07,
	0x02,
};

static u8 val_mclk_756[ARRAY_SIZE(nvp6324_regs_base_mclk)] = {
	0xdc,
	0x10,
	0x03,
	0x43,
	0x05,
	0x0c,
	0x07,
	0x0b,
	0x01,
	0x0e,
	0x04,
	0x1c,
	0x07,
	0x06,
	0x0d,
	0x0b,
	0x09,
	0x02,
};

static u8 val_mclk_1242[ARRAY_SIZE(nvp6324_regs_base_mclk)] = {
	0xdc,
	0x10,
	0x03,
	0x43,
	0x08,
	0x13,
	0x0b,
	0x12,
	0x02,
	0x12,
	0x07,
	0x2d,
	0x0b,
	0x09,
	0x15,
	0x11,
	0x0e,
	0x02,
};

struct nvp6324_mipi_clk {
	enum nvp6324_phy_mclks mode;
	struct reg_pack reg_pack;
};

struct nvp6324_mipi_clk nvp6324_mipi_clk_data[nvp6324_mclk_max +1] = {
	{ nvp6324_378mhz,
		{ nvp6324_regs_base_mclk,
		  val_mclk_378,
		  ARRAY_SIZE(nvp6324_regs_base_mclk)
		},
	},
	{ nvp6424_594mhz,
		{ nvp6324_regs_base_mclk,
		  val_mclk_594,
		  ARRAY_SIZE(nvp6324_regs_base_mclk)
		},
	},
	{ nvp6324_756mhz,
		{ nvp6324_regs_base_mclk,
		  val_mclk_756,
		  ARRAY_SIZE(nvp6324_regs_base_mclk)
		},
	},
	{ nvp6324_1242mhz,
		{ nvp6324_regs_base_mclk,
		  val_mclk_1242,
		  ARRAY_SIZE(nvp6324_regs_base_mclk)
		},
	}
};

static struct reg_base nvp6324_regs_base_mipi_fmt[] = {
	{0x21, 0x3e, 0xff, 0x0},
	{0x21, 0x3f, 0xff, 0x0},
	{0x20, 0x01, 0xff, 0x0},
};

static u8 val_mipi_fmt_1080p25[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0x00,
};

static u8 val_mipi_fmt_720p25[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0x55,
};

static u8 val_mipi_fmt_sdp25[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0xaa,
};

static u8 val_mipi_fmt_1080p30[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0x00,
};

static u8 val_mipi_fmt_720p30[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0x55,
};
static u8 val_mipi_fmt_sdp30[ARRAY_SIZE(nvp6324_regs_base_mipi_fmt)] = {
	0x00,
	0x00,
	0xaa,
};

struct nvp6324_mipi_fmt {
	struct reg_pack reg_pack;
};

struct nvp6324_mipi_fmt nvp6324_mipi_fmt_data[nvp6324_fps_max + 1][nvp6324_mode_max + 1] = {

	{
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_1080p25,
		  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			},
		},
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_720p25,
		 	  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			},
		},
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_sdp25,
			  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			}
		},
	},
	
	{
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_1080p30,
			  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			},
	},
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_720p30,
			  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			},
		},
		{
			{ nvp6324_regs_base_mipi_fmt,
			  val_mipi_fmt_sdp30,
			  ARRAY_SIZE(nvp6324_regs_base_mipi_fmt),
			},
		},
	},

	{
	},

	{
	},
};

static struct reg_base nvp6324_regs_base_mipi_type[] = {
	{0x21, 0x44, 0xff, 0x0},
	{0x21, 0x49, 0xff, 0x0},
	{0x21, 0x49, 0xff, 0x0},
	{0x21, 0x44, 0xff, 0x0},
	{0x21, 0x08, 0xff, 0x0},
	{0x21, 0x0f, 0xff, 0x0}, // MIPI_TX_FRAME_CNT_EN
	{0x21, 0x38, 0xff, 0x0}, // mipi data type
	{0x21, 0x39, 0xff, 0x0},
	{0x21, 0x3a, 0xff, 0x0},
	{0x21, 0x3b, 0xff, 0x0},
	{0x21, 0x07, 0xff, 0x0},

	{0x21, 0xc8, 0xff, 0x0}, // disable parallel
	{0x21, 0xc9, 0xff, 0x0},
	{0x21, 0xca, 0xff, 0x0},
	{0x21, 0xcb, 0xff, 0x0},
	{0x21, 0xcc, 0xff, 0x0},
	{0x21, 0xcd, 0xff, 0x0},
	{0x21, 0xce, 0xff, 0x0},
	{0x21, 0xcf, 0xff, 0x0},
};

static u8 val_mipi_type_yuv422[ARRAY_SIZE(nvp6324_regs_base_mipi_type)] = {
	0x00,
	0xf3,
	0xf0,
	0x02,
	0x40,
	0x01,
	0x1e,
	0x1e,
	0x1e,
	0x1e,
	0x07, /////mipi tx lanes active -1 was 4 -f   3 - 1lane   7-2lane  f-4lanes

	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};

static u8 val_mipi_type_yuv422_off[ARRAY_SIZE(nvp6324_regs_base_mipi_type)] = {
	0x00,
	0xf3,
	0xf0,
	0x02,
	0x40,
	0x01,
	0x1e,
	0x1e,
	0x1e,
	0x1e,
	0x87, /////mipi tx lanes active -1 was 4 -f   3 - 1lane   7-2lane  f-4lanes

	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};

#if 0
static u8 val_mipi_type_yuv420[ARRAY_SIZE(nvp6324_regs_base_mipi_type)] = {
	0x00,
	0xf3,
	0xf0,
	0x02,
	0x40,
	0x01,
	0x18,
	0x18,
	0x18,
	0x18,
	0x0f,

	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};

static u8 val_mipi_type_yuv420_legacy[ARRAY_SIZE(nvp6324_regs_base_mipi_type)] = {
	0x00,
	0xf3,
	0xf0,
	0x02,
	0x40,
	0x01,
	0x1a,
	0x1a,
	0x1a,
	0x1a,
	0x0f,

	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};
#endif

void nvp6324_mipi_init(uint8_t state)
{
	struct reg_pack reg_pack;
	
	if(state == 0){
		memcpy(&reg_pack, &nvp6324_mipi_clk_data[nvp6324_data.phy_mclks].reg_pack, sizeof(reg_pack));

		nvp6324_transfer_regs(&reg_pack, 0, 1);
	
		reg_pack.pbase = nvp6324_regs_base_mipi_type;
		reg_pack.pval = val_mipi_type_yuv422_off;
		reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_mipi_type);

		nvp6324_transfer_regs(&reg_pack, 0, 1);

		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 mipi init done , state = 0\n");
	}
	else
	{
		memcpy(&reg_pack, &nvp6324_mipi_clk_data[nvp6324_data.phy_mclks].reg_pack, sizeof(reg_pack));

		nvp6324_transfer_regs(&reg_pack, 0, 1);
	
		reg_pack.pbase = nvp6324_regs_base_mipi_type;
		reg_pack.pval = val_mipi_type_yuv422;
		reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_mipi_type);

		nvp6324_transfer_regs(&reg_pack, 0, 1);

		dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 mipi init done , state = 1\n");

	}	
}

void nvp6324_mipi_fmt_set(void)
{
	enum nvp6324_fps fps = nvp6324_data.current_fps;
	enum nvp6324_mode mode = nvp6324_data.current_mod_type;
	struct nvp6324_mipi_fmt *fmt = &nvp6324_mipi_fmt_data[fps][mode];
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 mipi fmt set start\n");
	
	nvp6324_transfer_regs(&fmt->reg_pack, 0, 1);

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 mipi fmt set done\n");
}

static struct reg_base nvp6324_regs_base_arb[] = {
	{0x20, 0x00, 0xff, 0x0},
	{0x20, 0x40, 0xff, 0x0},
	{0x20, 0x0f, 0xff, 0x0},
	{0x20, 0x0d, 0xff, 0x0},
	//{0x20, 0x3a, 0xff, 0x0},
	{0x20, 0x40, 0xff, 0x0},
	{0x20, 0x00, 0xff, 0x0},
};

static u8 val_arb_yuv422[ARRAY_SIZE(nvp6324_regs_base_arb)] = {
	0x00,   ////disable chan arb module 
	0x01,   //////sw reset
	0x00,   /////yuv 422 set
	0x01,	////32 bit mode for 4 lanes  1 -32 bit  0 - 16 bit for 2 lanes
	//0x0f,   /////No transmisoon when no 
	0x00,	//////sw reset
	0xff,   ////enable all chan arb module
};

#if 0
static u8 val_arb_yuv420[ARRAY_SIZE(nvp6324_regs_base_arb)] = {
	0x00,
	0x01,
	0xaa,
	0x01,
	0x00,
	0xff,
};
static u8 val_arb_yuv420_legacy[ARRAY_SIZE(nvp6324_regs_base_arb)] = {
	0x00,
	0x01,
	0x55,
	0x01,
	0x00,
	0xff,
};
#endif

void nvp6324_arb_init(void)
{
	struct reg_pack reg_pack;

	reg_pack.pbase = nvp6324_regs_base_arb;
	reg_pack.pval = val_arb_yuv422;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_arb);
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 arb init start\n");
	
	nvp6324_transfer_regs(&reg_pack, 0, 1);

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 arb init done\n");
}



////////////////////////////////MIPI END////////////////////////////////////////////////////////////////

////////////////////////////////EQ START///////////////////////////////////////////////////////////////

static struct reg_base nvp6324_regs_eq_base[] = {
	/* base */
	{0x05, 0x65, 0xff, 0x0}, // eq_bypass
	{0x05, 0x58, 0xff, 0x0}, // eq_band_sel
	{0x05, 0x5c, 0xff, 0x0}, // eq_gain_sel
	{0x0a, 0x3d, 0xff, 0x0}, // deq_a_on
	{0x0a, 0x3c, 0xff, 0x0}, // deq_a_sel

	/* coeff_ */
	{0x0a, 0x30, 0xff, 0x0}, // deqA_01
	{0x0a, 0x31, 0xff, 0x0}, // deqA_02
	{0x0a, 0x32, 0xff, 0x0}, // deqA_03
	{0x0a, 0x33, 0xff, 0x0}, // deqA_04
	{0x0a, 0x34, 0xff, 0x0}, // deqA_05
	{0x0a, 0x35, 0xff, 0x0}, // deqA_06
	{0x0a, 0x36, 0xff, 0x0}, // deqA_07
	{0x0a, 0x37, 0xff, 0x0}, // deqA_08
	{0x0a, 0x38, 0xff, 0x0}, // deqA_09
	{0x0a, 0x39, 0xff, 0x0}, // deqA_10
	{0x0a, 0x3a, 0xff, 0x0}, // deqA_11
	{0x0a, 0x3b, 0xff, 0x0}, // deqA_12

	/* color */
	{0x00, 0x24, 0xff, 0x0}, // contrast
	{0x00, 0x30, 0xff, 0x0}, // y_peaking_mode
	{0x00, 0x34, 0xff, 0x0}, // y_fir_mode
	{0x05, 0x31, 0xff, 0x0}, // c_filter
	{0x00, 0x5c, 0xff, 0x0}, // pal_cm_off
	{0x00, 0x40, 0xff, 0x0}, // hue
	{0x00, 0x44, 0xff, 0x0}, // u_gain
	{0x00, 0x48, 0xff, 0x0}, // v_gain
	{0x00, 0x4c, 0xff, 0x0}, // u_offset
	{0x00, 0x50, 0xff, 0x0}, // v_offset
	{0x00, 0x28, 0xff, 0x0}, // black_level
	{0x05, 0x27, 0xff, 0x0}, // acc_ref
	{0x05, 0x28, 0xff, 0x0}, // cti_delay
	{0x05, 0x2b, 0xff, 0x0}, // saturation_b
	{0x05, 0x24, 0xff, 0x0}, // burst_dec_a
	{0x05, 0x5f, 0xff, 0x0}, // burst_dec_b
	{0x05, 0xd1, 0xff, 0x0}, // burst_dec_c
	{0x05, 0xd5, 0xff, 0x0}, // c_option
	{0x0a, 0x25, 0xff, 0x0}, // y_filter_b
	{0x0a, 0x27, 0xff, 0x0}, // y_filter_b_sel

	/* timing_a */
	{0x00, 0x68, 0xff, 0x0}, // h_delay_a
	{0x05, 0x38, 0xff, 0x0}, // h_delay_b
	{0x00, 0x6c, 0x0f, 0x0}, // h_delay_c
	{0x00, 0x64, 0xff, 0x0}, // y_delay
	
	{0x01, 0x84, 0xff, 0x0}, // clk_adc
	{0x01, 0x88, 0xff, 0x0}, // clk_adc_pre
	{0x01, 0x8c, 0xff, 0x0}, // clk_adc_post

	{0x09, 0x96, 0xff, 0x0}, // h_scaler1
	{0x09, 0x97, 0xff, 0x0}, // h_scaler2
	{0x09, 0x98, 0xff, 0x0}, // h_scaler3
	{0x09, 0x99, 0xff, 0x0}, // h_scaler4
	{0x09, 0x9a, 0xff, 0x0}, // h_scaler5
	{0x09, 0x9b, 0xff, 0x0}, // h_scaler6
	{0x09, 0x9c, 0xff, 0x0}, // h_scaler7
	{0x09, 0x9d, 0xff, 0x0}, // h_scaler8
	{0x09, 0x9e, 0xff, 0x0}, // h_scaler9
	{0x09, 0x40, 0xff, 0x0}, // pn_auto
	{0x05, 0x90, 0xff, 0x0}, // comb_mode
	{0x05, 0xb9, 0xff, 0x0}, // h_pll_op_a
	{0x05, 0x57, 0xff, 0x0}, // mem_path
	{0x05, 0x25, 0xff, 0x0}, // fsc_lock_speed
	{0x00, 0x08, 0xff, 0x0}, // ahd_mode
	{0x00, 0x04, 0xff, 0x0}, // sd_mode
	{0x00, 0x0c, 0xff, 0x0}, // spl_mode
	{0x00, 0x78, 0xff, 0x0}, // vblk_end
	{0x05, 0x1d, 0xff, 0x0}, // afe_g_sel
	{0x05, 0x01, 0xff, 0x0}, // afe_ctr_clp
	{0x05, 0x05, 0xff, 0x0}, // d_agc_option	
	
};

static u8 val_eq_1080p25_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12

	/* color */
	0x86, // contrast
	0x00, // y_peaking_mode
	0x00, // y_fir_mode
	0x82, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xfe, // u_offset
	0xfb, // v_offset
	0x80, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x48, // h_delay_a
	0x13, // h_delay_b
	0x00, // h_delay_c
	0x05, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x00, // h_scaler1
	0x00, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x01, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xdc, // fsc_lock_speed
	0x03, // ahd_mode
	0x00, // sd_mode
	0x00, // spl_mode
	0x21, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x24, // d_agc_option
};

static u8 val_eq_720p25_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12

	/* color */
	0x88, // contrast
	0x03, // y_peaking_mode
	0x0f, // y_fir_mode
	0x82, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0x00, // u_offset
	0x00, // v_offset
	0x84, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x43, // h_delay_a
	0x13, // h_delay_b
	0x00, // h_delay_c
	0x05, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x00, // h_scaler1
	0x00, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x01, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xdc, // fsc_lock_speed
	0x0d, // ahd_mode
	0x00, // sd_mode
	0x00, // spl_mode
	0x21, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x24, // d_agc_option
};

static u8 val_eq_sdp25_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12

	/* color */
	0x90, // contrast
	0x00, // y_peaking_mode
	0x08, // y_fir_mode
	0x02, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0x00, // u_offset
	0x00, // v_offset
	0x90, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xc0, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x68, // h_delay_a
	0x00, // h_delay_b
	0x00, // h_delay_c
	0x07, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x10, // h_scaler1
	0x10, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x0d, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xcc, // fsc_lock_speed
	0x00, // ahd_mode
	0x0f, // sd_mode
	0x00, // spl_mode
	0x21, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x20, // d_agc_option
};

static u8 val_eq_1080p30_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12

	/* color */
	0x86, // contrast
	0x00, // y_peaking_mode
	0x00, // y_fir_mode
	0x82, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xfe, // u_offset
	0xfb, // v_offset
	0x80, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x48, // h_delay_a
	0x13, // h_delay_b
	0x00, // h_delay_c
	0x05, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x00, // h_scaler1
	0x00, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x01, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xdc, // fsc_lock_speed
	0x02, // ahd_mode
	0x00, // sd_mode
	0x00, // spl_mode
	0x21, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x24, // d_agc_option
};

static u8 val_eq_720p30_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12

	/* color */
	0x88, // contrast
	0x03, // y_peaking_mode
	0x0f, // y_fir_mode
	0x82, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0x00, // u_offset
	0x00, // v_offset
	0x84, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x4a, // h_delay_a
	0x00, // h_delay_b
	0x00, // h_delay_c
	0x05, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x00, // h_scaler1
	0x00, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x01, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xdc, // fsc_lock_speed
	0x0c, // ahd_mode
	0x00, // sd_mode
	0x00, // spl_mode
	0x21, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x24, // d_agc_option
};

static u8 val_eq_sdp30_single[ARRAY_SIZE(nvp6324_regs_eq_base)] = {
	/* base */
	0x00, // eq_bypass
	0x77, // eq_band_sel
	0x78, // eq_gain_sel
	0x00, // deq_a_on
	0x00, // deq_a_sel

	/* coeff_ */
	0xac, // deqA_01
	0x78, // deqA_02
	0x17, // deqA_03
	0xc1, // deqA_04
	0x40, // deqA_05
	0x00, // deqA_06
	0xc3, // deqA_07
	0x0a, // deqA_08
	0x00, // deqA_09
	0x02, // deqA_10
	0x00, // deqA_11
	0xb2, // deqA_12
	/* color */
	0x90, // contrast
	0x00, // y_peaking_mode
	0x08, // y_fir_mode
	0x82, // c_filter
	0x82, // pal_cm_off
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0x00, // u_offset
	0x00, // v_offset
	0x90, // black_level
	0x57, // acc_ref
	0x80, // cti_delay
	0xc0, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x80, // c_option
	0x10, // y_filter_b
	0x1e, // y_filter_b_sel

	/* timing_a */
	0x70, // h_delay_a
	0x00, // h_delay_b
	0x00, // h_delay_c
	0x18, // y_delay

	0x04, // clk_adc
	0x01, // clk_adc_pre
	0x02, // clk_adc_post

	0x10, // h_scaler1
	0x10, // h_scaler2
	0x00, // h_scaler3
	0x00, // h_scaler4
	0x00, // h_scaler5
	0x00, // h_scaler6
	0x00, // h_scaler7
	0x00, // h_scaler8
	0x00, // h_scaler9
	0x00, // pn_auto
	0x01, // comb_mode
	0x72, // h_pll_op_a
	0x00, // mem_path
	0xdc, // fsc_lock_speed
	0x00, // ahd_mode
	0x0e, // sd_mode
	0x00, // spl_mode
	0xc0, // vblk_end
	0x0c, // afe_g_sel
	0x2c, // afe_ctr_clp
	0x20, // d_agc_option
};

struct nvp6324_mode_eq {
	enum nvp6324_mode mode;
	struct reg_pack reg_pack;
};

struct nvp6324_mode_eq nvp6324_mode_eq_data[nvp6324_input_max + 1][nvp6324_fps_max + 1][nvp6324_mode_max + 1] = {
	{ /* single */
		{ /* 25 fps */
			{ nvp6324_mode_1080p,
				{ nvp6324_regs_eq_base,
				  val_eq_1080p25_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			},
			{ nvp6324_mode_720p,
				{ nvp6324_regs_eq_base,
				  val_eq_720p25_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			},
			{ nvp6324_mode_sd,
				{ nvp6324_regs_eq_base,
				  val_eq_sdp25_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			}
		},
		{ /* 30 fps */
			{ nvp6324_mode_1080p,
				{ nvp6324_regs_eq_base,
				  val_eq_1080p30_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			},
			{ nvp6324_mode_720p,
				{ nvp6324_regs_eq_base,
				  val_eq_720p30_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			},
			{ nvp6324_mode_sd,
				{ nvp6324_regs_eq_base,
				  val_eq_sdp30_single,
				  ARRAY_SIZE(nvp6324_regs_eq_base),
				},
			}
		},
	},
	{ /* differential */
	},
};

void nvp6324_video_eq_set(void)
{
	enum nvp6324_analog_input input = nvp6324_data.analog_input;
	enum nvp6324_fps fps = nvp6324_data.current_fps;
	enum nvp6324_mode mode = nvp6324_video_current_mode();
	struct nvp6324_mode_eq *eq = &nvp6324_mode_eq_data[input][fps][mode];
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video eq set start \n");
	
	nvp6324_transfer_regs(&eq->reg_pack, 0, 4);

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video eq set done\n");
}

////////////////////////////////EQ END///////////////////////////////////////////////////////////////

static void nvp6324_hardware_init(void)
{
	struct reg_pack reg_pack;
	enum nvp6324_analog_input input = nvp6324_data.analog_input;

	nvp6324_mipi_init(0);

	/* configure differ or single input */
	memcpy(&reg_pack, &nvp6324_input_mode_data[input].reg_pack, sizeof(reg_pack));
	nvp6324_transfer_regs(&reg_pack, 0, 4);

	nvp6324_manual_set();

	/* vafe fsc common set */
	reg_pack.pbase = nvp6324_regs_base_common;
	reg_pack.pval = val_common;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_common);
	nvp6324_transfer_regs(&reg_pack, 0, 4);

	memcpy(&reg_pack, &nvp6324_data.current_mode->reg_pack, sizeof(reg_pack));
	nvp6324_transfer_regs(&reg_pack, 0, 4);

	nvp6324_video_eq_set();

	nvp6324_mipi_fmt_set();
	nvp6324_arb_init();
	nvp6324_pattern_enable();

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 hardware init done\n");
}

static void nvp6324_hardware_preinit(void)
{
	int i = 0;

	//Pad Control Setting
	nvp6324_set_reg_bank(0x04);
	for (i = 0; i < 36; i++) {
		nvp6324_write_reg(0xa0 + i, 0x24);
	}

	// Clock Delay Setting
	nvp6324_set_reg_bank(0x01);
	for (i = 0; i < 4; i++) {
		nvp6324_write_reg(0xcc + i, 0x64);
	}

	// MIPI_V_REG_OFF
	nvp6324_set_reg_bank(0x21);
	nvp6324_write_reg(0x07, 0x80);
	nvp6324_write_reg(0x07, 0x00);

	// AGC_OFF  08.31
	nvp6324_set_reg_bank(0x0a);
	nvp6324_write_reg(0x77, 0x8F);
	nvp6324_write_reg(0xF7, 0x8F);
	nvp6324_write_reg(0xff, 0x0B);
	nvp6324_write_reg(0x77, 0x8F);
	nvp6324_write_reg(0xF7, 0x8F);

	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 hardware preinit done\n");
}

void nvp6324_print_status_regs(void){
	int regD = 0;
	nvp6324_set_reg_bank(0);
	
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

}
static void nvp6324_init_csi(void)      
{
	void *mipi_csi2_info;
	u32 mipi_reg;     
		
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);
	
	nvp6324_mipi_init(0);	

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return;
	}

	mipi_csi2_set_lanes(mipi_csi2_info, 2);  /////     
	
	mipi_csi2_reset_ahd(mipi_csi2_info);

 	mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
	
	nvp6324_mipi_init(1);

	msleep(400);//wait stable
	if (mipi_csi2_info) {
		unsigned int i = 0;
		/* wait for mipi sensor ready */
		while (1) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			if (mipi_reg != 0x200)
				break;
			if (i++ >= 20) {
				dev_info(&nvp6324_data.sen.i2c_client->dev,"mipi csi2 can not receive sensor clk! %x\n", mipi_reg);			
				nvp6324_print_status_regs();
				return;				
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
				nvp6324_print_status_regs();
				return;				
			}
			msleep(10);
		}	
		dev_info(&nvp6324_data.sen.i2c_client->dev,"mipi csi2 error status is: %x\n", mipi_reg);
	}
	
	
	nvp6324_print_status_regs();
	return;
}

int nvp6324_video_init(void)
{
	int ret = 0;

	nvp6324_data.analog_input = nvp6324_input_single;
	nvp6324_data.phy_mclks = nvp6324_756mhz;
	
	nvp6324_data.sen.streamcap.timeperframe.denominator = nvp6324_framerates[nvp6324_fps_25];
	nvp6324_data.sen.streamcap.timeperframe.numerator = 1;
	nvp6324_data.sen.streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	nvp6324_data.sen.streamcap.capturemode = 0;
	nvp6324_data.current_fps = nvp6324_fps_25;
	nvp6324_data.current_mod_type = nvp6324_mode_sd;
	nvp6324_data.current_mode = &nvp6324_mode_info_data[nvp6324_data.current_fps][nvp6324_data.current_mod_type];

	nvp6324_data.sen.pix.width = nvp6324_data.current_mode->width;
	nvp6324_data.sen.pix.height = nvp6324_data.current_mode->height;
	nvp6324_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;    /* YUV422 */
	nvp6324_data.sen.spix.swidth = nvp6324_data.current_mode->width - 1;
	nvp6324_data.sen.spix.sheight = nvp6324_data.current_mode->height;
	nvp6324_data.sen.spix.top = 0;		
	nvp6324_data.sen.spix.left = 0;
	nvp6324_data.std_id = V4L2_STD_ALL; 

	nvp6324_data.sen.mipi_camera = 1;

	nvp6324_hardware_preinit();

	nvp6324_hardware_init();
	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 video init done\n");

	return ret;
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
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_curr = nvp6324_data.sen.mclk;  //BT656 interlace clock mode 1 - prorgressive 0 - interlaced
	p->u.bt656.clock_max  = 27000000;
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
	if (fsize->index > nvp6324_mode_max)
		return -EINVAL;

	
	dev_info(&nvp6324_data.sen.i2c_client->dev, "nvp6324 ioctl_enum_framesizes w = %d h = %d\n", nvp6324_data.current_mode->width, nvp6324_data.current_mode->height);	

	fsize->discrete.width = nvp6324_data.current_mode->width;
	fsize->discrete.height  = nvp6324_data.current_mode->height;

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

	count = 0;
	for (i = 0; i < ARRAY_SIZE(nvp6324_framerates); i++) {
		for (j = 0; j <= nvp6324_mode_max; j ++) {
			if (fival->width == nvp6324_mode_info_data[i][j].width &&
			    fival->height == nvp6324_mode_info_data[i][j].height)
				count++;

			if (fival->index == (count - 1)) {
				fival->discrete.denominator = nvp6324_framerates[i];
				return 0;
			}
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
	nvp6324_init_csi();
	return 0;
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


