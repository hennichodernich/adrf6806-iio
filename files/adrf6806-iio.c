#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/version.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "adrf6806_defs.h"
#include "adrf6806-iio.h"

u32 adrf6806_tune(struct adrf6806_rf_phy_state *st, u32 tune_freq){
	struct adrf6806_rf_phy *phy = st->phy;
	struct spi_device *spi;
	u32 ideal_vco_freq, ideal_divider_times_denominator, writeval;
	u64 tmp;
	u32 denominator = 1250;

	if (phy==NULL)
		return 0;

	spi = phy->spi;

	if (spi==NULL){
		dev_err(&st->phy->indio_dev->dev, "couldn't find SPI device %d\n");
		return 0;
	}

	if (tune_freq < 35000000)
	{
		dev_err(&st->phy->indio_dev->dev, "frequency %d too low\n", tune_freq);
		return 0;
	}
	if ((tune_freq >= 35000000) && (tune_freq < 43750000))
	{
		st->DIV_AB = 5-2;
		st->DIV_SEL = 3;
	}
	if ((tune_freq >= 43750000) && (tune_freq < 58333333))
	{
		st->DIV_AB = 4-2;
		st->DIV_SEL = 3;
	}
	if ((tune_freq >= 58333333) && (tune_freq < 70000000))
	{
		st->DIV_AB = 3-2;
		st->DIV_SEL = 3;
	}
	if ((tune_freq >= 70000000) && (tune_freq < 87500000))
	{
		st->DIV_AB = 5-2;
		st->DIV_SEL = 2;
	}
	if ((tune_freq >= 87500000) && (tune_freq < 116700000))
	{
		st->DIV_AB = 4-2;
		st->DIV_SEL = 2;
	}
	if ((tune_freq >= 116700000) && (tune_freq < 140000000))
	{
		st->DIV_AB = 3-2;
		st->DIV_SEL = 2;
	}
	if ((tune_freq >= 140000000) && (tune_freq < 175000000))
	{
		st->DIV_AB = 5-2;
		st->DIV_SEL = 1;
	}
	if ((tune_freq >= 175000000) && (tune_freq < 233333333))
	{
		st->DIV_AB = 4-2;
		st->DIV_SEL = 1;
	}
	if ((tune_freq >= 233333333) && (tune_freq < 350000000))
	{
		st->DIV_AB = 3-2;
		st->DIV_SEL = 1;
	}
	if ((tune_freq >= 350000000) && (tune_freq <= 525000000))
	{
		st->DIV_AB = 2-2;
		st->DIV_SEL = 1;
	}
	if (tune_freq > 525000000)
	{
		dev_err(&st->phy->indio_dev->dev, "frequency %d too high\n", tune_freq);
		return 0;
	}
	st->lo_divider = (st->DIV_AB + 2) * (1 << st->DIV_SEL) * 2;
	dev_dbg(&spi->dev, "choosing LO divider %d\n", st->lo_divider);
	ideal_vco_freq = tune_freq * st->lo_divider;
	tmp = ideal_vco_freq;
	tmp *= denominator / 2;
	do_div(tmp, st->pfd_freq_Hz);
	ideal_divider_times_denominator = tmp;

	do_div(tmp, denominator);
	st->INT = tmp;
	st->MOD = denominator;
	st->FRAC = ideal_divider_times_denominator - (st->INT * denominator);

	if (st->FRAC==0)
	{
		st->DIV_MODE=1;
	}
	else
	{
		st->DIV_MODE=0;
	}

	dev_dbg(&spi->dev, "found divider: INT=%d, FRAC=%d, MOD=%d\n",st->INT, st->FRAC, st->MOD);

	st->vco_freq_Hz = st->INT * 2 * st->pfd_freq_Hz;
	tmp = st->pfd_freq_Hz;
	tmp *= st->FRAC * 2;
	do_div(tmp, st->MOD);
	st->vco_freq_Hz += tmp;

	tmp = st->vco_freq_Hz;
	do_div(tmp, st->lo_divider);
	st->lo_freq_Hz = tmp;

	tmp = st->vco_freq_Hz;
	do_div(tmp, st->ext_lo_divider);
	st->lo_out_freq_Hz = tmp;

	dev_dbg(&spi->dev, "f_VCO=%u, f_LO=%u, f_OUT=%u\n",st->vco_freq_Hz, st->lo_freq_Hz, st->lo_out_freq_Hz);

	//uint8_t writeorder[8] = {6, 5, 3, 4, 7, 1, 2, 0};

	st->CP_CURR_MUL = st->currmul_val - 1;

	writeval = (st->CP_EN ? ADRF6806_FLAG_CP_EN : 0)
				| (st->L3_EN ? ADRF6806_FLAG_L3_EN : 0)
				| (st->LV_EN ? ADRF6806_FLAG_LV_EN : 0)
				| (st->VCO_EN ? ADRF6806_FLAG_VCO_EN : 0)
				| (st->VCO_SW ? ADRF6806_FLAG_VCO_SW : 0)
				| ADRF6806_BITS_VCO_AMPL(st->VCO_AMPL)
				| (st->VCO_BS_SRC ? ADRF6806_FLAG_VCO_BS_SRC : 0)
				| (st->L3_EN ? ADRF6806_FLAG_L3_EN : 0)
				| ADRF6806_BITS_VCO_BANDSEL(st->VCO_BANDSEL);
	adrf6806_spi_write(spi, 6, writeval);

	writeval = (st->DEMOD_BIAS_EN ? ADRF6806_FLAG_DEMOD_BIAS_EN : 0)
				| (st->LP_EN ? ADRF6806_FLAG_LP_EN : 0)
				| (st->LO_INOUT_CTL ? ADRF6806_FLAG_LO_INOUT_CTL : 0)
				| (st->LO_DRV_EN ? ADRF6806_FLAG_LO_DRV_EN : 0);
	adrf6806_spi_write(spi, 5, writeval);

	writeval = (st->DITH_EN ? ADRF6806_FLAG_DITH_EN : 0 )
				|  ADRF6806_BITS_DITH_MAG(st->DITH_MAG)
				|  ADRF6806_BITS_DITH_RESTART(st->DITH_RESTART_VAL);
	adrf6806_spi_write(spi, 3, writeval);

	writeval = ADRF6806_BITS_REF_MUX_SEL(st->REF_MUX_SEL)
				| ADRF6806_BITS_INPUT_REF(st->REF_INPUT)
				| (st->CP_REF ? ADRF6806_FLAG_CP_REF : 0)
				| (st->PFD_Polarity ? ADRF6806_FLAG_PFD_POL : 0)
				| ADRF6806_BITS_PFD_PH_OFFS(st->PFD_PH_OFFS)
				| ADRF6806_BITS_CP_CURRMUL(st->CP_CURR_MUL)
				| (st->CP_CTL_SRC ? ADRF6806_FLAG_CP_CTL_SRC : 0)
				| ADRF6806_BITS_CP_CTL(st->CPCTRL)
				| ADRF6806_BITS_PFD_EDGE_SENS(st->PFD_EDGE_SENS)
				| ADRF6806_BITS_PFD_ABD(st->PFD_ABLDLY);
	adrf6806_spi_write(spi, 4, writeval);

	writeval = ADRF6806_BITS_DIV_AB(st->DIV_AB)
				| ADRF6806_BITS_DIV_SEL(st->DIV_SEL)
				| ADRF6806_BITS_ODIV_CTL(st->ODIV_CTL);
	adrf6806_spi_write(spi, 7, writeval);

	writeval = ADRF6806_BITS_MOD(st->MOD);
	adrf6806_spi_write(spi, 1, writeval);

	writeval = ADRF6806_BITS_FRAC(st->FRAC);
	adrf6806_spi_write(spi, 2, writeval);

	writeval = (st->DIV_MODE ? ADRF6806_FLAG_DIV_MODE : 0)
						|  ADRF6806_BITS_INT_DIV(st->INT);
	adrf6806_spi_write(spi, 0, writeval);



	return st->lo_freq_Hz;
}

int adrf6806_spi_write(struct spi_device *spi, u8 reg, u32 val)
{
	u8 buf[3];
	int ret;

	buf[0] = (val >> 16) & 0xFF;
	buf[1] = (val >> 8) & 0xFF;
	buf[2] = (val & 0xF8) | (reg & 0x07);

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "Write Error %d", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, val);

	return 0;
}



static int __adrf6806_of_get_u32(struct iio_dev *indio_dev,
			     struct device_node *np, const char *propname,
			     u32 defval, void *out_value, u32 size)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	u32 tmp = defval;
	int ret;

	ret = of_property_read_u32(np, propname, &tmp);

	if (out_value) {
		switch (size){
		case 1:
			*(u8*)out_value = tmp;
			break;
		case 2:
			*(u16*)out_value = tmp;
			break;
		case 4:
			*(u32*)out_value = tmp;
			break;
		default:
			ret = -EINVAL;
		}
	}
	return ret;
}

static void adrf6806_of_get_bool(struct iio_dev *indio_dev, struct device_node *np,
			       const char *propname, bool *out_value)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	*out_value = of_property_read_bool(np, propname);

	return;
}

static struct adrf6806_phy_platform_data
	*adrf6806_phy_parse_dt(struct iio_dev *iodev, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct adrf6806_rf_phy *phy = iio_priv(iodev);
	struct adrf6806_rf_phy_state *st = phy->state;
	struct adrf6806_phy_platform_data *pdata;
	u32 tmp;
	u64 tmpl;
	u32 array[6] = {0};
	int ret, i;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}
	return pdata;
}


struct adrf6806_rf_phy* adrf6806_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}

static void adrf6806_init_state(struct adrf6806_rf_phy *phy)
{
	struct adrf6806_rf_phy_state *st = phy->state;

	st->phy = phy;

	st->tune_freq_Hz=0;
	st->INT=56;
	st->DIV_MODE=1;
	st->MOD=1536;
	st->FRAC=768;
	st->DITH_MAG=3;
	st->DITH_EN=0;
	st->DITH_RESTART_VAL=1;
	st->REF_MUX_SEL=0;
	st->REF_INPUT=1;
	st->CP_REF=0;
	st->PFD_Polarity=1;
	st->PFD_PH_OFFS=10;
	st->currmul_val=2;
	st->CP_CURR_MUL=1;
	st->CP_CTL_SRC=1;
	st->CPCTRL=3;
	st->PFD_EDGE_SENS=0;
	st->PFD_ABLDLY=0;
	st->DEMOD_BIAS_EN=1;
	st->LP_EN=0;
	st->LO_INOUT_CTL=0;
	st->LO_DRV_EN=1;
	st->CP_EN=1;
	st->L3_EN=1;
	st->LV_EN=1;
	st->VCO_EN=1;
	st->VCO_SW=0;
	st->VCO_AMPL=55;
	st->VCO_BS_SRC=0;
	st->VCO_BANDSEL=32;
	st->DIV_AB=0;
	st->DIV_SEL=1;
	st->ODIV_CTL=1;
	st->lo_divider=8;
	st->ext_lo_divider=4;
	st->ref_in_Hz=25000000;


	st->pll_ref_div = 1 << (st->REF_INPUT-1);
	st->pfd_freq_Hz = st->ref_in_Hz / st->pll_ref_div;


	return;
}

static IIO_CONST_ATTR(in_voltage_rf_bandwidth_available, "[1000000 1 2000000]");

static struct attribute *adrf6806_phy_attributes[] = {
		&iio_const_attr_in_voltage_rf_bandwidth_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adrf6806_phy_attribute_group = {
	.attrs = adrf6806_phy_attributes,
};

static int adrf6806_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_OFFSET:
	case IIO_CHAN_INFO_SCALE:
	default:
		ret = -EINVAL;
	}

out_unlock:
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int adrf6806_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6806_rf_phy_state *st = phy->state;
	u32 code;
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_RAW:
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int adrf6806_phy_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6806_rf_phy_state *st = phy->state;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ: {
		break;
	}
	}

	return -EINVAL;
}

//this is referenced in adrf6806_probe(), reporting general information (not channel specific)

static const struct iio_info adrf6806_phy_info = {
	.read_raw = &adrf6806_phy_read_raw,
	.write_raw = &adrf6806_phy_write_raw,
	.read_avail = adrf6806_phy_read_avail,
	.attrs = &adrf6806_phy_attribute_group,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,0,0)
	.driver_module = THIS_MODULE,
#endif	
};

//here starts stuff dealing with the extended info

static ssize_t adrf6806_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6806_rf_phy_state *st = phy->state;
	u64 readin;
	bool res;
	int ret = 0;
	u32 u32_ret = 0;

	switch (private) {
		case LOEXT_FREQ:
				ret = kstrtoull(buf, 10, &readin);
			if (ret)
				return ret;
			break;
	}

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		st->tune_freq_Hz = (u32)readin;
		u32_ret = adrf6806_tune(st, st->tune_freq_Hz);
		if (u32_ret==0)	//tuning failed, because out of range
			ret = -EINVAL;
		else
			ret = 0;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t adrf6806_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct adrf6806_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6806_rf_phy_state *st = phy->state;
	u64 val = 0;
	size_t len;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		val = st->lo_freq_Hz;
		break;

	default:
		ret = 0;

	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}


#define _ADRF6806_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrf6806_phy_lo_read, \
	.write = adrf6806_phy_lo_write, \
	.private = _ident, \
}

#define _ADRF6806_EXT_LO_INFO_RO(_name, _ident) { \
	.name = _name, \
	.read = adrf6806_phy_lo_read, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info adrf6806_phy_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRF6806_EXT_LO_INFO("frequency", LOEXT_FREQ),
	_ADRF6806_EXT_LO_INFO_RO("frequency_available", LOEXT_FREQ_AVAILABLE),
	{ },
};


//this struct is referenced in adrf6806_probe()
//it reports one channel with extended information

static const struct iio_chan_spec adrf6806_phy_chan[] = {
{	/* RX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.extend_name = "RX_LO",
	.ext_info = adrf6806_phy_ext_info,
}
};







static int adrf6806_setup(struct adrf6806_rf_phy *phy)
{
	struct adrf6806_rf_phy_state *st = phy->state;
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	struct adrf6806_phy_platform_data *pd = phy->pdata;
	int ret;


	return 0;

}



static int adrf6806_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrf6806_rf_phy_state *st;
	struct adrf6806_rf_phy *phy;
	int ret, rev;

	dev_info(&spi->dev, "%s : enter (%s)", __func__,
		 spi_get_device_id(spi)->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->state = st;
	phy->indio_dev = indio_dev;
	phy->spi = spi;

	adrf6806_init_state(phy);

	phy->pdata = adrf6806_phy_parse_dt(indio_dev, &spi->dev);
	if (phy->pdata == NULL)
		return -EINVAL;

	//adrf6806_reset(phy);

	ret = adrf6806_setup(phy);

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = "adrf6806";

	indio_dev->info = &adrf6806_phy_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adrf6806_phy_chan;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_iio_device_unregister;

	dev_info(&spi->dev, "%s : ADRF6806 successfully initialized", __func__);

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);

	return ret;
}

static int adrf6806_remove(struct spi_device *spi)
{
	struct adrf6806_rf_phy *phy = adrf6806_spi_to_phy(spi);

	//do something

	return 0;
}

static const struct spi_device_id adrf6806_id[] = {
	{"adrf6806", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, adrf6806_id);

static struct spi_driver adrf6806_driver = {
	.driver = {
		.name	= "adrf6806",
		.owner	= THIS_MODULE,
	},
	.probe		= adrf6806_probe,
	.remove		= adrf6806_remove,
	.id_table	= adrf6806_id,
};
module_spi_driver(adrf6806_driver);

MODULE_AUTHOR("Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("Analog Devices ADRF6806 Receiver");
MODULE_LICENSE("GPL v2");
