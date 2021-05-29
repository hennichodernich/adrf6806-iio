#ifndef IIO_FREQUENCY_ADRF6806_H_
#define IIO_FREQUENCY_ADRF6806_H_

#include "adrf6806_defs.h"

struct adrf6806_phy_platform_data {
	int dummy;
};


struct adrf6806_rf_phy_state {
	  struct adrf6806_rf_phy *phy;
	  u32 tune_freq_Hz;
	  u16 INT;		//id
	  u8 DIV_MODE;
	  u16 MOD;		//md
	  u16 FRAC;		//fd
	  u16 DITH_MAG;
	  u8 DITH_EN;
	  u8 DITH_RESTART_VAL;
	  u8 REF_MUX_SEL;
	  u8 REF_INPUT;	//ref_in_path
	  u8 CP_REF;		//cp_iref_src
	  u8 PFD_Polarity;	//pfd_phs_off_pol
	  u8 PFD_PH_OFFS;	//pfd_phs_off_mult
	  u8 CP_CURR_MUL;	//cp_imult
	  u8 CP_CTL_SRC;
	  u8 CPCTRL;
	  u8 PFD_EDGE_SENS;	//div_path_edge ref_path_edge
	  u8 PFD_ABLDLY;
	  u8 DEMOD_BIAS_EN;	//mod_bias_en
	  u8 LP_EN;		//low_power_mode_6806
	  u8 LO_INOUT_CTL;	//ext_lo_en
	  u8 LO_DRV_EN;
	  u8 CP_EN;
	  u8 L3_EN;
	  u8 LV_EN;
	  u8 VCO_EN;
	  u8 VCO_SW;
	  u8 VCO_AMPL;
	  u8 VCO_BS_SRC;	//vco_bs_sw
	  u8 VCO_BANDSEL;	//vco_bs_sel_spi
	  u8 DIV_AB;	//scan_en
	  u8 DIV_SEL;	//dmod_div_6806
	  u8 ODIV_CTL;	//ext_lo_div_6806
	  u32 ref_in_Hz;
	  u16 pll_ref_div;
	  u16 in_mult;
	  u32 pfd_freq_Hz;
	  u32 lo_freq_Hz;
	  u32 lo_out_freq_Hz;
	  u32 vco_freq_Hz;
	  u16 ext_lo_divider;
	  u8 lo_divider;
	  u8 currmul_val;
};


struct adrf6806_rf_phy {
	struct spi_device 	*spi;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	struct adrf6806_rf_phy_state	*state;
	struct adrf6806_phy_platform_data *pdata;
};

enum lo_ext_info {
	LOEXT_FREQ,
	LOEXT_FREQ_AVAILABLE,
};


int adrf6806_spi_write(struct spi_device *spi, u8 reg, u32 val);
static int __adrf6806_of_get_u32(struct iio_dev *indio_dev,
			     struct device_node *np, const char *propname,
			     u32 defval, void *out_value, u32 size);

#define adrf6806_of_get_u32(iodev, dnp, name, def, outp) \
	__adrf6806_of_get_u32(iodev, dnp, name, def, outp, sizeof(*outp))

static void adrf6806_of_get_bool(struct iio_dev *indio_dev, struct device_node *np,
			       const char *propname, bool *out_value);

static struct adrf6806_phy_platform_data
	*adrf6806_phy_parse_dt(struct iio_dev *iodev, struct device *dev);

struct adrf6806_rf_phy* adrf6806_spi_to_phy(struct spi_device *spi);

static void adrf6806_init_state(struct adrf6806_rf_phy *phy);
static int adrf6806_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m);
static int adrf6806_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask);

static int adrf6806_phy_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask);
static ssize_t adrf6806_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len);

static ssize_t adrf6806_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf);
static int adrf6806_setup(struct adrf6806_rf_phy *phy);
static int adrf6806_probe(struct spi_device *spi);
static int adrf6806_remove(struct spi_device *spi);

#endif

