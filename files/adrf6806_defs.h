#ifndef ADRF6806_H
#define ADRF6806_H

#define ADRF6806_REG0	0x00
#define ADRF6806_REG1	0x01
#define ADRF6806_REG2	0x02
#define ADRF6806_REG3	0x03
#define ADRF6806_REG4	0x04
#define ADRF6806_REG5	0x05
#define ADRF6806_REG6	0x06
#define ADRF6806_REG7	0x07

//0 INT_DIV
#define ADRF6806_FLAG_DIV_MODE          0x0400
#define ADRF6806_BITS_INT_DIV(n)        ((n & 0x007f) << 3)
//1 MOD
#define ADRF6806_BITS_MOD(n)		((n & 0x07ff) << 3)
//2 FRAC
#define ADRF6806_BITS_FRAC(n)		((n & 0x07ff) << 3)
//3 DITHER
#define ADRF6806_BITS_DITH_MAG(n)	((n & 0x03) << 21)
#define ADRF6806_FLAG_DITH_EN		(1 << 20)
#define ADRF6806_BITS_DITH_RESTART(n)	((n & 0x01ffff) << 3)
//4 CP_PFD
#define ADRF6806_BITS_REF_MUX_SEL(n)	((n & 0x07) << 21)
#define ADRF6806_BITS_INPUT_REF(n)	((n & 0x03) << 19)
#define ADRF6806_FLAG_CP_REF		(1 << 18)
#define ADRF6806_FLAG_PFD_POL		(1 << 17)
#define ADRF6806_BITS_PFD_PH_OFFS(n)	((n & 0x1f) << 12)
#define ADRF6806_BITS_CP_CURRMUL(n)	((n & 0x03) << 10)
#define ADRF6806_FLAG_CP_CTL_SRC	(1 << 9)
#define ADRF6806_BITS_CP_CTL(n)		((n & 0x03) << 7)
#define ADRF6806_BITS_PFD_EDGE_SENS(n)	((n & 0x03) << 5)
#define ADRF6806_BITS_PFD_ABD(n)	((n & 0x03) << 3)
//5 LO_DEMOD
#define ADRF6806_FLAG_DEMOD_BIAS_EN	(1 << 7)
#define ADRF6806_FLAG_LP_EN		(1 << 5)
#define ADRF6806_FLAG_LO_INOUT_CTL	(1 << 4)
#define ADRF6806_FLAG_LO_DRV_EN		(1 << 3)
//6 VCO_CONTROL
#define ADRF6806_FLAG_CP_EN		(1 << 20)
#define ADRF6806_FLAG_L3_EN		(1 << 19)
#define ADRF6806_FLAG_LV_EN		(1 << 18)
#define ADRF6806_FLAG_VCO_EN		(1 << 17)
#define ADRF6806_FLAG_VCO_SW		(1 << 16)
#define ADRF6806_BITS_VCO_AMPL(n)	((n & 0x3f) << 10)
#define ADRF6806_FLAG_VCO_BS_SRC	(1 << 9)
#define ADRF6806_BITS_VCO_BANDSEL(n)	((n & 0x3f) << 3)
//7 LO_DIV_CONTROL
#define ADRF6806_BITS_DIV_AB(n)		((n & 0x03) << 8)
#define ADRF6806_BITS_DIV_SEL(n)	((n & 0x03) << 6)
#define ADRF6806_BITS_ODIV_CTL(n)	((n & 0x03) << 4)

typedef struct{
    uint8_t reg;
    char *name;
}t_regnamepair;

typedef struct{
  double tune_freq;
  int INT;		//id
  int DIV_MODE;		
  int MOD;		//md
  int FRAC;		//fd
  int DITH_MAG;
  int DITH_EN;
  int DITH_RESTART_VAL;
  int REF_MUX_SEL;
  int REF_INPUT;	//ref_in_path
  int CP_REF;		//cp_iref_src
  int PFD_Polarity;	//pfd_phs_off_pol
  int PFD_PH_OFFS;	//pfd_phs_off_mult
  int CP_CURR_MUL;	//cp_imult
  int CP_CTL_SRC;
  int CPCTRL;
  int PFD_EDGE_SENS;	//div_path_edge ref_path_edge
  int PFD_ABLDLY;
  int DEMOD_BIAS_EN;	//mod_bias_en
  int LP_EN;		//low_power_mode_6806
  int LO_INOUT_CTL;	//ext_lo_en
  int LO_DRV_EN;
  int CP_EN;
  int L3_EN;
  int LV_EN;
  int VCO_EN;
  int VCO_SW;
  int VCO_AMPL;
  int VCO_BS_SRC;	//vco_bs_sw
  int VCO_BANDSEL;	//vco_bs_sel_spi
  int DIV_AB;	//scan_en
  int DIV_SEL;	//dmod_div_6806
  int ODIV_CTL;	//ext_lo_div_6806
  double ref_in;
  double pll_ref_div;
  double in_mult;
  double pfd_freq;
  double lo_freq;
  double lo_out_freq;
  double vco_freq;
  int ext_lo_divider;
  int lo_divider;
  int currmul_val;
}t_adrf6806_settings;

#endif // ADRF6806_H
