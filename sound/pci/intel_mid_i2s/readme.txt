Merge of Nokia Maemo cmt-speech driver and Intel Moblin ifx-speech port driver

cmt-speeech on Maemo  uses OMAP  SSI physical interface
ifx-speeech on Moblin uses INTEL SSP physical interface (SSP configured as I2S). Currently supported modem: IFX 6160/6260.

upper interface out of Maemo: cs-core.c (re-use)
Maemo to Moblin mapping: cs-ssp.c
lower interface out of Moblin: ifx_i2s.c (re-use)
