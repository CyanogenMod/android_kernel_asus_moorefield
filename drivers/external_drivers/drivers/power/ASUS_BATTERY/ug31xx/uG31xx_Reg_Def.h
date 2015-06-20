/**
 * @filename RegDef.h
 *
 *  uG31xx register definition
 *
 * @author AllenTeng <allen.kuoliang.teng@gmail.com>
 * @revision  $Revision: 2 $
 * @note register table release date 2012/11/05
 */

#define INTERNAL_REGISTER_GROUP_A     (0x00)
  #define REG_MODE                    (INTERNAL_REGISTER_GROUP_A + 0)     ///< 0x00
    #define MODE_GG_RUN               (1<<4)
      #define GG_RUN_STANDBY_MODE     (0<<4)
      #define GG_RUN_OPERATION_MODE   (1<<4)
  #define REG_CTRL1                   (INTERNAL_REGISTER_GROUP_A + 1)     ///< 0x01
    #define CTRL1_PORDET              (1<<4)
      #define PORDET_R_NO_POR         (0<<4)
      #define PORDET_R_POR            (1<<4)
      #define PORDET_W_RELEASE        (0<<4)
      #define PORDET_W_SOFTRESET      (1<<4)
    #define CTRL1_VTM_EOC             (1<<3)
    #define CTRL1_GG_EOC              (1<<2)
    #define CTRL1_GG_RST              (1<<1)
    #define CTRL1_IO1DATA             (1<<0)
      #define IO1DATA_R_LOW           (0<<0)
      #define IO1DATA_R_HIGH          (1<<0)
      #define IO1DATA_W_LOW           (0<<0)
      #define IO1DATA_W_HIGH          (1<<0)

#define INTERNAL_REGISTER_GROUP_B     (0x02)
  #define REG_CHARGE_LOW              (INTERNAL_REGISTER_GROUP_B + 0)     ///< 0x02
  #define REG_CHARGE_HIGH             (INTERNAL_REGISTER_GROUP_B + 1)     ///< 0x03
  #define REG_COUNTER_LOW             (INTERNAL_REGISTER_GROUP_B + 2)     ///< 0x04
  #define REG_COUNTER_HIGH            (INTERNAL_REGISTER_GROUP_B + 3)     ///< 0x05
  #define REG_AVE_CURRENT_LOW         (INTERNAL_REGISTER_GROUP_B + 4)     ///< 0x06
  #define REG_AVE_CURRENT_HIGH        (INTERNAL_REGISTER_GROUP_B + 5)     ///< 0x07
  #define REG_AVE_VBAT1_LOW           (INTERNAL_REGISTER_GROUP_B + 6)     ///< 0x08
  #define REG_AVE_VBAT1_HIGH          (INTERNAL_REGISTER_GROUP_B + 7)     ///< 0x09
  #define REG_AVE_IT_LOW              (INTERNAL_REGISTER_GROUP_B + 8)     ///< 0x0A
  #define REG_AVE_IT_HIGH             (INTERNAL_REGISTER_GROUP_B + 9)     ///< 0x0B
  #define REG_AVE_OFFSET_CURRENT_LOW  (INTERNAL_REGISTER_GROUP_B + 10)    ///< 0x0C
  #define REG_AVE_OFFSET_CURRENT_HIGH (INTERNAL_REGISTER_GROUP_B + 11)    ///< 0x0D
  #define REG_AVE_ET_LOW              (INTERNAL_REGISTER_GROUP_B + 12)    ///< 0x0E
  #define REG_AVE_ET_HIGH             (INTERNAL_REGISTER_GROUP_B + 13)    ///< 0x0F
  #define REG_AVE_RID_LOW             (INTERNAL_REGISTER_GROUP_B + 14)    ///< 0x10
  #define REG_AVE_RID_HIGH            (INTERNAL_REGISTER_GROUP_B + 15)    ///< 0x11
  #define REG_ALARM1_STATUS           (INTERNAL_REGISTER_GROUP_B + 16)    ///< 0x12
    #define ALARM1_STATUS_COC_ALARM   (1<<5)
    #define ALARM1_STATUS_DOC_ALARM   (1<<4)
    #define ALARM1_STATUS_OIT_ALARM   (1<<3)
    #define ALARM1_STATUS_UIT_ALARM   (1<<2)
    #define ALARM1_STATUS_OET_ALARM   (1<<1)
    #define ALARM1_STATUS_UET_ALARM   (1<<0)
  #define REG_ALARM2_STATUS           (INTERNAL_REGISTER_GROUP_B + 17)    ///< 0x13
    #define ALARM2_STATUS_OV3_ALARM   (1<<5)
    #define ALARM2_STATUS_UV3_ALARM   (1<<4)
    #define ALARM2_STATUS_OV2_ALARM   (1<<3)
    #define ALARM2_STATUS_UV2_ALARM   (1<<2)
    #define ALARM2_STATUS_OV1_ALARM   (1<<1)
    #define ALARM2_STATUS_UV1_ALARM   (1<<0)
  #define REG_INTR_STATUS             (INTERNAL_REGISTER_GROUP_B + 18)    ///< 0x14
    #define INTR_STATUS_CBC_STS32     (1<<7)
      #define CBC_STS32_ENABLE        (0<<7)
      #define CBC_STS32_DISABLE       (1<<7)
    #define INTR_STATUS_CBC_STS21     (1<<6)
      #define CBC_STS21_ENABLE        (0<<6)
      #define CBC_STS21_DISABLE       (1<<6)
    #define INTR_STATUS_STB_STS       (1<<5)
    #define INTR_STATUS_ET_STS        (1<<4)
    #define INTR_STATUS_IT_STS        (1<<3)
    #define INTR_STATUS_RID_STS       (1<<2)
    #define INTR_STATUS_LVD_STS       (1<<1)
    #define INTR_STATUS_AL_STS        (1<<0)
  #define REG_ALARM_EN                (INTERNAL_REGISTER_GROUP_B + 19)    ///< 0x15
    #define ALARM_EN_COC_ALARM_EN     (1<<7)
    #define ALARM_EN_DOC_ALARM_EN     (1<<6)
    #define ALARM_EN_IT_ALARM_EN      (1<<5)
    #define ALARM_EN_ET_ALARM_EN      (1<<4)
    #define ALARM_EN_DECIMATE_RST     (1<<3)
    #define ALARM_EN_V3_ALARM_EN      (1<<2)
    #define ALARM_EN_V2_ALARM_EN      (1<<1)
    #define ALARM_EN_V1_ALARM_EN      (1<<0)
  #define REG_CTRL2                   (INTERNAL_REGISTER_GROUP_B + 20)    ///< 0x16
    #define CTRL2_IO4DATA             (1<<2)
      #define IO4DATA_R_LOW           (0<<2)
      #define IO4DATA_R_HIGH          (1<<2)
      #define IO4DATA_W_LOW           (0<<2)
      #define IO4DATA_W_HIGH          (1<<2)
    #define CTRL2_IO3DATA             (1<<1)
      #define IO3DATA_R_LOW           (0<<1)
      #define IO3DATA_R_HIGH          (1<<1)
      #define IO3DATA_W_LOW           (0<<1)
      #define IO3DATA_W_HIGH          (1<<1)
    #define CTRL2_IO2DATA             (1<<0)
      #define IO2DATA_R_LOW           (0<<0)
      #define IO2DATA_R_HIGH          (1<<0)
      #define IO2DATA_W_LOW           (0<<0)
      #define IO2DATA_W_HIGH          (1<<0)

#define INTERNAL_REGISTER_GROUP_C     (0x20)
  #define REG_RAM0                    (INTERNAL_REGISTER_GROUP_C + 0)     ///< 0x20
  #define REG_RAM1                    (INTERNAL_REGISTER_GROUP_C + 1)     ///< 0x21
  #define REG_RAM2                    (INTERNAL_REGISTER_GROUP_C + 2)     ///< 0x22
  #define REG_RAM3                    (INTERNAL_REGISTER_GROUP_C + 3)     ///< 0x23
  #define REG_RAM4                    (INTERNAL_REGISTER_GROUP_C + 4)     ///< 0x24
  #define REG_RAM5                    (INTERNAL_REGISTER_GROUP_C + 5)     ///< 0x25
  #define REG_RAM6                    (INTERNAL_REGISTER_GROUP_C + 6)     ///< 0x26
  #define REG_RAM7                    (INTERNAL_REGISTER_GROUP_C + 7)     ///< 0x27
  #define REG_RAM8                    (INTERNAL_REGISTER_GROUP_C + 8)     ///< 0x28
  #define REG_RAM9                    (INTERNAL_REGISTER_GROUP_C + 9)     ///< 0x29
  #define REG_RAM10                   (INTERNAL_REGISTER_GROUP_C + 10)    ///< 0x2A
  #define REG_RAM11                   (INTERNAL_REGISTER_GROUP_C + 11)    ///< 0x2B
  #define REG_RAM12                   (INTERNAL_REGISTER_GROUP_C + 12)    ///< 0x2C
  #define REG_RAM13                   (INTERNAL_REGISTER_GROUP_C + 13)    ///< 0x2D
  #define REG_RAM14                   (INTERNAL_REGISTER_GROUP_C + 14)    ///< 0x2E
  #define REG_RAM15                   (INTERNAL_REGISTER_GROUP_C + 15)    ///< 0x2F
  #define REG_RAM16                   (INTERNAL_REGISTER_GROUP_C + 16)    ///< 0x30
  #define REG_RAM17                   (INTERNAL_REGISTER_GROUP_C + 17)    ///< 0x31
  #define REG_RAM18                   (INTERNAL_REGISTER_GROUP_C + 18)    ///< 0x32
  #define REG_RAM19                   (INTERNAL_REGISTER_GROUP_C + 19)    ///< 0x33
  #define REG_RAM20                   (INTERNAL_REGISTER_GROUP_C + 20)    ///< 0x34
  #define REG_RAM21                   (INTERNAL_REGISTER_GROUP_C + 21)    ///< 0x35
  #define REG_RAM22                   (INTERNAL_REGISTER_GROUP_C + 22)    ///< 0x36
  #define REG_RAM23                   (INTERNAL_REGISTER_GROUP_C + 23)    ///< 0x37
  #define REG_RAM24                   (INTERNAL_REGISTER_GROUP_C + 24)    ///< 0x38
  #define REG_RAM25                   (INTERNAL_REGISTER_GROUP_C + 25)    ///< 0x39
  #define REG_RAM26                   (INTERNAL_REGISTER_GROUP_C + 26)    ///< 0x3A
  #define REG_RAM27                   (INTERNAL_REGISTER_GROUP_C + 27)    ///< 0x3B
  #define REG_RAM28                   (INTERNAL_REGISTER_GROUP_C + 28)    ///< 0x3C
  #define REG_RAM29                   (INTERNAL_REGISTER_GROUP_C + 29)    ///< 0x3D
  #define REG_RAM30                   (INTERNAL_REGISTER_GROUP_C + 30)    ///< 0x3E
  #define REG_RAM31                   (INTERNAL_REGISTER_GROUP_C + 31)    ///< 0x3F

#define INTERNAL_REGISTER_GROUP_D     (0x40)
  #define REG_VBAT2_LOW               (INTERNAL_REGISTER_GROUP_D + 0)     ///< 0x40
  #define REG_VBAT2_HIGH              (INTERNAL_REGISTER_GROUP_D + 1)     ///< 0x41
  #define REG_VBAT3_LOW               (INTERNAL_REGISTER_GROUP_D + 2)     ///< 0x42
  #define REG_VBAT3_HIGH              (INTERNAL_REGISTER_GROUP_D + 3)     ///< 0x43
  #define REG_VBAT1_LOW               (INTERNAL_REGISTER_GROUP_D + 4)     ///< 0x44
  #define REG_VBAT1_HIGH              (INTERNAL_REGISTER_GROUP_D + 5)     ///< 0x45
  #define REG_VBAT2_AVE_LOW           (INTERNAL_REGISTER_GROUP_D + 6)     ///< 0x46
  #define REG_VBAT2_AVE_HIGH          (INTERNAL_REGISTER_GROUP_D + 7)     ///< 0x47
  #define REG_VBAT3_AVE_LOW           (INTERNAL_REGISTER_GROUP_D + 8)     ///< 0x48
  #define REG_VBAT3_AVE_HIGH          (INTERNAL_REGISTER_GROUP_D + 9)     ///< 0x49
  #define REG_V1_LOW                  (INTERNAL_REGISTER_GROUP_D + 10)    ///< 0x4A
  #define REG_V1_HIGH                 (INTERNAL_REGISTER_GROUP_D + 11)    ///< 0x4B
  #define REG_V2_LOW                  (INTERNAL_REGISTER_GROUP_D + 12)    ///< 0x4C
  #define REG_V2_HIGH                 (INTERNAL_REGISTER_GROUP_D + 13)    ///< 0x4D
  #define REG_V3_LOW                  (INTERNAL_REGISTER_GROUP_D + 14)    ///< 0x4E
  #define REG_V3_HIGH                 (INTERNAL_REGISTER_GROUP_D + 15)    ///< 0x4F
  #define REG_INTR_TEMPER_LOW         (INTERNAL_REGISTER_GROUP_D + 16)    ///< 0x50
  #define REG_INTR_TEMPER_HIGH        (INTERNAL_REGISTER_GROUP_D + 17)    ///< 0x51
  #define REG_EXTR_TEMPER_LOW         (INTERNAL_REGISTER_GROUP_D + 18)    ///< 0x52
  #define REG_EXTR_TEMPER_HIGH        (INTERNAL_REGISTER_GROUP_D + 19)    ///< 0x53
  #define REG_RID_LOW                 (INTERNAL_REGISTER_GROUP_D + 20)    ///< 0x54
  #define REG_RID_HIGH                (INTERNAL_REGISTER_GROUP_D + 21)    ///< 0x55
  #define REG_CURRENT_LOW             (INTERNAL_REGISTER_GROUP_D + 22)    ///< 0x56
  #define REG_CURRENT_HIGH            (INTERNAL_REGISTER_GROUP_D + 23)    ///< 0x57
  #define REG_ADC1_OFFSET_LOW         (INTERNAL_REGISTER_GROUP_D + 24)    ///< 0x58
  #define REG_ADC1_OFFSET_HIGH        (INTERNAL_REGISTER_GROUP_D + 25)    ///< 0x59

  #define OTP6_BYTE1                  (INTERNAL_REGISTER_GROUP_D + 48)    ///< 0x70
    #define AVE_IT25_7_3              (63<<3)
    #define DELTA_ET_3_2              (3<<1)
    #define DELTA_VREF_4              (1<<0)
  #define OTP6_BYTE2                  (INTERNAL_REGISTER_GROUP_D + 49)    ///< 0x71
    #define AVE_IT25_15_8             (255<<0)
  #define OTP6_BYTE3                  (INTERNAL_REGISTER_GROUP_D + 50)    ///< 0x72
    #define AVE_IT80_7_3              (63<<3)
    #define INDEX_ADC1_200_25_4       (1<<2)
    #define INDEX_ADC1_100_25_4       (1<<1)
    #define INDEX_ADC2_200_25_4       (1<<0)
  #define OTP6_BYTE4                  (INTERNAL_REGISTER_GROUP_D + 51)    ///< 0x73
    #define AVE_IT80_15_8             (255<<0)

#define INTERNAL_REGISTER_GROUP_E     (0x8F)
  #define REG_TIMER                   (INTERNAL_REGISTER_GROUP_E + 0)     ///< 0x8F
    #define TIMER_TIMER_ITSET         (3<<6)
      #define TIMER_ITSET_4           (0<<6)
      #define TIMER_ITSET_9           (1<<6)
      #define TIMER_ITSET_14          (2<<6)
      #define TIMER_ITSET_19          (3<<6)
    #define TIMER_TIMER_ETSET         (3<<4)
      #define TIMER_ETSET_4           (0<<4)
      #define TIMER_ETSET_9           (1<<4)
      #define TIMER_ETSET_14          (2<<4)
      #define TIMER_ETSET_19          (3<<4)
    #define TIMER_TIMER_VSET          (3<<2)
      #define TIMER_VSET_4            (0<<2)
      #define TIMER_VSET_9            (1<<2)
      #define TIMER_VSET_14           (2<<2)
      #define TIMER_VSET_19           (3<<2)
    #define TIMER_TIMER_CSET          (3<<0)
      #define TIMER_CSET_4            (0<<0)
      #define TIMER_CSET_9            (1<<0)
      #define TIMER_CSET_14           (2<<0)
      #define TIMER_CSET_19           (3<<0)
  #define REG_CLK_DIVA                (INTERNAL_REGISTER_GROUP_E + 1)     ///< 0x90
    #define CLK_DIVA_FW_CLK_CDIV      (15<<4)
      #define FW_CLK_CDIV_31US        (0<<4)
      #define FW_CLK_CDIV_61US        (1<<4)
      #define FW_CLK_CDIV_122US       (2<<4)
      #define FW_CLK_CDIV_244US       (3<<4)
      #define FW_CLK_CDIV_488US       (4<<4)
      #define FW_CLK_CDIV_977US       (5<<4)
      #define FW_CLK_CDIV_2MS         (6<<4)
      #define FW_CLK_CDIV_4MS         (7<<4)
      #define FW_CLK_CDIV_8MS         (8<<4)
      #define FW_CLK_CDIV_16MS        (9<<4)
      #define FW_CLK_CDIV_31MS        (10<<4)
      #define FW_CLK_CDIV_62MS        (11<<4)
      #define FW_CLK_CDIV_125MS       (12<<4)
      #define FW_CLK_CDIV_250MS       (13<<4)
      #define FW_CLK_CDIV_500MS       (14<<4)
      #define FW_CLK_CDIV_1S          (15<<4)
    #define CLK_DIVA_FW_CLK_VDIV      (15<<0)
      #define FW_CLK_VDIV_31US        (0<<0)
      #define FW_CLK_VDIV_61US        (1<<0)
      #define FW_CLK_VDIV_122US       (2<<0)
      #define FW_CLK_VDIV_244US       (3<<0)
      #define FW_CLK_VDIV_488US       (4<<0)
      #define FW_CLK_VDIV_977US       (5<<0)
      #define FW_CLK_VDIV_2MS         (6<<0)
      #define FW_CLK_VDIV_4MS         (7<<0)
      #define FW_CLK_VDIV_8MS         (8<<0)
      #define FW_CLK_VDIV_16MS        (9<<0)
      #define FW_CLK_VDIV_31MS        (10<<0)
      #define FW_CLK_VDIV_62MS        (11<<0)
      #define FW_CLK_VDIV_125MS       (12<<0)
      #define FW_CLK_VDIV_250MS       (13<<0)
      #define FW_CLK_VDIV_500MS       (14<<0)
      #define FW_CLK_VDIV_1S          (15<<0)
  #define REG_CLK_DIVB                (INTERNAL_REGISTER_GROUP_E + 2)     ///< 0x91
    #define CLK_DIVB_FW_CLK_ETDIV     (15<<4)
      #define FW_CLK_ETDIV_31US       (0<<4)
      #define FW_CLK_ETDIV_61US       (1<<4)
      #define FW_CLK_ETDIV_122US      (2<<4)
      #define FW_CLK_ETDIV_244US      (3<<4)
      #define FW_CLK_ETDIV_488US      (4<<4)
      #define FW_CLK_ETDIV_977US      (5<<4)
      #define FW_CLK_ETDIV_2MS        (6<<4)
      #define FW_CLK_ETDIV_4MS        (7<<4)
      #define FW_CLK_ETDIV_8MS        (8<<4)
      #define FW_CLK_ETDIV_16MS       (9<<4)
      #define FW_CLK_ETDIV_31MS       (10<<4)
      #define FW_CLK_ETDIV_62MS       (11<<4)
      #define FW_CLK_ETDIV_125MS      (12<<4)
      #define FW_CLK_ETDIV_250MS      (13<<4)
      #define FW_CLK_ETDIV_500MS      (14<<4)
      #define FW_CLK_ETDIV_1S         (15<<4)
    #define CLK_DIVB_FW_CLK_ITDIV     (15<<0)
      #define FW_CLK_ITDIV_31US       (0<<0)
      #define FW_CLK_ITDIV_61US       (1<<0)
      #define FW_CLK_ITDIV_122US      (2<<0)
      #define FW_CLK_ITDIV_244US      (3<<0)
      #define FW_CLK_ITDIV_488US      (4<<0)
      #define FW_CLK_ITDIV_977US      (5<<0)
      #define FW_CLK_ITDIV_2MS        (6<<0)
      #define FW_CLK_ITDIV_4MS        (7<<0)
      #define FW_CLK_ITDIV_8MS        (8<<0)
      #define FW_CLK_ITDIV_16MS       (9<<0)
      #define FW_CLK_ITDIV_31MS       (10<<0)
      #define FW_CLK_ITDIV_62MS       (11<<0)
      #define FW_CLK_ITDIV_125MS      (12<<0)
      #define FW_CLK_ITDIV_250MS      (13<<0)
      #define FW_CLK_ITDIV_500MS      (14<<0)
      #define FW_CLK_ITDIV_1S         (15<<0)
  #define REG_INTR_CTRL_D             (INTERNAL_REGISTER_GROUP_E + 3)     ///< 0x92
    #define INTR_CTRL_D_GPIO4_SEL     (7<<3)
      #define GPIO4_SEL_GPIO1         (0<<3)
      #define GPIO4_SEL_ALARM         (1<<3)
      #define GPIO4_SEL_CBC_EN21      (2<<3)
      #define GPIO4_SEL_CBC_EN32      (3<<3)
      #define GPIO4_SEL_PWM           (4<<3)
      #define GPIO4_SEL_ADC1_D        (6<<3)
      #define GPIO4_SEL_ADC2_D        (7<<3)
    #define INTR_CTRL_D_GPIO3_SEL     (7<<0)
      #define GPIO3_SEL_GPIO1         (0<<0)
      #define GPIO3_SEL_ALARM         (1<<0)
      #define GPIO3_SEL_CBC_EN21      (2<<0)
      #define GPIO3_SEL_CBC_EN32      (3<<0)
      #define GPIO3_SEL_PWM           (4<<0)
      #define GPIO3_SEL_ADC1_D        (6<<0)
      #define GPIO3_SEL_ADC2_D        (7<<0)
  #define OSCTUNE_J1                  (INTERNAL_REGISTER_GROUP_E + 4)     ///< 0x93
    #define OSCTUNE_J_LOW             (255<<0)
  #define OSCTUNE_J2                  (INTERNAL_REGISTER_GROUP_E + 5)     ///< 0x94
    #define OSCTUNE_J_HIGH            (3<<0)
  #define OSCTUNE_K1                  (INTERNAL_REGISTER_GROUP_E + 6)     ///< 0x95
    #define OSCTUNE_K_LOW             (255<<0)
  #define OSCTUNE_K2                  (INTERNAL_REGISTER_GROUP_E + 7)     ///< 0x96
    #define OSCTUNE_K_HIGH            (3<<0)
  #define OSCTUNE_CNTA                (INTERNAL_REGISTER_GROUP_E + 8)     ///< 0x97
    #define OSCTUNE_CNT_LOW           (255<<0)
  #define OSCTUNE_CNTB                (INTERNAL_REGISTER_GROUP_E + 9)     ///< 0x98
    #define OSCTUNE_CNT_HIGH          (3<<0)
  #define ADC1_TARGET_A               (INTERNAL_REGISTER_GROUP_E + 10)    ///< 0x99
    #define TARGET_A_LOW              (255<<0)
  #define ADC1_TARGET_B               (INTERNAL_REGISTER_GROUP_E + 11)    ///< 0x9A
    #define TARGET_A_HIGH             (3<<0)
  #define REG_INTR_CTRL_A             (INTERNAL_REGISTER_GROUP_E + 12)    ///< 0x9B
    #define INTR_CTRL_A_GPIO2_SEL     (7<<5)
      #define GPIO2_SEL_GPIO1         (0<<5)
      #define GPIO2_SEL_ALARM         (1<<5)
      #define GPIO2_SEL_CBC_EN21      (2<<5)
      #define GPIO2_SEL_CBC_EN32      (3<<5)
      #define GPIO2_SEL_PWM           (4<<5)
    #define INTR_CTRL_A_GPIO1_SEL     (7<<2)
      #define GPIO1_SEL_GPIO1         (0<<2)
      #define GPIO1_SEL_ALARM         (1<<2)
      #define GPIO1_SEL_CBC_EN21      (2<<2)
      #define GPIO1_SEL_CBC_EN32      (3<<2)
      #define GPIO1_SEL_PWM           (4<<2)
      #define GPIO1_SEL_OSCOUT        (7<<2)
    #define INTR_CTRL_A_ADC2_EN       (1<<1)
    #define INTR_CTRL_A_ADC1_EN       (1<<0)
  #define REG_INTR_CTRL_B             (INTERNAL_REGISTER_GROUP_E + 13)    ///< 0x9C
    #define INTR_CTRL_B_OSC_CNT_EN    (1<<7)
    #define INTR_CTRL_B_PWM_EN        (1<<6)
    #define INTR_CTRL_B_CBC_32_EN     (1<<5)
    #define INTR_CTRL_B_CBC_21_EN     (1<<4)
    #define INTR_CTRL_B_STB_EN        (1<<3)
    #define INTR_CTRL_B_ET_EN         (1<<2)
    #define INTR_CTRL_B_IT_EN         (1<<1)
    #define INTR_CTRL_B_RID_EN        (1<<0)
  #define REG_ITNR_CTRL_C             (INTERNAL_REGISTER_GROUP_E + 14)    ///< 0x9D
    #define INTR_CTRL_C_BGRCAL_FINISH (1<<7)
    #define INTR_CTRL_C_FW_V_DIVIDE   (1<<6)
    #define INTR_CTRL_C_BGRCAL_START  (1<<5)
    #define INTR_CTRL_C_BGR_CALEN     (1<<4)
    #define INTR_CTRL_C_PWM_SET       (3<<2)
      #define PWM_SET_32K             (0<<2)
      #define PWM_SET_16K             (1<<2)
      #define PWM_SET_8K              (2<<2)
      #define PWM_SET_4K              (3<<2)
    #define INTR_CTRL_C_TIMER_SET     (3<<0)
      #define TIMER_SET_OV_UV         (0<<0)
      #define TIMER_SET_OC_UC         (1<<0)
      #define TIMER_SET_OIT_UIT       (2<<0)
      #define TIMER_SET_OET_UET       (3<<0)
  #define REG_CELL_EN                 (INTERNAL_REGISTER_GROUP_E + 15)    ///< 0x9E
    #define CELL_EN_APPLICATION       (7<<2)
      #define APPLICATION_UG3100      (0<<2)
      #define APPLICATION_UG3101      (1<<2)
      #define APPLICATION_UG3102      (2<<2)
      #define APPLICATION_UG3103_2    (4<<2)
      #define APPLICATION_UG3103_3    (5<<2)
    #define CELL_EN1                  (1<<1)
    #define CELL_EN0                  (1<<0)

#define INTERNAL_REGISTER_GROUP_F     (0x9F)
  #define REG_COC_LOW                 (INTERNAL_REGISTER_GROUP_F + 0)     ///< 0x9F
  #define REG_COC_HIGH                (INTERNAL_REGISTER_GROUP_F + 1)     ///< 0xA0
  #define REG_DOC_LOW                 (INTERNAL_REGISTER_GROUP_F + 2)     ///< 0xA1
  #define REG_DOC_HIGH                (INTERNAL_REGISTER_GROUP_F + 3)     ///< 0xA2
  #define REG_UC_LOW                  (INTERNAL_REGISTER_GROUP_F + 4)     ///< 0xA3
  #define REG_UC_HIGH                 (INTERNAL_REGISTER_GROUP_F + 5)     ///< 0xA4
  #define REG_OV1_LOW                 (INTERNAL_REGISTER_GROUP_F + 6)     ///< 0xA5
  #define REG_OV1_HIGH                (INTERNAL_REGISTER_GROUP_F + 7)     ///< 0xA6
  #define REG_UV1_LOW                 (INTERNAL_REGISTER_GROUP_F + 8)     ///< 0xA7
  #define REG_UV1_HIGH                (INTERNAL_REGISTER_GROUP_F + 9)     ///< 0xA8
  #define REG_OV2_LOW                 (INTERNAL_REGISTER_GROUP_F + 10)    ///< 0xA9
  #define REG_OV2_HIGH                (INTERNAL_REGISTER_GROUP_F + 11)    ///< 0xAA
  #define REG_UV2_LOW                 (INTERNAL_REGISTER_GROUP_F + 12)    ///< 0xAB
  #define REG_UV2_HIGH                (INTERNAL_REGISTER_GROUP_F + 13)    ///< 0xAC
  #define REG_OV3_LOW                 (INTERNAL_REGISTER_GROUP_F + 14)    ///< 0xAD
  #define REG_OV3_HIGH                (INTERNAL_REGISTER_GROUP_F + 15)    ///< 0xAE
  #define REG_UV3_LOW                 (INTERNAL_REGISTER_GROUP_F + 16)    ///< 0xAF
  #define REG_UV3_HIGH                (INTERNAL_REGISTER_GROUP_F + 17)    ///< 0xB0
  #define REG_OVP_LOW                 (INTERNAL_REGISTER_GROUP_F + 18)    ///< 0xB1
  #define REG_OVP_HIGH                (INTERNAL_REGISTER_GROUP_F + 19)    ///< 0xB2
  #define REG_UVP_LOW                 (INTERNAL_REGISTER_GROUP_F + 20)    ///< 0xB3
  #define REG_UVP_HIGH                (INTERNAL_REGISTER_GROUP_F + 21)    ///< 0xB4
  #define REG_INTR_OVER_TEMP_LOW      (INTERNAL_REGISTER_GROUP_F + 22)    ///< 0xB5
  #define REG_INTR_OVER_TEMP_HIGH     (INTERNAL_REGISTER_GROUP_F + 23)    ///< 0xB6
  #define REG_INTR_UNDER_TEMP_LOW     (INTERNAL_REGISTER_GROUP_F + 24)    ///< 0xB7
  #define REG_INTR_UNDER_TEMP_HIGH    (INTERNAL_REGISTER_GROUP_F + 25)    ///< 0xB8
  #define REG_EXTR_OVER_TEMP_LOW      (INTERNAL_REGISTER_GROUP_F + 26)    ///< 0xB9
  #define REG_EXTR_OVER_TEMP_HIGH     (INTERNAL_REGISTER_GROUP_F + 27)    ///< 0xBA
  #define REG_EXTR_UNDER_TEMP_LOW     (INTERNAL_REGISTER_GROUP_F + 28)    ///< 0xBB
  #define REG_EXTR_UNDER_TEMP_HIGH    (INTERNAL_REGISTER_GROUP_F + 29)    ///< 0xBC
  #define REG_CBC21_LOW               (INTERNAL_REGISTER_GROUP_F + 30)    ///< 0xBD
  #define REG_CBC21_HIGH              (INTERNAL_REGISTER_GROUP_F + 31)    ///< 0xBE
  #define REG_CBC32_LOW               (INTERNAL_REGISTER_GROUP_F + 32)    ///< 0xBF
  #define REG_CBC32_HIGH              (INTERNAL_REGISTER_GROUP_F + 33)    ///< 0xC0

#define INTERNAL_REGISTER_GROUP_G     (0xC1)
  #define REG_FW_CTRL                 (INTERNAL_REGISTER_GROUP_G + 0)     ///< 0xC1
    #define FW_CTRL_CHOP2_EN          (1<<3)
    #define FW_CTRL_CHOPPING2         (1<<2)
    #define FW_CTRL_CHOP1_EN          (1<<1)
    #define FW_CTRL_CHOPPING1         (1<<0)
  #define REG_OTP_CTRL                (INTERNAL_REGISTER_GROUP_G + 1)     ///< 0xC2
    #define OTP_CTRL_IT_CAL80         (1<<7)
    #define OTP_CTRL_IT_CAL25         (1<<6)
    #define OTP_CTRL_ADC2_200MV       (1<<5)
    #define OTP_CTRL_ADC2_100MV       (1<<4)
    #define OTP_CTRL_ADC1_200MV       (1<<3)
    #define OTP_CTRL_ADC1_100MV       (1<<2)
    #define OTP_CTRL_OTP_PTM          (3<<0)
  #define REG_OTP_PPROG_ON            (INTERNAL_REGISTER_GROUP_G + 2)     ///< 0xC3
    #define OTP_PPROG_ON_VALUE        (0xDD)
  #define REG_OTP_PPROG_OFF           (INTERNAL_REGISTER_GROUP_G + 3)     ///< 0xC4
    #define OTP_PPROG_OFF_VALUE       (0xDE)

#define INTERNAL_REGISTER_GROUP_H     (0xC5)
  #define REG_ADC_CTR_A               (INTERNAL_REGISTER_GROUP_H + 0)     ///< 0xC5
    #define ADC_CTR_A_SET_A           (3<<6)
      #define SET_A_CURRENT           (0<<6)
      #define SET_A_ET                (1<<6)
      #define SET_A_RID_IN            (2<<6)
      #define SET_A_IT                (3<<6)
    #define ADC_CTR_A_SET_B           (3<<4)
      #define SET_B_CURRENT           (0<<4)
      #define SET_B_ET                (1<<4)
      #define SET_B_RID_IN            (2<<4)
      #define SET_B_IT                (3<<4)
    #define ADC_CTR_A_SET_C           (3<<2)
      #define SET_C_CURRENT           (0<<2)
      #define SET_C_ET                (1<<2)
      #define SET_C_RID_IN            (2<<2)
      #define SET_C_IT                (3<<2)
    #define ADC_CTR_A_SET_D           (3<<0)
      #define SET_D_CURRENT           (0<<0)
      #define SET_D_ET                (1<<0)
      #define SET_D_RID_IN            (2<<0)
      #define SET_D_IT                (3<<0)
  #define REG_ADC_CTR_B               (INTERNAL_REGISTER_GROUP_H + 1)     ///< 0xC6
    #define ADC_CTR_B_SET_E           (3<<6)
      #define SET_E_CURRENT           (0<<6)
      #define SET_E_ET                (1<<6)
      #define SET_E_RID_IN            (2<<6)
      #define SET_E_IT                (3<<6)
    #define ADC_CTR_B_SET_F           (3<<4)
      #define SET_F_CURRENT           (0<<4)
      #define SET_F_ET                (1<<4)
      #define SET_F_RID_IN            (2<<4)
      #define SET_F_IT                (3<<4)
    #define ADC_CTR_B_SET_G           (3<<2)
      #define SET_G_CURRENT           (0<<2)
      #define SET_G_ET                (1<<2)
      #define SET_G_RID_IN            (2<<2)
      #define SET_G_IT                (3<<2)
    #define ADC_CTR_B_SET_H           (3<<0)
      #define SET_H_CURRENT           (0<<0)
      #define SET_H_ET                (1<<0)
      #define SET_H_RID_IN            (2<<0)
      #define SET_H_IT                (3<<0)
  #define REG_ADC_CTR_C               (INTERNAL_REGISTER_GROUP_H + 2)     ///< 0xC7
    #define ADC_CTR_C_SET_I           (3<<6)
      #define SET_I_CURRENT           (0<<6)
      #define SET_I_ET                (1<<6)
      #define SET_I_RID_IN            (2<<6)
      #define SET_I_IT                (3<<6)
    #define ADC_CTR_C_SET_J           (3<<4)
      #define SET_J_CURRENT           (0<<4)
      #define SET_J_ET                (1<<4)
      #define SET_J_RID_IN            (2<<4)
      #define SET_J_IT                (3<<4)
    #define ADC_CTR_C_SET_K           (3<<2)
      #define SET_K_CURRENT           (0<<2)
      #define SET_K_ET                (1<<2)
      #define SET_K_RID_IN            (2<<2)
      #define SET_K_IT                (3<<2)
    #define ADC_CTR_C_SET_L           (3<<0)
      #define SET_L_CURRENT           (0<<0)
      #define SET_L_ET                (1<<0)
      #define SET_L_RID_IN            (2<<0)
      #define SET_L_IT                (3<<0)
  #define REG_ADC_CTR_D               (INTERNAL_REGISTER_GROUP_H + 3)     ///< 0xC8
    #define ADC_CTR_D_SET_M           (3<<6)
      #define SET_M_CURRENT           (0<<6)
      #define SET_M_ET                (1<<6)
      #define SET_M_RID_IN            (2<<6)
      #define SET_M_IT                (3<<6)
    #define ADC_CTR_D_SET_N           (3<<4)
      #define SET_N_CURRENT           (0<<4)
      #define SET_N_ET                (1<<4)
      #define SET_N_RID_IN            (2<<4)
      #define SET_N_IT                (3<<4)
    #define ADC_CTR_D_SET_O           (3<<2)
      #define SET_O_CURRENT           (0<<2)
      #define SET_O_ET                (1<<2)
      #define SET_O_RID_IN            (2<<2)
      #define SET_O_IT                (3<<2)
    #define ADC_CTR_D_SET_P           (3<<0)
      #define SET_P_CURRENT           (0<<0)
      #define SET_P_ET                (1<<0)
      #define SET_P_RID_IN            (2<<0)
      #define SET_P_IT                (3<<0)
  #define REG_ADC_V1                  (INTERNAL_REGISTER_GROUP_H + 4)     ///< 0xC9
    #define ADC_CTR_V1_SET_V1         (3<<6)
      #define SET_V1_GND              (0<<6)
      #define SET_V1_VBAT1            (1<<6)
      #define SET_V1_VBAT2            (2<<6)
      #define SET_V1_VBAT3            (3<<6)
    #define ADC_CTR_V1_SET_V2         (3<<4)
      #define SET_V2_GND              (0<<4)
      #define SET_V2_VBAT1            (1<<4)
      #define SET_V2_VBAT2            (2<<4)
      #define SET_V2_VBAT3            (3<<4)
    #define ADC_CTR_V1_SET_V3         (3<<2)
      #define SET_V3_GND              (0<<2)
      #define SET_V3_VBAT1            (1<<2)
      #define SET_V3_VBAT2            (2<<2)
      #define SET_V3_VBAT3            (3<<2)
    #define ADC_CTR_V1_SET_V4         (3<<0)
      #define SET_V4_GND              (0<<0)
      #define SET_V4_VBAT1            (1<<0)
      #define SET_V4_VBAT2            (2<<0)
      #define SET_V4_VBAT3            (3<<0)
  #define REG_ADC_V2                  (INTERNAL_REGISTER_GROUP_H + 5)     ///< 0xCA
    #define ADC_CTR_V2_SET_V5         (3<<6)
      #define SET_V5_GND              (0<<6)
      #define SET_V5_VBAT1            (1<<6)
      #define SET_V5_VBAT2            (2<<6)
      #define SET_V5_VBAT3            (3<<6)
    #define ADC_CTR_V2_SET_V6         (3<<4)
      #define SET_V6_GND              (0<<4)
      #define SET_V6_VBAT1            (1<<4)
      #define SET_V6_VBAT2            (2<<4)
      #define SET_V6_VBAT3            (3<<4)
    #define ADC_CTR_V2_SET_V7         (3<<2)
      #define SET_V7_GND              (0<<2)
      #define SET_V7_VBAT1            (1<<2)
      #define SET_V7_VBAT2            (2<<2)
      #define SET_V7_VBAT3            (3<<2)
    #define ADC_CTR_V2_SET_V8         (3<<0)
      #define SET_V8_GND              (0<<0)
      #define SET_V8_VBAT1            (1<<0)
      #define SET_V8_VBAT2            (2<<0)
      #define SET_V8_VBAT3            (3<<0)
  #define REG_ADC_V3                  (INTERNAL_REGISTER_GROUP_H + 6)     ///< 0xCB
    #define ADC_CTR_V3_SET_V9         (3<<6)
      #define SET_V9_GND              (0<<6)
      #define SET_V9_VBAT1            (1<<6)
      #define SET_V9_VBAT2            (2<<6)
      #define SET_V9_VBAT3            (3<<6)
    #define ADC_CTR_V3_SET_V10        (3<<4)
      #define SET_V10_GND             (0<<4)
      #define SET_V10_VBAT1           (1<<4)
      #define SET_V10_VBAT2           (2<<4)
      #define SET_V10_VBAT3           (3<<4)
    #define ADC_CTR_V3_SET_V11        (3<<2)
      #define SET_V11_GND             (0<<2)
      #define SET_V11_VBAT1           (1<<2)
      #define SET_V11_VBAT2           (2<<2)
      #define SET_V11_VBAT3           (3<<2)
    #define ADC_CTR_V3_SET_V12        (3<<0)
      #define SET_V12_GND             (0<<0)
      #define SET_V12_VBAT1           (1<<0)
      #define SET_V12_VBAT2           (2<<0)
      #define SET_V12_VBAT3           (3<<0)

#define INTERNAL_REGISTER_GROUP_I     (0xCC)
  #define KCONFIG_D1                  (INTERNAL_REGISTER_GROUP_I + 0)     ///< 0xCC
    #define KCONFIG_D_LOW             (255<<0)
  #define KCONFIG_D2                  (INTERNAL_REGISTER_GROUP_I + 1)     ///< 0xCD
    #define KCONFIG_D_HIGH            (255<<0)

#define INTERNAL_REGISTER_GROUP_J     (0xCE)
  #define KCONFIG_A1                  (INTERNAL_REGISTER_GROUP_J + 0)     ///< 0xCE
    #define KCONFIG_A1_KGG1_OSC       (255<<0)
  #define KCONFIG_A2                  (INTERNAL_REGISTER_GROUP_J + 1)     ///< 0xCF
    #define KCONFIG_A2_KGG1_DSM2_L    (255<<0)
  #define KCONFIG_A3                  (INTERNAL_REGISTER_GROUP_J + 2)     ///< 0xD0
    #define KCONFIG_A3_KGG1_DSM2_M    (255<<0)
  #define KCONFIG_A4                  (INTERNAL_REGISTER_GROUP_J + 3)     ///< 0xD1
    #define KCONFIG_A4_KGG1_DSM2_H    (255<<0)
  #define KCONFIG_A5                  (INTERNAL_REGISTER_GROUP_J + 4)     ///< 0xD2
    #define KCONFIG_A5_KGG1_DSM1_L    (255<<0)
  #define KCONFIG_A6                  (INTERNAL_REGISTER_GROUP_J + 5)     ///< 0xD3
    #define KCONFIG_A6_KGG1_DSM1_M    (255<<0)
  #define KCONFIG_A7                  (INTERNAL_REGISTER_GROUP_J + 6)     ///< 0xD4
    #define KCONFIG_A7_KGG1_DSM1_H    (255<<0)
  #define KCONFIG_A8                  (INTERNAL_REGISTER_GROUP_J + 7)     ///< 0xD5
    #define KCONFIG_A8_KGG1_MBIAS_L   (255<<0)
  #define KCONFIG_A9                  (INTERNAL_REGISTER_GROUP_J + 8)     ///< 0xD6
    #define KCONFIG_A9_KGG1_MBIAS_H   (3<<0)

  #define KCONFIG_H1                  (INTERNAL_REGISTER_GROUP_J + 9)     ///< 0xD7
    #define KCONFIG_H1_KGG1_IDO_LOW   (15<<4)
    #define KCONFIG_H1_KGG1_MBIAS     (3<<2)
    #define KCONFIG_H1_KGG1_OSC       (3<<2)
  #define KCONFIG_H2                  (INTERNAL_REGISTER_GROUP_J + 10)    ///< 0xD8
    #define KCONFIG_H2_KGG1_IDO_HIGH  (255<<0)
  #define KCONFIG_H3                  (INTERNAL_REGISTER_GROUP_J + 11)    ///< 0xD9
    #define KCONFIG_H3_KGG1_BGAP_LOW  (63<<2)
    #define KCONFIG_H3_KGG1_DSM2      (3<<0)
  #define KCONFIG_H4                  (INTERNAL_REGISTER_GROUP_J + 12)    ///< 0xDA
    #define KCONFIG_H3_KGG1_BGAP_HIGH (127<<0)
  #define KCONFIG_H5                  (INTERNAL_REGISTER_GROUP_J + 13)    ///< 0xDB
    #define KCONFIG_H5_WIRE_V_DIVIDE  (1<<6)
    #define KCONFIG_H5_KGG1_BGAP_CAL  (63<<0)

#define INTERNAL_REGISTER_GROUP_K     (0xDC)
  #define KCONFIG_CAL1                (INTERNAL_REGISTER_GROUP_K + 0)     ///< 0xDC
    #define KCONFIG_CAL1_KGG1_OSC_L   (255<<0)
  #define KCONFIG_CAL2                (INTERNAL_REGISTER_GROUP_K + 1)     ///< 0xDD
    #define KCONFIG_CAL2_KGG1_OSC_H   (255<<0)
  #define KCONFIG_CAL3                (INTERNAL_REGISTER_GROUP_K + 2)     ///< 0xDE
    #define KCONFIG_CAL3_KGG1_DSM2    (255<<0)
  #define KCONFIG_CAL4                (INTERNAL_REGISTER_GROUP_K + 3)     ///< 0xDF
    #define KCONFIG_CAL4_KGG1_DSM1    (255<<0)

#define INTERNAL_REGISTER_GROUP_L     (0xE0)
  #define OTP1_BYTE1                  (INTERNAL_REGISTER_GROUP_L + 0)     ///< 0xE0
    #define DELTA_VREF_3_0            (15<<4)
    #define INDEX_ADC1_200_25_3_0     (15<<0)
  #define OTP1_BYTE2                  (INTERNAL_REGISTER_GROUP_L + 1)     ///< 0xE1
    #define FT_IT_6_3                 (15<<4)
    #define INDEX_ADC1_100_25_3_0     (15<<0)
  #define OTP1_BYTE3                  (INTERNAL_REGISTER_GROUP_L + 2)     ///< 0xE2
    #define FT_IT_10_7                (15<<4)
    #define INDEX_ADC2_200_25_3_0     (15<<0)
  #define OTP1_BYTE4                  (INTERNAL_REGISTER_GROUP_L + 3)     ///< 0xE3
    #define FT_IT_14_11               (15<<4)
    #define INDEX_ADC2_100_25_3_0     (15<<0)

  #define OTP2_BYTE1                  (INTERNAL_REGISTER_GROUP_L + 16)    ///< 0xF0
    #define DELTA_ET_1                (1<<7)
    #define INDEX_ADC2_100_25_4       (1<<6)
    #define DELTA_ET_0                (1<<5)
    #define PRODUCT_TYPE              (3<<3)
    #define ITDELTACODE25_10_8        (7<<0)
  #define OTP2_BYTE2                  (INTERNAL_REGISTER_GROUP_L + 17)    ///< 0xF1
    #define ITDELTACODE25_7_0         (255<<0)
  #define OTP2_BYTE3                  (INTERNAL_REGISTER_GROUP_L + 18)    ///< 0xF2
    #define OTP_CELL_EN               (31<<3)
    #define ITDELTACODE80_10_8        (7<<0)
  #define OTP2_BYTE4                  (INTERNAL_REGISTER_GROUP_L + 19)    ///< 0xF3
    #define ITDELTACODE80_7_0         (255<<0)
  
  #define OTP3_BYTE1                  (INTERNAL_REGISTER_GROUP_L + 20)    ///< 0xF4
    #define DEVADDR_9_0               (63<<2)
    #define ADC1DELTACODE25_200_9     (1<<1)
    #define ADC1DELTACODE25_200_8     (1<<0)
  #define OTP3_BYTE2                  (INTERNAL_REGISTER_GROUP_L + 21)    ///< 0xF5
    #define BGRTUNE_5_0               (63<<2)
    #define ADC1DELTACODE80_200_9_8   (3<<0)
  #define OTP3_BYTE3                  (INTERNAL_REGISTER_GROUP_L + 22)    ///< 0xF6
    #define OSCDELTACODE25            (255<<0)
  #define OTP3_BYTE4                  (INTERNAL_REGISTER_GROUP_L + 23)    ///< 0xF7
    #define OSCDELTACODE80            (255<<0)

  #define OTP4_BYTE1                  (INTERNAL_REGISTER_GROUP_L + 24)    ///< 0xF8
    #define ADC1DELTACODE25_200_7_0   (255<<0)
  #define OTP4_BYTE2                  (INTERNAL_REGISTER_GROUP_L + 25)    ///< 0xF9
    #define ADC1DELTACODE80_200_7_0   (255<<0)
  #define OTP4_BYTE3                  (INTERNAL_REGISTER_GROUP_L + 26)    ///< 0xFA
    #define ADC1DELTACODE25_100_7_0   (255<<0)
  #define OTP4_BYTE4                  (INTERNAL_REGISTER_GROUP_L + 27)    ///< 0xFB
    #define ADC1DELTACODE80_100_7_0   (255<<0)

  #define OTP5_BYTE1                  (INTERNAL_REGISTER_GROUP_L + 28)    ///< 0xFC
    #define ADC2DELTACODE25_100_6     (1<<7)
    #define ADC2DELTACODE25_100_5_0   (63<<1)
    #define ADC1DELTACODE25_100_8     (1<<0)
  #define OTP5_BYTE2                  (INTERNAL_REGISTER_GROUP_L + 29)    ///< 0xFD
    #define ADC2DELTACODE80_100_6_0   (127<<1)
    #define ADC1DELTACODE80_100_8     (1<<0)
  #define OTP5_BYTE3                  (INTERNAL_REGISTER_GROUP_L + 30)    ///< 0xFE
    #define ADC2DELTACODE25_200_7     (1<<7)
    #define ADC2DELTACODE25_200_6_0   (127<<0)
  #define OTP5_BYTE4                  (INTERNAL_REGISTER_GROUP_L + 31)    ///< 0xFF
    #define ADC2DELTACODE80_200_7_0   (255<<0)

