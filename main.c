/******************************************************************************
 * main.c
 * F28379D LaunchPad + 2x DRV8301 + 2x PMSM
 *
 * Mimari:
 *  - Üst seviye: Elektronik diferansiyel (Ackermann tabanlý hiz dagitimi)
 *  - Alt seviye: Her motor için hýz çevrimi + FOC akým çevrimi + SVPWM
 *
 * Motor-1 pin eþleþmeleri (önceden doðrulandý):
 *   IA-FB   -> ADCINC2
 *   IB-FB   -> ADCINB2
 *   IC-FB   -> ADCINA2
 *   DC-V-FB -> ADCIN14   (bu kodda ADCA channel 14 olarak kullanýldý)
 *   nFAULT  -> GPIO19
 *   nOCTW   -> GPIO18
 *   nSCS    -> GPIO61
 *   SCLK    -> GPIO60
 *   EN_GATE -> GPIO124
 *   DC_CAL  -> GPIO125
 *   QEP1A   -> GPIO20
 *   QEP1B   -> GPIO21
 *   QEP1I   -> GPIO99
 *   PWM     -> ePWM1/2/3
 *
 * Motor-2 pin eþleþmeleri (pin_config(2).xlsx dosyasýna göre):
 *   IA-FB   -> ADCINC4
 *   IB-FB   -> ADCINB4
 *   IC-FB   -> ADCINA4
 *   DC-V-FB -> ADCIN15   (bu kodda ADCA channel 15 olarak kullanýldý)
 *   nFAULT  -> GPIO139
 *   nOCTW   -> GPIO56
 *   nSCS    -> GPIO66
 *   SCLK    -> GPIO65
 *   EN_GATE -> GPIO26
 *   DC_CAL  -> GPIO27
 *   QEP2A   -> GPIO54
 *   QEP2B   -> GPIO55
 *   QEP2I   -> GPIO57
 *   PWM     -> ePWM4/5/6
 *
 * Kullaným:
 *   - gSystemEnable = 1        -> sistemi aktif eder
 *   - gVehicleSpeedRef_mps     -> taþýt hýz referansý
 *   - gSteerRef_rad            -> direksiyon/dümenleme açýsý
 *   - gM1.offset_request = 1   -> Motor-1 akým offset kalibrasyonu
 *   - gM2.offset_request = 1   -> Motor-2 akým offset kalibrasyonu
 *   - gM1.dc_cal_active = 1    -> Motor-1 DC_CAL pini aktif
 *   - gM2.dc_cal_active = 1    -> Motor-2 DC_CAL pini aktif
 ******************************************************************************/

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================== Genel ayarlar ============================== */
#define PWM_FREQ_HZ                20000.0f
#define CTRL_DT                    (1.0f / PWM_FREQ_HZ)
#define SPEED_LOOP_DIV             20U              /* 20 kHz / 20 = 1 kHz hýz çevrimi */
#define TBCLK_HZ                   50000000UL       /* SYSCLK=100MHz, HSPCLKDIV=2 */
#define TBPRD_VALUE                ((uint16_t)((TBCLK_HZ / PWM_FREQ_HZ) / 2.0f))
#define ADC_VREF                   3.3f
#define ADC_MAX_COUNT              4095.0f
#define TWO_PI_F                   6.28318530718f

#define OFFSET_SAMPLE_COUNT         2000U
#define OFFSET_DISCARD_COUNT        64U

/* ============================== Motor-1 pinleri ============================== */
#define M1_DRV_nOCTW_GPIO          18U
#define M1_DRV_nFAULT_GPIO         19U
#define M1_DRV_nSCS_GPIO           61U
#define M1_DRV_EN_GATE_GPIO        124U
#define M1_DRV_DC_CAL_GPIO         125U

#define M1_SPI_BASE                SPIA_BASE
#define M1_PWM_A_BASE              EPWM1_BASE
#define M1_PWM_B_BASE              EPWM2_BASE
#define M1_PWM_C_BASE              EPWM3_BASE
#define M1_EQEP_BASE               EQEP1_BASE
#define M1_ENCODER_CPR             4000.0f

/* ============================== Motor-2 pinleri ============================== */
#define M2_DRV_nOCTW_GPIO          56U
#define M2_DRV_nFAULT_GPIO         139U
#define M2_DRV_nSCS_GPIO           66U
#define M2_DRV_EN_GATE_GPIO        26U
#define M2_DRV_DC_CAL_GPIO         27U

#define M2_SPI_BASE                SPIB_BASE
#define M2_PWM_A_BASE              EPWM4_BASE
#define M2_PWM_B_BASE              EPWM5_BASE
#define M2_PWM_C_BASE              EPWM6_BASE
#define M2_EQEP_BASE               EQEP2_BASE
#define M2_ENCODER_CPR             4000.0f

/* ============================== DRV8301 register adresleri ============================== */
#define DRV8301_REG_STAT1          0x00U
#define DRV8301_REG_STAT2          0x01U
#define DRV8301_REG_CTRL1          0x02U
#define DRV8301_REG_CTRL2          0x03U

/* CTRL1 ayarlarý */
#define DRV_GATE_CURRENT_0P7A      0x1U
#define DRV_GATE_RESET_NORMAL      0x0U
#define DRV_PWM_MODE_6PWM          0x0U
#define DRV_OCP_MODE_CURRENT_LIMIT 0x0U
#define DRV_OC_ADJ_CODE            0x06U

/* CTRL2 ayarlarý */
#define DRV_OCTW_BOTH_OT_OC        0x0U
#define DRV_GAIN_10                0x0U
#define DRV_OC_TOFF_CBC            0x0U

/* ============================== Taþýt parametreleri ============================== */
#define VEHICLE_WHEEL_RADIUS_M     0.15f
#define VEHICLE_TRACK_M            0.40f
#define VEHICLE_WHEELBASE_M        0.60f
#define STEER_EPS_RAD              0.005f

/* ============================== Yardýmcý yapýlar ============================== */
typedef struct
{
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PIController;

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float psi_f;
    float pole_pairs;
    float i_max;
    float vbus_div_ratio;
    float current_shunt_ohm;
    float current_gain;
} MotorParams;

typedef struct
{
    /* ADC ham deðerler */
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint16_t vbus_raw;

    /* Filtreli ADC deðerleri */
    uint16_t ia_filt;
    uint16_t ib_filt;
    uint16_t ic_filt;
    uint16_t vbus_filt;

    /* Akým offsetleri */
    uint16_t ia_offset;
    uint16_t ib_offset;

    /* Fiziksel ölçümler */
    float ia_A;
    float ib_A;
    float ic_A;
    float vbus_V;

    /* Encoder / mekanik-elektriksel büyüklükler */
    int32_t qep_last;
    float theta_mech_rad;
    float theta_elec_rad;
    float speed_rpm;

    /* Referanslar */
    float speed_ref_rpm;
    float id_ref_A;
    float iq_ref_A;

    /* dq ekseni akýmlar */
    float id_A;
    float iq_A;

    /* dq gerilim komutlarý */
    float vd_V;
    float vq_V;

    /* Duty oranlarý */
    float duty_a;
    float duty_b;
    float duty_c;

    /* Durum / koruma */
    bool fault_active;
    bool octw_active;
    bool dc_cal_active;
    bool offset_request;
    bool offset_active;
    bool offset_done;

    /* Offset kalibrasyon sayaçlarý */
    uint32_t offset_discard;
    uint32_t offset_count;
    uint64_t ia_accum;
    uint64_t ib_accum;


    /* PI kontrolcüleri */
    PIController speed_pi;
    PIController id_pi;
    PIController iq_pi;
} MotorControl;

/* ============================== Global deðiþkenler ============================== */

/* Üst seviye giriþler */
volatile bool  gSystemEnable = false;
volatile float gVehicleSpeedRef_mps = 0.0f;
volatile float gSteerRef_rad = 0.0f;

/* Debug sayaçlarý */
volatile uint32_t gAdcIsrCount = 0U;
volatile uint32_t gSpeedLoopTick = 0U;

/* Taþýt parametreleri */
static const float gWheelRadius_m = VEHICLE_WHEEL_RADIUS_M;
static const float gTrack_m       = VEHICLE_TRACK_M;
static const float gWheelbase_m   = VEHICLE_WHEELBASE_M;

/* Motor parametreleri */
static MotorParams gM1Params = {
    .Rs = 0.4076258f,
    .Ld = 0.0001972132f,
    .Lq = 0.0001972132f,
    .psi_f = 0.03975862f,
    .pole_pairs = 4.0f,
    .i_max = 7.1f,
    .vbus_div_ratio = 0.1227f,
    .current_shunt_ohm = 0.01f,
    .current_gain = 10.0f
};

static MotorParams gM2Params = {
    .Rs = 0.4076258f,
    .Ld = 0.0001972132f,
    .Lq = 0.0001972132f,
    .psi_f = 0.03975862f,
    .pole_pairs = 4.0f,
    .i_max = 7.1f,
    .vbus_div_ratio = 0.1227f,
    .current_shunt_ohm = 0.01f,
    .current_gain = 10.0f
};

volatile MotorControl gM1;
volatile MotorControl gM2;

/* ============================== Fonksiyon prototipleri ============================== */
__interrupt void adca1_isr(void);

static void initSystem(void);
static void initGPIO(void);
static void initSPI(void);
static void initEQEP(void);
static void initEPWM(void);
static void initADC(void);
static void initDRV8301s(void);
static void initControlState(void);

static void runElectronicDifferential(float vx_ref_mps, float delta_rad,
                                      float *wL_ref_rpm, float *wR_ref_rpm);
static void runSpeedLoop(volatile MotorControl *m, const MotorParams *p);
static void runCurrentLoop(volatile MotorControl *m, const MotorParams *p);
static void updateEncoder(volatile MotorControl *m, uint32_t eqep_base,
                          float encoder_cpr, float pole_pairs);
static void updateProtections(volatile MotorControl *m,
                              uint32_t nFAULT_gpio, uint32_t nOCTW_gpio);
static void handleOffsetCalibration(volatile MotorControl *m);

static inline float satf(float x, float lo, float hi);
static float piRun(PIController *pi, float error, float dt);
static inline float wrap2pi(float x);
static void clarke(float ia, float ib, float *alpha, float *beta);
static void park(float alpha, float beta, float theta, float *d, float *q);
static void invPark(float vd, float vq, float theta, float *alpha, float *beta);
static void alphaBetaToDuty(float alpha, float beta, float vbus,
                            float *da, float *db, float *dc);

static inline uint16_t iir8_u16(uint16_t prev, uint16_t sample);
static inline uint16_t dutyToCmp(float duty);

static void applyMotorDuty(volatile MotorControl *m,
                           uint32_t epwmA, uint32_t epwmB, uint32_t epwmC);
static void setMotorNeutral(volatile MotorControl *m,
                            uint32_t epwmA, uint32_t epwmB, uint32_t epwmC);

static void HAL_readMotor1Adc(volatile MotorControl *m);
static void HAL_readMotor2Adc(volatile MotorControl *m);

/* DRV8301 SPI yardýmcýlarý */
static inline uint16_t drvBuildWord(uint16_t rw, uint16_t addr, uint16_t data);
static inline uint16_t spiTransfer16(uint32_t base, uint16_t tx);
static void drvWrite(uint32_t spi_base, uint32_t nSCS_gpio, uint16_t addr, uint16_t data);
static uint16_t drvRead(uint32_t spi_base, uint32_t nSCS_gpio, uint16_t addr);
static void drvConfigureDefault(uint32_t spi_base, uint32_t nSCS_gpio);

/* ============================== main ============================== */
int main(void)
{
    initSystem();
    initGPIO();
    initSPI();
    initEQEP();
    initEPWM();
    initADC();
    initDRV8301s();
    initControlState();

    EINT;
    ERTM;

    /* Ýlk deneme için örnek referans */
    gSystemEnable = true;
    gVehicleSpeedRef_mps = 2.0f;
    gSteerRef_rad = 0.0f;

    for(;;)
    {
        /* Ana döngüde fault takibi yapýlýr */
        updateProtections(&gM1, M1_DRV_nFAULT_GPIO, M1_DRV_nOCTW_GPIO);
        updateProtections(&gM2, M2_DRV_nFAULT_GPIO, M2_DRV_nOCTW_GPIO);

        /* Fault varsa ilgili motor nötrlenir */
        if(gM1.fault_active)
        {
            setMotorNeutral(&gM1, M1_PWM_A_BASE, M1_PWM_B_BASE, M1_PWM_C_BASE);
        }

        if(gM2.fault_active)
        {
            setMotorNeutral(&gM2, M2_PWM_A_BASE, M2_PWM_B_BASE, M2_PWM_C_BASE);
        }
    }
}

/* ============================== Sistem baþlangýcý ============================== */
static void initSystem(void)
{
    InitSysCtrl();

    DINT;
    InitPieCtrl();

    IER = 0U;
    IFR = 0U;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1U;
    EDIS;

    IER |= M_INT1;
}

static void initGPIO(void)
{
    /* Board LED */
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34U, GPIO_DIR_MODE_OUT);

    /* Motor-1 SPIA */
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPinConfig(GPIO_61_GPIO61);
    GPIO_setDirectionMode(M1_DRV_nSCS_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M1_DRV_nSCS_GPIO, 1U);

    /* Motor-2 SPIB */
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setPinConfig(GPIO_66_GPIO66);
    GPIO_setDirectionMode(M2_DRV_nSCS_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M2_DRV_nSCS_GPIO, 1U);

    /* Enable ve DC_CAL pinleri */
    GPIO_setPinConfig(GPIO_124_GPIO124);
    GPIO_setDirectionMode(M1_DRV_EN_GATE_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M1_DRV_EN_GATE_GPIO, 0U);

    GPIO_setPinConfig(GPIO_125_GPIO125);
    GPIO_setDirectionMode(M1_DRV_DC_CAL_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M1_DRV_DC_CAL_GPIO, 0U);

    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(M2_DRV_EN_GATE_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M2_DRV_EN_GATE_GPIO, 0U);

    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(M2_DRV_DC_CAL_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(M2_DRV_DC_CAL_GPIO, 0U);

    /* Fault / Warning pinleri */
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setDirectionMode(M1_DRV_nFAULT_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(M1_DRV_nFAULT_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(M1_DRV_nOCTW_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(M1_DRV_nOCTW_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_139_GPIO139);
    GPIO_setDirectionMode(M2_DRV_nFAULT_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(M2_DRV_nFAULT_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_56_GPIO56);
    GPIO_setDirectionMode(M2_DRV_nOCTW_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(M2_DRV_nOCTW_GPIO, GPIO_QUAL_ASYNC);

    /* PWM çýkýþlarý Motor-1 */
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    /* PWM çýkýþlarý Motor-2 */
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setPinConfig(GPIO_7_EPWM4B);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setPinConfig(GPIO_9_EPWM5B);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPinConfig(GPIO_11_EPWM6B);

    /* QEP1 */
    EALLOW;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20  = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21  = 1;
    GpioCtrlRegs.GPDGMUX1.bit.GPIO99 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO99  = 1;

    /* QEP2 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55  = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO57  = 1;
    EDIS;
}

static void initSPI(void)
{
    SPI_disableModule(M1_SPI_BASE);
    SPI_setConfig(M1_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1, SPI_MODE_MASTER, 1000000, 16);
    SPI_disableLoopback(M1_SPI_BASE);
    SPI_setEmulationMode(M1_SPI_BASE, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(M1_SPI_BASE);

    SPI_disableModule(M2_SPI_BASE);
    SPI_setConfig(M2_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1, SPI_MODE_MASTER, 1000000, 16);
    SPI_disableLoopback(M2_SPI_BASE);
    SPI_setEmulationMode(M2_SPI_BASE, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(M2_SPI_BASE);
}

static void initEQEP(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1U;
    CpuSysRegs.PCLKCR4.bit.EQEP2 = 1U;
    EDIS;

    /* Motor-1 encoder */
    EQep1Regs.QEPCTL.bit.QPEN = 0U;
    EQep1Regs.QDECCTL.all = 0U;
    EQep1Regs.QDECCTL.bit.QSRC = 0U;
    EQep1Regs.QEPCTL.bit.PCRM = 1U;
    EQep1Regs.QPOSMAX = (uint32_t)(M1_ENCODER_CPR - 1.0f);
    EQep1Regs.QPOSINIT = 0U;
    EQep1Regs.QPOSCNT = 0U;
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2U;
    EQep1Regs.QEPCTL.bit.QPEN = 1U;

    /* Motor-2 encoder */
    EQep2Regs.QEPCTL.bit.QPEN = 0U;
    EQep2Regs.QDECCTL.all = 0U;
    EQep2Regs.QDECCTL.bit.QSRC = 0U;
    EQep2Regs.QEPCTL.bit.PCRM = 1U;
    EQep2Regs.QPOSMAX = (uint32_t)(M2_ENCODER_CPR - 1.0f);
    EQep2Regs.QPOSINIT = 0U;
    EQep2Regs.QPOSCNT = 0U;
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2U;
    EQep2Regs.QEPCTL.bit.QPEN = 1U;
}

static void initEPWM(void)
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1U;

    /* Tüm PWM modülleri CPU1’de */
    DevCfgRegs.CPUSEL0.bit.EPWM1 = 0U;
    DevCfgRegs.CPUSEL0.bit.EPWM2 = 0U;
    DevCfgRegs.CPUSEL0.bit.EPWM3 = 0U;
    DevCfgRegs.CPUSEL0.bit.EPWM4 = 0U;
    DevCfgRegs.CPUSEL0.bit.EPWM5 = 0U;
    DevCfgRegs.CPUSEL0.bit.EPWM6 = 0U;

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1U;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    EDIS;

#define CONFIG_EPWM_REG(pwm)                       \
    pwm.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;    \
    pwm.TBCTL.bit.PHSEN     = TB_DISABLE;         \
    pwm.TBCTL.bit.PRDLD     = TB_SHADOW;          \
    pwm.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO;        \
    pwm.TBCTL.bit.HSPCLKDIV = TB_DIV2;            \
    pwm.TBCTL.bit.CLKDIV    = TB_DIV1;            \
    pwm.TBCTL.bit.FREE_SOFT = 2;                  \
    pwm.TBPRD               = TBPRD_VALUE;        \
    pwm.TBCTR               = 0U;                 \
    pwm.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         \
    pwm.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       \
    pwm.CMPA.bit.CMPA        = TBPRD_VALUE / 2U;  \
    pwm.AQCTLA.bit.CAU       = AQ_SET;            \
    pwm.AQCTLA.bit.CAD       = AQ_CLEAR;          \
    pwm.DBCTL.bit.OUT_MODE   = DB_FULL_ENABLE;    \
    pwm.DBCTL.bit.POLSEL     = DB_ACTV_HIC;       \
    pwm.DBCTL.bit.IN_MODE    = DBA_ALL;           \
    pwm.DBRED.bit.DBRED      = 50U;               \
    pwm.DBFED.bit.DBFED      = 50U;

    CONFIG_EPWM_REG(EPwm1Regs)
    CONFIG_EPWM_REG(EPwm2Regs)
    CONFIG_EPWM_REG(EPwm3Regs)
    CONFIG_EPWM_REG(EPwm4Regs)
    CONFIG_EPWM_REG(EPwm5Regs)
    CONFIG_EPWM_REG(EPwm6Regs)

    /* ADC tetikleme için EPWM1 kullan */
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1U;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;

#undef CONFIG_EPWM_REG

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;
}

static void initADC(void)
{
    EALLOW;
    DevCfgRegs.CPUSEL11.bit.ADC_A = 0U;
    DevCfgRegs.CPUSEL11.bit.ADC_B = 0U;
    DevCfgRegs.CPUSEL11.bit.ADC_C = 0U;

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1U;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1U;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1U;
    EDIS;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6U;

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    DEVICE_DELAY_US(1000);

    EALLOW;

    /* Motor-1: IA1, IB1, IC1, VBUS1 */
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = 2U;   /* ADCINC2 */
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdccRegs.ADCSOC0CTL.bit.ACQPS   = 99U;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = 2U;   /* ADCINB2 */
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS   = 99U;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 2U;   /* ADCINA2 */
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = 99U;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 14U;  /* ADCIN14 */
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = 99U;

    /* Motor-2: IA2, IB2, IC2, VBUS2 */
    AdccRegs.ADCSOC2CTL.bit.CHSEL   = 4U;   /* ADCINC4 */
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdccRegs.ADCSOC2CTL.bit.ACQPS   = 99U;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL   = 4U;   /* ADCINB4 */
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS   = 99U;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL   = 4U;   /* ADCINA4 */
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS   = 99U;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL   = 15U;  /* ADCIN15 */
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGGER_EPWM1_SOCA;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS   = 99U;

    /* ADCA EOC3 interrupt üretir */
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL  = 3U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E    = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1  = 1U;

    EDIS;
}

static void initDRV8301s(void)
{
    GPIO_writePin(M1_DRV_EN_GATE_GPIO, 1U);
    DEVICE_DELAY_US(1000);
    drvConfigureDefault(M1_SPI_BASE, M1_DRV_nSCS_GPIO);

    GPIO_writePin(M2_DRV_EN_GATE_GPIO, 1U);
    DEVICE_DELAY_US(1000);
    drvConfigureDefault(M2_SPI_BASE, M2_DRV_nSCS_GPIO);
}

static void initControlState(void)
{
    /* Motor-1 PI ayarlarý */
    gM1.speed_pi.kp = 0.05f;
    gM1.speed_pi.ki = 1.0f;
    gM1.speed_pi.out_min = -gM1Params.i_max;
    gM1.speed_pi.out_max =  gM1Params.i_max;

    gM1.id_pi.kp = 0.3f;
    gM1.id_pi.ki = 50.0f;
    gM1.id_pi.out_min = -12.0f;
    gM1.id_pi.out_max =  12.0f;

    gM1.iq_pi.kp = 0.3f;
    gM1.iq_pi.ki = 50.0f;
    gM1.iq_pi.out_min = -12.0f;
    gM1.iq_pi.out_max =  12.0f;

    gM1.ia_offset = 2048U;
    gM1.ib_offset = 2048U;

    /* Motor-2 PI ayarlarý */
    gM2.speed_pi.kp = 0.05f;
    gM2.speed_pi.ki = 1.0f;
    gM2.speed_pi.out_min = -gM2Params.i_max;
    gM2.speed_pi.out_max =  gM2Params.i_max;

    gM2.id_pi.kp = 0.3f;
    gM2.id_pi.ki = 50.0f;
    gM2.id_pi.out_min = -12.0f;
    gM2.id_pi.out_max =  12.0f;

    gM2.iq_pi.kp = 0.3f;
    gM2.iq_pi.ki = 50.0f;
    gM2.iq_pi.out_min = -12.0f;
    gM2.iq_pi.out_max =  12.0f;

    gM2.ia_offset = 2048U;
    gM2.ib_offset = 2048U;

    setMotorNeutral(&gM1, M1_PWM_A_BASE, M1_PWM_B_BASE, M1_PWM_C_BASE);
    setMotorNeutral(&gM2, M2_PWM_A_BASE, M2_PWM_B_BASE, M2_PWM_C_BASE);
}

/* ============================== ADC ISR ============================== */
__interrupt void adca1_isr(void)
{
    float wL_ref = 0.0f, wR_ref = 0.0f;

    gAdcIsrCount++;

    /* ADC örneklerini oku */
    HAL_readMotor1Adc(&gM1);
    HAL_readMotor2Adc(&gM2);

    /* Encoder bilgisini güncelle */
    updateEncoder(&gM1, M1_EQEP_BASE, M1_ENCODER_CPR, gM1Params.pole_pairs);
    updateEncoder(&gM2, M2_EQEP_BASE, M2_ENCODER_CPR, gM2Params.pole_pairs);

    /* Koruma durumlarýný güncelle */
    updateProtections(&gM1, M1_DRV_nFAULT_GPIO, M1_DRV_nOCTW_GPIO);
    updateProtections(&gM2, M2_DRV_nFAULT_GPIO, M2_DRV_nOCTW_GPIO);

    /* Offset kalibrasyonu */
    handleOffsetCalibration(&gM1);
    handleOffsetCalibration(&gM2);

    /* DC_CAL pinleri */
    GPIO_writePin(M1_DRV_DC_CAL_GPIO, gM1.dc_cal_active ? 1U : 0U);
    GPIO_writePin(M2_DRV_DC_CAL_GPIO, gM2.dc_cal_active ? 1U : 0U);

    /* Yavaþ çevrim: elektronik diferansiyel + hýz PI */
    gSpeedLoopTick++;
    if(gSpeedLoopTick >= SPEED_LOOP_DIV)
    {
        gSpeedLoopTick = 0U;

        if(gSystemEnable)
        {
            runElectronicDifferential(gVehicleSpeedRef_mps, gSteerRef_rad, &wL_ref, &wR_ref);
            gM1.speed_ref_rpm = wL_ref;
            gM2.speed_ref_rpm = wR_ref;
        }
        else
        {
            gM1.speed_ref_rpm = 0.0f;
            gM2.speed_ref_rpm = 0.0f;
        }

        runSpeedLoop(&gM1, &gM1Params);
        runSpeedLoop(&gM2, &gM2Params);
    }

    /* Hýzlý çevrim: FOC + SVPWM */
    runCurrentLoop(&gM1, &gM1Params);
    runCurrentLoop(&gM2, &gM2Params);

    applyMotorDuty(&gM1, M1_PWM_A_BASE, M1_PWM_B_BASE, M1_PWM_C_BASE);
    applyMotorDuty(&gM2, M2_PWM_A_BASE, M2_PWM_B_BASE, M2_PWM_C_BASE);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1U;
}

/* ============================== Elektronik diferansiyel ============================== */
static void runElectronicDifferential(float vx_ref_mps, float delta_rad,
                                      float *wL_ref_rpm, float *wR_ref_rpm)
{
    float R;
    float vL, vR;

    if(fabsf(delta_rad) < STEER_EPS_RAD)
    {
        *wL_ref_rpm = (vx_ref_mps / gWheelRadius_m) * (60.0f / TWO_PI_F);
        *wR_ref_rpm = *wL_ref_rpm;
        return;
    }

    R = gWheelbase_m / tanf(delta_rad);
    vL = vx_ref_mps * ((R - (gTrack_m * 0.5f)) / R);
    vR = vx_ref_mps * ((R + (gTrack_m * 0.5f)) / R);

    *wL_ref_rpm = (vL / gWheelRadius_m) * (60.0f / TWO_PI_F);
    *wR_ref_rpm = (vR / gWheelRadius_m) * (60.0f / TWO_PI_F);
}

/* ============================== Hýz çevrimi ============================== */
static void runSpeedLoop(volatile MotorControl *m, const MotorParams *p)
{
    float err;

    m->id_ref_A = 0.0f;
    err = m->speed_ref_rpm - m->speed_rpm;

    m->iq_ref_A = piRun((PIController *)&m->speed_pi, err, CTRL_DT * (float)SPEED_LOOP_DIV);
    m->iq_ref_A = satf(m->iq_ref_A, -p->i_max, p->i_max);
}

/* ============================== Akým çevrimi ============================== */
static void runCurrentLoop(volatile MotorControl *m, const MotorParams *p)
{
    float ialpha, ibeta;
    float valpha, vbeta;

    /* ADC -> fiziksel büyüklükler */
    m->ia_filt = iir8_u16(m->ia_filt, m->ia_raw);
    m->ib_filt = iir8_u16(m->ib_filt, m->ib_raw);
    m->ic_filt = iir8_u16(m->ic_filt, m->ic_raw);
    m->vbus_filt = iir8_u16(m->vbus_filt, m->vbus_raw);

    m->ia_A = (((float)((int32_t)m->ia_filt - (int32_t)m->ia_offset)) * ADC_VREF) /
              (ADC_MAX_COUNT * p->current_shunt_ohm * p->current_gain);

    m->ib_A = (((float)((int32_t)m->ib_filt - (int32_t)m->ib_offset)) * ADC_VREF) /
              (ADC_MAX_COUNT * p->current_shunt_ohm * p->current_gain);

    m->ic_A = -(m->ia_A + m->ib_A);

    m->vbus_V = (((float)m->vbus_filt * ADC_VREF) / ADC_MAX_COUNT) / p->vbus_div_ratio;
    if(m->vbus_V < 1.0f)
    {
        m->vbus_V = 1.0f;
    }

    /* Clarke + Park */
    clarke(m->ia_A, m->ib_A, &ialpha, &ibeta);
    park(ialpha, ibeta, m->theta_elec_rad, &m->id_A, &m->iq_A);

    /* PI kontrolcüleri */
    m->vd_V = piRun((PIController *)&m->id_pi, (m->id_ref_A - m->id_A), CTRL_DT);
    m->vq_V = piRun((PIController *)&m->iq_pi, (m->iq_ref_A - m->iq_A), CTRL_DT);

    /* Ters Park */
    invPark(m->vd_V, m->vq_V, m->theta_elec_rad, &valpha, &vbeta);

    /* Alpha-beta -> duty */
    alphaBetaToDuty(valpha, vbeta, m->vbus_V, &m->duty_a, &m->duty_b, &m->duty_c);

    /* Fault veya DC_CAL varsa nötrle */
    if(m->fault_active || m->dc_cal_active)
    {
        m->duty_a = 0.5f;
        m->duty_b = 0.5f;
        m->duty_c = 0.5f;
    }
}

/* ============================== Encoder güncelleme ============================== */
static void updateEncoder(volatile MotorControl *m, uint32_t eqep_base,
                          float encoder_cpr, float pole_pairs)
{
    int32_t newpos, delta;

    if(eqep_base == EQEP1_BASE)
    {
        newpos = (int32_t)EQep1Regs.QPOSCNT;
    }
    else
    {
        newpos = (int32_t)EQep2Regs.QPOSCNT;
    }

    delta = newpos - m->qep_last;
    m->qep_last = newpos;

    if(delta > (int32_t)(encoder_cpr * 0.5f))  delta -= (int32_t)encoder_cpr;
    if(delta < -(int32_t)(encoder_cpr * 0.5f)) delta += (int32_t)encoder_cpr;

    m->theta_mech_rad = wrap2pi((TWO_PI_F * (float)newpos) / encoder_cpr);
    m->theta_elec_rad = wrap2pi(m->theta_mech_rad * pole_pairs);
    m->speed_rpm = ((float)delta / encoder_cpr) * (60.0f / CTRL_DT);
}

/* ============================== Koruma takibi ============================== */
static void updateProtections(volatile MotorControl *m,
                              uint32_t nFAULT_gpio, uint32_t nOCTW_gpio)
{
    m->fault_active = (GPIO_readPin(nFAULT_gpio) == 0U);
    m->octw_active  = (GPIO_readPin(nOCTW_gpio)  == 0U);
}

/* ============================== Offset kalibrasyonu ============================== */
static void handleOffsetCalibration(volatile MotorControl *m)
{
    if(m->offset_request && !m->offset_active)
    {
        m->offset_request = false;
        m->offset_done = false;
        m->offset_active = true;
        m->offset_discard = 0U;
        m->offset_count = 0U;
        m->ia_accum = 0ULL;
        m->ib_accum = 0ULL;
    }

    if(!m->offset_active)
    {
        return;
    }

    if(m->offset_discard < 64U)
    {
        m->offset_discard++;
        return;
    }

    m->ia_accum += m->ia_filt;
    m->ib_accum += m->ib_filt;
    m->offset_count++;

    if(m->offset_count >= OFFSET_SAMPLE_COUNT)
    {
        m->ia_offset = (uint16_t)(m->ia_accum / m->offset_count);
        m->ib_offset = (uint16_t)(m->ib_accum / m->offset_count);
        m->offset_active = false;
        m->offset_done = true;
    }
}

/* ============================== Matematik yardýmcýlarý ============================== */
static inline float satf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static float piRun(PIController *pi, float error, float dt)
{
    float out;
    pi->integral += pi->ki * error * dt;
    pi->integral = satf(pi->integral, pi->out_min, pi->out_max);
    out = pi->kp * error + pi->integral;
    return satf(out, pi->out_min, pi->out_max);
}

static inline float wrap2pi(float x)
{
    while(x >= TWO_PI_F) x -= TWO_PI_F;
    while(x < 0.0f)      x += TWO_PI_F;
    return x;
}

static void clarke(float ia, float ib, float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * (1.0f / 1.73205080757f);
}

static void park(float alpha, float beta, float theta, float *d, float *q)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *d = alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

static void invPark(float vd, float vq, float theta, float *alpha, float *beta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *alpha = vd * c - vq * s;
    *beta  = vd * s + vq * c;
}

static void alphaBetaToDuty(float alpha, float beta, float vbus,
                            float *da, float *db, float *dc)
{
    float va, vb, vc;
    float vmax, vmin, voffset;

    va = alpha;
    vb = -0.5f * alpha + 0.86602540378f * beta;
    vc = -0.5f * alpha - 0.86602540378f * beta;

    vmax = fmaxf(va, fmaxf(vb, vc));
    vmin = fminf(va, fminf(vb, vc));
    voffset = 0.5f * (vmax + vmin);

    *da = satf(0.5f + ((va - voffset) / vbus), 0.02f, 0.98f);
    *db = satf(0.5f + ((vb - voffset) / vbus), 0.02f, 0.98f);
    *dc = satf(0.5f + ((vc - voffset) / vbus), 0.02f, 0.98f);
}

static inline uint16_t iir8_u16(uint16_t prev, uint16_t sample)
{
    return (uint16_t)((((uint32_t)prev) * 7U + sample + 4U) >> 3);
}

static inline uint16_t dutyToCmp(float duty)
{
    duty = satf(duty, 0.02f, 0.98f);
    return (uint16_t)((float)TBPRD_VALUE * duty);
}

/* ============================== PWM yardýmcýlarý ============================== */
static void applyMotorDuty(volatile MotorControl *m,
                           uint32_t epwmA, uint32_t epwmB, uint32_t epwmC)
{
    EPWM_setCounterCompareValue(epwmA, EPWM_COUNTER_COMPARE_A, dutyToCmp(m->duty_a));
    EPWM_setCounterCompareValue(epwmB, EPWM_COUNTER_COMPARE_A, dutyToCmp(m->duty_b));
    EPWM_setCounterCompareValue(epwmC, EPWM_COUNTER_COMPARE_A, dutyToCmp(m->duty_c));
}

static void setMotorNeutral(volatile MotorControl *m,
                            uint32_t epwmA, uint32_t epwmB, uint32_t epwmC)
{
    m->duty_a = 0.5f;
    m->duty_b = 0.5f;
    m->duty_c = 0.5f;
    applyMotorDuty(m, epwmA, epwmB, epwmC);
}

/* ============================== ADC okuma yardýmcýlarý ============================== */
static void HAL_readMotor1Adc(volatile MotorControl *m)
{
    /* ADCA SOC0 -> ADCINA2 (IC1)
       ADCA SOC1 -> ADCIN14 (VBUS1)
       ADCB SOC0 -> ADCINB2 (IB1)
       ADCC SOC0 -> ADCINC2 (IA1) */
    m->ia_raw   = (AdccResultRegs.ADCRESULT0 & 0x0FFFU);
    m->ib_raw   = (AdcbResultRegs.ADCRESULT0 & 0x0FFFU);
    m->ic_raw   = (AdcaResultRegs.ADCRESULT0 & 0x0FFFU);
    m->vbus_raw = (AdcaResultRegs.ADCRESULT1 & 0x0FFFU);
}

static void HAL_readMotor2Adc(volatile MotorControl *m)
{
    /* ADCA SOC2 -> ADCINA4 (IC2)
       ADCA SOC3 -> ADCIN15 (VBUS2)
       ADCB SOC2 -> ADCINB4 (IB2)
       ADCC SOC2 -> ADCINC4 (IA2) */
    m->ia_raw   = (AdccResultRegs.ADCRESULT2 & 0x0FFFU);
    m->ib_raw   = (AdcbResultRegs.ADCRESULT2 & 0x0FFFU);
    m->ic_raw   = (AdcaResultRegs.ADCRESULT2 & 0x0FFFU);
    m->vbus_raw = (AdcaResultRegs.ADCRESULT3 & 0x0FFFU);
}

/* ============================== DRV8301 SPI ============================== */
static inline uint16_t drvBuildWord(uint16_t rw, uint16_t addr, uint16_t data)
{
    return (uint16_t)(((rw & 0x1U) << 15) | ((addr & 0xFU) << 11) | (data & 0x07FFU));
}

static inline uint16_t spiTransfer16(uint32_t base, uint16_t tx)
{
    SPI_writeDataBlockingNonFIFO(base, tx);
    return SPI_readDataBlockingNonFIFO(base);
}

static void drvWrite(uint32_t spi_base, uint32_t nSCS_gpio, uint16_t addr, uint16_t data)
{
    GPIO_writePin(nSCS_gpio, 0U);
    DEVICE_DELAY_US(1);
    (void)spiTransfer16(spi_base, drvBuildWord(0U, addr, data));
    DEVICE_DELAY_US(1);
    GPIO_writePin(nSCS_gpio, 1U);
    DEVICE_DELAY_US(1);
}

static uint16_t drvRead(uint32_t spi_base, uint32_t nSCS_gpio, uint16_t addr)
{
    uint16_t rx;

    GPIO_writePin(nSCS_gpio, 0U);
    DEVICE_DELAY_US(1);
    (void)spiTransfer16(spi_base, drvBuildWord(1U, addr, 0U));
    DEVICE_DELAY_US(1);
    GPIO_writePin(nSCS_gpio, 1U);
    DEVICE_DELAY_US(1);

    GPIO_writePin(nSCS_gpio, 0U);
    DEVICE_DELAY_US(1);
    rx = spiTransfer16(spi_base, 0x0000U);
    DEVICE_DELAY_US(1);
    GPIO_writePin(nSCS_gpio, 1U);
    DEVICE_DELAY_US(1);

    return (uint16_t)(rx & 0x07FFU);
}

static void drvConfigureDefault(uint32_t spi_base, uint32_t nSCS_gpio)
{
    uint16_t ctrl1, ctrl2;

    ctrl1 = 0U;
    ctrl1 |= (DRV_GATE_CURRENT_0P7A & 0x3U);
    ctrl1 |= ((DRV_PWM_MODE_6PWM & 0x1U) << 3);
    ctrl1 |= ((DRV_OCP_MODE_CURRENT_LIMIT & 0x3U) << 4);
    ctrl1 |= ((DRV_OC_ADJ_CODE & 0x1FU) << 6);

    ctrl2 = 0U;
    ctrl2 |= (DRV_OCTW_BOTH_OT_OC & 0x3U);
    ctrl2 |= ((DRV_GAIN_10 & 0x3U) << 2);
    ctrl2 |= ((DRV_OC_TOFF_CBC & 0x1U) << 6);

    drvWrite(spi_base, nSCS_gpio, DRV8301_REG_CTRL1, ctrl1);
    drvWrite(spi_base, nSCS_gpio, DRV8301_REG_CTRL2, ctrl2);

    (void)drvRead(spi_base, nSCS_gpio, DRV8301_REG_CTRL1);
    (void)drvRead(spi_base, nSCS_gpio, DRV8301_REG_CTRL2);
}
