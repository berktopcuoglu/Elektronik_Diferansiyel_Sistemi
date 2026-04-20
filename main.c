#include "F28x_Project.h"
#include <math.h>
#include <stdint.h>

#ifndef DEVICE_SYSCLK_FREQ
#define DEVICE_SYSCLK_FREQ         200000000UL
#endif

#define SYSCLK_HZ                  DEVICE_SYSCLK_FREQ
#define PWM_FREQ_HZ                20000UL
#define CTRL_FREQ_HZ               PWM_FREQ_HZ
#define CTRL_TS                    (1.0f / (float)CTRL_FREQ_HZ)

#define TIMER0_FREQ_HZ             1000UL
#define TIMER0_TS                  (1.0f / (float)TIMER0_FREQ_HZ)

#define EPWMCLK_HZ                 SYSCLK_HZ
#define TBPRD_VAL                  (EPWMCLK_HZ / (2UL * PWM_FREQ_HZ))
#define ADC_ACQPS_12BIT            14U

#define ADC_MAX_COUNT              4095.0f
#define ADC_VREF                   3.0f

#define PI_F                       3.14159265358979323846f
#define TWO_PI_F                   6.28318530717958647692f
#define INV_SQRT3_F                0.5773502691896258f
#define SQRT3_BY_2_F               0.8660254037844386f

// ===================== Motor / Inverter =====================
#define MOTOR_POLE_PAIRS           4.0f
#define MOTOR_RS_PHASE_OHM         0.36f
#define MOTOR_LS_PHASE_H           0.00020f
#define PM_FLUX_WB                 0.0064f

#define CURRENT_SHUNT_OHM          0.01f
#define IA_SIGN                    (1.0f)
#define IB_SIGN                    (1.0f)

// ===================== Ölçüm / Koruma =====================
#define VBUS_NEAR_CLIP_RAW         4000U
#define VBUS_SAT_RAW               4088U
#define VBUS_CAL_MEASURED_V        24.0f
#define CURR_OFFSET_SAMPLES        2048U

#define I_FILT_ALPHA               0.05f
#define VBUS_FILT_ALPHA            0.05f

#define FOC_OC_LIMIT_A             7.1f

// Not: Bu projede hýz tahmini 1 kHz zaman tabanýnda yapýlmaktadýr.
// Elektriksel açý ise FOC tarafýnda 20 kHz ADC ISR içinde güncellenmelidir.
#define FOC_MIN_RUN_VBUS_V          5.0f

// ===================== Kontrol =====================
#define DRV8301_CSA_GAIN_DEFAULT_VV   20.0f

#define ALIGN_TIME_MS                 1200U
#define ALIGN_ELEC_ANGLE_RAD          0.0f
#define ALIGN_MOD_INDEX               0.15f
#define ENCODER_ELEC_ZERO_BIAS_RAD    0.0f

#define CLOSEDLOOP_SPEED_RAMP_RPM_PER_S 120.0f
#define OPENLOOP_ID_A                   0.0f
#define IQ_REF_SLEW_A_PER_S             20.0f

#define CURRENT_PI_KP                 0.35f
#define CURRENT_PI_KI                 450.0f

#define SPEED_PI_KP                   0.0030f
#define SPEED_PI_KI                   0.0400f
#define SPEED_PI_OUT_MAX_A            0.80f

#define SPEED_FILT_ALPHA              0.20f
#define TRACTION_SPEED_ENTER_BAND_RPM  15.0f
#define TRACTION_SPEED_EXIT_BAND_RPM    5.0f
#define TRACTION_MIN_RUN_IQ_A           0.18f
#define VERIFY_TIME_MS                  0U
#define VERIFY_IQ_A                     0.25f
#define ALIGN_OFFSET_SAMPLE_MS        200U

#define FOC_VOLT_LIMIT_FACTOR         0.95f
#define DUTY_MIN                      0.02f
#define DUTY_MAX                      0.98f
#define DEADTIME_NS                   500.0f

#define USE_DECOUPLING_FF             0

// ===================== Enkoder / eQEP =====================
#define ENCODER_LINES_PER_REV         1000U
#define ENCODER_COUNTS_PER_REV        (4U * ENCODER_LINES_PER_REV)
#define ENCODER_USE_EQEP1_INDEX       1U
#define ENCODER_SWAP_AB               0U
#define ENCODER_MECH_SIGN             (1.0f)
#define ENCODER_OFFSET_EXTRA_RAD      0.0f

// ===================== Pinler =====================
#define PIN_DRV_FAULT              19U
#define PIN_DRV_OCTW               18U
#define PIN_DRV_SPI_SIMO           58U
#define PIN_DRV_SPI_SOMI           59U
#define PIN_DRV_SPI_CLK            60U
#define PIN_DRV_SPI_CS             61U
#define PIN_DRV_EN_GATE            124U
#define PIN_DRV_DC_CAL             125U

// ===================== DRV8301 =====================
#define DRV8301_REG_STATUS1        0x0U
#define DRV8301_REG_STATUS2        0x1U
#define DRV8301_REG_CONTROL1       0x2U
#define DRV8301_REG_CONTROL2       0x3U

#define DRV8301_CTRL2_OCTW_BOTH    (0x0000U)
#define DRV8301_CTRL2_GAIN_10VV    (0x0000U)
#define DRV8301_CTRL2_GAIN_20VV    (0x0004U)
#define DRV8301_CTRL2_GAIN_40VV    (0x0008U)
#define DRV8301_CTRL2_GAIN_80VV    (0x000CU)
#define DRV8301_DEFAULT_GAIN_BITS  DRV8301_CTRL2_GAIN_20VV

typedef enum
{
    APP_STATE_STOP = 0,
    APP_STATE_ALIGN,
    APP_STATE_VERIFY,
    APP_STATE_CLOSEDLOOP,
    APP_STATE_FAULT
} AppState_e;

typedef struct
{
    float Kp;
    float Ki;
    float Ui;
    float OutMin;
    float OutMax;
} PI_Controller_t;

typedef struct
{
    float CsaGain_VV;
    Uint16 PwmEnabled;
    Uint16 AppState;
    Uint16 StartCmd;
    int16_t DirectionCmd;

    float IA_A;
    float IB_A;
    float IC_A;
    float Vbus_V;

    float IA_A_Filt;
    float IB_A_Filt;
    float Vbus_V_Filt;

    float SpeedRefRpmCmd;
    float SpeedCmdRpmRamp;
    float SpeedFbRpm;
    float SpeedFbRpmFilt;
    float OmegaMechCmdRad_s;
    float OmegaElecCmdRad_s;
    float OmegaMechFbRad_s;
    float OmegaElecFbRad_s;

    float ThetaCtrlRad;
    float ThetaMechEncRad;
    float ThetaElecEncRad;
    float ThetaElecOffsetRad;

    float IdRef_A;
    float IqRefTarget_A;
    float IqRefCmd_A;

    float Id_A;
    float Iq_A;

    float VdCmd_V;
    float VqCmd_V;

    float DutyA;
    float DutyB;
    float DutyC;

    Uint16 FaultActive;
    Uint16 OctwActive;
    Uint16 QepIndexSeen;
    int32_t QepCount;
    int32_t QepDeltaCount;
    Uint32 AlignCounterMs;
    Uint32 Timer0IsrCount;
    Uint32 AdcIsrCount;

    Uint32 seq;
} DebugSnapshot_t;

// ===================== Global =====================
volatile Uint16 gIA_raw = 0U;
volatile Uint16 gIB_raw = 0U;
volatile Uint16 gVBS_raw = 0U;

volatile float gIA_adc_V = 0.0f;
volatile float gIB_adc_V = 0.0f;
volatile float gVBS_adc_V = 0.0f;

volatile float gIA_offset_counts = 2048.0f;
volatile float gIB_offset_counts = 2048.0f;

volatile int32_t gIA_corr_counts = 0;
volatile int32_t gIB_corr_counts = 0;
volatile float gIA_corr_V = 0.0f;
volatile float gIB_corr_V = 0.0f;

volatile float gCsaGain_VV = DRV8301_CSA_GAIN_DEFAULT_VV;
volatile float gIA_A = 0.0f;
volatile float gIB_A = 0.0f;
volatile float gIC_A = 0.0f;
volatile float gIA_A_filt = 0.0f;
volatile float gIB_A_filt = 0.0f;

volatile float gVBS_bus_V = 0.0f;
volatile float gVBS_bus_V_filt = 0.0f;
volatile float gVbusDividerGain = 1.0f;
volatile float gVbusSatBus_V = 0.0f;
volatile Uint16 gVbusNearClip = 0U;
volatile Uint16 gVbusSaturated = 0U;

volatile float gIalpha_A = 0.0f;
volatile float gIbeta_A = 0.0f;
volatile float gId_A = 0.0f;
volatile float gIq_A = 0.0f;

volatile float gIdRefCmd_A = OPENLOOP_ID_A;
volatile float gIqRefCmd_A = 0.0f;
volatile float gIqRefTarget_A = 0.0f;

volatile float gSpeedRefRpmCmd = 50.0f;
volatile float gSpeedCmdRpmRamp = 0.0f;
volatile float gOmegaMechCmdRad_s = 0.0f;
volatile float gOmegaElecCmdRad_s = 0.0f;

volatile float gThetaCtrlRad = 0.0f;
volatile float gThetaOpenloopRad = 0.0f;
volatile float gThetaMechEncRad = 0.0f;
volatile float gThetaElecEncRad = 0.0f;
volatile float gThetaElecOffsetRad = 0.0f;

volatile float gSpeedMechFbRpm = 0.0f;
volatile float gSpeedMechFbRpmFilt = 0.0f;
volatile float gOmegaMechFbRad_s = 0.0f;
volatile float gOmegaElecFbRad_s = 0.0f;

volatile int32_t gQepCount = 0;
volatile int32_t gQepPrevCount = 0;
volatile int32_t gQepDeltaCount = 0;
volatile int32_t gQepDeltaAcc = 0;
volatile Uint16 gQepSpeedWindowCount = 0U;
volatile Uint16 gQepIndexSeen = 0U;
volatile float gAlignQepCountSum = 0.0f;
volatile Uint32 gAlignQepSampleCount = 0U;

volatile float gVdCmd_V = 0.0f;
volatile float gVqCmd_V = 0.0f;
volatile float gValphaCmd_V = 0.0f;
volatile float gVbetaCmd_V = 0.0f;
volatile float gVdqLimit_V = 0.0f;

volatile PI_Controller_t gIdPI = {CURRENT_PI_KP, CURRENT_PI_KI, 0.0f, -100.0f, 100.0f};
volatile PI_Controller_t gIqPI = {CURRENT_PI_KP, CURRENT_PI_KI, 0.0f, -100.0f, 100.0f};
volatile PI_Controller_t gSpeedPI = {SPEED_PI_KP, SPEED_PI_KI, 0.0f, -SPEED_PI_OUT_MAX_A, SPEED_PI_OUT_MAX_A};

volatile Uint32 gIA_sum = 0U;
volatile Uint32 gIB_sum = 0U;
volatile Uint16 gCalCount = 0U;
volatile Uint16 gCalActive = 0U;
volatile Uint16 gCalDone = 0U;

volatile Uint16 gDcCalPinState = 0U;
volatile Uint16 gGateEnableState = 0U;
volatile Uint16 gFaultState = 1U;
volatile Uint16 gOctwState = 1U;
volatile Uint16 gFaultActive = 0U;
volatile Uint16 gOctwActive = 0U;

volatile Uint16 gDrvStat1Raw = 0U;
volatile Uint16 gDrvStat2Raw = 0U;
volatile Uint16 gDrvCtrl1Raw = 0U;
volatile Uint16 gDrvCtrl2Raw = 0U;
volatile Uint16 gDrvStat1Data = 0U;
volatile Uint16 gDrvStat2Data = 0U;
volatile Uint16 gDrvCtrl1Data = 0U;
volatile Uint16 gDrvCtrl2Data = 0U;
volatile Uint16 gDrvFrameFault = 0U;
volatile Uint16 gDrvDeviceID = 0U;
volatile Uint16 gDrvLastRxWord = 0U;

volatile AppState_e gAppState = APP_STATE_STOP;
volatile Uint32 gAlignCounterMs = 0U;
volatile Uint32 gVerifyCounterMs = 0U;
volatile Uint32 gTimer0IsrCount = 0U;
volatile Uint32 gAdcIsrCount = 0U;

volatile float gDutyA = 0.5f;
volatile float gDutyB = 0.5f;
volatile float gDutyC = 0.5f;

volatile Uint16 gPwmEnabled = 0U;
volatile Uint16 gOverCurrentTrip = 0U;

volatile Uint16 gStartCmd = 0U;
volatile Uint16 gStartCmdPrev = 0U;
volatile int16_t gDirectionCmd = 1;

volatile Uint16 gSpiServiceReq = 0U;
volatile Uint16 gFilterInitDone = 0U;
volatile Uint16 gTractionDriveEnable = 0U;

volatile DebugSnapshot_t gDbg = {0};

// ===================== Prototypes =====================
static void GPIO_InitDrv8301Pins(void);
static void PWM_InitGpioOutputs_Motor1(void);

static void DRV8301_SetDCCal(Uint16 enable);
static void DRV8301_SetGateEnable(Uint16 enable);
static void DRV8301_ReadStatusPins(void);

static void SPIA_InitDrv8301(void);
static inline void DRV8301_CS_Low(void);
static inline void DRV8301_CS_High(void);
static Uint16 SPIA_Transfer16(Uint16 data);
static Uint16 DRV8301_ReadReg(Uint16 addr);
static Uint16 DRV8301_WriteReg(Uint16 addr, Uint16 data11);
static void DRV8301_ReadAllRegs(void);
static void DRV8301_SetCurrentAmpGain(Uint16 gainBits);

static void ADC_InitModules(void);
static void ADC_InitSOCs(void);

static void EQEP1_InitGpio(void);
static void EQEP1_InitModule(void);
static void EQEP1_ResetEstimator(void);
static void EQEP1_ResetAlignOffsetAccumulator(void);
static void EQEP1_RunAlignOffsetAccumulator_1kHz(void);
static void EQEP1_CaptureElecOffsetFromAlign(void);
static void EQEP1_UpdatePositionFromCount(void);
static void EQEP1_UpdateSpeed_1kHz(void);

static void PWM_InitInverter(void);
static void PWM_InitSingleEPwm(volatile struct EPWM_REGS *p);
static Uint16 PWM_DeadtimeCounts(float deadtime_ns);
static void PWM_EnableOutputs(Uint16 enable);
static void PWM_UpdateDutyABC(float dutyA, float dutyB, float dutyC);

static void CPU_TIMER0_Init1kHz(void);

static void StartCurrentOffsetCalibration(void);
static void WaitCurrentOffsetCalibrationDone(void);
static void CalibrateVbusDividerFromKnownBus(float measuredBusV);

static inline void ConvertRawToPhysical(void);
static inline void UpdateFiltersAndAlarms_ISR(void);
static inline void UpdateDebugSnapshot(void);

static inline float ClampF(float x, float xmin, float xmax);
static inline float WrapAngle_0_2pi(float x);
static inline float PI_Run(volatile PI_Controller_t *p, float ref, float fb, float Ts);
static inline float RampToTarget(float current, float target, float maxStep);

static void FOC_ResetControllers(void);
static void SVPWM_Apply(float vAlpha, float vBeta, float vdc);
static void Align_ApplyVoltageVector(float theta_e_align, float modIndex, float vdc);
static void FOC_RunCurrentLoop_ISR(void);
static void App_RunStateMachine_1kHz(void);
static void App_ForceStop(void);

interrupt void adca1_isr(void);
interrupt void cpu_timer0_isr(void);

// ===================== main =====================
void main(void)
{
    InitSysCtrl();
    InitGpio();

    DINT;
    InitPieCtrl();

    IER = 0x0000U;
    IFR = 0x0000U;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT  = &adca1_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    GPIO_InitDrv8301Pins();
    PWM_InitGpioOutputs_Motor1();
    EQEP1_InitGpio();
    SPIA_InitDrv8301();

    DRV8301_SetGateEnable(0U);
    DRV8301_SetDCCal(0U);

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    EDIS;

    ADC_InitModules();
    ADC_InitSOCs();
    EQEP1_InitModule();
    PWM_InitInverter();
    CPU_TIMER0_Init1kHz();

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;   // ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1U;   // CPU Timer0
    IER |= M_INT1;

    EINT;
    ERTM;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;

    DRV8301_SetGateEnable(1U);
    DELAY_US(5000);

    DRV8301_ReadStatusPins();
    DRV8301_ReadAllRegs();
    DRV8301_SetCurrentAmpGain(DRV8301_DEFAULT_GAIN_BITS);

    StartCurrentOffsetCalibration();
    WaitCurrentOffsetCalibrationDone();
    CalibrateVbusDividerFromKnownBus(VBUS_CAL_MEASURED_V);

    EQEP1_ResetEstimator();
    EQEP1_UpdatePositionFromCount();
    App_ForceStop();
    PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
    PWM_EnableOutputs(0U);

    for(;;)
    {
        if(gSpiServiceReq)
        {
            gSpiServiceReq = 0U;
            DRV8301_ReadAllRegs();
        }
    }
}

// ===================== Helper =====================
static inline float ClampF(float x, float xmin, float xmax)
{
    if(x < xmin) return xmin;
    if(x > xmax) return xmax;
    return x;
}

static inline float WrapAngle_0_2pi(float x)
{
    while(x >= TWO_PI_F) x -= TWO_PI_F;
    while(x < 0.0f) x += TWO_PI_F;
    return x;
}

static inline float PI_Run(volatile PI_Controller_t *p, float ref, float fb, float Ts)
{
    float err, up, ui_new, u;

    err = ref - fb;
    up = p->Kp * err;
    ui_new = p->Ui + (p->Ki * Ts * err);
    u = up + ui_new;

    if(u > p->OutMax)
    {
        u = p->OutMax;
        if(err > 0.0f) ui_new = p->Ui;
    }
    else if(u < p->OutMin)
    {
        u = p->OutMin;
        if(err < 0.0f) ui_new = p->Ui;
    }

    p->Ui = ui_new;
    return u;
}

static inline float RampToTarget(float current, float target, float maxStep)
{
    float err = target - current;
    if(err > maxStep)  return current + maxStep;
    if(err < -maxStep) return current - maxStep;
    return target;
}

// ===================== Init =====================
static void GPIO_InitDrv8301Pins(void)
{
    EALLOW;

    GPIO_SetupPinMux(PIN_DRV_FAULT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(PIN_DRV_FAULT, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(PIN_DRV_OCTW, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(PIN_DRV_OCTW, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(PIN_DRV_EN_GATE, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(PIN_DRV_EN_GATE, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(PIN_DRV_DC_CAL, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(PIN_DRV_DC_CAL, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(PIN_DRV_SPI_CS, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(PIN_DRV_SPI_CS, GPIO_OUTPUT, GPIO_PUSHPULL);

    EDIS;
    DRV8301_CS_High();
}

static void PWM_InitGpioOutputs_Motor1(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    EDIS;
}

static void ADC_InitModules(void)
{
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;

    AdcSetMode(ADC_ADCA, ADC_BITRESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_BITRESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_BITRESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    EDIS;
    DELAY_US(1000);
}

static void ADC_InitSOCs(void)
{
    EALLOW;

    // Ayný kanal eþleþmeleri korunmuþtur:
    //   Adcc SOC0 -> phase current A
    //   Adcb SOC0 -> phase current B
    //   Adca SOC1 -> DC bus
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = 2;
    AdccRegs.ADCSOC0CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = 2;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 14;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL  = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E    = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;

    EDIS;
}

static void EQEP1_InitGpio(void)
{
    GPIO_SetupPinMux(20U, GPIO_MUX_CPU1, 1U);    // EQEP1A
    GPIO_SetupPinOptions(20U, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(21U, GPIO_MUX_CPU1, 1U);    // EQEP1B
    GPIO_SetupPinOptions(21U, GPIO_INPUT, GPIO_PULLUP);

#if ENCODER_USE_EQEP1_INDEX
    // LaunchPad routing: GPIO99 / EQEP1I1
    GPIO_SetupPinMux(99U, GPIO_MUX_CPU1, 2U);
    GPIO_SetupPinOptions(99U, GPIO_INPUT, GPIO_PULLUP);
#endif
}

static void EQEP1_InitModule(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1U;
    EDIS;

    EQep1Regs.QDECCTL.all = 0U;
    EQep1Regs.QEPCTL.all  = 0U;
    EQep1Regs.QCAPCTL.all = 0U;

    EQep1Regs.QDECCTL.bit.QSRC = 0U;        // Quadrature count mode
    EQep1Regs.QDECCTL.bit.SWAP = ENCODER_SWAP_AB ? 1U : 0U;

    EQep1Regs.QPOSINIT = 0U;
    EQep1Regs.QPOSMAX  = (Uint32)(ENCODER_COUNTS_PER_REV - 1U);
    EQep1Regs.QPOSCNT  = 0U;

    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2U;
    EQep1Regs.QEPCTL.bit.PCRM      = 1U;    // Reset on max position
    EQep1Regs.QEPCTL.bit.IEI       = 0U;
    EQep1Regs.QEPCTL.bit.SWI       = 1U;
    EQep1Regs.QEPCTL.bit.QPEN      = 1U;

    EQep1Regs.QCLR.all = 0xFFFFU;
}

static void EQEP1_ResetEstimator(void)
{
    gQepCount = (int32_t)EQep1Regs.QPOSCNT;
    gQepPrevCount = gQepCount;
    gQepDeltaCount = 0;
    gQepDeltaAcc = 0;
    gQepSpeedWindowCount = 0U;
    gQepIndexSeen = 0U;

    gSpeedMechFbRpm = 0.0f;
    gSpeedMechFbRpmFilt = 0.0f;
    gOmegaMechFbRad_s = 0.0f;
    gOmegaElecFbRad_s = 0.0f;

    EQEP1_UpdatePositionFromCount();
}

static void EQEP1_ResetAlignOffsetAccumulator(void)
{
    gAlignQepCountSum = 0.0f;
    gAlignQepSampleCount = 0U;
}

static void EQEP1_RunAlignOffsetAccumulator_1kHz(void)
{
    if(gAlignCounterMs >= (ALIGN_TIME_MS - ALIGN_OFFSET_SAMPLE_MS))
    {
        gAlignQepCountSum += (float)((Uint32)EQep1Regs.QPOSCNT);
        gAlignQepSampleCount++;
    }
}

static void EQEP1_UpdatePositionFromCount(void)
{
    float thetaMech;

    gQepCount = (int32_t)EQep1Regs.QPOSCNT;
    thetaMech = ((float)gQepCount * TWO_PI_F) / (float)ENCODER_COUNTS_PER_REV;

    gThetaMechEncRad = WrapAngle_0_2pi(thetaMech);
    gThetaElecEncRad = WrapAngle_0_2pi((ENCODER_MECH_SIGN * MOTOR_POLE_PAIRS * gThetaMechEncRad) +
                                       gThetaElecOffsetRad +
                                       ENCODER_ELEC_ZERO_BIAS_RAD +
                                       ENCODER_OFFSET_EXTRA_RAD);

#if ENCODER_USE_EQEP1_INDEX
    if(EQep1Regs.QFLG.bit.IEL == 1U)
    {
        gQepIndexSeen = 1U;
        EQep1Regs.QCLR.bit.IEL = 1U;
    }
#endif
}

static void EQEP1_CaptureElecOffsetFromAlign(void)
{
    float avgCount;
    float thetaMechAvg;

    if(gAlignQepSampleCount == 0U)
    {
        avgCount = (float)((Uint32)EQep1Regs.QPOSCNT);
    }
    else
    {
        avgCount = gAlignQepCountSum / (float)gAlignQepSampleCount;
    }

    thetaMechAvg = (avgCount * TWO_PI_F) / (float)ENCODER_COUNTS_PER_REV;
    thetaMechAvg = WrapAngle_0_2pi(thetaMechAvg);

    gThetaElecOffsetRad = WrapAngle_0_2pi(ALIGN_ELEC_ANGLE_RAD -
                                          (ENCODER_MECH_SIGN * MOTOR_POLE_PAIRS * thetaMechAvg));

    EQEP1_UpdatePositionFromCount();
}

static void EQEP1_UpdateSpeed_1kHz(void)
{
    // Hýz hesabý 1 kHz zaman tabanýnda, pozisyon bilgisi ise ayrýca ADC ISR içinde de güncellenir.
    int32_t posNow, delta, halfCounts;
    float speedInstRpm;

    posNow = (int32_t)EQep1Regs.QPOSCNT;
    delta = posNow - gQepPrevCount;
    halfCounts = (int32_t)(ENCODER_COUNTS_PER_REV / 2U);

    if(delta > halfCounts)
    {
        delta -= (int32_t)ENCODER_COUNTS_PER_REV;
    }
    else if(delta < -halfCounts)
    {
        delta += (int32_t)ENCODER_COUNTS_PER_REV;
    }

    gQepPrevCount = posNow;
    gQepCount = posNow;
    gQepDeltaCount = delta;

    speedInstRpm = ((((float)delta) * 60.0f) /
                   ((float)ENCODER_COUNTS_PER_REV * TIMER0_TS)) * ENCODER_MECH_SIGN;

    gSpeedMechFbRpm = speedInstRpm;
    gSpeedMechFbRpmFilt += SPEED_FILT_ALPHA *
                           (gSpeedMechFbRpm - gSpeedMechFbRpmFilt);

    if(fabsf(gSpeedMechFbRpmFilt) < 1.0f)
    {
        gSpeedMechFbRpmFilt = 0.0f;
    }

    gOmegaMechFbRad_s = gSpeedMechFbRpmFilt * (TWO_PI_F / 60.0f);
    gOmegaElecFbRad_s = gOmegaMechFbRad_s * MOTOR_POLE_PAIRS;

    EQEP1_UpdatePositionFromCount();
}


static Uint16 PWM_DeadtimeCounts(float deadtime_ns)
{
    float tclk_ns = 1.0e9f / (float)EPWMCLK_HZ;
    float counts_f = deadtime_ns / tclk_ns;

    if(counts_f < 1.0f)    counts_f = 1.0f;
    if(counts_f > 1023.0f) counts_f = 1023.0f;

    return (Uint16)(counts_f + 0.5f);
}

static void PWM_InitSingleEPwm(volatile struct EPWM_REGS *p)
{
    Uint16 dbCounts = PWM_DeadtimeCounts(DEADTIME_NS);

    p->TBCTL.bit.CTRMODE   = 3;
    p->TBCTL.bit.PHSEN     = 0;
    p->TBCTL.bit.PRDLD     = 0;
    p->TBCTL.bit.SYNCOSEL  = 3;
    p->TBCTL.bit.HSPCLKDIV = 0;
    p->TBCTL.bit.CLKDIV    = 0;
    p->TBCTR               = 0;
    p->TBPRD               = (Uint16)TBPRD_VAL;

    p->CMPA.bit.CMPA        = (Uint16)(0.5f * (float)TBPRD_VAL);
    p->CMPCTL.bit.LOADAMODE = 0;
    p->CMPCTL.bit.SHDWAMODE = 0;

    p->AQCTLA.all = 0;
    p->AQCTLA.bit.CAU = AQ_CLEAR;
    p->AQCTLA.bit.CAD = AQ_SET;

    p->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    p->DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    p->DBCTL.bit.IN_MODE  = DBA_ALL;

    p->DBRED.bit.DBRED = dbCounts;
    p->DBFED.bit.DBFED = dbCounts;

    p->TZCTL.bit.TZA = TZ_FORCE_LO;
    p->TZCTL.bit.TZB = TZ_FORCE_LO;
    p->TZCLR.bit.OST = 1U;
    p->TZFRC.bit.OST = 1U;

    p->TBCTL.bit.CTRMODE = 2;
}

static void PWM_InitInverter(void)
{
    EALLOW;
    PWM_InitSingleEPwm(&EPwm1Regs);
    PWM_InitSingleEPwm(&EPwm2Regs);
    PWM_InitSingleEPwm(&EPwm3Regs);

    EPwm1Regs.CMPB.bit.CMPB      = (Uint16)(TBPRD_VAL - 80U);
    EPwm1Regs.ETSEL.bit.SOCAEN   = 1U;
    EPwm1Regs.ETSEL.bit.SOCASEL  = 6U;
    EPwm1Regs.ETPS.bit.SOCAPRD   = 1U;
    EPwm1Regs.ETCLR.bit.SOCA     = 1U;
    EDIS;
}

static void PWM_EnableOutputs(Uint16 enable)
{
    EALLOW;
    if(enable)
    {
        EPwm1Regs.TZCLR.bit.OST = 1U;
        EPwm2Regs.TZCLR.bit.OST = 1U;
        EPwm3Regs.TZCLR.bit.OST = 1U;
        gPwmEnabled = 1U;
    }
    else
    {
        EPwm1Regs.TZFRC.bit.OST = 1U;
        EPwm2Regs.TZFRC.bit.OST = 1U;
        EPwm3Regs.TZFRC.bit.OST = 1U;
        gPwmEnabled = 0U;
    }
    EDIS;
}

static void PWM_UpdateDutyABC(float dutyA, float dutyB, float dutyC)
{
    Uint16 cmpA, cmpB, cmpC;

    dutyA = ClampF(dutyA, DUTY_MIN, DUTY_MAX);
    dutyB = ClampF(dutyB, DUTY_MIN, DUTY_MAX);
    dutyC = ClampF(dutyC, DUTY_MIN, DUTY_MAX);

    cmpA = (Uint16)(dutyA * (float)TBPRD_VAL);
    cmpB = (Uint16)(dutyB * (float)TBPRD_VAL);
    cmpC = (Uint16)(dutyC * (float)TBPRD_VAL);

    EPwm1Regs.CMPA.bit.CMPA = cmpA;
    EPwm2Regs.CMPA.bit.CMPA = cmpB;
    EPwm3Regs.CMPA.bit.CMPA = cmpC;

    gDutyA = dutyA;
    gDutyB = dutyB;
    gDutyC = dutyC;
}

static void CPU_TIMER0_Init1kHz(void)
{
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200.0f, 1000.0f);

    CpuTimer0Regs.TCR.bit.TSS  = 1U;
    CpuTimer0Regs.TCR.bit.TRB  = 1U;
    CpuTimer0Regs.TCR.bit.TIE  = 1U;
    CpuTimer0Regs.TCR.bit.SOFT = 1U;
    CpuTimer0Regs.TCR.bit.FREE = 1U;
    CpuTimer0Regs.TCR.bit.TIF  = 1U;
    CpuTimer0Regs.TCR.bit.TSS  = 0U;
}

// ===================== DRV8301 =====================
static void DRV8301_SetDCCal(Uint16 enable)
{
    GPIO_WritePin(PIN_DRV_DC_CAL, enable ? 1U : 0U);
    gDcCalPinState = enable ? 1U : 0U;
}

static void DRV8301_SetGateEnable(Uint16 enable)
{
    GPIO_WritePin(PIN_DRV_EN_GATE, enable ? 1U : 0U);
    gGateEnableState = enable ? 1U : 0U;
}

static void DRV8301_ReadStatusPins(void)
{
    gFaultState = GPIO_ReadPin(PIN_DRV_FAULT);
    gOctwState  = GPIO_ReadPin(PIN_DRV_OCTW);

    gFaultActive = (gFaultState == 0U) ? 1U : 0U;
    gOctwActive  = (gOctwState  == 0U) ? 1U : 0U;
}

static void SPIA_InitDrv8301(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1U;

    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;

    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;

    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;
    EDIS;

    SpiaRegs.SPICCR.bit.SPISWRESET  = 0U;
    SpiaRegs.SPICCR.bit.SPICHAR     = 15U;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0U;
    SpiaRegs.SPICCR.bit.SPILBK      = 0U;

    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1U;
    SpiaRegs.SPICTL.bit.TALK         = 1U;
    SpiaRegs.SPICTL.bit.CLK_PHASE    = 0U;
    SpiaRegs.SPICTL.bit.SPIINTENA    = 0U;

    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 49U;
    SpiaRegs.SPIPRI.bit.FREE = 1U;
    SpiaRegs.SPIPRI.bit.SOFT = 1U;

    SpiaRegs.SPICCR.bit.SPISWRESET = 1U;
}

static inline void DRV8301_CS_Low(void)
{
    GPIO_WritePin(PIN_DRV_SPI_CS, 0U);
}

static inline void DRV8301_CS_High(void)
{
    GPIO_WritePin(PIN_DRV_SPI_CS, 1U);
}

static Uint16 SPIA_Transfer16(Uint16 data)
{
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1U) {}
    SpiaRegs.SPITXBUF = data;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0U) {}
    gDrvLastRxWord = SpiaRegs.SPIRXBUF;
    return gDrvLastRxWord;
}

static Uint16 DRV8301_ReadReg(Uint16 addr)
{
    Uint16 cmd, resp;
    cmd = 0x8000U | ((addr & 0x000FU) << 11);

    DRV8301_CS_Low();
    SPIA_Transfer16(cmd);
    DRV8301_CS_High();

    asm(" RPT #20 || NOP");

    DRV8301_CS_Low();
    resp = SPIA_Transfer16(0x0000U);
    DRV8301_CS_High();

    return resp;
}

static Uint16 DRV8301_WriteReg(Uint16 addr, Uint16 data11)
{
    Uint16 cmd, resp;
    cmd = ((addr & 0x000FU) << 11) | (data11 & 0x07FFU);

    DRV8301_CS_Low();
    SPIA_Transfer16(cmd);
    DRV8301_CS_High();

    asm(" RPT #20 || NOP");

    DRV8301_CS_Low();
    resp = SPIA_Transfer16(0x0000U);
    DRV8301_CS_High();

    return resp;
}

static void DRV8301_ReadAllRegs(void)
{
    gDrvStat1Raw = DRV8301_ReadReg(DRV8301_REG_STATUS1);
    gDrvStat1Raw = DRV8301_ReadReg(DRV8301_REG_STATUS1);

    gDrvStat2Raw = DRV8301_ReadReg(DRV8301_REG_STATUS2);
    gDrvStat2Raw = DRV8301_ReadReg(DRV8301_REG_STATUS2);

    gDrvCtrl1Raw = DRV8301_ReadReg(DRV8301_REG_CONTROL1);
    gDrvCtrl1Raw = DRV8301_ReadReg(DRV8301_REG_CONTROL1);

    gDrvCtrl2Raw = DRV8301_ReadReg(DRV8301_REG_CONTROL2);
    gDrvCtrl2Raw = DRV8301_ReadReg(DRV8301_REG_CONTROL2);

    gDrvStat1Data = gDrvStat1Raw & 0x07FFU;
    gDrvStat2Data = gDrvStat2Raw & 0x07FFU;
    gDrvCtrl1Data = gDrvCtrl1Raw & 0x07FFU;
    gDrvCtrl2Data = gDrvCtrl2Raw & 0x07FFU;

    gDrvFrameFault = (gDrvStat1Raw >> 15) & 0x1U;
    gDrvDeviceID   = (gDrvStat2Data & 0x000FU);
}

static void DRV8301_SetCurrentAmpGain(Uint16 gainBits)
{
    Uint16 ctrl2 = gDrvCtrl2Data;

    ctrl2 &= ~(0x000CU | 0x0003U);
    ctrl2 |= DRV8301_CTRL2_OCTW_BOTH;
    ctrl2 |= gainBits;

    DRV8301_WriteReg(DRV8301_REG_CONTROL2, ctrl2);
    DELAY_US(50);
    DRV8301_ReadAllRegs();

    switch(gainBits)
    {
        case DRV8301_CTRL2_GAIN_10VV: gCsaGain_VV = 10.0f; break;
        case DRV8301_CTRL2_GAIN_20VV: gCsaGain_VV = 20.0f; break;
        case DRV8301_CTRL2_GAIN_40VV: gCsaGain_VV = 40.0f; break;
        case DRV8301_CTRL2_GAIN_80VV: gCsaGain_VV = 80.0f; break;
        default: gCsaGain_VV = 20.0f; break;
    }
}

// ===================== Calibration =====================
static void StartCurrentOffsetCalibration(void)
{
    gIA_sum = 0U;
    gFilterInitDone = 0U;
    gIB_sum = 0U;
    gCalCount = 0U;
    gCalDone = 0U;
    gCalActive = 1U;

    PWM_EnableOutputs(0U);
    DRV8301_SetGateEnable(1U);
    DRV8301_SetDCCal(1U);

    DELAY_US(2000);
}

static void WaitCurrentOffsetCalibrationDone(void)
{
    while(gCalDone == 0U) {}

    gIA_offset_counts = ((float)gIA_sum) / (float)CURR_OFFSET_SAMPLES;
    gIB_offset_counts = ((float)gIB_sum) / (float)CURR_OFFSET_SAMPLES;

    DRV8301_SetDCCal(0U);
    DELAY_US(1000);
    DRV8301_ReadAllRegs();
}

static void CalibrateVbusDividerFromKnownBus(float measuredBusV)
{
    float adcPinV = ((float)gVBS_raw * ADC_VREF) / ADC_MAX_COUNT;

    if((measuredBusV > 0.1f) && (adcPinV > 0.01f))
    {
        gVbusDividerGain = measuredBusV / adcPinV;
        gVbusSatBus_V = ADC_VREF * gVbusDividerGain;
    }
}

// ===================== ISR içi ölçüm / kontrol =====================
static inline void ConvertRawToPhysical(void)
{
    gIA_adc_V  = ((float)gIA_raw  * ADC_VREF) / ADC_MAX_COUNT;
    gIB_adc_V  = ((float)gIB_raw  * ADC_VREF) / ADC_MAX_COUNT;
    gVBS_adc_V = ((float)gVBS_raw * ADC_VREF) / ADC_MAX_COUNT;

    gIA_corr_counts = (int32_t)gIA_raw - (int32_t)(gIA_offset_counts + 0.5f);
    gIB_corr_counts = (int32_t)gIB_raw - (int32_t)(gIB_offset_counts + 0.5f);

    gIA_corr_V = ((float)gIA_corr_counts * ADC_VREF) / ADC_MAX_COUNT;
    gIB_corr_V = ((float)gIB_corr_counts * ADC_VREF) / ADC_MAX_COUNT;

    gIA_A = IA_SIGN * (gIA_corr_V / (gCsaGain_VV * CURRENT_SHUNT_OHM));
    gIB_A = IB_SIGN * (gIB_corr_V / (gCsaGain_VV * CURRENT_SHUNT_OHM));
    gIC_A = -(gIA_A + gIB_A);

    gVBS_bus_V = (((float)gVBS_raw * ADC_VREF) / ADC_MAX_COUNT) * gVbusDividerGain;
    gVbusSatBus_V = ADC_VREF * gVbusDividerGain;
}

static inline void UpdateFiltersAndAlarms_ISR(void)
{
    if(gFilterInitDone == 0U)
    {
        gIA_A_filt = gIA_A;
        gIB_A_filt = gIB_A;
        gVBS_bus_V_filt = gVBS_bus_V;
        gFilterInitDone = 1U;
    }
    else
    {
        gIA_A_filt += I_FILT_ALPHA * (gIA_A - gIA_A_filt);
        gIB_A_filt += I_FILT_ALPHA * (gIB_A - gIB_A_filt);
        gVBS_bus_V_filt += VBUS_FILT_ALPHA * (gVBS_bus_V - gVBS_bus_V_filt);
    }

    gVbusNearClip = (gVBS_raw >= VBUS_NEAR_CLIP_RAW) ? 1U : 0U;
    gVbusSaturated = (gVBS_raw >= VBUS_SAT_RAW) ? 1U : 0U;
}

static void SVPWM_Apply(float vAlpha, float vBeta, float vdc)
{
    float va, vb, vc;
    float vmax, vmin, vcom;
    float dutyA, dutyB, dutyC;

    if(vdc < 1.0f)
    {
        PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
        return;
    }

    va = vAlpha;
    vb = -0.5f * vAlpha + SQRT3_BY_2_F * vBeta;
    vc = -0.5f * vAlpha - SQRT3_BY_2_F * vBeta;

    vmax = va;
    if(vb > vmax) vmax = vb;
    if(vc > vmax) vmax = vc;

    vmin = va;
    if(vb < vmin) vmin = vb;
    if(vc < vmin) vmin = vc;

    vcom = 0.5f * (vmax + vmin);

    dutyA = 0.5f + ((va - vcom) / vdc);
    dutyB = 0.5f + ((vb - vcom) / vdc);
    dutyC = 0.5f + ((vc - vcom) / vdc);

    PWM_UpdateDutyABC(dutyA, dutyB, dutyC);
}

static void Align_ApplyVoltageVector(float theta_e_align, float modIndex, float vdc)
{
    float vLimit = 0.577f * vdc * FOC_VOLT_LIMIT_FACTOR;
    float vMag   = modIndex * vLimit;
    float vAlpha = vMag * cosf(theta_e_align);
    float vBeta  = vMag * sinf(theta_e_align);

    SVPWM_Apply(vAlpha, vBeta, vdc);
}

static void FOC_ResetControllers(void)
{
    gIdPI.Ui = 0.0f;
    gIqPI.Ui = 0.0f;
    gSpeedPI.Ui = 0.0f;

    gIalpha_A = 0.0f;
    gIbeta_A  = 0.0f;
    gId_A     = 0.0f;
    gIq_A     = 0.0f;

    gIdRefCmd_A = OPENLOOP_ID_A;
    gIqRefCmd_A = 0.0f;
    gIqRefTarget_A = 0.0f;

    gVdCmd_V = 0.0f;
    gVqCmd_V = 0.0f;
    gValphaCmd_V = 0.0f;
    gVbetaCmd_V = 0.0f;
}

static void FOC_RunCurrentLoop_ISR(void)
{
    float c, s;
    float iAlpha, iBeta;
    float id, iq;
    float vd_pi, vq_pi;
    float vd, vq;
    float vAlpha, vBeta;
    float vdc, vLimit, vNorm, scale;
    float omega_e;

    vdc = gVBS_bus_V;
    if(vdc < FOC_MIN_RUN_VBUS_V)
    {
        PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
        return;
    }

    c = cosf(gThetaCtrlRad);
    s = sinf(gThetaCtrlRad);

    iAlpha = gIA_A;
    iBeta  = (gIA_A + 2.0f * gIB_A) * INV_SQRT3_F;

    id =  c * iAlpha + s * iBeta;
    iq = -s * iAlpha + c * iBeta;

    gIalpha_A = iAlpha;
    gIbeta_A  = iBeta;
    gId_A     = id;
    gIq_A     = iq;

    vLimit = 0.577f * vdc * FOC_VOLT_LIMIT_FACTOR;
    gVdqLimit_V = vLimit;

    gIdPI.OutMin = -vLimit;
    gIdPI.OutMax =  vLimit;
    gIqPI.OutMin = -vLimit;
    gIqPI.OutMax =  vLimit;

    vd_pi = PI_Run(&gIdPI, gIdRefCmd_A, id, CTRL_TS);
    vq_pi = PI_Run(&gIqPI, gIqRefCmd_A, iq, CTRL_TS);

#if USE_DECOUPLING_FF
    omega_e = gOmegaElecFbRad_s;
    vd = vd_pi - (omega_e * MOTOR_LS_PHASE_H * iq);
    vq = vq_pi + (omega_e * (MOTOR_LS_PHASE_H * id + PM_FLUX_WB));
#else
    omega_e = 0.0f;
    (void)omega_e;
    vd = vd_pi;
    vq = vq_pi;
#endif

    vNorm = sqrtf((vd * vd) + (vq * vq));
    if(vNorm > vLimit)
    {
        scale = vLimit / vNorm;
        vd *= scale;
        vq *= scale;
    }

    gVdCmd_V = vd;
    gVqCmd_V = vq;

    vAlpha = c * vd - s * vq;
    vBeta  = s * vd + c * vq;

    gValphaCmd_V = vAlpha;
    gVbetaCmd_V  = vBeta;

    SVPWM_Apply(vAlpha, vBeta, vdc);
}

static void App_ForceStop(void)
{
    gAppState = APP_STATE_STOP;
    gAlignCounterMs = 0U;
    gVerifyCounterMs = 0U;

    gSpeedCmdRpmRamp = 0.0f;
    gOmegaMechCmdRad_s = 0.0f;
    gOmegaElecCmdRad_s = 0.0f;

    gThetaOpenloopRad = ALIGN_ELEC_ANGLE_RAD;
    gThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
    gThetaElecOffsetRad = 0.0f;
    gTractionDriveEnable = 0U;
    gOverCurrentTrip = 0U;

    EQEP1_ResetAlignOffsetAccumulator();
    EQEP1_ResetEstimator();
    FOC_ResetControllers();
    PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
    PWM_EnableOutputs(0U);
}

static void App_RunStateMachine_1kHz(void)
{
    float signRef;
    float speedTargetRpm;
    float speedRampStep;
    float iqRampStep;

    EQEP1_UpdateSpeed_1kHz();

    if(gAppState == APP_STATE_FAULT)
    {
        PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
        PWM_EnableOutputs(0U);
        FOC_ResetControllers();
        gStartCmdPrev = gStartCmd;
        return;
    }

    if((gStartCmd != 0U) && (gStartCmdPrev == 0U))
    {
        EQEP1_ResetAlignOffsetAccumulator();
        EQEP1_ResetEstimator();
        FOC_ResetControllers();
        gOverCurrentTrip = 0U;
        gAlignCounterMs = 0U;
        gVerifyCounterMs = 0U;
        gThetaOpenloopRad = ALIGN_ELEC_ANGLE_RAD;
        gThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
        gSpeedCmdRpmRamp = 0.0f;
        gTractionDriveEnable = 0U;
        gOmegaMechCmdRad_s = 0.0f;
        gOmegaElecCmdRad_s = 0.0f;
        PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
        PWM_EnableOutputs(1U);
        gAppState = APP_STATE_ALIGN;
    }

    if(gStartCmd == 0U)
    {
        if(gAppState != APP_STATE_STOP)
        {
            App_ForceStop();
        }
        gStartCmdPrev = gStartCmd;
        return;
    }

    signRef = (gDirectionCmd >= 0) ? 1.0f : -1.0f;
    speedTargetRpm = signRef * fabsf(gSpeedRefRpmCmd);
    speedRampStep = CLOSEDLOOP_SPEED_RAMP_RPM_PER_S * TIMER0_TS;
    iqRampStep = IQ_REF_SLEW_A_PER_S * TIMER0_TS;

    switch(gAppState)
    {
        case APP_STATE_STOP:
            break;

        case APP_STATE_ALIGN:
            gAlignCounterMs++;
            EQEP1_RunAlignOffsetAccumulator_1kHz();
            gSpeedCmdRpmRamp = 0.0f;
            gOmegaMechCmdRad_s = 0.0f;
            gOmegaElecCmdRad_s = 0.0f;
            gIdRefCmd_A = OPENLOOP_ID_A;
            gIqRefTarget_A = 0.0f;
            gIqRefCmd_A = RampToTarget(gIqRefCmd_A, gIqRefTarget_A, iqRampStep);

            if(gAlignCounterMs >= ALIGN_TIME_MS)
            {
                EQEP1_CaptureElecOffsetFromAlign();
                EQEP1_ResetEstimator();

                gThetaCtrlRad = gThetaElecEncRad;
                gSpeedCmdRpmRamp = 0.0f;
                gOmegaMechCmdRad_s = 0.0f;
                gOmegaElecCmdRad_s = 0.0f;
                gIdRefCmd_A = OPENLOOP_ID_A;
                gIqRefCmd_A = 0.0f;
                gIqRefTarget_A = 0.0f;
                gSpeedPI.Ui = 0.0f;
                gTractionDriveEnable = 0U;
                gAppState = APP_STATE_CLOSEDLOOP;
            }
            break;

        case APP_STATE_VERIFY:
            gVerifyCounterMs++;
            gSpeedCmdRpmRamp = 0.0f;
            gOmegaMechCmdRad_s = 0.0f;
            gOmegaElecCmdRad_s = 0.0f;
            gIdRefCmd_A = OPENLOOP_ID_A;
            gIqRefTarget_A = signRef * VERIFY_IQ_A;
            gIqRefCmd_A = RampToTarget(gIqRefCmd_A, gIqRefTarget_A, iqRampStep);

            if(gVerifyCounterMs >= VERIFY_TIME_MS)
            {
                gSpeedCmdRpmRamp = 0.0f;
                gOmegaMechCmdRad_s = 0.0f;
                gOmegaElecCmdRad_s = 0.0f;
                gIdRefCmd_A = OPENLOOP_ID_A;
                gIqRefCmd_A = 0.0f;
                gIqRefTarget_A = 0.0f;
                gSpeedPI.Ui = 0.0f;
                gTractionDriveEnable = 0U;
                gAppState = APP_STATE_CLOSEDLOOP;
            }
            break;

        case APP_STATE_CLOSEDLOOP:
        {
            float speedErr;
            float iqCmd;

            gSpeedCmdRpmRamp = RampToTarget(gSpeedCmdRpmRamp, speedTargetRpm, speedRampStep);
            gOmegaMechCmdRad_s = gSpeedCmdRpmRamp * (TWO_PI_F / 60.0f);
            gOmegaElecCmdRad_s = gOmegaMechCmdRad_s * MOTOR_POLE_PAIRS;
            gIdRefCmd_A = OPENLOOP_ID_A;

            if(fabsf(gSpeedCmdRpmRamp) < 1.0f)
            {
                gTractionDriveEnable = 0U;
                gSpeedPI.Ui = 0.0f;
                gIqRefTarget_A = 0.0f;
            }
            else if(gDirectionCmd >= 0)
            {
                speedErr = gSpeedCmdRpmRamp - gSpeedMechFbRpmFilt;

                if(speedErr >= TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    gTractionDriveEnable = 1U;
                }
                else if(speedErr <= -TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    gTractionDriveEnable = 0U;
                    gSpeedPI.Ui = 0.0f;
                }

                if(gTractionDriveEnable)
                {
                    iqCmd = PI_Run(&gSpeedPI, gSpeedCmdRpmRamp, gSpeedMechFbRpmFilt, TIMER0_TS);
                    iqCmd = ClampF(iqCmd, 0.0f, SPEED_PI_OUT_MAX_A);

                    if((gSpeedCmdRpmRamp > 20.0f) && (speedErr > TRACTION_SPEED_EXIT_BAND_RPM) &&
                       (iqCmd < TRACTION_MIN_RUN_IQ_A))
                    {
                        iqCmd = TRACTION_MIN_RUN_IQ_A;
                    }

                    gIqRefTarget_A = iqCmd;
                }
                else
                {
                    gIqRefTarget_A = 0.0f;
                }
            }
            else
            {
                speedErr = gSpeedCmdRpmRamp - gSpeedMechFbRpmFilt;

                if(speedErr <= -TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    gTractionDriveEnable = 1U;
                }
                else if(speedErr >= TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    gTractionDriveEnable = 0U;
                    gSpeedPI.Ui = 0.0f;
                }

                if(gTractionDriveEnable)
                {
                    iqCmd = PI_Run(&gSpeedPI, gSpeedCmdRpmRamp, gSpeedMechFbRpmFilt, TIMER0_TS);
                    iqCmd = ClampF(iqCmd, -SPEED_PI_OUT_MAX_A, 0.0f);

                    if((gSpeedCmdRpmRamp < -20.0f) && (speedErr < -TRACTION_SPEED_EXIT_BAND_RPM) &&
                       (iqCmd > -TRACTION_MIN_RUN_IQ_A))
                    {
                        iqCmd = -TRACTION_MIN_RUN_IQ_A;
                    }

                    gIqRefTarget_A = iqCmd;
                }
                else
                {
                    gIqRefTarget_A = 0.0f;
                }
            }

            gIqRefCmd_A = RampToTarget(gIqRefCmd_A, gIqRefTarget_A, iqRampStep);
            break;
        }

        case APP_STATE_FAULT:
        default:
            break;
    }

    gStartCmdPrev = gStartCmd;
}

static inline void UpdateDebugSnapshot(void)
{
    gDbg.CsaGain_VV = gCsaGain_VV;
    gDbg.PwmEnabled = gPwmEnabled;
    gDbg.AppState = (Uint16)gAppState;
    gDbg.StartCmd = gStartCmd;
    gDbg.DirectionCmd = gDirectionCmd;

    gDbg.IA_A = gIA_A;
    gDbg.IB_A = gIB_A;
    gDbg.IC_A = gIC_A;
    gDbg.Vbus_V = gVBS_bus_V;

    gDbg.IA_A_Filt = gIA_A_filt;
    gDbg.IB_A_Filt = gIB_A_filt;
    gDbg.Vbus_V_Filt = gVBS_bus_V_filt;

    gDbg.SpeedRefRpmCmd = gSpeedRefRpmCmd;
    gDbg.SpeedCmdRpmRamp = gSpeedCmdRpmRamp;
    gDbg.SpeedFbRpm = gSpeedMechFbRpm;
    gDbg.SpeedFbRpmFilt = gSpeedMechFbRpmFilt;
    gDbg.OmegaMechCmdRad_s = gOmegaMechCmdRad_s;
    gDbg.OmegaElecCmdRad_s = gOmegaElecCmdRad_s;
    gDbg.OmegaMechFbRad_s = gOmegaMechFbRad_s;
    gDbg.OmegaElecFbRad_s = gOmegaElecFbRad_s;

    gDbg.ThetaCtrlRad = gThetaCtrlRad;
    gDbg.ThetaMechEncRad = gThetaMechEncRad;
    gDbg.ThetaElecEncRad = gThetaElecEncRad;
    gDbg.ThetaElecOffsetRad = gThetaElecOffsetRad;

    gDbg.IdRef_A = gIdRefCmd_A;
    gDbg.IqRefTarget_A = gIqRefTarget_A;
    gDbg.IqRefCmd_A = gIqRefCmd_A;

    gDbg.Id_A = gId_A;
    gDbg.Iq_A = gIq_A;

    gDbg.VdCmd_V = gVdCmd_V;
    gDbg.VqCmd_V = gVqCmd_V;

    gDbg.DutyA = gDutyA;
    gDbg.DutyB = gDutyB;
    gDbg.DutyC = gDutyC;

    gDbg.FaultActive = gFaultActive;
    gDbg.OctwActive = gOctwActive;
    gDbg.QepIndexSeen = gQepIndexSeen;
    gDbg.QepCount = gQepCount;
    gDbg.QepDeltaCount = gQepDeltaCount;
    gDbg.AlignCounterMs = gAlignCounterMs;
    gDbg.Timer0IsrCount = gTimer0IsrCount;
    gDbg.AdcIsrCount = gAdcIsrCount;

    gDbg.seq++;
}

// ===================== 1 kHz Timer ISR =====================
interrupt void cpu_timer0_isr(void)
{
    gTimer0IsrCount++;
    App_RunStateMachine_1kHz();

    CpuTimer0Regs.TCR.bit.TIF = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// ===================== ADC ISR =====================
interrupt void adca1_isr(void)
{
    gIA_raw  = AdccResultRegs.ADCRESULT0;
    gIB_raw  = AdcbResultRegs.ADCRESULT0;
    gVBS_raw = AdcaResultRegs.ADCRESULT1;

    gAdcIsrCount++;

    if(gCalActive)
    {
        gIA_sum += gIA_raw;
        gIB_sum += gIB_raw;
        gCalCount++;

        if(gCalCount >= CURR_OFFSET_SAMPLES)
        {
            gCalActive = 0U;
            gCalDone = 1U;
        }
    }

    ConvertRawToPhysical();
    UpdateFiltersAndAlarms_ISR();
    DRV8301_ReadStatusPins();

    if((gFaultActive != 0U) ||
       (fabsf(gIA_A) > FOC_OC_LIMIT_A) ||
       (fabsf(gIB_A) > FOC_OC_LIMIT_A))
    {
        gOverCurrentTrip = ((fabsf(gIA_A) > FOC_OC_LIMIT_A) ||
                            (fabsf(gIB_A) > FOC_OC_LIMIT_A)) ? 1U : 0U;

        gAppState = APP_STATE_FAULT;
        PWM_EnableOutputs(0U);
        PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
        FOC_ResetControllers();
    }
    else
    {
        switch(gAppState)
        {
            case APP_STATE_STOP:
                PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
                break;

            case APP_STATE_ALIGN:
                gThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
                Align_ApplyVoltageVector(ALIGN_ELEC_ANGLE_RAD, ALIGN_MOD_INDEX, gVBS_bus_V);
                break;

            case APP_STATE_VERIFY:
                EQEP1_UpdatePositionFromCount();
                gThetaCtrlRad = gThetaElecEncRad;
                FOC_RunCurrentLoop_ISR();
                break;

            case APP_STATE_CLOSEDLOOP:
                EQEP1_UpdatePositionFromCount();
                gThetaCtrlRad = gThetaElecEncRad;
                FOC_RunCurrentLoop_ISR();
                break;

            case APP_STATE_FAULT:
            default:
                PWM_UpdateDutyABC(0.5f, 0.5f, 0.5f);
                PWM_EnableOutputs(0U);
                break;
        }
    }

    if((gAdcIsrCount % 2000U) == 0U)
    {
        gSpiServiceReq = 1U;
    }

    UpdateDebugSnapshot();

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;

    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 == 1U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
