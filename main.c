
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
#define SQRT3_F                    1.7320508075688772f

// ===================== Motor / Inverter =====================
#define MOTOR_POLE_PAIRS              4.0f
#define MOTOR_RS_PHASE_OHM            0.36f
#define MOTOR_LS_PHASE_H              0.00020f
#define PM_FLUX_WB                    0.0064f

#define CURRENT_SHUNT_OHM             0.01f
#define IA_SIGN                       (1.0f)
#define IB_SIGN                       (1.0f)

// ===================== Ölçüm / Koruma =====================
#define VBUS_NEAR_CLIP_RAW            4000U
#define VBUS_SAT_RAW                  4088U
#define VBUS_CAL_MEASURED_V           24.0f
#define CURR_OFFSET_SAMPLES           2048U

#define I_FILT_ALPHA                  0.05f
#define VBUS_FILT_ALPHA               0.05f

#define FOC_OC_LIMIT_A                7.1f
#define FOC_MIN_RUN_VBUS_V            5.0f

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
#define TRACTION_SPEED_ENTER_BAND_RPM 15.0f
#define TRACTION_SPEED_EXIT_BAND_RPM   5.0f
#define TRACTION_MIN_RUN_IQ_A          0.18f
#define VERIFY_TIME_MS                 0U
#define VERIFY_IQ_A                    0.25f
#define ALIGN_OFFSET_SAMPLE_MS         200U

#define FOC_VOLT_LIMIT_FACTOR         0.95f
#define DUTY_MIN                      0.02f
#define DUTY_MAX                      0.98f
#define DEADTIME_NS                   500.0f

#define USE_DECOUPLING_FF             0

// ===================== ADC trigger seçimi =====================
// Tüm motor akým/Vbus örneklemeleri ayný anda, ePWM1 SOCA ile tetiklenir.
#define ADC_TRIGSEL_EPWM1_SOCA        5U

// Motor-1 ADC kanal/soc eþleþmeleri
#define M1_ADCC_SOC_IA                0U
#define M1_ADCB_SOC_IB                0U
#define M1_ADCA_SOC_VBUS              1U
#define M1_ADCC_CH_IA                 2U      // ADCINC2
#define M1_ADCB_CH_IB                 2U      // ADCINB2
#define M1_ADCA_CH_VBUS               14U     // ADCIN14

// Motor-2 ADC kanal/soc eþleþmeleri
#define M2_ADCC_SOC_IA                1U
#define M2_ADCB_SOC_IB                1U
#define M2_ADCA_SOC_VBUS              2U
#define M2_ADCC_CH_IA                 4U      // ADCINC4
#define M2_ADCB_CH_IB                 4U      // ADCINB4
#define M2_ADCA_CH_VBUS               15U     // ADCIN15


// ===================== Elektronik Diferansiyel / Ackermann =====================
// Varsayým:
//   Motor-1 = arka sol teker
//   Motor-2 = arka sað teker
//   + direksiyon açýsý = sola dönüþ
//   + taþýt hýzý = ileri yönde çizgisel hýz
#define ED_ENABLE_DEFAULT               1U
#define ED_MOTOR1_IS_REAR_LEFT          1U
#define ED_WHEELBASE_M                  0.60f
#define ED_TRACK_WIDTH_M                0.40f
#define ED_WHEEL_RADIUS_M               0.10f
#define ED_GEAR_RATIO                   5.0f
#define ED_VEHICLE_MAX_SPEED_MPS        8.0f
#define ED_STEER_LIMIT_DEG              35.0f
#define ED_MOTOR_RPM_LIMIT              5000.0f

// ===================== DRV8301 =====================
#define DRV8301_REG_STATUS1           0x0U
#define DRV8301_REG_STATUS2           0x1U
#define DRV8301_REG_CONTROL1          0x2U
#define DRV8301_REG_CONTROL2          0x3U

#define DRV8301_CTRL2_OCTW_BOTH       (0x0000U)
#define DRV8301_CTRL2_GAIN_10VV       (0x0000U)
#define DRV8301_CTRL2_GAIN_20VV       (0x0004U)
#define DRV8301_CTRL2_GAIN_40VV       (0x0008U)
#define DRV8301_CTRL2_GAIN_80VV       (0x000CU)
#define DRV8301_DEFAULT_GAIN_BITS     DRV8301_CTRL2_GAIN_20VV

typedef enum
{
    APP_STATE_STOP = 0,
    APP_STATE_ALIGN,
    APP_STATE_VERIFY,
    APP_STATE_CLOSEDLOOP,
    APP_STATE_FAULT
} AppState_e;

typedef enum
{
    SPI_MODULE_A = 0,
    SPI_MODULE_B = 1
} SpiModule_e;

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
    float vCmd_mps;
    float deltaDeg;
    float deltaRad;
    float absDeltaRad;
    float absV_mps;
    float speedSign;
    float turnRadius_m;
    float vInner_mps;
    float vOuter_mps;
    float vLeft_mps;
    float vRight_mps;
    float wLeftWheel_rpm;
    float wRightWheel_rpm;
    float mLeft_rpm;
    float mRight_rpm;
    float m1Ref_rpm;
    float m2Ref_rpm;
    float denom;
} ElectronicDiffDebug_t;

typedef struct
{
    SpiModule_e SpiModule;
    Uint16 PinFault;
    Uint16 PinOctw;
    Uint16 PinSpiSimo;
    Uint16 PinSpiSomi;
    Uint16 PinSpiClk;
    Uint16 PinSpiCs;
    Uint16 PinEnGate;
    Uint16 PinDcCal;

    Uint16 EqepModule;                 // 1 -> EQEP1, 2 -> EQEP2
    Uint16 PinEqepA;
    Uint16 PinEqepB;
    Uint16 PinEqepI;
    Uint16 EncoderUseIndex;
    Uint16 EncoderSwapAB;
    float  EncoderMechSign;
    Uint16 EncoderCountsPerRev;

    volatile struct EPWM_REGS *PwmA;
    volatile struct EPWM_REGS *PwmB;
    volatile struct EPWM_REGS *PwmC;

    Uint16 PwmGpioBase;                // Motor1: 0, Motor2: 6
} MotorHw_t;

typedef struct
{
    Uint16 IA_raw;
    Uint16 IB_raw;
    Uint16 VBS_raw;

    float IA_adc_V;
    float IB_adc_V;
    float VBS_adc_V;

    float IA_offset_counts;
    float IB_offset_counts;

    int32_t IA_corr_counts;
    int32_t IB_corr_counts;
    float IA_corr_V;
    float IB_corr_V;

    float CsaGain_VV;
    float IA_A;
    float IB_A;
    float IC_A;
    float IA_A_filt;
    float IB_A_filt;

    float VBS_bus_V;
    float VBS_bus_V_filt;
    float VbusDividerGain;
    float VbusSatBus_V;
    Uint16 VbusNearClip;
    Uint16 VbusSaturated;

    float Ialpha_A;
    float Ibeta_A;
    float Id_A;
    float Iq_A;

    float IdRefCmd_A;
    float IqRefCmd_A;
    float IqRefTarget_A;

    float SpeedRefRpmCmd;
    float SpeedCmdRpmRamp;
    float OmegaMechCmdRad_s;
    float OmegaElecCmdRad_s;

    float ThetaCtrlRad;
    float ThetaOpenloopRad;
    float ThetaMechEncRad;
    float ThetaElecEncRad;
    float ThetaElecOffsetRad;

    float SpeedMechFbRpm;
    float SpeedMechFbRpmFilt;
    float OmegaMechFbRad_s;
    float OmegaElecFbRad_s;

    int32_t QepCount;
    int32_t QepPrevCount;
    int32_t QepDeltaCount;
    Uint16 QepIndexSeen;
    float AlignQepCountSum;
    Uint32 AlignQepSampleCount;

    float VdCmd_V;
    float VqCmd_V;
    float ValphaCmd_V;
    float VbetaCmd_V;
    float VdqLimit_V;

    PI_Controller_t IdPI;
    PI_Controller_t IqPI;
    PI_Controller_t SpeedPI;

    Uint32 IA_sum;
    Uint32 IB_sum;
    Uint16 CalCount;
    Uint16 CalActive;
    Uint16 CalDone;

    Uint16 DcCalPinState;
    Uint16 GateEnableState;
    Uint16 FaultState;
    Uint16 OctwState;
    Uint16 FaultActive;
    Uint16 OctwActive;

    Uint16 DrvStat1Raw;
    Uint16 DrvStat2Raw;
    Uint16 DrvCtrl1Raw;
    Uint16 DrvCtrl2Raw;
    Uint16 DrvStat1Data;
    Uint16 DrvStat2Data;
    Uint16 DrvCtrl1Data;
    Uint16 DrvCtrl2Data;
    Uint16 DrvFrameFault;
    Uint16 DrvDeviceID;
    Uint16 DrvLastRxWord;

    AppState_e AppState;
    Uint32 AlignCounterMs;
    Uint32 VerifyCounterMs;
    Uint32 Timer0IsrCount;
    Uint32 AdcIsrCount;

    float DutyA;
    float DutyB;
    float DutyC;

    Uint16 PwmEnabled;
    Uint16 OverCurrentTrip;

    Uint16 StartCmd;
    Uint16 StartCmdPrev;
    int16_t DirectionCmd;

    Uint16 SpiServiceReq;
    Uint16 FilterInitDone;
    Uint16 TractionDriveEnable;
} MotorCtrl_t;

// ===================== Global =====================
static const MotorHw_t gM1Hw =
{
    SPI_MODULE_A,
    19U, 18U, 58U, 59U, 60U, 61U, 124U, 125U,
    1U, 20U, 21U, 99U, 1U, 0U, 1.0f, 4000U,
    &EPwm1Regs, &EPwm2Regs, &EPwm3Regs,
    0U
};

static const MotorHw_t gM2Hw =
{
    SPI_MODULE_B,
    139U, 56U, 63U, 64U, 65U, 66U, 26U, 27U,
    2U, 54U, 55U, 57U, 1U, 0U, 1.0f, 4000U,
    &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
    6U
};

volatile MotorCtrl_t gM1 = {0};
volatile MotorCtrl_t gM2 = {0};

volatile Uint16 gEdEnable = ED_ENABLE_DEFAULT;
volatile Uint16 gEdStartCmd = 0U;
volatile float  gVehicleSpeedCmd_mps = 0.0f;     // signed, + ileri
volatile float  gSteerAngleCmd_deg   = 0.0f;     // + sola dönüþ
volatile float  gSteerAngleCmd_rad   = 0.0f;
volatile float  gEdTurnRadius_m      = 0.0f;
volatile float  gEdYawRateRef_rad_s  = 0.0f;
volatile float  gEdRearLeftLinRef_mps  = 0.0f;
volatile float  gEdRearRightLinRef_mps = 0.0f;
volatile float  gEdRearLeftWheelRef_rpm  = 0.0f;
volatile float  gEdRearRightWheelRef_rpm = 0.0f;
volatile float  gEdRearLeftMotorRef_rpm  = 0.0f;
volatile float  gEdRearRightMotorRef_rpm = 0.0f;
volatile float  gEdMotor1Ref_rpm = 0.0f;
volatile float  gEdMotor2Ref_rpm = 0.0f;
volatile ElectronicDiffDebug_t gEdDbg = {0};

// ===================== Prototypes =====================
static inline float ClampF(float x, float xmin, float xmax);
static inline float WrapAngle_0_2pi(float x);
static inline float PI_Run(volatile PI_Controller_t *p, float ref, float fb, float Ts);
static inline float RampToTarget(float current, float target, float maxStep);

static void Motor_InitDefaults(volatile MotorCtrl_t *m);
static void GPIO_InitDrv8301Pins(const MotorHw_t *hw);
static void PWM_InitGpioOutputs(const MotorHw_t *hw);
static void EQEP_InitGpio(const MotorHw_t *hw);
static void SPI_InitDrv8301(const MotorHw_t *hw);

static void DRV8301_SetDCCal(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable);
static void DRV8301_SetGateEnable(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable);
static void DRV8301_ReadStatusPins(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static inline void DRV8301_CS_Low(const MotorHw_t *hw);
static inline void DRV8301_CS_High(const MotorHw_t *hw);
static Uint16 SPI_Transfer16(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 data);
static Uint16 DRV8301_ReadReg(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 addr);
static Uint16 DRV8301_WriteReg(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 addr, Uint16 data11);
static void DRV8301_ReadAllRegs(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void DRV8301_SetCurrentAmpGain(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 gainBits);

static void ADC_InitModules(void);
static void ADC_InitSOCs(void);

static void EQEP_InitModule(const MotorHw_t *hw);
static void EQEP_ResetEstimator(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void EQEP_ResetAlignOffsetAccumulator(volatile MotorCtrl_t *m);
static void EQEP_RunAlignOffsetAccumulator_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void EQEP_CaptureElecOffsetFromAlign(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void EQEP_UpdatePositionFromCount(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void EQEP_UpdateSpeed_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw);

static void PWM_InitInverters(void);
static void PWM_InitSingleEPwm(volatile struct EPWM_REGS *p);
static Uint16 PWM_DeadtimeCounts(float deadtime_ns);
static void PWM_EnableOutputs(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable);
static void PWM_UpdateDutyABC(volatile MotorCtrl_t *m, const MotorHw_t *hw, float dutyA, float dutyB, float dutyC);

static void CPU_TIMER0_Init1kHz(void);

static void StartCurrentOffsetCalibrationAll(void);
static void WaitCurrentOffsetCalibrationDoneAll(void);
static void CalibrateVbusDividersAll(float measuredBusV);

static inline void ConvertRawToPhysical(volatile MotorCtrl_t *m);
static inline void UpdateFiltersAndAlarms_ISR(volatile MotorCtrl_t *m);

static void FOC_ResetControllers(volatile MotorCtrl_t *m);
static void SVPWM_Apply(volatile MotorCtrl_t *m, const MotorHw_t *hw, float vAlpha, float vBeta, float vdc);
static void Align_ApplyVoltageVector(volatile MotorCtrl_t *m, const MotorHw_t *hw, float theta_e_align, float modIndex, float vdc);
static void FOC_RunCurrentLoop_ISR(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void App_RunStateMachine_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw);
static void App_ForceStop(volatile MotorCtrl_t *m, const MotorHw_t *hw);

static void ElectronicDiff_UpdateRefs_1kHz(void);

interrupt void adca1_isr(void);
interrupt void cpu_timer0_isr(void);

// ===================== main =====================
void main(void)
{
    InitSysCtrl();
    InitGpio();

    Motor_InitDefaults(&gM1);
    Motor_InitDefaults(&gM2);

    DINT;
    InitPieCtrl();

    IER = 0x0000U;
    IFR = 0x0000U;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT  = &adca1_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    GPIO_InitDrv8301Pins(&gM1Hw);
    GPIO_InitDrv8301Pins(&gM2Hw);

    PWM_InitGpioOutputs(&gM1Hw);
    PWM_InitGpioOutputs(&gM2Hw);

    EQEP_InitGpio(&gM1Hw);
    EQEP_InitGpio(&gM2Hw);

    SPI_InitDrv8301(&gM1Hw);
    SPI_InitDrv8301(&gM2Hw);

    DRV8301_SetGateEnable(&gM1, &gM1Hw, 0U);
    DRV8301_SetGateEnable(&gM2, &gM2Hw, 0U);
    DRV8301_SetDCCal(&gM1, &gM1Hw, 0U);
    DRV8301_SetDCCal(&gM2, &gM2Hw, 0U);

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    EDIS;

    ADC_InitModules();
    ADC_InitSOCs();
    EQEP_InitModule(&gM1Hw);
    EQEP_InitModule(&gM2Hw);
    PWM_InitInverters();
    CPU_TIMER0_Init1kHz();

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1U;   // ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1U;   // CPU Timer0
    IER |= M_INT1;

    EINT;
    ERTM;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;

    DRV8301_SetGateEnable(&gM1, &gM1Hw, 1U);
    DRV8301_SetGateEnable(&gM2, &gM2Hw, 1U);
    DELAY_US(5000);

    DRV8301_ReadStatusPins(&gM1, &gM1Hw);
    DRV8301_ReadStatusPins(&gM2, &gM2Hw);

    DRV8301_ReadAllRegs(&gM1, &gM1Hw);
    DRV8301_ReadAllRegs(&gM2, &gM2Hw);

    DRV8301_SetCurrentAmpGain(&gM1, &gM1Hw, DRV8301_DEFAULT_GAIN_BITS);
    DRV8301_SetCurrentAmpGain(&gM2, &gM2Hw, DRV8301_DEFAULT_GAIN_BITS);

    StartCurrentOffsetCalibrationAll();
    WaitCurrentOffsetCalibrationDoneAll();
    CalibrateVbusDividersAll(VBUS_CAL_MEASURED_V);

    EQEP_ResetEstimator(&gM1, &gM1Hw);
    EQEP_ResetEstimator(&gM2, &gM2Hw);
    EQEP_UpdatePositionFromCount(&gM1, &gM1Hw);
    EQEP_UpdatePositionFromCount(&gM2, &gM2Hw);

    App_ForceStop(&gM1, &gM1Hw);
    App_ForceStop(&gM2, &gM2Hw);

    PWM_UpdateDutyABC(&gM1, &gM1Hw, 0.5f, 0.5f, 0.5f);
    PWM_UpdateDutyABC(&gM2, &gM2Hw, 0.5f, 0.5f, 0.5f);
    PWM_EnableOutputs(&gM1, &gM1Hw, 0U);
    PWM_EnableOutputs(&gM2, &gM2Hw, 0U);

    for(;;)
    {
        if(gM1.SpiServiceReq)
        {
            gM1.SpiServiceReq = 0U;
            DRV8301_ReadAllRegs(&gM1, &gM1Hw);
        }

        if(gM2.SpiServiceReq)
        {
            gM2.SpiServiceReq = 0U;
            DRV8301_ReadAllRegs(&gM2, &gM2Hw);
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

static void Motor_InitDefaults(volatile MotorCtrl_t *m)
{
    m->IA_offset_counts = 2048.0f;
    m->IB_offset_counts = 2048.0f;
    m->CsaGain_VV = DRV8301_CSA_GAIN_DEFAULT_VV;
    m->VbusDividerGain = 1.0f;

    m->IdRefCmd_A = OPENLOOP_ID_A;
    m->IqRefCmd_A = 0.0f;
    m->IqRefTarget_A = 0.0f;

    m->SpeedRefRpmCmd = 50.0f;
    m->SpeedCmdRpmRamp = 0.0f;

    m->DirectionCmd = 1;
    m->AppState = APP_STATE_STOP;

    m->DutyA = 0.5f;
    m->DutyB = 0.5f;
    m->DutyC = 0.5f;

    m->IdPI.Kp = CURRENT_PI_KP;
    m->IdPI.Ki = CURRENT_PI_KI;
    m->IdPI.Ui = 0.0f;
    m->IdPI.OutMin = -100.0f;
    m->IdPI.OutMax =  100.0f;

    m->IqPI.Kp = CURRENT_PI_KP;
    m->IqPI.Ki = CURRENT_PI_KI;
    m->IqPI.Ui = 0.0f;
    m->IqPI.OutMin = -100.0f;
    m->IqPI.OutMax =  100.0f;

    m->SpeedPI.Kp = SPEED_PI_KP;
    m->SpeedPI.Ki = SPEED_PI_KI;
    m->SpeedPI.Ui = 0.0f;
    m->SpeedPI.OutMin = -SPEED_PI_OUT_MAX_A;
    m->SpeedPI.OutMax =  SPEED_PI_OUT_MAX_A;
}

// ===================== Init =====================
static void GPIO_InitDrv8301Pins(const MotorHw_t *hw)
{
    EALLOW;
    GPIO_SetupPinMux(hw->PinFault, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(hw->PinFault, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(hw->PinOctw, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(hw->PinOctw, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(hw->PinEnGate, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(hw->PinEnGate, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(hw->PinDcCal, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(hw->PinDcCal, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(hw->PinSpiCs, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(hw->PinSpiCs, GPIO_OUTPUT, GPIO_PUSHPULL);
    EDIS;

    GPIO_WritePin(hw->PinSpiCs, 1U);
}

static void PWM_InitGpioOutputs(const MotorHw_t *hw)
{
    Uint16 base = hw->PwmGpioBase;
    EALLOW;
    GpioCtrlRegs.GPAPUD.all |= ((Uint32)0x3FU << base);

    GpioCtrlRegs.GPAMUX1.all &= ~((Uint32)0xFFFU << (base * 2U));
    GpioCtrlRegs.GPAMUX1.all |=  ((Uint32)0x555U << (base * 2U));
    EDIS;
}

static void EQEP_InitGpio(const MotorHw_t *hw)
{
    if(hw->EqepModule == 1U)
    {
        GPIO_SetupPinMux(hw->PinEqepA, GPIO_MUX_CPU1, 1U);
        GPIO_SetupPinOptions(hw->PinEqepA, GPIO_INPUT, GPIO_PULLUP);

        GPIO_SetupPinMux(hw->PinEqepB, GPIO_MUX_CPU1, 1U);
        GPIO_SetupPinOptions(hw->PinEqepB, GPIO_INPUT, GPIO_PULLUP);

        if(hw->EncoderUseIndex)
        {
            GPIO_SetupPinMux(hw->PinEqepI, GPIO_MUX_CPU1, 2U);
            GPIO_SetupPinOptions(hw->PinEqepI, GPIO_INPUT, GPIO_PULLUP);
        }
    }
    else
    {
        GPIO_SetupPinMux(hw->PinEqepA, GPIO_MUX_CPU1, 5U);
        GPIO_SetupPinOptions(hw->PinEqepA, GPIO_INPUT, GPIO_PULLUP);

        GPIO_SetupPinMux(hw->PinEqepB, GPIO_MUX_CPU1, 5U);
        GPIO_SetupPinOptions(hw->PinEqepB, GPIO_INPUT, GPIO_PULLUP);

        if(hw->EncoderUseIndex)
        {
            GPIO_SetupPinMux(hw->PinEqepI, GPIO_MUX_CPU1, 5U);
            GPIO_SetupPinOptions(hw->PinEqepI, GPIO_INPUT, GPIO_PULLUP);
        }
    }
}

static void SPI_InitDrv8301(const MotorHw_t *hw)
{
    EALLOW;
    if(hw->SpiModule == SPI_MODULE_A)
    {
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
    else
    {
        CpuSysRegs.PCLKCR8.bit.SPI_B = 1U;

        GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;
        GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;
        GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;
        GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;

        GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
        GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;
        GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;
        GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;

        GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
        GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;
        GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;
        GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;

        GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;
        GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3;

        EDIS;

        SpibRegs.SPICCR.bit.SPISWRESET  = 0U;
        SpibRegs.SPICCR.bit.SPICHAR     = 15U;
        SpibRegs.SPICCR.bit.CLKPOLARITY = 0U;
        SpibRegs.SPICCR.bit.SPILBK      = 0U;

        SpibRegs.SPICTL.bit.MASTER_SLAVE = 1U;
        SpibRegs.SPICTL.bit.TALK         = 1U;
        SpibRegs.SPICTL.bit.CLK_PHASE    = 0U;
        SpibRegs.SPICTL.bit.SPIINTENA    = 0U;

        SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49U;
        SpibRegs.SPIPRI.bit.FREE = 1U;
        SpibRegs.SPIPRI.bit.SOFT = 1U;
        SpibRegs.SPICCR.bit.SPISWRESET = 1U;
    }
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

    // Motor-1
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = M1_ADCC_CH_IA;
    AdccRegs.ADCSOC0CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = M1_ADCB_CH_IB;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = M1_ADCA_CH_VBUS;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    // Motor-2
    AdccRegs.ADCSOC1CTL.bit.CHSEL   = M2_ADCC_CH_IA;
    AdccRegs.ADCSOC1CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL   = M2_ADCB_CH_IB;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL   = M2_ADCA_CH_VBUS;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS   = ADC_ACQPS_12BIT;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGSEL_EPWM1_SOCA;

    // ADCA INT1, motor-2 VBUS tamamlandýðýnda oluþur; bu anda tüm sonuçlar hazýrdýr.
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL  = 2U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E    = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1  = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1  = 1U;
    EDIS;
}

static void EQEP_InitModule(const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q;

    EALLOW;
    if(hw->EqepModule == 1U)
    {
        CpuSysRegs.PCLKCR4.bit.EQEP1 = 1U;
        q = &EQep1Regs;
    }
    else
    {
        CpuSysRegs.PCLKCR4.bit.EQEP2 = 1U;
        q = &EQep2Regs;
    }
    EDIS;

    q->QDECCTL.all = 0U;
    q->QEPCTL.all  = 0U;
    q->QCAPCTL.all = 0U;

    q->QDECCTL.bit.QSRC = 0U;
    q->QDECCTL.bit.SWAP = hw->EncoderSwapAB ? 1U : 0U;

    q->QPOSINIT = 0U;
    q->QPOSMAX  = (Uint32)(hw->EncoderCountsPerRev - 1U);
    q->QPOSCNT  = 0U;

    q->QEPCTL.bit.FREE_SOFT = 2U;
    q->QEPCTL.bit.PCRM      = 1U;
    q->QEPCTL.bit.IEI       = 0U;
    q->QEPCTL.bit.SWI       = 1U;
    q->QEPCTL.bit.QPEN      = 1U;

    q->QCLR.all = 0xFFFFU;
}

static void EQEP_ResetEstimator(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q = (hw->EqepModule == 1U) ? &EQep1Regs : &EQep2Regs;

    m->QepCount = (int32_t)q->QPOSCNT;
    m->QepPrevCount = m->QepCount;
    m->QepDeltaCount = 0;
    m->QepIndexSeen = 0U;

    m->SpeedMechFbRpm = 0.0f;
    m->SpeedMechFbRpmFilt = 0.0f;
    m->OmegaMechFbRad_s = 0.0f;
    m->OmegaElecFbRad_s = 0.0f;

    EQEP_UpdatePositionFromCount(m, hw);
}

static void EQEP_ResetAlignOffsetAccumulator(volatile MotorCtrl_t *m)
{
    m->AlignQepCountSum = 0.0f;
    m->AlignQepSampleCount = 0U;
}

static void EQEP_RunAlignOffsetAccumulator_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q = (hw->EqepModule == 1U) ? &EQep1Regs : &EQep2Regs;

    if(m->AlignCounterMs >= (ALIGN_TIME_MS - ALIGN_OFFSET_SAMPLE_MS))
    {
        m->AlignQepCountSum += (float)((Uint32)q->QPOSCNT);
        m->AlignQepSampleCount++;
    }
}

static void EQEP_UpdatePositionFromCount(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q = (hw->EqepModule == 1U) ? &EQep1Regs : &EQep2Regs;
    float thetaMech;

    m->QepCount = (int32_t)q->QPOSCNT;
    thetaMech = ((float)m->QepCount * TWO_PI_F) / (float)hw->EncoderCountsPerRev;

    m->ThetaMechEncRad = WrapAngle_0_2pi(thetaMech);
    m->ThetaElecEncRad = WrapAngle_0_2pi((hw->EncoderMechSign * MOTOR_POLE_PAIRS * m->ThetaMechEncRad) +
                                         m->ThetaElecOffsetRad +
                                         ENCODER_ELEC_ZERO_BIAS_RAD);

    if(hw->EncoderUseIndex)
    {
        if(q->QFLG.bit.IEL == 1U)
        {
            m->QepIndexSeen = 1U;
            q->QCLR.bit.IEL = 1U;
        }
    }
}

static void EQEP_CaptureElecOffsetFromAlign(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q = (hw->EqepModule == 1U) ? &EQep1Regs : &EQep2Regs;
    float avgCount;
    float thetaMechAvg;

    if(m->AlignQepSampleCount == 0U)
    {
        avgCount = (float)((Uint32)q->QPOSCNT);
    }
    else
    {
        avgCount = m->AlignQepCountSum / (float)m->AlignQepSampleCount;
    }

    thetaMechAvg = (avgCount * TWO_PI_F) / (float)hw->EncoderCountsPerRev;
    thetaMechAvg = WrapAngle_0_2pi(thetaMechAvg);

    m->ThetaElecOffsetRad = WrapAngle_0_2pi(ALIGN_ELEC_ANGLE_RAD -
                                            (hw->EncoderMechSign * MOTOR_POLE_PAIRS * thetaMechAvg));

    EQEP_UpdatePositionFromCount(m, hw);
}

static void EQEP_UpdateSpeed_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    volatile struct EQEP_REGS *q = (hw->EqepModule == 1U) ? &EQep1Regs : &EQep2Regs;
    int32_t posNow, delta, halfCounts;
    float speedInstRpm;

    posNow = (int32_t)q->QPOSCNT;
    delta = posNow - m->QepPrevCount;
    halfCounts = (int32_t)(hw->EncoderCountsPerRev / 2U);

    if(delta > halfCounts)
    {
        delta -= (int32_t)hw->EncoderCountsPerRev;
    }
    else if(delta < -halfCounts)
    {
        delta += (int32_t)hw->EncoderCountsPerRev;
    }

    m->QepPrevCount = posNow;
    m->QepCount = posNow;
    m->QepDeltaCount = delta;

    speedInstRpm = ((((float)delta) * 60.0f) /
                    ((float)hw->EncoderCountsPerRev * TIMER0_TS)) * hw->EncoderMechSign;

    m->SpeedMechFbRpm = speedInstRpm;
    m->SpeedMechFbRpmFilt += SPEED_FILT_ALPHA * (m->SpeedMechFbRpm - m->SpeedMechFbRpmFilt);

    if(fabsf(m->SpeedMechFbRpmFilt) < 1.0f)
    {
        m->SpeedMechFbRpmFilt = 0.0f;
    }

    m->OmegaMechFbRad_s = m->SpeedMechFbRpmFilt * (TWO_PI_F / 60.0f);
    m->OmegaElecFbRad_s = m->OmegaMechFbRad_s * MOTOR_POLE_PAIRS;

    EQEP_UpdatePositionFromCount(m, hw);
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

static void PWM_InitInverters(void)
{
    EALLOW;
    PWM_InitSingleEPwm(&EPwm1Regs);
    PWM_InitSingleEPwm(&EPwm2Regs);
    PWM_InitSingleEPwm(&EPwm3Regs);
    PWM_InitSingleEPwm(&EPwm4Regs);
    PWM_InitSingleEPwm(&EPwm5Regs);
    PWM_InitSingleEPwm(&EPwm6Regs);

    // Tek ADC tetikleyici: ePWM1 SOCA
    EPwm1Regs.CMPB.bit.CMPB      = (Uint16)(TBPRD_VAL - 80U);
    EPwm1Regs.ETSEL.bit.SOCAEN   = 1U;
    EPwm1Regs.ETSEL.bit.SOCASEL  = 6U;
    EPwm1Regs.ETPS.bit.SOCAPRD   = 1U;
    EPwm1Regs.ETCLR.bit.SOCA     = 1U;
    EDIS;
}

static void PWM_EnableOutputs(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable)
{
    EALLOW;
    if(enable)
    {
        hw->PwmA->TZCLR.bit.OST = 1U;
        hw->PwmB->TZCLR.bit.OST = 1U;
        hw->PwmC->TZCLR.bit.OST = 1U;
        m->PwmEnabled = 1U;
    }
    else
    {
        hw->PwmA->TZFRC.bit.OST = 1U;
        hw->PwmB->TZFRC.bit.OST = 1U;
        hw->PwmC->TZFRC.bit.OST = 1U;
        m->PwmEnabled = 0U;
    }
    EDIS;
}

static void PWM_UpdateDutyABC(volatile MotorCtrl_t *m, const MotorHw_t *hw, float dutyA, float dutyB, float dutyC)
{
    Uint16 cmpA, cmpB, cmpC;

    dutyA = ClampF(dutyA, DUTY_MIN, DUTY_MAX);
    dutyB = ClampF(dutyB, DUTY_MIN, DUTY_MAX);
    dutyC = ClampF(dutyC, DUTY_MIN, DUTY_MAX);

    cmpA = (Uint16)(dutyA * (float)TBPRD_VAL);
    cmpB = (Uint16)(dutyB * (float)TBPRD_VAL);
    cmpC = (Uint16)(dutyC * (float)TBPRD_VAL);

    hw->PwmA->CMPA.bit.CMPA = cmpA;
    hw->PwmB->CMPA.bit.CMPA = cmpB;
    hw->PwmC->CMPA.bit.CMPA = cmpC;

    m->DutyA = dutyA;
    m->DutyB = dutyB;
    m->DutyC = dutyC;
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
static void DRV8301_SetDCCal(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable)
{
    GPIO_WritePin(hw->PinDcCal, enable ? 1U : 0U);
    m->DcCalPinState = enable ? 1U : 0U;
}

static void DRV8301_SetGateEnable(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 enable)
{
    GPIO_WritePin(hw->PinEnGate, enable ? 1U : 0U);
    m->GateEnableState = enable ? 1U : 0U;
}

static void DRV8301_ReadStatusPins(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    m->FaultState = GPIO_ReadPin(hw->PinFault);
    m->OctwState  = GPIO_ReadPin(hw->PinOctw);

    m->FaultActive = (m->FaultState == 0U) ? 1U : 0U;
    m->OctwActive  = (m->OctwState  == 0U) ? 1U : 0U;
}

static inline void DRV8301_CS_Low(const MotorHw_t *hw)
{
    GPIO_WritePin(hw->PinSpiCs, 0U);
}

static inline void DRV8301_CS_High(const MotorHw_t *hw)
{
    GPIO_WritePin(hw->PinSpiCs, 1U);
}

static Uint16 SPI_Transfer16(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 data)
{
    if(hw->SpiModule == SPI_MODULE_A)
    {
        while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1U) {}
        SpiaRegs.SPITXBUF = data;
        while(SpiaRegs.SPISTS.bit.INT_FLAG == 0U) {}
        m->DrvLastRxWord = SpiaRegs.SPIRXBUF;
    }
    else
    {
        while(SpibRegs.SPISTS.bit.BUFFULL_FLAG == 1U) {}
        SpibRegs.SPITXBUF = data;
        while(SpibRegs.SPISTS.bit.INT_FLAG == 0U) {}
        m->DrvLastRxWord = SpibRegs.SPIRXBUF;
    }

    return m->DrvLastRxWord;
}

static Uint16 DRV8301_ReadReg(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 addr)
{
    Uint16 cmd, resp;
    cmd = 0x8000U | ((addr & 0x000FU) << 11);

    DRV8301_CS_Low(hw);
    SPI_Transfer16(m, hw, cmd);
    DRV8301_CS_High(hw);

    asm(" RPT #20 || NOP");

    DRV8301_CS_Low(hw);
    resp = SPI_Transfer16(m, hw, 0x0000U);
    DRV8301_CS_High(hw);

    return resp;
}

static Uint16 DRV8301_WriteReg(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 addr, Uint16 data11)
{
    Uint16 cmd, resp;
    cmd = ((addr & 0x000FU) << 11) | (data11 & 0x07FFU);

    DRV8301_CS_Low(hw);
    SPI_Transfer16(m, hw, cmd);
    DRV8301_CS_High(hw);

    asm(" RPT #20 || NOP");

    DRV8301_CS_Low(hw);
    resp = SPI_Transfer16(m, hw, 0x0000U);
    DRV8301_CS_High(hw);

    return resp;
}

static void DRV8301_ReadAllRegs(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    m->DrvStat1Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_STATUS1);
    m->DrvStat1Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_STATUS1);

    m->DrvStat2Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_STATUS2);
    m->DrvStat2Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_STATUS2);

    m->DrvCtrl1Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_CONTROL1);
    m->DrvCtrl1Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_CONTROL1);

    m->DrvCtrl2Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_CONTROL2);
    m->DrvCtrl2Raw = DRV8301_ReadReg(m, hw, DRV8301_REG_CONTROL2);

    m->DrvStat1Data = m->DrvStat1Raw & 0x07FFU;
    m->DrvStat2Data = m->DrvStat2Raw & 0x07FFU;
    m->DrvCtrl1Data = m->DrvCtrl1Raw & 0x07FFU;
    m->DrvCtrl2Data = m->DrvCtrl2Raw & 0x07FFU;

    m->DrvFrameFault = (m->DrvStat1Raw >> 15) & 0x1U;
    m->DrvDeviceID   = (m->DrvStat2Data & 0x000FU);
}

static void DRV8301_SetCurrentAmpGain(volatile MotorCtrl_t *m, const MotorHw_t *hw, Uint16 gainBits)
{
    Uint16 ctrl2 = m->DrvCtrl2Data;

    ctrl2 &= ~(0x000CU | 0x0003U);
    ctrl2 |= DRV8301_CTRL2_OCTW_BOTH;
    ctrl2 |= gainBits;

    DRV8301_WriteReg(m, hw, DRV8301_REG_CONTROL2, ctrl2);
    DELAY_US(50);
    DRV8301_ReadAllRegs(m, hw);

    switch(gainBits)
    {
        case DRV8301_CTRL2_GAIN_10VV: m->CsaGain_VV = 10.0f; break;
        case DRV8301_CTRL2_GAIN_20VV: m->CsaGain_VV = 20.0f; break;
        case DRV8301_CTRL2_GAIN_40VV: m->CsaGain_VV = 40.0f; break;
        case DRV8301_CTRL2_GAIN_80VV: m->CsaGain_VV = 80.0f; break;
        default: m->CsaGain_VV = 20.0f; break;
    }
}

// ===================== Calibration =====================
static void StartCurrentOffsetCalibrationAll(void)
{
    gM1.IA_sum = 0U;
    gM1.IB_sum = 0U;
    gM1.CalCount = 0U;
    gM1.CalDone = 0U;
    gM1.CalActive = 1U;
    gM1.FilterInitDone = 0U;

    gM2.IA_sum = 0U;
    gM2.IB_sum = 0U;
    gM2.CalCount = 0U;
    gM2.CalDone = 0U;
    gM2.CalActive = 1U;
    gM2.FilterInitDone = 0U;

    PWM_EnableOutputs(&gM1, &gM1Hw, 0U);
    PWM_EnableOutputs(&gM2, &gM2Hw, 0U);

    DRV8301_SetGateEnable(&gM1, &gM1Hw, 1U);
    DRV8301_SetGateEnable(&gM2, &gM2Hw, 1U);

    DRV8301_SetDCCal(&gM1, &gM1Hw, 1U);
    DRV8301_SetDCCal(&gM2, &gM2Hw, 1U);

    DELAY_US(2000);
}

static void WaitCurrentOffsetCalibrationDoneAll(void)
{
    while((gM1.CalDone == 0U) || (gM2.CalDone == 0U)) {}

    gM1.IA_offset_counts = ((float)gM1.IA_sum) / (float)CURR_OFFSET_SAMPLES;
    gM1.IB_offset_counts = ((float)gM1.IB_sum) / (float)CURR_OFFSET_SAMPLES;

    gM2.IA_offset_counts = ((float)gM2.IA_sum) / (float)CURR_OFFSET_SAMPLES;
    gM2.IB_offset_counts = ((float)gM2.IB_sum) / (float)CURR_OFFSET_SAMPLES;

    DRV8301_SetDCCal(&gM1, &gM1Hw, 0U);
    DRV8301_SetDCCal(&gM2, &gM2Hw, 0U);

    DELAY_US(1000);
    DRV8301_ReadAllRegs(&gM1, &gM1Hw);
    DRV8301_ReadAllRegs(&gM2, &gM2Hw);
}

static void CalibrateVbusDividersAll(float measuredBusV)
{
    float adcPinV;

    adcPinV = ((float)gM1.VBS_raw * ADC_VREF) / ADC_MAX_COUNT;
    if((measuredBusV > 0.1f) && (adcPinV > 0.01f))
    {
        gM1.VbusDividerGain = measuredBusV / adcPinV;
        gM1.VbusSatBus_V = ADC_VREF * gM1.VbusDividerGain;
    }

    adcPinV = ((float)gM2.VBS_raw * ADC_VREF) / ADC_MAX_COUNT;
    if((measuredBusV > 0.1f) && (adcPinV > 0.01f))
    {
        gM2.VbusDividerGain = measuredBusV / adcPinV;
        gM2.VbusSatBus_V = ADC_VREF * gM2.VbusDividerGain;
    }
}

// ===================== ISR içi ölçüm / kontrol =====================
static inline void ConvertRawToPhysical(volatile MotorCtrl_t *m)
{
    m->IA_adc_V  = ((float)m->IA_raw  * ADC_VREF) / ADC_MAX_COUNT;
    m->IB_adc_V  = ((float)m->IB_raw  * ADC_VREF) / ADC_MAX_COUNT;
    m->VBS_adc_V = ((float)m->VBS_raw * ADC_VREF) / ADC_MAX_COUNT;

    m->IA_corr_counts = (int32_t)m->IA_raw - (int32_t)(m->IA_offset_counts + 0.5f);
    m->IB_corr_counts = (int32_t)m->IB_raw - (int32_t)(m->IB_offset_counts + 0.5f);

    m->IA_corr_V = ((float)m->IA_corr_counts * ADC_VREF) / ADC_MAX_COUNT;
    m->IB_corr_V = ((float)m->IB_corr_counts * ADC_VREF) / ADC_MAX_COUNT;

    m->IA_A = IA_SIGN * (m->IA_corr_V / (m->CsaGain_VV * CURRENT_SHUNT_OHM));
    m->IB_A = IB_SIGN * (m->IB_corr_V / (m->CsaGain_VV * CURRENT_SHUNT_OHM));
    m->IC_A = -(m->IA_A + m->IB_A);

    m->VBS_bus_V = (((float)m->VBS_raw * ADC_VREF) / ADC_MAX_COUNT) * m->VbusDividerGain;
    m->VbusSatBus_V = ADC_VREF * m->VbusDividerGain;
}

static inline void UpdateFiltersAndAlarms_ISR(volatile MotorCtrl_t *m)
{
    if(m->FilterInitDone == 0U)
    {
        m->IA_A_filt = m->IA_A;
        m->IB_A_filt = m->IB_A;
        m->VBS_bus_V_filt = m->VBS_bus_V;
        m->FilterInitDone = 1U;
    }
    else
    {
        m->IA_A_filt += I_FILT_ALPHA * (m->IA_A - m->IA_A_filt);
        m->IB_A_filt += I_FILT_ALPHA * (m->IB_A - m->IB_A_filt);
        m->VBS_bus_V_filt += VBUS_FILT_ALPHA * (m->VBS_bus_V - m->VBS_bus_V_filt);
    }

    m->VbusNearClip = (m->VBS_raw >= VBUS_NEAR_CLIP_RAW) ? 1U : 0U;
    m->VbusSaturated = (m->VBS_raw >= VBUS_SAT_RAW) ? 1U : 0U;
}

static void SVPWM_Apply(volatile MotorCtrl_t *m, const MotorHw_t *hw, float vAlpha, float vBeta, float vdc)
{
    float vRef;
    float angle;
    float theta_s;
    float k;
    float T1;
    float T2;
    float T0;
    float halfT0;
    float dutyA, dutyB, dutyC;
    Uint16 sector;

    (void)m;

    if(vdc < 1.0f)
    {
        PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
        return;
    }

    vRef = sqrtf((vAlpha * vAlpha) + (vBeta * vBeta));
    if(vRef < 1.0e-6f)
    {
        PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
        return;
    }

    angle = atan2f(vBeta, vAlpha);
    if(angle < 0.0f)
    {
        angle += TWO_PI_F;
    }
    else if(angle >= TWO_PI_F)
    {
        angle -= TWO_PI_F;
    }

    sector = (Uint16)(angle / (PI_F / 3.0f)) + 1U;
    if(sector > 6U)
    {
        sector = 6U;
    }

    theta_s = angle - ((float)(sector - 1U) * (PI_F / 3.0f));

    k  = (SQRT3_F * vRef) / vdc;
    T1 = k * sinf((PI_F / 3.0f) - theta_s);
    T2 = k * sinf(theta_s);

    if(T1 < 0.0f) T1 = 0.0f;
    if(T2 < 0.0f) T2 = 0.0f;

    if((T1 + T2) > 1.0f)
    {
        float scale = 1.0f / (T1 + T2);
        T1 *= scale;
        T2 *= scale;
    }

    T0 = 1.0f - T1 - T2;
    if(T0 < 0.0f)
    {
        T0 = 0.0f;
    }
    halfT0 = 0.5f * T0;

    switch(sector)
    {
        case 1U:
            dutyA = T1 + T2 + halfT0;
            dutyB = T2 + halfT0;
            dutyC = halfT0;
            break;

        case 2U:
            dutyA = T1 + halfT0;
            dutyB = T1 + T2 + halfT0;
            dutyC = halfT0;
            break;

        case 3U:
            dutyA = halfT0;
            dutyB = T1 + T2 + halfT0;
            dutyC = T2 + halfT0;
            break;

        case 4U:
            dutyA = halfT0;
            dutyB = T1 + halfT0;
            dutyC = T1 + T2 + halfT0;
            break;

        case 5U:
            dutyA = T2 + halfT0;
            dutyB = halfT0;
            dutyC = T1 + T2 + halfT0;
            break;

        case 6U:
        default:
            dutyA = T1 + T2 + halfT0;
            dutyB = halfT0;
            dutyC = T1 + halfT0;
            break;
    }

    PWM_UpdateDutyABC(m, hw, dutyA, dutyB, dutyC);
}

static void Align_ApplyVoltageVector(volatile MotorCtrl_t *m, const MotorHw_t *hw, float theta_e_align, float modIndex, float vdc)
{
    float vLimit = 0.577f * vdc * FOC_VOLT_LIMIT_FACTOR;
    float vMag   = modIndex * vLimit;
    float vAlpha = vMag * cosf(theta_e_align);
    float vBeta  = vMag * sinf(theta_e_align);

    SVPWM_Apply(m, hw, vAlpha, vBeta, vdc);
}

static void FOC_ResetControllers(volatile MotorCtrl_t *m)
{
    m->IdPI.Ui = 0.0f;
    m->IqPI.Ui = 0.0f;
    m->SpeedPI.Ui = 0.0f;

    m->Ialpha_A = 0.0f;
    m->Ibeta_A  = 0.0f;
    m->Id_A     = 0.0f;
    m->Iq_A     = 0.0f;

    m->IdRefCmd_A = OPENLOOP_ID_A;
    m->IqRefCmd_A = 0.0f;
    m->IqRefTarget_A = 0.0f;

    m->VdCmd_V = 0.0f;
    m->VqCmd_V = 0.0f;
    m->ValphaCmd_V = 0.0f;
    m->VbetaCmd_V = 0.0f;
}

static void FOC_RunCurrentLoop_ISR(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    float c, s;
    float iAlpha, iBeta;
    float id, iq;
    float vd_pi, vq_pi;
    float vd, vq;
    float vAlpha, vBeta;
    float vdc, vLimit, vNorm, scale;
    float omega_e;

    (void)hw;

    vdc = m->VBS_bus_V;
    if(vdc < FOC_MIN_RUN_VBUS_V)
    {
        PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
        return;
    }

    c = cosf(m->ThetaCtrlRad);
    s = sinf(m->ThetaCtrlRad);

    iAlpha = m->IA_A;
    iBeta  = (m->IA_A + 2.0f * m->IB_A) * INV_SQRT3_F;

    id =  c * iAlpha + s * iBeta;
    iq = -s * iAlpha + c * iBeta;

    m->Ialpha_A = iAlpha;
    m->Ibeta_A  = iBeta;
    m->Id_A     = id;
    m->Iq_A     = iq;

    vLimit = 0.577f * vdc * FOC_VOLT_LIMIT_FACTOR;
    m->VdqLimit_V = vLimit;

    m->IdPI.OutMin = -vLimit;
    m->IdPI.OutMax =  vLimit;
    m->IqPI.OutMin = -vLimit;
    m->IqPI.OutMax =  vLimit;

    vd_pi = PI_Run(&m->IdPI, m->IdRefCmd_A, id, CTRL_TS);
    vq_pi = PI_Run(&m->IqPI, m->IqRefCmd_A, iq, CTRL_TS);

#if USE_DECOUPLING_FF
    omega_e = m->OmegaElecFbRad_s;
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

    m->VdCmd_V = vd;
    m->VqCmd_V = vq;

    vAlpha = c * vd - s * vq;
    vBeta  = s * vd + c * vq;

    m->ValphaCmd_V = vAlpha;
    m->VbetaCmd_V  = vBeta;

    SVPWM_Apply(m, hw, vAlpha, vBeta, vdc);
}

static void App_ForceStop(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    m->AppState = APP_STATE_STOP;
    m->AlignCounterMs = 0U;
    m->VerifyCounterMs = 0U;

    m->SpeedCmdRpmRamp = 0.0f;
    m->OmegaMechCmdRad_s = 0.0f;
    m->OmegaElecCmdRad_s = 0.0f;

    m->ThetaOpenloopRad = ALIGN_ELEC_ANGLE_RAD;
    m->ThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
    m->ThetaElecOffsetRad = 0.0f;
    m->TractionDriveEnable = 0U;
    m->OverCurrentTrip = 0U;

    EQEP_ResetAlignOffsetAccumulator(m);
    EQEP_ResetEstimator(m, hw);
    FOC_ResetControllers(m);
    PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
    PWM_EnableOutputs(m, hw, 0U);
}

static void App_RunStateMachine_1kHz(volatile MotorCtrl_t *m, const MotorHw_t *hw)
{
    float signRef;
    float speedTargetRpm;
    float speedRampStep;
    float iqRampStep;

    EQEP_UpdateSpeed_1kHz(m, hw);
    m->Timer0IsrCount++;

    if(m->AppState == APP_STATE_FAULT)
    {
        PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
        PWM_EnableOutputs(m, hw, 0U);
        FOC_ResetControllers(m);
        m->StartCmdPrev = m->StartCmd;
        return;
    }

    if((m->StartCmd != 0U) && (m->StartCmdPrev == 0U))
    {
        EQEP_ResetAlignOffsetAccumulator(m);
        EQEP_ResetEstimator(m, hw);
        FOC_ResetControllers(m);
        m->OverCurrentTrip = 0U;
        m->AlignCounterMs = 0U;
        m->VerifyCounterMs = 0U;
        m->ThetaOpenloopRad = ALIGN_ELEC_ANGLE_RAD;
        m->ThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
        m->SpeedCmdRpmRamp = 0.0f;
        m->TractionDriveEnable = 0U;
        m->OmegaMechCmdRad_s = 0.0f;
        m->OmegaElecCmdRad_s = 0.0f;
        PWM_UpdateDutyABC(m, hw, 0.5f, 0.5f, 0.5f);
        PWM_EnableOutputs(m, hw, 1U);
        m->AppState = APP_STATE_ALIGN;
    }

    if(m->StartCmd == 0U)
    {
        if(m->AppState != APP_STATE_STOP)
        {
            App_ForceStop(m, hw);
        }
        m->StartCmdPrev = m->StartCmd;
        return;
    }

    signRef = (m->DirectionCmd >= 0) ? 1.0f : -1.0f;
    speedTargetRpm = signRef * fabsf(m->SpeedRefRpmCmd);
    speedRampStep = CLOSEDLOOP_SPEED_RAMP_RPM_PER_S * TIMER0_TS;
    iqRampStep = IQ_REF_SLEW_A_PER_S * TIMER0_TS;

    switch(m->AppState)
    {
        case APP_STATE_STOP:
            break;

        case APP_STATE_ALIGN:
            m->AlignCounterMs++;
            EQEP_RunAlignOffsetAccumulator_1kHz(m, hw);

            m->SpeedCmdRpmRamp = 0.0f;
            m->OmegaMechCmdRad_s = 0.0f;
            m->OmegaElecCmdRad_s = 0.0f;
            m->IdRefCmd_A = OPENLOOP_ID_A;
            m->IqRefTarget_A = 0.0f;
            m->IqRefCmd_A = RampToTarget(m->IqRefCmd_A, m->IqRefTarget_A, iqRampStep);

            if(m->AlignCounterMs >= ALIGN_TIME_MS)
            {
                EQEP_CaptureElecOffsetFromAlign(m, hw);
                EQEP_ResetEstimator(m, hw);

                m->ThetaCtrlRad = m->ThetaElecEncRad;
                m->SpeedCmdRpmRamp = 0.0f;
                m->OmegaMechCmdRad_s = 0.0f;
                m->OmegaElecCmdRad_s = 0.0f;
                m->IdRefCmd_A = OPENLOOP_ID_A;
                m->IqRefCmd_A = 0.0f;
                m->IqRefTarget_A = 0.0f;
                m->SpeedPI.Ui = 0.0f;
                m->TractionDriveEnable = 0U;

                if(VERIFY_TIME_MS > 0U)
                {
                    m->AppState = APP_STATE_VERIFY;
                }
                else
                {
                    m->AppState = APP_STATE_CLOSEDLOOP;
                }
            }
            break;

        case APP_STATE_VERIFY:
            m->VerifyCounterMs++;
            m->SpeedCmdRpmRamp = 0.0f;
            m->OmegaMechCmdRad_s = 0.0f;
            m->OmegaElecCmdRad_s = 0.0f;
            m->IdRefCmd_A = OPENLOOP_ID_A;
            m->IqRefTarget_A = signRef * VERIFY_IQ_A;
            m->IqRefCmd_A = RampToTarget(m->IqRefCmd_A, m->IqRefTarget_A, iqRampStep);

            if(m->VerifyCounterMs >= VERIFY_TIME_MS)
            {
                m->SpeedCmdRpmRamp = 0.0f;
                m->OmegaMechCmdRad_s = 0.0f;
                m->OmegaElecCmdRad_s = 0.0f;
                m->IdRefCmd_A = OPENLOOP_ID_A;
                m->IqRefCmd_A = 0.0f;
                m->IqRefTarget_A = 0.0f;
                m->SpeedPI.Ui = 0.0f;
                m->TractionDriveEnable = 0U;
                m->AppState = APP_STATE_CLOSEDLOOP;
            }
            break;

        case APP_STATE_CLOSEDLOOP:
        {
            float speedErr;
            float iqCmd;

            m->SpeedCmdRpmRamp = RampToTarget(m->SpeedCmdRpmRamp, speedTargetRpm, speedRampStep);
            m->OmegaMechCmdRad_s = m->SpeedCmdRpmRamp * (TWO_PI_F / 60.0f);
            m->OmegaElecCmdRad_s = m->OmegaMechCmdRad_s * MOTOR_POLE_PAIRS;
            m->IdRefCmd_A = OPENLOOP_ID_A;

            if(fabsf(m->SpeedCmdRpmRamp) < 1.0f)
            {
                m->TractionDriveEnable = 0U;
                m->SpeedPI.Ui = 0.0f;
                m->IqRefTarget_A = 0.0f;
            }
            else if(m->DirectionCmd >= 0)
            {
                speedErr = m->SpeedCmdRpmRamp - m->SpeedMechFbRpmFilt;

                if(speedErr >= TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    m->TractionDriveEnable = 1U;
                }
                else if(speedErr <= -TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    m->TractionDriveEnable = 0U;
                    m->SpeedPI.Ui = 0.0f;
                }

                if(m->TractionDriveEnable)
                {
                    iqCmd = PI_Run(&m->SpeedPI, m->SpeedCmdRpmRamp, m->SpeedMechFbRpmFilt, TIMER0_TS);
                    iqCmd = ClampF(iqCmd, 0.0f, SPEED_PI_OUT_MAX_A);

                    if((m->SpeedCmdRpmRamp > 20.0f) && (speedErr > TRACTION_SPEED_EXIT_BAND_RPM) &&
                       (iqCmd < TRACTION_MIN_RUN_IQ_A))
                    {
                        iqCmd = TRACTION_MIN_RUN_IQ_A;
                    }

                    m->IqRefTarget_A = iqCmd;
                }
                else
                {
                    m->IqRefTarget_A = 0.0f;
                }
            }
            else
            {
                speedErr = m->SpeedCmdRpmRamp - m->SpeedMechFbRpmFilt;

                if(speedErr <= -TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    m->TractionDriveEnable = 1U;
                }
                else if(speedErr >= TRACTION_SPEED_ENTER_BAND_RPM)
                {
                    m->TractionDriveEnable = 0U;
                    m->SpeedPI.Ui = 0.0f;
                }

                if(m->TractionDriveEnable)
                {
                    iqCmd = PI_Run(&m->SpeedPI, m->SpeedCmdRpmRamp, m->SpeedMechFbRpmFilt, TIMER0_TS);
                    iqCmd = ClampF(iqCmd, -SPEED_PI_OUT_MAX_A, 0.0f);

                    if((m->SpeedCmdRpmRamp < -20.0f) && (speedErr < -TRACTION_SPEED_EXIT_BAND_RPM) &&
                       (iqCmd > -TRACTION_MIN_RUN_IQ_A))
                    {
                        iqCmd = -TRACTION_MIN_RUN_IQ_A;
                    }

                    m->IqRefTarget_A = iqCmd;
                }
                else
                {
                    m->IqRefTarget_A = 0.0f;
                }
            }

            m->IqRefCmd_A = RampToTarget(m->IqRefCmd_A, m->IqRefTarget_A, iqRampStep);
            break;
        }

        case APP_STATE_FAULT:
        default:
            break;
    }

    m->StartCmdPrev = m->StartCmd;
}


static void ElectronicDiff_UpdateRefs_1kHz(void)
{
    if(gEdEnable == 0U)
    {
        return;
    }

    gEdDbg.vCmd_mps = ClampF(gVehicleSpeedCmd_mps, -ED_VEHICLE_MAX_SPEED_MPS, ED_VEHICLE_MAX_SPEED_MPS);
    gEdDbg.deltaDeg = ClampF(gSteerAngleCmd_deg, -ED_STEER_LIMIT_DEG, ED_STEER_LIMIT_DEG);
    gEdDbg.deltaRad = gEdDbg.deltaDeg * (PI_F / 180.0f);
    gEdDbg.absDeltaRad = fabsf(gEdDbg.deltaRad);
    gEdDbg.absV_mps = fabsf(gEdDbg.vCmd_mps);
    gEdDbg.speedSign = (gEdDbg.vCmd_mps >= 0.0f) ? 1.0f : -1.0f;
    gEdDbg.turnRadius_m = 0.0f;
    gEdDbg.vInner_mps = 0.0f;
    gEdDbg.vOuter_mps = 0.0f;
    gEdDbg.vLeft_mps = 0.0f;
    gEdDbg.vRight_mps = 0.0f;
    gEdDbg.wLeftWheel_rpm = 0.0f;
    gEdDbg.wRightWheel_rpm = 0.0f;
    gEdDbg.mLeft_rpm = 0.0f;
    gEdDbg.mRight_rpm = 0.0f;
    gEdDbg.m1Ref_rpm = 0.0f;
    gEdDbg.m2Ref_rpm = 0.0f;

    gSteerAngleCmd_rad = gEdDbg.deltaRad;

    if(gEdDbg.absV_mps < 1.0e-6f)
    {
        gEdYawRateRef_rad_s = 0.0f;
    }
    else if(gEdDbg.absDeltaRad < 1.0e-6f)
    {
        gEdDbg.vInner_mps = gEdDbg.absV_mps;
        gEdDbg.vOuter_mps = gEdDbg.absV_mps;
        gEdDbg.vLeft_mps = gEdDbg.vCmd_mps;
        gEdDbg.vRight_mps = gEdDbg.vCmd_mps;
        gEdYawRateRef_rad_s = 0.0f;
    }
    else
    {
        gEdDbg.turnRadius_m = ED_WHEELBASE_M / tanf(gEdDbg.absDeltaRad);

        if(gEdDbg.turnRadius_m < (ED_TRACK_WIDTH_M * 0.5f))
        {
            gEdDbg.turnRadius_m = ED_TRACK_WIDTH_M * 0.5f;
        }

        gEdDbg.vInner_mps = gEdDbg.absV_mps * ((gEdDbg.turnRadius_m - (ED_TRACK_WIDTH_M * 0.5f)) / gEdDbg.turnRadius_m);
        gEdDbg.vOuter_mps = gEdDbg.absV_mps * ((gEdDbg.turnRadius_m + (ED_TRACK_WIDTH_M * 0.5f)) / gEdDbg.turnRadius_m);

        if(gEdDbg.deltaRad > 0.0f)
        {
            gEdDbg.vLeft_mps = gEdDbg.speedSign * gEdDbg.vInner_mps;
            gEdDbg.vRight_mps = gEdDbg.speedSign * gEdDbg.vOuter_mps;
            gEdYawRateRef_rad_s = gEdDbg.vCmd_mps / gEdDbg.turnRadius_m;
        }
        else
        {
            gEdDbg.vLeft_mps = gEdDbg.speedSign * gEdDbg.vOuter_mps;
            gEdDbg.vRight_mps = gEdDbg.speedSign * gEdDbg.vInner_mps;
            gEdYawRateRef_rad_s = -gEdDbg.vCmd_mps / gEdDbg.turnRadius_m;
        }
    }

    gEdTurnRadius_m = gEdDbg.turnRadius_m;
    gEdRearLeftLinRef_mps = gEdDbg.vLeft_mps;
    gEdRearRightLinRef_mps = gEdDbg.vRight_mps;

    gEdDbg.denom = TWO_PI_F * ED_WHEEL_RADIUS_M;
    gEdDbg.wLeftWheel_rpm  = (fabsf(gEdDbg.vLeft_mps)  / gEdDbg.denom) * 60.0f;
    gEdDbg.wRightWheel_rpm = (fabsf(gEdDbg.vRight_mps) / gEdDbg.denom) * 60.0f;

    gEdDbg.mLeft_rpm  = gEdDbg.wLeftWheel_rpm  * ED_GEAR_RATIO;
    gEdDbg.mRight_rpm = gEdDbg.wRightWheel_rpm * ED_GEAR_RATIO;

    if(gEdDbg.vLeft_mps < 0.0f)  gEdDbg.mLeft_rpm  = -gEdDbg.mLeft_rpm;
    if(gEdDbg.vRight_mps < 0.0f) gEdDbg.mRight_rpm = -gEdDbg.mRight_rpm;

    gEdRearLeftWheelRef_rpm  = (gEdDbg.vLeft_mps  >= 0.0f) ? gEdDbg.wLeftWheel_rpm  : -gEdDbg.wLeftWheel_rpm;
    gEdRearRightWheelRef_rpm = (gEdDbg.vRight_mps >= 0.0f) ? gEdDbg.wRightWheel_rpm : -gEdDbg.wRightWheel_rpm;
    gEdRearLeftMotorRef_rpm  = gEdDbg.mLeft_rpm;
    gEdRearRightMotorRef_rpm = gEdDbg.mRight_rpm;

#if ED_MOTOR1_IS_REAR_LEFT
    gEdDbg.m1Ref_rpm = gEdDbg.mLeft_rpm;
    gEdDbg.m2Ref_rpm = gEdDbg.mRight_rpm;
#else
    gEdDbg.m1Ref_rpm = gEdDbg.mRight_rpm;
    gEdDbg.m2Ref_rpm = gEdDbg.mLeft_rpm;
#endif

    gEdDbg.m1Ref_rpm = ClampF(gEdDbg.m1Ref_rpm, -ED_MOTOR_RPM_LIMIT, ED_MOTOR_RPM_LIMIT);
    gEdDbg.m2Ref_rpm = ClampF(gEdDbg.m2Ref_rpm, -ED_MOTOR_RPM_LIMIT, ED_MOTOR_RPM_LIMIT);

    gEdMotor1Ref_rpm = gEdDbg.m1Ref_rpm;
    gEdMotor2Ref_rpm = gEdDbg.m2Ref_rpm;

    gM1.StartCmd = gEdStartCmd;
    gM2.StartCmd = gEdStartCmd;

    gM1.DirectionCmd = (gEdDbg.m1Ref_rpm >= 0.0f) ? 1 : -1;
    gM2.DirectionCmd = (gEdDbg.m2Ref_rpm >= 0.0f) ? 1 : -1;

    gM1.SpeedRefRpmCmd = fabsf(gEdDbg.m1Ref_rpm);
    gM2.SpeedRefRpmCmd = fabsf(gEdDbg.m2Ref_rpm);
}

// ===================== 1 kHz Timer ISR =====================
interrupt void cpu_timer0_isr(void)
{
    ElectronicDiff_UpdateRefs_1kHz();
    App_RunStateMachine_1kHz(&gM1, &gM1Hw);
    App_RunStateMachine_1kHz(&gM2, &gM2Hw);

    CpuTimer0Regs.TCR.bit.TIF = 1U;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// ===================== ADC ISR =====================
interrupt void adca1_isr(void)
{
    gM1.IA_raw  = AdccResultRegs.ADCRESULT0;
    gM1.IB_raw  = AdcbResultRegs.ADCRESULT0;
    gM1.VBS_raw = AdcaResultRegs.ADCRESULT1;

    gM2.IA_raw  = AdccResultRegs.ADCRESULT1;
    gM2.IB_raw  = AdcbResultRegs.ADCRESULT1;
    gM2.VBS_raw = AdcaResultRegs.ADCRESULT2;

    gM1.AdcIsrCount++;
    gM2.AdcIsrCount++;

    if(gM1.CalActive)
    {
        gM1.IA_sum += gM1.IA_raw;
        gM1.IB_sum += gM1.IB_raw;
        gM1.CalCount++;

        if(gM1.CalCount >= CURR_OFFSET_SAMPLES)
        {
            gM1.CalActive = 0U;
            gM1.CalDone = 1U;
        }
    }

    if(gM2.CalActive)
    {
        gM2.IA_sum += gM2.IA_raw;
        gM2.IB_sum += gM2.IB_raw;
        gM2.CalCount++;

        if(gM2.CalCount >= CURR_OFFSET_SAMPLES)
        {
            gM2.CalActive = 0U;
            gM2.CalDone = 1U;
        }
    }

    ConvertRawToPhysical(&gM1);
    ConvertRawToPhysical(&gM2);

    UpdateFiltersAndAlarms_ISR(&gM1);
    UpdateFiltersAndAlarms_ISR(&gM2);

    DRV8301_ReadStatusPins(&gM1, &gM1Hw);
    DRV8301_ReadStatusPins(&gM2, &gM2Hw);

    if((gM1.FaultActive != 0U) ||
       (fabsf(gM1.IA_A) > FOC_OC_LIMIT_A) ||
       (fabsf(gM1.IB_A) > FOC_OC_LIMIT_A))
    {
        gM1.OverCurrentTrip = ((fabsf(gM1.IA_A) > FOC_OC_LIMIT_A) ||
                               (fabsf(gM1.IB_A) > FOC_OC_LIMIT_A)) ? 1U : 0U;

        gM1.AppState = APP_STATE_FAULT;
        PWM_EnableOutputs(&gM1, &gM1Hw, 0U);
        PWM_UpdateDutyABC(&gM1, &gM1Hw, 0.5f, 0.5f, 0.5f);
        FOC_ResetControllers(&gM1);
    }
    else
    {
        switch(gM1.AppState)
        {
            case APP_STATE_STOP:
                PWM_UpdateDutyABC(&gM1, &gM1Hw, 0.5f, 0.5f, 0.5f);
                break;

            case APP_STATE_ALIGN:
                gM1.ThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
                Align_ApplyVoltageVector(&gM1, &gM1Hw, ALIGN_ELEC_ANGLE_RAD, ALIGN_MOD_INDEX, gM1.VBS_bus_V);
                break;

            case APP_STATE_VERIFY:
                EQEP_UpdatePositionFromCount(&gM1, &gM1Hw);
                gM1.ThetaCtrlRad = gM1.ThetaElecEncRad;
                FOC_RunCurrentLoop_ISR(&gM1, &gM1Hw);
                break;

            case APP_STATE_CLOSEDLOOP:
                EQEP_UpdatePositionFromCount(&gM1, &gM1Hw);
                gM1.ThetaCtrlRad = gM1.ThetaElecEncRad;
                FOC_RunCurrentLoop_ISR(&gM1, &gM1Hw);
                break;

            case APP_STATE_FAULT:
            default:
                PWM_UpdateDutyABC(&gM1, &gM1Hw, 0.5f, 0.5f, 0.5f);
                PWM_EnableOutputs(&gM1, &gM1Hw, 0U);
                break;
        }
    }

    if((gM2.FaultActive != 0U) ||
       (fabsf(gM2.IA_A) > FOC_OC_LIMIT_A) ||
       (fabsf(gM2.IB_A) > FOC_OC_LIMIT_A))
    {
        gM2.OverCurrentTrip = ((fabsf(gM2.IA_A) > FOC_OC_LIMIT_A) ||
                               (fabsf(gM2.IB_A) > FOC_OC_LIMIT_A)) ? 1U : 0U;

        gM2.AppState = APP_STATE_FAULT;
        PWM_EnableOutputs(&gM2, &gM2Hw, 0U);
        PWM_UpdateDutyABC(&gM2, &gM2Hw, 0.5f, 0.5f, 0.5f);
        FOC_ResetControllers(&gM2);
    }
    else
    {
        switch(gM2.AppState)
        {
            case APP_STATE_STOP:
                PWM_UpdateDutyABC(&gM2, &gM2Hw, 0.5f, 0.5f, 0.5f);
                break;

            case APP_STATE_ALIGN:
                gM2.ThetaCtrlRad = ALIGN_ELEC_ANGLE_RAD;
                Align_ApplyVoltageVector(&gM2, &gM2Hw, ALIGN_ELEC_ANGLE_RAD, ALIGN_MOD_INDEX, gM2.VBS_bus_V);
                break;

            case APP_STATE_VERIFY:
                EQEP_UpdatePositionFromCount(&gM2, &gM2Hw);
                gM2.ThetaCtrlRad = gM2.ThetaElecEncRad;
                FOC_RunCurrentLoop_ISR(&gM2, &gM2Hw);
                break;

            case APP_STATE_CLOSEDLOOP:
                EQEP_UpdatePositionFromCount(&gM2, &gM2Hw);
                gM2.ThetaCtrlRad = gM2.ThetaElecEncRad;
                FOC_RunCurrentLoop_ISR(&gM2, &gM2Hw);
                break;

            case APP_STATE_FAULT:
            default:
                PWM_UpdateDutyABC(&gM2, &gM2Hw, 0.5f, 0.5f, 0.5f);
                PWM_EnableOutputs(&gM2, &gM2Hw, 0U);
                break;
        }
    }

    if((gM1.AdcIsrCount % 2000U) == 0U)
    {
        gM1.SpiServiceReq = 1U;
        gM2.SpiServiceReq = 1U;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;

    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 == 1U)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
