/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  閳ユ柡鈧柡鈧柡鈧柡鈧柡鈧柡鈧拷
 *  as + 1
 *
 ******************************************************************************
 */
#include "bmi_EKF.h"
#include "rp_config.h"
#if IMU_USE_EKF==1
/* 闁插秴濮忛崝鐘烩偓鐔峰 */
#define GRAVITY_EARTH  (9.80665f)
/* 閻晠妯€鐎圭偘绶ョ€规矮绠� */
arm_matrix_instance_f32 EKFTrans;
arm_matrix_instance_f32 EKFSrc;
arm_matrix_instance_f32 EKFDst;

/**
 * @brief   閸ф劖鐖ｉ崣妯诲床闁插洨鏁-Y-X濞喲勫鐟欐帗寮挎潻甯礉閸楀厖绮犻梽鈧摶杞板崕閸ф劖鐖ｇ化璇叉倻娴滄垵褰撮崸鎰垼缁褰夐幑顫厬閿涳拷
 *          閸ф劖鐖ｇ化缁樺瘻閻撗呯搏闂勨偓閾昏桨鍗嶼鏉炴番鈧箠鏉炴番鈧箚鏉炲娈戞い鍝勭碍閺冨娴�
 *          濮ｅ繋绔村▎鈩冩鏉烆剛娈戦崣鍌濃偓鍐ㄦ綏閺嶅洨閮存稉鍝勭秼閸撳秹妾ч摶杞板崕閸ф劖鐖ｇ化锟�  
 *  @param
 *  @arz
 *      闂勨偓閾昏桨鍗巟鏉炵繝绗宺oll鏉炵繝绠ｉ梻瀵告畱婢剁顫楅敍灞藉礋娴ｅ秳璐熸惔锟�
 *  @ary
 *      闂勨偓閾昏桨鍗巟鏉炵繝绗寉aw鏉炵繝绠ｉ梻瀵告畱婢剁顫楅敍灞藉礋娴ｅ秳璐熸惔锟�
 *  @arx
 *      闂勨偓閾昏桨鍗巠鏉炵繝绗寉aw鏉炵繝绠ｉ梻瀵告畱婢剁顫楅敍灞藉礋娴ｅ秳璐熸惔锟�
 */
gimbal_transform_t EKFgim_trans = {
    .arz = 90.0f,
    .ary = 0.0f,
    .arx = 0.0f,
    .trans = {0.0f},
};

QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 閸╄桨绨幍鈺佺潔閸椻€崇毜閺囧吋鎶ゅ▔銏㈡畱婵寧鈧浇袙缁犳鍨垫慨瀣
 * @param[in] process_noise1 鐠佸墽鐤嗛崶娑樺帗閺佹壆娈戞潻鍥┾柤閸ｎ亜锛愰崡蹇旀煙瀹割喚鐓╅梼纰夌礉鐡掑﹤鐨憴锝囩暬閺佺増宓佺搾濠傞挬濠婃埊绱濈搾濠傘亣缁崵绮虹€电懓鎻╅柅鐔峰綁閸栨牜娈戦崣宥呯安鐡掑﹤鎻�   10
 * @param[in] process_noise2 鐠佸墽鐤嗛梽鈧摶杞板崕闂嗚泛浜告导鎷岊吀鏉╁洨鈻奸崳顏勶紣閸楀繑鏌熷顔剧叐闂冿拷     0.001
 * @param[in] measure_noise  鐠佸墽鐤嗛崝鐘烩偓鐔峰鐠佲剝绁撮柌蹇撴珨婢规澘宕楅弬鐟版▕閻晠妯€閿涘矁绉虹亸蹇擃嚠閸旂娀鈧喎瀹崇搾濠佷繆娴犱紮绱濈化鑽ょ埠鐎电懓鎻╅柅鐔峰綁閸栨牜娈戦崣宥呯安鐡掑﹤鎻�       1000000
 * @param[in] lambda         鐠佸墽鐤嗗〒鎰Х閸ョ姴鐡欓梼鍙夘剾闂勨偓閾昏桨鍗庨梿璺轰焊娴兼媽顓搁崡蹇旀煙瀹割喛绻冩惔锔芥暪閺侊拷         0.9996
 */
void IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda)
{
    uint8_t first_init = (QEKF_INS.Initialized == 0U) ? 1U : 0U;
    KalmanFilter_t *kf = &QEKF_INS.IMU_QuaternionEKF;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 3.5e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;

    // 閸掓繂顫愰崠鏍叐闂冪數娣惔锔夸繆閹拷
    if (first_init != 0U)
    {
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 婵寧鈧礁鍨垫慨瀣

    // 閼奉亜鐣炬稊澶婂毐閺佹澘鍨垫慨瀣,閻€劋绨幍鈺佺潔閹存牕顤冮崝鐖噁閻ㄥ嫬鐔€绾偓閸旂喕鍏�
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 鐠佹儳鐣鹃弽鍥х箶娴ｏ拷,閻€劏鍤滅€规艾鍤遍弫鐗堟禌閹诡晳f閺嶅洤鍣銉╊€冩稉顓犳畱SetK(鐠侊紕鐣绘晶鐐垫抄)娴犮儱寮穢hatupdate(閸氬酣鐛欐导鎷岊吀/閾诲秴鎮�)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;
    }

    memset(kf->xhat_data, 0, sizeof(float) * kf->xhatSize);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        kf->xhat_data[i] = init_quaternion[i];
    }

    memset(kf->xhatminus_data, 0, sizeof(float) * kf->xhatSize);
    memset(kf->FilteredValue, 0, sizeof(float) * kf->xhatSize);
    memset(kf->Pminus_data, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memset(kf->FT_data, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memset(kf->K_data, 0, sizeof(float) * kf->xhatSize * kf->zSize);
    memset(kf->S_data, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memset(kf->temp_matrix_data, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memset(kf->temp_matrix_data1, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memset(kf->temp_vector_data, 0, sizeof(float) * kf->xhatSize);
    memset(kf->temp_vector_data1, 0, sizeof(float) * kf->xhatSize);
    memset(kf->P_data, 0, sizeof(float) * kf->xhatSize * kf->xhatSize);
    memcpy(kf->F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(kf->P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));

    kf->MeasurementValidNum = 0;
    memset(QEKF_INS.q, 0, sizeof(QEKF_INS.q));
    memset(QEKF_INS.GyroBias, 0, sizeof(QEKF_INS.GyroBias));
    memset(QEKF_INS.Gyro, 0, sizeof(QEKF_INS.Gyro));
    memset(QEKF_INS.Accel, 0, sizeof(QEKF_INS.Accel));
    QEKF_INS.StableFlag = 0;
    QEKF_INS.Roll = 0.0f;
    QEKF_INS.Pitch = 0.0f;
    QEKF_INS.Yaw = 0.0f;
    QEKF_INS.YawTotalAngle = 0.0f;
    QEKF_INS.YawAngleLast = 0.0f;
    QEKF_INS.YawRoundCount = 0;
    QEKF_INS.ChiSquare_Data[0] = 0.0f;
    QEKF_INS.Initialized = 1;
}

/**
 * @brief 閸╄桨绨幍鈺佺潔閸椻€崇毜閺囧吋鎶ゅ▔銏狀嚠閸ユ稑鍘撻弫鎷岀箻鐞涘本娲块弬锟�
 * @param[in]       闂勨偓閾昏桨鍗庨弫鐗堝祦 gx gy gz in rad/s
 * @param[in]       閸旂娀鈧喎瀹崇拋鈩冩殶閹癸拷 ax ay az in m/s
 * @param[in]       閺佺増宓侀弴瀛樻煀閸涖劍婀� in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,閻€劋绨弴瀛樻煀瀹搞儰缍旈悙鐟邦槱閻ㄥ嫮濮搁幀浣芥祮缁夌眽閻晠妯€
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 濮濄倝鍎撮崚鍡氼啎鐎规氨濮搁幀浣芥祮缁夎崵鐓╅梼绀旈惃鍕箯娑撳﹨顫楅柈銊ュ瀻 4x4鐎涙劗鐓╅梼锟�,閸楋拷0.5(Ohm-Ohm^bias)*deltaT,閸欏厖绗呯憴鎺撴箒娑撯偓娑擄拷2x2閸楁洑缍呴梼闈涘嚒缂佸繐鍨垫慨瀣婵傛垝绨�
    // 濞夈劍鍓伴崷鈺瑀edict濮濐檶閻ㄥ嫬褰告稉濠咁潡閺勶拷4x2閻ㄥ嫰娴傞惌鈺呮█,閸ョ姵顒濆В蹇旑偧predict閻ㄥ嫭妞傞崐娆撳厴娴兼俺鐨熼悽鈺〆mcpy閻€劌宕熸担宥夋█鐟曞棛娲婇崜宥勭鏉烆喚鍤庨幀褍瀵查崥搴ｆ畱閻晠妯€
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

		QEKF_INS.Accel[0] = ax;
		QEKF_INS.Accel[1] = ay;
		QEKF_INS.Accel[2] = az;
    // set z,閸楁洑缍呴崠鏍櫢閸旀稑濮為柅鐔峰閸氭垿鍣�
    arm_sqrt_f32(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2], &QEKF_INS.accl_norm);
		accelInvNorm = 1.0f / QEKF_INS.accl_norm;
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 閻€劌濮為柅鐔峰閸氭垿鍣洪弴瀛樻煀闁插繑绁撮崐锟�
    }

    // 鐠侊紕鐣婚梽鈧摶杞板崕閺佺増宓侀崪灞藉闁喎瀹抽弫鐗堝祦閻ㄥ嫬缍婃稉鈧崠鏍р偓纭风礉閻€劋绨崚銈嗘焽瑜版挸澧犻梽鈧摶杞板崕閻ㄥ嫯绻嶉崝銊уЦ閹拷
    arm_sqrt_f32(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] + QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] + QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2], &QEKF_INS.gyro_norm);


    // 婵″倹鐏夌憴鎺椻偓鐔峰鐏忓繋绨梼鍫濃偓闂寸瑬閸旂娀鈧喎瀹虫径鍕艾鐠佹儳鐣鹃懠鍐ㄦ纯閸愶拷,鐠併倓璐熸潻鎰З缁嬪啿鐣�,閸旂娀鈧喎瀹抽崣顖欎簰閻€劋绨穱顔筋劀鐟欐帡鈧喎瀹�
    // 缁嬪秴鎮楅崷銊︽付閸氬海娈戞慨鎸庘偓浣规纯閺備即鍎撮崚鍡曠窗閸掆晝鏁tableFlag閺夈儳鈥樼€癸拷
    if (QEKF_INS.accl_norm > 9.8f - 5.5f && QEKF_INS.accl_norm < 9.8f + 5.5f)
    {
        QEKF_INS.StableFlag = 1;
    }
    else
    {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,鏉╁洨鈻奸崳顏勶紣閸滃矁顫囧ù瀣珨婢规壆鐓╅梼锟�
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 鐠嬪啰鏁alman_filter.c鐏忎浇顥婃總鐣屾畱閸戣姤鏆�,濞夈劍鍓伴崙鐘遍嚋User_Funcx_f閻ㄥ嫯鐨熼悽锟�
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 閼惧嘲褰囬摶宥呮値閸氬海娈戦弫鐗堝祦,閸栧懏瀚崶娑樺帗閺佹澘鎷皒y闂嗗爼顥濋崐锟�
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 婢堆囧劥閸掑棙妞傞崐妾㈡潪鎾偓姘亯,閺冪姵纭剁憴鍌涚ゴyaw閻ㄥ嫭绱撶粔锟�

    // 閸掆晝鏁ら崶娑樺帗閺佹澘寮界憴锝嗩儌閹峰顫�
    arm_atan2_f32(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f, &QEKF_INS.Yaw);
    arm_atan2_f32(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f, &QEKF_INS.Roll);
		float sintemp, costemp;
		sintemp	= -2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2]);
    arm_sqrt_f32(1 - sintemp*sintemp, &costemp);
    arm_atan2_f32(sintemp, costemp, &QEKF_INS.Pitch);
		
		QEKF_INS.Yaw *= 57.295779513f;
		QEKF_INS.Roll *= 57.295779513f;
		QEKF_INS.Pitch *= 57.295779513f;
    // get Yaw total, yaw閺佺増宓侀崣顖濆厴娴兼俺绉存潻锟�360,婢跺嫮鎮婃稉鈧稉瀣煙娓氬灝鍙炬禒鏍у閼虫垝濞囬悽锟�(婵″倸鐨梽鈧摶锟�)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;
    }
		
		
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++; // 閸掓繂顫愰崠鏍︾秵闁碍鎶ゅ▔銏㈡暏,鐠佲剝鏆熷ù瀣槸閻拷
}

/**
 * @brief 閻€劋绨弴瀛樻煀缁炬寧鈧冨閸氬海娈戦悩鑸碘偓浣芥祮缁夎崵鐓╅梼绀旈崣鍏呯瑐鐟欐帞娈戞稉鈧稉锟�4x2閸掑棗娼￠惌鈺呮█,缁嬪秴鎮楅悽銊ょ艾閸楀繑鏌熷顔剧叐闂冪閻ㄥ嫭娲块弬锟�;
 *        楠炶泛顕梿鑸电磽閻ㄥ嫭鏌熷顔跨箻鐞涘矂妾洪崚锟�,闂冨弶顒涙潻鍥у閺€鑸垫殐楠炲爼妾洪獮鍛存Щ濮濄垹褰傞弫锟�
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    // quaternion normalize鐏忓棗娲撻崗鍐╂殶鐟欏嫯瀵栭崠鏍﹁礋閸楁洑缍呴崶娑樺帗閺侊拷
		q0 = kf->xhatminus_data[0];
		arm_quaternion_normalize_f32(kf->xhatminus_data, kf->xhatminus_data, 1);
		
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,闂冨弶顒涢梿鍫曨棟閸欏倹鏆熸潻鍥у閺€鑸垫殐
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 闂勬劕绠�,闂冨弶顒涢崣鎴炴殠
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 閸︺劌浼愭担婊呭仯婢跺嫯顓哥粻妤勵潎濞村鍤遍弫鐧�(x)閻ㄥ嚙acobi閻晠妯€H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 閸掆晝鏁ょ憴鍌涚ゴ閸婄厧鎷伴崗鍫ョ崣娴兼媽顓稿妤€鍩岄張鈧导妯兼畱閸氬酣鐛欐导鎷岊吀
 *        閸旂姴鍙嗘禍鍡楀幢閺傝顥呮灞间簰閸掋倖鏌囬摶宥呮値閸旂娀鈧喎瀹抽惃鍕蒋娴犺埖妲搁崥锔藉姬鐡掞拷
 *        閸氬本妞傚鏇炲弳閸欐垶鏆庢穱婵囧Б娣囨繆鐦夐幁璺哄А瀹搞儱鍠屾稉瀣畱韫囧懓顩﹂柌蹇旂ゴ閺囧瓨鏌�
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H璺疨'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H璺疨'(k)璺疕T
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H璺疨'(k)璺疕T + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 鐠侊紕鐣绘０鍕ゴ瀵版鍩岄惃鍕櫢閸旀稑濮為柅鐔峰閺傜懓鎮�(闁俺绻冩慨鎸庘偓浣藉箯閸欐牜娈�)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 鐠侊紕鐣绘０鍕ゴ閸婄厧鎷伴崥鍕嚋鏉炲娈戦弬鐟版倻娴ｆ瑥楦�
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 閸掆晝鏁ら崝鐘烩偓鐔峰鐠佲剝鏆熼幑顔绘叏濮濓拷
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,閸椻剝鏌熷Λ鈧锟�
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H璺疨'(k)璺疕T + R)璺�(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging,rk瀵板牆鐨敍宀冾嚛閺勫孩鎶ゅ▔銏犳珤閺€鑸垫殐
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged,瑜版挸澧爎k婢堆傜艾闂冨牆鈧》绱濇稉鏂剧閸撳秵鎶ゅ▔銏犳珤婢跺嫪绨弨鑸垫殐
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 鏉炴垝缍嬮棃娆愵剾閺冩湹绮涢弮鐘崇《闁俺绻冮崡鈩冩煙濡偓妤狅拷
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 濠娿倖灏濋崳銊ュ絺閺侊拷
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating,閹镐胶鐢婚弴瀛樻煀P閻晠妯€娴ｆ寧鎶ゅ▔銏犳珤閺€鑸垫殐
        }
        else
        {
            //  濞堝妯婇張顏堚偓姘崇箖閸椻剝鏌熷Λ鈧锟� 鏉炴垝缍嬬€涙ê婀潻鎰З閸旂娀鈧喎瀹抽敍灞剧ゴ闁插繐鈧棿绗夐崣顖欎繆閿涘奔绮庢０鍕ゴ
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating,鐠哄疇绻働閻晠妯€閻ㄥ嫭娲块弬锟�
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain,濠娿倖灏濋崳銊ヮ槱娴滃骸褰傞弫锝嗗灗閼板嵐k閸婇棿绗夋径褌绨梼鍫濃偓锟�
    {
        // scale adaptive,rk鐡掑﹤鐨崚娆忣杻閻╁﹨绉烘径锟�,閸氾箑鍨弴瀵告祲娣囷繝顣╁ù瀣偓锟�
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K,鐠侊紕鐣婚崡鈥崇毜閺囩厧顤冮惄锟�
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)璺疕T
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive,闁俺绻冮崡鈩冩煙濡偓妤犲矉绱濋崝銊︹偓浣界殶閺佹潙宕辩亸鏃€娴栨晶鐐垫抄閺夊啴鍣�
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)璺�(z(k) - H璺痻hat'(k))

    // 闂嗚埖绱撴穱顔筋劀闂勬劕绠�,娑撯偓閼割兛绗夋导姘箒鏉╁洤銇囬惃鍕磽缁夛拷
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; ++i)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 娑撳秳鎱ㄥ顤縜w鏉炲瓨鏆熼幑锟�
//    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF鐟欏倹绁撮悳顖濆Ν,閸忚泛鐤勭亸杈ㄦЦ閹跺﹥鏆熼幑顔碱槻閸掓湹绔存稉锟�
 *
 * @param kf kf缁鐎风€规矮绠�
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
  * @brief  闂勨偓閾昏桨鍗庨崸鎰垼閸欐ɑ宕查崚婵嗩潗閸栨牭绱濋懟銉ょ瑝闂団偓鐟曚礁褰夐幑銏犲讲閸︹暐mu_sensor.c娑撶挶mu_init鐏忓棗鍙惧▔銊╁櫞
  * @param  
  * @retval 
  */
void transform_init(gimbal_transform_t *gim_trans)
{
    float arz, ary, arx;

	/* 鐟欐帒瀹抽崡鏇氱秴鏉烆剚宕查敍鍧眔瀵冨閿涳拷 */
	arz = gim_trans->arz * (double)0.017453;
	ary = gim_trans->ary * (double)0.017453;
	arx = gim_trans->arx * (double)0.017453;

	/* 閺冨娴嗛惌鈺呮█鐠у鈧》绱欐稉澶夐嚋閺冨娴嗛惌鈺呮█閸欑姴濮為敍锟� */
	gim_trans->trans[0] = arm_cos_f32(arz)*arm_cos_f32(ary);
	gim_trans->trans[1] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) - arm_sin_f32(arz)*arm_cos_f32(arx);
	gim_trans->trans[2] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) + arm_sin_f32(arz)*arm_sin_f32(arx);
	gim_trans->trans[3] = arm_sin_f32(arz)*arm_cos_f32(ary);
	gim_trans->trans[4] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) + arm_cos_f32(arz)*arm_cos_f32(arx);
	gim_trans->trans[5] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) - arm_cos_f32(arz)*arm_sin_f32(arx);
	gim_trans->trans[6] = -arm_sin_f32(ary);
	gim_trans->trans[7] = arm_cos_f32(ary)*arm_sin_f32(arx);
	gim_trans->trans[8] = arm_cos_f32(ary)*arm_cos_f32(arx);
	
    /* 3x3閸欐ɑ宕查惌鈺呮█閸掓繂顫愰崠锟� */
	arm_mat_init_f32(&EKFTrans, 3, 3, (float *)gim_trans->trans); 
}

/**
  * @brief  鐏忓棝妾ч摶杞板崕閸ф劖鐖ｉ崣妯诲床娑撹桨绨崣鏉挎綏閺嶅浄绱濋懟銉ょ瑝闂団偓鐟曚礁褰夐幑銏犲讲閸︹暐mu_protocol.c娑撶挶mu_update鐏忓棗鍙惧▔銊╁櫞
  * @brief  閸ф劖鐖ｉ崣妯诲床闁插洨鏁-Y-X濞喲勫鐟欐帗寮挎潻甯礉閸楀厖绮犻梽鈧摶杞板崕閸ф劖鐖ｇ化璇叉倻娴滄垵褰撮崸鎰垼缁褰夐幑顫厬閿涘苯娼楅弽鍥╅兇閹稿鍙庣紒鏇㈡閾昏桨鍗嶼鏉炴番鈧箠鏉炴番鈧箚鏉炲娈戞い鍝勭碍閺冨娴�
  *					濮ｅ繋绔村▎鈩冩鏉烆剛娈戦崣鍌濃偓鍐ㄦ綏閺嶅洨閮存稉鍝勭秼閸撳秹妾ч摶杞板崕閸ф劖鐖ｇ化锟�
  * @param[in]  (int16_t) gx,  gy,  gz,  ax,  ay,  az
  * @param[out] (float *) gx, gy, gz, aax, ay, az
  */
void Vector_Transform(float gx, float gy, float gz,\
	                  float ax, float ay, float az,\
	                  float *ggx, float *ggy, float *ggz,\
					  float *aax, float *aay, float *aaz)
{
    /* 闂勨偓閾昏桨鍗庢潏鎾冲弳鏉堟挸鍤弫鎵矋鐎规矮绠� */
    float gyro_in[3], gyro_out[3];
    /* 閸旂娀鈧喎瀹虫潏鎾冲弳鏉堟挸鍤弫鎵矋鐎规矮绠� */
    float acc_in[3], acc_out[3];

	/* 闂勨偓閾昏桨鍗庣挧瀣偓锟� */
	gyro_in[0] = (float)gx, gyro_in[1] = (float)gy, gyro_in[2] = (float)gz;
	/* 閸旂娀鈧喎瀹崇拋陇绁撮崐锟� */
	acc_in[0] = (float)ax, acc_in[1] = (float)ay, acc_in[2] = (float)az;
	
	/* 闂勨偓閾昏桨鍗庨崸鎰垼閸欐ɑ宕� */
	arm_mat_init_f32(&EKFSrc, 1, 3, gyro_in);
	arm_mat_init_f32(&EKFDst, 1, 3, gyro_out);
	arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
	*ggx = gyro_out[0], *ggy = gyro_out[1], *ggz = gyro_out[2];
	
	/* 閸旂娀鈧喎瀹崇拋鈥虫綏閺嶅洤褰夐幑锟� */
	arm_mat_init_f32(&EKFSrc, 1, 3, acc_in);
	arm_mat_init_f32(&EKFDst, 1, 3, acc_out);
	arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
	*aax = acc_out[0], *aay = acc_out[1], *aaz = acc_out[2];
}

/**
 * @brief  閼惧嘲褰囨稉鏍櫕閸ф劖鐖ｇ化鑽ゆ畱閸旂娀鈧喎瀹�
 */
void BMI_Get_Acceleration(float pitch, float roll, float yaw,\
						  float ax, float ay, float az,\
						  float *accx, float *accy, float *accz)
{
	float imu_accx, imu_accy, imu_accz;
	
    /* 鐟欐帒瀹抽崚绉歰瀵冨閸掞拷 */
	pitch *= (double)0.017453;
	yaw   *= (double)0.017453;
	roll  *= (double)0.017453;

	imu_accx = ax + arm_sin_f32(pitch) * GRAVITY_EARTH;
	imu_accy = ay - arm_sin_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
	imu_accz = az - arm_cos_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
	
	*accx = imu_accx * arm_cos_f32(pitch) + imu_accz * arm_sin_f32(pitch);
	*accy = imu_accy * arm_cos_f32(roll) - imu_accz * arm_sin_f32(roll);
	*accz = imu_accz * arm_cos_f32(pitch) * arm_cos_f32(roll) - imu_accx * arm_sin_f32(pitch) * arm_cos_f32(roll) \
			+ imu_accy * arm_sin_f32(roll) * arm_cos_f32(pitch);
	
}

#endif
