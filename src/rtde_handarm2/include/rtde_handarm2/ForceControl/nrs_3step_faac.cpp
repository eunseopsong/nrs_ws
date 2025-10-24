#include "Y2ForceCon/nrs_3step_faac.hpp"

/* Acceleration calculation using kalman filter */

SimpleKalmanFilter::SimpleKalmanFilter(double dt, std::vector<double>& process_noise, std::vector<double>& measurement_noise) {
    this->dt = dt;

    this->z_xpre = 0.0;
    this->z_vpre = 0.0;
    this->z_v = 0.0;  // Initial measurement vel
    this->z_a = 0.0;  // Initial measurement acc

    this->x = 0.0;  // Initial position
    this->P_x = 1.0;  // Initial covariance (guess)
    this->Q_x = process_noise[0];  // Process noise covariance (scalar)
    this->R_x = measurement_noise[0];  // Measurement noise covariance (scalar)

    this->v = 0.0;  // Initial velocity
    this->P_v = 1.0;  // Initial covariance (guess)
    this->Q_v = process_noise[1];  // Process noise covariance (scalar)
    this->R_v = measurement_noise[1];  // Measurement noise covariance (scalar)

    this->a = 0.0;  // Initial acceleration
    this->P_a = 1.0;  // Initial covariance (guess)
    this->Q_a = process_noise[2];  // Process noise covariance (scalar)
    this->R_a = measurement_noise[2];  // Measurement noise covariance (scalar)
}

// 예측 단계
void SimpleKalmanFilter::predict() {
    // 위치 예측 (단순화된 1차원 모델)
    x = x;  // 위치는 변하지 않음 (단일 변수 추적)
    P_x = P_x + Q_x;  // 공분산 예측

    // 속도 예측 (단순화된 1차원 모델)
    v = v;  // 속도는 변하지 않음 (단일 변수 추적)
    P_v = P_v + Q_v;  // 공분산 예측

    // 가속 예측 (단순화된 1차원 모델)
    a = a;  // 가속은 변하지 않음 (단일 변수 추적)
    P_a = P_a + Q_a;  // 공분산 예측
}

// 업데이트 단계
void SimpleKalmanFilter::update(double z_x) {
    
    // vel, acc 계산산
    z_v = (z_x-z_xpre)/dt;
    z_a = (z_v-z_vpre)/dt;
    z_xpre = z_x;
    z_vpre = z_v;

    // 측정 잔차 (y = z - x)
    double y_x = z_x - x;
    double y_v = z_v - v;
    double y_a = z_a - a;

    // 칼만 게인 계산: K = P * (P + R)^-1
    double K_x = P_x / (P_x + R_x);
    double K_v = P_v / (P_v + R_v);
    double K_a = P_a / (P_a + R_a);

    // 상태 업데이트
    x +=  K_x * y_x;
    v +=  K_v * y_v;
    a +=  K_a * y_a;

    // 공분산 업데이트
    P_x = (1 - K_x) * P_x;
    P_v = (1 - K_v) * P_v;
    P_a = (1 - K_a) * P_a;
}




/*** 3 - Step FAAC ***/
Nrs3StepFAAC::Nrs3StepFAAC(double _dt, std::vector<double> _process_noise, std::vector<double> _measurement_noise)
: dt(_dt), kf(_dt, _process_noise, _measurement_noise)
{
    // Constructor implementation
    // Initialize any member variables or resources here if needed

    /*** FAAC Initialization ***/
    Pre_Ferr = 0; // Force Error, Force Error Dot
    contact_flag = false; // Contact flag (To monitoring the contact status)

    /*** FAAC MDK Limitation Set (Very Important) ***/
    FAAC_Md_Limit = {1, 10}; // Lower/Upper limit of Mass (Unit: kg)
    FAAC_MDR_Limit = {1, 20}; // Lower/Upper limit of MDR (Multiplication of Md & Dd)
    FAAC_MdMin_limit = {FAAC_Md_Limit[0], 5}; // Lower/Upper limit of MdMin (Unit: kg)

    /*** Fuzzy Common Setup ***/
    FAAC_MinMax = 6.0; // Min Max for Normalization
    FAAC_array_interval = 1; // Interval for FAAC array
    window_size = 200; // Window size for STD calculation
    contact_threshold = 5; // Contact threshold used for contact detection (Unit: N)

    /*** MDK Recovery Setup ***/
    Tank_MM = {0.2, 10}; // Tank Min Max (for MDK recovery)
    Zeta_min = 1; // zeta min. at MDK recovery
    tau_basic = 1; // Basic recovery time (Unit: s)
    tau_max = 5; // Max. recovery time limit (Unit: s)
    gamma_d = 5; // Weight for dampling ratio effect (for MDK recovery)

    /*** Fuzzy Input limitations ***/
    FEDMM = 1000; // Force Error dot Min. Max. 255
    FEMM =  5.5; // Force Error Min. Max. 5.5
    EpsilonMM = 5000000; // Epsilon Min. Max. 100
    XcMSTDMM = 2*pow(10,-3); // STD of Xc-X 2*10^-5

    /*** Fuzzy Output limitations ***/
    DMMM = 0.5; // Delta M Min. Max.
    DMDRatio_MM = 2.5; // Delta MDRatio Min. Max.
    DMdMM = 0.05; // Delta MdMin Min. Max.

}

void Nrs3StepFAAC::FAAC_Init(const double Init_md_, const double Init_dd_, const double Init_kd_)
{
    // Initialize any member variables or resources here if needed
    Init_md = Init_md_;
    Init_dd = Init_dd_;
    Init_kd = Init_kd_;

    Updated_md = Init_md; // Updated Mass
    Updated_dd = Init_dd; // Updated Damping
    Updated_kd = Init_kd; // Updated Stiffness
    Init_MDR = Init_dd / Init_md; // Initial MassDamper Ratio
    Pre_Ferr = 0; // Force Error, Force Error Dot
    contact_flag = false; // Contact flag (To monitoring the contact status)
}

/* Step 0: Sub-algorithms calculation */
double Nrs3StepFAAC::FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max)
{
    /* To normalize & denormalize the input/output value */
    double Gamma, Normalized_out;
    Gamma = ((FAAC_Max)-(FAAC_Min))/(Input_Max-Input_Min);
    Normalized_out = ((FAAC_Max)+(FAAC_Min))/2 + Gamma*(Input - (Input_Max+Input_Min)/2);
    return Normalized_out;
}

double Nrs3StepFAAC::FAAC_Epsilon_Calc(double xc, double fext, double fd, double md, double dd)
{
    /* epsilon = |fext - fd - (md * acc + dd * vel)| 
    Used in 3rd step */ 

    kf.predict();
    kf.update(xc);

    double acc = kf.getAcceleration();
    double vel = kf.getVelocity();

    double epsilon = fabs(fext - fd - (md * acc + dd * vel));

    return epsilon;
}

double Nrs3StepFAAC::FAAC_XeSTD_Calc(const double Xe) 
{
    /* Xe = Xc-X (at contact direction)
    Used in 3rd step */

    /* Data gathering */
    STD_window.push_back(Xe); // Add the new value to the window
    if (STD_window.size() > window_size) { // Maintain a window size
        STD_window.erase(STD_window.begin()); // Remove the oldest value
    }

    /* Mean Calculation */
    double sum = 0.0;
    for (double value : STD_window) {
        sum += value;
    }
    double mean = sum / STD_window.size();

    /* Covariance Calculation */
    double variance = 0.0;
    for (double value : STD_window) {
        variance += (value - mean) * (value - mean);
    }
    variance /= STD_window.size();  // 모집단 표준편차 (sample 표준편차는 n-1로 나누기)

    return std::sqrt(variance); // Return the standard deviation
}

/* Step 1: Delta M calculation */
double Nrs3StepFAAC::FAAC_DelM_Calc(double F_Err, double FDot_Err)
{
    double FAAC_Nor_input1 = FDot_Err; // Force Error Dot input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= FEDMM) {FAAC_Nor_input1 = FEDMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -FEDMM) {FAAC_Nor_input1 = -FEDMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-FEDMM,FEDMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= map.FAAC_array_size[0]) {index1 = map.FAAC_array_size[0];}
    if(index2 >= map.FAAC_array_size[1]) {index2 = map.FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = map.FAAC_DelM_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DMMM,DMMM);
}

/* Step 2: Delta MDRatio calculation */
double Nrs3StepFAAC::FAAC_DelMDRatio_Calc(double F_Err, double FDot_Err)
{
    double FAAC_Nor_input1 = FDot_Err; // High-Pass Filtered Measured Force input
    double FAAC_Nor_input2 = F_Err; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= FEDMM) {FAAC_Nor_input1 = FEDMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -FEDMM) {FAAC_Nor_input1 = -FEDMM;}

    if (FAAC_Nor_input2 >= FEMM) {FAAC_Nor_input2 = FEMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -FEMM) {FAAC_Nor_input2 = -FEMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-FEDMM,FEDMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-FEMM,FEMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= map.FAAC_array_size[0]) {index1 = map.FAAC_array_size[0];}
    if(index2 >= map.FAAC_array_size[1]) {index2 = map.FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = map.FAAC_DelMDRatio_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DMDRatio_MM,DMDRatio_MM);
}

/* Step 3: Delta Md Minimum Limitation Calculation */
double Nrs3StepFAAC::FAAC_DMdM_Calc(double epsilon, double Xc_X_std)
{
    double FAAC_Nor_input1 = epsilon; // High-Pass Filtered Measured Force input
    double FAAC_Nor_input2 = Xc_X_std; // Force Error input

    // Step0 : Input value saturation
    if (FAAC_Nor_input1 >= EpsilonMM) {FAAC_Nor_input1 = EpsilonMM;} // Over the Max.
    else if (FAAC_Nor_input1 <= -EpsilonMM) {FAAC_Nor_input1 = -EpsilonMM;}

    if (FAAC_Nor_input2 >= XcMSTDMM) {FAAC_Nor_input2 = XcMSTDMM;} // Over the Max.
    else if (FAAC_Nor_input2 <= -XcMSTDMM) {FAAC_Nor_input2 = -XcMSTDMM;} 

    // Step1 : Input value normalization
    double Normalized_input1 = FAAC_Normalize(FAAC_Nor_input1,-EpsilonMM,EpsilonMM,-FAAC_MinMax,FAAC_MinMax);
    double Normalized_input2 = FAAC_Normalize(FAAC_Nor_input2,-XcMSTDMM,XcMSTDMM,-FAAC_MinMax,FAAC_MinMax);
    
    /* Index saturation */
    int index1 = abs((int)floor((Normalized_input1+FAAC_MinMax)/FAAC_array_interval));
    int index2 = abs((int)floor((Normalized_input2+FAAC_MinMax)/FAAC_array_interval));

    if(index1 >= map.FAAC_array_size[0]) {index1 = map.FAAC_array_size[0];}
    if(index2 >= map.FAAC_array_size[1]) {index2 = map.FAAC_array_size[1];}

    // Step2 : Output value matching - DelK & DelEtha
    double Normalized_output = map.FAAC_DMdM_array[index1][index2];

    // Step3 : Output denormalization
    return FAAC_Normalize(Normalized_output,-FAAC_MinMax,FAAC_MinMax,-DMdMM,DMdMM);
}

/* Step 4: MDK Recovery at Contact Transition */
void Nrs3StepFAAC::FAAC_MDKTransi_Calc(const double Tank_energy)
{
    // Tank based scale factor calculation
    double sigma = clamp((Tank_energy-Tank_MM[0])/(Tank_MM[1]-Tank_MM[0]), 0.0, 1.0); // Clamp the value between 0 and 1

    // Damping ratio based scale factor calculation
    double dampling_ratio = Updated_dd / 2*sqrt(Updated_md*(Updated_kd + 0.001)); // Damping ratio calculation (0.001 is a small value to avoid division by zero)
    double phi = clamp((Zeta_min-dampling_ratio)/Zeta_min, 0.0, 1.0); // Clamp the value between 0 and 1

    // Dynamic recovery time constant calculation
    double tau_dynamic = std::min(tau_basic*std::log(1+1/(sigma+0.001))*(1+gamma_d*phi) , tau_max); // Dynamic recovery time constant calculation (0.001 is a small value to avoid division by zero)

    // MDK Recovery calculation
    Updated_md += (1/tau_dynamic)*(Init_md-Updated_md)*dt;
    Updated_dd += (1/tau_dynamic)*(Init_dd-Updated_dd)*dt;
    Updated_kd += (1/tau_dynamic)*(Init_kd-Updated_kd)*dt;
}

Nrs3StepFAAC_MDK Nrs3StepFAAC::FAAC_MDKob_RUN(
    const double Tank_Energy, const double F_ext, const double Fd, const double Xc, const double X)
{
    Nrs3StepFAAC_MDK FAAC_MDK;

    /*** Get Tank Energy ***/


    /*** Fuzzy Input Calculation ***/
    /*  --- Ferr & FDot_err Calculation */
    FAAC_Ferr = Fd - F_ext; // Force Error
    FAAC_FDot_Err = (FAAC_Ferr - Pre_Ferr) / dt; // Force Error Dot
    Pre_Ferr = FAAC_Ferr; // Update Pre_Ferr for next iteration

    /*  --- Epsilon Calculation */
    FAAC_Epsilon = FAAC_Epsilon_Calc(Xc, F_ext, Fd, Updated_md, Updated_dd); // Example values for xc, fext, fd, md, dd

    /*  --- Standard Diviation of Xe Calculation */
    double FAAC_Xe = Xc - X; // Xe = Xc - X (at contact direction)
    FAAC_XeSTD = FAAC_XeSTD_Calc(FAAC_Xe); // Example values for STD_window, window_size, Xe


    /*** Contact Status Decision ***/
    if (fabs(F_ext) >= contact_threshold || fabs(Fd) >= 0.01) {contact_flag = true;} // Contact detected
    else {contact_flag = false;} // No contact

    /*** MD Calculation at Contact ***/
    if(contact_flag)
    {
        /*  --- Delta Md Calculation */
        double FAAC_DelM = FAAC_DelM_Calc(FAAC_Ferr, FAAC_FDot_Err); // Example values for F_Err, FDot_Err

        /*  --- Delta MDR Calculation */
        double FAAC_DelMDR = FAAC_DelMDRatio_Calc(FAAC_Ferr, FAAC_FDot_Err); // Example values for F_Err, FDot_Err

        /*  --- Delta Md Min. Limitation Calculation */
        double FAAC_DMdM = FAAC_DMdM_Calc(FAAC_Epsilon, FAAC_XeSTD); // Example values for epsilon, Xc_X_std

        /*  --- MD Satuaration Using Limitations */
        Updated_kd = 0; // Stiffness calculation (Stiffness = Kd)

        /* Mass Min. Limit Saturation */
        FAAC_Md_Limit[0] += FAAC_DMdM; // Update the lower limit of Mass (Unit: kg)
        FAAC_Md_Limit[0] = (FAAC_Md_Limit[0] >= FAAC_MdMin_limit[1]) ? FAAC_MdMin_limit[1] : FAAC_Md_Limit[0];
        FAAC_Md_Limit[0] = (FAAC_Md_Limit[0] <= FAAC_MdMin_limit[0]) ? FAAC_MdMin_limit[0] : FAAC_Md_Limit[0];

        /* Mass saturation */
        Updated_md += FAAC_DelM;
        Updated_md = (Updated_md >= FAAC_Md_Limit[1]) ? FAAC_Md_Limit[1] : Updated_md;
        Updated_md = (Updated_md <= FAAC_Md_Limit[0]) ? FAAC_Md_Limit[0] : Updated_md;

        /* MDR saturation */
        Updated_MDR += FAAC_DelMDR;
        Updated_MDR = (Updated_MDR >= FAAC_MDR_Limit[1]) ? FAAC_MDR_Limit[1] : Updated_MDR;
        Updated_MDR = (Updated_MDR <= FAAC_MDR_Limit[0]) ? FAAC_MDR_Limit[0] : Updated_MDR;
        Updated_dd = Updated_MDR * Init_MDR * Updated_md;
    }
    /* MDK Calculation at Contact Transition */
    else if(!contact_flag && fabs(Init_md-Updated_md) > 0.01) // No contact
    {
        /*  --- MD Recovery Calculation */
        FAAC_MDKTransi_Calc(Tank_Energy);
    }
    else
    {
        Updated_md = Init_md; // Reset to initial values
        Updated_dd = Init_dd; // Reset to initial values
        Updated_kd = Init_kd; // Reset to initial values
        Updated_MDR = 1;
    }

    /* Mass, Damping, Stiffness Transmit to Return */
    FAAC_MDK.Mass = Updated_md; // Mass
    FAAC_MDK.Damping = Updated_dd; // Damping
    FAAC_MDK.Stiffness = Updated_kd; // Stiffness
    return FAAC_MDK;
}
