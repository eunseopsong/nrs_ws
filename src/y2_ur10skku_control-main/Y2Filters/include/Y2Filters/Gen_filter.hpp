#include <iostream>
#include <vector>
#include <numeric>


/*** Kalman Filter Class ***/
class Y_KalmanFilter
{
    public:

        /***  General Kalman Filter Parameters ***/

        /* Transition matrix: 2x2 */
        float Phi_matrix[4];
        /* Q covariance plant noise matrix: 2x2 */
        float Q_matrix[4];
        /* Sensitivity matrix: 1X2 */
        float H_matrix[2];
        /* Observation noise: R covariance matrix 1x1 */
        float R_matrix;
        /* P plus current covariance matrix 2x2: estimate error */
        float P_plus[4];
        /* x plus current state vector 2x1: value, speed */
        float x_plus[2];

        /***  1D Kalman filter parameters ***/
        float x_pre, p_pre; 
        float Q, R;

        /***  General Kalman Filter Functions ***/

        Y_KalmanFilter() {}
        ~Y_KalmanFilter() {}

        float KalmanFilter(float input); 
        float KalmanFilter1D(float input);


    private:

};

/*** Moving Everage Filter Class ***/
class Y_MovFilter
{
    public: 
        /***  Moving Everage Filter Parameters ***/

        int mv_num = 1;
        int counter = 0;
        std::vector<float> saved_data;
        float running_sum = 0.0f;  // Running sum to avoid O(n) accumulate

        /***  Moving Everage Filter Functions ***/

        Y_MovFilter(int mv_num_);
        Y_MovFilter() : Y_MovFilter(0) {}  // 기본 생성자 (다른 class 에서 array로 instance 사용시 필수)
        ~Y_MovFilter() {}

        float MovFilter(float input);

    private:
        float output = 0;

};


/*** Frequency Filter Class ***/
class Y_FreqFilter
{
    public:
    /*** Frequency Filter Parameters ***/
    /* Common parameters */
    double Ts; // sampling time(s)

    /* High Pass Filter Parameters */
    double HFP_timeZone[3] = {0,};  // Do not change (defualt:0)

	double HPF_cutF = 5; // cut-off frequency(Hz)
	double HPF_zeta = 0.7; // damping ratio
	
    /* Low Pass Filter Parameters */
    double PastInput = 0; // Do not change (defualt:0)
    double PastOutput = 0; // Do not change (defualt:0)

	double LPF_cutF = 5; // cut-off frequency(Hz)

    /* Band Stop Filter Parameters */
    double BSF_timeZone[3] = {0,}; // Do not change (defualt:0)

	double BSF_cutF = 5; // cut-off frequency, stop frequency(Hz)
	double BSF_BW = 5; // stop frequency width(Hz)

    /*** Frequency Filter Functions ***/
    Y_FreqFilter(double Ts_);
    Y_FreqFilter() : Y_FreqFilter(0.0) {}  // 기본 생성자 (다른 class 에서 array로 instance 사용시 필수)

    ~Y_FreqFilter() {}

    float HPF(float input);
    float LPF(float input);
    float BSF(float input);
};
