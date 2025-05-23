/* Export structures */
struct kalman_filter_par
{
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
};

struct kalman_filter1D_par
{
    float x_pre, p_pre; 
    float Q, R;

};

struct Moving_avg_par
{
    float saved_data[1000]={0,};
    int mv_num;
    int counter = 0;
};

struct High_pass_par
{
	double timeZone[3] = {0,};  // Do not change (defualt:0)

	double f_cut = 5; // cut-off frequency(Hz)
	double zeta = 0.7; // damping ratio
	double ts = 0.001; // sampling time(s)
	
};

struct Low_pass_par
{
	double PastInput = 0; // Do not change (defualt:0)
    double PastOutput = 0; // Do not change (defualt:0)

	double CutOffFrequency = 5; // cut-off frequency(Hz)
	double SamplingFrequency = 1000; // sampling frequency(Hz)
	
};

struct Band_stop_par
{
	double timeZone[3] = {0,}; // Do not change (defualt:0)

	double f_peak = 5; // stop frequency(Hz)
	double bandWidth = 5; // stop frequency width(Hz)
	double ts = 0.001; // sampling time(s)
	
};


class Yoon_filters
{
    private:

    public:
        Yoon_filters() {}
        ~Yoon_filters() {}
        
        /* Filter parameter instance generation */
        kalman_filter_par KF_par;
        kalman_filter1D_par KF1D_par;
        Moving_avg_par MV_par;
        High_pass_par HPF_par;
        Low_pass_par LPF_par;
        Band_stop_par BSF_par;

        float KalmanFilter(float input); 
        float KalmanFilter1D(float input);
        float MovingAvgFilter(float input);
        float HighPassFilter(float input);
        float LowPassFilter(float input);
        float BandStopFilter(float input);
};