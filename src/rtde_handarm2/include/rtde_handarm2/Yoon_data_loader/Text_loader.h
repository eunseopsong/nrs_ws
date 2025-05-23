#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <Eigen/Dense>

#define YTxtRaw_number 100
#define YTxtCol_number 7 // Only 3~9


class Tot_txt_load
{
    private:
        FILE* File_instance;

        Eigen::MatrixXd Decr_RD_points = Eigen::MatrixXd::Zero(1,YTxtCol_number);
        Eigen::MatrixXd Inst_RD_points = Eigen::MatrixXd::Zero(1,YTxtCol_number);
    public:
        Tot_txt_load(char* txt_name);
        ~Tot_txt_load() {};

        void Tot_data_read(Eigen::MatrixXd& data_out);

};