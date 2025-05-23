#include "Text_loader.h"

Tot_txt_load::Tot_txt_load(char* txt_name)
{
    this->File_instance = fopen(txt_name,"r");
    if(this->File_instance == NULL)
    {
        printf("Wrong path entered");
    }
    
}

void Tot_txt_load::Tot_data_read(Eigen::MatrixXd& data_out)
{
    int reti = -1;
    float output[YTxtCol_number] = {0,};
    uint32_t Read_counter = 0;

    while(1)
    {
        #if(YTxtCol_number == 3)
        reti = fscanf(this->File_instance,"%f %f %f",
        &output[0],&output[1],&output[2]);
        #elif(YTxtCol_number == 4)
        reti = fscanf(this->File_instance,"%f %f %f %f",
        &output[0],&output[1],&output[2],&output[3]);
        #elif(YTxtCol_number == 5)
        reti = fscanf(this->File_instance,"%f %f %f %f %f",
        &output[0],&output[1],&output[2],&output[3],&output[4]);
        #elif(YTxtCol_number == 6)
        reti = fscanf(this->File_instance,"%f %f %f %f %f %f",
        &output[0],&output[1],&output[2],&output[3],&output[4],&output[5]);
        #elif(YTxtCol_number == 7)
        reti = fscanf(this->File_instance,"%f %f %f %f %f %f %f",
        &output[0],&output[1],&output[2],&output[3],&output[4],&output[5],
        &output[6]);
        #elif(YTxtCol_number == 8)
        reti = fscanf(this->File_instance,"%f %f %f %f %f %f %f %f",
        &output[0],&output[1],&output[2],&output[3],&output[4],&output[5],
        &output[6],&output[7]);
        #elif(YTxtCol_number == 9)
        reti = fscanf(this->File_instance,"%f %f %f %f %f %f %f %f %f",
        &output[0],&output[1],&output[2],&output[3],&output[4],&output[5],
        &output[6],&output[7],&output[8]);
        #endif

        if(reti == -1) 
        {
            data_out = this->Decr_RD_points;
            fclose(this->File_instance);
            break;
        }
        else
        {
            if(Read_counter != 0) 
            {
                this->Inst_RD_points = this->Decr_RD_points;
                this->Decr_RD_points.resize(Read_counter+1,YTxtCol_number);
                this->Decr_RD_points.topRows(Read_counter) = this->Inst_RD_points;
            }
            else this->Decr_RD_points.topRows(Read_counter+1) = this->Inst_RD_points;
            
            for(int i=0;i<YTxtCol_number;i++) this->Decr_RD_points(Read_counter,i) = (double)output[i];

            Read_counter++;
        } 
    }
}