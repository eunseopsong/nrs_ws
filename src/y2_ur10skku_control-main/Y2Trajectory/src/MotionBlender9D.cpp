#include "Y2Trajectory/MotionBlender9D.hpp"

YMatrix MotionBlender9D::blendMotion(double Defualt_travelTime)
{
    /*** Error Handling ***/
    if(position.rows() < 2)
    {
        std::cerr << "Error: Not enough positions to blend motion." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if(position.cols() != FROFILE_DOF)
    {
        std::cerr << "Error: Position matrix must have " << FROFILE_DOF << " columns." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if(velocity.size() != position.rows())
    {
        std::cerr << "Error: Velocity vector size must match the number of positions." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    /*** Interpolation ***/
    YMatrix interpolated(1,FROFILE_DOF+1);  // Placeholder for interpolated data

    std::vector<double> pre_row = position[0];
    for(int interpol_counter=1; interpol_counter<position.rows(); interpol_counter++)
    {   
        auto row = position[interpol_counter];
        /* Basic transform */
        Position3D startPosi(pre_row[0], pre_row[1], pre_row[2]);
        Position3D endPosi(row[0], row[1], row[2]);
        SpatialAngle startOmega(pre_row[3], pre_row[4], pre_row[5]);
        SpatialAngle endOmega(row[3], row[4], row[5]);
        Quaternion startQuat = YMatrix::spatialAngleToQuaternion(startOmega);
        Quaternion endQuat = YMatrix::spatialAngleToQuaternion(endOmega);

        if(interpol_counter==1){
            interpolated[0] = {
            startPosi.x, startPosi.y, startPosi.z,
            startQuat.w, startQuat.x, startQuat.y, startQuat.z,
            row[6], row[7], row[8]
        };}

        /* Travel time calculation */
        auto Posi_dist_vector = endPosi - startPosi;
        auto Posi_dist = Posi_dist_vector.magnitude();
        auto Angle_btw_oris = fabs(YMatrix::angleBetweenQuaternions(startQuat, endQuat));

        auto Lin_travelTime = (velocity[interpol_counter] != 0.0) ? static_cast<double>(Posi_dist)/velocity[interpol_counter] : 0.0;
        auto Ang_travelTime = (ang_velocity[interpol_counter] != 0.0) ? static_cast<double>(Angle_btw_oris)/ang_velocity[interpol_counter] : 0.0;
        
        auto travelTime = (Lin_travelTime > Ang_travelTime)? Lin_travelTime:Ang_travelTime;

        /* 각속도 제한에 따른 travelTime 재조정 */
        if(travelTime != 0) {travelTime = (Angle_btw_oris/travelTime > angVelLimit) ? Angle_btw_oris / angVelLimit : travelTime;}
        else {travelTime = (Angle_btw_oris != 0) ? Angle_btw_oris / angVelLimit : 0.0;}

        /* In the case of holding time != 0 */
        if(holding_time[interpol_counter] != 0)
        {
            startPosi = endPosi;
            startQuat = endQuat;
            travelTime = holding_time[interpol_counter];
        }

        /* Defualt travel time setting in the case of all zero-set */
        if((velocity[interpol_counter] == 0.0) && (ang_velocity[interpol_counter] == 0.0)
            && (holding_time[interpol_counter] == 0))
        {travelTime = Defualt_travelTime;}

        /* Interpolation */
        int interpol_num = static_cast<int>(travelTime / samplingTime);

        // Interpolation for position
        auto Interpol_posi = PositionInterpolator::interpolate(startPosi, endPosi, interpol_num);

        // Interpolation for orientation
        auto Interpol_ori = QuaternionInterpolator::interpolate(startQuat, endQuat, interpol_num);

        // Interpolation for force
        YMatrix Interpol_force(interpol_num, 3);
        for(int i = 0; i < interpol_num; i++)
        {
            Interpol_force[i][0] = row[6]; // fx
            Interpol_force[i][1] = row[7]; // fy
            Interpol_force[i][2] = row[8]; // fz
        }

        // Append interpolated data
        for(int i = 0; i < interpol_num; i++)
        {
            YMatrix interpolated_data = {{
                Interpol_posi[i].x, Interpol_posi[i].y, Interpol_posi[i].z,
                Interpol_ori[i].w, Interpol_ori[i].x, Interpol_ori[i].y, Interpol_ori[i].z,
                Interpol_force[i][0], Interpol_force[i][1], Interpol_force[i][2]
            }};
            interpolated.appendV(interpolated_data); // Vertical append
        }

        pre_row = row;
    }

    /*** Acceleration/Deceleration Profiling ***/
    YMatrix Blened_motionQ(interpolated.rows(),FROFILE_DOF+1); // Quaternion based
    // YMatrix Blened_motionRPY(interpolated.rows(),FROFILE_DOF); // RPY based
    YMatrix Blened_motionOmega(interpolated.rows(),FROFILE_DOF); // Spatial angle based

    for(int i = 0; i < interpolated.cols(); i++)
    {
        // Extract the ith column from the interpolated matrix
        YMatrix column = interpolated.extract(0, i, interpolated.rows(), 1);

        // Perform acceleration/deceleration profiling on the extracted column 
        YAccProfiler profiler(column.toVector(), startingTime, lastRestingTime, accelerationTime, samplingTime);
        YMatrix profiled_column = profiler.AccDecProfiling();
        
        if (Blened_motionQ.rows() != profiled_column.rows()) {
            Blened_motionQ.resize(profiled_column.rows(), FROFILE_DOF + 1);
        }
        // Append the profiled column to the blended motion matrix
        Blened_motionQ.insert(0, i, profiled_column.extract(0, 1, profiled_column.rows(), 1)); // Horizontal append
    }

    /*** Convert Quaternion to Spatial Angle ***/
    Blened_motionOmega.resize(Blened_motionQ.rows(), FROFILE_DOF);
    for(int i = 0; i < Blened_motionQ.rows(); i++)
    {
        // Extract the quaternion from the blended motion matrix
        std::vector<double> quat = Blened_motionQ.extract(i, 3, 1, 4)[0];

        // quaternion 정규화
        double qua_norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
        if (qua_norm < 1e-10) {
            throw std::invalid_argument("Quaternion norm is too small, cannot normalize.");
        }

        Quaternion quaternion(quat[0]/qua_norm, quat[1]/qua_norm, quat[2]/qua_norm, quat[3]/qua_norm);

        // Convert quaternion to Spatial angle
        SpatialAngle Omega = YMatrix::quaternionToSpatialAngle(quaternion);

        // Append the RPY values to the blended motion matrix
        for( int j = 0; j <FROFILE_DOF; j++)
        {
            if(j<3){Blened_motionOmega[i][j] = Blened_motionQ[i][j];} // Position
            else if ((j >= 3)&&(j < 6)){Blened_motionOmega[i][j] = (j==3)* Omega.x + (j==4)*Omega.y + (j==5)*Omega.z;} // Orientation
            else {Blened_motionOmega[i][j] = Blened_motionQ[i][j+1];} // Force/moment
        }
    }

    // return interpolated; // Return the interpolated motion data
    // return Blened_motionQ;  // Return the blended motion data in quaternion format
    return Blened_motionOmega;  // Return the blended motion data

}


