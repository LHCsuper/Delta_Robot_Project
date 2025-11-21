
#include "inverse_kin.h"

inline double square(double x)
{
    return x * x;
}


int inverse_kin :: calculations(double position_data[3], double fi_array[3])
{
    double x = position_data[0];
    double y = position_data[1];
    double z = position_data[2];
    //Constants used to calculate Inverse Kinematics
    double Ei[3] = {2 * Length * (y + a), -Length * (sqrt(3) * (x + b) + y + c), Length * (sqrt(3) * (x - b) - y - c)};
    double Fi[3] = {2 * z * Length, 2 * z * Length, 2 * z * Length};
    double Gi[3] = {square(x) + square(y) + square(z) + square(a) + square(Length) + 2 * y * a - square(length), 
                   square(x) + square(y) + square(z) + square(b) + square(c) + square(Length) + 2 * x * b + 2 * y * c - square(length), 
                   square(x) + square(y) + square(z) + square(b) + square(c) + square(Length) - 2 * x * b + 2 * y * c - square(length)};

    //Calculate motor angles, check if they are within specified range
    for (int i=0; i<3; i++){
      double temp = 2 * atan((-Fi[i] + sqrt(square(Fi[i]) + square(Ei[i]) - square(Gi[i]))) / (Gi[i] - Ei[i]));
      if (temp >= OUT_OF_RANGE_ANGLE_LOW && temp <= OUT_OF_RANGE_ANGLE_HIGH){
        fi_array[i] = temp * 180 / 3.14;        
      }
      else{
        double temp = 2 * atan((-Fi[i] - sqrt(square(Fi[i]) + square(Ei[i]) - square(Gi[i]))) / (Gi[i] - Ei[i]));
        if (temp >= OUT_OF_RANGE_ANGLE_LOW && temp <= OUT_OF_RANGE_ANGLE_HIGH){
          fi_array[i] = temp * 180 / 3.14;
        }
        else{
          fi_array[0] = -100;
          fi_array[1] = -100;
          fi_array[2] = -100;
          return 0; //-1 is error - coordiantes out of range
        }
      }
    }
  return 1;
};