#ifndef __MAT6X6D_H_INCLUDE__
#define __MAT6X6D_H_INCLUDE__


#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <vector>
#include <random>

#include "Vec3d.h"

class Mat6x6d{
private:
    double m_x[6][6];
public:
    Mat6x6d(){};
    ~Mat6x6d(){};
    Mat6x6d(const double *x) // x|0|1|2|3|4|... -> m_x|00|01|02|10|11|12|...
    {
        for(int i=0; i<6;i++)
        {
            for(int j=0; j<6; j++)
            {
                m_x[i][j] = x[i*6 + j];
            }
        }
    }
    Mat6x6d(const double x[][6])
    {
        for(int i=0; i<6;i++)
        {
            for(int j=0; j<6; j++)
            {
                m_x[i][j] = x[i][j];
            }
        }
    }
    Mat6x6d(const double &d11,const double &d12,const double &d13, const double &d14,const double &d15,const double &d16  ,
            const double &d21,const double &d22,const double &d23, const double &d24,const double &d25,const double &d26  ,
            const double &d31,const double &d32,const double &d33, const double &d34,const double &d35,const double &d36  ,
            const double &d41,const double &d42,const double &d43, const double &d44,const double &d45,const double &d46  ,
            const double &d51,const double &d52,const double &d53, const double &d54,const double &d55,const double &d56  ,
            const double &d61,const double &d62,const double &d63, const double &d64,const double &d65,const double &d66  ) {
                m_x[0][0] = d11; m_x[0][1] = d12; m_x[0][2] = d13; m_x[0][3] = d14; m_x[0][4] = d15; m_x[0][5] = d16;
                m_x[1][0] = d21; m_x[1][1] = d22; m_x[1][2] = d23; m_x[1][3] = d24; m_x[1][4] = d25; m_x[1][5] = d26;
                m_x[2][0] = d31; m_x[2][1] = d32; m_x[2][2] = d33; m_x[2][3] = d34; m_x[2][4] = d35; m_x[2][5] = d36;
                m_x[3][0] = d41; m_x[3][1] = d42; m_x[3][2] = d43; m_x[3][3] = d44; m_x[3][4] = d45; m_x[3][5] = d46;
                m_x[4][0] = d51; m_x[4][1] = d52; m_x[4][2] = d53; m_x[4][3] = d54; m_x[4][4] = d55; m_x[4][5] = d56;
                m_x[5][0] = d61; m_x[5][1] = d62; m_x[5][2] = d63; m_x[5][3] = d64; m_x[5][4] = d65; m_x[5][5] = d66;
            }

    double &set(int index1, int index2) {
        return m_x[index1][index2];
    }

    double get(int index1, int index2) const {
        return m_x[index1][index2];
    }
 
};

std::ostream & operator << (std::ostream &os, const Mat6x6d &m);

const Mat6x6d zero6x6(0.,0.,0.,0.,0.,0.,
                      0.,0.,0.,0.,0.,0.,
                      0.,0.,0.,0.,0.,0.,
                      0.,0.,0.,0.,0.,0.,
                      0.,0.,0.,0.,0.,0.,
                      0.,0.,0.,0.,0.,0.);
#endif