#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <vector>
#include <random>

#include "Vec3d.h"

#ifndef __MAT3X3D_H_INCLUDE__
#define __MAT3X3D_H_INCLUDE__


class Mat3x3d{
private:
    double m_x[3][3];
public:
    Mat3x3d(){};
    ~Mat3x3d(){};
    Mat3x3d(const double *x) // x|0|1|2|3|4|... -> m_x|00|01|02|10|11|12|...
    {
        for(int i=0; i<3;i++)
        {
            for(int j=0; j<3; j++)
            {
                m_x[i][j] = x[i*3 + j];
            }
        }
    }
    Mat3x3d(const double x[][3])
    {
        for(int i=0; i<3;i++)
        {
            for(int j=0; j<3; j++)
            {
                m_x[i][j] = x[i][j];
            }
        }
    }
    Mat3x3d(const double &d11,const double &d12,const double &d13,
            const double &d21,const double &d22,const double &d23,
            const double &d31,const double &d32,const double &d33) {
                m_x[0][0] = d11;
                m_x[0][1] = d12;
                m_x[0][2] = d13;
                m_x[1][0] = d21;
                m_x[1][1] = d22;
                m_x[1][2] = d23;
                m_x[2][0] = d31;
                m_x[2][1] = d32;
                m_x[2][2] = d33;
            }

    double &operator [](int index)
    {
        return m_x[index/int(3)][index%int(3)];
    };
    double const &operator [](int index) const 
    {
        return m_x[index/int(3)][index%int(3)];
    };
    Vec3d operator * (const Vec3d &v) const 
    {
        Vec3d result(m_x[0][0]*v[0]+m_x[0][1]*v[1]+m_x[0][2]*v[2],
                     m_x[1][0]*v[0]+m_x[1][1]*v[1]+m_x[1][2]*v[2],
                     m_x[2][0]*v[0]+m_x[2][1]*v[1]+m_x[2][2]*v[2]);
        return result;
    };
    Mat3x3d operator * (const Mat3x3d &m) const
    {
        return Mat3x3d(m_x[0][0]*m[0] + m_x[0][1]*m[3] + m_x[0][2]*m[6],
                       m_x[0][0]*m[1] + m_x[0][1]*m[4] + m_x[0][2]*m[7],
                       m_x[0][0]*m[2] + m_x[0][1]*m[5] + m_x[0][2]*m[8],
                       
                       m_x[1][0]*m[0] + m_x[1][1]*m[3] + m_x[1][2]*m[6],
                       m_x[1][0]*m[1] + m_x[1][1]*m[4] + m_x[1][2]*m[7],
                       m_x[1][0]*m[2] + m_x[1][1]*m[5] + m_x[1][2]*m[8],
                       
                       m_x[2][0]*m[0] + m_x[2][1]*m[3] + m_x[2][2]*m[6],
                       m_x[2][0]*m[1] + m_x[2][1]*m[4] + m_x[2][2]*m[7],
                       m_x[2][0]*m[2] + m_x[2][1]*m[5] + m_x[2][2]*m[8]);
    }
    double dGet (int i, int j) const{
        return m_x[i][j];
    }
    Mat3x3d T() const {
        return Mat3x3d (m_x[0][0], m_x[1][0], m_x[2][0], 
                        m_x[0][1], m_x[1][1], m_x[2][1],
                        m_x[0][2], m_x[1][2], m_x[2][2] );
    }

    Vec3d get_row(int index) const {
        return Vec3d(m_x[index][0], m_x[index][1], m_x[index][2]);
    }

    Vec3d get_colum(int index) const{
        return Vec3d(m_x[0][index], m_x[1][index], m_x[2][index]);
    }
};

std::ostream & operator << (std::ostream &os, const Mat3x3d &m);

const Mat3x3d zero3x3(0.,0.,0.,0.,0.,0.,0.,0.,0.);
const Mat3x3d eye3x3(1.,0.,0.,0.,1.,0.,0.,0.,1.);
const Mat3x3d one3x3(1.,1.,1.,1.,1.,1.,1.,1.,1.);

#endif