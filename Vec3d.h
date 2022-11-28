
#ifndef __VEC3D_H_INCLUDE__
#define __VEC3D_H_INCLUDE__

#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <vector>
#include <random>

class Vec3d{
private:
    double m_x[3];
public:
    Vec3d(){}
    ~Vec3d(){}

    Vec3d(double x, double y, double z) {
        m_x[0] = x;
        m_x[1] = y;
        m_x[2] = z;   
    }

    Vec3d(double *x) {
        m_x[0] = x[0];
        m_x[1] = x[1];
        m_x[2] = x[2];   
    }

    Vec3d(const double *x) {
        m_x[0] = x[0];
        m_x[1] = x[1];
        m_x[2] = x[2];   
    }

    double &operator [](int index) {
        return m_x[index];
    }

    double const &operator [](int index) const  {
        return m_x[index];
    }

    Vec3d(const Vec3d &v) {
        m_x[0] = v[0];
        m_x[1] = v[1];
        m_x[2] = v[2];
    }
    double dget(int i) const{
        return m_x[i];
    }

    Vec3d &operator =(const Vec3d &v) {
        m_x[0] = v[0];
        m_x[1] = v[1];
        m_x[2] = v[2];
        return *this;
    }

    Vec3d &operator +=(const Vec3d &v) {
        m_x[0] += v[0];
        m_x[1] += v[1];
        m_x[2] += v[2];
        return *this;        
    }

    Vec3d &operator -=(const Vec3d &v) {
        m_x[0] -= v[0];
        m_x[1] -= v[1];
        m_x[2] -= v[2];
        return *this;        
    }

    Vec3d &operator *=(const double &d) {
        m_x[0] *=d;
        m_x[1] *=d;
        m_x[2] *=d;
        return *this;        
    }

    Vec3d operator + () {
        return *this;
    }

    Vec3d operator + (const Vec3d &v) {
        return Vec3d(this->m_x[0]+v[0],this->m_x[1]+v[1],this->m_x[2]+v[2]);
    }

    Vec3d operator + (const double &d) {
        Vec3d result(this->m_x[0]+d, this->m_x[1]+d,this->m_x[2]+d);
        return result;
    }

    Vec3d operator - () {
        return Vec3d(-m_x[0],-m_x[1],-m_x[2]);
    }

    Vec3d operator - (const Vec3d &v) {
        Vec3d result(this->m_x[0]-v[0],this->m_x[1]-v[1],this->m_x[2]-v[2]);
        return result;        
    }

    Vec3d operator - (const double &d) {
        Vec3d result(this->m_x[0]-d,this->m_x[1]-d,this->m_x[2]-d);
        return result;        
    }

    Vec3d operator * (const Vec3d &v) const {
        Vec3d result( this->m_x[1]*v[2] - this->m_x[2]*v[1],
                      this->m_x[2]*v[0] - this->m_x[0]*v[2],
                      this->m_x[0]*v[1] - this->m_x[1]*v[0]);
        return result;
    }

    Vec3d operator * (const double &d) const {
        Vec3d result(this->m_x[0]*d, this->m_x[1]*d,this->m_x[2]*d);
        return result;
    }

    Vec3d operator / (const double &d) const {
        Vec3d result(this->m_x[0]/d, this->m_x[1]/d,this->m_x[2]/d);
        return result;
    }

    double dot(const Vec3d &v) {
        return this->m_x[0]*v[0] +this->m_x[1]*v[1] +this->m_x[2]*v[2] ;
    }

    double dot(const Vec3d &v) const {
        return this->m_x[0]*v[0] +this->m_x[1]*v[1] +this->m_x[2]*v[2] ;
    }

    double norm() const {
        return std::sqrt(m_x[0]*m_x[0] + m_x[1]*m_x[1] + m_x[2]*m_x[2]);
    }

    Vec3d cross(const Vec3d &v) const {
        Vec3d result( this->m_x[1]*v[2] - this->m_x[2]*v[1],
                      this->m_x[2]*v[0] - this->m_x[0]*v[2],
                      this->m_x[0]*v[1] - this->m_x[1]*v[0]);
        return result;        
    }

    void copy(double *d, int i){
        d[i  ] = this->m_x[0];
        d[i+1] = this->m_x[1];
        d[i+2] = this->m_x[2];
    }

    void print() {
        std::cout<<" "<<m_x[0]<<" "<<m_x[1]<<" "<<m_x[2]<<std::endl;
    }
};

std::ostream & operator << (std::ostream &os, const Vec3d &v);

const Vec3d zero3(0.,0.,0.);
const Vec3d localX(1.,0.,0.);
const Vec3d localY(0.,1.,0.);
const Vec3d localZ(0.,0.,1.);
const Vec3d One3(1.,1.,1);


#endif