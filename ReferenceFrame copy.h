#ifndef __REFERENCEFRAME_H_INCLUDE__
#define __REFERENCEFRAME_H_INCLUDE__

#include "Frame.h"

class ReferenceFrame {
    private:
    Frame base_frame;
    Frame relative_frame;

    Frame reference_frame;

    public:
    ReferenceFrame(){};
    ReferenceFrame(int label, const Frame &baseframe, const Frame &relativeframe);
    ReferenceFrame(int label, const ReferenceFrame &baseframe, const Frame &relative_frame);
    ReferenceFrame(const ReferenceFrame &reffelm);
    ~ReferenceFrame(){};

    void print_reference(const int mode = 0) ;
    void write_reference(std::ofstream &out) const;

    private:
    Frame make_reference_frame(int label,const Frame &baseframe, const Frame &relativeframe);

    public:
    ReferenceFrame &operator=(const ReferenceFrame &refflm);
    Frame get_base_frame() const {return base_frame;};
    Frame get_relative_frame() const {return relative_frame;};
    Frame get_frame() const {return reference_frame;};

};


#endif