#ifndef __REFERENCEFRAME_H_INCLUDE__
#define __REFERENCEFRAME_H_INCLUDE__

#include "Frame.h"

class ReferenceFrame : public Frame{
    protected:
    Frame m_base_frame;
    Frame m_relative_frame;

    public:
    ReferenceFrame(){};
    ReferenceFrame(int label, const Frame &base_frame, const Frame &relative_frame);
    ReferenceFrame(int label, const ReferenceFrame &base_frame, const Frame &relative_frame);
    ReferenceFrame(const ReferenceFrame &reffelm);
    ~ReferenceFrame(){};

    void print_reference() ;
    void write_reference(std::ofstream &out) const;

    private:
    Frame make_reference_frame(int label,const Frame &base_frame, const Frame &relative_frame);

    public:
    ReferenceFrame &operator=(const ReferenceFrame &refflm);
    Frame get_base_frame() const {return m_base_frame;};
    Frame get_relative_frame() const {return m_relative_frame;};
    const Frame &get_frame() const {return *this;};

};


#endif