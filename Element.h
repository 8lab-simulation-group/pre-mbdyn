#ifndef __SECTION_H_INCLUDE__
#define __SECTION_H_INCLUDE__

#include "Node.h"

class Element {
    protected:
    int     elem_label;
    int     num_node;
    std::vector<int> node_label;
    bool    out_flg;

    public:
    Element(){};
    Element(int num_node);
    Element(const Element &elm);
    virtual ~Element(){};

    Element &operator=(const Element &elm);
    virtual void write_in_file(std::ofstream &ost) const {};
    virtual void print_element() const {};
};


#endif