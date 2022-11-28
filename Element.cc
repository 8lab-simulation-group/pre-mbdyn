#include "Element.h"

Element::Element(int numnode){
    num_node = numnode;
    node_label.resize(numnode);    
}

Element::Element(const Element &elm){
    elem_label = elm.elem_label;
    num_node = elm.num_node;
    node_label = elm.node_label;
    out_flg = elm.out_flg;
}

Element &
Element::operator=(const Element &elm) {
    elem_label = elm.elem_label;
    num_node = elm.num_node;
    node_label = elm.node_label;
    out_flg = elm.out_flg;  
    return *this;  
}