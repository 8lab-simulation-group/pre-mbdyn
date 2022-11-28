#ifndef __NODE_H_INCLUDE__
#define __NODE_H_INCLUDE__

#include "ReferenceFrame.h"

class Node : public ReferenceFrame{

    private:
    int node_label;
    int output_flg;

    public:
    Node(){};
    Node(int label, const Frame &baseframe, const Frame &offset, int outflag);
    Node(int label, const ReferenceFrame &baseframe, const Frame &offset, int outflag);
    Node(int label, const ReferenceFrame &frame, int outflag);
    Node(const Node &node);
    ~Node(){};

    Node &operator = (const Node &node);
    
    void write_node(std::ofstream &ofs) const ;
    const ReferenceFrame &get_reference() const {return *this;};
    void print_node();
    int get_label() const {return node_label;};

};

class StaticNode {
    private:
    int node_label;
    int reference_label;
    int output_flag;
    
    public:
    StaticNode(){};
    StaticNode(int label, int reflabel, int outflg);
    StaticNode(const StaticNode &snd);
    ~StaticNode(){};

    StaticNode &operator=(const StaticNode &snd);

    void write_node(std::ofstream &ofs) const;
    int get_label() const {return node_label;};
};






#endif