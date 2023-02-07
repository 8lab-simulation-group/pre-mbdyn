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
    ReferenceFrame &get_reference() {return *this;};
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

class DummyNode  {
    public:
    int dummynode_label;
    Node  node;
    int reference_label;
    Frame offset;
    int outflag;

    DummyNode(){};
    DummyNode(int label, const Node &node, const Frame &offset, int outflag);
    DummyNode(int label, const Node &node, const ReferenceFrame &reference, int outflag);
    DummyNode(const DummyNode &dnode);
    ~DummyNode(){};

    DummyNode &operator = (const DummyNode &node);
    
    void write_node(std::ofstream &ofs) const ;
    void print_node()const ;
    int get_label() const {return dummynode_label;};    
};





#endif