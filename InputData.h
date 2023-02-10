#include <iostream>
#include <map>
#include <vector>

#include "Vec3d.h"

#ifndef __INPUT_DATA_H_INCLUDE_
#define __INPUT_DATA_H_INCLUDE_

class InputData 
{
private: 

    struct i_var {
        int value;
        bool set;
    };
    struct d_var {
        double value;
        bool set;
    };
    struct v_var {
        std::vector<double> value;
        bool set;
    };
    struct s_var {
        std::string value;
        bool set;
    };

    // conteiner whitch store variables
    std::vector<i_var> i_container;
    std::vector<d_var> d_container;
    std::vector<v_var> v_container;
    std::vector<s_var> s_container;


    // binding name of variables and pointer of varialbes
    std::map<std::string, i_var*> i_variables_map;
    std::map<std::string, d_var*> d_variables_map;
    std::map<std::string, v_var*> v_variables_map;
    std::map<std::string, s_var*> s_variables_map ;

    const std::vector<std::string> i_variables_name = {
        "NumBl"       ,
        "TwrNodes"    ,
        "BldNodes"    ,
        "NacelleNodes",

        "PtfmSgDOF"   ,
        "PtfmSwDOF"   ,
        "PtfmHvDOF"   ,
        "PtfmRDOF"    ,
        "PtfmPDOF"    ,
        "PtfmYDOF"    ,

        "NTwInpSt"    ,
        "NTwInpMdSh"  ,
        "NBlInpSt"    , 
        "NBlInpMdSh"  ,

        "CalcTMode"   ,
        "CalcBMode"   ,
        "NPtfmDivH"   ,
        "NPtfmDivV"
    };

    const std::vector<std::string> d_variables_name = {
        "InitialTime"      ,  
        "FinalTime"        ,
        "DT"               ,
        "Gravity"          ,
        "RotSpeed"         ,
        "TipRad"           ,
        "HubRad"           ,
        "OverHang"         ,
        "TowerHt"          ,
        "NacCMxn"          ,
        "NacCMyn"          ,
        "NacCMzn"          ,
        "Twr2Shft"         ,
        "TwrRBHt"          ,
        "ShftTilt"         ,
        "YawBrMass"        ,
        "NacMass"          ,
        "HubMass"          ,
        "RFrlMass"         ,
        "BoomMass"         ,
        "TFinMass"         ,
        "NacYIner"         ,
        "GenIner"          ,
        "HubIner"          ,
        "SmllNmBr"         ,
        "GBoxEff"          ,
        "GenEff"           ,
        "GBRatio"          ,
        "DTTorSpr"         ,
        "DTTorDmp"         ,
        "YawSpr"           ,
        "YawDamp"          ,
        "LSSLength"        ,
        "HSSLength"        ,
        "GenLength"        ,

        "PtfmSurge"        ,
        "PtfmSway"         ,
        "PtfmHeave"        ,
        "PtfmRoll"         ,
        "PtfmPitch"        ,
        "PtfmYaw"          ,
        "TwrDraft"         ,
        "PtfmCM"           ,
        "PtfmRef"          ,
        "PtfmMass"         ,
        "PtfmRIner"        ,
        "PtfmPIner"        ,
        "PtfmYIner"        ,

        "TwrFADmp1"     ,
        "TwrFADmp2"     ,
        "TwrSSDmp1"     ,
        "TwrSSDmp2"     ,
        "FAStTunr1"     ,
        "FAStTunr2"     ,
        "SSStTunr1"     ,
        "SSStTunr2"     ,
        "AdjTwMa"       ,
        "AdjFASt"       ,
        "AdjSSSt"       ,  

        "BldFlDmp1"   ,  
        "BldFlDmp2"   ,  
        "BldEdDmp1"   ,  
        "FlStTunr1"   ,  
        "FlStTunr2"   ,  
        "AdjBlMs"     , 
        "AdjFlSt"     ,  
        "AdjEdSt"     ,

        "WaterDens",
        "Waterdepth",
        "H3",
        "T3",
        "WaveAngle",
        "NWave",
        "Alfa",
        "Beta",
        "CAV",
        "CDV"
    };

    const std::vector<std::string> v_variables_name = {
        "PreCone"         ,
        "HtFract"         ,
        "TMassDen"        ,
        "TwFAStif"        ,
        "TwSSStif"        ,
        "TwGJStif"        ,
        "TwEAStif"        ,
        "TwFAIner"        ,
        "TwSSIner"        ,
        "TwFAcgOf"        ,
        "TwSScgOf"        ,
        "TwFAM1Sh"        ,
        "TwFAM2Sh"        ,
        "TwSSM1Sh"        ,
        "TwSSM2Sh"        ,
        "BlFract"         ,  
        "AeroCent"        ,  
        "StrcTwst"        ,  
        "BMassDen"        ,  
        "FlpStff"         ,  
        "EdgStff"         ,  
        "GJStff"          ,  
        "EAStff"          ,  
        "Alpha"           ,  
        "FlpIner"         ,  
        "EdgIner"         ,  
        "PrecrvRef"       ,  
        "PreswpRef"       ,  
        "FlpcgOf"         ,  
        "EdgcgOf"         ,  
        "FlpEAOf"         ,  
        "EdgEAOf"         , 
        "BldFl1Sh"        ,
        "BldFl2Sh"        ,
        "BldEdgSh"        ,
        "RNodes"          ,
        "AeroTwst"        ,
        "DRNodes"         ,
        "Chord"           ,

        "Seed"            ,
        "CM",
        "CD",
        "EZCOORD",
        "ELENGTH",
        "EDIAMETER",
        "EMASS",
        "FAStif",
        "SSStif",
        "GJStif",
        "EAStif",
        "InerFA",
        "InerSS",
        "InerYaw",
        "MOORING",
        "VZCOORD",
        "VAREA",
        "VVLOME",
        "VNORM",
        "VNODE"
    };

    const std::vector<std::string> s_variables_name = {
        "InpFile",
        "PtfmFile",
        "PtfmLdFile",
        "TwrFile",
        "BldFile"
    };

    const std::string comment_symbol = "#";

private:
    template<class T, typename U>
    inline void initiarize_vector(std::vector<T> &v, U init_value);
    void input(const std::string &input_file);
    int count_words(const std::string &str);

    void convert_file_to_strings(const std::string &file_name, std::vector<std::string> &str);
    bool check_contain(const std::vector<std::string> &list, const std::string &str) ;
    bool has_digit(const std::string &str);

public:
    explicit InputData(const std::string &input_file);
    ~InputData();
    void print_data();
    double get_value(const std::string &arg_name, int index=-1) ;
    std::vector<double> get_vector(const std::string &arg_name);

};


#endif 