#include "Nacelle.h"
#include <cmath>
#include <math.h>

Nacelle::Nacelle(int label, const Node &tower_top, InputData *ID) 
: nacelle_label(label), tower_top_node(tower_top), inputdata(ID) // const メンバ変数を引数で初期化
{
    num_blds = inputdata->get_value("NumBl");
    num_nodes = 5 + num_blds; // (nacell + HSS + LSS + Generator + Hub) + pitch_bottom
    num_rigidbodies = num_nodes;

    HSS_label       = nacelle_label + 100;
    LSS_label       = nacelle_label + 200;
    Generator_label = nacelle_label + 300;
    Hub_label       = nacelle_label + 400;

    HSS_length = inputdata->get_value("HSSLength");
    LSS_length = inputdata->get_value("LSSLength");
    Generator_length = inputdata->get_value("GenLength");
    Overhang = inputdata->get_value("OverHang");
    shaft_tilt_angle = inputdata->get_value("ShftTilt")*M_PI/180.0;
    tower_to_shaft = inputdata->get_value("Twr2Shft");
    nacelle_CMx = inputdata->get_value("NacCMxn");
    nacelle_CMz = inputdata->get_value("NacCMzn");
    preconed_angle = inputdata->get_value("PreCone",1)*M_PI/180.0;
    Hub_radius = inputdata->get_value("HubRad");

    small_number = inputdata->get_value("SmllNmBr");

    nacelle_mass = inputdata->get_value("NacMass") + small_number;
    HSS_mass = small_number;
    LSS_mass = small_number;
    Generator_mass = small_number;
    Hub_mass = inputdata->get_value("HubMass");

    dtrain_spring_constant = inputdata->get_value("DTTorSpr")*1000; // convert kN/rad -> N/rad
    dtrain_dumping_constant = inputdata->get_value("DTTorDmp")*1000;
    init_rot_speed = inputdata->get_value("RotSpeed")*2*M_PI/60;


    double nacelle_yaw_inertia = inputdata->get_value("NacYIner");
    double nacelle_local_z_inertia = nacelle_yaw_inertia - nacelle_mass*( pow(nacelle_CMx,2) + pow(nacelle_CMz,2) );
    double generator_y_inertia = inputdata->get_value("GenIner");
    double gearbox_ratio = inputdata->get_value("GBRatio");

    nacelle_inertia = Vec3d(small_number, small_number, nacelle_local_z_inertia);
    HSS_inertia = Vec3d(small_number, small_number, small_number);
    LSS_inertia = Vec3d(small_number, small_number, small_number);
    Generator_inertia = Vec3d(generator_y_inertia*pow(gearbox_ratio, 2) , small_number, small_number);
    Hub_inertia = Vec3d(inputdata->get_value("HubIner"), small_number, small_number);

    Pitch_Bottom.resize(num_blds);
    Pitch_bottom_total.resize(num_blds);

    set_reference();
    set_nodes();
    set_rigidbodies();
    set_joints();
}

void
Nacelle::set_reference() {
    
    // nacelle ----------------------------------------------------------------------------------
    // make nacelle referece
    ReferenceFrame towertop_reference = tower_top_node.get_reference();
    nacelle_reference = ReferenceFrame(nacelle_label, towertop_reference, offset_null); 
    // nacelle coordinate system is coincident tower top system

    // nacelle CM system
    // vector form towertop to nacelle in global coordinate
    Vec3d  towertop_to_nacelleCM(nacelle_CMx, 0 ,nacelle_CMz );
    Frame offset_towertop_nacelleCM(0,towertop_to_nacelleCM,eye3x3, zero3,zero3);

    nacelle_CM_reference = ReferenceFrame(nacelle_label+5, nacelle_reference, offset_towertop_nacelleCM);

    // yaw bearing
    yaw_bearing_label = nacelle_label + 10;
    yaw_bearing_reference = ReferenceFrame(yaw_bearing_label, nacelle_reference, offset_null);

    // HSS coordinate system ---------------------------------------------------------------------
    Vec3d towertop_to_HSS(0.,0.,tower_to_shaft);

    // rotate shaft_tilt_angle about tower_top y-axis
    Vec3d e1( std::cos(shaft_tilt_angle), 0.,std::sin(shaft_tilt_angle));
    Vec3d e3(-std::sin(shaft_tilt_angle), 0.,std::cos(shaft_tilt_angle));

    Frame offset_towertop_to_HSS(0.,towertop_to_HSS, e1,e3,zero3,zero3);
    HSS_reference = ReferenceFrame(HSS_label, towertop_reference, offset_towertop_to_HSS);
    
    double distance_HSS_CM = Overhang + LSS_length + 0.5 * HSS_length;
    Frame offset_HSS_CM(0.,Vec3d(distance_HSS_CM,0.,0.), eye3x3, zero3, zero3);

    HSS_CM_reference = ReferenceFrame(HSS_label + 5, HSS_reference, offset_HSS_CM);

    // HSS Bearing
    HSS_bearing_label = HSS_label+10;
    e1 = Vec3d(0.,0.,1.);
    e3 = Vec3d(1.,0.,0.);
    HSS_bearing_reference = ReferenceFrame(HSS_bearing_label, HSS_reference, Frame(0.,zero3, e1,e3,zero3,zero3));

    // HSS-Genearator
    HSS_Generator_label = HSS_label+20;
    HSS_Generator_reference = ReferenceFrame(HSS_Generator_label, HSS_reference, offset_null);

    // LSS -----------------------------------------------------------------------------------------
    // the lss coordinate systtem is coincident hss coordinate
    LSS_reference = ReferenceFrame(LSS_label, HSS_reference, offset_null);     

    double distance_LSS_CM = Overhang + 0.5 * LSS_length;
    Frame offset_LSS_CM(0., Vec3d(distance_LSS_CM, 0., 0.), eye3x3, zero3, zero3);
    LSS_CM_reference = ReferenceFrame(LSS_label + 5, LSS_reference, offset_LSS_CM);

    // HSS-LSS bearing
    HSS_LSS_bearing_label = LSS_label + 10;
    e1 = Vec3d(0.,0.,1.);
    e3 = Vec3d(1.,0.,0.);
    Frame offset_HSS_LSS_Bearing(0., Vec3d(distance_LSS_CM, 0., 0.), e1,e3, zero3, zero3);
    HSS_LSS_bearing_reference = ReferenceFrame(HSS_LSS_bearing_label, HSS_reference,offset_HSS_LSS_Bearing);

    // Generator ----------------------------------------------------------------------------------
    // the generator coordinate system is also coincident HSS coordinate
    Generator_reference = ReferenceFrame(Generator_label, HSS_reference, offset_null);

    double distance_Gen_CM = Overhang + LSS_length + HSS_length + 0.5 * Generator_length;
    Frame offset_Gen_CM(0., Vec3d(distance_Gen_CM, 0., 0.), eye3x3, zero3, zero3);
    Generator_CM_reference = ReferenceFrame( Generator_label + 5, Generator_reference, offset_Gen_CM);

    // hub ----------------------------------------------------------------------------------------
    // Hub is the position of overhang in the x-axis direction in the HSS coordinate system
    Vec3d offset_HSS_to_hub(Overhang, 0.,0.);
    Hub_reference = ReferenceFrame(Hub_label, HSS_reference, Frame(0.,offset_HSS_to_hub,eye3x3,zero3,zero3));
    Hub_CM_reference = ReferenceFrame( Hub_label+5, Hub_reference, offset_null);

    // LSS-Hub joint
    LSS_Hub_label = Hub_label +1 ;
    LSS_Hub_reference = ReferenceFrame(LSS_Hub_label, Hub_reference, offset_null);


    // pitch bearign bottom----------------
    for(int i = 0; i < num_blds; i++) {
        Pitch_Bottom[i].label = Hub_label + 100*(i+1);

        double azim_angle = 2*M_PI/num_blds * i;
        e1 = Vec3d(1.,0.,0.);
        e3 = Vec3d(0.,-std::sin(azim_angle), std::cos(azim_angle));
        // coordinate in line shaft axis
        Pitch_Bottom[i].inline_shaft_reference = ReferenceFrame(Hub_label + 10 + (i+1) , Hub_reference, Frame(0.,zero3, e1,e3,zero3,zero3));
        
        // preconed coordinate system
        e1 = Vec3d(std::cos(preconed_angle), 0., -std::sin(preconed_angle));
        e3 = Vec3d(std::sin(preconed_angle), 0.,  std::cos(preconed_angle));

        Pitch_Bottom[i].preconed_reference = ReferenceFrame(Hub_label + 20 + (i+1), Pitch_Bottom[i].inline_shaft_reference, Frame(0.,zero3,e1,e3,zero3,zero3));

        Pitch_Bottom[i].reference = ReferenceFrame(Pitch_Bottom[i].label, Pitch_Bottom[i].preconed_reference, Frame(0.,Vec3d(0.,0.,Hub_radius),eye3x3,zero3,zero3));
    } 
}

void
Nacelle::set_nodes(){
    //nacelle
    nacelle_node = Node(nacelle_label, nacelle_reference, offset_null, 0);
    //HSS
    HSS_node = Node(HSS_label, HSS_reference, offset_null, 0);
    //LSS
    LSS_node = Node(LSS_label, LSS_reference, offset_null, 0);
    //generator
    Generator_node = Node(Generator_label, Generator_reference, offset_null, 0);
    //hub
    Hub_node = Node(Hub_label, Hub_reference, offset_null, 0);
    // pitch bearing bottom
    for(int i=0; i<num_blds; i++) {
        Pitch_Bottom[i].node = Node(Pitch_Bottom[i].label, Pitch_Bottom[i].reference, offset_null, 0);
    }

    Vec3d e1(0.,0.,1.);
    Vec3d e3(1.,0.,0.);
    ShaftCS = DummyNode(nacelle_label +10, nacelle_node, HSS_reference, 0);
    ReferenceFrame nhr(0.,HSS_reference, Frame(0,zero3, e1, e3,zero3, zero3));
    NacelleHubRef = DummyNode(nacelle_label + 20, nacelle_node, nhr,0);
    ReferenceFrame rotazim(0., LSS_reference, Frame(0,zero3, e1, e3,zero3, zero3));
    ReferenceFrame genazim(0., HSS_reference, Frame(0,zero3, e1, e3,zero3, zero3));
    RotAzim = DummyNode(LSS_label + 10, LSS_node, rotazim, 0 );
    GenAzim = DummyNode(HSS_label + 10, HSS_node, genazim, 0 );
}
void
Nacelle::set_rigidbodies() {

    //nacelle
    nacelle_rigidbody = RigidBody(nacelle_label, nacelle_node, nacelle_mass, nacelle_CM_reference, nacelle_inertia, 0);
    //HSS
    HSS_rigidbody = RigidBody(HSS_label, HSS_node, HSS_mass, HSS_CM_reference, HSS_inertia, 0);
    //LSS
    LSS_rigidbody = RigidBody(LSS_label, LSS_node, LSS_mass, LSS_CM_reference, LSS_inertia, 0);
    //generator
    Generator_rigidbody = RigidBody(Generator_label, Generator_node, Generator_mass, Generator_CM_reference, Generator_inertia, 0);
    //hub
    Hub_rigidbody = RigidBody(Hub_label, Hub_node, Hub_mass, Hub_CM_reference, Hub_inertia, 0);
    // pitch bearing bottom
    for(int i=0; i<num_blds; i++) {
        double temp_mass = small_number;
        Vec3d temp_inertia(small_number, small_number, small_number);
        Pitch_Bottom[i].rigidbody = RigidBody(Pitch_Bottom[i].label, Pitch_Bottom[i].node, temp_mass, zero3, temp_inertia, 0);
    }

}

void
Nacelle::set_joints() {
    // yaw bearing
    double init_theta = 0.0;
    Node node1 = tower_top_node;
    Node node2 = nacelle_node;
    yaw_bearing_revolute = RevoluteJoint(yaw_bearing_label,node1,node2,yaw_bearing_reference,init_theta,0 );
    yaw_bearing_total = TotalJoint(yaw_bearing_label+10, node1, node2, yaw_bearing_reference, "BearingLock", 0);

    // High Speed Shaft Bearing
    init_theta = M_PI;
    node1 = HSS_node;
    node2 = nacelle_node;
    HSS_bearing_revolute = RevoluteJoint(HSS_bearing_label, node1, node2, HSS_bearing_reference, init_theta, 0);
    Vec3d angular_velocity(0.,0.,init_rot_speed);
    HSS_bearing_lock = TotalJoint(HSS_bearing_label+1, node1, node2, HSS_bearing_reference, "Angular Velocity",0,angular_velocity);

    // HSS-Generator
    node1 = HSS_node;
    node2 = Generator_node;
    HSS_Generator_total = TotalJoint(HSS_Generator_label, node1, node2, HSS_Generator_reference, "Total",0);

    // HSS-LSS bearing
    init_theta = 0.0;
    node1 = HSS_node;
    node2 = LSS_node;
    HSS_LSS_bearing_revolute = RevoluteJoint(HSS_LSS_bearing_label, node1, node2, HSS_LSS_bearing_reference, init_theta, 0);
    drivetrain_defomablehinge = DeformableHinge(HSS_LSS_bearing_label+1, node1, node2, HSS_LSS_bearing_reference, dtrain_spring_constant, dtrain_dumping_constant,0);
    HSS_LSS_bearing_lock = TotalJoint(HSS_LSS_bearing_label+2, node1, node2, HSS_LSS_bearing_reference, "Angular Velocity", 0, zero3);

    // LSS-Hub joint
    node1 = LSS_node;
    node2 = Hub_node;
    LSS_Hub_total = TotalJoint(LSS_Hub_label, node1, node2, LSS_Hub_reference, "Total",0);    

    // pitch bottom joint
    for(int i=0; i<num_blds;i++) {
        node1 = Hub_node;
        node2 = Pitch_Bottom[i].node;
        int label = Pitch_Bottom[i].label;
        ReferenceFrame positon = node2.get_reference();
        Pitch_bottom_total[i].total = TotalJoint(label, node1, node2, positon,"Total",0);        
    }
    num_joints = 9 + num_blds;
}

void
Nacelle::write_reference_in(std::ofstream &output_file) const {
    output_file<<"#-----Nacelle------"<<std::endl;
    output_file<<"# Nacelle reference"<<std::endl;
    nacelle_reference.write_reference(output_file);

    output_file<<"# Nacelle CM reference"<<std::endl;
    nacelle_CM_reference.write_reference(output_file);   
    
    output_file<<"# yaw_bearing_reference"<<std::endl;
    yaw_bearing_reference.write_reference(output_file); 

    output_file<<"#-----HSS----------"<<std::endl;
    output_file<<"# HSS reference"<<std::endl;
    HSS_reference.write_reference(output_file);

    output_file<<"# HSS CM reference"<<std::endl;
    HSS_CM_reference.write_reference(output_file);

    output_file<<"# HSS bearing reference"<<std::endl;
    HSS_bearing_reference.write_reference(output_file);

    output_file<<"# HSS_Generator joint reference"<<std::endl;
    HSS_Generator_reference.write_reference(output_file);   

    output_file<<"#-----LSS----------"<<std::endl;
    output_file<<"# LSS reference"<<std::endl;
    LSS_reference.write_reference(output_file);

    output_file<<"# LSS CM reference"<<std::endl;
    LSS_CM_reference.write_reference(output_file);

    output_file<<"# HSS-LSS Bearing reference"<<std::endl;
    HSS_LSS_bearing_reference.write_reference(output_file);


    output_file<<"#-----Generator----------"<<std::endl;
    output_file<<"# Generator reference"<<std::endl;
    Generator_reference.write_reference(output_file);
    output_file<<"# Generator CM reference"<<std::endl;
    Generator_CM_reference.write_reference(output_file);

    output_file<<"#-----Hub----------"<<std::endl;
    output_file<<"# Hub reference"<<std::endl;
    Hub_reference.write_reference(output_file);
    output_file<<"# Hub CM reference"<<std::endl;
    Hub_CM_reference.write_reference(output_file);

    output_file<<"# LSS-Hub reference"<<std::endl;
    LSS_Hub_reference.write_reference(output_file);

    for(int i=0; i<num_blds; i++) {
        output_file<<"#-----PitchPlate "<<i+1<< "Reference------"<<std::endl;
        output_file<<"# in-line shaft axis reference"<<std::endl;
        Pitch_Bottom[i].inline_shaft_reference.write_reference(output_file);

        output_file<<"# coned reference"<<std::endl;
        Pitch_Bottom[i].preconed_reference.write_reference(output_file);

        output_file<<"# Pitch Bearing Bottom "<<i+1<<std::endl;
        Pitch_Bottom[i].reference.write_reference(output_file);

    }
}

void
Nacelle::write_nodes_in(std::ofstream &output_file) const {
    output_file<<"#-----Nacelle------"<<std::endl;

    output_file<<"# Nacelle node"<<std::endl;
    nacelle_node.write_node(output_file);   

    output_file<<"#-----HSS----------"<<std::endl;
    output_file<<"# HSS node"<<std::endl;
    HSS_node.write_node(output_file);

    output_file<<"#-----LSS----------"<<std::endl;
    output_file<<"# LSS node"<<std::endl;
    LSS_node.write_node(output_file);

    output_file<<"#-----Generator----------"<<std::endl;
    output_file<<"# Generator node"<<std::endl;
    Generator_node.write_node(output_file);

    output_file<<"#-----Hub----------"<<std::endl;
    output_file<<"# Hub node"<<std::endl;
    Hub_node.write_node(output_file);

    for(int i=0; i<num_blds; i++) {
        output_file<<"#-----PitchPlate "<<i+1<< " node------"<<std::endl;
        Pitch_Bottom[i].node.write_node(output_file);
    }
    output_file<<"#-----Nacelle Dummy Node----------"<<std::endl;
    output_file<<"# ShaftSC"<<std::endl;
    ShaftCS.write_node(output_file);

    output_file<<"# NacelleHubRef"<<std::endl;
    NacelleHubRef.write_node(output_file);

    output_file<<"# RotoAzim"<<std::endl;
    RotAzim.write_node(output_file);

    output_file<<"# GenAzim" <<std::endl;
    GenAzim.write_node(output_file);
}   

void
Nacelle::write_rigidbodies_in(std::ofstream &output_file) const {
    output_file<<"#-----Nacelle------"<<std::endl;

    output_file<<"# Nacelle rigidbody"<<std::endl;
    nacelle_rigidbody.write_in_file(output_file);   

    output_file<<"#-----HSS----------"<<std::endl;
    output_file<<"# HSS rigidbody"<<std::endl;
    HSS_rigidbody.write_in_file(output_file);

    output_file<<"#-----LSS----------"<<std::endl;
    output_file<<"# LSS rigidbody"<<std::endl;
    LSS_rigidbody.write_in_file(output_file);

    output_file<<"#-----Generator----------"<<std::endl;
    output_file<<"# Generator rigidbody"<<std::endl;
    Generator_rigidbody.write_in_file(output_file);

    output_file<<"#-----Hub----------"<<std::endl;
    output_file<<"# Hub rigidbody"<<std::endl;
    Hub_rigidbody.write_in_file(output_file);

    for(int i=0; i<num_blds; i++) {
        output_file<<"#-----PitchPlate "<<i+1<< " node------"<<std::endl;
        Pitch_Bottom[i].rigidbody.write_in_file(output_file);
    }
}

void 
Nacelle::write_joints_in(std::ofstream &output_file) const {
    output_file<<"#----- yaw bearing ------"<<std::endl;
    output_file<<"# revolute"<<std::endl;
    yaw_bearing_revolute.write_in_file(output_file);

    output_file<<"# total"<<std::endl;
    yaw_bearing_total.write_in_file(output_file);

    output_file<<"#----- HSS bearing ------"<<std::endl;
    HSS_bearing_revolute.write_in_file(output_file);

    output_file<<"#initial lock"<<std::endl;
    output_file << "driven :"<<HSS_bearing_lock.get_label()<<", " <<"string, \"Time<=InitRotSpdContTime\""<< ", " << std::endl;
    HSS_bearing_lock.write_in_file(output_file);

    output_file<<"#----- HSS-Generator joint ------"<<std::endl;
    HSS_Generator_total.write_in_file(output_file);

    output_file<<"#----- HSS-LSS bearing ------"<<std::endl;
    HSS_LSS_bearing_revolute.write_in_file(output_file);
    output_file<<"#draive train"<<std::endl;
    drivetrain_defomablehinge.write_in_file(output_file);

    output_file<<"#initial lock"<<std::endl;
    output_file << "driven :"<<HSS_LSS_bearing_lock.get_label()<<", " <<"string, \"Time<=LSSHSSContTime\""<< ", " << std::endl;
    HSS_LSS_bearing_lock.write_in_file(output_file);

    output_file<<"#-----LSS-Hub joint ------"<<std::endl;
    LSS_Hub_total.write_in_file(output_file);    

    // pitch bottom joint
    for(int i=0; i<num_blds;i++) {
        output_file<<"#--------Hub - PitchBottom"<<i+1<<std::endl;
        Pitch_bottom_total[i].total.write_in_file(output_file);        
    }

};

int 
Nacelle::get_num_nodes() const {
    // nacelle nodes + dummynode
    return num_nodes + 4 ;
}

int 
Nacelle::get_num_rigid_bodies() const {
    return num_rigidbodies;
}

int
Nacelle::get_num_joints() const {
    // Nacelle joints 
    return num_joints ;
}

const ReferenceFrame 
Nacelle::get_top_reference(int i) const 
{
    return Pitch_Bottom[i-1].reference;
};