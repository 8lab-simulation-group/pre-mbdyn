#include <string>
#include <sstream>
#include <cctype>
#include <algorithm>

#include "InputData.h"

InputData::InputData(const std::string &input_file) {
    // allocate size of variables 
    i_container.resize(i_variables_name.size());
    d_container.resize(d_variables_name.size());
    v_container.resize(v_variables_name.size());
    s_container.resize(s_variables_name.size());

    // initialize all container
    initiarize_vector(i_container, 0);
    initiarize_vector(d_container, 0.0);
    std::vector<double> v{};
    initiarize_vector(v_container, v);
    std::string str = " ";
    initiarize_vector(s_container, str);

    // map names to varialbe pointers
    for (int i = 0; i < i_variables_name.size(); i++) {
        i_variables_map[i_variables_name[i]] = &i_container[i];
    }
    for (int i = 0; i < d_variables_name.size(); i++) {
        d_variables_map[d_variables_name[i]] = &d_container[i];
    }
    for (int i = 0; i < v_variables_name.size(); i++) {
        v_variables_map[v_variables_name[i]] = &v_container[i];
    }
    for (int i = 0; i < s_variables_name.size(); i++) {
        s_variables_map[s_variables_name[i]] = &s_container[i];
    }

    s_variables_map.at("InpFile")->value = input_file;
    s_variables_map.at("InpFile")->set = true;

    input(input_file);

    if(s_variables_map.at("PtfmFile")->set) {
        input(s_variables_map.at("PtfmFile")->value);
    } else {
        std::cerr<<"platform input file is not setted"<<std::endl;
    }

    if(s_variables_map.at("TwrFile")->set) {
        input(s_variables_map.at("TwrFile")->value);
    } else {
        std::cerr<<"Tower input file is not setted"<<std::endl;
    }

    if(s_variables_map.at("BldFile")->set) {
        input(s_variables_map.at("BldFile")->value);
    } else {
        std::cerr<<"blade input file is not setted"<<std::endl;
    }

    if(s_variables_map.at("PtfmLdFile")->set) {
        input(s_variables_map.at("PtfmLdFile")->value);
    } else {
        std::cerr<<"blade input file is not setted"<<std::endl;
    }
}

InputData::~InputData() {
    // nothing to do
};

template<class T, typename U>
void InputData::initiarize_vector(std::vector<T> &vec, U initial_value) {
    for(T &elem : vec) {
        elem.value = initial_value;
        elem.set = false;
    }
}

void InputData::input(const std::string &input_file) {

    std::vector<std::string> ip_lines;
    convert_file_to_strings(input_file, ip_lines);

    std::vector<std::string> names;

    for(std::string &line : ip_lines) {

        std::stringstream ssr_line(line);
        std::string name;
        int num_line_elem = count_words(line);

        if(num_line_elem == 2) { //it means in line : "name" value
            ssr_line >> name;
        
            // set value to map container
            if(check_contain(i_variables_name, name)) {
                ssr_line >> i_variables_map[name]->value;
                i_variables_map[name]->set = true;

            }else if(check_contain(d_variables_name, name)) {
                ssr_line >> d_variables_map[name]->value;
                d_variables_map[name]->set = true;

            }else if(check_contain(s_variables_name, name)) {
                ssr_line >> s_variables_map[name]->value;
                s_variables_map[name]->set = true;            

            }else if(check_contain(v_variables_name, name)) {
                double t;
                ssr_line >> t;
                v_variables_map[name]->value.push_back(t);
                v_variables_map[name]->set = true; 

            }else {
                std::cerr<<"The variable name '"<<name<<"' in '"<<input_file<<"' is invalid"<<std::endl;
                exit(0);
            }

        }else {      //it means in line : "name"  "name" "name" ... or : "value" "value" "value" ...
                 
            std::vector<std::string> temp;
            names.resize(num_line_elem);
            temp.resize(num_line_elem);

            for(int i=0; i<num_line_elem; i++) {
                ssr_line >> temp[i];
            }

            if(!has_digit(temp[0])) { // it meas in line : "name"  "name" "name" ...
                for(int i = 0; i < num_line_elem; i++) {
                    names[i] = temp[i];
                }

            } else {   // it meas in line :  "value" "value" "value" ...
                for(int i = 0; i < num_line_elem; i++) {
                    if(check_contain(v_variables_name, names[i])) {
                        v_variables_map[names[i]]->value.push_back(stod(temp[i]));
                        v_variables_map[names[i]]->set = true;
                    }else {
                        std::cerr<<"The variable name '"<<name<<"' in '"<<input_file<<"' is invalid"<<std::endl;
                        exit(0);
                    }
                }                
            }
        }
    }
}

void InputData::convert_file_to_strings(const std::string &file_name, std::vector<std::string> &str) {
    // getting data from blade input file
    std::ifstream reading_file;
    reading_file.open(file_name, std::ios::in);

    if(!reading_file.is_open()){
        std::cerr<<"Could not open the file:"<<file_name<<":"<<std::endl;
        exit(0);
    }

    std::string temp_line;
    while(std::getline(reading_file,temp_line))
    {
        std::string::size_type posComment = temp_line.find_first_of(comment_symbol);
        if(posComment!=std::string::npos)
        { 
            temp_line.erase(posComment, temp_line.length());
        }
        
        if(!temp_line.empty())
        {
            str.push_back(temp_line);
        }
    }  
}

int InputData::count_words(const std::string &str) {
    int i=0;
    std::stringstream ssr(str);
    std::string word;

    while(ssr>>word) {
        i++;
    }
    return i;
}

bool InputData::check_contain(const std::vector<std::string> &list, const std::string &str) {
    bool flg = false;
    for(const std::string &elem : list) {
        if (elem == str) {
            flg = true;
        }
    }
    return flg;
}

bool InputData::has_digit(const std::string &str) {
    for (char const &c : str) {
        // function isdigit : if c is 0~9 than return true
        if (std::isdigit(c)) return true;
    }
    return false;
}

double InputData::get_value(const std::string &arg_name, int index) 
{
    if(check_contain(i_variables_name, arg_name)) {
        if(i_variables_map[arg_name]->set) {
            return double(i_variables_map[arg_name]->value);
        } else {
            std::cerr<<"The variable '"<<arg_name<<"' is not taken from input file";
            return -1;
        }

    } else if (check_contain(d_variables_name, arg_name)) {
        if(d_variables_map[arg_name]->set) {
            return double(d_variables_map[arg_name]->value);
        } else {
            std::cerr<<"The variable '"<<arg_name<<"' is not taken from input file";
            return -1;
        }

    } else if (check_contain(v_variables_name, arg_name)) {
        if(v_variables_map[arg_name]->set) {

            if((index <=0)||(index > v_variables_map[arg_name]->value.size())) {
                std::cerr<<"The index is invalid '"<<arg_name<<"' :"<<index<<std::endl
                         <<"The range of '"<<arg_name<<"' is 1~"<<v_variables_map[arg_name]->value.size()<<std::endl;
                return -1;
            } else {
                return double(v_variables_map[arg_name]->value[index-1]);
            }

        } else {
            std::cerr<<"The variable '"<<arg_name<<"' is not taken from input file";
            return -1;
        }
    }
    else {
        std::cerr<<"The variable name "<<arg_name<<"is invalid in func:get_value of class InputData"<<std::endl;
        exit(0);
        return -1;
    }
}


void InputData::print_data() {
    for(const std::string &str : i_variables_name) {
        std::cout<<""<<str
                 <<"\t"<<i_variables_map[str]->value
                 <<"\tset :"<<i_variables_map[str]->set<<std::endl;
    }
    for(const std::string &str : d_variables_name) {
        std::cout<<""<<str
                 <<"\t"<<d_variables_map[str]->value
                 <<"\tset :"<<d_variables_map[str]->set<<std::endl;
    }
    for(const std::string &str : s_variables_name) {
        std::cout<<""<<str
                 <<"\t"<<s_variables_map[str]->value
                 <<"\tset :"<<s_variables_map[str]->set<<std::endl;
    }
    for(const std::string &str : v_variables_name) {
        std::cout<<""<<str
                 <<"\tset :"<<v_variables_map[str]->set
                 <<std::endl;
        for(int i=0; i<v_variables_map[str]->value.size(); i++) {
            std::cout<<""<<i+1<<"\t"<<v_variables_map[str]->value[i]<<std::endl;
        }
    }
}
