#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

class loadParams {

    public:
    
        std::vector<double> focal;
        std::vector<double> principal;
        std::vector<double> dist;
        std::string filename;

        loadParams();
        void loadCameraParams();
};

loadParams::loadParams() :
    filename("./src/mydevice.xml")
    {}

void loadParams::loadCameraParams(){

    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(filename, pt);

    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.FocalLengthRGB")) {

        focal.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
 
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.PrincipalPointRGB")) {

        principal.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
  
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.DistortionRGB")) {

        dist.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
    
}