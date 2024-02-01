
#include <sstream>
#include "custom_urdf/constraint.h"
#include <algorithm>
// #include <console_bridge/console.h>
#include "custom_urdf/tinyxml.h"
#include "custom_urdf/urdf_parser.h"
namespace urdf
{

    bool parseConstraint(Constraint &constraint, TiXmlElement *config)
    {
        constraint.clear();

        // Get Joint Name
        const char *name = config->Attribute("name");
        if (!name)
        {
            printf("unnamed constraint found\n");
            return false;
        }
        constraint.name = name;

        // Get Predecessor Link
        TiXmlElement *predecessor_xml = config->FirstChildElement("predecessor");
        if (predecessor_xml)
        {
            const char *pname = predecessor_xml->Attribute("link");
            if (!pname)
            {
                printf("[Constraint] no predecessor link name\n");
            }
            else
            {
                constraint.predecessor_link_name = std::string(pname);
            }
        }
        else
        {
            printf("[Constraint] no predecessor link\n");
            return false;
        }

        // Get Successor Link
        TiXmlElement *successor_xml = config->FirstChildElement("successor");
        if (successor_xml)
        {
            const char *sname = successor_xml->Attribute("link");
            if (!sname)
            {
                printf("[Constraint] no successor link name\n");
            }
            else
            {
                constraint.successor_link_name = std::string(sname);
            }
        }
        else
        {
            printf("[Constraint] no successor link\n");
            return false;
        }

        // Get Joint type
        const char *type_char = config->Attribute("type");
        if (!type_char)
        {
            return false;
        }

        std::string type_str = type_char;
        if (type_str == "position")
        {
            constraint.type = Constraint::POSITION;
            constraint.predecessor_to_constraint_origin_transform = std::make_shared<Pose>();
            constraint.successor_to_constraint_origin_transform = std::make_shared<Pose>();

            // Get transform from Predecessor Link to Constraint Frame
            TiXmlElement *predecessor_origin_xml = predecessor_xml->FirstChildElement("origin");
            if (!predecessor_origin_xml)
            {
                constraint.predecessor_to_constraint_origin_transform->clear();
            }
            else
            {
                if (!parsePose(*constraint.predecessor_to_constraint_origin_transform,
                               predecessor_origin_xml))
                {
                    constraint.predecessor_to_constraint_origin_transform->clear();
                    return false;
                }
            }

            // Get transform from Successor Link to Joint Frame
            TiXmlElement *successor_origin_xml = successor_xml->FirstChildElement("origin");
            if (!successor_origin_xml)
            {
                constraint.successor_to_constraint_origin_transform->clear();
            }
            else
            {
                if (!parsePose(*constraint.successor_to_constraint_origin_transform,
                               successor_origin_xml))
                {
                    constraint.successor_to_constraint_origin_transform->clear();
                    return false;
                }
            }
        }
        else if (type_str == "rolling")
        {
            constraint.type = Constraint::ROLLING;
            constraint.ratio = std::make_shared<double>();

            // Get ratio of radii of the predecessor and successor rolling contacts
            TiXmlElement *ratio_xml = config->FirstChildElement("ratio");
            if (!ratio_xml)
            {
                constraint.ratio = std::make_shared<double>(1.0);
            }
            else
            {
                try
                {
                    double ratio = std::stod(ratio_xml->Attribute("value"));
                    constraint.ratio = std::make_shared<double>(ratio);
                }
                catch (int e)
                {
                    std::stringstream stm;
                    stm << "Ratio [" << ratio_xml->Attribute("value") << "] is not a float";
                    return false;
                }
            }
        }
        else
        {
            return false;
        }

        return true;
    }

}
