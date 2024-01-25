
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

        // Get transform from Predecessor Link to Constraint Frame
        TiXmlElement *predecessor_origin_xml = config->FirstChildElement("predecessor_origin");
        if (!predecessor_origin_xml)
        {
            constraint.predecessor_to_constraint_origin_transform.clear();
        }
        else
        {
            if (!parsePose(constraint.predecessor_to_constraint_origin_transform,
                           predecessor_origin_xml))
            {
                constraint.predecessor_to_constraint_origin_transform.clear();
                return false;
            }
        }

        // Get Predecessor Link
        TiXmlElement *predecessor_xml = config->FirstChildElement("predecessor");
        if (predecessor_xml)
        {
            const char *pname = predecessor_xml->Attribute("link");
            if (!pname)
            {
                printf("[joint] no predecessor link name\n");
            }
            else
            {
                constraint.predecessor_link_name = std::string(pname);
            }
        }

        // Get transform from Successor Link to Joint Frame
        TiXmlElement *successor_origin_xml = config->FirstChildElement("successor_origin");
        if (!successor_origin_xml)
        {
            constraint.successor_to_constraint_origin_transform.clear();
        }
        else
        {
            if (!parsePose(constraint.successor_to_constraint_origin_transform,
                           successor_origin_xml))
            {
                constraint.successor_to_constraint_origin_transform.clear();
                return false;
            }
        }

        // Get Successor Link
        TiXmlElement *successor_xml = config->FirstChildElement("successor");
        if (successor_xml)
        {
            const char *sname = successor_xml->Attribute("link");
            if (!sname)
            {
                printf("[joint] no successor link name\n");
            }
            else
            {
                constraint.successor_link_name = std::string(sname);
            }
        }

        // Get Joint type
        const char *type_char = config->Attribute("type");
        if (!type_char)
        {
            return false;
        }

        std::string type_str = type_char;
        if (type_str == "position")
            constraint.type = Constraint::POSITION;
        else if (type_str == "rolling")
            constraint.type = Constraint::ROLLING;
        else
        {
            return false;
        }

        return true;
    }

}
