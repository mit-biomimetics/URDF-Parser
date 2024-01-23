
#include <sstream>
#include "custom_urdf/constraint_joint.h"
#include <algorithm>
// #include <console_bridge/console.h>
#include "custom_urdf/tinyxml.h"
#include "custom_urdf/urdf_parser.h"
namespace dynacore
{
    namespace urdf
    {

        bool parseConstraint(ConstraintJoint &constraint, TiXmlElement *config)
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

            // Get transform from Parent Link to Joint Frame
            TiXmlElement *parent_origin_xml = config->FirstChildElement("parent_origin");
            if (!parent_origin_xml)
            {
                constraint.parent_to_joint_origin_transform.clear();
            }
            else
            {
                if (!parsePose(constraint.parent_to_joint_origin_transform, parent_origin_xml))
                {
                    constraint.parent_to_joint_origin_transform.clear();
                    return false;
                }
            }

            // Get Parent Link
            TiXmlElement *parent_xml = config->FirstChildElement("parent");
            if (parent_xml)
            {
                const char *pname = parent_xml->Attribute("link");
                if (!pname)
                {
                    printf("[joint] no parent link name\n");
                }
                else
                {
                    constraint.parent_link_name = std::string(pname);
                }
            }

            // Get transform from Child Link to Joint Frame
            TiXmlElement *child_origin_xml = config->FirstChildElement("child_origin");
            if (!child_origin_xml)
            {
                constraint.child_to_joint_origin_transform.clear();
            }
            else
            {
                if (!parsePose(constraint.child_to_joint_origin_transform, child_origin_xml))
                {
                    constraint.child_to_joint_origin_transform.clear();
                    return false;
                }
            }

            // Get Child Link
            TiXmlElement *child_xml = config->FirstChildElement("child");
            if (child_xml)
            {
                const char *pname = child_xml->Attribute("link");
                if (!pname)
                {
                    printf("[joint] no child link name\n");
                }
                else
                {
                    constraint.child_link_name = std::string(pname);
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
                constraint.type = ConstraintJoint::POSITION;
            else if (type_str == "rotation")
                constraint.type = ConstraintJoint::ROTATION;
            else
            {
                return false;
            }

            // TODO(@MatthewChignoli): This will get deleted
            // Get Joint Axis
            // axis
            TiXmlElement *axis_xml = config->FirstChildElement("axis");
            if (!axis_xml)
            {
                constraint.axis = Vector3(1.0, 0.0, 0.0);
            }
            else
            {
                if (axis_xml->Attribute("xyz"))
                {
                    try
                    {
                        constraint.axis.init(axis_xml->Attribute("xyz"));
                    }
                    catch (ParseError &e)
                    {
                        constraint.axis.clear();
                        return false;
                    }
                }
            }

            return true;
        }

    }
}
