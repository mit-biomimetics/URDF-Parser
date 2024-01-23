#ifndef DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H
#define DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H

#include "link.h"

namespace dynacore
{
    namespace urdf
    {

        class ConstraintJoint
        {
        public:
            ConstraintJoint() { this->clear(); };

            std::string name;
            enum
            {
                UNKNOWN,
                POSITION,
                ROTATION
            } type;

            // TODO(@MatthewChignoli): Instead of parent and child, what can we call them? Because there is not directionality...
            // TODO(@MatthewChignoli): Should we just save the names of the links? We can save the links themselves?

            /// child Link element
            ///   child link frame is the same as the Joint frame
            std::string child_link_name;
            /// transform from Child Link frame to Joint frame
            Pose child_to_joint_origin_transform;

            /// parent Link element
            ///   origin specifies the transform from Parent Link to Joint Frame
            std::string parent_link_name;
            /// transform from Parent Link frame to Joint frame
            Pose parent_to_joint_origin_transform;

            // TODO(@MatthewChignoli): Add explanation comments
            std::vector<std::shared_ptr<Link>> nca_to_parent_subtree, nca_to_child_subtree;

            void clear()
            {
                this->child_link_name.clear();
                this->child_to_joint_origin_transform.clear();
                this->parent_link_name.clear();
                this->parent_to_joint_origin_transform.clear();
                this->type = UNKNOWN;
            };
        };

    }
}

#endif
