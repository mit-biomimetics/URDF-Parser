
// TODO(@MatthewChignoli): How much should this share with the joint class?

#ifndef DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H
#define DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H

// #include "joint.h"
// #include "pose.h"
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
                REVOLUTE,
                CONTINUOUS,
                PRISMATIC,
                PLANAR,
                ROLLING,
                FIXED
            } type;

            // TODO(@MatthewChignoli): Does ROLLING need more info than we currently have?
            /// \brief     type_       meaning of axis_
            /// ------------------------------------------------------
            ///            UNKNOWN     unknown type
            ///            REVOLUTE    rotation axis
            ///            PRISMATIC   translation axis
            ///            PLANAR      plane normal axis
            ///            ROLLING     rolling axis
            ///            FIXED       N/A
            Vector3 axis;

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

            // TODO(@MatthewChignoli): Better naming? Should this even be here?
            // I think this will create a circular dependency with the Link class?
            std::vector<std::shared_ptr<Link>> nca_to_parent_subtree, nca_to_child_subtree;

            void clear()
            {
                this->axis.clear();
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
