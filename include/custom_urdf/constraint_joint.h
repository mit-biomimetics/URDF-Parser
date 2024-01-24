#ifndef DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H
#define DYNACORE_URDF_INTERFACE_CONSTRANT_JOINT_H

// TODO(@MatthewChignoli): Make sure to also remove dynacore from all of the guards

#include "link.h"

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
        // TODO(@MatthewChignoli): for sure we should not be using the term origin. Offset is probably better.

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

        std::vector<std::shared_ptr<Link>> allLinks() const
        {
            std::vector<std::shared_ptr<Link>> links;
            links.insert(links.end(), nca_to_parent_subtree.begin(), nca_to_parent_subtree.end());
            links.insert(links.end(), nca_to_child_subtree.begin(), nca_to_child_subtree.end());
            return links;
        }

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

#endif
