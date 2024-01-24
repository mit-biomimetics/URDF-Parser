#ifndef URDF_INTERFACE_CONSTRANT_H
#define URDF_INTERFACE_CONSTRANT_H

#include "link.h"

namespace urdf
{

    class Constraint
    {
    public:
        Constraint() { this->clear(); };

        // TODO(@MatthewChignoli): Does rolling make more sense than rotation? Either way, it is important to communicate what the origin transform is for the different types of constraints. Maybe transmission would make more sense.
        std::string name;
        enum
        {
            UNKNOWN,
            POSITION,
            ROTATION
        } type;

        /// predecessor Link element
        ///   origin specifies the transform from predecessor Link to Joint Frame
        std::string predecessor_link_name;
        Pose predecessor_to_constraint_origin_transform;

        /// successor Link element
        ///   origin specifies the transform from the succesor link to the constraint frame
        std::string successor_link_name;
        Pose successor_to_constraint_origin_transform;

        /// subtrees rooted at the nearest common ancestor
        ///   the subtrees begin at (but do not include) the nearest common ancestor and end at 
        ///   (and do include) the predecessor/successor links
        std::vector<std::shared_ptr<Link>> nca_to_predecessor_subtree, nca_to_successor_subtree;

        // TODO(@MatthewChignoli): return const?
        std::vector<std::shared_ptr<Link>> allLinks() const
        {
            std::vector<std::shared_ptr<Link>> links;
            links.insert(links.end(), nca_to_predecessor_subtree.begin(),
                         nca_to_predecessor_subtree.end());
            links.insert(links.end(), nca_to_successor_subtree.begin(),
                         nca_to_successor_subtree.end());
            return links;
        }

        void clear()
        {
            this->successor_link_name.clear();
            this->successor_to_constraint_origin_transform.clear();
            this->predecessor_link_name.clear();
            this->predecessor_to_constraint_origin_transform.clear();
            this->type = UNKNOWN;
        };
    };

}

#endif
