#ifndef URDF_INTERFACE_CONSTRANT_H
#define URDF_INTERFACE_CONSTRANT_H

#include "link.h"

namespace urdf
{

    class Constraint
    {
    public:
        Constraint() { this->clear(); };

        std::string name;
        enum
        {
            UNKNOWN,
            POSITION,
            ROLLING
        } type;

        /// \brief     type_       meaning of <link>_to_constraint_origin_transform
        /// ------------------------------------------------------
        ///           UNKNOWN     unknown type
        ///           POSITION    Transform from <link> frame to constraint frame. The origins of 
        ///                       the constraint frames as computed via the paths from the nearest 
        ///                       common ancestor through the predecessor and successor must be 
        ///                       coincident.
        ///           ROLLING     Transform from <link> frame to a frame on the rolling contact 
        ///                       surface of <link>. The magnitude of the translation is equal to 
        ///                       the radius of the rolling contact and is therefore used to 
        ///                       determine the reduction ratio of the transmission. 

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
