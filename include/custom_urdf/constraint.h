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

        std::string predecessor_link_name;
        std::string successor_link_name;

        /// subtrees rooted at the nearest common ancestor
        ///   the subtrees begin at (but do not include) the nearest common ancestor and end at
        ///   (and do include) the predecessor/successor links
        std::vector<std::shared_ptr<Link>> nca_to_predecessor_subtree, nca_to_successor_subtree;

        // TODO(@MatthewChignoli): This is kind of hacky...
        // TODO(@MatthewChignoli): At the very least we need to add comments
        // Position constraint specific members
        std::shared_ptr<Pose> predecessor_to_constraint_origin_transform;
        std::shared_ptr<Pose> successor_to_constraint_origin_transform;

        // Rolling constraint specific members
        std::shared_ptr<double> ratio;

        // TODO(@MatthewChignoli): add stuff for the differential constraint

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
            this->name.clear();
            this->successor_link_name.clear();
            this->predecessor_link_name.clear();
            this->nca_to_predecessor_subtree.clear();
            this->nca_to_successor_subtree.clear();
            this->predecessor_to_constraint_origin_transform.reset();
            this->successor_to_constraint_origin_transform.reset();
            this->ratio.reset();
            this->type = UNKNOWN;
        };
    };

}

#endif
