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

        // TODO(@MatthewChignoli): Maybe make an "is_planar" member to check if the constraint is planar

        std::string predecessor_link_name;
        std::string successor_link_name;

        // TODO(@MatthewChignoli): This is kind of hacky...
        // TODO(@MatthewChignoli): At the very least we need to add comments
        std::shared_ptr<Pose> predecessor_to_constraint_origin_transform;
        std::shared_ptr<Pose> successor_to_constraint_origin_transform;

        std::shared_ptr<double> ratio;
        std::shared_ptr<int> polarity;

        // TODO(@MatthewChignoli): add stuff for the differential constraint

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
            this->name.clear();
            this->successor_link_name.clear();
            this->predecessor_link_name.clear();
            this->nca_to_predecessor_subtree.clear();
            this->nca_to_successor_subtree.clear();
            this->predecessor_to_constraint_origin_transform.reset();
            this->successor_to_constraint_origin_transform.reset();
            this->ratio.reset();
            this->polarity.reset();
            this->type = UNKNOWN;
        };
    };

}

#endif
