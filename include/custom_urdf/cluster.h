#ifndef URDF_INTERFACE_CLUSTER_H
#define URDF_INTERFACE_CLUSTER_H

#include "link.h"
#include "constraint_joint.h"

namespace urdf
{

    class Cluster
    {
    public:
        std::string name;

        std::vector<std::shared_ptr<Link>> links;
        std::vector<std::shared_ptr<Cluster>> child_clusters;
        std::vector<std::shared_ptr<ConstraintJoint>> constraint_joints;

        std::shared_ptr<Cluster> getParent() const { return parent_cluster_.lock(); }
        void setParent(const std::shared_ptr<Cluster> &parent) { parent_cluster_ = parent; }

    private:
        std::weak_ptr<Cluster> parent_cluster_;
    };

}

#endif
