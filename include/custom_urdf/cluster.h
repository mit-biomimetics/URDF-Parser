#ifndef DYNACORE_URDF_INTERFACE_CLUSTER_H
#define DYNACORE_URDF_INTERFACE_CLUSTER_H

#include "link.h"
#include "constraint_joint.h"

namespace dynacore
{
    namespace urdf
    {

        class Cluster
        {
        public:
            std::vector<std::shared_ptr<Link>> links;
            std::vector<std::shared_ptr<ConstraintJoint>> constraint_joints;
        };

    }
}

#endif
