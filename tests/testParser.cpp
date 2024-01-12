#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"

using namespace dynacore::urdf;

GTEST_TEST(UrdfParser, parseUrdfFiles)
{
    // std::shared_ptr<dynacore::urdf::ModelInterface> model;
    // model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", true);
    // model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", true);

    std::vector<std::shared_ptr<ModelInterface>> models;
    // models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", false));
    // models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false));
    models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/six_bar.urdf", false));

    for (const std::shared_ptr<ModelInterface> model : models)
    {

        // Print the name of the model
        std::cout << "Model name: " << model->getName() << std::endl;

        // Print the names of the links
        std::vector<std::shared_ptr<dynacore::urdf::Link>> links;
        model->getLinks(links);
        std::cout << "Links:" << std::endl;
        for (int i = 0; i < links.size(); i++)
        {
            std::cout << "  " << links[i]->name << std::endl;
        }

        // Print the name of the root link
        std::cout << "Root link: " << model->getRoot()->name << std::endl;

        // Print the names of the joints
        std::cout << "Joints:" << std::endl;
        for (int i = 0; i < links.size(); i++)
        {
            if (links[i]->parent_joint)
                std::cout << "  " << links[i]->parent_joint->name << std::endl;
        }

        // Print the types of joints
        std::cout << "Joint types:" << std::endl;
        for (int i = 0; i < links.size(); i++)
        {
            if (links[i]->parent_joint)
            {
                if (links[i]->parent_joint->type == dynacore::urdf::Joint::REVOLUTE)
                    std::cout << "  "
                              << "REVOLUTE" << std::endl;
                else if (links[i]->parent_joint->type == dynacore::urdf::Joint::CONTINUOUS)
                    std::cout << "  "
                              << "CONTINUOUS" << std::endl;
                else if (links[i]->parent_joint->type == dynacore::urdf::Joint::PRISMATIC)
                    std::cout << "  "
                              << "PRISMATIC" << std::endl;
                else if (links[i]->parent_joint->type == dynacore::urdf::Joint::FIXED)
                    std::cout << "  "
                              << "FIXED" << std::endl;
                else if (links[i]->parent_joint->type == dynacore::urdf::Joint::FLOATING)
                    std::cout << "  "
                              << "FLOATING" << std::endl;
                else if (links[i]->parent_joint->type == dynacore::urdf::Joint::PLANAR)
                    std::cout << "  "
                              << "PLANAR" << std::endl;
                else
                    std::cout << "  "
                              << "UNKNOWN" << std::endl;
            }

            // if (links[i]->parent_joint)
            // std::cout << "  " << links[i]->parent_joint->type << std::endl;
        }

        // Print the clusters and the names of the links they contain
        std::vector<std::shared_ptr<dynacore::urdf::Cluster>> clusters= model->getClusters();
        std::cout << "Clusters:" << std::endl;
        for (int i = 0; i < clusters.size(); i++)
        {
            std::cout << "Cluster #" << i << std::endl;
            std::shared_ptr<dynacore::urdf::Cluster> cluster = clusters[i];
            for (int j = 0; j < cluster->links.size(); j++)
            {
                std::cout << "    " << cluster->links[j]->name << std::endl;
            }
        }

        // TODO(@MatthewChignoli): add a formal check on the clusters
    }
}
