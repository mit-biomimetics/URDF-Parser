#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"

using namespace dynacore::urdf;

GTEST_TEST(UrdfParser, parseUrdfFiles)
{
    std::vector<std::shared_ptr<ModelInterface>> models;
    models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", false));
    models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah_leg.urdf", false));
    models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false));
    models.push_back(parseURDFFile("/home/matt/repos/URDF-Parser/six_bar.urdf", false));

    for (const std::shared_ptr<ModelInterface> model : models)
    {
        ASSERT_TRUE(model != nullptr);
        std::cout << "Model name: " << model->getName() << " parsed successfully." << std::endl;
    }

    // for (const std::shared_ptr<ModelInterface> model : models)
    // {
    //     // Print the name of the model
    //     std::cout << "Model name: " << model->getName() << std::endl;

    //     // Print the names of the links
    //     std::vector<std::shared_ptr<dynacore::urdf::Link>> links;
    //     model->getLinks(links);
    //     std::cout << "\nLinks:" << std::endl;
    //     for (int i = 0; i < links.size(); i++)
    //     {
    //         std::cout << "  " << links[i]->name << std::endl;
    //     }

    //     // Print the name of the root link
    //     std::cout << "\nRoot link: " << model->getRoot()->name << std::endl;

    //     // Print the names of the joints
    //     std::cout << "\nJoints:" << std::endl;
    //     for (int i = 0; i < links.size(); i++)
    //     {
    //         if (links[i]->parent_joint)
    //             std::cout << "  " << links[i]->parent_joint->name << std::endl;
    //     }

    //     // Print the types of joints
    //     std::cout << "\nJoint types:" << std::endl;
    //     for (int i = 0; i < links.size(); i++)
    //     {
    //         if (links[i]->parent_joint)
    //         {
    //             if (links[i]->parent_joint->type == dynacore::urdf::Joint::REVOLUTE)
    //                 std::cout << "  "
    //                           << "REVOLUTE" << std::endl;
    //             else if (links[i]->parent_joint->type == dynacore::urdf::Joint::CONTINUOUS)
    //                 std::cout << "  "
    //                           << "CONTINUOUS" << std::endl;
    //             else if (links[i]->parent_joint->type == dynacore::urdf::Joint::PRISMATIC)
    //                 std::cout << "  "
    //                           << "PRISMATIC" << std::endl;
    //             else if (links[i]->parent_joint->type == dynacore::urdf::Joint::FIXED)
    //                 std::cout << "  "
    //                           << "FIXED" << std::endl;
    //             else if (links[i]->parent_joint->type == dynacore::urdf::Joint::FLOATING)
    //                 std::cout << "  "
    //                           << "FLOATING" << std::endl;
    //             else if (links[i]->parent_joint->type == dynacore::urdf::Joint::PLANAR)
    //                 std::cout << "  "
    //                           << "PLANAR" << std::endl;
    //             else
    //                 std::cout << "  "
    //                           << "UNKNOWN" << std::endl;
    //         }

    //         // if (links[i]->parent_joint)
    //         // std::cout << "  " << links[i]->parent_joint->type << std::endl;
    //     }

    //     // Print the clusters and the names of the links they contain
    //     const std::map<std::string, std::shared_ptr<dynacore::urdf::Cluster>> clusters = model->getClusters();
    //     std::cout << "\nClusters:" << std::endl;
    //     for (std::map<std::string, std::shared_ptr<dynacore::urdf::Cluster>>::const_iterator cluster = clusters.begin(); cluster != clusters.end(); cluster++)
    //     {
    //         std::cout << "  " << cluster->first << std::endl;
    //         for (std::shared_ptr<Link> link : cluster->second->links)
    //         {
    //             std::cout << "    " << link->name << std::endl;
    //         }
    //     }
    // }

}
