#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"

// TODO(@MatthewChignoli): Maybe move these link tests to their own unit test file?
GTEST_TEST(UrdfParser, supportingChain)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", true);

    using LinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    LinkPtr base = model->getLink("base_link");
    LinkPtr link1 = model->getLink("link1");
    LinkPtr link2 = model->getLink("link2");
    LinkPtr link3 = model->getLink("link3");

    {
        std::vector<LinkPtr> supporting_chain;
        EXPECT_EQ(base->getSupportingChain(), supporting_chain);
    }

    {
        std::vector<LinkPtr> supporting_chain;
        supporting_chain.push_back(base);
        EXPECT_EQ(link1->getSupportingChain(), supporting_chain);
    }

    {
        std::vector<LinkPtr> supporting_chain;
        supporting_chain.push_back(link1);
        supporting_chain.push_back(base);
        EXPECT_EQ(link2->getSupportingChain(), supporting_chain);
    }

    {
        std::vector<LinkPtr> supporting_chain;
        supporting_chain.push_back(base);
        EXPECT_EQ(link3->getSupportingChain(), supporting_chain);
    }
}

GTEST_TEST(UrdfParser, supportingChainStartingFrom)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", true);

    using LinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    LinkPtr base = model->getLink("base_link");
    LinkPtr link1 = model->getLink("link1");
    LinkPtr link2 = model->getLink("link2");

    {
        std::vector<LinkPtr> supporting_chain;
        EXPECT_EQ(link1->getSupportingChainStartingFrom(base), supporting_chain);
    }

    {
        std::vector<LinkPtr> supporting_chain;
        supporting_chain.push_back(link1);
        EXPECT_EQ(link2->getSupportingChainStartingFrom(base), supporting_chain);
    }

}

GTEST_TEST(UrdfParser, nearestCommonAncestor)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", true);

    using LinkPtr = std::shared_ptr<const dynacore::urdf::Link>;
    LinkPtr base = model->getLink("base_link");
    LinkPtr link1 = model->getLink("link1");
    LinkPtr link2 = model->getLink("link2");
    LinkPtr link3 = model->getLink("link3");

    EXPECT_EQ(model->nearestCommonAncestor(base, base), base);
    EXPECT_EQ(model->nearestCommonAncestor(base, link1), base);
    EXPECT_EQ(model->nearestCommonAncestor(base, link2), base);
    EXPECT_EQ(model->nearestCommonAncestor(base, link3), base);
    
    EXPECT_EQ(model->nearestCommonAncestor(link1, base), base);
    EXPECT_EQ(model->nearestCommonAncestor(link1, link1), link1);
    EXPECT_EQ(model->nearestCommonAncestor(link1, link2), link1);
    EXPECT_EQ(model->nearestCommonAncestor(link1, link3), base);

    EXPECT_EQ(model->nearestCommonAncestor(link2, base), base);
    EXPECT_EQ(model->nearestCommonAncestor(link2, link1), link1);
    EXPECT_EQ(model->nearestCommonAncestor(link2, link2), link2);
    EXPECT_EQ(model->nearestCommonAncestor(link2, link3), base);

    EXPECT_EQ(model->nearestCommonAncestor(link3, base), base);
    EXPECT_EQ(model->nearestCommonAncestor(link3, link1), base);
    EXPECT_EQ(model->nearestCommonAncestor(link3, link2), base);
    EXPECT_EQ(model->nearestCommonAncestor(link3, link3), link3);

}

GTEST_TEST(UrdfParser, parseUrdfFiles)
{
    GTEST_SKIP();

    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    // model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", true);
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", true);

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

    // What we eventually want is to print the clusters.
    // And then later we need to figure out how to turn constraints into phi, K, and k.

    // Let's start with adding the rotors to the URDF
}
