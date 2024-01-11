#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"

// TODO(@MatthewChignoli): Maybe move these link tests to their own unit test file?
GTEST_TEST(Link, supportingChain)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);

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

GTEST_TEST(Link, supportingChainStartingFrom)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);

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

GTEST_TEST(Link, nearestCommonAncestor)
{
    std::shared_ptr<dynacore::urdf::ModelInterface> model;
    model = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);

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
