#include "gtest/gtest.h"
#include "custom_urdf/urdf_parser.h"

// TODO(@MatthewChignoli): Test for other URDFs as well
// TODO(@MatthewChignoli): Is there a way to template the test on the URDF file? Like so that we can test for multiple robots without having to copy and paste the same test code?

class FourBar_LinkTests : public ::testing::Test
{
protected:
    using LinkPtr = std::shared_ptr<dynacore::urdf::Link>;

    void SetUp() override
    {
        model_ = dynacore::urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);
        model_->getLink("base_link", base_);
        model_->getLink("link1", link1_);
        model_->getLink("link2", link2_);
        model_->getLink("link3", link3_);
    }

    std::shared_ptr<dynacore::urdf::ModelInterface> model_;
    LinkPtr base_, link1_, link2_, link3_;
};

// TODO(@MatthewChignoli): This is actually a model test now?
TEST_F(FourBar_LinkTests, supportingChain)
{
    // Ground truth supporting chains
    std::vector<LinkPtr> base_supporting_chain;
    base_supporting_chain.push_back(base_);

    std::vector<LinkPtr> link1_supporting_chain;
    link1_supporting_chain.push_back(link1_);
    link1_supporting_chain.push_back(base_);

    std::vector<LinkPtr> link2_supporting_chain;
    link2_supporting_chain.push_back(link2_);
    link2_supporting_chain.push_back(link1_);
    link2_supporting_chain.push_back(base_);

    std::vector<LinkPtr> link3_supporting_chain;
    link3_supporting_chain.push_back(link3_);
    link3_supporting_chain.push_back(base_);

    // Compare to algorithmically computed supporting chains
    std::vector<LinkPtr> supporting_chain;
    model_->getSupportingChain("base_link", supporting_chain);
    EXPECT_EQ(supporting_chain, base_supporting_chain);

    model_->getSupportingChain("link1", supporting_chain);
    EXPECT_EQ(supporting_chain, link1_supporting_chain);

    model_->getSupportingChain("link2", supporting_chain);
    EXPECT_EQ(supporting_chain, link2_supporting_chain);

    model_->getSupportingChain("link3", supporting_chain);
    EXPECT_EQ(supporting_chain, link3_supporting_chain);

}

TEST_F(FourBar_LinkTests, subtreesBetweenLinks)
{
    // Ground truth supporting chains between links
    std::vector<LinkPtr> base_to_base_subtree;
    std::vector<LinkPtr> base_to_link1_subtree;
    base_to_link1_subtree.push_back(link1_);
    std::vector<LinkPtr> base_to_link2_subtree;
    base_to_link2_subtree.push_back(link1_);
    base_to_link2_subtree.push_back(link2_);
    std::vector<LinkPtr> base_to_link3_subtree;
    base_to_link3_subtree.push_back(link3_);

    std::vector<LinkPtr> link1_to_link1_subtree;
    std::vector<LinkPtr> link1_to_link2_subtree;
    link1_to_link2_subtree.push_back(link2_);

    std::vector<LinkPtr> link2_to_link2_subtree;

    std::vector<LinkPtr> link3_to_link3_subtree;

    // Compare to algorithmically computed supporting chains
    std::vector<LinkPtr> supporting_chain;
    model_->getSubtreeBetweenLinks("base_link", "base_link", supporting_chain);
    EXPECT_EQ(supporting_chain, base_to_base_subtree);

    model_->getSubtreeBetweenLinks("base_link", "link1", supporting_chain);
    EXPECT_EQ(supporting_chain, base_to_link1_subtree);

    model_->getSubtreeBetweenLinks("base_link", "link2", supporting_chain);
    EXPECT_EQ(supporting_chain, base_to_link2_subtree);

    model_->getSubtreeBetweenLinks("base_link", "link3", supporting_chain);
    EXPECT_EQ(supporting_chain, base_to_link3_subtree);

    model_->getSubtreeBetweenLinks("link1", "link1", supporting_chain);
    EXPECT_EQ(supporting_chain, link1_to_link1_subtree);

    model_->getSubtreeBetweenLinks("link1", "link2", supporting_chain);
    EXPECT_EQ(supporting_chain, link1_to_link2_subtree);

    model_->getSubtreeBetweenLinks("link2", "link2", supporting_chain);
    EXPECT_EQ(supporting_chain, link2_to_link2_subtree);

    model_->getSubtreeBetweenLinks("link3", "link3", supporting_chain);
    EXPECT_EQ(supporting_chain, link3_to_link3_subtree);
}

TEST_F(FourBar_LinkTests, nearestCommonAncestor)
{
    // Base
    EXPECT_EQ(model_->nearestCommonAncestor(base_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link1_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link2_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link3_), base_);

    // Link 1
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link1_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link2_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link3_), base_);

    // Link 2
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link1_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link2_), link2_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link3_), base_);

    // Link 3
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link1_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link2_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link3_), link3_);
}

TEST_F(FourBar_LinkTests, neighbors)
{
    // Ground truth neighbors
    std::vector<LinkPtr> base_neighbors;
    base_neighbors.push_back(link1_);
    base_neighbors.push_back(link3_);

    std::vector<LinkPtr> link1_neighbors;
    link1_neighbors.push_back(link2_);

    std::vector<LinkPtr> link2_neighbors;
    link2_neighbors.push_back(link3_);

    std::vector<LinkPtr> link3_neighbors;
    link3_neighbors.push_back(link1_);

    // Compare to algorithmically computed neighbors
    std::vector<LinkPtr> neighbors;
    EXPECT_EQ(base_->neighbors, base_neighbors);
    EXPECT_EQ(link1_->neighbors, link1_neighbors);
    EXPECT_EQ(link2_->neighbors, link2_neighbors);
    EXPECT_EQ(link3_->neighbors, link3_neighbors);
}
