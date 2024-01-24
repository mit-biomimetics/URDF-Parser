#include "gtest/gtest.h"
#include "custom_urdf/urdf_parser.h"

// TODO(@MatthewChignoli): Test for other URDFs as well
// TODO(@MatthewChignoli): Is there a way to template the test on the URDF file? Like so that we can test for multiple robots without having to copy and paste the same test code?

// TODO(@MatthewChignoli): The tests themselves are good, but the amount of duplicate code and the poor organization of the tests is not good. Need to refactor.

class FourBar_LinkTests : public ::testing::Test
{
protected:
    using LinkPtr = std::shared_ptr<urdf::Link>;

    void SetUp() override
    {
        model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);
        model_->getLink("base_link", base_);
        model_->getLink("link1", link1_);
        model_->getLink("link2", link2_);
        model_->getLink("link3", link3_);

        // Support chains
        base_supporting_chain_.push_back(base_);
        link1_supporting_chain_.push_back(base_);
        link1_supporting_chain_.push_back(link1_);
        link2_supporting_chain_.push_back(base_);
        link2_supporting_chain_.push_back(link1_);
        link2_supporting_chain_.push_back(link2_);
        link3_supporting_chain_.push_back(base_);
        link3_supporting_chain_.push_back(link3_);

        // Subtrees between links
        base_to_link1_subtree_.push_back(link1_);
        base_to_link2_subtree_.push_back(link1_);
        base_to_link2_subtree_.push_back(link2_);
        base_to_link3_subtree_.push_back(link3_);
        link1_to_link2_subtree_.push_back(link2_);

        // Neighbors
        base_neighbors_.push_back(link1_);
        base_neighbors_.push_back(link3_);
        link1_neighbors_.push_back(link2_);
        link2_neighbors_.push_back(link3_);
        link3_neighbors_.push_back(link1_);
    }

    std::shared_ptr<urdf::ModelInterface> model_;
    LinkPtr base_, link1_, link2_, link3_;

    // Ground truth supporting chains
    std::vector<LinkPtr> base_supporting_chain_;
    std::vector<LinkPtr> link1_supporting_chain_;
    std::vector<LinkPtr> link2_supporting_chain_;
    std::vector<LinkPtr> link3_supporting_chain_;

    // Ground truth subtrees between links
    std::vector<LinkPtr> base_to_base_subtree_;
    std::vector<LinkPtr> base_to_link1_subtree_;
    std::vector<LinkPtr> base_to_link2_subtree_;
    std::vector<LinkPtr> base_to_link3_subtree_;
    std::vector<LinkPtr> link1_to_link1_subtree_;
    std::vector<LinkPtr> link1_to_link2_subtree_;
    std::vector<LinkPtr> link2_to_link2_subtree_;
    std::vector<LinkPtr> link3_to_link3_subtree_;

    // Ground truth neighbors
    std::vector<LinkPtr> base_neighbors_;
    std::vector<LinkPtr> link1_neighbors_;
    std::vector<LinkPtr> link2_neighbors_;
    std::vector<LinkPtr> link3_neighbors_;
};

// TODO(@MatthewChignoli): This is actually a model test now?
TEST_F(FourBar_LinkTests, supportingChain)
{
    std::vector<LinkPtr> supporting_chain;
    model_->getSupportingChain("base_link", supporting_chain);
    EXPECT_EQ(supporting_chain, base_supporting_chain_);

    model_->getSupportingChain("link1", supporting_chain);
    EXPECT_EQ(supporting_chain, link1_supporting_chain_);

    model_->getSupportingChain("link2", supporting_chain);
    EXPECT_EQ(supporting_chain, link2_supporting_chain_);

    model_->getSupportingChain("link3", supporting_chain);
    EXPECT_EQ(supporting_chain, link3_supporting_chain_);
}

TEST_F(FourBar_LinkTests, subtreesBetweenLinks)
{
    std::vector<LinkPtr> subtree;
    model_->getSubtreeBetweenLinks("base_link", "base_link", subtree);
    EXPECT_EQ(subtree, base_to_base_subtree_);

    model_->getSubtreeBetweenLinks("base_link", "link1", subtree);
    EXPECT_EQ(subtree, base_to_link1_subtree_);

    model_->getSubtreeBetweenLinks("base_link", "link2", subtree);
    EXPECT_EQ(subtree, base_to_link2_subtree_);

    model_->getSubtreeBetweenLinks("base_link", "link3", subtree);
    EXPECT_EQ(subtree, base_to_link3_subtree_);

    model_->getSubtreeBetweenLinks("link1", "link1", subtree);
    EXPECT_EQ(subtree, link1_to_link1_subtree_);

    model_->getSubtreeBetweenLinks("link1", "link2", subtree);
    EXPECT_EQ(subtree, link1_to_link2_subtree_);

    model_->getSubtreeBetweenLinks("link2", "link2", subtree);
    EXPECT_EQ(subtree, link2_to_link2_subtree_);

    model_->getSubtreeBetweenLinks("link3", "link3", subtree);
    EXPECT_EQ(subtree, link3_to_link3_subtree_);
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
    EXPECT_EQ(base_->neighbors, base_neighbors_);
    EXPECT_EQ(link1_->neighbors, link1_neighbors_);
    EXPECT_EQ(link2_->neighbors, link2_neighbors_);
    EXPECT_EQ(link3_->neighbors, link3_neighbors_);
}

// TODO(@MatthewChignoli): Separate file for the six bar? There is for sure a way to abstract these tests to be valid for a N-bar mechanism. Can actually probably abstract to any robot
class SixBar_LinkTests : public ::testing::Test
{
protected:
    using LinkPtr = std::shared_ptr<urdf::Link>;

    void SetUp() override
    {
        model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/six_bar.urdf", false);
        model_->getLink("base_link", base_);
        model_->getLink("link1", link1_);
        model_->getLink("link2", link2_);
        model_->getLink("link3", link3_);
        model_->getLink("link4", link4_);
        model_->getLink("link5", link5_);

        // Support chains
        base_supporting_chain_.push_back(base_);
        link1_supporting_chain_.push_back(base_);
        link1_supporting_chain_.push_back(link1_);
        link2_supporting_chain_.push_back(base_);
        link2_supporting_chain_.push_back(link1_);
        link2_supporting_chain_.push_back(link2_);
        link3_supporting_chain_.push_back(base_);
        link3_supporting_chain_.push_back(link1_);
        link3_supporting_chain_.push_back(link2_);
        link3_supporting_chain_.push_back(link3_);
        link4_supporting_chain_.push_back(base_);
        link4_supporting_chain_.push_back(link4_);
        link5_supporting_chain_.push_back(base_);
        link5_supporting_chain_.push_back(link4_);
        link5_supporting_chain_.push_back(link5_);

        // Neighbors
        base_neighbors_.push_back(link1_);
        base_neighbors_.push_back(link4_);
        link1_neighbors_.push_back(link2_);
        link2_neighbors_.push_back(link3_);
        link3_neighbors_.push_back(link4_);
        link4_neighbors_.push_back(link5_);
        link5_neighbors_.push_back(link1_);
    }

    std::shared_ptr<urdf::ModelInterface> model_;
    LinkPtr base_, link1_, link2_, link3_, link4_, link5_;

    // Ground truth supporting chains
    std::vector<LinkPtr> base_supporting_chain_;
    std::vector<LinkPtr> link1_supporting_chain_;
    std::vector<LinkPtr> link2_supporting_chain_;
    std::vector<LinkPtr> link3_supporting_chain_;
    std::vector<LinkPtr> link4_supporting_chain_;
    std::vector<LinkPtr> link5_supporting_chain_;

    // Ground truth neighbors
    std::vector<LinkPtr> base_neighbors_;
    std::vector<LinkPtr> link1_neighbors_;
    std::vector<LinkPtr> link2_neighbors_;
    std::vector<LinkPtr> link3_neighbors_;
    std::vector<LinkPtr> link4_neighbors_;
    std::vector<LinkPtr> link5_neighbors_;
};

TEST_F(SixBar_LinkTests, supportingChain)
{
    std::vector<LinkPtr> supporting_chain;
    model_->getSupportingChain("base_link", supporting_chain);
    EXPECT_EQ(supporting_chain, base_supporting_chain_);

    model_->getSupportingChain("link1", supporting_chain);
    EXPECT_EQ(supporting_chain, link1_supporting_chain_);

    model_->getSupportingChain("link2", supporting_chain);
    EXPECT_EQ(supporting_chain, link2_supporting_chain_);

    model_->getSupportingChain("link3", supporting_chain);
    EXPECT_EQ(supporting_chain, link3_supporting_chain_);

    model_->getSupportingChain("link4", supporting_chain);
    EXPECT_EQ(supporting_chain, link4_supporting_chain_);

    model_->getSupportingChain("link5", supporting_chain);
    EXPECT_EQ(supporting_chain, link5_supporting_chain_);
}

TEST_F(SixBar_LinkTests, nearestCommonAncestor)
{
    // Base
    EXPECT_EQ(model_->nearestCommonAncestor(base_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link1_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link2_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link3_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link4_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(base_, link5_), base_);

    // Link 1
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link1_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link2_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link3_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link4_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link1_, link5_), base_);

    // Link 2
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link1_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link2_), link2_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link3_), link2_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link4_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link2_, link5_), base_);

    // Link 3
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link1_), link1_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link2_), link2_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link3_), link3_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link4_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link3_, link5_), base_);

    // Link 4
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, link1_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, link2_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, link3_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, link4_), link4_);
    EXPECT_EQ(model_->nearestCommonAncestor(link4_, link5_), link4_);

    // Link 5
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, base_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, link1_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, link2_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, link3_), base_);
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, link4_), link4_);
    EXPECT_EQ(model_->nearestCommonAncestor(link5_, link5_), link5_);
}

TEST_F(SixBar_LinkTests, neighbors)
{
    EXPECT_EQ(base_->neighbors, base_neighbors_);
    EXPECT_EQ(link1_->neighbors, link1_neighbors_);
    EXPECT_EQ(link2_->neighbors, link2_neighbors_);
    EXPECT_EQ(link3_->neighbors, link3_neighbors_);
    EXPECT_EQ(link4_->neighbors, link4_neighbors_);
    EXPECT_EQ(link5_->neighbors, link5_neighbors_);
}
