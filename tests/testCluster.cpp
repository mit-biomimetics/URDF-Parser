#include "gtest/gtest.h"
#include "custom_urdf/urdf_parser.h"

// TODO(@MatthewChignoli): Test for other URDFs as well
// TODO(@MatthewChignoli): Is there a way to template the test on the URDF file? Like so that we can test for multiple robots without having to copy and paste the same test code?
// TODO(@MatthewChignoli): The tests themselves are good, but the amount of duplicate code and the poor organization of the tests is not good. Need to refactor.

class FourBar_ClusterTests : public ::testing::Test
{
protected:
    using LinkPtr = std::shared_ptr<urdf::Link>;
    using ClusterPtr = std::shared_ptr<urdf::Cluster>;

    void SetUp() override
    {
        model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/four_bar.urdf", false);

        base_cluster_ = model_->getClusterContaining("base_link");
        links_cluster_ = model_->getClusterContaining("link1");

        base_children_.push_back(links_cluster_);
        links_parent_ = base_cluster_;
    }

    std::shared_ptr<urdf::ModelInterface> model_;
    ClusterPtr base_cluster_, links_cluster_;

    // Ground Truth children
    std::vector<ClusterPtr> base_children_;
    std::vector<ClusterPtr> links_children_;

    // Ground Truth parents
    ClusterPtr base_parent_;
    ClusterPtr links_parent_;
};

TEST_F(FourBar_ClusterTests, parents)
{
    EXPECT_EQ(base_parent_, base_cluster_->getParent());
    EXPECT_EQ(links_parent_, links_cluster_->getParent());
}

TEST_F(FourBar_ClusterTests, children)
{
    EXPECT_EQ(base_children_, base_cluster_->child_clusters);
    EXPECT_EQ(links_children_, links_cluster_->child_clusters);
}

// TODO(@MatthewChignoli): Now we need to try this for the mini_cheetah...
// TODO(@MatthewChignoli): The way of naming the clusters become much more important here..
class MiniCheetah_ClusterTests : public ::testing::Test
{
protected:
    using LinkPtr = std::shared_ptr<urdf::Link>;
    using ClusterPtr = std::shared_ptr<urdf::Cluster>;

    void SetUp() override
    {
        model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/mini_cheetah.urdf", false);

        body_cluster = model_->getClusterContaining("body");
        abduct_fr_cluster = model_->getClusterContaining("abduct_fr");
        abduct_fl_cluster = model_->getClusterContaining("abduct_fl");
        abduct_hr_cluster = model_->getClusterContaining("abduct_hr");
        abduct_hl_cluster = model_->getClusterContaining("abduct_hl");
        hip_fr_cluster = model_->getClusterContaining("hip_rotor_fr");
        hip_fl_cluster = model_->getClusterContaining("hip_rotor_fl");
        hip_hr_cluster = model_->getClusterContaining("hip_rotor_hr");
        hip_hl_cluster = model_->getClusterContaining("hip_rotor_hl");
        knee_fr_cluster = model_->getClusterContaining("knee_rotor_fr");
        knee_fl_cluster = model_->getClusterContaining("knee_rotor_fl");
        knee_hr_cluster = model_->getClusterContaining("knee_rotor_hr");
        knee_hl_cluster = model_->getClusterContaining("knee_rotor_hl");

        // Ground Truth Parents
        abduct_fr_parent_ = body_cluster;
        abduct_fl_parent_ = body_cluster;
        abduct_hr_parent_ = body_cluster;
        abduct_hl_parent_ = body_cluster;
        hip_fr_parent_ = abduct_fr_cluster;
        hip_fl_parent_ = abduct_fl_cluster;
        hip_hr_parent_ = abduct_hr_cluster;
        hip_hl_parent_ = abduct_hl_cluster;
        knee_fr_parent_ = hip_fr_cluster;
        knee_fl_parent_ = hip_fl_cluster;
        knee_hr_parent_ = hip_hr_cluster;
        knee_hl_parent_ = hip_hl_cluster;

        // Ground Truth Children
        body_children_.push_back(abduct_fl_cluster);
        body_children_.push_back(abduct_fr_cluster);
        body_children_.push_back(abduct_hl_cluster);
        body_children_.push_back(abduct_hr_cluster);
        abduct_fr_children_.push_back(hip_fr_cluster);
        abduct_fl_children_.push_back(hip_fl_cluster);
        abduct_hr_children_.push_back(hip_hr_cluster);
        abduct_hl_children_.push_back(hip_hl_cluster);
        hip_fr_children_.push_back(knee_fr_cluster);
        hip_fl_children_.push_back(knee_fl_cluster);
        hip_hr_children_.push_back(knee_hr_cluster);
        hip_hl_children_.push_back(knee_hl_cluster);
    }

    std::shared_ptr<urdf::ModelInterface> model_;

    ClusterPtr body_cluster;
    ClusterPtr abduct_fr_cluster, abduct_fl_cluster, abduct_hr_cluster, abduct_hl_cluster;
    ClusterPtr hip_fr_cluster, hip_fl_cluster, hip_hr_cluster, hip_hl_cluster;
    ClusterPtr knee_fr_cluster, knee_fl_cluster, knee_hr_cluster, knee_hl_cluster;

    // Ground Truth parents
    ClusterPtr body_parent_;
    ClusterPtr abduct_fr_parent_, abduct_fl_parent_, abduct_hr_parent_, abduct_hl_parent_;
    ClusterPtr hip_fr_parent_, hip_fl_parent_, hip_hr_parent_, hip_hl_parent_;
    ClusterPtr knee_fr_parent_, knee_fl_parent_, knee_hr_parent_, knee_hl_parent_;

    // Ground Truth children
    std::vector<ClusterPtr> body_children_;
    std::vector<ClusterPtr> abduct_fr_children_, abduct_fl_children_, abduct_hr_children_, abduct_hl_children_;
    std::vector<ClusterPtr> hip_fr_children_, hip_fl_children_, hip_hr_children_, hip_hl_children_;
    std::vector<ClusterPtr> knee_fr_children_, knee_fl_children_, knee_hr_children_, knee_hl_children_;
};

TEST_F(MiniCheetah_ClusterTests, parents)
{
    EXPECT_EQ(body_parent_, body_cluster->getParent());
    EXPECT_EQ(abduct_fr_parent_, abduct_fr_cluster->getParent());
    EXPECT_EQ(abduct_fl_parent_, abduct_fl_cluster->getParent());
    EXPECT_EQ(abduct_hr_parent_, abduct_hr_cluster->getParent());
    EXPECT_EQ(abduct_hl_parent_, abduct_hl_cluster->getParent());
    EXPECT_EQ(hip_fr_parent_, hip_fr_cluster->getParent());
    EXPECT_EQ(hip_fl_parent_, hip_fl_cluster->getParent());
    EXPECT_EQ(hip_hr_parent_, hip_hr_cluster->getParent());
    EXPECT_EQ(hip_hl_parent_, hip_hl_cluster->getParent());
    EXPECT_EQ(knee_fr_parent_, knee_fr_cluster->getParent());
    EXPECT_EQ(knee_fl_parent_, knee_fl_cluster->getParent());
    EXPECT_EQ(knee_hr_parent_, knee_hr_cluster->getParent());
    EXPECT_EQ(knee_hl_parent_, knee_hl_cluster->getParent());
}

TEST_F(MiniCheetah_ClusterTests, children)
{
    EXPECT_EQ(body_children_, body_cluster->child_clusters);
    EXPECT_EQ(abduct_fr_children_, abduct_fr_cluster->child_clusters);
    EXPECT_EQ(abduct_fl_children_, abduct_fl_cluster->child_clusters);
    EXPECT_EQ(abduct_hr_children_, abduct_hr_cluster->child_clusters);
    EXPECT_EQ(abduct_hl_children_, abduct_hl_cluster->child_clusters);
    EXPECT_EQ(hip_fr_children_, hip_fr_cluster->child_clusters);
    EXPECT_EQ(hip_fl_children_, hip_fl_cluster->child_clusters);
    EXPECT_EQ(hip_hr_children_, hip_hr_cluster->child_clusters);
    EXPECT_EQ(hip_hl_children_, hip_hl_cluster->child_clusters);
    EXPECT_EQ(knee_fr_children_, knee_fr_cluster->child_clusters);
    EXPECT_EQ(knee_fl_children_, knee_fl_cluster->child_clusters);
    EXPECT_EQ(knee_hr_children_, knee_hr_cluster->child_clusters);
    EXPECT_EQ(knee_hl_children_, knee_hl_cluster->child_clusters);
}
