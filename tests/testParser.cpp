#include "gtest/gtest.h"

#include "custom_urdf/urdf_parser.h"

using namespace urdf;

std::vector<std::string> GetTestUrdfFiles()
{
    std::vector<std::string> test_urdf_files;
    test_urdf_files.push_back("mini_cheetah");
    test_urdf_files.push_back("mini_cheetah_leg");
    test_urdf_files.push_back("four_bar");
    test_urdf_files.push_back("six_bar");
    test_urdf_files.push_back("planar_leg_linkage");
    test_urdf_files.push_back("revolute_rotor_chain");
    return test_urdf_files;
}

class ParserTest : public ::testing::TestWithParam<std::string>
{
};

INSTANTIATE_TEST_CASE_P(ParserTest, ParserTest, ::testing::ValuesIn(GetTestUrdfFiles()));

TEST_P(ParserTest, createModelFromUrdfFile)
{
    std::shared_ptr<ModelInterface> model = parseURDFFile("/home/matt/repos/URDF-Parser/" +
                                                          GetParam() + ".urdf");
    ASSERT_TRUE(model != nullptr);
}

using StrStrMap = std::map<std::string, std::string>;
std::vector<std::pair<std::string, StrStrMap>> GetLinksAndParents()
{
    std::vector<std::pair<std::string, StrStrMap>> robots_links_parents;

    StrStrMap four_bar_links_parents;
    four_bar_links_parents.insert(std::make_pair("link1", "base_link"));
    four_bar_links_parents.insert(std::make_pair("link2", "link1"));
    four_bar_links_parents.insert(std::make_pair("link3", "base_link"));
    robots_links_parents.push_back(std::make_pair("four_bar",
                                                  four_bar_links_parents));

    StrStrMap mini_cheetah_leg_links_parents;
    mini_cheetah_leg_links_parents.insert(std::make_pair("abduct", "base"));
    mini_cheetah_leg_links_parents.insert(std::make_pair("abduct_rotor", "base"));
    mini_cheetah_leg_links_parents.insert(std::make_pair("thigh", "abduct"));
    mini_cheetah_leg_links_parents.insert(std::make_pair("hip_rotor", "abduct"));
    mini_cheetah_leg_links_parents.insert(std::make_pair("shank", "thigh"));
    mini_cheetah_leg_links_parents.insert(std::make_pair("knee_rotor", "thigh"));
    robots_links_parents.push_back(std::make_pair("mini_cheetah_leg",
                                                  mini_cheetah_leg_links_parents));

    // TODO(@MatthewChignoli): Can add more robots here if desired...

    return robots_links_parents;
}
class ParentLinkTest : public ::testing::TestWithParam<std::pair<std::string, StrStrMap>>
{
protected:
    ParentLinkTest()
    {
        model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/" + GetParam().first + ".urdf");
    }
    std::shared_ptr<urdf::ModelInterface> model_;
};

INSTANTIATE_TEST_CASE_P(ParentLinkTest, ParentLinkTest, ::testing::ValuesIn(GetLinksAndParents()));

TEST_P(ParentLinkTest, parent)
{
    const StrStrMap &links_and_parents = GetParam().second;
    for (const auto &link_and_parent : links_and_parents)
    {
        EXPECT_EQ(link_and_parent.second,
                  model_->getLink(link_and_parent.first)->getParent()->name);
    }
}

// TODO(@MatthewChignoli): Make a list of remaining tests that I will eventually make before realeasing the repo. They should mostly just be the existing tests, but with the new setup that has less redundant code
// Tests needed:
// - children for links
// - supporting chains for links
// - subtrees for links
// - subtrees between links for links
// - nearest common ancestor for links
// - neighbors for links
// - parents for clusters
// - children for clusters
