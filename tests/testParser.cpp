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

// class ParentClusterTest : public ::testing::TestWithParam<std::string>
// {
// protected:
//     ParentClusterTest()
//     {
//         model_ = urdf::parseURDFFile("/home/matt/repos/URDF-Parser/" + GetParam() + ".urdf", false);
//     }

//     std::shared_ptr<urdf::ModelInterface> model_;

// };
