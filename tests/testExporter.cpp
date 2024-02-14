#include "gtest/gtest.h"

#include <cstdio>
#include "custom_urdf/urdf_parser.h"

using namespace urdf;

const std::string urdf_directory = "/home/matt/repos/URDF-Parser/";
// const std::string urdf_directory = "/Users/matthewchignoli/repos/URDF-Parser/";

std::vector<std::string> GetTestUrdfFiles()
{
    std::vector<std::string> test_urdf_files;
    test_urdf_files.push_back("mini_cheetah");
    test_urdf_files.push_back("mini_cheetah_leg");
    test_urdf_files.push_back("four_bar");
    test_urdf_files.push_back("six_bar");
    test_urdf_files.push_back("planar_leg_linkage");
    test_urdf_files.push_back("revolute_rotor_chain");
    test_urdf_files.push_back("mit_humanoid_leg");
    return test_urdf_files;
}

class ExporterTest : public ::testing::TestWithParam<std::string>
{
    void SetUp() override
    {
        urdf_file_name = urdf_directory + GetParam() + ".urdf";
        exported_file_name = urdf_directory + GetParam() + "_exported.urdf";
    }

    void TearDown() override
    {
        std::remove(exported_file_name.c_str());
    }

protected:
    std::string urdf_file_name;
    std::string exported_file_name;
};

INSTANTIATE_TEST_SUITE_P(ExporterTest, ExporterTest, ::testing::ValuesIn(GetTestUrdfFiles()));

TEST_P(ExporterTest, parseAndExport)
{
    std::shared_ptr<ModelInterface> model = parseURDFFile(urdf_file_name);
    ASSERT_TRUE(model != nullptr);
    TiXmlDocument *xml_doc = exportURDF(model);
    ASSERT_TRUE(xml_doc != nullptr);
    xml_doc->SaveFile(exported_file_name);
    delete xml_doc;
}
