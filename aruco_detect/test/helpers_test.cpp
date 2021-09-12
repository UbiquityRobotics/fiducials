#include <helpers.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::ElementsAre;

TEST(HelperTest, split) {
    std::vector<std::string> v;
    split(v, "a,b,c", ',');
    ASSERT_THAT(v, ElementsAre("a", "b", "c"));
}

TEST(HelperTest, splitSingleElement) {
    std::vector<std::string> v;
    split(v, "a", ',');
    ASSERT_THAT(v, ElementsAre("a"));
}

TEST(HelperTest, splitEmpty) {
    std::vector<std::string> v;
    split(v, "", ',');
    ASSERT_THAT(v, ElementsAre());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}