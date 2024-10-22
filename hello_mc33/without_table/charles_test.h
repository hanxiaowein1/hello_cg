#ifndef _CHARLES_TEST_H_
#define _CHARLES_TEST_H_

#include "gtest/gtest.h"

class GlobalTest
{
    ~GlobalTest() = default;
    void SetUp() {};
    void TearDown() {};
};

#endif