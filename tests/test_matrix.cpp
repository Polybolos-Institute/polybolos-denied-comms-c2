#include <gtest/gtest.h>

#include "filters/matrix.h"

using namespace Polybolos::Fusion;

TEST(Mat, IdentityInverse) {
    Mat I = Mat::identity(4);
    Mat inv = I.inverse();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(inv(i, j), (i == j) ? 1.0 : 0.0, 1e-12);
        }
    }
}

TEST(Mat, Multiply) {
    Mat A(2, 2);
    A(0, 0) = 1;
    A(0, 1) = 2;
    A(1, 0) = 3;
    A(1, 1) = 4;
    Mat B = Mat::identity(2);
    Mat C = A * B;
    EXPECT_DOUBLE_EQ(C(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(C(1, 1), 4.0);
}

TEST(Mat, Transpose) {
    Mat A(2, 3, 0.0);
    A(0, 2) = 7.0;
    Mat T = A.transpose();
    EXPECT_EQ(T.rows(), 3);
    EXPECT_EQ(T.cols(), 2);
    EXPECT_DOUBLE_EQ(T(2, 0), 7.0);
}

TEST(Mat, DiagFactory) {
    Mat D = Mat::diag({1.0, 2.0, 3.0});
    EXPECT_DOUBLE_EQ(D(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(D(1, 1), 2.0);
    EXPECT_DOUBLE_EQ(D(2, 2), 3.0);
    EXPECT_DOUBLE_EQ(D(0, 1), 0.0);
}

TEST(Mat, Trace) {
    Mat D = Mat::diag({1.0, 2.0, 3.0});
    EXPECT_DOUBLE_EQ(D.trace(), 6.0);
}

TEST(Mat, Scale) {
    Mat A = Mat::identity(2) * 3.0;
    EXPECT_DOUBLE_EQ(A(0, 0), 3.0);
}

TEST(Mat, Inverse2x2) {
    Mat A(2, 2);
    A(0, 0) = 4;
    A(0, 1) = 7;
    A(1, 0) = 2;
    A(1, 1) = 6;
    Mat inv = A.inverse();
    Mat I = A * inv;
    EXPECT_NEAR(I(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(I(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(I(0, 1), 0.0, 1e-9);
}
