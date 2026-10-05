#include <gtest/gtest.h>
#include <cerrno>
#include <csignal>
#include <sys/time.h>
#include <unistd.h>

#include "api/frame_wait.hpp"

TEST(FrameWait, ReportsTimeoutInsteadOfStaleErrno) {
    int descriptors[2];
    ASSERT_EQ(pipe(descriptors), 0);
    errno = ENOTTY;
    EXPECT_FALSE(Withrobot::wait_for_frame(descriptors[0], 0));
    EXPECT_EQ(errno, ETIMEDOUT);
    close(descriptors[0]);
    close(descriptors[1]);
}

TEST(FrameWait, RecognizesReadableDescriptor) {
    int descriptors[2];
    ASSERT_EQ(pipe(descriptors), 0);
    ASSERT_EQ(write(descriptors[1], "x", 1), 1);
    EXPECT_TRUE(Withrobot::wait_for_frame(descriptors[0], 0));
    close(descriptors[0]);
    close(descriptors[1]);
}

TEST(FrameWait, ReturnsInterruptedWaitToCaller) {
    int descriptors[2];
    ASSERT_EQ(pipe(descriptors), 0);
    struct sigaction action {}, previous {};
    action.sa_handler = [](int) {};
    sigemptyset(&action.sa_mask);
    ASSERT_EQ(sigaction(SIGALRM, &action, &previous), 0);
    struct itimerval timer {};
    timer.it_value.tv_usec = 10000;
    ASSERT_EQ(setitimer(ITIMER_REAL, &timer, nullptr), 0);
    EXPECT_FALSE(Withrobot::wait_for_frame(descriptors[0], 1));
    EXPECT_EQ(errno, EINTR);
    timer.it_value.tv_usec = 0;
    setitimer(ITIMER_REAL, &timer, nullptr);
    sigaction(SIGALRM, &previous, nullptr);
    close(descriptors[0]);
    close(descriptors[1]);
}

TEST(FrameWait, ReturnsDescriptorErrorsToCaller) {
    int descriptors[2];
    ASSERT_EQ(pipe(descriptors), 0);
    close(descriptors[0]);
    EXPECT_FALSE(Withrobot::wait_for_frame(descriptors[0], 0));
    EXPECT_EQ(errno, EBADF);
    close(descriptors[1]);
}
