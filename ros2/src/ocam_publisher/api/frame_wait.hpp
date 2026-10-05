#pragma once

// Extracted from the Withrobot Camera API, Copyright (C) 2016 Withrobot. Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cerrno>
#include <sys/select.h>

namespace Withrobot {

inline bool wait_for_frame(int fd, unsigned int timeout_sec) {
    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(fd, &descriptors);
    timeval timeout {static_cast<time_t>(timeout_sec), 0};
    const int result = select(fd + 1, &descriptors, nullptr, nullptr, &timeout);
    if (result == 0) errno = ETIMEDOUT;
    return result > 0;
}

}
