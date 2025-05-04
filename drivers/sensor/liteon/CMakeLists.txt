# Copyright (c) 2025 Konrad Sikora
# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_LTR329 ltr329)
add_subdirectory_ifdef(CONFIG_LTR_F216A ltrf216a)
add_subdirectory_ifdef(CONFIG_LTR390 ltr390)
# zephyr-keep-sorted-stop
