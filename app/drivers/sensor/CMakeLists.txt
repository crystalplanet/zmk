# Copyright (c) 2020-2021 The ZMK Contributors
# SPDX-License-Identifier: MIT

add_subdirectory_ifdef(CONFIG_ZMK_BATTERY battery)
add_subdirectory_ifdef(CONFIG_CY8CMBR3106S cy8cmbr3106s)
add_subdirectory_ifdef(CONFIG_EC11 ec11)