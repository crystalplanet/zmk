# Copyright (c) 2021 Kuba Birecki
# SPDX-License-Identifier: MIT

board_runner_args(nrfjprog "--nrf-family=NRF52")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
