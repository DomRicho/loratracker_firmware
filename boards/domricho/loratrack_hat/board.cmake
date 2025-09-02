# west flash -r openocd
 
# west flash -r dfu-util
 
board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")
 
board_runner_args(jlink "--device=STM32H7B0RB" "--speed=4000")
 
board_runner_args(pyocd "--target=STM32H7")
 
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
 
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
 
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
 
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
 
include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
