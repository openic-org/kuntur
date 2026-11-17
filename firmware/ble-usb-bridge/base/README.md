# Board-support base (vendored)

Unmodified copies of the NUCLEO-WB09KE board-support files the bridge builds
against: startup code, linker script, HAL MSP, system init, BLE NVM/platform
glue, debug GPIO and USART interface, and the headers on their include paths.

Source: `openic-org/stm32ble-test`, commit `9e3679b`: directory `client/`, plus `common/ble/cpu_context_switch.s`.
Until 2026-11 the Makefile reached into a sibling checkout of that repo
(`CLIENT := ../../stm32ble-test/client`); vendoring makes the bridge build
from this repository alone. Licence: Apache-2.0 (`LICENSE`, from that repo),
with ST's own headers carrying their original notices.

Files the bridge overrides live one level up (`../Core`, `../STM32_BLE/App`)
and come first on the include path, exactly as before.
