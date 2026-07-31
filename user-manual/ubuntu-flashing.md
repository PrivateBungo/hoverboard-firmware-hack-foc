# Ubuntu build and flashing manual

This guide describes how to download, build, and flash this firmware from an
Ubuntu laptop using an ST-Link programmer. It assumes a supported STM32/GD32
hoverboard mainboard and an SWD connection.

> **Safety:** Disconnect the hoverboard battery while making wiring changes.
> Keep the wheels off the ground for the first test, and keep clear of the
> motors. Incorrect firmware settings or wiring can cause unexpected motion.

## 1. Install the Ubuntu tools

Open a terminal and install Git, the ARM compiler, Make, and the ST-Link tools:

```bash
sudo apt update
sudo apt install git make gcc-arm-none-eabi stlink-tools
```

The `stlink-tools` package supplies `st-flash`, which is the default programmer
used by the repository scripts.

## 2. Connect the programmer

Connect an ST-Link to the mainboard's SWD header:

| ST-Link | Mainboard |
| --- | --- |
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| 3.3 V reference | 3.3 V |

Double-check the pinout for your board revision before applying power. Do not
connect the ST-Link 5 V pin to a 3.3 V rail. The programmer and target must
share ground.

Confirm that Ubuntu can see the programmer:

```bash
st-info --probe
```

If this works only with `sudo`, reconnect the ST-Link after installing
`stlink-tools` so its udev rules are applied. Logging out and back in may also
be necessary.

## 3. Download (sync) the repository

For the first checkout, replace `<repository-url>` with the Git URL supplied by
the repository's **Code** button:

```bash
git clone <repository-url>
cd hoverboard-firmware-hack-foc
```

For an existing checkout, preserve local work and update it with:

```bash
cd hoverboard-firmware-hack-foc
git status
git pull --ff-only
```

Do not pull until `git status` shows that changes you care about are committed
or otherwise saved. The scripts deliberately do not run `git pull`; updating
source code and programming hardware are separate, explicit operations.

## 4. Select and review the firmware configuration

Review `Inc/config.h` before building. In particular, select the correct input
variant and verify all board, control, current, speed, and battery settings for
your hardware. Flashing defaults that do not match the wiring can be unsafe.

You can pass a variant to Make without editing the Makefile. For example:

```bash
./scripts/build.sh --clean VARIANT=VARIANT_USART
```

Omit `VARIANT=...` when the desired configuration is already selected in
`Inc/config.h`.

## 5. Build

For a clean first build, run:

```bash
./scripts/build.sh --clean
```

Later incremental builds can use:

```bash
./scripts/build.sh
```

Successful builds create `build/hover.elf`, `build/hover.hex`, and
`build/hover.bin`.

## 6. Flash

With the ST-Link attached and the target powered as required by your board,
run:

```bash
./scripts/flash.sh
```

The script erases/writes the firmware through SWD and resets the controller.
To build and immediately flash in one step, use:

```bash
./scripts/build-and-flash.sh --clean
```

You may also use ST's STM32CubeProgrammer instead of `st-flash`. Install it
from ST, then point the script at its CLI executable:

```bash
PROGRAMMER_CLI="$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI" \
  ./scripts/flash.sh
```

## 7. First start and troubleshooting

1. Disconnect the programmer before putting the enclosure back into service.
2. Keep driven wheels unloaded and be ready to remove power during first start.
3. Verify the controls, motor direction, current limits, and fault behavior at
   low demand before applying a load.

Common problems:

- **`arm-none-eabi-gcc: command not found`:** install `gcc-arm-none-eabi`.
- **`firmware file not found`:** run `./scripts/build.sh` successfully first.
- **`st-flash is not installed`:** install `stlink-tools`.
- **No ST-Link found:** check USB, SWDIO, SWCLK, target voltage, and common
  ground; then rerun `st-info --probe`.
- **Permission denied opening USB:** reconnect the programmer after package
  installation, or log out and back in so Ubuntu reloads the udev permissions.
- **A stale or surprising build:** rerun `./scripts/build.sh --clean` with the
  intended variant.
