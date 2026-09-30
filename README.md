# TIE Fighter on the ESP32-P4

This project runs the 1995 DOS CD version of [Star Wars: TIE Fighter](https://en.wikipedia.org/wiki/Star_Wars:_TIE_Fighter) on an ESP32-P4 board.

The project consists of two main parts:
1. an ahead-of-time binary translator that converts x86 game binaries into C code
2. an emulator which provides services to the translated code at run time and interprets small portions of code which aren't translated.

The emulator supports the following features:
- x86 interpreter
- DOS, DOS/4GW, BIOS
- interrupt controller, time, VGA, keyboard controller, Sound Blaster 16 and its OPL3 FM synthesizer.

The whole thing runs just a bit faster than necessary for most in-game scenes. Loading transitions and occasional in-game situations may cause the emulator to lag.

## Supported hardware

The project runs on [Waveshare ESP32-P4 DEV-KIT](https://docs.waveshare.com/ESP32-P4-Module-DEV-KIT) with the 7-DSI-TOUCH-A 7" panel. The code is structured so that supporting other boards should be straightforward, see `components/board`.

ESP32-P4 rev. 1.x chips with 360 MHz CPU frequency are sufficient to run this project. 3.x chips with 400 MHz CPU frequency will also work, of course.

The following board features are used:

- LCD panel
- speaker
- USB-A port(s) for a keyboard and a mouse
- microSD slot for the game's files

## What you need

- The 1995 CD release of the game
  - I recommend getting it [here from GOG](https://www.gog.com/en/game/star_wars_tie_fighter_special_edition) if you don't have already, specifically its Linux installer:`gog_star_wars_tie_fighter_special_edition_2.1.0.3.sh`.
- The board above
- A microSD card at least 1 GB, formatted as FAT32.
- ESP-IDF from the master branch (v6.1-dev-... at the time of writing)
- Python 3 with [capstone](https://www.capstone-engine.org/) package

## Extracting the game and preparing SD card

Create two directories on the SD card:

| On the card | DOS Drive | Contents |
|---|---|---|
| `/tie/c` | C: | The installed game: `Z_TIE__.EXE`, `IMUSE.EXE`, the `RESOURCE`, `MISSION` and other directories. |
| `/tie/d` | D: | The files on the game's CD. |

The game writes its pilot files (`*.TFR`) and settings (`*.CFG`) to `C:`, which is mapped to `/tie/c` on SD card.

If you got the above mentioned Linux edition from GOG, you can use `tools/sdcard.py` to prepare the card:

```sh
python3 tools/sdcard.py ~/Downloads/gog_star_wars_tie_fighter_special_edition_2.1.0.3.sh
```
This creates `sdcard/tie/c` and `sdcard/tie/d` directories here in the repository.

Note: `sdcard/tie/c` subdirectory is also read at project build time as part of AoT binary translation.

Copy `sdcard/tie` to the root of the card:

```sh
cp -r sdcard/tie /media/$USER/SDCARD/
```
(or `/Volumes/SDCARD/` on Mac)

## Building

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # capstone, for the translator
. $IDF_PATH/export.sh
idf.py build flash monitor
```

`idf.py menuconfig` has the project's options under *TIE Fighter emulator*:

- the board
- the speaker maximum volume (you can reduce volume later via in-game menus)
- the SD card's bus speed

## Playing

The board starts the game when it powers up. Plug a keyboard and a mouse into the USB-A ports.

## License

This project is licensed under the Apache License 2.0; see `LICENSE`.

## Third-party code

`components/machine/opl3.c` is a port of the YMF262 emulation in [ymfm](https://github.com/aaronsgiles/ymfm) by Aaron Giles, BSD-3-Clause; see `components/machine/LICENSE.ymfm`.
