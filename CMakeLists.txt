cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(comotion_tracker)

target_sources(app PRIVATE
  src/main.c
  src/gps.c
  src/audio.c
  src/sdcard.c
  src/battery.c
  src/intensity.c
  src/impact.c
  src/ble_adv.c
)
