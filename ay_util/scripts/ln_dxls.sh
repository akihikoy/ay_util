#!/bin/bash
#\file    ln_dxls.sh
#\brief   Find Dynamixel U2D2 devices and create symbolic links.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.17, 2025
#--------------------------------------------------
TARGET_DEVICE_KEY="usb-FTDI_USB__-__Serial_Converter"
DXLDEV_PREFIX="/media/usb_dxl"
DXLDEV_NUM=2
DXLDEV_IDX_START=1
#--------------------------------------------------

usage="`basename $0` [OPTIONS]
Find Dynamixel U2D2 devices and create symbolic links.
  OPTIONS:
    [-devn STR]   : set TARGET_DEVICE_KEY (default: $TARGET_DEVICE_KEY)
    [-pre STR]    : set DXLDEV_PREFIX (default: $DXLDEV_PREFIX)
    [-num INT]    : set DXLDEV_NUM (default: $DXLDEV_NUM)
    [-ist INT]    : set DXLDEV_IDX_START (default: $DXLDEV_IDX_START)
    [-help]       : show help
"
#--------------------------------------------------

while true; do
  case "$1" in
    -help|--help) echo "usage: $usage"; exit 0 ;;
    -devn) TARGET_DEVICE_KEY=$2; shift 2 ;;
    -pre)  DXLDEV_PREFIX=$2; shift 2 ;;
    -num)  DXLDEV_NUM=$2; shift 2 ;;
    -ist)  DXLDEV_IDX_START=$2; shift 2 ;;
    -*) echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 1 ;;
    '') break ;;
    *) echo "invalid option: $1"; echo ""; echo "usage: $usage"; exit 1 ;;
  esac
done
#--------------------------------------------------

echo "`basename $0` configuration:
  TARGET_DEVICE_KEY=$TARGET_DEVICE_KEY
  DXLDEV_PREFIX=$DXLDEV_PREFIX
  DXLDEV_NUM=$DXLDEV_NUM
  DXLDEV_IDX_START=$DXLDEV_IDX_START
"

# Initialize an empty array to hold matching device paths
MATCHING_PATHS=()

# Collect matching devices under /dev/serial/by-id
for dev in /dev/serial/by-id/*; do
  if [[ "$(basename "$dev")" == *"$TARGET_DEVICE_KEY"* ]]; then
    MATCHING_PATHS+=("$dev")
  fi
done

# List the matched paths
echo "[$TARGET_DEVICE_KEY] Dynamixel device(s) found:"
for path in "${MATCHING_PATHS[@]}"; do
  echo "  $path"
done
echo ""

echo "Following symbolic links are created:"
num_devices=$(( ${#MATCHING_PATHS[@]} < $DXLDEV_NUM ? ${#MATCHING_PATHS[@]} : $DXLDEV_NUM ))
for (( i=0; i<num_devices; i++ )); do
  echo "  $DXLDEV_PREFIX$((i+DXLDEV_IDX_START)) --> ${MATCHING_PATHS[$i]}"
done
echo ""

#--------------------------------------------------

for (( i=0; i<num_devices; i++ )); do
  echo "$DXLDEV_PREFIX$((i+DXLDEV_IDX_START)) --> ${MATCHING_PATHS[$i]}"
  sudo ln -is ${MATCHING_PATHS[$i]} $DXLDEV_PREFIX$((i+DXLDEV_IDX_START))
done

echo "Finished."
