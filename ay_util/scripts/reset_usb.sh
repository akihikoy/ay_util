#!/bin/bash
#\file    reset_usb.sh
#\brief   Reset USB device by unbiding and binding a USB port.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Mar.10, 2025

# Default USB is usb2 (can be specified via command line).
USB_NUM=${1:-"usb2"}

VERBOSE=false
if [[ "$2" == "-verbose" ]]; then
  VERBOSE=true
fi
if [[ "$2" == "-noverbose" ]]; then
  VERBOSE=false
fi

USB_PATH=$(readlink -f /sys/bus/usb/devices/$USB_NUM)
echo "Detected USB Path: $USB_NUM -> $USB_PATH"

if [[ $USB_NUM != usb* ]]; then
  echo "Error: USB number should be like usb1, usb2, etc."
  exit 1
fi

FULL_PATH=$(readlink -f /sys/bus/usb/devices/$USB_NUM)

if echo "$FULL_PATH" | grep -q "/pci"; then
  echo "Detected PCI-based USB controller."
  PCI_ID=$(echo "$FULL_PATH" | grep -o '[0-9a-f]\{4\}:[0-9]\{2\}:[0-9]\{2\}\.[0-9]')
  DRIVER="xhci_hcd"
  $VERBOSE && lsusb
  echo "---"
  echo "Unbinding PCI USB controller ($PCI_ID)..."
  echo "$PCI_ID" | sudo tee /sys/bus/pci/drivers/$DRIVER/unbind
  sleep 0.5
  $VERBOSE && lsusb
  echo "---"
  sleep 0.5
  echo "Re-binding PCI USB controller ($PCI_ID)..."
  echo "$PCI_ID" | sudo tee /sys/bus/pci/drivers/$DRIVER/bind
  $VERBOSE && lsusb
  echo "---"
  sleep 2.0
  $VERBOSE && lsusb
elif echo "$FULL_PATH" | grep -q "/platform"; then
  PLATFORM_ID=$(basename $(dirname "$FULL_PATH"))
  DRIVER_PATH=$(ls -d /sys/bus/platform/drivers/*/$PLATFORM_ID 2>/dev/null)
  DRIVER_NAME=$(basename $(dirname $DRIVER_PATH))
  PLATFORM_ID=$(basename $DRIVER_PATH)
  $VERBOSE && lsusb
  echo "---"
  echo "Detected platform USB controller: $DRIVER_NAME/$PLATFORM_ID"
  echo "Unbinding platform USB controller..."
  echo "$PLATFORM_ID" | sudo tee /sys/bus/platform/drivers/$DRIVER_NAME/unbind
  sleep 0.5
  $VERBOSE && lsusb
  echo "---"
  sleep 0.5
  echo "Re-binding platform USB controller..."
  echo "$PLATFORM_ID" | sudo tee /sys/bus/platform/drivers/$DRIVER_NAME/bind
  $VERBOSE && lsusb
  echo "---"
  sleep 2.0
  $VERBOSE && lsusb
else
  echo "Unknown USB controller type. Abort."
  exit 1
fi

echo "Done."

