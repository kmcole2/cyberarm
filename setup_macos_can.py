#!/usr/bin/env python3
"""
Verify macOS CAN adapter connectivity.
Run this to confirm your AgileX USB-to-CAN adapter is detected.
"""
import usb.core
import usb.util
import sys


GS_USB_VENDOR_IDS = [
    (0x1D50, 0x606F),  # CANable / gs_usb standard
    (0x1209, 0x2323),  # CANable 2.0
    (0x0483, 0x5740),  # STM32 CDC (some AgileX adapters)
]


def find_can_adapters():
    found = []
    for vid, pid in GS_USB_VENDOR_IDS:
        devices = usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
        for dev in devices:
            found.append((vid, pid, dev))

    if not found:
        all_devices = list(usb.core.find(find_all=True))
        print(f"No known CAN adapters found. {len(all_devices)} USB devices detected.")
        print("\nAll USB devices:")
        for dev in all_devices:
            manufacturer = ""
            product = ""
            try:
                manufacturer = usb.util.get_string(dev, dev.iManufacturer) or ""
            except Exception:
                pass
            try:
                product = usb.util.get_string(dev, dev.iProduct) or ""
            except Exception:
                pass
            print(f"  VID:PID = {dev.idVendor:04x}:{dev.idProduct:04x}  {manufacturer} {product}")
        return []

    print(f"Found {len(found)} CAN adapter(s):")
    for vid, pid, dev in found:
        manufacturer = ""
        product = ""
        try:
            manufacturer = usb.util.get_string(dev, dev.iManufacturer) or ""
        except Exception:
            pass
        try:
            product = usb.util.get_string(dev, dev.iProduct) or ""
        except Exception:
            pass
        print(f"  VID:PID = {vid:04x}:{pid:04x}  {manufacturer} {product}")
    return found


def test_gs_usb_connection():
    try:
        from gs_usb.gs_usb import GsUsb
        devs = GsUsb.scan()
        if devs:
            print(f"\ngs_usb driver found {len(devs)} device(s) — ready for use!")
            return True
        else:
            print("\ngs_usb driver found no devices.")
            print("If your adapter showed up in the USB listing above,")
            print("it may use a different protocol (try slcan instead).")
            return False
    except Exception as e:
        print(f"\ngs_usb scan failed: {e}")
        return False


if __name__ == "__main__":
    print("=== macOS CAN Adapter Detection ===\n")
    adapters = find_can_adapters()
    print()
    gs_ok = test_gs_usb_connection()

    if not adapters and not gs_ok:
        print("\n--- Troubleshooting ---")
        print("1. Plug in your AgileX USB-to-CAN adapter")
        print("2. Check System Information > USB to see if it appears")
        print("3. If it shows as a serial device, try: ls /dev/tty.usb*")
        print("   and use bustype='slcan' instead of 'gs_usb'")
        sys.exit(1)

    print("\n--- Next Steps ---")
    if gs_ok:
        print("Your adapter works with gs_usb. Use bustype='gs_usb' in the SDK.")
    else:
        print("Try checking if the adapter appears as a serial port:")
        print("  ls /dev/tty.usb*")
        print("If yes, use bustype='slcan' and pass the serial port as channel.")
