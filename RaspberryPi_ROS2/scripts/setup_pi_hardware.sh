#!/bin/bash
# ──────────────────────────────────────────────────────────────────────
# AMR — one-time hardware interface setup on the Raspberry Pi 5.
#
#   sudo bash scripts/setup_pi_hardware.sh                       # 16 MHz MCP2515, 1 Mbit/s, INT on GPIO25
#   sudo bash scripts/setup_pi_hardware.sh --oscillator 8000000 --bitrate 500000
#   sudo bash scripts/setup_pi_hardware.sh --usb                 # USB CAN adapter (gs_usb): no overlay
#
# What it does (safe to run again):
#   1. /boot/firmware/config.txt: SPI + the MCP2515 overlay (can0), I2C (IMU), full USB current
#   2. can0.service: brings can0 up at boot with an explicit bit timing
#   3. reminds you to reboot
#
# Why an explicit bit timing: the hoverboard board has no crystal (internal RC oscillator,
# up to ~2 % off). The kernel's default timing uses a synchronisation jump width of 1 time
# quantum, which tolerates only about half of that. The timing below matches the one the
# body controller (Blackpill) computes for the same crystal and bitrate.
#
# Wiring and the full pinout: ELECTRICAL.md in the repository root.
# ──────────────────────────────────────────────────────────────────────
set -euo pipefail

OSCILLATOR=16000000
BITRATE=1000000
INTERRUPT=25
USB=false

while [ $# -gt 0 ]; do
    case "$1" in
        --oscillator) OSCILLATOR="$2"; shift 2 ;;
        --bitrate)    BITRATE="$2"; shift 2 ;;
        --interrupt)  INTERRUPT="$2"; shift 2 ;;
        --usb)        USB=true; shift ;;
        -h|--help)    sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)"; exit 1 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash $0 $*"
    exit 1
fi

# ── Bit timing (same layouts as Blackpill_CAN_Software/lib/MCP2515) ───
# The MCP2515's time quantum is 2 * BRP / Fosc (the kernel driver's clock is Fosc / 2).
if $USB; then
    # The adapter's clock is not known here: let the kernel compute the timing, widen the SJW.
    TIMING="bitrate ${BITRATE} sample-point 0.75 sjw 2"
    SUMMARY="${BITRATE} bit/s, kernel timing, SJW 2"
else
    CLOCK=$((OSCILLATOR / 2))
    RATIO=0
    if [ $((CLOCK % BITRATE)) -eq 0 ]; then
        RATIO=$((CLOCK / BITRATE))    # time quanta per bit x prescaler
    fi
    if   [ "$RATIO" -ge 16 ] && [ $((RATIO % 16)) -eq 0 ]; then NTQ=16; PROP=4; PS1=6; PS2=5; SJW=4    # sample point 68.75 %
    elif [ "$RATIO" -ge 10 ] && [ $((RATIO % 10)) -eq 0 ]; then NTQ=10; PROP=2; PS1=4; PS2=3; SJW=2    # 70 %
    elif [ "$RATIO" -ge 8 ]  && [ $((RATIO % 8))  -eq 0 ]; then NTQ=8;  PROP=1; PS1=3; PS2=3; SJW=2    # 62.5 %
    else
        echo "✗ ${BITRATE} bit/s is not reachable with a ${OSCILLATOR} Hz crystal"
        echo "  (an 8 MHz crystal reaches at most 500 kbit/s)."
        exit 1
    fi
    TQ_NS=$((1000000000 / (BITRATE * NTQ)))
    TIMING="tq ${TQ_NS} prop-seg ${PROP} phase-seg1 ${PS1} phase-seg2 ${PS2} sjw ${SJW}"
    SUMMARY="${BITRATE} bit/s, ${NTQ} time quanta of ${TQ_NS} ns, SJW ${SJW}"
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  AMR — Raspberry Pi hardware interfaces"
echo "  CAN: ${SUMMARY}"
if $USB; then
    echo "  Adapter: USB (gs_usb)"
else
    echo "  Adapter: MCP2515 on SPI0 CE0, ${OSCILLATOR} Hz crystal, INT on GPIO${INTERRUPT}"
fi
echo "═══════════════════════════════════════════════════════════════"

# ── 1. Boot configuration ─────────────────────────────────────────────
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
if [ ! -f "$CONFIG" ]; then
    echo "✗ No config.txt found (is this a Raspberry Pi?)"
    exit 1
fi
BEGIN="# >>> AMR hardware (setup_pi_hardware.sh) >>>"
END="# <<< AMR hardware <<<"
cp -n "$CONFIG" "${CONFIG}.amr-backup" 2>/dev/null || true
# Drop the block written by an earlier run, then write the current one.
sed -i "/^${BEGIN}$/,/^${END}$/d" "$CONFIG"
{
    echo "$BEGIN"
    echo "dtparam=i2c_arm=on"
    # Powered through the GPIO header the Pi 5 cannot negotiate USB-PD and limits its USB ports
    # to 600 mA in total: not enough for two lidar motors.
    echo "usb_max_current_enable=1"
    if ! $USB; then
        echo "dtparam=spi=on"
        echo "dtoverlay=mcp2515-can0,oscillator=${OSCILLATOR},interrupt=${INTERRUPT}"
    fi
    echo "$END"
} >> "$CONFIG"
echo "  ✓ ${CONFIG} updated (original kept as ${CONFIG}.amr-backup)"

# ── 2. can0 at boot ───────────────────────────────────────────────────
cat > /etc/systemd/system/can0.service <<EOF
[Unit]
Description=AMR CAN bus (can0, ${BITRATE} bit/s)
BindsTo=sys-subsystem-net-devices-can0.device
After=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=-/sbin/ip link set can0 down
ExecStart=/sbin/ip link set can0 up type can ${TIMING} restart-ms 100
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=sys-subsystem-net-devices-can0.device
EOF
# An older setup from docs/hardware.md would fight over the interface.
rm -f /etc/systemd/network/80-can0.network
systemctl daemon-reload
systemctl enable can0.service > /dev/null
echo "  ✓ can0.service installed and enabled"

if ip link show can0 > /dev/null 2>&1; then
    systemctl restart can0.service && echo "  ✓ can0 is up" || echo "  ! can0 did not come up: journalctl -u can0"
fi

echo ""
echo "  Reboot to load the overlay:  sudo reboot"
echo "  Then check:"
echo "    ip -details link show can0      # state ERROR-ACTIVE, the timing above"
echo "    candump can0                    # 0x011 from the body controller, 0x181/0x281/0x381 from the drive"
echo "    sudo i2cdetect -y 1             # 68 = MPU9250"
echo "═══════════════════════════════════════════════════════════════"
