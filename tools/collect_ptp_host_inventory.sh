#!/usr/bin/env bash
# Read-only host/NIC/PTP inventory for RobDog navigation TIME-045/TIME-052.
# Does NOT configure interfaces, stop services, change clocks, or start ptp4l.
set -u
umask 077

usage() {
  cat <<'USAGE'
Usage: collect_ptp_host_inventory.sh [OUTPUT_DIR]

Collects a timestamped, read-only inventory of Linux, Jetson, NIC, PHC,
linuxptp package/version, and competing clock-discipline services.

The default output is ./ptp_host_inventory_<UTC timestamp>.
No sudo is required for the basic inventory. Some details may be unavailable
without privileges; unavailable commands are recorded rather than treated as
fatal. Review the output before sharing it externally.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then usage; exit 0; fi
if [[ "$#" -gt 1 ]]; then usage >&2; exit 2; fi

stamp="$(date -u +%Y%m%dT%H%M%SZ 2>/dev/null || printf 'unknown-time')"
out="${1:-./ptp_host_inventory_${stamp}}"
if [[ -e "$out" ]]; then
  printf 'ERROR: output path already exists: %s\n' "$out" >&2
  exit 2
fi
mkdir -p "$out" || exit 1
chmod 700 "$out" 2>/dev/null || true
index="$out/00_index.txt"

{
  echo "collection_utc=$stamp"
  echo "hostname=$(hostname 2>/dev/null || echo unavailable)"
  echo "uid=$(id -u 2>/dev/null || echo unavailable)"
  echo "output_dir=$out"
  echo "mode=read-only; no network configuration or clock changes"
} > "$index"

run_capture() {
  local name="$1"; shift
  local file="$out/$name.txt"
  {
    printf '# command:'
    printf ' %q' "$@"
    printf '\n# collected_utc: %s\n' "$(date -u +%FT%TZ 2>/dev/null || echo unavailable)"
    "$@" 2>&1 || printf '\n[command_exit_status=%s]\n' "$?"
  } > "$file"
  printf '%s\n' "$name.txt" >> "$index"
}

capture_shell() {
  local name="$1" body="$2"
  {
    printf '# collected_utc: %s\n' "$(date -u +%FT%TZ 2>/dev/null || echo unavailable)"
    bash -c "$body" 2>&1 || printf '\n[command_exit_status=%s]\n' "$?"
  } > "$out/$name.txt"
  printf '%s\n' "$name.txt" >> "$index"
}

run_capture "01_os_release" cat /etc/os-release
run_capture "02_kernel" uname -a
run_capture "03_architecture" dpkg --print-architecture
capture_shell "04_jetson_release" 'for f in /etc/nv_tegra_release /etc/nv_boot_control.conf; do if [ -r "$f" ]; then echo "### $f"; cat "$f"; fi; done'
capture_shell "05_device_tree_model" 'for f in /proc/device-tree/model /proc/device-tree/compatible; do if [ -r "$f" ]; then echo "### $f"; tr "\000" "\n" < "$f"; fi; done'
run_capture "06_clock_status" timedatectl status
capture_shell "07_clock_services" 'for s in chrony chronyd systemd-timesyncd ntp ntpd ptp4l phc2sys; do printf "### %s: " "$s"; systemctl is-active "$s" 2>&1 || true; done; echo "### timedatectl NTP"; timedatectl show -p NTP -p NTPSynchronized 2>&1 || true'
capture_shell "08_packages" 'if command -v dpkg-query >/dev/null 2>&1; then dpkg-query -W -f="${binary:Package}\t${Version}\n" "linuxptp*" "ethtool*" "chrony*" "systemd-timesyncd" 2>&1 || true; fi; for c in ptp4l phc2sys pmc ethtool; do if command -v "$c" >/dev/null 2>&1; then echo "### $c"; "$c" -v 2>&1 | head -n 5; fi; done'
run_capture "09_ip_link" ip -details -statistics link show
run_capture "10_ip_addr" ip -brief address show
capture_shell "11_interfaces" 'for p in /sys/class/net/*; do [ -e "$p" ] || continue; i="${p##*/}"; echo "### $i"; for f in address operstate mtu speed duplex; do [ -r "$p/$f" ] && printf "%s=" "$f" && cat "$p/$f"; done; [ -r "$p/device/uevent" ] && sed -n "1,20p" "$p/device/uevent"; done'
capture_shell "12_ethtool_inventory" 'for p in /sys/class/net/*; do [ -e "$p" ] || continue; i="${p##*/}"; [ "$i" = lo ] && continue; echo "===== $i: driver ====="; ethtool -i "$i" 2>&1 || true; echo "===== $i: timestamp capabilities ====="; ethtool -T "$i" 2>&1 || true; done'
capture_shell "13_phc_inventory" 'for p in /sys/class/ptp/ptp*; do [ -e "$p" ] || continue; echo "### $p"; for f in clock_name max_adjustment n_alarms n_ext_ts n_pins n_per_out; do [ -r "$p/$f" ] && printf "%s=" "$f" && cat "$p/$f"; done; done; echo "### /dev/ptp*"; ls -l /dev/ptp* 2>&1 || true'
capture_shell "14_lspci" 'if command -v lspci >/dev/null 2>&1; then lspci -nnk; else echo "lspci unavailable"; fi'
capture_shell "15_dmesg_ptp_net" 'dmesg 2>&1 | grep -Ei "ptp|phc|timestamp|mgbe|eqos|ethernet|phy|firmware" | tail -n 400 || true'
capture_shell "16_kernel_ptp_interfaces" 'for p in /sys/class/net/*/device/ptp; do [ -e "$p" ] && { echo "### $p"; readlink -f "$p"; }; done; for p in /sys/class/net/*/device/ptp_index; do [ -r "$p" ] && { echo "### $p"; cat "$p"; }; done'
capture_shell "17_irq_summary" 'cat /proc/interrupts 2>/dev/null | grep -Ei "mgbe|eqos|ethernet|ptp|phy" || true'
capture_shell "18_network_offloads" 'for p in /sys/class/net/*; do [ -e "$p" ] || continue; i="${p##*/}"; [ "$i" = lo ] && continue; echo "===== $i ====="; ethtool -k "$i" 2>&1 | grep -Ei "timestamp|vlan|gro|gso|tso|rx-checksum|tx-checksum" || true; done'
capture_shell "19_processes" 'ps -eo pid,comm,args 2>/dev/null | grep -E "[p]tp4l|[p]hc2sys|[c]hronyd|[n]tpd|[s]ystemd-timesyncd" || true'
capture_shell "20_limits_and_permissions" 'printf "CAP_NET_ADMIN/CAP_SYS_TIME are not modified by this script.\n"; id; ls -l /dev/ptp* 2>&1 || true'
capture_shell "21_pmc_read_only_datasets" 'if command -v pmc >/dev/null 2>&1; then d="${PTP_DOMAIN:-0}"; echo "domain=$d"; for ds in PORT_DATA_SET PARENT_DATA_SET CURRENT_DATA_SET TIME_PROPERTIES_DATA_SET TIME_STATUS_NP; do echo "===== GET $ds ====="; pmc -u -b 0 -d "$d" "GET $ds" 2>&1 || echo "[pmc_get_failed=$ds]"; done; else echo "pmc unavailable"; fi'
capture_shell "22_ptp_service_journal" 'if command -v journalctl >/dev/null 2>&1; then journalctl -u ptp4l -u phc2sys -n 300 --no-pager 2>&1 || true; else echo "journalctl unavailable"; fi'
(
  cd "$out" || exit 1
  find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 |
    sort -z |
    xargs -0 -r sha256sum > SHA256SUMS
)
printf 'Inventory written to: %s\n' "$out"
printf 'Review files for device identifiers before sharing.\n'
