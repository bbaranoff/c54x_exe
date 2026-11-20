#!/bin/bash
# isoler.sh - reserve CPU cores for the real-time chain (c54x_exe, QEMU, ...) and keep the desktop off them.
#
#   ./isoler.sh on        1. cores: user.slice / system.slice / init.scope restricted to the OTHER cpus
#                            (systemd AllowedCPUs=, runtime: gone at reboot), movable IRQs moved away too
#                         2. priority: writes /run/c54x-isol.env; run.sh reads it and launches the chain
#                            with taskset -c <reserved> chrt -f <prio> (its RT_CPUS / RT_FIFO), in banc.slice
#                         3. frequency: performance profile + governor, warns when on battery
#   ./isoler.sh off       undo everything
#   ./isoler.sh status    what is in place, who runs on the reserved cpus
#   ./isoler.sh grub      4. print (does NOT apply) the boot-time isolation: isolcpus / nohz_full / rcu_nocbs
#
#   CPUS=12-15   reserved logical cpus (default: the last two physical cores, with their SMT siblings)
#   FIFO=20      SCHED_FIFO priority of the chain (stay below 50; FIFO= empty: keep run.sh's nice)
#
# run.sh picks this up by itself (launch only; RT_CPUS / RT_FIFO in the environment keep priority,
# ISOL_FAIT=1 ./run.sh ignores the isolation). Nothing here survives a reboot.
set -uo pipefail
ENVF=/run/c54x-isol.env
ETAT=/run/c54x-isol.etat
TRANCHES=(user.slice system.slice init.scope)
dire()  { printf '\033[1m[isoler]\033[0m %s\n' "$*"; }
rater() { printf '\033[1;31m[isoler] FAILED:\033[0m %s\n' "$*" >&2; exit 1; }

# expand a cpu list "0-3,8" into "0 1 2 3 8"
etendre() { local p a b o=(); for p in ${1//,/ }; do a=${p%-*}; b=${p#*-}; o+=($(seq "$a" "$b")); done; echo "${o[*]}"; }
# the last two physical cores, all their threads
defaut_cpus() {
    local c; c="$(lscpu -p=CORE | grep -v '^#' | sort -un | tail -2 | tr '\n' ' ')"
    lscpu -p=CPU,CORE | grep -v '^#' | awk -F, -v c=" $c" 'index(c, " "$2" ")' | cut -d, -f1 | sort -n | paste -sd,
}
CPUS="${CPUS:-$( [ -r "$ENVF" ] && . "$ENVF" && echo "$ISOL_CPUS" || defaut_cpus)}"
FIFO="${FIFO-20}"
TOUS="$(cat /sys/devices/system/cpu/online)"
AUTRES="$(comm -23 <(etendre "$TOUS" | tr ' ' '\n' | sort) <(etendre "$CPUS" | tr ' ' '\n' | sort) | sort -n | paste -sd,)"
masque() { local m=0 c; for c in $(etendre "$1"); do m=$(( m | 1 << c )); done; printf '%x' "$m"; }

irq_ecarter() {   # movable IRQs off the reserved cpus (managed ones refuse the write: left alone)
    local f n=0; : > "$ETAT.irq"
    for f in /proc/irq/[0-9]*/smp_affinity_list; do
        local v; v="$(cat "$f" 2>/dev/null)" || continue
        if echo "$AUTRES" > "$f" 2>/dev/null; then echo "$f $v" >> "$ETAT.irq"; n=$((n + 1)); fi
    done
    dire "   $n IRQs moved to cpus $AUTRES"
}
irq_rendre() {
    [ -f "$ETAT.irq" ] || return 0
    local f v; while read -r f v; do echo "$v" > "$f" 2>/dev/null; done < "$ETAT.irq"; rm -f "$ETAT.irq"
}
frequence() {   # $1 = performance | restore
    local g
    if [ "$1" = performance ]; then
        { command -v powerprofilesctl >/dev/null && powerprofilesctl get; } > "$ETAT.profil" 2>/dev/null
        cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor > "$ETAT.gouv" 2>/dev/null
        [ -s "$ETAT.profil" ] && powerprofilesctl set performance 2>/dev/null
        for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$g" 2>/dev/null; done
        for g in /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference; do echo performance > "$g" 2>/dev/null; done
    else
        [ -s "$ETAT.gouv" ] && for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do cat "$ETAT.gouv" > "$g" 2>/dev/null; done
        [ -s "$ETAT.profil" ] && powerprofilesctl set "$(cat "$ETAT.profil")" 2>/dev/null
        rm -f "$ETAT.profil" "$ETAT.gouv"
    fi
}
secteur() {
    local f; for f in /sys/class/power_supply/*/online; do [ -r "$f" ] || continue; [ "$(cat "$f")" = 1 ] && return 0; done
    ls /sys/class/power_supply/*/online >/dev/null 2>&1 || return 0   # no supply info: a desktop
    return 1
}

case "${1:-status}" in
on)
    [ "$(id -u)" = 0 ] || rater "root needed"
    [ -n "$AUTRES" ] || rater "CPUS=$CPUS leaves nothing for the rest of the system"
    dire "1. reserved cpus $CPUS; desktop and services on $AUTRES"
    for t in "${TRANCHES[@]}"; do systemctl set-property --runtime "$t" AllowedCPUs="$AUTRES" || rater "AllowedCPUs on $t"; done
    irq_ecarter
    printf 'ISOL_CPUS=%s\nISOL_AUTRES=%s\nISOL_FIFO=%s\n' "$CPUS" "$AUTRES" "$FIFO" > "$ENVF"
    dire "2. chain: taskset -c $CPUS ${FIFO:+chrt -f $FIFO }(read by run.sh from $ENVF)"
    [ -s "$ETAT.gouv" ] || frequence performance
    dire "3. governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)$(command -v powerprofilesctl >/dev/null && echo ", profile $(powerprofilesctl get)")"
    secteur || printf '\033[1;33m[isoler] on BATTERY: the laptop throttles, plug it in\033[0m\n'
    dire "a chain already running keeps its old placement: ./run.sh --stop then ./run.sh" ;;
off)
    [ "$(id -u)" = 0 ] || rater "root needed"
    for t in "${TRANCHES[@]}"; do
        systemctl set-property --runtime "$t" AllowedCPUs="$TOUS"
        rm -f "/run/systemd/system.control/$t.d/50-AllowedCPUs.conf"
    done
    systemctl daemon-reload
    irq_rendre; frequence restore; rm -f "$ENVF"
    dire "isolation removed: every cpu ($TOUS) back to the system, governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)" ;;
status)
    if [ -r "$ENVF" ]; then dire "ON: reserved $CPUS, rest $AUTRES, FIFO ${FIFO:-none}"; else dire "OFF (would reserve $CPUS)"; fi
    for t in "${TRANCHES[@]}"; do printf '  %-13s effective cpus %s\n' "$t" "$(cat "/sys/fs/cgroup/$t/cpuset.cpus.effective" 2>/dev/null || echo "$TOUS (no cpuset)")"; done
    printf '  governor      %s   boost %s   mains %s\n' "$(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)" \
        "$(cat /sys/devices/system/cpu/cpufreq/boost 2>/dev/null || echo '?')" "$(secteur && echo yes || echo NO)"
    printf '  boot          isolcpus=%s nohz_full=%s\n' "$(cat /sys/devices/system/cpu/isolated)" "$(grep -v null /sys/devices/system/cpu/nohz_full 2>/dev/null)"
    echo "  user processes still ALLOWED on the reserved cpus (pid policy rtprio allowed command; none = clean):"
    local_m=$(( 0x$(masque "$CPUS") ))
    for st in /proc/[0-9]*/status; do
        d=${st%/status}; [ -s "$d/cmdline" ] || continue          # kernel threads: step 4 (grub)
        m="$(awk '/^Cpus_allowed:/ { gsub(",", "", $2); print $2 }' "$st" 2>/dev/null)"; [ -n "$m" ] || continue
        (( 0x$m & local_m )) && printf '    %s %s %s\n' "${d#/proc/}" "$(ps -o cls=,rtprio= -p "${d#/proc/}" | tr -s ' ')" "$(awk '/^Cpus_allowed_list:/ { print $2 }' "$st") $(cat "$d/comm")"
    done | head -20 ;;
grub)
    cat <<EOF
Step 4, only if 'on' is not enough (kernel threads and the timer tick still visit the reserved cpus).
In /etc/default/grub, append to GRUB_CMDLINE_LINUX_DEFAULT:

    isolcpus=$CPUS nohz_full=$CPUS rcu_nocbs=$CPUS

then:  update-grub && reboot      (check: cat /sys/devices/system/cpu/isolated)
The cpus then stay empty unless a process is pinned there: keep running ./isoler.sh on after each boot,
it is what makes run.sh pin the chain. Undo: remove the three options, update-grub, reboot.
EOF
    ;;
*)  sed -n '2,17p' "$0" ;;
esac
