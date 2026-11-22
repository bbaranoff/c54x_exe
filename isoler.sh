#!/bin/bash
# isoler.sh - reserve CPU cores for the real-time chain (c54x_exe, QEMU, ...), for the bridge and for the
# core network, and keep the desktop off them.
#
#   ./isoler.sh on        1. cores: user.slice / system.slice / init.scope restricted to the OTHER cpus
#                            (systemd AllowedCPUs=, runtime: gone at reboot), movable IRQs moved away too
#                         2. chain: writes /run/c54x-isol.env; run.sh reads it and launches the chain
#                            with taskset -c <reserved> chrt -f <prio> (its RT_CPUS / RT_FIFO), in banc.slice
#                         3. bridge + core network, on their own reserved cpus (CPUS_RESEAU):
#                            - the BRIDGE (pont_dsp.py / pont.py / air.py) alone on the first core of the set
#                              (run.sh: RT_CPUS_PONT), nothing else is ever scheduled there
#                            - the core network (osmo-stp/hlr/mgw/msc/bsc/ggsn/sgsn, sip-connector, asterisk,
#                              and the BTS side: osmo-bts-trx, osmo-pcu) in reseau.slice on the rest of the set:
#                              a runtime drop-in Slice=reseau.slice per unit (taken at the unit's next start),
#                              the same daemons launched outside systemd are moved there at once
#                            - a qosmo (qemu-system-arm) launched outside run.sh (the fork's 40-qemu.sh, a
#                              tmux pane) is moved to banc.slice and pinned on QEMU's core
#                         4. frequency: performance profile + governor, warns when on battery
#                         5. kernel housekeeping off the reserved cpus: unbound workqueues and the movable
#                            kernel threads (jbd2, kswapd, rcu_preempt...) on the desktop's cpus
#                         6. the real-time GUARD (c54x-garde, see below)
#   ./isoler.sh adopter   only the "moved at once" part of step 3, for daemons started after "on"
#                         (osmo-operator's start.sh calls it once the core and the BTS are up)
#   ./isoler.sh off       undo everything
#   ./isoler.sh status    what is in place, who runs on the reserved cpus
#   ./isoler.sh grub      print (does NOT apply) the boot-time isolation: isolcpus / nohz_full / rcu_nocbs
#   ./isoler.sh garde     the guard loop itself (run by "on" as the transient unit c54x-garde)
#
# THE GUARD: REAL TIME WITHOUT FREEZING THE MACHINE. A SCHED_FIFO thread that never sleeps (QEMU's TCG
# vCPU, a polling loop) leaves the kernel threads bound to its cpu (kworker, ksoftirqd) 5 % of the time
# (sched_rt_runtime_us); this kernel has no RCU_BOOST either. Work queued there then crawls, and whatever
# waits for it (file system, memory, GPU) with it: the whole desktop freezes. Such a thread gains nothing
# from SCHED_FIFO on a cpu reserved for it. c54x-garde (FIFO 50, on the desktop's cpus) keeps SCHED_FIFO
# for the threads that sleep and wake up (what real time is for) and:
#   - a user real-time thread above GARDE_BUSY % (90) of a cpu for GARDE_DUREE s (5) in a row goes back to
#     SCHED_OTHER nice GARDE_NICE (-10): still alone on its reserved cpu, but the kernel gets in again
#   - a kernel stall (RCU stall, workqueue lockup, soft lockup, hung task) or Tctl above TEMP_MAX (95 °C):
#     EVERY user real-time thread goes back to SCHED_OTHER at once
# Each action is logged (journal: -t c54x-garde, and ./isoler.sh status). GARDE=0 ./isoler.sh on: no guard.
#
#   BUREAU=4           logical cpus left to the desktop and the other services (default: 4 when the machine
#                      still has four cores for the bench then, 2 otherwise); everything else is shared
#                      between the chain and the bridge + core network, according to the number of cores of
#                      the machine (table below, "THE LAYOUT FOLLOWS THE MACHINE")
#   CPUS=8-15          cpus of the chain, to force them
#   CPUS_RESEAU=2-7    cpus of the bridge + core network, to force them (CPUS_RESEAU= empty: none, bridge and
#                      core network stay where they were)
#   FIFO=20            SCHED_FIFO priority of the chain (stay below 50; FIFO= empty: keep run.sh's nice)
#   RESEAU_UNITES="osmo-msc ..."   the units of the core network (default: the list above)
#   GARDE=1            the guard (GARDE=0: none); GARDE_BUSY=90 GARDE_DUREE=5 GARDE_NICE=-10 TEMP_MAX=95
#   RELANCER=1         ./isoler.sh on restarts the core units that are running, so that they move now
#
# run.sh picks this up by itself (launch only; RT_CPUS / RT_FIFO / RT_CPUS_PONT in the environment keep
# priority, ISOL_FAIT=1 ./run.sh ignores the isolation). Nothing here survives a reboot.
set -uo pipefail
ENVF=/run/c54x-isol.env
ETAT=/run/c54x-isol.etat
TRANCHES=(user.slice system.slice init.scope)
TRANCHE_RESEAU=reseau.slice
DROPIN=50-c54x-isol.conf
RESEAU_UNITES="${RESEAU_UNITES:-osmo-stp osmo-hlr osmo-mgw osmo-msc osmo-bsc osmo-ggsn osmo-sgsn osmo-pcu osmo-bts-trx osmo-sip-connector asterisk osmo-hnbgw osmo-cbc osmo-smlc}"
dire()  { printf '\033[1m[isoler]\033[0m %s\n' "$*"; }
alerte() { printf '\033[1;33m[isoler] %s\033[0m\n' "$*"; }
rater() { printf '\033[1;31m[isoler] FAILED:\033[0m %s\n' "$*" >&2; exit 1; }

# expand a cpu list "0-3,8" into "0 1 2 3 8"
etendre() { local p a b o=(); for p in ${1//,/ }; do a=${p%-*}; b=${p#*-}; o+=($(seq "$a" "$b")); done; echo "${o[*]-}"; }
# cpu list $1 without the cpus of $2
moins() { comm -23 <(etendre "$1" | tr ' ' '\n' | sort) <(etendre "$2" | tr ' ' '\n' | sort) | grep . | sort -n | paste -sd,; }
topo() { lscpu -p=CPU,CORE | grep -v '^#'; }                       # "cpu,core" lines
# the "cpu,core" lines of the cpu list $1, sorted by core then cpu
topo_de() { topo | awk -F, -v l=" $(etendre "$1") " 'index(l, " "$1" ")' | sort -t, -k2n -k1n; }
# THE LAYOUT FOLLOWS THE MACHINE. The desktop keeps BUREAU logical cpus, rounded up to whole physical cores:
# 4 when that still leaves four cores to the bench, 2 otherwise. Every other core goes to the bench:
#   cores for the bench   1    2    3    4    5    6    7    8   ...
#   chain                 1    2    2    2    3    3    4    4   (half, rounded up; two as soon as possible)
#   bridge + core network 0    0    1    2    2    3    3    4   (the rest)
# e.g. 8 cores / 16 threads: desktop 0-3, bridge 4-5, core network 6-9, chain 10-15 (dsp 10, qemu 12-13,
# rest 14-15); 4 cores / 8 threads: desktop 0-1, bridge 2, core network 3, chain 4-7. A single core: nothing
# is reserved. The physical cores, the best for real time first: the fast ones (hybrid cpus: the P cores,
# within 20 % of the fastest), then the highest numbered; core 0 (boot cpu, IRQs) goes to the desktop.
BUREAU="${BUREAU:-auto}"
coeurs_tries() {
    topo | sort -t, -k2n -k1n | awk -F, '!v[$2]++ {
            f = 0; if ((getline l < ("/sys/devices/system/cpu/cpu" $1 "/cpufreq/cpuinfo_max_freq")) > 0) f = l + 0
            fr[$2] = f; if (f > max) max = f }
        END { for (c in fr) print (fr[c] >= 0.8 * max), c }' | sort -k1,1nr -k2,2nr | cut -d' ' -f2
}
cpus_des_coeurs() { topo | awk -F, -v c=" $(echo $1) " 'index(c, " "$2" ")' | cut -d, -f1 | sort -n | paste -sd,; }
# how many physical cores the desktop keeps
coeurs_bureau() {
    local n t; n="$(coeurs_tries | wc -l)"; t="$(topo | wc -l)"
    t=$(( (t + n - 1) / n ))                                   # threads per core
    if [ "$BUREAU" = auto ]; then
        if [ $(( n - (4 + t - 1) / t )) -ge 4 ]; then n=$(( (4 + t - 1) / t )); else n=$(( (2 + t - 1) / t )); fi
    else
        n=$(( (BUREAU + t - 1) / t )); [ "$n" -ge 1 ] || n=1
    fi
    echo "$n"
}
defaut_cpus() {
    local m k; m=$(( $(coeurs_tries | wc -l) - $(coeurs_bureau) ))
    [ "$m" -ge 1 ] || return 0
    k=$(( (m + 1) / 2 )); [ "$m" -ge 2 ] && [ "$k" -lt 2 ] && k=2
    cpus_des_coeurs "$(coeurs_tries | head -"$k")"
}
# the cores the chain leaves, minus the desktop's
defaut_reseau() {
    local l k
    l="$(coeurs_tries | grep -vxFf <(topo_de "$CPUS" | cut -d, -f2 | sort -u; echo x))"
    k=$(( $(echo "$l" | grep -c .) - $(coeurs_bureau) ))
    [ "$k" -ge 1 ] || return 0
    cpus_des_coeurs "$(echo "$l" | head -"$k")"
}
env_lu() { [ -r "$ENVF" ] || return 1; ( . "$ENVF"; eval "echo \"\${$1-}\"" ); }
CPUS="${CPUS:-$(env_lu ISOL_CPUS || defaut_cpus)}"
FIFO="${FIFO-$(env_lu ISOL_FIFO || echo 20)}"
CPUS_RESEAU="${CPUS_RESEAU-$(env_lu ISOL_CPUS_RESEAU || defaut_reseau)}"
TOUS="$(cat /sys/devices/system/cpu/online)"
AUTRES="$(moins "$(moins "$TOUS" "$CPUS")" "$CPUS_RESEAU")"
# split of the chain's cpus (needs two cores): DSP = first thread of the first core, QEMU = the whole
# second core (its vCPU and its main loop), the rest of the chain = what is left: the DSP's SMT sibling
# with two cores; the cores after the second with three or more, and the DSP's sibling then stays idle.
# C_INSTANCES: where the other instances (INSTANCE=2, 3: mobile over the air, second TRX) run: the rest
# with three cores or more (they leave the DSP and QEMU of instance 1 alone), the whole set otherwise.
_l="$(topo_de "$CPUS")"
_c1="$(echo "$_l" | head -1 | cut -d, -f2)"; _c2="$(echo "$_l" | cut -d, -f2 | grep -vx "$_c1" | head -1)"
C_DSP=; C_QEMU=; C_RESTE=; C_INSTANCES="$CPUS"
if [ -n "$_c2" ]; then
    C_DSP="$(echo "$_l" | head -1 | cut -d, -f1)"
    C_QEMU="$(echo "$_l" | awk -F, -v c="$_c2" '$2 == c { print $1 }' | paste -sd,)"
    C_RESTE="$(echo "$_l" | awk -F, -v a="$_c1" -v c="$_c2" '$2 != c && $2 != a { print $1 }' | paste -sd,)"
    if [ -n "$C_RESTE" ]; then C_INSTANCES="$C_RESTE"
    else C_RESTE="$(echo "$_l" | awk -F, -v d="$C_DSP" -v c="$_c2" '$2 != c && $1 != d { print $1 }' | paste -sd,)"; fi
    [ -n "$C_RESTE" ] || C_RESTE="$C_QEMU"      # no SMT: the rest shares QEMU's core
fi
# split of the network cpus: two cores or more = the bridge has the whole first core (its SMT sibling stays
# idle), the core network the others; one core = the bridge on its first thread, the core network on the
# sibling; one single cpu = both on it
C_PONT=; C_CN=
if [ -n "$CPUS_RESEAU" ]; then
    _l="$(topo_de "$CPUS_RESEAU")"
    _c1="$(echo "$_l" | head -1 | cut -d, -f2)"
    if [ -n "$(echo "$_l" | cut -d, -f2 | grep -vx "$_c1" | head -1)" ]; then
        C_PONT="$(echo "$_l" | awk -F, -v c="$_c1" '$2 == c { print $1 }' | paste -sd,)"
    else
        C_PONT="$(echo "$_l" | head -1 | cut -d, -f1)"
    fi
    C_CN="$(moins "$CPUS_RESEAU" "$C_PONT")"
    [ -n "$C_CN" ] || C_CN="$C_PONT"
fi
masque() { local m=0 c; for c in $(etendre "$1"); do m=$(( m | 1 << c )); done; printf '%x' "$m"; }

irq_ecarter() {   # movable IRQs off the reserved cpus (managed ones refuse the write: left alone)
    local f n=0 v
    for f in /proc/irq/[0-9]*/smp_affinity_list; do
        v="$(cat "$f" 2>/dev/null)" || continue
        if echo "$AUTRES" > "$f" 2>/dev/null; then
            grep -q "^$f " "$ETAT.irq" 2>/dev/null || echo "$f $v" >> "$ETAT.irq"   # a second "on" keeps the ORIGINAL value
            n=$((n + 1))
        fi
    done
    dire "   $n IRQs moved to cpus $AUTRES"
}
irq_rendre() {
    [ -f "$ETAT.irq" ] || return 0
    local f v; while read -r f v; do echo "$v" > "$f" 2>/dev/null; done < "$ETAT.irq"; rm -f "$ETAT.irq"
}
# --- kernel housekeeping ----------------------------------------------------------------------------
# hex mask in the sysfs format (32-bit groups separated by commas)
masque_sysfs() {
    local c i m=0 w=() s=""
    for c in $(etendre "$1"); do w[c / 32]=$(( ${w[c / 32]:-0} | 1 << (c % 32) )); [ $(( c / 32 )) -gt "$m" ] && m=$(( c / 32 )); done
    for (( i = m; i >= 0; i-- )); do if [ "$i" = "$m" ]; then s="$(printf '%x' "${w[i]:-0}")"; else s="$s,$(printf '%08x' "${w[i]:-0}")"; fi; done
    echo "$s"
}
WQ=/sys/devices/virtual/workqueue/cpumask
noyau_ecarter() {   # unbound workqueues and movable kernel threads on the desktop's cpus
    local d n=0
    if [ -w "$WQ" ]; then
        [ -s "$ETAT.wq" ] || cat "$WQ" > "$ETAT.wq"
        masque_sysfs "$AUTRES" > "$WQ" 2>/dev/null || alerte "workqueue cpumask refused"
    fi
    for d in /proc/[0-9]*; do
        [ -z "$( { tr -d '\0' < "$d/cmdline" | head -c1; } 2>/dev/null)" ] || continue          # kernel threads only
        [ "$(awk '/^Cpus_allowed_list:/ { print $2 }' "$d/status" 2>/dev/null)" = "$TOUS" ] || continue   # per-cpu: left alone
        if taskset -pc "$AUTRES" "${d#/proc/}" >/dev/null 2>&1; then echo "${d#/proc/}" >> "$ETAT.kthr"; n=$((n + 1)); fi
    done
    dire "5. kernel housekeeping on $AUTRES: unbound workqueues, $n kernel threads (the per-cpu ones stay)"
}
noyau_rendre() {
    local p
    [ -s "$ETAT.wq" ] && [ -w "$WQ" ] && cat "$ETAT.wq" > "$WQ" 2>/dev/null
    [ -f "$ETAT.kthr" ] && while read -r p; do taskset -pc "$TOUS" "$p" >/dev/null 2>&1; done < "$ETAT.kthr"
    rm -f "$ETAT.wq" "$ETAT.kthr"
}

# --- the real-time guard (see the header) -----------------------------------------------------------
GARDE_UNITE=c54x-garde
GARDE_LOG=/run/c54x-isol.garde
GARDE_BUSY="${GARDE_BUSY:-90}"
GARDE_DUREE="${GARDE_DUREE:-5}"
GARDE_NICE="${GARDE_NICE:--10}"
TEMP_MAX="${TEMP_MAX:-95}"
noter() { logger -t "$GARDE_UNITE" -- "$*" 2>/dev/null; printf '%s %s\n' "$(date +%T)" "$*" >> "$GARDE_LOG"; }
# the real-time (FIFO/RR) threads of user processes: "tid tid ..."
threads_rt() {
    local tid cls
    ps -eLo tid=,cls= 2>/dev/null | while read -r tid cls; do
        case "$cls" in FF|RR) ;; *) continue ;; esac
        [ -n "$( { tr -d '\0' < "/proc/$tid/cmdline" | head -c1; } 2>/dev/null)" ] && echo "$tid"
    done | paste -sd' '
}
nom_thread() { printf '%s[%s/%s]' "$(cat "/proc/$1/comm" 2>/dev/null)" "$(awk '/^Tgid:/ { print $2 }' "/proc/$1/status" 2>/dev/null)" "$1"; }
retrograder() {   # $1 = tid, $2 = why
    local nom; nom="$(nom_thread "$1")"
    chrt -o -p 0 "$1" 2>/dev/null || return 0
    renice -n "$GARDE_NICE" -p "$1" >/dev/null 2>&1
    noter "$nom: SCHED_FIFO -> SCHED_OTHER nice $GARDE_NICE ($2)"
}
retrograder_tout() {   # $1 = why
    local t; noter "EMERGENCY: $1 -> every real-time thread back to SCHED_OTHER"
    for t in $(threads_rt); do retrograder "$t" "emergency"; done
}
temperature() {   # Tctl / package in degrees, empty if unknown
    local h; for h in /sys/class/hwmon/hwmon*; do
        case "$(cat "$h/name" 2>/dev/null)" in k10temp|zenpower|coretemp) echo $(( $(cat "$h/temp1_input" 2>/dev/null || echo 0) / 1000 )); return ;; esac
    done
}
garde() {
    local tick t l s tid up0 up1 dt c
    tick="$(getconf CLK_TCK)"
    declare -A prec=() chaud=()
    noter "guard active: a real-time thread above $GARDE_BUSY % for ${GARDE_DUREE} s goes back to SCHED_OTHER; kernel stall or Tctl > $TEMP_MAX °C: all of them"
    # the kernel log, from now on: stalls -> emergency
    ( dmesg -W 2>/dev/null || dmesg -w 2>/dev/null ) | while read -r l; do
        case "$l" in
            *"detected stall"*|*"rcu: INFO"*|*"workqueue lockup"*|*"soft lockup"*|*"hard LOCKUP"*|*"blocked for more than"*)
                retrograder_tout "kernel: ${l:0:140}" ;;
            *"RT throttling activated"*) noter "kernel: RT throttling activated (a real-time cpu is saturated)" ;;
        esac
    done &
    read -r up0 _ < /proc/uptime; up0=${up0/./}
    while :; do
        sleep 1
        read -r up1 _ < /proc/uptime; up1=${up1/./}; dt=$(( up1 - up0 )); up0=$up1; [ "$dt" -gt 0 ] || dt=1   # centiseconds
        declare -A cur=()
        for tid in $(threads_rt); do
            read -r l < "/proc/$tid/stat" 2>/dev/null || continue
            s=${l##*) }; set -- $s
            cur[$tid]=$(( ${12} + ${13} ))
        done
        for tid in "${!cur[@]}"; do
            # busy % = ticks * 100 / (tick * dt / 100)
            if [ -n "${prec[$tid]:-}" ] && [ $(( (cur[$tid] - prec[$tid]) * 10000 )) -ge $(( GARDE_BUSY * tick * dt )) ]; then
                chaud[$tid]=$(( ${chaud[$tid]:-0} + 1 ))
            else
                chaud[$tid]=0
            fi
            if [ "${chaud[$tid]}" -ge "$GARDE_DUREE" ]; then
                retrograder "$tid" "busy above $GARDE_BUSY % of its cpu for ${GARDE_DUREE} s: it never sleeps"; chaud[$tid]=0
            fi
        done
        prec=(); for tid in "${!cur[@]}"; do prec[$tid]=${cur[$tid]}; done
        for tid in "${!chaud[@]}"; do [ -n "${cur[$tid]:-}" ] || unset "chaud[$tid]"; done
        unset cur
        c="$(temperature)"
        if [ -n "$c" ] && [ "$c" -gt "$TEMP_MAX" ]; then retrograder_tout "Tctl $c °C > $TEMP_MAX °C"; sleep 30; fi
    done
}
garde_lancer() {
    systemctl stop "$GARDE_UNITE" 2>/dev/null; systemctl reset-failed "$GARDE_UNITE" 2>/dev/null
    if [ "${GARDE:-1}" != 1 ]; then alerte "6. NO guard (GARDE=0): a SCHED_FIFO thread that never sleeps can freeze the machine"; return 0; fi
    systemd-run --quiet --collect --unit="$GARDE_UNITE" --slice=system.slice \
        -p CPUSchedulingPolicy=fifo -p CPUSchedulingPriority=50 -p CPUAffinity="${AUTRES//,/ }" \
        --setenv=GARDE_BUSY="$GARDE_BUSY" --setenv=GARDE_DUREE="$GARDE_DUREE" --setenv=GARDE_NICE="$GARDE_NICE" \
        --setenv=TEMP_MAX="$TEMP_MAX" /bin/bash "$(readlink -f "$0")" garde \
        && dire "6. guard $GARDE_UNITE: spinning real-time threads (> $GARDE_BUSY % for ${GARDE_DUREE} s) back to SCHED_OTHER; kernel stall / Tctl > $TEMP_MAX °C: all" \
        || alerte "6. the guard $GARDE_UNITE did not start: the chain runs SCHED_FIFO without a net"
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

# --- core network ---------------------------------------------------------------------------------
unites_reseau() {   # the units of RESEAU_UNITES that exist on this machine
    local u; for u in $RESEAU_UNITES; do
        [ "$(systemctl show -p LoadState --value "$u.service" 2>/dev/null)" = loaded ] && echo "$u"
    done
}
# the core daemons that do NOT belong to a systemd service (launched by a script, a tmux pane, run.sh's
# BTS restart) and are not in the network slice yet: "pid pid ..."
errants_reseau() {
    local u p cg
    for u in $RESEAU_UNITES; do
        for p in $(pgrep -x -- "${u:0:15}" 2>/dev/null); do
            cg="$(cut -d: -f3- "/proc/$p/cgroup" 2>/dev/null)" || continue
            case "$cg" in /"$TRANCHE_RESEAU"/*|*/"$u.service") ;; *) echo "$p" ;; esac    # its own unit: a restart moves it
        done
    done | paste -sd' '
}
reseau_poser() {
    local u a_relancer=() n=0 errants
    mkdir -p /run/systemd/system
    printf '[Unit]\nDescription=Core network of the bench, on its reserved cpus (isoler.sh)\n[Slice]\nAllowedCPUs=%s\n' "$C_CN" \
        > "/run/systemd/system/$TRANCHE_RESEAU"
    for u in $(unites_reseau); do
        mkdir -p "/run/systemd/system/$u.service.d"
        printf '# isoler.sh: the core network on its reserved cpus\n[Service]\nSlice=%s\n' "$TRANCHE_RESEAU" \
            > "/run/systemd/system/$u.service.d/$DROPIN"
        n=$((n + 1))
    done
    systemctl daemon-reload
    systemctl start "$TRANCHE_RESEAU" || rater "$TRANCHE_RESEAU does not start"
    systemctl set-property --runtime "$TRANCHE_RESEAU" AllowedCPUs="$C_CN" || rater "AllowedCPUs on $TRANCHE_RESEAU"
    for u in $(unites_reseau); do
        systemctl is-active --quiet "$u.service" || continue
        [ "$(systemctl show -p ControlGroup --value "$u.service")" = "/$TRANCHE_RESEAU/$u.service" ] || a_relancer+=("$u")
    done
    dire "3. bridge alone on $C_PONT; core network ($n units, $TRANCHE_RESEAU) on $C_CN"
    if [ "${#a_relancer[@]}" -gt 0 ]; then
        if [ "${RELANCER:-0}" = 1 ]; then
            dire "   restarting ${a_relancer[*]}"
            systemctl restart "${a_relancer[@]/%/.service}" || alerte "restart failed: systemctl status ${a_relancer[*]}"
        else
            alerte "running since before the isolation, still on $AUTRES: ${a_relancer[*]}"
            alerte "   they move at their next start: systemctl restart ${a_relancer[*]}   (or RELANCER=1 ./isoler.sh on)"
        fi
    fi
    adopter
}
# a transient scope "$1-adopte-*.scope" in the slice $2 that takes the processes $3.. (their threads follow)
adopter_dans() {
    local nom="$1" tranche="$2"; shift 2
    busctl call org.freedesktop.systemd1 /org/freedesktop/systemd1 org.freedesktop.systemd1.Manager StartTransientUnit \
        'ssa(sv)a(sa(sv))' "$nom-adopte-$(date +%s%N).scope" fail 2 PIDs au "$#" "$@" Slice s "$tranche" 0 >/dev/null
}
noms_de() { local p; for p in "$@"; do printf '%s(%s) ' "$(cat "/proc/$p/comm" 2>/dev/null)" "$p"; done; }
# qemu-system-arm outside banc.slice: not launched by run.sh, so confined with the desktop
errants_qemu() {
    local p; for p in $(pgrep -x qemu-system-arm 2>/dev/null); do
        case "$(cut -d: -f3- "/proc/$p/cgroup" 2>/dev/null)" in /banc.slice/*) ;; *) echo "$p" ;; esac
    done | paste -sd' '
}
adopter() {
    local errants p
    errants="$(errants_reseau)"
    if [ -n "$errants" ] && [ -n "$C_CN" ]; then
        # shellcheck disable=SC2086
        if adopter_dans reseau "$TRANCHE_RESEAU" $errants; then
            dire "   launched outside systemd, moved to $TRANCHE_RESEAU ($C_CN) now: $(noms_de $errants)"
        else
            alerte "could not move to $TRANCHE_RESEAU: pids $errants"
        fi
    fi
    errants="$(errants_qemu)"
    if [ -n "$errants" ]; then
        # shellcheck disable=SC2086
        if adopter_dans qosmo banc.slice $errants; then
            for p in $errants; do taskset -a -cp "${C_QEMU:-$CPUS}" "$p" >/dev/null 2>&1; done
            dire "   qosmo launched outside run.sh, moved to banc.slice and pinned on ${C_QEMU:-$CPUS}: $(noms_de $errants)"
        else
            alerte "could not move qosmo to banc.slice: pids $errants"
        fi
    fi
}
reseau_oter() {
    local f
    for f in /run/systemd/system/*.service.d/"$DROPIN"; do
        [ -e "$f" ] || continue
        rm -f "$f"; rmdir "$(dirname "$f")" 2>/dev/null
    done
    if [ -e "/run/systemd/system/$TRANCHE_RESEAU" ] || systemctl is-active --quiet "$TRANCHE_RESEAU"; then
        systemctl set-property --runtime "$TRANCHE_RESEAU" AllowedCPUs="$TOUS" 2>/dev/null
    fi
    rm -f "/run/systemd/system/$TRANCHE_RESEAU" "/run/systemd/system.control/$TRANCHE_RESEAU.d/50-AllowedCPUs.conf"
    rmdir "/run/systemd/system.control/$TRANCHE_RESEAU.d" 2>/dev/null
}
# user processes ALLOWED on the cpus $1, except those whose cgroup matches the regex $2
intrus() {
    local st d m cg masq=$(( 0x$(masque "$1") ))
    for st in /proc/[0-9]*/status; do
        d=${st%/status}; [ -n "$( { tr -d '\0' < "$d/cmdline" | head -c1; } 2>/dev/null)" ] || continue   # kernel threads: step 5 (grub)
        m="$(awk '/^Cpus_allowed:/ { gsub(",", "", $2); print $2 }' "$st" 2>/dev/null)"; [ -n "$m" ] || continue
        (( 0x$m & masq )) || continue
        if [ -n "${2:-}" ]; then cg="$(cut -d: -f3- "$d/cgroup" 2>/dev/null)"; [[ "$cg" =~ $2 ]] && continue; fi
        printf '    %s %s %s\n' "${d#/proc/}" "$(ps -o cls=,rtprio= -p "${d#/proc/}" | tr -s ' ')" "$(awk '/^Cpus_allowed_list:/ { print $2 }' "$st") $(cat "$d/comm" 2>/dev/null)"
    done | head -20
}

case "${1:-status}" in
on)
    [ "$(id -u)" = 0 ] || rater "root needed"
    [ -n "$CPUS" ] || rater "a single physical core: nothing to reserve"
    [ -n "$AUTRES" ] || rater "CPUS=$CPUS CPUS_RESEAU=$CPUS_RESEAU leave nothing for the rest of the system"
    [ -z "$CPUS_RESEAU" ] || [ "$(moins "$CPUS_RESEAU" "$CPUS")" = "$(moins "$CPUS_RESEAU" "")" ] || rater "CPUS_RESEAU=$CPUS_RESEAU overlaps CPUS=$CPUS"
    [ -z "$(moins "$CPUS,${CPUS_RESEAU:-$CPUS}" "$TOUS")" ] || rater "cpus not online ($TOUS): $(moins "$CPUS,${CPUS_RESEAU:-$CPUS}" "$TOUS")"
    dire "1. reserved cpus: chain $CPUS${CPUS_RESEAU:+, bridge + core network $CPUS_RESEAU}; desktop and services on $AUTRES"
    for t in "${TRANCHES[@]}"; do systemctl set-property --runtime "$t" AllowedCPUs="$AUTRES" || rater "AllowedCPUs on $t"; done
    irq_ecarter
    printf 'ISOL_CPUS=%s\nISOL_AUTRES=%s\nISOL_FIFO=%s\nISOL_CPUS_DSP=%s\nISOL_CPUS_QEMU=%s\nISOL_CPUS_RESTE=%s\nISOL_CPUS_RESEAU=%s\nISOL_CPUS_PONT=%s\nISOL_CPUS_CN=%s\nISOL_CPUS_INSTANCES=%s\n' \
        "$CPUS" "$AUTRES" "$FIFO" "$C_DSP" "$C_QEMU" "$C_RESTE" "$CPUS_RESEAU" "$C_PONT" "$C_CN" "$C_INSTANCES" > "$ENVF"
    dire "2. chain: ${FIFO:+chrt -f $FIFO, }$([ -n "$C_DSP" ] && echo "dsp on $C_DSP, qemu on $C_QEMU, rest on $C_RESTE, other instances on $C_INSTANCES" || echo "taskset -c $CPUS") (read by run.sh from $ENVF)"
    if [ -n "$CPUS_RESEAU" ]; then reseau_poser; else reseau_oter; systemctl daemon-reload; dire "3. bridge and core network: not isolated (CPUS_RESEAU empty)"; adopter; fi
    [ -s "$ETAT.gouv" ] || frequence performance
    dire "4. governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)$(command -v powerprofilesctl >/dev/null && echo ", profile $(powerprofilesctl get)")"
    secteur || alerte "on BATTERY: the laptop throttles, plug it in"
    noyau_ecarter
    garde_lancer
    dire "a chain (and its bridge) already running keeps its old placement: ./run.sh --stop then ./run.sh" ;;
adopter)
    [ "$(id -u)" = 0 ] || rater "root needed"
    [ -r "$ENVF" ] || { dire "isolation OFF: nothing to move"; exit 0; }
    adopter ;;
off)
    [ "$(id -u)" = 0 ] || rater "root needed"
    for t in "${TRANCHES[@]}"; do
        systemctl set-property --runtime "$t" AllowedCPUs="$TOUS"
        rm -f "/run/systemd/system.control/$t.d/50-AllowedCPUs.conf"
    done
    reseau_oter
    systemctl daemon-reload
    systemctl stop "$GARDE_UNITE" 2>/dev/null
    irq_rendre; noyau_rendre; frequence restore; rm -f "$ENVF"
    dire "isolation removed: every cpu ($TOUS) back to the system, governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)" ;;
status)
    if [ -r "$ENVF" ]; then dire "ON: chain $CPUS, bridge ${C_PONT:-not isolated}, core network ${C_CN:-not isolated}, rest $AUTRES, FIFO ${FIFO:-none}"
    else dire "OFF (would reserve: chain ${CPUS:-none}${C_DSP:+ [dsp $C_DSP, qemu $C_QEMU, rest $C_RESTE]}, bridge ${C_PONT:-none}, core network ${C_CN:-none}; desktop $AUTRES)"; fi
    for t in "${TRANCHES[@]}" "$TRANCHE_RESEAU"; do printf '  %-13s effective cpus %s\n' "$t" "$(cat "/sys/fs/cgroup/$t/cpuset.cpus.effective" 2>/dev/null || echo "$TOUS (no cpuset)")"; done
    printf '  governor      %s   boost %s   mains %s\n' "$(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)" \
        "$(cat /sys/devices/system/cpu/cpufreq/boost 2>/dev/null || echo '?')" "$(secteur && echo yes || echo NO)"
    # real-time budget: a SCHED_FIFO cpu above sched_rt_runtime_us (950 ms per s) is frozen for the rest of the second
    _rt="$CPUS${C_PONT:+,$C_PONT}"
    printf '  rt budget     %s ms used of %s per second, cpus %s (throttled now: %s; since boot: %s)\n' \
        "$(for c in $(etendre "$_rt"); do grep -A12 "^rt_rq\[$c\]" /sys/kernel/debug/sched/debug 2>/dev/null | awk '/rt_time/ { printf "%d ", $3 }'; done)" \
        "$(( $(cat /proc/sys/kernel/sched_rt_runtime_us) / 1000 ))" "$_rt" \
        "$(for c in $(etendre "$_rt"); do grep -A12 "^rt_rq\[$c\]" /sys/kernel/debug/sched/debug 2>/dev/null | awk '/rt_throttled/ { printf "%d ", $3 }'; done)" \
        "$(dmesg 2>/dev/null | grep -c 'RT throttling activated')"
    printf '  guard         %s (%s)\n' "$(systemctl is-active "$GARDE_UNITE" 2>/dev/null)" "$GARDE_UNITE; journalctl -t $GARDE_UNITE"
    [ -s "$GARDE_LOG" ] && tail -5 "$GARDE_LOG" | sed 's/^/    /'
    printf '  housekeeping  workqueues %s, temperature %s °C\n' "$(cat "$WQ" 2>/dev/null)" "$(temperature)"
    printf '  boot          isolcpus=%s nohz_full=%s\n' "$(cat /sys/devices/system/cpu/isolated)" "$(grep -v null /sys/devices/system/cpu/nohz_full 2>/dev/null)"
    echo "  bridge (pid cpus-allowed on-cpu policy rtprio command; none = not running):"
    for p in $(pgrep -f 'pont/(pont_dsp|pont|air)\.py' 2>/dev/null); do
        printf '    %s %s %s %s\n' "$p" "$(awk '/^Cpus_allowed_list:/ { print $2 }' "/proc/$p/status" 2>/dev/null)" "$(ps -o psr=,cls=,rtprio= -p "$p" | tr -s ' ')" "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-60)"
    done
    echo "  core network (unit state slice main-pid cpus-allowed):"
    for u in $(unites_reseau); do
        p="$(systemctl show -p MainPID --value "$u.service")"
        printf '    %-19s %-9s %-13s %s %s\n' "$u" "$(systemctl is-active "$u.service")" "$(systemctl show -p Slice --value "$u.service")" \
            "$([ "$p" != 0 ] && echo "$p" || echo -)" "$([ "$p" != 0 ] && awk '/^Cpus_allowed_list:/ { print $2 }' "/proc/$p/status" 2>/dev/null)"
    done
    _e="$(errants_reseau)"; [ -n "$_e" ] && echo "    outside systemd and outside $TRANCHE_RESEAU (./isoler.sh adopter moves them): $_e"
    echo "  qosmo (pid cpus-allowed on-cpu policy rtprio cgroup; none = not running):"
    for p in $(pgrep -x qemu-system-arm 2>/dev/null); do
        printf '    %s %s %s %s\n' "$p" "$(awk '/^Cpus_allowed_list:/ { print $2 }' "/proc/$p/status" 2>/dev/null)" "$(ps -o psr=,cls=,rtprio= -p "$p" | tr -s ' ')" "$(cut -d: -f3- "/proc/$p/cgroup" 2>/dev/null)"
    done
    _e="$(errants_qemu)"; [ -n "$_e" ] && [ -r "$ENVF" ] && echo "    outside banc.slice (./isoler.sh adopter moves them): $_e"
    echo "  user processes still ALLOWED on the chain's and the bridge's cpus (pid policy rtprio allowed command; none = clean):"
    intrus "$_rt"
    if [ -n "$C_CN" ] && [ "$C_CN" != "$C_PONT" ]; then
        echo "  processes outside $TRANCHE_RESEAU still ALLOWED on the core network's cpus (none = clean):"
        intrus "$C_CN" "^/$TRANCHE_RESEAU(/|\$)"
    fi ;;
garde)
    [ "$(id -u)" = 0 ] || rater "root needed"
    garde ;;
grub)
    cat <<EOF
Step 5, only if 'on' is not enough (kernel threads and the timer tick still visit the reserved cpus).
In /etc/default/grub, append to GRUB_CMDLINE_LINUX_DEFAULT:

    isolcpus=$CPUS${C_PONT:+,$C_PONT} nohz_full=$CPUS${C_PONT:+,$C_PONT} rcu_nocbs=$CPUS${C_PONT:+,$C_PONT}

then:  update-grub && reboot      (check: cat /sys/devices/system/cpu/isolated)
The chain's and the bridge's cpus then stay empty unless a process is pinned there: keep running
./isoler.sh on after each boot, it is what makes run.sh pin the chain and the bridge. The core network's
cpus${C_CN:+ ($C_CN)} are left out on purpose: isolcpus switches the load balancing off, and its daemons need it
to spread over their cpus. Undo: remove the three options, update-grub, reboot.
EOF
    ;;
*)  sed -n '2,/^set -uo pipefail/p' "$0" | grep '^#' ;;
esac
