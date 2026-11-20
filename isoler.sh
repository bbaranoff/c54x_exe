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
#   ./isoler.sh adopter   only the "moved at once" part of step 3, for daemons started after "on"
#                         (osmo-operator's start.sh calls it once the core and the BTS are up)
#   ./isoler.sh off       undo everything
#   ./isoler.sh status    what is in place, who runs on the reserved cpus
#   ./isoler.sh grub      5. print (does NOT apply) the boot-time isolation: isolcpus / nohz_full / rcu_nocbs
#
#   CPUS=12-15         cpus of the chain (default: the last two physical cores, with their SMT siblings)
#   CPUS_RESEAU=8-11   cpus of the bridge + core network (default: the two cores before the chain's when six
#                      cores or more are left, one core when three to five are left; CPUS_RESEAU= empty: none,
#                      bridge and core network stay where they were)
#   FIFO=20            SCHED_FIFO priority of the chain (stay below 50; FIFO= empty: keep run.sh's nice)
#   RESEAU_UNITES="osmo-msc ..."   the units of the core network (default: the list above)
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
# the last two physical cores, all their threads
defaut_cpus() {
    local c; c="$(lscpu -p=CORE | grep -v '^#' | sort -un | tail -2 | tr '\n' ' ')"
    topo | awk -F, -v c=" $c" 'index(c, " "$2" ")' | cut -d, -f1 | sort -n | paste -sd,
}
# among the cores the chain leaves: the last two (six or more left), the last one (three to five), none
defaut_reseau() {
    local l n k c
    l="$(topo | awk -F, -v l=" $(etendre "$CPUS") " '!index(l, " "$1" ") { print $2 }' | sort -un)"
    n="$(echo "$l" | grep -c .)"
    if [ "$n" -ge 6 ]; then k=2; elif [ "$n" -ge 3 ]; then k=1; else return 0; fi
    c="$(echo "$l" | tail -"$k" | tr '\n' ' ')"
    topo | awk -F, -v c=" $c" 'index(c, " "$2" ")' | cut -d, -f1 | sort -n | paste -sd,
}
env_lu() { [ -r "$ENVF" ] || return 1; ( . "$ENVF"; eval "echo \"\${$1-}\"" ); }
CPUS="${CPUS:-$(env_lu ISOL_CPUS || defaut_cpus)}"
FIFO="${FIFO-$(env_lu ISOL_FIFO || echo 20)}"
CPUS_RESEAU="${CPUS_RESEAU-$(env_lu ISOL_CPUS_RESEAU || defaut_reseau)}"
TOUS="$(cat /sys/devices/system/cpu/online)"
AUTRES="$(moins "$(moins "$TOUS" "$CPUS")" "$CPUS_RESEAU")"
# split of the chain's cpus (needs two cores): DSP = first thread of the first core, QEMU = the whole
# second core (its vCPU and its main loop), the rest of the chain = what is left
_l="$(topo_de "$CPUS")"
_c1="$(echo "$_l" | head -1 | cut -d, -f2)"; _c2="$(echo "$_l" | cut -d, -f2 | grep -vx "$_c1" | head -1)"
C_DSP=; C_QEMU=; C_RESTE=
if [ -n "$_c2" ]; then
    C_DSP="$(echo "$_l" | head -1 | cut -d, -f1)"
    C_QEMU="$(echo "$_l" | awk -F, -v c="$_c2" '$2 == c { print $1 }' | paste -sd,)"
    C_RESTE="$(echo "$_l" | awk -F, -v d="$C_DSP" -v c="$_c2" '$2 != c && $1 != d { print $1 }' | paste -sd,)"
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
    [ -n "$AUTRES" ] || rater "CPUS=$CPUS CPUS_RESEAU=$CPUS_RESEAU leave nothing for the rest of the system"
    [ -z "$CPUS_RESEAU" ] || [ "$(moins "$CPUS_RESEAU" "$CPUS")" = "$(moins "$CPUS_RESEAU" "")" ] || rater "CPUS_RESEAU=$CPUS_RESEAU overlaps CPUS=$CPUS"
    [ -z "$(moins "$CPUS,${CPUS_RESEAU:-$CPUS}" "$TOUS")" ] || rater "cpus not online ($TOUS): $(moins "$CPUS,${CPUS_RESEAU:-$CPUS}" "$TOUS")"
    dire "1. reserved cpus: chain $CPUS${CPUS_RESEAU:+, bridge + core network $CPUS_RESEAU}; desktop and services on $AUTRES"
    for t in "${TRANCHES[@]}"; do systemctl set-property --runtime "$t" AllowedCPUs="$AUTRES" || rater "AllowedCPUs on $t"; done
    irq_ecarter
    printf 'ISOL_CPUS=%s\nISOL_AUTRES=%s\nISOL_FIFO=%s\nISOL_CPUS_DSP=%s\nISOL_CPUS_QEMU=%s\nISOL_CPUS_RESTE=%s\nISOL_CPUS_RESEAU=%s\nISOL_CPUS_PONT=%s\nISOL_CPUS_CN=%s\n' \
        "$CPUS" "$AUTRES" "$FIFO" "$C_DSP" "$C_QEMU" "$C_RESTE" "$CPUS_RESEAU" "$C_PONT" "$C_CN" > "$ENVF"
    dire "2. chain: ${FIFO:+chrt -f $FIFO, }$([ -n "$C_DSP" ] && echo "dsp on $C_DSP, qemu on $C_QEMU, rest on $C_RESTE" || echo "taskset -c $CPUS") (read by run.sh from $ENVF)"
    if [ -n "$CPUS_RESEAU" ]; then reseau_poser; else reseau_oter; systemctl daemon-reload; dire "3. bridge and core network: not isolated (CPUS_RESEAU empty)"; adopter; fi
    [ -s "$ETAT.gouv" ] || frequence performance
    dire "4. governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)$(command -v powerprofilesctl >/dev/null && echo ", profile $(powerprofilesctl get)")"
    secteur || alerte "on BATTERY: the laptop throttles, plug it in"
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
    irq_rendre; frequence restore; rm -f "$ENVF"
    dire "isolation removed: every cpu ($TOUS) back to the system, governor $(sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | paste -sd,)" ;;
status)
    if [ -r "$ENVF" ]; then dire "ON: chain $CPUS, bridge ${C_PONT:-not isolated}, core network ${C_CN:-not isolated}, rest $AUTRES, FIFO ${FIFO:-none}"
    else dire "OFF (would reserve: chain $CPUS, bridge ${C_PONT:-none}, core network ${C_CN:-none})"; fi
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
*)  sed -n '2,33p' "$0" ;;
esac
