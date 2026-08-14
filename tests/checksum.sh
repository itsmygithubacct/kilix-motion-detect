#!/bin/sh
# Run a deterministic workload and hold its trailing checksum= line to a
# committed baseline.
#
# The fuzz harness and the bench workload both fold every motion decision
# into one checksum, but printing it proves nothing: a change that shifts
# decisions everywhere - the exact regression class that would invalidate
# the measured false-positive and recall tables - still exits zero.  This
# wrapper makes the suite fail on any drift.  A deliberate behavior change
# re-baselines with KMD_REBASELINE=1 in the same commit that explains it.
#
# The baselines assume IEEE-754 float arithmetic; gcc and clang on x86-64,
# sanitizers included, reproduce them bit for bit.

set -eu

expected_file=$1
shift

output=$("$@")
printf '%s\n' "$output"
checksum=${output##*checksum=}

if [ "${KMD_REBASELINE:-0}" = "1" ]; then
    printf '%s\n' "$checksum" > "$expected_file"
    printf 'rebaselined %s\n' "$expected_file"
elif [ -f "$expected_file" ]; then
    read -r expected < "$expected_file"
    if [ "$checksum" != "$expected" ]; then
        printf 'checksum %s does not match %s from %s\n' \
            "$checksum" "$expected" "$expected_file" >&2
        exit 1
    fi
    printf 'checksum matches %s\n' "$expected_file"
else
    printf 'no baseline %s; checksum not compared\n' "$expected_file"
fi
