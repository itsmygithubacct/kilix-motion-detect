#!/bin/sh
set -eu

: "${KMD_STREAM:?set KMD_STREAM to the built stream command}"

fail()
{
    printf 'stream test failed: %s\n' "$1" >&2
    exit 1
}

if "$KMD_STREAM" 2>/dev/null; then
    fail "missing arguments were accepted"
fi
if "$KMD_STREAM" nope 1 2>/dev/null; then
    fail "malformed width was accepted"
fi
if "$KMD_STREAM" 2 1 0 2 2>/dev/null; then
    fail "non-Boolean contrast was accepted"
fi
if "$KMD_STREAM" 2 1 0 1 1 30 10 extra 2>/dev/null; then
    fail "extra argument was accepted"
fi

summary=$(printf '\000\000' | "$KMD_STREAM" 2 1 2>&1 >/dev/null) ||
    fail "one complete frame was rejected"
case "$summary" in
    *"frames=1"*) ;;
    *) fail "complete-frame summary is wrong" ;;
esac

if output=$(printf '\000' | "$KMD_STREAM" 2 1 2>&1); then
    fail "partial frame was silently accepted"
fi
case "$output" in
    *"truncated input frame"*) ;;
    *) fail "partial-frame diagnostic is missing" ;;
esac

printf 'ok stream argument and frame validation\n'
