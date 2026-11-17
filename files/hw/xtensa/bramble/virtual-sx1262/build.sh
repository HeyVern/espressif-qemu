#!/bin/sh
# Build the model.
#
#   ./build.sh            everything, then run both test binaries
#   ./build.sh static     libvirtualsx1262.a       - for C and C++ hosts
#   ./build.sh shared     libvirtualsx1262.{so,dylib,dll} - for hosts loading at runtime
#   ./build.sh test       build and run the tests
#   ./build.sh sanitize   the tests under AddressSanitizer and UBSan
#   ./build.sh pedantic   the conversion and cast audit still owed on the model
#
# STRICT=1 turns warnings into errors, which is what CI uses. It is off by
# default so a host bisecting an old compiler is not blocked by a new warning.
set -e

OUT=${OUT:-build}
CXX=${CXX:-c++}
CC=${CC:-cc}

# The warnings that find bugs. These are errors in CI.
WARN="-Wall -Wextra -Wpedantic -Wshadow"
[ -n "$STRICT" ] && WARN="$WARN -Werror"

# -Wconversion, -Wsign-conversion and -Wold-style-cast are deliberately NOT in
# that set. The model is register work: packing datasheet fields into bytes is
# narrowing on purpose, and a C-style cast reads the way the datasheet does.
# Turning them on means auditing every narrowing in the model to tell the
# deliberate ones from the accidental ones, which is real work and owed, not a
# flag flip. `./build.sh pedantic` runs them so the backlog is visible instead
# of hidden, and it is not wired into CI until that audit is done.
PEDANTIC_WARN="-Wconversion -Wsign-conversion -Wold-style-cast"

CXXFLAGS="-std=c++17 -O2 -Iinclude -Isrc $WARN ${CXXFLAGS}"
# The C test exists to prove the header is valid C. Held to the same standard the
# hosts compile at: QEMU is C11, and -Wpedantic catches a C++ism slipping in.
CFLAGS_C="-std=c11 -O2 -Iinclude -Wall -Wextra -Wpedantic ${CFLAGS}"
[ -n "$STRICT" ] && CFLAGS_C="$CFLAGS_C -Werror"

SRC="src/VirtualSX1262.cpp src/cad.cpp src/commands.cpp src/spi.cpp src/abi.cpp src/listening.cpp src/continuous.cpp"

# SHARED_RUNTIME is the toolchain runtime the shared build links against.
#
# Windows takes it statically. The DLL is opened by an emulator at run time -
# QEMU through LoadLibrary, Renode through LoadLibraryW behind a P/Invoke - on a
# machine that has never had a toolchain on it. Linked the default way it
# imports libstdc++-6.dll, which exists inside MSYS2 and almost nowhere else, so
# the build and its tests pass on the runner and the library cannot be opened
# on the target:
#
#   qemu-system-xtensa.exe: sx1262: cannot load the chip model at
#   ...\libvirtualsx1262.dll: The specified module could not be found.
#
# That message names this DLL rather than the dependency it could not find,
# which is why it reads as a missing library rather than a missing runtime.
#
# Shipping the runtime beside this file does not fix it: Windows resolves a
# dynamically loaded DLL's own imports from the loading *process's* directory,
# not from the directory of the DLL being loaded, so a copy would have to sit
# next to every host that opens us. Static is the only version that travels.
# -static rather than -static-libstdc++, because libstdc++ brings
# libwinpthread-1.dll in behind it and the pair have to go together.
#
# Linux and macOS keep the dynamic link. libstdc++ is part of the base system
# on both, and a plugin carrying its own copy into a host process that already
# has one is the worse bet.
case "$(uname -s 2>/dev/null)" in
  # CRT_fp10.o by name: it defines __mingw_fe_dfl_env, which libmingwex's
  # fesetenv refers to, and gcc adds it to a program's link but not a DLL's.
  # MinGW-w64 r532 (gcc 16.2) is where a static DLL started needing it.
  MINGW*|MSYS*|CYGWIN*) SHARED_EXT=dll
    SHARED_RUNTIME="-static"
    fp=$($CXX -print-file-name=CRT_fp10.o 2>/dev/null || true)
    [ -f "$fp" ] && SHARED_RUNTIME="$SHARED_RUNTIME $fp" ;;
  Darwin)               SHARED_EXT=dylib; SHARED_RUNTIME="" ;;
  *)                    SHARED_EXT=so;    SHARED_RUNTIME="" ;;
esac

mkdir -p "$OUT"

# -fno-exceptions is deliberate: the ABI reports failure by return value, and an
# exception unwinding into C or across a P/Invoke boundary is undefined.
build_static() {
  for f in $SRC; do
    $CXX $CXXFLAGS -fno-exceptions -c "$f" -o "$OUT/$(basename "${f%.cpp}").o"
  done
  ar rcs "$OUT/libvirtualsx1262.a" "$OUT"/*.o
  echo "built $OUT/libvirtualsx1262.a"
}

build_shared() {
  $CXX $CXXFLAGS -fPIC -fno-exceptions -shared $SRC $SHARED_RUNTIME \
    -o "$OUT/libvirtualsx1262.$SHARED_EXT"
  echo "built $OUT/libvirtualsx1262.$SHARED_EXT"
  check_shared_imports
}

# The last time this went wrong the build was green, the tests passed, the
# artefact was the right size and nothing could open it. So the build now says
# what its own product needs rather than leaving that to be found on somebody
# else's machine. kernel32 and msvcrt are on every supported Windows; anything
# else is a file that has to travel with us, and does not.
check_shared_imports() {
  [ "$SHARED_EXT" = dll ] || return 0
  if ! command -v objdump >/dev/null 2>&1; then
    echo "build.sh: no objdump, so the DLL's imports went unchecked" >&2
    return 0
  fi
  stray=$(objdump -p "$OUT/libvirtualsx1262.dll" \
    | sed -n 's/^	DLL Name: //p' \
    | tr 'A-Z' 'a-z' | sort -u \
    | grep -v -e '^kernel32\.dll$' -e '^msvcrt\.dll$' || true)
  if [ -n "$stray" ]; then
    echo "build.sh: the DLL imports something that will not be on the target:" >&2
    echo "$stray" | sed 's/^/  /' >&2
    echo "  a host opening it gets \"The specified module could not be found\"," >&2
    echo "  naming this DLL rather than the one above." >&2
    exit 1
  fi
  echo "checked $OUT/libvirtualsx1262.dll: nothing imported beyond kernel32 and msvcrt"
}

# One binary per subject rather than one big one, because the house limit on
# file length applies to tests too and a single file had already outgrown it.
# They share test/harness.h and nothing else, so each counts its own failures.
TESTS="test_reception test_listening test_interrupts test_spi test_cad test_modem test_commands test_continuous"

build_tests() {
  for t in $TESTS; do
    $CXX $CXXFLAGS -Itest $SRC "test/$t.cpp" -o "$OUT/$t"
  done
  # Compiled by the C compiler against the static library, which is the whole
  # point: a header that only works in C++ fails QEMU and Renode, not this.
  build_static >/dev/null
  # -lm and -lstdc++ by hand: the C driver links neither implicitly, and the
  # model uses fmax while the ABI is C++ underneath.
  $CC $CFLAGS_C test/test_c_abi.c "$OUT/libvirtualsx1262.a" -lstdc++ -lm -o "$OUT/test_c_abi"
  echo "built $TESTS and test_c_abi in $OUT"
}

run_tests() {
  for t in $TESTS; do
    "$OUT/$t"
  done
  "$OUT/test_c_abi"
}

# Sequential, never `a && b`: set -e is suspended inside the left operand of an
# && list, so a failing compile there printed an error and carried on to run a
# binary that was never built.
case "${1:-all}" in
  static) build_static ;;
  shared) build_shared ;;
  test)   build_tests; run_tests ;;
  pedantic)
    # Expected to fail today. It exists to measure the audit, not to gate it.
    $CXX $CXXFLAGS -Itest $PEDANTIC_WARN -fsyntax-only $SRC test/*.cpp
    ;;
  sanitize)
    SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1"
    for t in $TESTS; do
      $CXX $CXXFLAGS -Itest $SAN $SRC "test/$t.cpp" -o "$OUT/${t}_san"
    done
    # The C test gets the same treatment. Its own stack overflow is what proved
    # this needed to cover both binaries rather than just the C++ one.
    $CC $CFLAGS_C $SAN -Iinclude test/test_c_abi.c $SRC -lstdc++ -lm -o "$OUT/test_c_abi_san" 2>/dev/null \
      || $CXX $CXXFLAGS $SAN -x c++ test/test_c_abi.c $SRC -o "$OUT/test_c_abi_san"
    for t in $TESTS; do
      ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 "$OUT/${t}_san"
    done
    ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 "$OUT/test_c_abi_san"
    ;;
  all)    build_static; build_shared; build_tests; run_tests ;;
  *)      echo "build.sh: unknown target $1" >&2; exit 2 ;;
esac
