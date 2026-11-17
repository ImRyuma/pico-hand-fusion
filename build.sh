#!/usr/bin/env bash
# Builds the OpenXR layer, the companion APK and the Magisk zip.
#
# Requirements (paths can be overridden with environment variables):
#   ANDROID_NDK   Android NDK r27+          (default: ~/Android/ndk)
#   BUILD_TOOLS   Android SDK build-tools   (default: ~/Android/build-tools)
#   ANDROID_JAR   android.jar, API 29+      (default: ~/Android/platforms/android.jar)
#   JAVA_HOME     JDK 17+                   (default: ~/Android/jdk)
#
# Output: out/libhandfusion.ryuma.so, out/companion.apk, out/pico_hand_fusion.zip
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SDK="${ANDROID_TOOLS_ROOT:-$HOME/Android}"
ANDROID_NDK="${ANDROID_NDK:-$SDK/ndk}"
BUILD_TOOLS="${BUILD_TOOLS:-$SDK/build-tools}"
ANDROID_JAR="${ANDROID_JAR:-$SDK/platforms/android.jar}"
JAVA_HOME="${JAVA_HOME:-$SDK/jdk}"
OUT="$HERE/out"
API=29

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) HOST_TAG=windows-x86_64; EXE=.exe; BAT=.bat ;;
  Darwin) HOST_TAG=darwin-x86_64; EXE=; BAT= ;;
  *) HOST_TAG=linux-x86_64; EXE=; BAT= ;;
esac

# The SDK .bat wrappers (d8, apksigner) look up java through JAVA_HOME / PATH.
export PATH="$JAVA_HOME/bin:$PATH"
if command -v cygpath >/dev/null 2>&1; then export JAVA_HOME="$(cygpath -w "$JAVA_HOME")"; else export JAVA_HOME; fi

mkdir -p "$OUT"
rm -rf "${OUT:?}"/*

# ------------------------------------------------------------------ zygisk library
echo "== Building the Zygisk library"
CXX="$ANDROID_NDK/toolchains/llvm/prebuilt/$HOST_TAG/bin/clang++$EXE"
"$CXX" --target=aarch64-linux-android$API -std=c++17 -O2 -fPIC -shared \
  -Wall -Wextra -fvisibility=hidden -fno-exceptions -fno-rtti \
  -I"$HERE/layer/include" \
  "$HERE/layer/src/zygisk_main.cpp" "$HERE/layer/src/layer.cpp" "$HERE/layer/src/plthook.cpp" \
  "$HERE/layer/src/fusion.cpp" "$HERE/layer/src/config.cpp" "$HERE/layer/src/recorder.cpp" \
  -static-libstdc++ -Wl,--exclude-libs,ALL -llog -Wl,--no-undefined -Wl,-z,max-page-size=16384 -Wl,--gc-sections -s \
  -o "$OUT/arm64-v8a.so"

# --------------------------------------------------------------- companion
echo "== Building the companion APK"
WORK="$OUT/apk"
mkdir -p "$WORK/classes" "$WORK/dex"
"$BUILD_TOOLS/aapt2$EXE" link -I "$ANDROID_JAR" --manifest "$HERE/companion/AndroidManifest.xml" \
  -o "$WORK/unsigned.apk"
# Java 8 API from the JDK (d8 desugars lambdas), Android API from android.jar.
"$JAVA_HOME/bin/javac$EXE" --release 8 -nowarn -encoding UTF-8 \
  -classpath "$ANDROID_JAR" -d "$WORK/classes" \
  $(find "$HERE/companion/src" -name '*.java')
"$BUILD_TOOLS/d8$BAT" --min-api $API --lib "$ANDROID_JAR" --output "$WORK/dex" \
  $(find "$WORK/classes" -name '*.class')
(cd "$WORK/dex" && "$JAVA_HOME/bin/jar$EXE" uf "$WORK/unsigned.apk" classes.dex)
"$BUILD_TOOLS/zipalign$EXE" -f -p 4 "$WORK/unsigned.apk" "$WORK/aligned.apk"

KEYSTORE="${KEYSTORE:-$HERE/companion/debug.keystore}"
if [ ! -f "$KEYSTORE" ]; then
  "$JAVA_HOME/bin/keytool$EXE" -genkeypair -keystore "$KEYSTORE" -storepass android -keypass android \
    -alias handfusion -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=Hand Fusion" >/dev/null
fi
"$BUILD_TOOLS/apksigner$BAT" sign --ks "$KEYSTORE" --ks-pass pass:android --key-pass pass:android \
  --out "$OUT/companion.apk" "$WORK/aligned.apk"
rm -rf "$WORK"

# ------------------------------------------------------------------ magisk
echo "== Packaging the Magisk module"
STAGE="$OUT/module"
cp -r "$HERE/magisk" "$STAGE"
mkdir -p "$STAGE/zygisk"
cp "$OUT/arm64-v8a.so" "$STAGE/zygisk/arm64-v8a.so"
cp "$OUT/companion.apk" "$STAGE/companion.apk"
(cd "$STAGE" && "$JAVA_HOME/bin/jar$EXE" cfM "$OUT/pico_hand_fusion.zip" .)
rm -rf "$STAGE"

echo "== Done"
ls -l "$OUT"
