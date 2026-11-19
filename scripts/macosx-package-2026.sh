#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 ]]; then
  printf 'Usage: %s <cmake-build-directory>\n' "$0" >&2
  exit 1
fi

source_dir=$(cd "$(dirname "$0")/.." && pwd)
build_dir=$(cd "$1" && pwd)
app_name='OpenSCAD 2026.app'
destination="$build_dir/$app_name"
if [[ -e "$destination" || -L "$destination" ]]; then
  printf 'Destination already exists: %s\n' "$destination" >&2
  exit 1
fi

stage=$(mktemp -d "$build_dir/.openscad-2026.XXXXXX")
trap 'rm -rf "$stage"' EXIT
app="$stage/$app_name"
ditto "$build_dir/OpenSCAD.app" "$app"

plist="$app/Contents/Info.plist"
/usr/libexec/PlistBuddy \
  -c 'Set :CFBundleName OpenSCAD 2026' \
  -c 'Add :CFBundleDisplayName string OpenSCAD 2026' \
  -c 'Set :CFBundleIdentifier org.openscad.OpenSCAD.arm64' \
  -c 'Add :OpenSCADSettingsName string OpenSCADARM' \
  -c 'Add :OpenSCADMessageLocale string fr_FR.UTF-8' \
  -c 'Add :NSRequiresAquaSystemAppearance bool true' \
  -c 'Add :CFBundleLocalizations array' \
  -c 'Add :CFBundleLocalizations:0 string fr' \
  -c 'Add :CFBundleLocalizations:1 string en' \
  "$plist"

for catalog in "$source_dir"/locale/*/LC_MESSAGES/openscad.mo; do
  [[ -f "$catalog" ]] || continue
  bundled_catalog="$app/Contents/Resources/locale/${catalog#"$source_dir/locale/"}"
  mkdir -p "$(dirname "$bundled_catalog")"
  install -m 644 "$catalog" "$bundled_catalog"
done
test -s "$app/Contents/Resources/locale/fr/LC_MESSAGES/openscad.mo"

binary="$app/Contents/MacOS/OpenSCAD"
lib3mf_load_path=$(otool -L "$binary" | sed -nE 's/^[[:space:]]+(.+\/lib3mf(\.[^/]*)?\.dylib) \(compatibility version .*/\1/p')
if [[ -n "$lib3mf_load_path" ]]; then
  lib3mf_source=$(sed -n 's/^Lib3MF_LIBRARY:FILEPATH=//p' "$build_dir/CMakeCache.txt")
  library_name=${lib3mf_load_path##*/}
  bundled_library="$app/Contents/Frameworks/$library_name"
  mkdir -p "$app/Contents/Frameworks"
  install -m 755 "$lib3mf_source" "$bundled_library"
  install_name_tool -change "$lib3mf_load_path" "@executable_path/../Frameworks/$library_name" "$binary"
  # CMake may add the local lib3mf directory to the build RPATH.
  lib3mf_dir=$(dirname "$lib3mf_source")
  if otool -l "$binary" | awk '/cmd LC_RPATH/{getline; getline; sub(/^ *path /, ""); sub(/ \(offset [0-9]+\)$/, ""); print}' | grep -Fx "$lib3mf_dir" >/dev/null; then
    install_name_tool -delete_rpath "$lib3mf_dir" "$binary"
  fi
  codesign --force --sign - "$bundled_library"
fi

plutil -lint "$plist"
codesign --force --sign - "$app"
codesign --verify --deep --strict "$app"
mv -n "$app" "$build_dir/"
test ! -e "$app"
printf '%s\n' "$destination"
