#!/usr/bin/env bash
set -euo pipefail

echo "🔧 Formatting Python files..."
black .
isort .

echo "🔧 Formatting C++ files..."
find src -name '*.cpp' -o -name '*.hpp' | xargs -r clang-format -i

echo "🔧 Formatting YAML files..."
find . -name '*.yaml' -o -name '*.yml' | xargs -r yamllint -f parsable || true

echo "🔧 Formatting XML files..."
for f in $(find . -name '*.xml'); do
  xmllint --format "$f" --output "$f"
done

echo "🔧 Formatting CMake files..."
for f in $(find . -name 'CMakeLists.txt' -o -name '*.cmake'); do
  cmake-format -i "$f"
done

echo "✅ Formatting complete!"