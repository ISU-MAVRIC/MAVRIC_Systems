# Code Formatting

This repository uses automated code formatting to maintain consistent code style across all files.

## Formatters Used

### Python Files (*.py)
- **Black**: Primary code formatter with 88 character line length
- **isort**: Import sorting compatible with Black profile

### JSON Files (*.json)
- **Prettier**: JSON formatter with consistent indentation and formatting

### CMake Files (CMakeLists.txt, *.cmake)
- **cmake-format**: CMake file formatter

### C/C++ Files (*.cpp, *.c, *.h, *.hpp)
- **clang-format**: C++ formatter based on Google style guide

## GitHub Actions

### Automatic Formatting (`code-format.yml`)
- Runs on pushes to main/master/develop branches
- Runs on pull requests
- Can be triggered manually via workflow_dispatch
- Automatically formats code and commits changes if needed

### Format Checking (`code-format-check.yml`)
- Runs on pull requests
- Checks if code is properly formatted
- Fails if formatting issues are found (no auto-fix)

## Local Development

### Setup
Install the formatting tools locally:

```bash
# Python tools
pip install black isort

# JavaScript/JSON tools
npm install -g prettier

# CMake and C++ tools
sudo apt-get install cmake-format clang-format
```

### Manual Formatting

#### Format Python files
```bash
# Format with Black
black --line-length 88 --target-version py38 .

# Sort imports with isort
isort --profile black .
```

#### Format JSON files
```bash
prettier --write "**/*.json"
```

#### Format CMake files
```bash
find . -name "CMakeLists.txt" -o -name "*.cmake" | xargs cmake-format -i
```

#### Format C/C++ files
```bash
find . \( -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp" \) | xargs clang-format -i
```

### Check Formatting
```bash
# Check Python formatting
black --check --diff .
isort --check-only --diff --profile black .

# Check JSON formatting
prettier --check "**/*.json"

# Check CMake formatting
find . -name "CMakeLists.txt" -o -name "*.cmake" | xargs cmake-format --check

# Check C/C++ formatting
find . \( -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp" \) -exec clang-format --dry-run --Werror {} \;
```

## Configuration Files

- `pyproject.toml`: Black and isort configuration
- `.prettierrc`: Prettier configuration for JSON files
- `.clang-format`: Clang-format configuration for C/C++ files

## Excluded Directories

The following directories are excluded from formatting:
- `.git/`
- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `node_modules/`

## Integration

The formatting is integrated into the CI/CD pipeline:
1. Pull requests are checked for proper formatting
2. Code is automatically formatted on pushes to main branches
3. Manual formatting can be triggered via GitHub Actions interface