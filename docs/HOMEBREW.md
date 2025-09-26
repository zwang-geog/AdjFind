# Homebrew Distribution

This project can be installed on macOS using Homebrew, the standard package manager for macOS.

## Installation

### Option 1: Install from GitHub Tap (Recommended)
```bash
# Add the repository as a tap
brew tap zwang-geog/AdjFind https://github.com/zwang-geog/AdjFind.git

# Install from the tap
brew install zwang-geog/adjfind/adjfind
```

### Option 2: Install from GitHub URL
```bash
# Install directly from GitHub repository
brew install --build-from-source https://raw.githubusercontent.com/zwang-geog/AdjFind/master/Formula/adjfind.rb
```

### Option 3: Install from Local Formula
```bash
# Clone the repository
git clone https://github.com/zwang-geog/AdjFind.git
cd AdjFind

# Install using the local formula
brew install Formula/adjfind.rb
```

## Usage

After installation, you can use `adjfind` from anywhere in your terminal:

```bash
adjfind --help
adjfind --version
```

## Uninstallation

To uninstall `adjfind`:

```bash
brew uninstall adjfind
```

## Development

To test the formula locally:
```bash
# Make sure you're in the repository root
brew install Formula/adjfind.rb
```

## Creating a Homebrew Tap

To make your formula available to others via `brew install`, you can create a Homebrew tap:

1. Create a new repository named `homebrew-adjfind`
2. Add the formula file to `Formula/adjfind.rb`
3. Users can then install with:
   ```bash
   brew tap zwang-geog/adjfind
   brew install adjfind
   ```

## Troubleshooting

### Formula Not Found
If you get "No available formula with the name 'adjfind'", make sure you're using the full formula name:
```bash
brew install zwang-geog/adjfind/adjfind
```

### Version Issues
If you encounter version-related errors, make sure the formula has a proper version attribute in the `Formula/adjfind.rb` file.

### Dependencies
The formula automatically installs all required dependencies:
- CMake (build tool)
- GDAL (geospatial data library)
- Boost (C++ libraries)
- nlohmann-json (JSON library)
