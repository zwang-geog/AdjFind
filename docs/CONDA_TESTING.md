# Testing conda Package Build

This guide walks you through testing the `adjfind` conda package build using `meta.yaml` on a Linux virtual machine (or any Unix-like system).

## Prerequisites

- A Linux VM (Ubuntu, CentOS, etc.) or macOS
- Internet connection
- Basic command-line knowledge

## Step 1: Install Miniconda

### Download Miniconda

**For Linux (x86_64):**
```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
```

**For Linux (ARM64/aarch64):**
```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
```

**For macOS (Intel):**
```bash
cd ~
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
```

**For macOS (Apple Silicon):**
```bash
cd ~
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
```

### Install Miniconda

```bash
# Make the installer executable
chmod +x Miniconda3-latest-*.sh

# Run the installer (follow prompts)
bash Miniconda3-latest-*.sh

# Accept the license agreement
# Choose installation location (default: ~/miniconda3)
# Choose "yes" to initialize conda
```

### Initialize Conda

After installation, close and reopen your terminal, or run:

```bash
source ~/.bashrc
# or
source ~/.zshrc  # if using zsh
```

Verify installation:
```bash
conda --version
```

You should see something like: `conda 24.x.x`

## Step 2: Set Up conda-build Environment

### Add conda-forge Channel

```bash
conda config --add channels conda-forge
conda config --set channel_priority strict
```

### Install conda-build

```bash
conda install conda-build
```

Press twice a for accept, later press y

Verify installation:
```bash
conda build --version
```

## Step 3: Get the Source Code

### Option A: Clone from GitHub (Recommended)

```bash
cd ~
git clone https://github.com/zwang-geog/AdjFind.git
cd AdjFind
```

### Option B: Download Source Archive

If you just need to test with the current `meta.yaml`:

```bash
cd ~
mkdir adjfind-test
cd adjfind-test
# Copy your meta.yaml file here
```

## Step 4: Build the Package

### Navigate to Project Directory

```bash
cd ~/AdjFind  # or wherever you cloned/downloaded the repo
```

### Build the Package

```bash
conda build .
```

This will:
1. Download all dependencies (GDAL, Boost, compilers, etc.)
2. Extract the source code
3. Configure and build using CMake
4. Run the test commands (`adjfind --help`, `adjfind --version`) automatically
5. Create a conda package file (`.conda` or `.tar.bz2` format)

### Expected Output

You should see output like:
```
BUILD START: adjfind-0.1.3-0
...
TEST START: adjfind-0.1.3-0
...
+ adjfind --help
[help output]
+ adjfind --version
AdjFind v0.1.3
...
TEST END: adjfind-0.1.3-0
...
Successfully built adjfind-0.1.3-0
```

**Note:** The tests (`adjfind --help` and `adjfind --version`) run automatically during the build process. If you see both commands succeed in the test output, your build is successful!

The built package will be located at:
```bash
~/miniconda3/conda-bld/linux-64/adjfind-0.1.3-*.conda  # Linux (modern .conda format)
# or
~/miniconda3/conda-bld/linux-64/adjfind-0.1.3-*.tar.bz2  # Linux (legacy format)
# or
~/miniconda3/conda-bld/osx-64/adjfind-0.1.3-*.conda   # macOS Intel
# or
~/miniconda3/conda-bld/osx-arm64/adjfind-0.1.3-*.conda  # macOS ARM
```

**Important:** The package is built but **not automatically installed** in your current environment. You need to install it separately (see Step 5 below).

## Step 5: Test the Built Package

**Option A: Install in Current Environment (Quick Test)**

If you just want to use `adjfind` in your current environment:

```bash
conda install --use-local adjfind
adjfind --help
adjfind --version
```

**Option B: Install in a Clean Test Environment (Recommended)**

For a clean test environment:

### Create a Test Environment

```bash
conda create -n test-adjfind -y
conda activate test-adjfind
```

### Install the Locally Built Package

```bash
conda install --use-local adjfind
```

Or install directly from the built file:
```bash
# For .conda format (modern)
conda install ~/miniconda3/conda-bld/linux-64/adjfind-0.1.3-*.conda

# For .tar.bz2 format (legacy)
conda install ~/miniconda3/conda-bld/linux-64/adjfind-0.1.3-*.tar.bz2
```

**Note:** If you try to run `adjfind` in your base environment after building (without installing), you'll get "command not found". This is expected - you need to install the package first (see above).

### Test the Installation

```bash
# Test help command
adjfind --help

# Test version command
adjfind --version
```

Both commands should work without errors.

### Verify Dependencies

Check that runtime dependencies are correctly installed:
```bash
conda list
```

You should see:
- `adjfind`
- `libgdal-core` (via run_exports)
- `libboost` (via run_exports)

## Step 6: Clean Up (Optional)

### Remove Test Environment

```bash
conda deactivate
conda env remove -n test-adjfind
```

### Clean Build Cache

If a build fails or you want to rebuild from scratch:

**Option 1: Purge All Build Cache (Complete Clean)**
```bash
conda build purge
```

This removes all build artifacts, work directories, and cached packages.

**Option 2: Purge Specific Package**
```bash
conda build purge adjfind
```

This removes only the build cache for `adjfind`.

**Option 3: Remove Build Directory Manually**
```bash
# Remove the entire build directory
rm -rf ~/miniconda3/conda-bld/adjfind_*

# Or remove specific build
rm -rf ~/miniconda3/conda-bld/adjfind_<timestamp>
```

**Option 4: Clean and Rebuild in One Command**
```bash
conda build purge adjfind && conda build .
```

**Option 5: Force Rebuild (Skip Cache)**
```bash
conda build . --no-cache
```

This rebuilds without using cached packages, but keeps the build directory.

**Option 6: Nuclear Option - Remove Everything**
```bash
# Remove all conda build artifacts
rm -rf ~/miniconda3/conda-bld/*

# Then rebuild
conda build .
```

**After Purging, Rebuild:**
```bash
cd ~/adjfind  # or your project directory
conda build .
```

## Troubleshooting

### Issue: Build Failed - How to Clean and Rebuild

**Symptoms:** Build fails with errors, or you want to rebuild from scratch.

**Solution 1: Quick Clean and Rebuild**
```bash
conda build purge adjfind
conda build .
```

**Solution 2: Check Build Logs**
```bash
# Find the build directory
ls -la ~/miniconda3/conda-bld/adjfind_*/

# View the build log
cat ~/miniconda3/conda-bld/adjfind_*/work/build_env_setup.sh
cat ~/miniconda3/conda-bld/adjfind_*/work/build.log
```

**Solution 3: Clean Everything and Rebuild**
```bash
# Remove all build artifacts
conda build purge

# Or manually remove
rm -rf ~/miniconda3/conda-bld/*

# Rebuild
conda build .
```

**Solution 4: Rebuild Without Cache**
```bash
conda build . --no-cache
```

**Solution 5: Check for Locked Files**
If build directory is locked or in use:
```bash
# Find and kill any conda-build processes
ps aux | grep conda-build
kill <process_id>

# Then clean and rebuild
conda build purge adjfind
conda build .
```

### Issue: "Could not find GDAL" or "Could not find Boost"

**Solution:** Make sure you've added conda-forge channel:
```bash
conda config --add channels conda-forge
conda config --set channel_priority strict
```

### Issue: Build Fails with Compiler Errors

**Solution:** Install compilers explicitly:
```bash
conda install -c conda-forge compilers -y
```

### Issue: "conda-build: command not found"

**Solution:** Install conda-build:
```bash
conda install conda-build -y
```

### Issue: Build Succeeds but Tests Fail

**Solution:** Check if the executable is in PATH:
```bash
which adjfind
```

If not found, check the build log:
```bash
cat ~/miniconda3/conda-bld/linux-64/work/build_env_setup.sh
```

### Issue: Permission Denied on Installer

**Solution:** Make installer executable:
```bash
chmod +x Miniconda3-latest-*.sh
```

### Issue: Build Takes Too Long

**Solution:** This is normal for the first build. Subsequent builds will be faster due to caching.

## Advanced: Building for Multiple Platforms

### Build for Specific Platform

```bash
# Build for Linux 64-bit
conda build . -c conda-forge

# Build for macOS Intel
CONDA_SUBDIR=osx-64 conda build . -c conda-forge

# Build for macOS ARM
CONDA_SUBDIR=osx-arm64 conda build . -c conda-forge
```

### Build All Platforms (Requires Cross-Compilation Setup)

This is typically done by conda-forge's CI/CD infrastructure, not locally.

## Verifying Package Contents

### Inspect the Built Package

```bash
conda inspect linkages adjfind
conda inspect objects adjfind
```

### Check Package Metadata

```bash
conda search --info --use-local adjfind
```

## Next Steps

Once local testing passes:

1. **Submit to conda-forge**: Create a PR to `staged-recipes` repository
2. **Monitor CI builds**: Check that all platforms build successfully
3. **Address reviewer comments**: Fix any issues raised during review
4. **Maintain the feedstock**: Once accepted, maintain the conda-forge feedstock

## Additional Resources

- [conda-build Documentation](https://docs.conda.io/projects/conda-build/)
- [conda-forge Documentation](https://conda-forge.org/docs/)
- [conda-forge Staged Recipes](https://github.com/conda-forge/staged-recipes)

## Quick Reference Commands

```bash
# Install Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Setup
conda config --add channels conda-forge
conda install conda-build -y

# Build (tests run automatically during build)
conda build .

# Install and test in a new environment
conda create -n test-adjfind -y
conda activate test-adjfind
conda install --use-local adjfind
adjfind --help
adjfind --version

# Or install in current environment
conda install --use-local adjfind
```

