# ============================================================================
# HLV EV Battery Enhancement - System Distribution Makefile
# ============================================================================

.PHONY: all verify diagnostics install update rollback clean help

# Directories
INSTALL_DIR = /opt/hlv_enhancement
LOG_FILE = $(INSTALL_DIR)/logs/install.log
LOCAL_LOG = logs/install.log

all: help

help:
	@echo "HLV EV Battery Enhancement Software - Distribution Makefile"
	@echo "===================================================================="
	@echo "Available Targets:"
	@echo "  make verify      - Run host system environment checks (OS, Python, HW, permissions)"
	@echo "  make diagnostics - Run pre-flight EV battery diagnostics and safety validation"
	@echo "  make install     - Run diagnostics, backup previous version, compile & install to /opt"
	@echo "  make update      - Pull latest updates, re-run diagnostics, and apply updates"
	@echo "  make rollback    - Restore the previous stable version using the last backup"
	@echo "  make clean       - Clean temporary build outputs"
	@echo "===================================================================="

# 1. verify — run environment checks
verify:
	@echo "Running System Environment Verification..."
	@python3 -c "import platform, sys, os; \
		print('Host OS:', platform.system()); \
		print('Python Version:', platform.python_version()); \
		assert platform.system().lower() in ['linux', 'darwin'], 'OS not supported'; \
		assert sys.version_info >= (3, 8), 'Python 3.8+ required';"
	@echo "Checking compilers..."
	@which g++ >/dev/null || which clang++ >/dev/null || (echo "ERROR: C++ Compiler not found." && exit 1)
	@echo "✓ Environment verification passed successfully."

# 2. diagnostics — run pre-flight EV battery diagnostics and compatibility checks
diagnostics:
	@echo "Running Pre-Flight EV Battery Diagnostics and Compatibility Validation..."
	@python3 scripts/diagnostics.py

# 3. install — install all dependencies, set up runtime environment, deploy enhancement software, log installation
install:
	@# Enforce sudo / root permissions
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "ERROR: Target 'install' must be run with sudo or root privileges."; \
		exit 1; \
	fi

	@echo "===================================================================="
	@echo "HLV EV SOFTWARE INSTALLATION PROCESS"
	@echo "===================================================================="

	@# Enforce safety: Refuse installation if diagnostics fail
	@echo "Executing safety checks and diagnostics..."
	@if ! python3 scripts/diagnostics.py; then \
		echo "CRITICAL: Pre-flight diagnostics failed. Installation is BLOCKED for safety reasons." | tee -a $(LOCAL_LOG) 2>/dev/null || true; \
		exit 1; \
	fi

	@# Set up target directories
	@echo "Setting up installation directories under $(INSTALL_DIR)..."
	@mkdir -p $(INSTALL_DIR)/bin
	@mkdir -p $(INSTALL_DIR)/lib
	@mkdir -p $(INSTALL_DIR)/python
	@mkdir -p $(INSTALL_DIR)/python_fallback
	@mkdir -p $(INSTALL_DIR)/config
	@mkdir -p $(INSTALL_DIR)/logs
	@mkdir -p $(INSTALL_DIR)/backups
	@mkdir -p $(INSTALL_DIR)/docs

	@# Trigger backup of any previous installation
	@echo "Checking for previous installations to backup..."
	@./scripts/rollback.sh backup

	@# Compile high-performance C++ CLI and Shared Library
	@echo "Compiling high-performance C++ HLV components..."
	@cmake -B build -DCMAKE_BUILD_TYPE=Release
	@cmake --build build

	@# Copy compiled assets to target directory
	@echo "Deploying binaries and shared libraries..."
	@cp -p hlv_core/bin/hlv_enhancer $(INSTALL_DIR)/bin/
	@cp -p hlv_core/lib/libhlv_enhancer.* $(INSTALL_DIR)/lib/

	@# Copy scripts and utilities
	@echo "Deploying system scripts..."
	@cp -p scripts/*.py $(INSTALL_DIR)/bin/
	@cp -p scripts/*.sh $(INSTALL_DIR)/bin/
	@chmod +x $(INSTALL_DIR)/bin/*

	@# Copy Python wrapper and fallbacks
	@echo "Deploying Python integration wrappers..."
	@cp -p hlv_core/python/hlv_enhancer.py $(INSTALL_DIR)/python/
	@cp -p hlv_core/python_fallback/hlv_enhancer_fallback.py $(INSTALL_DIR)/python_fallback/

	@# Copy configuration profiles
	@echo "Deploying configurations..."
	@cp -p config/*.json $(INSTALL_DIR)/config/

	@# Copy documentation
	@echo "Deploying manuals and safety documentation..."
	@cp -p docs/*.md $(INSTALL_DIR)/docs/

	@# Finalize permissions
	@chmod 644 $(INSTALL_DIR)/config/*.json
	@chmod 644 $(INSTALL_DIR)/docs/*.md

	@# Log the outcome
	@echo "Logging installation details..."
	@echo "[`date +'%Y-%m-%d %H:%M:%S'`] SUCCESS: HLV EV Software suite successfully installed to $(INSTALL_DIR)." | tee -a $(LOG_FILE) $(LOCAL_LOG) 2>/dev/null || true
	@echo "===================================================================="
	@echo "✓ HLV Software Suite successfully deployed to $(INSTALL_DIR)."
	@echo "===================================================================="

# 4. update — pull latest version, re-run safety checks, apply updates
update:
	@# Enforce sudo / root permissions
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "ERROR: Target 'update' must be run with sudo or root privileges."; \
		exit 1; \
	fi

	@echo "===================================================================="
	@echo "HLV EV SOFTWARE UPDATE PROCESS"
	@echo "===================================================================="
	@echo "Pulling latest version updates..."
	@if [ -d .git ]; then \
		git pull origin main || echo "Local environment updated (no remote connection or already up to date)."; \
	else \
		echo "Not a git repository. Simulating remote updates check..."; \
	fi

	@# Re-run safety and diagnostic checks
	@echo "Re-running host and battery safety checks..."
	@if ! python3 scripts/diagnostics.py; then \
		echo "CRITICAL: Pre-flight diagnostics failed after updates. Update BLOCKED for vehicle safety." | tee -a $(LOG_FILE) $(LOCAL_LOG) 2>/dev/null || true; \
		exit 1; \
	fi

	@# Re-run installation to apply updates
	@echo "Applying updates to target deployment..."
	@$(MAKE) install

	@echo "[`date +'%Y-%m-%d %H:%M:%S'`] SUCCESS: HLV Software suite successfully updated." | tee -a $(LOG_FILE) $(LOCAL_LOG) 2>/dev/null || true
	@echo "===================================================================="
	@echo "✓ HLV Software Suite successfully updated."
	@echo "===================================================================="

# 5. rollback — restore previous version using stored backup
rollback:
	@# Enforce sudo / root permissions
	@if [ "$$(id -u)" -ne 0 ]; then \
		echo "ERROR: Target 'rollback' must be run with sudo or root privileges."; \
		exit 1; \
	fi

	@echo "===================================================================="
	@echo "HLV EV SOFTWARE ROLLBACK PROCESS"
	@echo "===================================================================="
	@./scripts/rollback.sh restore
	@echo "===================================================================="

# 6. clean - clean build directories
clean:
	@echo "Cleaning temporary build outputs..."
	@rm -rf build/
	@echo "✓ Build directories cleaned."
