#!/bin/bash
# Arduino Project Build Script
# Author: M. Rakeh Saleem
# Description: Automated build and deployment script for Arduino projects

set -e  # Exit on any error

# Configuration
ARDUINO_CLI="arduino-cli"
BOARD_MANAGER_URL="https://arduino.esp8266.com/stable/package_esp8266com_index.json"
PROJECTS_DIR="optimized_projects"
BUILD_DIR="build"
LOG_DIR="logs"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if arduino-cli is installed
check_arduino_cli() {
    if ! command -v $ARDUINO_CLI &> /dev/null; then
        error "Arduino CLI not found. Please install it first."
    fi
    log "Arduino CLI found: $($ARDUINO_CLI version)"
}

# Setup Arduino CLI
setup_arduino_cli() {
    log "Setting up Arduino CLI..."
    
    # Initialize Arduino CLI
    $ARDUINO_CLI core update-index
    
    # Install required cores
    $ARDUINO_CLI core install arduino:avr
    $ARDUINO_CLI core install teensy:avr
    
    # Install required libraries
    $ARDUINO_CLI lib install "Wire"
    $ARDUINO_CLI lib install "SPI"
    $ARDUINO_CLI lib install "SD"
    $ARDUINO_CLI lib install "Time"
    $ARDUINO_CLI lib install "RH_RF95"
    $ARDUINO_CLI lib install "DS1307RTC"
    
    success "Arduino CLI setup completed"
}

# Create build directories
create_directories() {
    log "Creating build directories..."
    mkdir -p $BUILD_DIR
    mkdir -p $LOG_DIR
    success "Directories created"
}

# Build a specific project
build_project() {
    local project_name=$1
    local board=$2
    local project_path="$PROJECTS_DIR/$project_name"
    
    if [ ! -d "$project_path" ]; then
        error "Project directory not found: $project_path"
    fi
    
    log "Building project: $project_name for board: $board"
    
    # Compile the project
    if $ARDUINO_CLI compile \
        --fqbn $board \
        --build-path "$BUILD_DIR/$project_name" \
        --output-dir "$BUILD_DIR/$project_name" \
        "$project_path" \
        > "$LOG_DIR/${project_name}_build.log" 2>&1; then
        success "Project $project_name built successfully"
    else
        error "Failed to build project $project_name. Check log: $LOG_DIR/${project_name}_build.log"
    fi
}

# Build all optimized projects
build_all_projects() {
    log "Building all optimized projects..."
    
    # ATtiny85 projects
    build_project "Attiny_low_power_optimized" "attiny:avr:attiny85"
    
    # Arduino projects
    build_project "lora_node_optimized" "arduino:avr:uno"
    
    # Teensy projects
    build_project "Data_logger_optimized" "teensy:avr:teensy40"
    
    success "All projects built successfully"
}

# Run tests
run_tests() {
    log "Running project tests..."
    
    # Check if all required files exist
    local required_files=(
        "shared_libraries/SensorConfig.h"
        "shared_libraries/PowerManager.h"
        "shared_libraries/I2CManager.h"
        "config/project_config.h"
        "config/hardware_config.h"
    )
    
    for file in "${required_files[@]}"; do
        if [ ! -f "$file" ]; then
            error "Required file missing: $file"
        fi
    done
    
    success "All required files present"
    
    # Check for common issues
    check_code_quality
}

# Check code quality
check_code_quality() {
    log "Checking code quality..."
    
    # Check for TODO comments
    local todo_count=$(find $PROJECTS_DIR -name "*.ino" -exec grep -l "TODO\|FIXME" {} \; | wc -l)
    if [ $todo_count -gt 0 ]; then
        warning "Found $todo_count files with TODO/FIXME comments"
    fi
    
    # Check for hardcoded values
    local hardcoded_count=$(find $PROJECTS_DIR -name "*.ino" -exec grep -l "0x[0-9A-Fa-f]\{2\}" {} \; | wc -l)
    if [ $hardcoded_count -gt 0 ]; then
        warning "Found $hardcoded_count files with hardcoded hex values"
    fi
    
    success "Code quality check completed"
}

# Generate project report
generate_report() {
    log "Generating project report..."
    
    local report_file="$LOG_DIR/project_report_$(date +'%Y%m%d_%H%M%S').txt"
    
    {
        echo "Arduino Projects Collection - Build Report"
        echo "Generated: $(date)"
        echo "=========================================="
        echo ""
        
        echo "Project Statistics:"
        echo "- Total optimized projects: $(find $PROJECTS_DIR -name "*.ino" | wc -l)"
        echo "- Total shared libraries: $(find shared_libraries -name "*.h" | wc -l)"
        echo "- Total configuration files: $(find config -name "*.h" | wc -l)"
        echo ""
        
        echo "Build Status:"
        if [ -d "$BUILD_DIR" ]; then
            echo "- Build directory exists: Yes"
            echo "- Build files: $(find $BUILD_DIR -name "*.hex" | wc -l)"
        else
            echo "- Build directory exists: No"
        fi
        echo ""
        
        echo "Recent Changes:"
        git log --oneline -10 2>/dev/null || echo "No git history available"
        
    } > "$report_file"
    
    success "Report generated: $report_file"
}

# Clean build artifacts
clean_build() {
    log "Cleaning build artifacts..."
    rm -rf $BUILD_DIR
    rm -rf $LOG_DIR
    success "Build artifacts cleaned"
}

# Show help
show_help() {
    echo "Arduino Projects Build Script"
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  build-all     Build all optimized projects"
    echo "  build <name>  Build specific project"
    echo "  test          Run project tests"
    echo "  report        Generate project report"
    echo "  clean         Clean build artifacts"
    echo "  setup         Setup Arduino CLI and dependencies"
    echo "  help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 setup"
    echo "  $0 build-all"
    echo "  $0 build Attiny_low_power_optimized"
    echo "  $0 test"
}

# Main script logic
main() {
    case "${1:-help}" in
        "setup")
            check_arduino_cli
            setup_arduino_cli
            create_directories
            ;;
        "build-all")
            check_arduino_cli
            create_directories
            build_all_projects
            generate_report
            ;;
        "build")
            if [ -z "$2" ]; then
                error "Project name required for build command"
            fi
            check_arduino_cli
            create_directories
            build_project "$2" "arduino:avr:uno"
            ;;
        "test")
            run_tests
            ;;
        "report")
            generate_report
            ;;
        "clean")
            clean_build
            ;;
        "help"|*)
            show_help
            ;;
    esac
}

# Run main function with all arguments
main "$@"
