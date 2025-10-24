@echo off
REM Arduino Project Build Script for Windows
REM Author: M. Rakeh Saleem
REM Description: Automated build and deployment script for Arduino projects

setlocal enabledelayedexpansion

REM Configuration
set ARDUINO_CLI=arduino-cli
set PROJECTS_DIR=optimized_projects
set BUILD_DIR=build
set LOG_DIR=logs

REM Colors (Windows doesn't support colors in batch, but we can use echo)
set ERROR=[ERROR]
set SUCCESS=[SUCCESS]
set WARNING=[WARNING]
set INFO=[INFO]

REM Check if arduino-cli is installed
:check_arduino_cli
where %ARDUINO_CLI% >nul 2>&1
if %errorlevel% neq 0 (
    echo %ERROR% Arduino CLI not found. Please install it first.
    exit /b 1
)
echo %INFO% Arduino CLI found
goto :eof

REM Setup Arduino CLI
:setup_arduino_cli
echo %INFO% Setting up Arduino CLI...
%ARDUINO_CLI% core update-index
%ARDUINO_CLI% core install arduino:avr
%ARDUINO_CLI% core install teensy:avr
%ARDUINO_CLI% lib install "Wire"
%ARDUINO_CLI% lib install "SPI"
%ARDUINO_CLI% lib install "SD"
%ARDUINO_CLI% lib install "Time"
%ARDUINO_CLI% lib install "RH_RF95"
%ARDUINO_CLI% lib install "DS1307RTC"
echo %SUCCESS% Arduino CLI setup completed
goto :eof

REM Create build directories
:create_directories
echo %INFO% Creating build directories...
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
if not exist %LOG_DIR% mkdir %LOG_DIR%
echo %SUCCESS% Directories created
goto :eof

REM Build a specific project
:build_project
set project_name=%1
set board=%2
set project_path=%PROJECTS_DIR%\%project_name%

if not exist "%project_path%" (
    echo %ERROR% Project directory not found: %project_path%
    exit /b 1
)

echo %INFO% Building project: %project_name% for board: %board%

REM Compile the project
%ARDUINO_CLI% compile --fqbn %board% --build-path "%BUILD_DIR%\%project_name%" --output-dir "%BUILD_DIR%\%project_name%" "%project_path%" > "%LOG_DIR%\%project_name%_build.log" 2>&1
if %errorlevel% neq 0 (
    echo %ERROR% Failed to build project %project_name%. Check log: %LOG_DIR%\%project_name%_build.log
    exit /b 1
)
echo %SUCCESS% Project %project_name% built successfully
goto :eof

REM Build all optimized projects
:build_all_projects
echo %INFO% Building all optimized projects...

REM ATtiny85 projects
call :build_project "Attiny_low_power_optimized" "attiny:avr:attiny85"

REM Arduino projects
call :build_project "lora_node_optimized" "arduino:avr:uno"

REM Teensy projects
call :build_project "Data_logger_optimized" "teensy:avr:teensy40"

echo %SUCCESS% All projects built successfully
goto :eof

REM Run tests
:run_tests
echo %INFO% Running project tests...

REM Check if all required files exist
set required_files=shared_libraries\SensorConfig.h shared_libraries\PowerManager.h shared_libraries\I2CManager.h config\project_config.h config\hardware_config.h

for %%f in (%required_files%) do (
    if not exist "%%f" (
        echo %ERROR% Required file missing: %%f
        exit /b 1
    )
)

echo %SUCCESS% All required files present
goto :eof

REM Generate project report
:generate_report
echo %INFO% Generating project report...

for /f "tokens=2 delims==" %%a in ('wmic OS Get localdatetime /value') do set "dt=%%a"
set "YY=%dt:~2,2%" & set "YYYY=%dt:~0,4%" & set "MM=%dt:~4,2%" & set "DD=%dt:~6,2%"
set "HH=%dt:~8,2%" & set "Min=%dt:~10,2%" & set "Sec=%dt:~12,2%"
set "timestamp=%YYYY%%MM%%DD%_%HH%%Min%%Sec%"

set report_file=%LOG_DIR%\project_report_%timestamp%.txt

(
echo Arduino Projects Collection - Build Report
echo Generated: %date% %time%
echo ==========================================
echo.
echo Project Statistics:
echo - Total optimized projects: 
dir /b %PROJECTS_DIR%\*.ino 2>nul | find /c /v ""
echo - Total shared libraries:
dir /b shared_libraries\*.h 2>nul | find /c /v ""
echo - Total configuration files:
dir /b config\*.h 2>nul | find /c /v ""
echo.
echo Build Status:
if exist "%BUILD_DIR%" (
    echo - Build directory exists: Yes
    echo - Build files:
    dir /b %BUILD_DIR%\*.hex 2>nul | find /c /v ""
) else (
    echo - Build directory exists: No
)
echo.
) > "%report_file%"

echo %SUCCESS% Report generated: %report_file%
goto :eof

REM Clean build artifacts
:clean_build
echo %INFO% Cleaning build artifacts...
if exist %BUILD_DIR% rmdir /s /q %BUILD_DIR%
if exist %LOG_DIR% rmdir /s /q %LOG_DIR%
echo %SUCCESS% Build artifacts cleaned
goto :eof

REM Show help
:show_help
echo Arduino Projects Build Script
echo.
echo Usage: %0 [OPTION]
echo.
echo Options:
echo   build-all     Build all optimized projects
echo   build ^<name^>  Build specific project
echo   test          Run project tests
echo   report        Generate project report
echo   clean         Clean build artifacts
echo   setup         Setup Arduino CLI and dependencies
echo   help          Show this help message
echo.
echo Examples:
echo   %0 setup
echo   %0 build-all
echo   %0 build Attiny_low_power_optimized
echo   %0 test
goto :eof

REM Main script logic
if "%1"=="setup" (
    call :check_arduino_cli
    call :setup_arduino_cli
    call :create_directories
) else if "%1"=="build-all" (
    call :check_arduino_cli
    call :create_directories
    call :build_all_projects
    call :generate_report
) else if "%1"=="build" (
    if "%2"=="" (
        echo %ERROR% Project name required for build command
        exit /b 1
    )
    call :check_arduino_cli
    call :create_directories
    call :build_project "%2" "arduino:avr:uno"
) else if "%1"=="test" (
    call :run_tests
) else if "%1"=="report" (
    call :generate_report
) else if "%1"=="clean" (
    call :clean_build
) else (
    call :show_help
)

endlocal
