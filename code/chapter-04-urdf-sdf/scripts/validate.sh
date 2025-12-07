#!/bin/bash

# URDF/SDF Validation Script
# Validates all robot descriptions in the repository

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================="
echo "URDF/SDF Validation Script"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Counter for tests
TESTS_PASSED=0
TESTS_FAILED=0

# Function to print test results
pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((TESTS_PASSED++))
}

fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((TESTS_FAILED++))
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Check dependencies
echo "Checking dependencies..."
if ! command -v check_urdf &> /dev/null; then
    fail "check_urdf not found. Install with: sudo apt install liburdfdom-tools"
    exit 1
else
    pass "check_urdf found"
fi

if ! command -v ros2 &> /dev/null; then
    warn "ROS 2 not found. Xacro conversion will be skipped."
else
    pass "ROS 2 found"
fi

if ! command -v gz &> /dev/null; then
    warn "Gazebo not found. SDF validation will be skipped."
else
    pass "Gazebo found"
fi

echo ""
echo "========================================="
echo "Validating URDF Files"
echo "========================================="

# Validate humanoid_12dof.urdf
URDF_FILE="$PROJECT_DIR/urdf/humanoid_12dof.urdf"
if [ -f "$URDF_FILE" ]; then
    echo ""
    echo "Testing: humanoid_12dof.urdf"
    if check_urdf "$URDF_FILE" > /tmp/check_urdf_output.txt 2>&1; then
        pass "humanoid_12dof.urdf is valid"

        # Check for required properties
        if grep -q "root Link: base_link" /tmp/check_urdf_output.txt; then
            pass "  Root link is base_link"
        else
            fail "  Root link is not base_link"
        fi

        # Count joints (should have 12 revolute joints)
        JOINT_COUNT=$(grep -c "child(" /tmp/check_urdf_output.txt || true)
        if [ "$JOINT_COUNT" -ge 8 ]; then
            pass "  Found $JOINT_COUNT child links"
        else
            fail "  Only found $JOINT_COUNT child links (expected >= 8)"
        fi
    else
        fail "humanoid_12dof.urdf validation failed"
        cat /tmp/check_urdf_output.txt
    fi
else
    fail "humanoid_12dof.urdf not found"
fi

# Validate Xacro file (if ROS 2 is available)
if command -v ros2 &> /dev/null; then
    echo ""
    echo "========================================="
    echo "Testing Xacro Conversion"
    echo "========================================="

    XACRO_FILE="$PROJECT_DIR/urdf/humanoid.xacro"
    if [ -f "$XACRO_FILE" ]; then
        echo ""
        echo "Testing: humanoid.xacro"

        # Convert xacro to URDF
        if ros2 run xacro xacro "$XACRO_FILE" > /tmp/humanoid_from_xacro.urdf 2>/tmp/xacro_errors.txt; then
            pass "Xacro conversion successful"

            # Validate converted URDF
            if check_urdf /tmp/humanoid_from_xacro.urdf > /tmp/check_urdf_xacro.txt 2>&1; then
                pass "Converted URDF is valid"
            else
                fail "Converted URDF validation failed"
                cat /tmp/check_urdf_xacro.txt
            fi
        else
            fail "Xacro conversion failed"
            cat /tmp/xacro_errors.txt
        fi
    else
        warn "humanoid.xacro not found"
    fi
fi

# Validate SDF conversion (if Gazebo is available)
if command -v gz &> /dev/null; then
    echo ""
    echo "========================================="
    echo "Testing SDF Conversion"
    echo "========================================="

    if [ -f "$URDF_FILE" ]; then
        echo ""
        echo "Converting humanoid_12dof.urdf to SDF"

        if gz sdf -p "$URDF_FILE" > /tmp/humanoid.sdf 2>/tmp/gz_sdf_errors.txt; then
            pass "URDF to SDF conversion successful"

            # Validate SDF
            if gz sdf --check /tmp/humanoid.sdf > /tmp/gz_sdf_check.txt 2>&1; then
                pass "Generated SDF is valid"
            else
                fail "Generated SDF validation failed"
                cat /tmp/gz_sdf_check.txt
            fi
        else
            fail "URDF to SDF conversion failed"
            cat /tmp/gz_sdf_errors.txt
        fi
    fi
fi

# Summary
echo ""
echo "========================================="
echo "Validation Summary"
echo "========================================="
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All validations passed!${NC}"
    exit 0
else
    echo -e "${RED}Some validations failed.${NC}"
    exit 1
fi
