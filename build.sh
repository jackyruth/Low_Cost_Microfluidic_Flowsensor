#!/usr/bin/env bash

print_help() {
    echo "Usage:"
    echo "./build.sh [-fcnh] [PROGRAM] "
    echo ""
    echo "Positional Args:"
    echo "      PROGRAM (program directory under microvalve/programs/*)"
    echo ""
    echo "Optional Flags: "
    echo "      -f     flash to board"
    echo "      -c     clean build directory"
    echo "      -n     no build option, skips build"
    echo "      -h     help (this output)"
}

FLASH="false"
CLEAN="false"
NO_BUILD="false"
BOARD_SELECTION="verdi_l432kb"
WORKSPACE_DIR=$(pwd)

while getopts "fcnh" OPTION; do
    case $OPTION in

    f)
        FLASH="true"
        ;;
    c)
        CLEAN="true"
        ;;
    n)
        NO_BUILD="true"
        ;;
    h)
        print_help
        exit 0
        ;;
    ?)
        echo "ERROR: Option not recognized:"
        print_help
        exit 0
        ;;

    esac
done

PROGRAM_SELECTION=${@: -1}

if [ -z "$PROGRAM_SELECTION" ]; then
    echo "ERROR: No program selected"
    print_help
    exit 0
fi


if [ $NO_BUILD = "true" ]; then
    echo "NO_BUILD set, skipping build..."
else
    echo "Building ${PROGRAM_SELECTION} ..."
    if [ $CLEAN = "true" ]; then
        west build -b ${BOARD_SELECTION} --pristine -s microvalve/programs/${PROGRAM_SELECTION} -- -DBOARD_ROOT=${WORKSPACE_DIR}/microvalve
    else
        west build -b ${BOARD_SELECTION} -s microvalve/programs/${PROGRAM_SELECTION} -- -DBOARD_ROOT=${WORKSPACE_DIR}/microvalve
    fi
fi

if [ $FLASH = "true" ]; then
    west flash
fi
