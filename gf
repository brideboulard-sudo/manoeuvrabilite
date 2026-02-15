#!/bin/bash

# gf - Project helper script for manoeuvrabilite
# This script provides quick commands for project maneuverability

show_help() {
    cat << EOF
gf - Project Helper Script

Usage: gf [command]

Commands:
    help        Show this help message
    status      Show project status
    info        Show project information

Examples:
    gf help     Display this help message
    gf status   Show current project status
    gf info     Display project information

EOF
}

show_status() {
    echo "Project: manoeuvrabilite"
    echo "Status: Active"
    echo "Files:"
    ls -lh
}

show_info() {
    echo "=== Project Information ==="
    echo "Name: manoeuvrabilite"
    echo "Description: projet 1"
    echo ""
    echo "Repository contents:"
    find . -type f -not -path './.git/*' | sort
}

# Main script logic
case "${1:-help}" in
    help|--help|-h)
        show_help
        ;;
    status)
        show_status
        ;;
    info)
        show_info
        ;;
    *)
        echo "Unknown command: $1"
        echo "Run 'gf help' for usage information"
        exit 1
        ;;
esac
