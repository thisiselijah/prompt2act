#!/usr/bin/env python3
"""
Python Version Helper Script
This script helps manage different Python versions in the ROS container.
"""

import subprocess
import sys
import os

def get_python_versions():
    """Get available Python versions"""
    versions = {
        'system': '/usr/bin/python3',
        'python39': '/root/.pyenv/versions/3.9.19/bin/python'
    }
    
    available = {}
    for name, path in versions.items():
        if os.path.exists(path):
            try:
                result = subprocess.run([path, '--version'], 
                                      capture_output=True, text=True)
                available[name] = {
                    'path': path,
                    'version': result.stdout.strip()
                }
            except:
                pass
    
    return available

def run_with_python39(script_path, *args):
    """Run a Python script with Python 3.9"""
    python39_path = '/root/.pyenv/versions/3.9.19/bin/python'
    if os.path.exists(python39_path):
        cmd = [python39_path, script_path] + list(args)
        subprocess.run(cmd)
    else:
        print("Python 3.9 not found!")
        sys.exit(1)

def main():
    if len(sys.argv) < 2:
        print("Available Python versions:")
        versions = get_python_versions()
        for name, info in versions.items():
            print(f"  {name}: {info['version']} ({info['path']})")
        print("\nUsage:")
        print("  python3 python_version_helper.py show - Show available versions")
        print("  python3 python_version_helper.py run39 <script.py> - Run script with Python 3.9")
        return
    
    command = sys.argv[1]
    
    if command == 'show':
        versions = get_python_versions()
        for name, info in versions.items():
            print(f"{name}: {info['version']} ({info['path']})")
    
    elif command == 'run39':
        if len(sys.argv) < 3:
            print("Please provide a Python script to run")
            sys.exit(1)
        run_with_python39(*sys.argv[2:])
    
    else:
        print(f"Unknown command: {command}")

if __name__ == '__main__':
    main()
