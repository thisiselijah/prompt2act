# Python Version Management in ROS Container

This container has two Python versions installed:

## Available Python Versions

1. **System Python 3.8** (`/usr/bin/python3`)
   - Used by ROS Noetic (default)
   - Compatible with ROS packages
   - Default when running `python3`

2. **Python 3.9** (`/root/.pyenv/versions/3.9.19/bin/python`)
   - Required for `google-genai` package
   - Managed by pyenv
   - Available via aliases and helper scripts

## Usage Examples

### Check Available Versions
```bash
python3 /usr/local/bin/python_version_helper.py show
```

### Run Scripts with Python 3.9
```bash
# Using the helper script
python3 /usr/local/bin/python_version_helper.py run39 your_script.py

# Using the alias (after sourcing bashrc)
python39 your_script.py

# Using the wrapper script
use-python39 python your_script.py
```

### Install Packages for Specific Python Versions
```bash
# Install for Python 3.8 (system/ROS)
python3 -m pip install package_name

# Install for Python 3.9
python39 -m pip install package_name
# or
pip39 install package_name
```

### Example: Using google-genai
```python
#!/usr/bin/env python3
# This script should be run with Python 3.9
import google.generativeai as genai

# Your code here...
```

Run it with:
```bash
python39 your_genai_script.py
```

## Environment Variables

- `PYTHON39_PATH`: Points to Python 3.9 executable
- Aliases are available in bash: `python39`, `pip39`

## Notes

- ROS nodes should generally use system Python 3.8 for compatibility
- Use Python 3.9 specifically for packages that require it (like google-genai)
- The pyenv environment is automatically sourced in bash sessions
