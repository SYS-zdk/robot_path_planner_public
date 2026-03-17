#!/usr/bin/env python3
import sys
from pathlib import Path
import re

OLD = 'robot_path_planner'
NEW = 'robot_path_planner'

# Paths to skip (generated or large directories)
SKIP_DIRS = {'.git', 'build', 'devel', 'install', '.venv', '__pycache__'}
# File extensions to process (text files)
EXTS = {'.py', '.cmake', '.txt', '.md', '.xml', '.launch', '.yaml', '.yml', '.cfg', '.sh', '.cpp', '.c', '.h', '.hpp', '.cc', '.hh', '.json', '.rst', '.in'}

repo_root = Path(__file__).resolve().parents[1]
print('Repo root:', repo_root)

changed_files = []

# Case-preserving replacement function
def case_preserve_replace(match):
    s = match.group(0)
    # if all upper
    if s.isupper():
        return NEW.upper()
    # if all lower
    if s.islower():
        return NEW
    # Title case (first letter upper, rest lower) - for snake case it's unlikely
    if s[0].isupper() and s[1:].islower():
        return NEW.capitalize()
    # Mixed case or other: default to lower
    return NEW

# Regex for case-insensitive search of the exact token
pattern = re.compile(re.escape(OLD), re.IGNORECASE)

for path in repo_root.rglob('*'):
    if path.is_dir():
        if path.name in SKIP_DIRS:
            # skip tree
            # print('Skipping dir', path)
            continue
        else:
            continue
    # skip binary files and files in skip dirs in their path
    if any(part in SKIP_DIRS for part in path.parts):
        continue
    if path.suffix.lower() not in EXTS and path.name not in ('CMakeLists.txt', 'package.xml'):
        continue
    try:
        text = path.read_text(encoding='utf-8')
    except Exception:
        continue
    if re.search(pattern, text):
        new_text = re.sub(pattern, case_preserve_replace, text)
        if new_text != text:
            path.write_text(new_text, encoding='utf-8')
            changed_files.append(str(path.relative_to(repo_root)))

print('Modified files:')
for f in changed_files:
    print(f)

print(f'Total modified: {len(changed_files)}')

# Also suggest next steps
print('\nNote: generated files in `devel/` and `build/` are intentionally skipped.\nIf you want them updated too, please confirm, and/or consider rebuilding the workspace after renaming the source files.')
