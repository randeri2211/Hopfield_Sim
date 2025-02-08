import os
import shutil
from datetime import datetime

def backup_files(source_dir='.'):
    """Backup .py and .json files with timestamped copies"""
    # Create backup directory name
    timestamp = datetime.now().strftime("%d%m%Y_%H%M")
    backup_dir = os.path.join(source_dir, f'backup_{timestamp}')
    
    # Create backup directory
    os.makedirs(backup_dir, exist_ok=True)
    
    # Walk through directory tree
    for root, dirs, files in os.walk(source_dir):
        # Skip directories containing 'backup'
        if 'backup' in root.lower():
            continue
            
        for file in files:
            if file.endswith(('.py', '.json')):
                src_path = os.path.join(root, file)
                
                # Create relative path for destination
                rel_path = os.path.relpath(root, source_dir)
                dest_dir = os.path.join(backup_dir, rel_path)
                os.makedirs(dest_dir, exist_ok=True)
                
                # Copy file with timestamp
                dest_path = os.path.join(dest_dir, f"{os.path.splitext(file)[0]}_{timestamp}{os.path.splitext(file)[1]}")
                shutil.copy2(src_path, dest_path)
                print(f"Backed up: {src_path} -> {dest_path}")

if __name__ == '__main__':
    backup_files()
