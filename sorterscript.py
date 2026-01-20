import os
import shutil
from pathlib import Path
from datetime import datetime, timedelta

DOWNLOADS_PATH = Path.home() / "Downloads"

FILE_TYPES = {
    "Images": [".jpg",".jpeg",".png",".gif"],
    "Documents": [".pdf",".xlsx",".txt",".docx"],
    "Zips": [".zip",".rar",".7z",".7zip",".tar"],
    "Binaries": [".exe",".bat"],
    "Music": [".mp3",".wav",".ogg"]
}

def organize_downloads():
    if not DOWNLOADS_PATH.exists():
        print("Папка не найдена")
        return
    
    offset = datetime.now() - timedelta(days=60)

    for file in DOWNLOADS_PATH.iterdir():
        if file.is_dir():
            continue

        file_mod_time = datetime.fromtimestamp(file.stat().st_mtime)

        if file_mod_time < offset:
            move_file(file, "To_Delete")
        else:
            moved = False
            for category, extensions in FILE_TYPES.items():
                if file.suffix.lower() in extensions:
                    move_file(file,category)
                    moved = True
                    break

            if not moved:
                move_file(file, "Others")
    
    remove_empty_folders()

def remove_empty_folders():
    for item in DOWNLOADS_PATH.iterdir():
        if item.is_dir():
            if not any(item.iterdir()):
                item.rmdir()
                print(f"Удалена пустая папка: {item.name}")

def move_file(file_path, category_name):
    target_dir = DOWNLOADS_PATH / category_name
    target_dir.mkdir(exist_ok=True)

    target_path = target_dir / file_path.name
    counter = 1
    while target_path.exists():
        target_path = target_dir / f"{file_path.stem}_{counter}{file_path.suffix}"
        counter += 1

    try:
        shutil.move(str(file_path), str(target_path))
        print(f"Перемещено: {file_path.name} -> {category_name}/{target_path.name}")
    except Exception as e:
        print(f"Ошибочка при перемещении {file_path.name}: {e}")

if __name__ == "__main__":
    organize_downloads()