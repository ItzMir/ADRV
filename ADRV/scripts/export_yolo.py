#!/usr/bin/env python3
# ~/Documents/adrv/scripts/export_yolo.py
"""
One-shot: download YOLOv8n and export to NCNN for the Raspberry Pi.
Run once, from the project root, with the venv active:

    python scripts/export_yolo.py

Result: models/yolov8n_ncnn_model/  (what config.py expects)
"""
import shutil
import sys
from pathlib import Path


def main():
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed in this venv.")
        print("Fix: pip install -r requirements.txt")
        sys.exit(1)

    project_root = Path(__file__).resolve().parent.parent
    models_dir = project_root / "models"
    models_dir.mkdir(exist_ok=True)

    print("[export_yolo] Loading yolov8n.pt (downloads once, ~6 MB)...")
    model = YOLO("yolov8n.pt")

    print("[export_yolo] Exporting to NCNN @ imgsz=320 ...")
    export_path = Path(model.export(format="ncnn", imgsz=320, half=False,
                                    int8=False, dynamic=False, simplify=True))
    print(f"[export_yolo] Raw export: {export_path}")

    dest = models_dir / "yolov8n_ncnn_model"
    if dest.exists():
        shutil.rmtree(dest)
    shutil.move(str(export_path), str(dest))
    print(f"[export_yolo] Installed to: {dest}")

    required = ["model.ncnn.param", "model.ncnn.bin", "metadata.yaml"]
    missing = [f for f in required if not (dest / f).exists()]
    if missing:
        print(f"[export_yolo] WARNING missing: {missing}")
        sys.exit(2)
    print("[export_yolo] All required NCNN files present. Done.")


if __name__ == "__main__":
    main()
