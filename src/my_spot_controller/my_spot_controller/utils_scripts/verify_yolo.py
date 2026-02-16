#!/usr/bin/env python3
import sys
print("=" * 60)
print("🔍 Verifying Ultralytics installation...")
print("=" * 60)

# Check imports
try:
    import cv2
    print(f"✅ OpenCV: {cv2.__version__}")
except ImportError as e:
    print(f"❌ OpenCV import failed: {e}")
    sys.exit(1)

try:
    import torch
    print(f"✅ PyTorch: {torch.__version__}")
    print(f"   CUDA available: {torch.cuda.is_available()}")
except ImportError as e:
    print(f"❌ PyTorch import failed: {e}")
    sys.exit(1)

try:
    from ultralytics import YOLO
    print("✅ Ultralytics imported successfully")
except ImportError as e:
    print(f"❌ Ultralytics import failed: {e}")
    sys.exit(1)

# Download YOLO11 models
print("\n🚀 Downloading YOLO11 pose models...")
try:
    model = YOLO('yolo11n-pose.pt')
    print("✅ yolo11n-pose.pt downloaded")
    
    # Test inference
    import numpy as np
    dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
    results = model(dummy_img, verbose=False)
    print("✅ Model inference test passed")
    
    # Optional TensorRT export
    if torch.cuda.is_available():
        print("\n⚙️ Exporting TensorRT model (GPU detected)...")
        try:
            model.export(format='engine', half=True, device=0)
            print("✅ TensorRT export successful")
        except Exception as e:
            print(f"⚠️ TensorRT export failed: {e}")
    
except Exception as e:
    print(f"❌ Model download/test failed: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("✅ YOLO11 installation complete!")
print("=" * 60)
