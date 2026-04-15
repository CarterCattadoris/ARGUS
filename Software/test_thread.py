import threading
from ultralytics import YOLO
import numpy as np

print("Loading model...")
model = YOLO("../models/best_ncnn_model", task="detect")

def run():
    print("Predicting...")
    img = np.zeros((416, 416, 3), dtype=np.uint8)
    res = model.predict(img, imgsz=416, verbose=False)
    print("Done")

t = threading.Thread(target=run)
t.start()
t.join()
