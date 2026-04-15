import os
import random
import shutil

def split_data():
    base_path = '/home/eon/CSE398/ARGUS/models/annotatedData'
    train_img_path = os.path.join(base_path, 'train', 'images')
    train_lbl_path = os.path.join(base_path, 'train', 'labels')

    val_img_path = os.path.join(base_path, 'valid', 'images')
    val_lbl_path = os.path.join(base_path, 'valid', 'labels')

    test_img_path = os.path.join(base_path, 'test', 'images')
    test_lbl_path = os.path.join(base_path, 'test', 'labels')

    for p in [val_img_path, val_lbl_path, test_img_path, test_lbl_path]:
        os.makedirs(p, exist_ok=True)

    # Get list of images
    images = [f for f in os.listdir(train_img_path) if f.endswith(('.jpg', '.png', '.jpeg'))]
    random.seed(42)  # For reproducibility
    random.shuffle(images)

    total_images = len(images)
    print(f"Total images found: {total_images}")

    # Calculate split sizes (e.g., 70% train, 15% val, 15% test)
    val_count = int(total_images * 0.15)
    test_count = int(total_images * 0.15)
    
    val_images = images[:val_count]
    test_images = images[val_count:val_count+test_count]

    print(f"Moving {len(val_images)} images to validation set...")
    for img in val_images:
        # Move image
        shutil.move(os.path.join(train_img_path, img), os.path.join(val_img_path, img))
        # Move corresponding label
        lbl = os.path.splitext(img)[0] + '.txt'
        lbl_src = os.path.join(train_lbl_path, lbl)
        if os.path.exists(lbl_src):
            shutil.move(lbl_src, os.path.join(val_lbl_path, lbl))

    print(f"Moving {len(test_images)} images to test set...")
    for img in test_images:
        # Move image
        shutil.move(os.path.join(train_img_path, img), os.path.join(test_img_path, img))
        # Move corresponding label
        lbl = os.path.splitext(img)[0] + '.txt'
        lbl_src = os.path.join(train_lbl_path, lbl)
        if os.path.exists(lbl_src):
            shutil.move(lbl_src, os.path.join(test_lbl_path, lbl))

    print(f"Done. Remaining in train: {total_images - len(val_images) - len(test_images)}")

if __name__ == '__main__':
    split_data()
