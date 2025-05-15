## Data set Structure 

> [!WARNING]
> Extracted images must save in `images` folder!

```
data_source/ 
├── images/ 
│   ├── folder_1/ 
│   │   ├── {folder_1}_frame_00000.png
│   │   ├── {folder_1}_frame_00000.json
│   │   └── ... 
│   ├── folder_2/ 
│   │   ├── {folder_2}_frame_00000.png 
│   │   └── {folder_2}_frame_00000.json 
│   │   └── ... 
│   └── synthetic/ 
│   |   ├── {synthetic}_frame_00000.png 
│   |   └── {synthetic}_frame_00000.json 
│   │   └── ... 
└── videos/ 
	├── {recieved_date}_teams/
	│   └── {folder_1}.mp4 
	└── {recieved_date}_aws/ 
		└── {folder_2}.mp4
```
### Labeling 
You need install **[Anylabeling](https://github.com/vietanhdev/anylabeling.git)**. 
- [ ] Choose `Polygon` or press `P`.
- [ ] Label the objects.
>[!IMPORTANT]
>Label the each line. Then labeling together but name is same class name. Both are `line`.
>![[Pasted image 20250509163829.png]]
![[Pasted image 20250509164123.png]]

| **class name list** |
| :-----------------: |
|        line         |
|   concrate_guide    |
|    wooden_guide     |

### Training 
- [ ] Prepare a dataset like this structure.
```
data  
├── train  
│   ├── image1.png 
│   └── image1.txt 
└── val
	├── image_test1.png 
	└── image_test2.txt
```
- [ ] Create a `train.py` 
```python 
from ultralytics import YOLO
# Load a model
model = YOLO("yolov8n-seg.pt")  # load a pretrained model (if you aren't installed it is automatically install)

# Train using the single most idle GPU
results = model.train(data="data.yaml", epochs=50, imgsz=640, device=-1)
```
The`data.yaml` is routing the training and validation images folder. 
```yaml
train: './data/train'
val: './data/val'
nc: 3
names: ['line', 'concrate_guide''wooden_guide']
```
