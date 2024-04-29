# Find best transform between two trajectories

This program finds the best transform between two trajectories such that the dtw cost between two trajectories achieve lowest point

## Run
```bash
python findBestTransform.py --ref <ref csv path> --est <est csv path> --outcsv [output trasformed est]
```

Result:
```bash
python findBestTransform.py --ref reference_pose.csv --est estimated_pose.csv
Epoch [10/100], Loss: 0.3356
Epoch [20/100], Loss: 0.3303
Epoch [30/100], Loss: 0.3213
Epoch [40/100], Loss: 0.3165
Epoch [50/100], Loss: 0.3141
Epoch [60/100], Loss: 0.3143
Epoch [70/100], Loss: 0.3138
Epoch [80/100], Loss: 0.3150
Epoch [90/100], Loss: 0.3148
Epoch [100/100], Loss: 0.3140


====================================
Best transform:
Tanslation: tensor([0.2464, 0.3453])
Rotation: tensor([0.0724])
Loss: tensor(0.3130)
====================================
```
