# Find best transform between two trajectories

This program finds the best transform between two trajectories such that the dtw cost between two trajectories achieve lowest point

## Run
```bash
python findBestTransform.py --ref reference_pose.csv --est estimated_pose.csv
```

Result:
```bash
python findBestTransform.py --ref reference_pose.csv --est estimated_pose.csv
Epoch [10/100], Loss: 0.5253
Epoch [20/100], Loss: 0.5123
Epoch [30/100], Loss: 0.5139
Epoch [40/100], Loss: 0.5134
Epoch [50/100], Loss: 0.5124
Epoch [60/100], Loss: 0.5122
Epoch [70/100], Loss: 0.5123
Epoch [80/100], Loss: 0.5122
Epoch [90/100], Loss: 0.5121
Epoch [100/100], Loss: 0.5122
Model parameters after training:
translation tensor([-0.1446,  0.2797])
rotation tensor([-0.0478])
```
