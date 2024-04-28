import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def create_transform(t_x, t_y, theta):
    translation = np.array([t_x, t_y])
    rotation = rotation_matrix(theta)
    
    transform = np.eye(3, dtype=float)
    transform[:2, :2] = rotation
    transform[:2, 2] = translation
    
    return transform

def apply_transform(transform, x):
    x_h = np.ones((x.shape[0] + 1, x.shape[1]), dtype=x.dtype)
    x_h[:-1, :] = x
    x_t = np.dot(transform, x_h)
    return x_t[:2] / x_t[-1]

def transform_poses(estimated_df, reference_poses=[0, 0, 0]):
    est_global = create_transform(estimated_df.iloc[0]['x'], estimated_df.iloc[0]['y'], estimated_df.iloc[0]['theta']) # 3*3
    ref_global = create_transform(reference_poses[0], reference_poses[1], reference_poses[2]) # 3*3
    # M(ref, est) = M(ref, global) * M(est, global).T 
    ref_rel_est = ref_global @ np.linalg.inv(est_global)  # This is the transformation from est to ref
    ref_rel_est_angle = reference_poses[2] - estimated_df.iloc[0]['theta'] 
    
    transformed_df = estimated_df.copy()
    for index, row in transformed_df.iterrows():
        est_pose = np.array([[row['x'], row['y']]]).T
        transformed_pose = apply_transform(ref_rel_est, est_pose) 
        transformed_df.at[index, 'x'] = transformed_pose[0]
        transformed_df.at[index, 'y'] = transformed_pose[1]
        transformed_df.at[index, 'theta'] = row['theta'] + ref_rel_est_angle
    
    return transformed_df

def plot_trajectory(df, label, color='blue'):
    plt.plot(df['x'], df['y'], label=label, color=color)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Trajectory')
    plt.legend()

def main():
    parser = argparse.ArgumentParser(description="Plot trajectories from CSV files")
    parser.add_argument("--est", type=str, help="Path to the estimated pose CSV file")
    parser.add_argument("--ref", type=str, help="Path to the reference pose CSV file")
    parser.add_argument("--output", "-o", type=str, default="trajectory_plot.png", help="Output file path for saving the figure")
    args = parser.parse_args()

    estimated_df = pd.read_csv(args.est)
    reference_df = pd.read_csv(args.ref)

    # Transform poses to global frame (0, 0, 0 is default)
    aligned_estimated_df = transform_poses(estimated_df)
    aligned_reference_df = transform_poses(reference_df)

    plot_trajectory(aligned_reference_df, label='reference_pose', color='red')
    plot_trajectory(aligned_estimated_df, label='estimated_pose', color='blue')

    plt.savefig(args.output)

    plt.show()

if __name__ == "__main__":
    main()


### Run: python plot_trajectory.py --est estimated_pose.csv --ref reference_pose.csv -o trajectory_plot.png
