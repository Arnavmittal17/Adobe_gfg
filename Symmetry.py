import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.spatial.distance import euclidean
from scipy.interpolate import splprep, splev

def read_csv_(csv_path):
    np_path_XYs = np.genfromtxt(csv_path, delimiter=',')
    path_XYs = []
    for i in np.unique(np_path_XYs[:, 0]):
        npXYs = np_path_XYs[np_path_XYs[:, 0] == i][:, 1:]
        XYs = []
        for j in np.unique(npXYs[:, 0]):
            XY = npXYs[npXYs[:, 0] == j][:, 1:]
            XYs.append(XY)
        path_XYs.append((i, XYs))
    return path_XYs

def check_horizontal_symmetry(XY):
    """Check if the shape has horizontal symmetry."""
    if len(XY) == 0:
        return False
    
    x_mean = np.mean(XY[:, 0])
    mirrored_XY = np.copy(XY)
    mirrored_XY[:, 0] = 2 * x_mean - mirrored_XY[:, 0]
    
    return np.allclose(np.sort(XY, axis=0), np.sort(mirrored_XY, axis=0), atol=1e-3)

def check_vertical_symmetry(XY):
    """Check if the shape has vertical symmetry."""
    if len(XY) == 0:
        return False
    
    y_mean = np.mean(XY[:, 1])
    mirrored_XY = np.copy(XY)
    mirrored_XY[:, 1] = 2 * y_mean - mirrored_XY[:, 1]
    
    return np.allclose(np.sort(XY, axis=0), np.sort(mirrored_XY, axis=0), atol=1e-3)

def rotate(XY, angle):
    """Rotate the shape by the given angle."""
    theta = np.deg2rad(angle)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return np.dot(XY, R.T)

def check_rotational_symmetry(XY, angles=[90, 180, 270]):
    """Check if the shape has rotational symmetry."""
    def check_rotation(XY, angle):
        """Check if the shape matches itself after rotation."""
        rotated_XY = rotate(XY, angle)
        return np.allclose(np.sort(XY, axis=0), np.sort(rotated_XY, axis=0), atol=1e-3)

    return any(check_rotation(XY, angle) for angle in angles)

def analyze_symmetry(XYs):
    results = []
    for XY in XYs:
        symmetries = {
            "Horizontal": check_horizontal_symmetry(XY),
            "Vertical": check_vertical_symmetry(XY),
            "Rotational": check_rotational_symmetry(XY)
        }
        results.append(symmetries)
    return results

def plot_shapes_with_symmetry(paths_XYs):
    colors = ['red', 'green', 'blue']
    fig, ax = plt.subplots(tight_layout=True, figsize=(10, 10))
    
    for i, (shape_id, XYs) in enumerate(paths_XYs):
        c = colors[i % len(colors)]
        for XY in XYs:
            ax.plot(XY[:, 0], XY[:, 1], c=c, linewidth=2)
            
            if check_horizontal_symmetry(XY):
                x_mean = np.mean(XY[:, 0])
                ax.axvline(x=x_mean, color=c, linestyle='--')
            if check_vertical_symmetry(XY):
                y_mean = np.mean(XY[:, 1])
                ax.axhline(y=y_mean, color=c, linestyle='--')
            if check_rotational_symmetry(XY):
                for angle in [90, 180, 270]:
                    rotated_XY = rotate(XY, angle)
                    ax.plot(rotated_XY[:, 0], rotated_XY[:, 1], c=c, linestyle=':')

    ax.set_aspect('equal')
    plt.show()

csv_path = "/content/occlusion2_sol.csv"

# Read the input CSV
paths_XYs = read_csv_(csv_path)

# Analyze the symmetry of each shape
symmetry_results = {}
for shape_id, XYs in paths_XYs:
    symmetry_results[shape_id] = analyze_symmetry(XYs)

# Print symmetry results
for shape_id, results in symmetry_results.items():
    print(f"Shape ID {shape_id}:")
    for idx, result in enumerate(results):
        print(f"  Subshape {idx}:")
        for sym_type, is_symmetrical in result.items():
            print(f"    {sym_type} Symmetry: {'Yes' if is_symmetrical else 'No'}")

# Plot shapes with symmetry checks
plot_shapes_with_symmetry(paths_XYs)
