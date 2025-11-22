# 🦾 MuJoCo Robotic Arm Depth Reconstruction

### Simulating depth perception and 3D surface reconstruction with a robotic arm in **MuJoCo**.

<p align="center">
  <img src="https://raw.githubusercontent.com/Sjschhabra/mujoco-depth-reconstruction/refs/heads/main/images/newplot.png"
       width="47%" alt="3D Reconstruction Plot"/>
  <img src="https://raw.githubusercontent.com/Sjschhabra/mujoco-depth-reconstruction/refs/heads/main/images/download.gif"
       width="47%" alt="MuJoCo Simulation Screenshot"/>
  <img src="https://raw.githubusercontent.com/Sjschhabra/mujoco-depth-reconstruction/refs/heads/main/images/download2.gif"
       width="47%" alt="MuJoCo Simulation Screenshot"/>
</p>

---

## 🧩 Overview

This project simulates a **robotic arm mounted on a linear slider** equipped with a **stereo depth camera** on its end effector.
The arm moves through space and captures **depth images** of multiple objects placed in its environment.

The captured noisy depth maps are then used to **reconstruct a 3D surface** of the scene by transforming image coordinates into world coordinates and fusing them across multiple camera poses.

---

## ⚙️ Simulation Setup

* **Environment:** MuJoCo (8 DoF arm + slider)
* **Sensor:** Stereo depth camera on end effector (`fovy ≈ 90°`)
* **Scene Objects:** Sphere, cube, and cylinder
* **Noise Model:** Gaussian depth noise proportional to range
* **Output:** Reconstructed 3D point cloud and smoothed mesh surface

---

## 🧠 Pipeline

1. **Simulation & Depth Capture**

   * The arm follows a trajectory across the environment.
   * At each step, the end-effector camera captures a **noisy depth image**.

2. **Camera-to-World Transformation**

   * Apply the **camera intrinsic matrix** derived from FoV.
   * Compute `(X, Y, Z)` real-world coordinates using depth projection equations.
   * Transform points to world frame via camera rotation and translation.

3. **Point Cloud Fusion**

   * Merge and filter all valid 3D points.
   * Clip to region of interest:

     ```python
     x_min, x_max = 0.09, 1
     y_min, y_max = 0.05, 1
     z_min, z_max = 0.06, 3
     ```

4. **Surface Reconstruction**

   * Convert fused point cloud to `open3d.geometry.PointCloud`
   * Estimate normals and run **Poisson surface reconstruction**:

     ```python
     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
     ```
   * Clean up low-density vertices and smooth the mesh.

5. **Visualization**

   * Display reconstructed mesh using **Plotly 3D Mesh** for interactive viewing.

---

## 📊 Result

* A realistic robotic perception pipeline — **robot motion → noisy depth → 3D reconstruction**
* Demonstrates how to reconstruct physical geometry from synthetic sensor data inside a physics simulator.

<p align="center">
  <img src="https://raw.githubusercontent.com/Sjschhabra/mujoco-depth-reconstruction/refs/heads/main/images/Screenshot%202025-11-19%20113425.png" width="80%" alt="3D Reconstruction Example">
</p>

---

## 🧰 Tools & Libraries

| Category        | Tools                         |
| --------------- | ----------------------------- |
| Simulation      | [MuJoCo](https://mujoco.org/) |
| Visualization   | Plotly, Open3D                |
| Data Processing | NumPy                         |
| Language        | Python 3                      |

---

## 🚀 Future Work

* Integrate **real-time SLAM** or **ICP** for incremental reconstruction
* Test with **different noise models** or stereo baselines
* Use **real-world depth data** for comparison

---

## 📜 How to Run

```bash
# Clone the repo
git clone https://github.com/Sjschhabra/mujoco-depth-reconstruction.git
cd mujoco-depth-reconstruction

# Install dependencies
pip install mujoco open3d plotly numpy

# Run the simulation
# File: Project.ipynb
```

---

## 🙌 Acknowledgements

This project was inspired by industry applications of robotic perception and 3D reconstruction using simulated depth data.
I built it out of curiosity — and because it brings together several fascinating challenges:
spatial transformations, camera geometry, and robotic kinematics.
