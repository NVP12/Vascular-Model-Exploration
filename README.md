
# 🧠 Vascular Model Exploration in MuJoCo

This repository contains the simulation framework developed as part of the **RPM Lab Programming Challenge** (University of Minnesota, Fall 2025 intake). The goal is to simulate navigation through human vascular structures using a dynamic camera (representing a catheter tip) inside the MuJoCo physics engine.

---

## 📌 Objective

To create an interactive MuJoCo simulation that enables:
- Loading and visualizing STL-based 3D vascular models.
- Manual navigation through the vasculature using keyboard inputs.
- Dual-view rendering: one for external/global view and another from the first-person camera’s point-of-view inside the vessels.

This project aims to demonstrate visual understanding and simulation capabilities for robotic navigation in medical environments.

---

## 🧩 Features

- ✅ Loads and scales STL files of neurovascular structures.
- ✅ Supports mesh downsampling for rendering efficiency.
- ✅ Implements a controllable "camera drone" that mimics catheter navigation.
- ✅ Renders two windows:
  - **Global View** – static camera to track the navigation inside vessels.
  - **FPV (First-Person View)** – immersive view from inside the vessels.
- ✅ Demonstrated on multiple anatomical models.

---

## 📂 Folder Structure

***
Vascular-Model-Exploration/
├── mujoco-3.3.2/ # MuJoCo simulation environment
│ ├── model/
│ │ └── vasculature/ # STL models and MuJoCo XML files
│ └── bin/ # MuJoCo executables (simulate, record, etc.)
├── P1_case1_/ # Sample vascular case with .py and .xml
├── simulate/ # Custom simulation logic (if modified)
└── README.md # Project overview (this file)
***
