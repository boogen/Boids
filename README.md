# BoidController – Unity Boids Simulation

This project implements a **high-performance Boids (flocking behavior) simulation** in Unity.  
It is based on the original work by [Keijiro Takahashi (MIT License, 2014)](https://github.com/keijiro/Boids), extended with **Unity Jobs**, **Burst compilation**, and **Native Collections** for massive flock sizes (10,000+ boids).

---

## ✨ Features

- **Flocking rules**:
  - Alignment (match velocity with neighbors)
  - Cohesion (move toward group center)
  - Separation (avoid crowding)
- **Boundary handling** with soft turning and strength scaling.
- **Spatial hashing** for efficient neighbor search.
- **Unity Job System + Burst** for multithreaded performance.
- **Instanced rendering** (`Graphics.DrawMeshInstanced`) for thousands of boids.
- **Per-boid noise** for natural flock variation.

---

## 📂 Project Structure

- **BoidController.cs**  
  Main script that:
  - Spawns boids.
  - Manages jobs (`BuildHashJob`, `BoidSteerJob`, `IntegrateAndApplyJob`).
  - Updates positions, velocities, and renders instances.
- **Jobs**  
  - `BuildHashJob`: Builds a spatial hash for neighbor queries.  
  - `BoidSteerJob`: Applies flocking rules and boundary constraints.  
  - `IntegrateAndApplyJob`: Integrates positions and applies transforms for rendering.

---


## ⚙️ Inspector Parameters

### Spawn
- `boidCount`: Number of boids to spawn.
- `spawnExtents`: Initial spawn area size.
- `boidPrefab`: Prefab used for each boid.
- `boidsParent`: Optional parent GameObject.

### Behavior
- `neighborRadius`: Neighborhood influence distance.
- `separationRadius`: Minimum spacing between boids.
- `maxSpeed`: Velocity clamp.
- `maxForce`: Steering force clamp.
- `alignmentWeight`: Strength of alignment rule.
- `cohesionWeight`: Strength of cohesion rule.
- `separationWeight`: Strength of separation rule.

### Bounds
- `boundsRadius`: Simulation boundary radius.
- `boundsSoftZone`: Percentage of boundary where turning begins.
- `boundsTurnStrength`: Turning force near boundary.

### Noise
- `speedNoise`: Random jitter in velocity.
- `noiseTimeScale`: Speed of noise variation.

---

## 📊 Performance Notes

- Designed for **10,000+ boids in real-time**.
- Uses `NativeArray` with `UninitializedMemory` for allocation efficiency.
- Batch rendering in chunks of **1023 instances**.
- Parallel neighbor search using **NativeParallelMultiHashMap**.

---

## 📜 License

This project is released under the **MIT License**.  
Originally developed by Keijiro Takahashi, extended for Unity Jobs & Burst.
