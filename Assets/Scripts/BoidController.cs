//
// Boids - Flocking behavior simulation.
//
// Copyright (C) 2014 Keijiro Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
using System.Runtime.CompilerServices;

public class BoidController : MonoBehaviour
{

    [Header("Spawn")]
    public int boidCount = 10000;
    public Vector3 spawnExtents = new Vector3(60f, 10f, 60f);
    public GameObject boidPrefab;
    public GameObject boidsParent;

    [Header("Behavior")]
    [Min(0.1f)] public float neighborRadius = 3.5f;    // influences cell size
    [Min(0.05f)] public float separationRadius = 1.1f;
    public float maxSpeed = 8f;
    public float maxForce = 12f;
    public float alignmentWeight = 1.0f;
    public float cohesionWeight = 0.9f;
    public float separationWeight = 1.5f;

    [Header("Bounds")]
    public float boundsRadius = 70f;
    [Range(0f, 1f)] public float boundsSoftZone = 0.85f; // start turning before edge
    public float boundsTurnStrength = 18f;

    [Range(0f, 1f)] public float speedNoise = 0.12f;             // ±% speed jitter
    public float noiseTimeScale = 0.9f;                           // Hz-ish (Perlin-ish surrogate)

    public float3 FlockCenter { get; private set; }

    // Internal
    Material _material;
    Mesh _mesh;
    NativeArray<float3> _positions;
    NativeArray<float3> _velocities;
    NativeArray<float3> _velocitiesNext;
    NativeArray<Matrix4x4> _matrices;

    // Spatial hash (2D XZ)
    NativeParallelMultiHashMap<int, int> _cellMap;
    float _cellSize; // ~ neighborRadius
    JobHandle _handle;

    void Start()
    {
        if (boidPrefab == null)
        {
            Debug.LogError("Assign a boid prefab.");
            enabled = false;
            return;
        }

        _positions = new NativeArray<float3>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _velocities = new NativeArray<float3>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _velocitiesNext = new NativeArray<float3>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _matrices = new NativeArray<Matrix4x4>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        var meshFilter = boidPrefab.GetComponentInChildren<MeshFilter>();
        var meshRenderer = boidPrefab.GetComponentInChildren<MeshRenderer>();
        _mesh = meshFilter.sharedMesh;
        _material = meshRenderer.sharedMaterial;

        // Spawn
        var rnd = new Unity.Mathematics.Random((uint)System.Environment.TickCount);

        for (int i = 0; i < boidCount; ++i)
        {
            var pos = new float3(
                transform.position.x + rnd.NextFloat(-spawnExtents.x, spawnExtents.x),
                transform.position.y + rnd.NextFloat(-spawnExtents.y, spawnExtents.y),
                transform.position.z + rnd.NextFloat(-spawnExtents.z, spawnExtents.z)
            );

            // random XZ velocity
            var dir = math.normalize(new float3(rnd.NextFloat(-1f, 1f), 0f, rnd.NextFloat(-1f, 1f)));
            var vel = dir * rnd.NextFloat(0.25f * maxSpeed, 0.75f * maxSpeed);

            _positions[i] = pos;
            _velocities[i] = vel;

        }

        // Spatial hash capacity ~ N * (average cell population)
        _cellMap = new NativeParallelMultiHashMap<int, int>(boidCount * 2, Allocator.Persistent);
        _cellSize = math.max(0.75f * neighborRadius, 0.1f);
    }

    void Update()
    {
        float dt = math.min(Time.deltaTime, 1f / 30f);

        _cellMap.Clear();
        var build = new BuildHashJob
        {
            positions = _positions,
            cellSize = _cellSize,
            writer = _cellMap.AsParallelWriter()
        }.Schedule(_positions.Length, 128);


        // 2) Steer
        var steer = new BoidSteerJob
        {
            positions = _positions,
            velocities = _velocities,        // READ
            outVelocities = _velocitiesNext,    // WRITE
            cellMap = _cellMap,
            cellSize = _cellSize,

            neighborRadius = neighborRadius,
            separationRadius = separationRadius,
            maxSpeed = maxSpeed,
            maxForce = maxForce,
            alignmentWeight = alignmentWeight,
            cohesionWeight = cohesionWeight,
            separationWeight = separationWeight,

            // bounds
            center = float3.zero,
            boundsRadius = boundsRadius,
            softZone = boundsSoftZone,
            boundsTurnStrength = boundsTurnStrength,

            // NEW: noise
            time = (float)Time.time,
            noiseTimeScale = noiseTimeScale,
            speedNoise = speedNoise,

            deltaTime = dt
        }.Schedule(_positions.Length, 128, build);

        // 3) Integrate + write to transforms
        var integrate = new IntegrateAndApplyJob
        {
            positions = _positions,
            matrices = _matrices,
            velocities = _velocitiesNext,
            deltaTime = dt
        }.Schedule(_positions.Length, 128, steer);

        _handle = integrate;
        JobHandle.ScheduleBatchedJobs();
    }

    void LateUpdate()
    {
        _handle.Complete();
        var tmp = _velocities;
        _velocities = _velocitiesNext;
        _velocitiesNext = tmp;

        // compute and cache flock center AFTER jobs finished writing
        float3 sum = float3.zero;
        for (int i = 0; i < _positions.Length; i++)
            sum += _positions[i];

        FlockCenter = sum / math.max(1, _positions.Length);

        int batchSize = 1023;
        for (int i = 0; i < boidCount; i += batchSize)
        {
            int count = math.min(batchSize, boidCount - i);
            var array = new Matrix4x4[count];
            NativeArray<Matrix4x4>.Copy(_matrices, i, array, 0, count);
            Graphics.DrawMeshInstanced(_mesh, 0, _material, array);
        }
    }

    void OnDestroy()
    {
        if (_handle.IsCompleted == false) _handle.Complete();

        if (_positions.IsCreated) _positions.Dispose();
        if (_velocities.IsCreated) _velocities.Dispose();
        if (_velocitiesNext.IsCreated) _velocitiesNext.Dispose();
        if (_cellMap.IsCreated) _cellMap.Dispose();
        if (_matrices.IsCreated) _matrices.Dispose();
    }

    [BurstCompile]
    struct BuildHashJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        public float cellSize;
        public NativeParallelMultiHashMap<int, int>.ParallelWriter writer;

        public void Execute(int index)
        {
            var p = positions[index];
            var cell = new int3((int)math.floor(p.x / cellSize),
                                (int)math.floor(p.y / cellSize),
                                (int)math.floor(p.z / cellSize));
            writer.Add(Hash(cell), index);
        }

        static int Hash(int3 c)
        {
            unchecked
            {
                return c.x * 73856093
                     ^ c.y * 19349663
                     ^ c.z * 83492791;
            }
        }
    }

    [BurstCompile]
    struct BoidSteerJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> velocities;
        public NativeArray<float3> outVelocities;

        [ReadOnly] public NativeParallelMultiHashMap<int, int> cellMap;
        public float cellSize;

        // flock weights
        public float neighborRadius, separationRadius;
        public float maxSpeed, maxForce;
        public float alignmentWeight, cohesionWeight, separationWeight;

        // bounds
        public float3 center;
        public float boundsRadius, softZone, boundsTurnStrength;

        // NEW: controller bias
        public float3 controllerForward;
        public float3 controllerPosition;
        public float controllerAlignWeight;
        public float controllerCohesionWeight;

        // NEW: noise
        public float time;
        public float noiseTimeScale;
        public float speedNoise;

        public float deltaTime;


        public void Execute(int i)
        {
            float3 pos = positions[i];
            float3 vel = velocities[i];

            // --- Neighbor accumulation ---
            float3 sumAlign = 0f;
            float3 sumCoh = 0f;
            float3 sumSep = 0f;
            int neighborCount = 0;

            int cellX = (int)math.floor(pos.x / cellSize);
            int cellY = (int)math.floor(pos.y / cellSize);
            int cellZ = (int)math.floor(pos.z / cellSize);
            float nRadSqr = neighborRadius * neighborRadius;
            float sRadSqr = separationRadius * separationRadius;

            for (int dz = -1; dz <= 1; dz++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        var c = new int3(cellX + dx, cellY + dy, cellZ + dz);
                        NativeParallelMultiHashMapIterator<int> it;
                        int idx;

                        if (cellMap.TryGetFirstValue(Hash(c), out idx, out it))
                        {
                            do
                            {
                                if (idx == i) continue;

                                float3 p2 = positions[idx];
                                float3 to = p2 - pos;
                                float d2 = math.lengthsq(to);

                                if (d2 <= nRadSqr)
                                {
                                    neighborCount++;

                                    sumAlign += velocities[idx];
                                    sumCoh += p2;

                                    if (d2 <= sRadSqr && d2 > 1e-6f)
                                    {
                                        // push away (stronger when closer)
                                        sumSep -= to / math.max(1e-3f, math.sqrt(d2));
                                    }
                                }
                            }
                            while (cellMap.TryGetNextValue(out idx, ref it));
                        }
                    }
                }
            }

            float3 accel = 0f;

            // --- Flocking steers ---
            if (neighborCount > 0)
            {
                float inv = 1f / neighborCount;

                // Alignment
                float3 desiredAlign = SafeNormalize(sumAlign * inv) * maxSpeed;
                float3 steerAlign = Limit(desiredAlign - vel, maxForce);
                accel += steerAlign * alignmentWeight;

                // Cohesion
                float3 centerOfMass = sumCoh * inv;
                float3 desiredCoh = SafeNormalize(centerOfMass - pos) * maxSpeed;
                float3 steerCoh = Limit(desiredCoh - vel, maxForce);
                accel += steerCoh * cohesionWeight;

                // Separation
                float3 desiredSep = math.lengthsq(sumSep) > 0f ? SafeNormalize(sumSep) * maxSpeed : 0f;
                float3 steerSep = Limit(desiredSep - vel, maxForce);
                accel += steerSep * separationWeight;
            }


            // --- Bounds steering (soft) ---
            float dist = math.length(pos - center);
            float trigger = boundsRadius * softZone;
            if (dist > trigger)
            {
                float3 desired = SafeNormalize(center - pos) * maxSpeed;
                float t = math.saturate((dist - trigger) / math.max(1e-3f, boundsRadius - trigger));
                float3 steer = Limit(desired - vel, maxForce) * (boundsTurnStrength * t);
                accel += steer;
            }

            // --- Integrate, clamp, noise, drag ---
            vel += accel * deltaTime;

            // clamp speed
            float spd = math.length(vel);
            if (spd > maxSpeed) vel *= (maxSpeed / math.max(spd, 1e-6f));

            // mild per-boid speed noise (1 ± speedNoise)
            if (speedNoise > 0f)
            {
                // deterministic phase per boid + evolving over time
                uint h = (uint)i;
                h ^= 2747636419u; h *= 2654435769u; h ^= h >> 16; h *= 2654435769u; h ^= h >> 16;
                float phase = (h & 0x00FFFFFFu) / 16777216f * 100f; // [0,100)
                float wiggle = math.sin((time * noiseTimeScale) + phase); // [-1,1]
                float scale = 1f + (wiggle * speedNoise);
                vel *= scale;
            }

            // tiny drag to kill outward spiral drift
            vel *= (1f - 0.05f * deltaTime);

            outVelocities[i] = vel;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3 Limit(float3 v, float max)
        {
            float l = math.length(v);
            if (l > max) return v * (max / math.max(l, 1e-6f));
            return v;
        }

        static int Hash(int3 c)
        {
            unchecked
            {
                return c.x * 73856093
                     ^ c.y * 19349663
                     ^ c.z * 83492791;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3 SafeNormalize(float3 v)
        {
            float l = math.length(v);
            return l > 1e-6f ? v / l : float3.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float Hash01(uint n)
        {
            // fast uint hash → [0,1)
            n ^= 2747636419u; n *= 2654435769u; n ^= n >> 16; n *= 2654435769u; n ^= n >> 16;
            return (n & 0x00FFFFFFu) / 16777216f; // 24-bit mantissa slice
        }
    }

    [BurstCompile]
    struct IntegrateAndApplyJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> velocities;
        public NativeArray<Matrix4x4> matrices;
        public float deltaTime;

        public void Execute(int index)
        {
            float3 p = positions[index] + velocities[index] * deltaTime;
            positions[index] = p;


            quaternion rot = quaternion.identity;
            float3 v = velocities[index];
            if (math.lengthsq(v) > 1e-6f)
                rot = quaternion.LookRotationSafe(new float3(v.x, 0f, v.z), math.up());

            matrices[index] = Matrix4x4.TRS(p, rot, Vector3.one);
        }
    }
}
