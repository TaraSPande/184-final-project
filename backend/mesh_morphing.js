import { Vec3 } from './vertex.js';

export function uniformSample(vertices, sampleCount) {
    const sampled = [];
    const step = Math.floor(vertices.length / sampleCount);
    for (let i = 0; i < sampleCount; i++) {
        sampled.push(vertices[i * step]);
    }
    return sampled;
}

export function findClosestPoints(source, target) {
    const correspondence = [];
    for (let i = 0; i < source.length; i++) {
        let minDist = Infinity;
        let closestIdx = 0;
        for (let j = 0; j < target.length; j++) {
            const dx = source[i].x - target[j].x;
            const dy = source[i].y - target[j].y;
            const dz = source[i].z - target[j].z;
            const dist = dx * dx + dy * dy + dz * dz;
            if (dist < minDist) {
                minDist = dist;
                closestIdx = j;
            }
        }
        correspondence.push(closestIdx);
    }
    return correspondence;
}

export function morphMeshes(startVertices, endVertices, t) {
    // First uniformly sample both meshes to have the same number of vertices
    const sampleCount = Math.min(startVertices.length, endVertices.length);
    const sampledStart = uniformSample(startVertices, sampleCount);
    const sampledEnd = uniformSample(endVertices, sampleCount);
    
    // Find correspondence between sampled vertices
    const correspondence = findClosestPoints(sampledStart, sampledEnd);
    
    // Perform morphing
    const morphed = [];
    for (let i = 0; i < sampledStart.length; i++) {
        const start = sampledStart[i];
        const end = sampledEnd[correspondence[i]];
        morphed.push(new Vec3(
            start.x * (1 - t) + end.x * t,
            start.y * (1 - t) + end.y * t,
            start.z * (1 - t) + end.z * t
        ));
    }
    
    return morphed;
} 