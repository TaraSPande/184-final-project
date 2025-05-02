import { Vec3 } from './vertex.js';

export function findClosestPointsOriginal(source, target) {
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

export function findClosestPointsUnique(source, target) {
    const correspondence = [];
    const used = new Set();
    
    for (let i = 0; i < source.length; i++) {
        let minDist = Infinity;
        let closestIdx = 0;
        for (let j = 0; j < target.length; j++) {
        if (used.has(j)) continue;
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
    used.add(closestIdx);
    }
    return correspondence;
}

export function findClosestPointsBiasExtremes(source, target) {
    const correspondence = [];

    // Step 1: Compute center of target
    const center = new Vec3(0, 0, 0);
    for (let v of target) {
        center.x += v.x;
        center.y += v.y;
        center.z += v.z;
    }
    center.x /= target.length;
    center.y /= target.length;
    center.z /= target.length;

    // Step 2: Precompute "extremeness" (distance from center)
    const targetScores = target.map(v => {
        const dx = v.x - center.x;
        const dy = v.y - center.y;
        const dz = v.z - center.z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    });

    const maxScore = Math.max(...targetScores);

    // Step 3: Match each source point to best-scoring target
    for (let i = 0; i < source.length; i++) {
        let bestIdx = 0;
        let bestScore = Infinity;
        for (let j = 0; j < target.length; j++) {
            const dx = source[i].x - target[j].x;
            const dy = source[i].y - target[j].y;
            const dz = source[i].z - target[j].z;
            const dist = dx * dx + dy * dy + dz * dz;

            // Penalize matching to center-ish points
            const extremeness = targetScores[j] / maxScore; // [0, 1]
            const biasedDist = dist / (0.1 + extremeness);  // Boost far points

            if (biasedDist < bestScore) {
                bestScore = biasedDist;
                bestIdx = j;
            }
        }
        correspondence.push(bestIdx);
    }

    return correspondence;
}

export function findClosestPointsBiasedUnique(source, target) {
    const correspondence = [];

    // Step 1: Compute center of target mesh
    const center = new Vec3(0, 0, 0);
    for (const v of target) {
        center.x += v.x;
        center.y += v.y;
        center.z += v.z;
    }
    center.x /= target.length;
    center.y /= target.length;
    center.z /= target.length;

    // Step 2: Compute "extremeness" = distance from center
    const targetScores = target.map(v => {
        const dx = v.x - center.x;
        const dy = v.y - center.y;
        const dz = v.z - center.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    });

    const maxScore = Math.max(...targetScores);

    // Step 3: For each source point, find best available (unused) target point
    const used = new Set();
    for (let i = 0; i < source.length; i++) {
        let bestIdx = -1;
        let bestScore = Infinity;

        for (let j = 0; j < target.length; j++) {
            if (used.has(j)) continue;

            const dx = source[i].x - target[j].x;
            const dy = source[i].y - target[j].y;
            const dz = source[i].z - target[j].z;
            const dist = dx * dx + dy * dy + dz * dz;

            const extremeness = targetScores[j] / (maxScore || 1); // normalize to [0,1]
            const biasedDist = dist / (0.1 + extremeness); // lower bias means preference for extreme

            if (biasedDist < bestScore) {
                bestScore = biasedDist;
                bestIdx = j;
            }
        }

        if (bestIdx !== -1) {
            correspondence.push(bestIdx);
            used.add(bestIdx);
        } else {
            // fallback if we run out of unused points
            correspondence.push(0);
        }
    }

    return correspondence;
}