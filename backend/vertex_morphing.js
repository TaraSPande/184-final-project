import { Vec3 } from './vertex.js';

export const MeshDeformation = {
	bezierInterpolate(start, end, t) {
		const result = [];
		for (let i = 0; i < start.length; i++) {
			result.push(start[i].mul(1 - t).add(end[i].mul(t)));
		}
		return result;
	},

	laplacianSmoothing(vertices, adjacencyList, iterations) {
		for (let iter = 0; iter < iterations; iter++) {
			const smoothed = [];
			for (let i = 0; i < vertices.length; i++) {
				let sum = new Vec3();
				let count = 0;
				for (let neighbor of adjacencyList[i]) {
					sum = sum.add(vertices[neighbor]);
					count++;
				}
				smoothed[i] = sum.mul(1 / count);
			}
			for (let i = 0; i < vertices.length; i++) {
				vertices[i] = smoothed[i];
			}
		}
	}
};

export function calculateCentroid(points) {
	let centroid = new Vec3();
	for (const p of points) {
		centroid = centroid.add(p);
	}
	return centroid.mul(1 / points.length);
}

export function findClosestPoints(source, target) {
	const closest = [];
	for (let i = 0; i < source.length; i++) {
		let minDist = Infinity;
		let closestIdx = -1;
		for (let j = 0; j < target.length; j++) {
			const dist = source[i].sub(target[j]).magnitude();
			if (dist < minDist) {
				minDist = dist;
				closestIdx = j;
			}
		}
		closest[i] = closestIdx;
	}
	return closest;
}

export function applyTransformation(source, translation, rotationMatrix) {
	for (let i = 0; i < source.length; i++) {
		const v = source[i];
		const xNew = rotationMatrix[0][0] * v.x + rotationMatrix[0][1] * v.y + rotationMatrix[0][2] * v.z;
		const yNew = rotationMatrix[1][0] * v.x + rotationMatrix[1][1] * v.y + rotationMatrix[1][2] * v.z;
		const zNew = rotationMatrix[2][0] * v.x + rotationMatrix[2][1] * v.y + rotationMatrix[2][2] * v.z;
		source[i] = new Vec3(xNew + translation.x, yNew + translation.y, zNew + translation.z);
	}
}

export function ICP(source, target, maxIterations = 10) {
	for (let iter = 0; iter < maxIterations; iter++) {
		const closestPoints = findClosestPoints(source, target);
		const sourceCentroid = calculateCentroid(source);
		const targetMatched = closestPoints.map(i => target[i]);
		const targetCentroid = calculateCentroid(targetMatched);

		// Dummy rotation matrix (identity); real implementation needs SVD
		const rotationMatrix = [
			[1, 0, 0],
			[0, 1, 0],
			[0, 0, 1]
		];
		const translation = targetCentroid.sub(sourceCentroid);
		applyTransformation(source, translation, rotationMatrix);
	}
}