import { Vec3 } from './vertex.js';

export class VertexInterpolation {
  // Linear interpolation between two vertex positions
  static linearInterpolate(v1, v2, t) {
    return v1.mul(1.0 - t).add(v2.mul(t));
  }

  // Linear interpolation between two meshes (represented as vertex vectors)
  export static linearInterpolateMeshes(mesh1, mesh2, t) {
    if (mesh1.length !== mesh2.length) {
      // Meshes must have the same number of vertices for simple linear interpolation
      return [];
    }

  export const result = new Array(mesh1.length);
    for (let i = 0; i < mesh1.length; i++) {
      result[i] = VertexInterpolation.linearInterpolate(mesh1[i], mesh2[i], t);
    }
    return result;
  }

  // Cubic Bézier curve interpolation for a single vertex
  export static bezierInterpolate(p0, p1, p2, p3, t) {
    const t2 = t * t;
    const t3 = t2 * t;
    const mt = 1.0 - t;
    const mt2 = mt * mt;
    const mt3 = mt2 * mt;

    return p0.mul(mt3)
      .add(p1.mul(3.0 * mt2 * t))
      .add(p2.mul(3.0 * mt * t2))
      .add(p3.mul(t3));
  }

  // Blend shape interpolation (linear combination of multiple meshes)
  export static blendShapes(meshes, weights) {
    if (meshes.length === 0 || weights.length !== meshes.length) {
      return [];
    }

    const vertexCount = meshes[0].length;
    const result = new Array(vertexCount).fill(new Vector3D(0, 0, 0));

    for (let m = 0; m < meshes.length; m++) {
      if (meshes[m].length !== vertexCount) {
        continue; // Skip meshes with incorrect vertex count
      }

      for (let v = 0; v < vertexCount; v++) {
        result[v] = result[v].add(meshes[m][v].mul(weights[m]));
      }
    }

    return result;
  }

  // Catmull-Rom spline interpolation for smooth keyframe animation
  export static catmullRomInterpolate(p0, p1, p2, p3, t) {
    const t2 = t * t;
    const t3 = t2 * t;

    const a = p1.mul(0.5)
      .add(p0.mul(-0.5 * t + 1.5 * t2 - 1.5 * t3))
      .add(p2.mul(2.0 * t - 2.0 * t2 + t3))
      .add(p3.mul(-0.5 * t2 + 0.5 * t3));
    return a;
  }

  // Multi-mesh keyframe interpolation using Catmull-Rom
  export static interpolateKeyframes(keyframes, current, t) {
    if (keyframes.length < 4) {
      return [];
    }

    const prev = current > 0 ? current - 1 : 0;
    const next = current < keyframes.length - 2 ? current + 2 : current + 1;
    const start = current;
    const end = current + 1;

    const result = new Array(keyframes[0].length);

    for (let v = 0; v < keyframes[0].length; v++) {
      result[v] = VertexInterpolation.catmullRomInterpolate(
        keyframes[prev][v],
        keyframes[start][v],
        keyframes[end][v],
        keyframes[next][v],
        t
      );
    }

    return result;
  }

  // Barycentric interpolation for a point in a triangle
  export static barycentricInterpolate(p1, p2, p3, point) {
    const normal = p2.sub(p1).cross(p3.sub(p1)).normalize();
    const projected = point.sub(normal.mul(point.sub(p1).dot(normal)));

    const v0 = p2.sub(p1);
    const v1 = p3.sub(p1);
    const v2 = projected.sub(p1);

    const d00 = v0.dot(v0);
    const d01 = v0.dot(v1);
    const d11 = v1.dot(v1);
    const d20 = v2.dot(v0);
    const d21 = v2.dot(v1);

    const denom = d00 * d11 - d01 * d01;
    if (Math.abs(denom) < 1e-6) return p1; // Degenerate case

    const v = (d11 * d20 - d01 * d21) / denom;
    const w = (d00 * d21 - d01 * d20) / denom;
    const u = 1.0 - v - w;

    return p1.mul(u).add(p2.mul(v)).add(p3.mul(w));
  }
}
