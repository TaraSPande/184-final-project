#include "CGL/vector3D.h"
#include <vector>

using namespace CGL;

class VertexInterpolation {
public:
    // Linear interpolation between two vertex positions
    static Vector3D linearInterpolate(const Vector3D& v1, const Vector3D& v2, double t) {
        return v1 * (1.0 - t) + v2 * t;
    }

    // Linear interpolation between two meshes (represented as vertex vectors)
    static std::vector<Vector3D> linearInterpolateMeshes(
        const std::vector<Vector3D>& mesh1, 
        const std::vector<Vector3D>& mesh2, 
        double t) {
        
        if (mesh1.size() != mesh2.size()) {
            // Meshes must have same number of vertices for simple linear interpolation
            return std::vector<Vector3D>();
        }
        
        std::vector<Vector3D> result(mesh1.size());
        for (size_t i = 0; i < mesh1.size(); i++) {
            result[i] = linearInterpolate(mesh1[i], mesh2[i], t);
        }
        return result;
    }

    // Cubic Bézier curve interpolation for a single vertex
    static Vector3D bezierInterpolate(
        const Vector3D& p0,   // Start point
        const Vector3D& p1,   // Control point 1
        const Vector3D& p2,   // Control point 2 
        const Vector3D& p3,   // End point
        double t) {
        
        double t2 = t * t;
        double t3 = t2 * t;
        double mt = 1.0 - t;
        double mt2 = mt * mt;
        double mt3 = mt2 * mt;
        
        return p0 * mt3 + p1 * (3.0 * mt2 * t) + p2 * (3.0 * mt * t2) + p3 * t3;
    }

    // Blend shape interpolation (linear combination of multiple meshes)
    static std::vector<Vector3D> blendShapes(
        const std::vector<std::vector<Vector3D>>& meshes,
        const std::vector<double>& weights) {
        
        if (meshes.empty() || weights.size() != meshes.size()) {
            return std::vector<Vector3D>();
        }
        
        size_t vertexCount = meshes[0].size();
        std::vector<Vector3D> result(vertexCount, Vector3D(0, 0, 0));
        
        for (size_t m = 0; m < meshes.size(); m++) {
            if (meshes[m].size() != vertexCount) {
                continue; // Skip meshes with incorrect vertex count
            }
            
            for (size_t v = 0; v < vertexCount; v++) {
                result[v] += meshes[m][v] * weights[m];
            }
        }
        
        return result;
    }
    
    // Catmull-Rom spline interpolation for smooth keyframe animation
    static Vector3D catmullRomInterpolate(
        const Vector3D& p0,  // Previous keyframe
        const Vector3D& p1,  // Start keyframe
        const Vector3D& p2,  // End keyframe
        const Vector3D& p3,  // Next keyframe
        double t) {          // Value between 0 and 1
        
        double t2 = t * t;
        double t3 = t2 * t;
        
        Vector3D a = 0.5 * (2.0 * p1 + (-p0 + p2) * t + 
                           (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                           (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
        return a;
    }
    
    // Multi-mesh keyframe interpolation using Catmull-Rom
    static std::vector<Vector3D> interpolateKeyframes(
        const std::vector<std::vector<Vector3D>>& keyframes,
        int current,  // Current keyframe index
        double t) {   // Value between 0 and 1
        
        if (keyframes.size() < 4) {
            return std::vector<Vector3D>();
        }
        
        int prev = (current > 0) ? current - 1 : 0;
        int next = (current < static_cast<int>(keyframes.size()) - 2) ? current + 2 : current + 1;
        int start = current;
        int end = current + 1;
        
        std::vector<Vector3D> result(keyframes[0].size());
        
        for (size_t v = 0; v < keyframes[0].size(); v++) {
            result[v] = catmullRomInterpolate(
                keyframes[prev][v],
                keyframes[start][v],
                keyframes[end][v],
                keyframes[next][v],
                t
            );
        }
        
        return result;
    }

    static Vector3D barycentricInterpolate(const Vector3D& p1, const Vector3D& p2, 
                                          const Vector3D& p3, const Vector3D& point) {
        // Project point onto the triangle plane
        Vector3D normal = cross(p2 - p1, p3 - p1).unit();
        Vector3D projected = point - dot(point - p1, normal) * normal;
        
        // Compute barycentric coordinates
        Vector3D v0 = p2 - p1;
        Vector3D v1 = p3 - p1;
        Vector3D v2 = projected - p1;
        
        double d00 = dot(v0, v0);
        double d01 = dot(v0, v1);
        double d11 = dot(v1, v1);
        double d20 = dot(v2, v0);
        double d21 = dot(v2, v1);
        
        double denom = d00 * d11 - d01 * d01;
        if (abs(denom) < 1e-6) return p1; // Degenerate case
        
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;
        
        return u * p1 + v * p2 + w * p3;
    }
};
