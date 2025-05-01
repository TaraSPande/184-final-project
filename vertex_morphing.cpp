#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    Vec3 cross(const Vec3& other) const {
        return Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    Vec3 normalize() const {
        float mag = magnitude();
        return Vec3(x / mag, y / mag, z / mag);
    }
};

enum InterpolationType {
    Linear,
    QuadraticBezier,
    CubicBezier
};

Vec3 interpolateLinear(const Vec3& a, const Vec3& b, float t) {
    return a * (1 - t) + b * t;
}

Vec3 interpolateQuadraticBezier(const Vec3& a, const Vec3& b, const Vec3& control, float t) {
    Vec3 ab = interpolateLinear(a, control, t);
    Vec3 bc = interpolateLinear(control, b, t);
    return interpolateLinear(ab, bc, t);
}

Vec3 interpolateCubicBezier(const Vec3& a, const Vec3& b, const Vec3& c1, const Vec3& c2, float t) {
    Vec3 ab = interpolateLinear(a, c1, t);
    Vec3 bc = interpolateLinear(c1, c2, t);
    Vec3 cd = interpolateLinear(c2, b, t);
    Vec3 abc = interpolateLinear(ab, bc, t);
    Vec3 bcd = interpolateLinear(bc, cd, t);
    return interpolateLinear(abc, bcd, t);
}

std::vector<Vec3> morphMeshes(
    const std::vector<Vec3>& meshA,
    const std::vector<Vec3>& meshB,
    const std::vector<int>& correspondence,
    float t,
    InterpolationType method
) {
    std::vector<Vec3> result;
    for (size_t i = 0; i < meshA.size(); ++i) {
        Vec3 a = meshA[i];
        Vec3 b = meshB[correspondence[i]];
        Vec3 morphed;
        switch (method) {
            case Linear:
                morphed = interpolateLinear(a, b, t);
                break;
            case QuadraticBezier: {
                Vec3 control = interpolateLinear(a, b, 0.5f);
                morphed = interpolateQuadraticBezier(a, b, control, t);
                break;
            }
            case CubicBezier: {
                Vec3 c1 = interpolateLinear(a, b, 0.33f);
                Vec3 c2 = interpolateLinear(a, b, 0.66f);
                morphed = interpolateCubicBezier(a, b, c1, c2, t);
                break;
            }
        }
        result.push_back(morphed);
    }
    return result;
}

std::vector<Vec3> uniformSample(const std::vector<Vec3>& vertices, int sampleCount) {
    std::vector<Vec3> sampled;
    int step = vertices.size() / sampleCount;
    for (int i = 0; i < sampleCount; ++i) {
        sampled.push_back(vertices[i * step]);
    }
    return sampled;
}

class MeshDeformation {
    public:
        static void linearInterpolateMeshes(const std::vector<Vec3>& start, const std::vector<Vec3>& end, float t, std::vector<Vec3>& result) {
            // renamed from bezierInterpolate
            result.clear();
            for (size_t i = 0; i < start.size(); ++i) {
                result.push_back(start[i] * (1 - t) + end[i] * t);
            }
        }
    
        static void laplacianSmoothing(std::vector<Vec3>& vertices, const std::vector<std::vector<int> >& adjacencyList, int iterations) {
            std::vector<Vec3> smoothed(vertices.size());
            for (int iter = 0; iter < iterations; ++iter) {
                for (size_t i = 0; i < vertices.size(); ++i) {
                    Vec3 sum(0, 0, 0);
                    int count = 0;
                    for (int neighbor : adjacencyList[i]) {
                        sum = sum + vertices[neighbor];
                        ++count;
                    }
                    if (count > 0)
                        smoothed[i] = sum * (1.0f / count);
                    else
                        smoothed[i] = vertices[i];
                }
                vertices = smoothed;
            }
        }
    };

// Calculate centroid for set of points
Vec3 calculateCentroid(const std::vector<Vec3>& points) {
    Vec3 centroid(0.0f, 0.0f, 0.0f);
    for (const auto& p : points) {
        centroid = centroid + p;
    }
    return centroid * (1.0f / points.size());
}

// Find the closest point in target mesh for every point in source mesh
std::vector<int> findClosestPoints(const std::vector<Vec3>& source, const std::vector<Vec3>& target) {
    std::vector<int> closestPoints(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
        float minDist = std::numeric_limits<float>::max();
        int closestIdx = -1;
        for (size_t j = 0; j < target.size(); ++j) {
            float dist = (source[i] - target[j]).magnitude();
            if (dist < minDist) {
                minDist = dist;
                closestIdx = j;
            }
        }
        closestPoints[i] = closestIdx;
    }
    return closestPoints;
}

// Apply translation and rotation to source mesh
void applyTransformation(std::vector<Vec3>& source, const Vec3& translation, const std::vector<std::vector<float> >& rotationMatrix) {
    for (auto& vertex : source) {
        // Apply rotation (using a simple 3x3 matrix multiplication)
        float xNew = rotationMatrix[0][0] * vertex.x + rotationMatrix[0][1] * vertex.y + rotationMatrix[0][2] * vertex.z;
        float yNew = rotationMatrix[1][0] * vertex.x + rotationMatrix[1][1] * vertex.y + rotationMatrix[1][2] * vertex.z;
        float zNew = rotationMatrix[2][0] * vertex.x + rotationMatrix[2][1] * vertex.y + rotationMatrix[2][2] * vertex.z;

        // Apply translation
        vertex = Vec3(xNew + translation.x, yNew + translation.y, zNew + translation.z);
    }
}

// Iterative Closest Point 
void ICP(std::vector<Vec3>& source, const std::vector<Vec3>& target, int maxIterations = 10) {
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Find closest points
        std::vector<int> closestPoints = findClosestPoints(source, target);

        // Compute centroids
        Vec3 sourceCentroid = calculateCentroid(source);
        std::vector<Vec3> targetMatchedPoints;
        for (int i : closestPoints) {
            targetMatchedPoints.push_back(target[i]);
        }
        Vec3 targetCentroid = calculateCentroid(targetMatchedPoints);

        // Cross-covariance matrix (optimal rotation)
        std::vector<std::vector<float> > rotationMatrix(3, std::vector<float>(3, 0));
        Vec3 translation = targetCentroid - sourceCentroid;

        // Apply the transformation (translation + rotation)
        applyTransformation(source, translation, rotationMatrix);
    }
}

bool compareAnglePairs(const std::pair<float, Vec3>& a, const std::pair<float, Vec3>& b) {
    return a.first < b.first;
}

// Vertice by angle in x and y plane
std::vector<Vec3> orderVerticesByAngle(const std::vector<Vec3>& vertices) {
    Vec3 centroid = calculateCentroid(vertices);
    std::vector<std::pair<float, Vec3> > anglesAndVertices;

    for (const auto& v : vertices) {
        // Calculate angle in X-Y plane
        float angle = std::atan2(v.y - centroid.y, v.x - centroid.x);
        anglesAndVertices.push_back(std::make_pair(angle, v));
    }

    // Use named comparator function for sorting
    std::sort(anglesAndVertices.begin(), anglesAndVertices.end(), compareAnglePairs);

    std::vector<Vec3> orderedVertices;
    for (const auto& pair : anglesAndVertices) {
        orderedVertices.push_back(pair.second);
    }
    return orderedVertices;
}


// Ordered sampling
std::vector<Vec3> orderedSample(const std::vector<Vec3>& vertices, int sampleCount) {
    std::vector<Vec3> ordered = orderVerticesByAngle(vertices);
    std::vector<Vec3> sampled;
    int step = ordered.size() / sampleCount;
    for (int i = 0; i < sampleCount; ++i) {
        sampled.push_back(ordered[i * step]);
    }
    return sampled;
}