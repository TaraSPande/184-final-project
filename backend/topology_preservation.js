export function smoothMeshPositions(mesh, smoothingWeight) {
  // First pass: compute new positions
  for (const vertex of mesh.vertices) {
    if (vertex.isBoundary()) {
      vertex.newPosition = vertex.position;
      continue;
    }

    let neighborSum = new Vector3D(0, 0, 0);
    let count = 0;

    let h = vertex.halfedge;
    let start = h;
    do {
      neighborSum = neighborSum.add(h.twin.vertex.position);
      count++;
      h = h.twin.next;
    } while (h !== start);

    const centroid = neighborSum.multiply(1.0 / count);
    const displacement = centroid.subtract(vertex.position);

    vertex.newPosition = vertex.position.add(displacement.multiply(smoothingWeight));
  }

  // Second pass: apply the smoothed positions
  for (const vertex of mesh.vertices) {
    vertex.position = vertex.newPosition;
  }
}