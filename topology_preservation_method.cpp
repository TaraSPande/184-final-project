void smoothMeshPositions(HalfedgeMesh& mesh, double smoothingWeight) {
  // Store new positions but don’t apply them until the end
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    // If boundary, you might skip or handle differently:
    if (v->isBoundary()) {
      v->newPosition = v->position;
      continue;
    }

    // Sum neighbor positions
    Vector3D neighborSum(0.,0.,0.);
    int n = 0;
    HalfedgeCIter h = v->halfedge();
    do {
      neighborSum += h->twin()->vertex()->position;
      n++;
      h = h->twin()->next();
    } while (h != v->halfedge());

    // Basic Laplacian update
    Vector3D centroid = neighborSum / (double)n;
    Vector3D displacement = centroid - v->position;

    // Weighted step toward neighbor centroid
    v->newPosition = v->position + smoothingWeight * displacement;
  }

  // Now apply
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->position = v->newPosition;
  }
}

