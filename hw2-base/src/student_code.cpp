#include "student_code.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

/******************************************************************************
 **********************   PART 1:  DE CASTELJAU (2D)   ************************
 ******************************************************************************/

std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
  // TODO Part 1.
  std::vector<Vector2D> addedPoints;
  for (int i = 0; i < (int)points.size() - 1; i++) {
    Vector2D point = (1.0 - t) * points[i] + t * points[i + 1];
    addedPoints.push_back(point);
  }
  return addedPoints;
}

/******************************************************************************
 **********************   PART 2:  DE CASTELJAU (3D)   ************************
 ******************************************************************************/

std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const {
  // TODO Part 2.
  std::vector<Vector3D> addedPoints;
  for (int i = 0; i < (int)points.size() - 1; i++) {
    Vector3D point = (1.0 - t) * points[i] + t * points[i + 1];
    addedPoints.push_back(point);
  }
  return addedPoints;
}

Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const {
  // TODO Part 2.
  std::vector<Vector3D> current = points;
  while (current.size() > 1) {
    current = evaluateStep(current, t);
  }
  return current[0];
}

Vector3D BezierPatch::evaluate(double u, double v) const {
  // TODO Part 2.
  // 1) Evaluate in the u direction for each row of control points
  std::vector<Vector3D> curvePointsU;
  for (int i = 0; i < (int)controlPoints.size(); i++) {
    curvePointsU.push_back(evaluate1D(controlPoints[i], u));
  }

  // 2) Now evaluate in the v direction across those results
  Vector3D finalValue = evaluate1D(curvePointsU, v);
  return finalValue;
}

/******************************************************************************
 **********************   PART 3:  NORMAL AT A VERTEX   ************************
 ******************************************************************************/

Vector3D Vertex::normal(void) const {
  // TODO Part 3.
  // Returns an approximate unit normal at this vertex, computed by
  // taking the area-weighted average of the normals of neighboring
  // triangles, then normalizing.

  Vector3D N(0.0, 0.0, 0.0);

  // We'll assume the mesh is manifold and h is valid
  HalfedgeCIter h = halfedge();
  if (h->isBoundary()) {
    // If on boundary, just return zero or do something else safe
    return N;
  }

  // Walk around this vertex
  HalfedgeCIter start = h;
  do {
    // Triangular face
    Vector3D p0 = h->vertex()->position;
    Vector3D p1 = h->next()->vertex()->position;
    Vector3D p2 = h->next()->next()->vertex()->position;

    // Face normal weighted by area
    Vector3D faceNormal = cross((p1 - p0), (p2 - p0));
    N += faceNormal;

    h = h->twin()->next();
  } while (h != start);

  return N.unit();
}

/******************************************************************************
 **********************   PART 4:  EDGE FLIP (LOCAL)   ************************
 ******************************************************************************/

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // Check for boundary --- never flip a boundary edge
  HalfedgeIter h0 = e0->halfedge();
  HalfedgeIter h1 = h0->twin();
  FaceIter f0 = h0->face();
  FaceIter f1 = h1->face();
  if (f0->isBoundary() || f1->isBoundary()) {
    return e0;
  }

  // Tri f0: h0, h2, h3
  // Tri f1: h1, h4, h5
  HalfedgeIter h2 = h0->next();
  HalfedgeIter h3 = h2->next();
  HalfedgeIter h4 = h1->next();
  HalfedgeIter h5 = h4->next();

  // The four vertices
  VertexIter a = h3->vertex();
  VertexIter b = h0->vertex();
  VertexIter c = h2->vertex();
  VertexIter d = h5->vertex();

  // Replace diagonal (b->c) with (a->d)
  // f0 => (a,d,c)
  h0->next() = h5; 
  h5->next() = h2;
  h2->next() = h0;
  // f1 => (a,b,d)
  h3->next() = h4;
  h4->next() = h1;
  h1->next() = h3;

  // Reassign halfedge->vertex()
  h0->vertex() = a; // a->d
  h1->vertex() = d; // d->a
  h2->vertex() = c;
  h3->vertex() = a;
  h4->vertex() = b;
  h5->vertex() = d;

  // Reassign face pointers
  h0->face() = f0;
  h2->face() = f0;
  h5->face() = f0;
  f0->halfedge() = h0;

  h1->face() = f1;
  h3->face() = f1;
  h4->face() = f1;
  f1->halfedge() = h3; // or h1

  // Optionally fix vertex->halfedge()
  if (b->halfedge() == h0 || b->halfedge() == h1) {
    b->halfedge() = h4;
  }
  if (c->halfedge() == h0 || c->halfedge() == h1) {
    c->halfedge() = h2;
  }
  if (a->halfedge() == h2 || a->halfedge() == h4 || a->halfedge() == h5) {
    a->halfedge() = h0;
  }
  if (d->halfedge() == h2 || d->halfedge() == h3) {
    d->halfedge() = h1;
  }

  return e0;
}

/******************************************************************************
 **********************   PART 5 (+ EX. CREDIT):  EDGE SPLIT (LOCAL)   ********
 ******************************************************************************/

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {

  // Gather the two halfedges and their faces
  HalfedgeIter h0 = e0->halfedge();
  HalfedgeIter h1 = h0->twin();
  FaceIter f0 = h0->face();
  FaceIter f1 = h1->face();

  bool f0_boundary = f0->isBoundary();
  bool f1_boundary = f1->isBoundary();

  // If BOTH faces are boundary (which shouldn't happen in a manifold 
  // unless it's truly a boundary "dangling" edge), just ignore:
  if (f0_boundary && f1_boundary) {
    return VertexIter();
  }

  // If exactly one face is boundary, do the "one-face" split logic
  if (f0_boundary != f1_boundary) {

    // We'll call the boundary face fB and the triangle face fT
    // We want to do basically the same local insertion of a midpoint
    // but only subdivide the single triangle.
    FaceIter fB, fT;
    HalfedgeIter hB, hT;
    if (f0_boundary) {
      fB = f0; fT = f1; hB = h0; hT = h1;
    } else {
      fB = f1; fT = f0; hB = h1; hT = h0;
    }

    // Ensure the triangle face fT is indeed a triangle
    HalfedgeIter hTnext = hT->next();
    HalfedgeIter hTnext2 = hTnext->next();
    if (hTnext2->next() != hT) {
      // not a triangle, we won't handle it
      return VertexIter();
    }

    // The endpoints of the boundary edge are hB->vertex() and hB->twin()->vertex()
    VertexIter b = hB->vertex();
    VertexIter c = hB->twin()->vertex();

    // Opposite vertex in the triangle face
    HalfedgeIter hOpp = hT->next()->next(); // the 'third' corner halfedge
    VertexIter d = hOpp->vertex();

    // 1) Create new vertex m at midpoint(b->pos, c->pos)
    VertexIter m = newVertex();
    m->position = 0.5 * (b->position + c->position);

    // 2) Reuse e0 to represent (b <-> m)
    h0->vertex() = b;  
    h0->edge() = e0;
    h1->vertex() = m;  
    h1->edge() = e0;
    e0->halfedge() = h0; 
    h0->twin() = h1;
    h1->twin() = h0;

    // 3) Create new edge e_mc = (m <-> c) plus new halfedges
    EdgeIter e_mc = newEdge();
    HalfedgeIter h_mc = newHalfedge();
    HalfedgeIter h_cm = newHalfedge();
    e_mc->halfedge() = h_mc;

    // link them
    h_mc->vertex() = m; 
    h_mc->edge()   = e_mc;
    h_mc->twin()   = h_cm;
    h_cm->vertex() = c;
    h_cm->edge()   = e_mc;
    h_cm->twin()   = h_mc;

    // 4) We subdivide the *triangle* face fT into two triangles:
    // original tri was (b,c,d).  We want (b,m,d) and (m,c,d).

    // Let hT point to halfedge (b->c).  We must rewire so that
    // fT uses edges (b->m->d->b).  Meanwhile, the new face f2 uses (m->c->d->m).

    FaceIter f2 = newFace();

    HalfedgeIter hTnextNext = hT->next()->next(); // c->d 
    // We'll rename them for clarity:
    HalfedgeIter h_cd = hT->next();     // b->c => next => c->d
    HalfedgeIter h_db = hTnextNext;     // c->d => next => d->b

    // create e_md for (m->d)
    EdgeIter e_md = newEdge();
    HalfedgeIter h_md = newHalfedge();
    HalfedgeIter h_dm = newHalfedge();
    e_md->halfedge() = h_md;
    h_md->vertex() = m; 
    h_md->edge()   = e_md;
    h_md->twin()   = h_dm;
    h_dm->vertex() = d;
    h_dm->edge()   = e_md;
    h_dm->twin()   = h_md;

    // Wire up face fT => (b->m->d->b):
    hT->next()   = h_md;  
    h_md->next() = h_db;  
    h_db->next() = hT;    
    hT->face()   = fT;
    h_md->face() = fT;
    h_db->face() = fT;
    fT->halfedge() = hT;

    // The new face f2 => (m->c->d->m):
    h_mc->face() = f2; 
    h_cd->face() = f2; 
    h_dm->face() = f2; 
    f2->halfedge() = h_mc;

    // next pointers for f2:
    h_mc->next() = h_cd; // m->c -> c->d
    h_cd->next() = h_dm; // c->d -> d->m
    h_dm->next() = h_mc; // d->m -> m->c

    // 5) Update the boundary face fB by splitting its single edge (b->c) 
    // into (b->m) and (m->c). We'll do so by rewiring the boundary loop.

    HalfedgeIter hBnext = hB->next(); 
    // Suppose hB was (b->c). We'll keep hB as (b->m):
    hB->vertex() = b;
    hB->next() = h_mc; 
    hB->face() = fB; 
    fB->halfedge() = hB;

    // Then h_mc->next() leads to hBnext, completing the boundary loop.
    h_mc->next() = hBnext;

    // fix vertex->halfedge() pointers
    b->halfedge() = h0;   // b->m
    if (c->halfedge()->vertex() != c) {
      c->halfedge() = h_cm; 
    }
    if (d->halfedge()->vertex() != d) {
      d->halfedge() = h_db; 
    }
    m->halfedge() = h1;   // m->b

    // Return the new midpoint vertex
    return m;
  }

  // --------------------------
  // Otherwise, we have an *interior* edge (both faces are interior).
  // Your original code for splitting an interior edge remains unchanged:
  // --------------------------

  // Verify each face is a triangle
  if ( f0_boundary || f1_boundary ) {
    // shouldn't happen here, but just in case
    return VertexIter();
  }

  HalfedgeIter h2 = h0->next();
  HalfedgeIter h3 = h2->next();
  if (h3->next() != h0) {
    // not a triangle
    return VertexIter();
  }
  HalfedgeIter h4 = h1->next();
  HalfedgeIter h5 = h4->next();
  if (h5->next() != h1) {
    // not a triangle
    return VertexIter();
  }

  // The four distinct vertices: (b->c) plus (a) and (d)
  VertexIter b = h0->vertex(); // b
  VertexIter c = h1->vertex(); // c
  VertexIter a = h3->vertex(); // a
  VertexIter d = h5->vertex(); // d

  // 1. Create new vertex m at midpoint of (b, c)
  VertexIter m = newVertex();
  m->position = (b->position + c->position) * 0.5;

  // 2. Reuse e0 for (b<->m). So h0 -> b->m, h1 -> m->b
  h0->vertex() = b;
  h0->edge()   = e0;
  h1->vertex() = m;
  h1->edge()   = e0;
  e0->halfedge() = h0;
  h0->twin() = h1;
  h1->twin() = h0;

  // 3. Create three new edges for (m->c), (m->a), (m->d)
  EdgeIter e_mc = newEdge();
  EdgeIter e_ma = newEdge();
  EdgeIter e_md = newEdge();
  e_ma->isNew = true;
  e_md->isNew = true;

  // And six new halfedges
  HalfedgeIter h_mc = newHalfedge(); // (m->c)
  HalfedgeIter h_cm = newHalfedge(); // (c->m)
  HalfedgeIter h_ma = newHalfedge(); // (m->a)
  HalfedgeIter h_am = newHalfedge(); // (a->m)
  HalfedgeIter h_md = newHalfedge(); // (m->d)
  HalfedgeIter h_dm = newHalfedge(); // (d->m)

  // Setup edge pointers
  e_mc->halfedge() = h_mc;
  e_ma->halfedge() = h_ma;
  e_md->halfedge() = h_md;

  // Link halfedges for e_mc
  h_mc->vertex() = m;
  h_mc->edge()   = e_mc;
  h_mc->twin()   = h_cm;

  h_cm->vertex() = c;
  h_cm->edge()   = e_mc;
  h_cm->twin()   = h_mc;

  // Link halfedges for e_ma
  h_ma->vertex() = m;
  h_ma->edge()   = e_ma;
  h_ma->twin()   = h_am;

  h_am->vertex() = a;
  h_am->edge()   = e_ma;
  h_am->twin()   = h_ma;

  // Link halfedges for e_md
  h_md->vertex() = m;
  h_md->edge()   = e_md;
  h_md->twin()   = h_dm;

  h_dm->vertex() = d;
  h_dm->edge()   = e_md;
  h_dm->twin()   = h_md;

  // 4. Create two new faces (f2, f3)
  FaceIter f2 = newFace();
  FaceIter f3 = newFace();

  // We'll form 4 triangles:
  //   f0 => (a, b, m)
  //   f1 => (m, b, d)
  //   f2 => (a, m, c)
  //   f3 => (c, m, d)

  // Face f0 => (a->b->m->a) uses: h3(a->b), h0(b->m), h_ma(m->a)
  h3->next()   = h0;   // a->b -> b->m
  h0->next()   = h_ma; // b->m -> m->a
  h_ma->next() = h3;   // m->a -> a->b

  h3->face()   = f0;
  h0->face()   = f0;
  h_ma->face() = f0;
  f0->halfedge() = h3;

  // Face f1 => (m->b->d->m) uses: h1(m->b), h4(b->d), h_dm(d->m)
  h1->next()   = h4;   // m->b -> b->d
  h4->next()   = h_dm; // b->d -> d->m
  h_dm->next() = h1;   // d->m -> m->b

  h1->face()   = f1;
  h4->face()   = f1;
  h_dm->face() = f1;
  f1->halfedge() = h1;

  // Face f2 => (a->m->c->a) uses: h_am(a->m), h_mc(m->c), h2(c->a)
  h_am->next() = h_mc; // a->m -> m->c
  h_mc->next() = h2;   // m->c -> c->a
  h2->next()   = h_am; // c->a -> a->m

  h_am->face() = f2;
  h_mc->face() = f2;
  h2->face()   = f2;
  f2->halfedge() = h_am;

  // Face f3 => (c->m->d->c) uses: h_cm(c->m), h_md(m->d), h5(d->c)
  h_cm->next() = h_md; // c->m -> m->d
  h_md->next() = h5;   // m->d -> d->c
  h5->next()   = h_cm; // d->c -> c->m

  h_cm->face() = f3;
  h_md->face() = f3;
  h5->face()   = f3;
  f3->halfedge() = h_cm;

  // 5. Update vertex->halfedge() if needed
  if (b->halfedge()->vertex() != b) {
    b->halfedge() = h0;  // b->m
  }
  if (c->halfedge()->vertex() != c) {
    c->halfedge() = h2;  // c->a or c->m
  }
  if (a->halfedge()->vertex() != a) {
    a->halfedge() = h3;  // a->b
  }
  if (d->halfedge()->vertex() != d) {
    d->halfedge() = h4;  // b->d or d->m
  }
  m->halfedge() = h1;    // m->b

  // done
  return m;
}

/******************************************************************************
 **********************   PART 6:  UPSAMPLING (LOOP)   ************************
 ******************************************************************************/

void MeshResampler::upsample(HalfedgeMesh& mesh) {

  //
  // STEP A: compute new positions (but do *not* update old positions yet)
  //

  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->isNew = false;

    // Gather neighbors
    HalfedgeCIter h = v->halfedge();
    Vector3D neighborSum(0.,0.,0.);
    int n = 0;
    do {
      neighborSum += h->twin()->vertex()->position;
      n++;
      h = h->twin()->next();
    } while (h != v->halfedge());

    // Compute u depending on the valence
    double u = (n == 3) ? 3.0/16.0 : 3.0/(8.0*n);

    // Store the new position in v->newPosition
    v->newPosition = (1.0 - n*u)*v->position + u*neighborSum;
  }

  //
  // For edges, compute the midpoint newPosition via
  //    3/8*(A + B) + 1/8*(C + D)
  //
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    e->isNew = false;
    HalfedgeIter h0 = e->halfedge();
    HalfedgeIter h1 = h0->twin();
    VertexIter A = h0->vertex();
    VertexIter B = h1->vertex();
    VertexIter C = h0->next()->next()->vertex();
    VertexIter D = h1->next()->next()->vertex();
    e->newPosition = 3.0/8.0*(A->position + B->position)
                     + 1.0/8.0*(C->position + D->position);
  }

  //
  // STEP B: split every “original” edge
  //
  vector<EdgeIter> originalEdges;
  originalEdges.reserve(mesh.nEdges());
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    originalEdges.push_back(e);
  }

  for (EdgeIter e : originalEdges) {
    VertexIter m = mesh.splitEdge(e);
    // The new vertex might be invalid if boundary-splitting wasn't handled
    // (or the edge is a boundary in some partial or error case).
    if (m == VertexIter()) continue;

    // Mark this new vertex and set its new position
    m->isNew = true;
    m->newPosition = e->newPosition;

    // Optionally mark edges from m to old vertices as "isNew = true" ...
    // ...
  }

  //
  // STEP C: flip each new edge that connects an old vertex and a new vertex
  //
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    if (!e->isNew) continue; // only consider edges newly created by the split

    VertexIter v1 = e->halfedge()->vertex();
    VertexIter v2 = e->halfedge()->twin()->vertex();

    // Flip if exactly one of v1 or v2 is new
    if (v1->isNew != v2->isNew) {
      // also skip boundary if not implementing boundary flips
      if (!e->halfedge()->face()->isBoundary() &&
          !e->halfedge()->twin()->face()->isBoundary()) {
        mesh.flipEdge(e);
      }
    }
  }

  //
  // STEP D: copy each vertex’s newPosition back to its actual position
  //
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->position = v->newPosition;
  }
}

} // namespace CGL
