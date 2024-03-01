#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> intermediates = std::vector<Vector2D>();
    for (int i = 0; i < points.size() - 1; i++) {
      intermediates.push_back((1 - t) * points[i] + t * points[i + 1]);
    }

    return intermediates;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> intermediates = std::vector<Vector3D>();
    for (int i = 0; i < points.size() - 1; i++) {
      intermediates.push_back((1 - t) * points[i] + t * points[i + 1]);
    }

    return intermediates;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> result = points;
    do {
      result = evaluateStep(result, t);
    } while (result.size() > 1);

    return result[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    std::vector<Vector3D> u_intermediates;
    for (int i = 0; i < controlPoints.size(); i++) {
      u_intermediates.push_back(evaluate1D(controlPoints[i], u));
    }

    return evaluate1D(u_intermediates, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    HalfedgeCIter h = this->halfedge();

    Vector3D unit_normal = Vector3D();

    do {
      // h_twin is a halfedge of the current triangle
      HalfedgeCIter h_twin = h->twin();

      // v0, v1, v2 are the vertices of the current triangle
      VertexCIter v0 = h_twin->vertex();
      VertexCIter v1 = h_twin->next()->vertex();
      VertexCIter v2 = h_twin->next()->next()->vertex();

      // n is the normal of the current triangle
      Vector3D n = cross(v1->position - v0->position, v2->position - v1->position);

      // Adding the normal of the current triangle to the sum
      unit_normal += n;

      // Moving to the next halfedge of the current triangle
      h = h_twin->next();
    } while (h != this->halfedge());

    return unit_normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // Checking if boundary
    if (e0->isBoundary()) {
      return e0;
    }

    // Faces
    FaceIter f0 = e0->halfedge()->face();
    FaceIter f1 = e0->halfedge()->twin()->face();

    // Half Edges
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();

    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
 
    // Vertices of the triangles
    VertexIter a = h0->next()->next()->vertex();
    VertexIter b = h0->vertex();
    VertexIter c = h3->vertex();
    VertexIter d = h3->next()->next()->vertex(); 

    // Update half-edge on vertex pointers
    a->halfedge() = h2;
    b->halfedge() = h4;
    c->halfedge() = h1;
    d->halfedge() = h5;

    // Update half-edge on face pointers
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    // Update half-edge on edge pointers (not necessary)

    /////////////////////////////////////////////////////////////

    // Update vertex pointers on half-edges
    h0->vertex() = a;
    h3->vertex() = d;

    // Update face pointers on half-edges
    h5->face() = f0;
    h2->face() = f1;
    
    // Update edge pointers on half-edges (not necessary)

    // Update twin pointers on half-edges (not necessary)

    // Update next pointers on half-edges
    h0->next() = h5;
    h5->next() = h1;
    h1->next() = h0;

    h3->next() = h2;
    h2->next() = h4;
    h4->next() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    
    if (e0->halfedge()->face()->isBoundary() || e0->halfedge()->twin()->face()->isBoundary()) {
        return VertexIter(); // Early return if edge is a boundary edge
    }

    // Faces
    FaceIter f0 = e0->halfedge()->face();
    FaceIter f1 = e0->halfedge()->twin()->face();

    // Half Edges
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();

    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
 
    // Vertices of the triangles
    VertexIter a = h0->next()->next()->vertex();
    VertexIter b = h0->vertex();
    VertexIter c = h3->vertex();
    VertexIter d = h3->next()->next()->vertex(); 

    // Creating new mesh elements
    VertexIter m = newVertex();
    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();
    HalfedgeIter h6 = newHalfedge();
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();


    // Setting the new vertex position to the midpoint
    m->position = (b->position + c->position) / 2.0;

    // Update half-edge on vertex pointers
    m->halfedge() = h0;
    c->halfedge() = h1;
    a->halfedge() = h2;
    b->halfedge() = h4;
    d->halfedge() = h5;

    // Update half-edge on face pointers
    f0->halfedge() = h1;
    f1->halfedge() = h5;
    f2->halfedge() = h2;
    f3->halfedge() = h4;

    // Update half-edge on edge pointers
    e0->halfedge() = h0;
    e1->halfedge() = h6;
    e2->halfedge() = h8;
    e3->halfedge() = h10;

    e0->isNew = false;
    e1->isNew = true;
    e2->isNew = false;
    e3->isNew = true;

    // Updated half edges
    // next, twin, vertex, edge, face
    h0->setNeighbors(h1, h3, m, e0, f0);
    h1->setNeighbors(h6, h1->twin(), c, h1->edge(), f0);
    h6->setNeighbors(h0, h7, a, e1, f0);

    h3->setNeighbors(h11, h0, c, e0, f1);
    h11->setNeighbors(h5, h10, m, e3, f1); 
    h5->setNeighbors(h3, h5->twin(), d, h5->edge(), f1);

    h8->setNeighbors(h7, h9, b, e2, f2);
    h7->setNeighbors(h2, h6, m, e1, f2);
    h2->setNeighbors(h8, h2->twin(), a, h2->edge(), f2);

    h9->setNeighbors(h4, h8, m, e2, f3);
    h4->setNeighbors(h10, h4->twin(), b, h4->edge(), f3);
    h10->setNeighbors(h9, h11, d, e3, f3);

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;

      Size n = v->degree();
      double u = (n == 3) ? 3.0 / 16.0 : 3.0 / (8.0 * n);
      
      Vector3D neighbor_sum = Vector3D();
      HalfedgeIter h = v->halfedge();
      do {
        neighbor_sum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      v->newPosition = (1 - n * u) * v->position + u * neighbor_sum;
    }

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;

      HalfedgeIter h0 = e->halfedge();
      HalfedgeIter h1 = h0->twin();

      VertexIter a = h0->vertex();
      VertexIter b = h1->vertex();
      VertexIter c = h0->next()->next()->vertex();
      VertexIter d = h1->next()->next()->vertex();

      e->newPosition = 0.375 * (a->position + b->position) + 0.125 * (c->position + d->position);
    }

    std::vector<EdgeIter> originalEdges;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
        originalEdges.push_back(e);
        e->isNew = false; // Mark original edges as not new
    }


    for (EdgeIter e : originalEdges) {
        VertexIter newVertex = mesh.splitEdge(e);
        newVertex->isNew = true; // Mark the new vertex as new
        // Set the new vertex position immediately after splitting
        newVertex->position = e->newPosition;
    }


    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew) {
        VertexIter a = e->halfedge()->vertex();
        VertexIter b = e->halfedge()->twin()->vertex();

        if ((a->isNew && !b->isNew) || (!a->isNew && b->isNew)) {
          mesh.flipEdge(e);
        }
      }
    }

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      if (!v->isNew) {
        v->position = v->newPosition;
      }
    }
  }
}
