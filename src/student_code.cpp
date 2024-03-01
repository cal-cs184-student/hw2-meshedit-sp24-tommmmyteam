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

    // Reassign next pointers to reroute the half-edge loop
    h0->next() = h5;
    h5->next() = h2;
    h2->next() = h0;

    h1->next() = h3;
    h3->next() = h4;
    h4->next() = h1;

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

    // FaceIter f0 = e0->halfedge()->face();
    // FaceIter f1 = e0->halfedge()->twin()->face();

    // // Setting correct vertices halfedge pointers
    // // "a" and "d" does not change, since we are only adding potential half-edges, not removing
    // e0->halfedge()->next()->vertex()->halfedge() = e0->halfedge()->next(); // for "c"
    // e0->halfedge()->vertex()->halfedge() = e0->halfedge()->twin()->next(); // For "b"

    // // Setting correct edge halfedge pointers
    // // No need?

    // // Setting correct face halfedge pointers
    // e0->halfedge()->face()->halfedge() = e0->halfedge(); // for f0
    // e0->halfedge()->twin()->face()->halfedge() = e0->halfedge()->twin(); // for f1

    // // Setting correct halfedge face pointer
    // e0->halfedge()->next()->next()->face() = f1; // for "ab"
    // e0->halfedge()->twin()->next()->next()->face() = f0; // for "cd"

    // // Setting correct halfedge next pointers
    // HalfedgeIter e0_halfedge_next_new = e0->halfedge()->twin()->next()->next();
    // VertexIter e0_vertex_new = e0->halfedge()->next()->next()->vertex();

    // HalfedgeIter e0_halfedge_twin_next_new = e0->halfedge()->next()->next();
    // VertexIter e0_vertex_twin_new = e0->halfedge()->twin()->next()->next()->vertex();
    
    // e0->halfedge()->next()->next() = e0->halfedge();  // for "ca"
    // e0->halfedge()->next()->next()->next() = e0->halfedge()->twin()->next(); // for "ab"
    // e0->halfedge()->twin()->next()->next() = e0->halfedge()->twin(); // for "bd"
    // e0->halfedge()->twin()->next()->next()->next() = e0->halfedge()->next(); // for "dc"

    // // Setting current halfedge / twin parameters
    // e0->halfedge()->next() = e0_halfedge_next_new;
    // e0->halfedge()->twin()->next() = e0_halfedge_twin_next_new;

    // e0->halfedge()->vertex() = e0_vertex_new;
    // e0->halfedge()->twin()->vertex() = e0_vertex_twin_new;

    // // This method should flip the given edge and return an iterator to the flipped edge.
    // return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return VertexIter();
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

  }
}
