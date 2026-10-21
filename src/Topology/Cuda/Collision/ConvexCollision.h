/**
 * Copyright 2026 PeriDyno contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "Vector.h"
#include "Primitive/Primitive3D.h"

/**
 * @brief Generic narrow phase for any pair of convex primitives.
 *
 *	Every primitive supported by DiscreteElements (box, tet, triangle, sphere, capsule, medial cone and medial slab)
 *	is the convex hull of at most eight spheres, so a single support function covers all of them.
 *	GJK computes the distance between separated shapes, EPA computes the penetration of overlapping shapes.
 *	The result is a single contact; dedicated routines in CollisionDetectionAlgorithm are preferred whenever
 *	a pair needs a multi-point manifold (e.g. box-box, box-triangle).
 */
namespace dyno
{
	template<typename Real>
	struct TSphereHull
	{
		typedef Vector<Real, 3> Coord;

		Coord p[8];
		Real r[8];
		int n = 0;

		DYN_FUNC void add(const Coord& c, Real radius) { if (n < 8) { p[n] = c; r[n] = radius; n++; } }

		/**
		 * @brief Support point of the hull in direction d (d does not need to be normalized)
		 */
		DYN_FUNC Coord support(const Coord& d) const
		{
			Real len = d.norm();
			Coord dn = len > Real(0) ? d / len : Coord(0);

			int best = 0;
			Real bestVal = p[0].dot(dn) + r[0];
			for (int i = 1; i < n; i++)
			{
				Real val = p[i].dot(dn) + r[i];
				if (val > bestVal) { bestVal = val; best = i; }
			}
			return p[best] + dn * r[best];
		}

		DYN_FUNC Coord center() const
		{
			Coord c(0);
			for (int i = 0; i < n; i++) c += p[i];
			return n > 0 ? c / Real(n) : c;
		}
	};

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TSphere3D<Real>& s) { TSphereHull<Real> h; h.add(s.center, s.radius); return h; }

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TCapsule3D<Real>& c) { TSphereHull<Real> h; h.add(c.startPoint(), c.radius); h.add(c.endPoint(), c.radius); return h; }

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TOrientedBox3D<Real>& b)
	{
		TSphereHull<Real> h;
		for (int i = 0; i < 8; i++)
		{
			Real su = (i & 1) ? Real(1) : Real(-1);
			Real sv = (i & 2) ? Real(1) : Real(-1);
			Real sw = (i & 4) ? Real(1) : Real(-1);
			h.add(b.center + b.u * (su * b.extent[0]) + b.v * (sv * b.extent[1]) + b.w * (sw * b.extent[2]), Real(0));
		}
		return h;
	}

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TTet3D<Real>& t) { TSphereHull<Real> h; for (int i = 0; i < 4; i++) h.add(t.v[i], Real(0)); return h; }

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TTriangle3D<Real>& t) { TSphereHull<Real> h; for (int i = 0; i < 3; i++) h.add(t.v[i], Real(0)); return h; }

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TMedialCone3D<Real>& c) { TSphereHull<Real> h; h.add(c.v[0], c.radius[0]); h.add(c.v[1], c.radius[1]); return h; }

	template<typename Real>
	DYN_FUNC TSphereHull<Real> makeSphereHull(const TMedialSlab3D<Real>& s) { TSphereHull<Real> h; for (int i = 0; i < 3; i++) h.add(s.v[i], s.radius[i]); return h; }

	/**
	 * @brief Result of a convex query.
	 *	normal points from A to B, distance is the signed separation (negative when overlapping),
	 *	pointA/pointB are the witness points on the surfaces of A and B.
	 */
	template<typename Real>
	struct TConvexContact
	{
		Vector<Real, 3> normal;
		Vector<Real, 3> pointA;
		Vector<Real, 3> pointB;
		Real distance;
	};

	template<typename Real>
	class ConvexCollision
	{
	public:
		typedef Vector<Real, 3> Coord;
		typedef TSphereHull<Real> Hull;

		struct SupportPoint
		{
			Coord w;	// a - b
			Coord a;
			Coord b;
		};

		/**
		 * @brief Compute the signed distance between two convex hulls.
		 * @return false if the shapes are farther apart than maxDistance
		 */
		DYN_FUNC static bool query(TConvexContact<Real>& out, const Hull& A, const Hull& B, Real maxDistance)
		{
			// Shrink both hulls by their smallest radius. For rounded shapes the shrunk cores are usually still
			// separated when the shapes touch, and GJK on the cores is exact while EPA on curved surfaces is not.
			Real rA = A.r[0], rB = B.r[0];
			for (int i = 1; i < A.n; i++) rA = minimum(rA, A.r[i]);
			for (int i = 1; i < B.n; i++) rB = minimum(rB, B.r[i]);

			if (rA + rB > Real(0))
			{
				Hull coreA = A, coreB = B;
				for (int i = 0; i < coreA.n; i++) coreA.r[i] -= rA;
				for (int i = 0; i < coreB.n; i++) coreB.r[i] -= rB;

				TConvexContact<Real> core;
				int result = gjk(core, coreA, coreB, maxDistance + rA + rB);
				if (result == 0) return false;
				if (result == 1 && core.distance > Real(1e-4))
				{
					out.normal = core.normal;
					out.pointA = core.pointA + core.normal * rA;
					out.pointB = core.pointB - core.normal * rB;
					out.distance = core.distance - rA - rB;
					return true;
				}
			}

			int result = gjk(out, A, B, maxDistance);
			return result != 0;
		}

	private:
		/**
		 * @return 0: farther than maxDistance, 1: separated (out is filled), 2: overlapping (out filled by EPA)
		 */
		DYN_FUNC static int gjk(TConvexContact<Real>& out, const Hull& A, const Hull& B, Real maxDistance)
		{
			SupportPoint simplex[4];
			int count = 0;

			Coord d = B.center() - A.center();
			if (d.normSquared() < Real(1e-12)) d = Coord(1, 0, 0);

			simplex[0] = supportAB(A, B, -d);
			count = 1;

			Coord closest = simplex[0].w;
			Real lambda[4] = { Real(1), 0, 0, 0 };

			const int maxIter = 32;
			const Real eps = Real(1e-6);

			bool overlap = false;
			for (int iter = 0; iter < maxIter; iter++)
			{
				Real dist2 = closest.normSquared();
				if (dist2 < eps * eps) { overlap = true; break; }

				SupportPoint s = supportAB(A, B, -closest);

				// No progress towards the origin: closest is the answer
				Real progress = closest.dot(closest) - closest.dot(s.w);
				if (progress <= eps * sqrt(dist2) + Real(1e-12)) break;

				bool duplicate = false;
				for (int i = 0; i < count; i++)
					if ((simplex[i].w - s.w).normSquared() < Real(1e-12)) duplicate = true;
				if (duplicate) break;

				simplex[count++] = s;

				if (!reduceSimplex(simplex, count, lambda, closest)) { overlap = true; break; }
			}

			if (!overlap)
			{
				Coord pa(0), pb(0);
				for (int i = 0; i < count; i++) { pa += simplex[i].a * lambda[i]; pb += simplex[i].b * lambda[i]; }

				Real dist = closest.norm();
				if (dist > maxDistance) return 0;

				// closest = pa - pb, normal from A to B
				out.normal = dist > Real(0) ? -closest / dist : Coord(0, 1, 0);
				out.pointA = pa;
				out.pointB = pb;
				out.distance = dist;
				return 1;
			}

			return epa(out, A, B, simplex, count) ? 2 : 0;
		}

		DYN_FUNC static SupportPoint supportAB(const Hull& A, const Hull& B, const Coord& d)
		{
			SupportPoint s;
			s.a = A.support(d);
			s.b = B.support(-d);
			s.w = s.a - s.b;
			return s;
		}

		/**
		 * @brief Replace the simplex by the smallest sub-simplex containing the point closest to the origin.
		 * @return false if the origin is enclosed by a tetrahedron
		 */
		DYN_FUNC static bool reduceSimplex(SupportPoint* s, int& count, Real* lambda, Coord& closest)
		{
			if (count == 2) return segment(s, count, lambda, closest);
			if (count == 3) return triangle(s, count, lambda, closest);
			if (count == 4) return tetrahedron(s, count, lambda, closest);

			lambda[0] = Real(1);
			closest = s[0].w;
			return true;
		}

		DYN_FUNC static bool segment(SupportPoint* s, int& count, Real* lambda, Coord& closest)
		{
			Coord a = s[0].w, b = s[1].w;
			Coord ab = b - a;
			Real denom = ab.dot(ab);
			Real t = denom > Real(0) ? -a.dot(ab) / denom : Real(0);
			if (t <= Real(0)) { count = 1; lambda[0] = Real(1); closest = a; return true; }
			if (t >= Real(1)) { s[0] = s[1]; count = 1; lambda[0] = Real(1); closest = b; return true; }
			lambda[0] = Real(1) - t; lambda[1] = t;
			closest = a + ab * t;
			return true;
		}

		// Closest point on triangle (Ericson, Real-Time Collision Detection, 5.1.5)
		DYN_FUNC static bool triangle(SupportPoint* s, int& count, Real* lambda, Coord& closest)
		{
			Coord a = s[0].w, b = s[1].w, c = s[2].w;
			Coord ab = b - a, ac = c - a, ap = -a;

			Real d1 = ab.dot(ap), d2 = ac.dot(ap);
			if (d1 <= 0 && d2 <= 0) { count = 1; lambda[0] = 1; closest = a; return true; }

			Coord bp = -b;
			Real d3 = ab.dot(bp), d4 = ac.dot(bp);
			if (d3 >= 0 && d4 <= d3) { s[0] = s[1]; count = 1; lambda[0] = 1; closest = b; return true; }

			Real vc = d1 * d4 - d3 * d2;
			if (vc <= 0 && d1 >= 0 && d3 <= 0)
			{
				Real v = d1 / (d1 - d3);
				count = 2; lambda[0] = 1 - v; lambda[1] = v;
				closest = a + ab * v;
				return true;
			}

			Coord cp = -c;
			Real d5 = ab.dot(cp), d6 = ac.dot(cp);
			if (d6 >= 0 && d5 <= d6) { s[0] = s[2]; count = 1; lambda[0] = 1; closest = c; return true; }

			Real vb = d5 * d2 - d1 * d6;
			if (vb <= 0 && d2 >= 0 && d6 <= 0)
			{
				Real w = d2 / (d2 - d6);
				s[1] = s[2];
				count = 2; lambda[0] = 1 - w; lambda[1] = w;
				closest = a + ac * w;
				return true;
			}

			Real va = d3 * d6 - d5 * d4;
			if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
			{
				Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				s[0] = s[1]; s[1] = s[2];
				count = 2; lambda[0] = 1 - w; lambda[1] = w;
				closest = b + (c - b) * w;
				return true;
			}

			Real denom = Real(1) / (va + vb + vc);
			Real v = vb * denom, w = vc * denom;
			count = 3;
			lambda[0] = 1 - v - w; lambda[1] = v; lambda[2] = w;
			closest = a + ab * v + ac * w;
			return true;
		}

		DYN_FUNC static bool tetrahedron(SupportPoint* s, int& count, Real* lambda, Coord& closest)
		{
			// Faces opposite to each vertex, oriented so that the remaining vertex is behind
			const int faces[4][3] = { {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1} };

			Real bestDist2 = REAL_MAX;
			SupportPoint best[3];
			Real bestLambda[3] = { 0, 0, 0 };
			int bestCount = 0;
			bool inside = true;

			for (int f = 0; f < 4; f++)
			{
				SupportPoint a = s[faces[f][0]], b = s[faces[f][1]], c = s[faces[f][2]];
				SupportPoint d = s[f];

				Coord n = (b.w - a.w).cross(c.w - a.w);
				Real sOrigin = -a.w.dot(n);
				Real sOpposite = (d.w - a.w).dot(n);

				// Origin is on the same side as the opposite vertex: not outside this face
				if (sOrigin * sOpposite >= 0) continue;

				inside = false;

				SupportPoint tri[3] = { a, b, c };
				int triCount = 3;
				Real triLambda[4];
				Coord triClosest;
				triangle(tri, triCount, triLambda, triClosest);

				Real dist2 = triClosest.normSquared();
				if (dist2 < bestDist2)
				{
					bestDist2 = dist2;
					bestCount = triCount;
					for (int i = 0; i < triCount; i++) { best[i] = tri[i]; bestLambda[i] = triLambda[i]; }
					closest = triClosest;
				}
			}

			if (inside) return false;

			count = bestCount;
			for (int i = 0; i < bestCount; i++) { s[i] = best[i]; lambda[i] = bestLambda[i]; }
			return true;
		}

		struct Face
		{
			int v[3];
			Coord n;
			Real d;
			bool valid;
		};

		DYN_FUNC static bool makeFace(Face& f, const SupportPoint* verts, int a, int b, int c)
		{
			f.v[0] = a; f.v[1] = b; f.v[2] = c;
			Coord n = (verts[b].w - verts[a].w).cross(verts[c].w - verts[a].w);
			Real len = n.norm();
			if (len < Real(1e-12)) { f.valid = false; return false; }
			f.n = n / len;
			f.d = f.n.dot(verts[a].w);
			f.valid = true;
			return true;
		}

		/**
		 * @brief Grow a degenerate simplex (fewer than four points) into a tetrahedron enclosing the origin
		 */
		DYN_FUNC static bool expandSimplex(const Hull& A, const Hull& B, SupportPoint* s, int& count)
		{
			const Coord axes[6] = { Coord(1, 0, 0), Coord(-1, 0, 0), Coord(0, 1, 0), Coord(0, -1, 0), Coord(0, 0, 1), Coord(0, 0, -1) };

			if (count == 1)
			{
				for (int i = 0; i < 6 && count < 2; i++)
				{
					SupportPoint p = supportAB(A, B, axes[i]);
					if ((p.w - s[0].w).normSquared() > Real(1e-10)) s[count++] = p;
				}
			}

			if (count == 2)
			{
				Coord dir = s[1].w - s[0].w;
				Coord ref = fabs(dir[0]) < Real(0.6) ? Coord(1, 0, 0) : Coord(0, 1, 0);
				Coord perp = dir.cross(ref);
				for (int i = 0; i < 2 && count < 3; i++)
				{
					SupportPoint p = supportAB(A, B, i == 0 ? perp : -perp);
					if ((p.w - s[0].w).cross(dir).normSquared() > Real(1e-12)) s[count++] = p;
				}
			}

			if (count == 3)
			{
				Coord n = (s[1].w - s[0].w).cross(s[2].w - s[0].w);
				for (int i = 0; i < 2 && count < 4; i++)
				{
					SupportPoint p = supportAB(A, B, i == 0 ? n : -n);
					if (fabs((p.w - s[0].w).dot(n)) > Real(1e-10)) s[count++] = p;
				}
			}

			return count == 4;
		}

		DYN_FUNC static bool epa(TConvexContact<Real>& out, const Hull& A, const Hull& B, SupportPoint* simplex, int count)
		{
			const int MAX_VERTS = 40;
			const int MAX_FACES = 80;

			if (count < 4 && !expandSimplex(A, B, simplex, count))
			{
				// Touching but degenerate: report a zero-depth contact along the center line
				Coord d = B.center() - A.center();
				Real len = d.norm();
				out.normal = len > Real(0) ? d / len : Coord(0, 1, 0);
				out.pointB = B.support(-out.normal);
				out.pointA = A.support(out.normal);
				out.distance = Real(0);
				return true;
			}

			SupportPoint verts[MAX_VERTS];
			Face faces[MAX_FACES];
			int numVerts = 4, numFaces = 0;
			for (int i = 0; i < 4; i++) verts[i] = simplex[i];

			// Orient the initial tetrahedron outwards
			if ((verts[1].w - verts[0].w).cross(verts[2].w - verts[0].w).dot(verts[3].w - verts[0].w) > 0)
			{
				SupportPoint tmp = verts[1]; verts[1] = verts[2]; verts[2] = tmp;
			}
			makeFace(faces[numFaces++], verts, 0, 1, 2);
			makeFace(faces[numFaces++], verts, 0, 3, 1);
			makeFace(faces[numFaces++], verts, 0, 2, 3);
			makeFace(faces[numFaces++], verts, 1, 3, 2);

			int bestFace = -1;
			for (int iter = 0; iter < 32; iter++)
			{
				bestFace = -1;
				Real bestD = REAL_MAX;
				for (int f = 0; f < numFaces; f++)
				{
					if (faces[f].valid && faces[f].d < bestD) { bestD = faces[f].d; bestFace = f; }
				}
				if (bestFace < 0) return false;

				Face& fb = faces[bestFace];
				SupportPoint p = supportAB(A, B, fb.n);
				Real growth = p.w.dot(fb.n) - fb.d;
				if (growth < Real(1e-5) || numVerts >= MAX_VERTS) break;

				int pv = numVerts;
				verts[numVerts++] = p;

				// Remove faces visible from p and collect the horizon
				int edges[MAX_FACES * 3][2];
				int numEdges = 0;
				for (int f = 0; f < numFaces; f++)
				{
					if (!faces[f].valid) continue;
					if (faces[f].n.dot(p.w - verts[faces[f].v[0]].w) > Real(0))
					{
						for (int e = 0; e < 3; e++)
						{
							int a = faces[f].v[e], b = faces[f].v[(e + 1) % 3];
							bool shared = false;
							for (int k = 0; k < numEdges; k++)
							{
								if (edges[k][0] == b && edges[k][1] == a)
								{
									edges[k][0] = edges[numEdges - 1][0];
									edges[k][1] = edges[numEdges - 1][1];
									numEdges--;
									shared = true;
									break;
								}
							}
							if (!shared && numEdges < MAX_FACES * 3) { edges[numEdges][0] = a; edges[numEdges][1] = b; numEdges++; }
						}
						faces[f].valid = false;
					}
				}

				// Compact faces
				int w = 0;
				for (int f = 0; f < numFaces; f++) if (faces[f].valid) faces[w++] = faces[f];
				numFaces = w;

				if (numFaces + numEdges > MAX_FACES) break;
				for (int e = 0; e < numEdges; e++)
				{
					Face nf;
					if (makeFace(nf, verts, edges[e][0], edges[e][1], pv)) faces[numFaces++] = nf;
				}
			}

			if (bestFace < 0 || bestFace >= numFaces) return false;

			Face& f = faces[bestFace];
			// Barycentric coordinates of the origin projection on the closest face
			Coord a = verts[f.v[0]].w, b = verts[f.v[1]].w, c = verts[f.v[2]].w;
			Coord proj = f.n * f.d;
			Coord v0 = b - a, v1 = c - a, v2 = proj - a;
			Real d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1), d20 = v2.dot(v0), d21 = v2.dot(v1);
			Real denom = d00 * d11 - d01 * d01;
			Real v = 0, wgt = 0;
			if (fabs(denom) > Real(1e-20))
			{
				v = (d11 * d20 - d01 * d21) / denom;
				wgt = (d00 * d21 - d01 * d20) / denom;
			}
			Real u = 1 - v - wgt;

			out.pointA = verts[f.v[0]].a * u + verts[f.v[1]].a * v + verts[f.v[2]].a * wgt;
			out.pointB = verts[f.v[0]].b * u + verts[f.v[1]].b * v + verts[f.v[2]].b * wgt;
			// Translating A by -f.n * f.d separates the shapes, so f.n points from A to B
			out.normal = f.n;
			out.distance = -f.d;
			return true;
		}
	};
}
