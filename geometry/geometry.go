package geometry

import (
	Vec "diesel.com/diesel/vector"
	Mgl "github.com/go-gl/mathgl/mgl32"
)

const (
	EPSILON = 0.00001
)

//diesel geometry library - primarily for particle boundary collision detection
//may have to be extended for physical rendering properties
//This class will handle plane mesh object signed distance functions for particle collision
//Particle System runs from world coordinate intersections so no modelview / projection transforms

//These are for software object management and not yet being handled as single point buffer arrays for GPU
//Ideally the physics is handled by gl vertex shaders

//SPH Collision depends on Clockwise Winded Triangles -- Tried to make it agnostic to the orientation of the Normal
// By flipping based on position but it just wasn't working

type Triangle struct {
	Verts [3]*Vec.Vec32
}

//Triangle Mesh Storage
type Mesh struct {
	Vertexes []Vec.Vec32
	Normals  []Vec.Vec32
}

type Sphere struct {
	radius float32
	origin Vec.Vec32
}

//Triangle Subtractions

func InitTriangle(a Vec.Vec32, b Vec.Vec32, c Vec.Vec32) Triangle {
	V := Triangle{}
	V.Verts[0] = &a
	V.Verts[1] = &b
	V.Verts[2] = &c

	return V
}

func InitMesh(vertices []Vec.Vec32) Mesh {
	nMesh := Mesh{}
	nMesh.Vertexes = vertices
	nMesh.Normals = make([]Vec.Vec32, len(vertices)/3)
	index := 0
	for i := 0; i < len(vertices)-3; i += 3 {
		thisTriangle := InitTriangle(vertices[i], vertices[i+1], vertices[i+2])
		nMesh.Normals[index] = thisTriangle.Normal()
		index++
	}

	return nMesh
}

func (tri *Triangle) Origin() *Triangle {
	V := Triangle{}
	V.Verts[0] = tri.Verts[0].Sub(*tri.Verts[0])
	V.Verts[1] = tri.Verts[1].Sub(*tri.Verts[1])
	V.Verts[2] = tri.Verts[2].Sub(*tri.Verts[2])

	return &V
}

func (tri *Triangle) Normal() Vec.Vec32 {
	N := Vec.Cross(Vec.Sub(*tri.Verts[1], *tri.Verts[0]), Vec.Sub(*tri.Verts[2], *tri.Verts[0]))
	return Vec.Normalize(N)
}

//Barycentric Collission Test returns float32 distance, bool collision detected
func (t *Triangle) Collision(P *Vec.Vec32) (float32, bool) {

	//Measures if  a point projected into the triangles plane gives a barycentric coord
	_, Distance, isBarycentric := t.Barycentric(P)
	return Distance, isBarycentric

}

//Computes Ray Triangle Collision with Plane Equation substitution and Boundary Test
func (t *Triangle) RayTriCollision(point Vec.Vec32, vel Vec.Vec32, n Vec.Vec32, dt float32) (float32, Vec.Vec32, bool) {
	t0 := t.Verts[0]
	t1 := t.Verts[1]
	t2 := t.Verts[2]

	//Sanitize the Normal with respect to point position
	pt2Vtx := Vec.Sub(point, *t1)
	dotP2X := Vec.Dot(n, pt2Vtx)

	if dotP2X >= 0 {
		n.Scale(-1.0)
	}

	nDotRay := Vec.Dot(n, vel)

	//Parallel to plane No Intersection
	if Mgl.Abs(nDotRay) < EPSILON {
		return 0, Vec.Vec32{}, false
	}

	//Intersection Dynamics for our point
	d := Vec.Dot(n, *t0)
	k := (Vec.Dot(n, point) + d) / nDotRay
	p0 := Vec.Add(point, Vec.Scale(vel, k))
	dist := Vec.Length(Vec.Sub(point, p0))

	p1 := Vec.Add(point, Vec.Scale(vel, (dt))) //dt fluids small

	//Snap P1 to the Plane If within Bounding Region
	p1Dist := Vec.Length(Vec.Sub(p0, p1))

	//If we're within SOS collision time factor
	if p1Dist < Vec.Length(vel)*dt { //Should just be a few milliseconds
		p1 = Vec.Add(p0, Vec.Scale(vel, dt))
	}
	dir2Plane := Vec.Sub(p0, p1)

	//Check that point crosses the Plane
	p0Trans := Vec.Add(point, dir2Plane) //Point Near Plane if dir2Plane is
	dist2 := Vec.Length(Vec.Sub(p0, p0Trans))

	//If directions are opposite dot < 0 (general)
	if dist2 < dist && Vec.Dot(n, dir2Plane) > 0 { //Point Was only moved closer
		return dist, p0, false
	}

	//Plane has been crossed

	//At this point the point has crossed the plane
	//We need to check if it was within the bounds with an edge test of all three edges
	//Edge 0 Test

	e0 := Vec.Sub(*t1, *t0)
	vp0 := Vec.Sub(point, *t0)
	C0 := Vec.Cross(e0, vp0) //Vector Perpendicular to Triangle Plane

	if Vec.Dot(n, C0) > 0 { //P is on the right side
		return dist, p0, false
	}

	e1 := Vec.Sub(*t2, *t1)
	vp1 := Vec.Sub(point, *t1)
	C1 := Vec.Cross(e1, vp1)

	if Vec.Dot(n, C1) > 0 { //P is on the right side
		return dist, p0, false
	}

	//Edge 2
	e2 := Vec.Sub(*t0, *t2)
	vp2 := Vec.Sub(point, *t2)
	C2 := Vec.Cross(e2, vp2)

	if Vec.Dot(n, C2) > 0 {
		return dist, p0, false
	}

	return dist, p0, true

}

//Given particle w/ velocity determine barycentric collisions and if a collision occurs
//This code should be ray triangle intersection code similar to ray tracing
func (g *Mesh) Collision(P Vec.Vec32, V Vec.Vec32, dt float32) (Vec.Vec32, Vec.Vec32, bool) {

	VERTS := len(g.Vertexes)
	for i := 0; i < VERTS; i += 3 {
		triangle := InitTriangle(g.Vertexes[i], g.Vertexes[i+1], g.Vertexes[i+2])
		n := triangle.Normal()

		_, p0, c0 := triangle.RayTriCollision(P, V, n, dt) //Valid Collision with First Coord

		//Collision
		if c0 {
			return n, p0, true
		}

	}

	return Vec.Vec32{}, Vec.Vec32{}, false
}

//Planar Projection Transform of a triangle onto a Normal Vector
func (t *Triangle) Project(N Vec.Vec32) Triangle {
	nTri := Triangle{}
	a := Vec.ProjPlane(*t.Verts[0], N)
	b := Vec.ProjPlane(*t.Verts[1], N)
	c := Vec.ProjPlane(*t.Verts[2], N)
	nTri.Verts[0] = &a
	nTri.Verts[1] = &b
	nTri.Verts[2] = &c
	return nTri
}

//````````````````````````````````Planar Point R Formulation
func (tri *Triangle) Barycentric(p *Vec.Vec32) (*Vec.Vec2, float32, bool) {

	//Perform V0 Origin Transform
	T := tri
	XY := Vec.Vec32{0, 0, 1}
	TN := T.Normal()
	P1 := Vec.ProjPlane(*p, TN)
	//Project our Point Into the Triangle // Then Project into the XY Plane
	ProjTriangle := T.Project(XY)
	P := Vec.ProjPlane(P1, XY)
	DISTANCE := P1.Sub(*p).Length()
	//Now Project All Points into the XZ Plane (XY?) to give cartesian Coordinates
	AP := ProjTriangle.Verts[0]
	BP := ProjTriangle.Verts[1]
	CP := ProjTriangle.Verts[2]

	//Calculate Triangle Displacement
	m0 := AP[0] - CP[0] // x1
	m1 := BP[0] - CP[0] // x2
	m2 := AP[1] - CP[1] // y1
	m3 := BP[1] - CP[1] // y2
	//Form 3 x 3 matrix
	Mat2 := Vec.Mat2{m0, m1, m2, m3}

	//Displace the point coordinate by the C point offset
	R := P.Sub(*CP)
	//Cartesian Point
	DVec := Vec.Vec2{R[0], R[1]}
	//Mat 3x 3 Inverse Calcula
	Inv, _ := Mat2.Inverse()
	//  fmt.Printf(Inv.String())
	RVec, _ := Inv.CrossVec(&DVec)
	bRes := false
	if RVec[0]+RVec[1] <= 1 {
		bRes = true
	}
	return RVec, DISTANCE, bRes

}

//Triangle Mesh Box with 12 Triangles // 36 Vertexes
func Box(w float32, h float32, d float32, o Vec.Vec32) *Mesh {
	const TRIANGLES = 12
	var Verts = make([]Vec.Vec32, 12*3)
	x := o[0]
	y := o[1]
	z := o[2]

	p := w / 2
	q := h / 2
	s := d / 2

	//FRONT FACE -Z
	Verts[0] = Vec.Vec32{x - p, y - q, z - s} //LFB
	Verts[1] = Vec.Vec32{x - p, y + q, z - s} //LFT
	Verts[2] = Vec.Vec32{x + p, y - q, z - s} //RFB

	Verts[3] = Vec.Vec32{x - p, y + q, z - s} //LFT
	Verts[4] = Vec.Vec32{x + p, y + q, z - s} //RFT
	Verts[5] = Vec.Vec32{x + p, y - q, z - s} //RFB

	//BACK FACE -Z
	Verts[6] = Vec.Vec32{x - p, y - q, z + s} //LBB
	Verts[7] = Vec.Vec32{x - p, y + q, z + s} //LBT
	Verts[8] = Vec.Vec32{x + p, y - q, z + s} //RBB

	Verts[9] = Vec.Vec32{x - p, y + q, z + s}  //LBT
	Verts[10] = Vec.Vec32{x + p, y + q, z + s} //RBT
	Verts[11] = Vec.Vec32{x + p, y - q, z + s} //RBB

	//BOTTOM FACE -Y
	Verts[12] = Vec.Vec32{x - p, y - q, z + s} //LFB
	Verts[13] = Vec.Vec32{x - p, y - q, z - s} //LBB
	Verts[14] = Vec.Vec32{x + p, y - q, z - s} //RBB

	Verts[15] = Vec.Vec32{x - p, y - q, z + s} //LFB
	Verts[16] = Vec.Vec32{x + p, y - q, z - s} //RBB
	Verts[17] = Vec.Vec32{x + p, y - q, z + s} //RFB

	//Top FACE - Y
	Verts[18] = Vec.Vec32{x - p, y + q, z + s} //LFT
	Verts[19] = Vec.Vec32{x - p, y + q, z - s} //LBT
	Verts[20] = Vec.Vec32{x + p, y + q, z - s} //RBT

	Verts[21] = Vec.Vec32{x - p, y + q, z - s} //LBT
	Verts[22] = Vec.Vec32{x + p, y + q, z - s} //RBT
	Verts[23] = Vec.Vec32{x + p, y + q, z + s} //RFT

	//LEFT FACE - X
	Verts[24] = Vec.Vec32{x - p, y - q, z + s} //LFB
	Verts[25] = Vec.Vec32{x - p, y + q, z + s} //LFT
	Verts[26] = Vec.Vec32{x - p, y - q, z - s} //LBB

	Verts[27] = Vec.Vec32{x - p, y + q, z + s} //LFT
	Verts[28] = Vec.Vec32{x - p, y + q, z - s} //LBT
	Verts[29] = Vec.Vec32{x - p, y - q, z - s} //LBB

	//Right FACE - X
	Verts[30] = Vec.Vec32{x + p, y + q, z + s} //LFT
	Verts[31] = Vec.Vec32{x + p, y - q, z + s} //LFB
	Verts[32] = Vec.Vec32{x + p, y - q, z - s} //LBB

	Verts[33] = Vec.Vec32{x + p, y + q, z + s} //RFT
	Verts[34] = Vec.Vec32{x + p, y + q, z - s} //RFB
	Verts[35] = Vec.Vec32{x + p, y - q, z - s} //RBB

	boxMesh := InitMesh(Verts)
	return &boxMesh

}
