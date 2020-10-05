package geometry

import (
	Vec "diesel.com/diesel/vector"
	//Mgl "github.com/go-gl/mathgl/mgl32"
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
func (t *Triangle) Collision(P *Vec.Vec32) (Vec.Vec32, bool) {

	//Measures if  a point projected into the triangles plane gives a barycentric coord
	coord, isBarycentric := t.Barycentric(P)
	return coord, isBarycentric

}

//Barycentric Focused Collision Test
func (t *Triangle) BarycentricCollision(P Vec.Vec32, V Vec.Vec32, dt float32) (Vec.Vec32, Vec.Vec32, bool) {
	t0 := *t.Verts[0]
	//Take Care of Normal
	n := t.Normal()
	v0 := Vec.Sub(t0, P)
	dv0 := Vec.Dot(n, v0)

	//Point Normal Towards Point - If < 0 Normal is away from the point
	if dv0 >= 0 {
		n.Scale(-1.0)
		dv0 = Vec.Dot(n, v0)
	}

	nDotRay := Vec.Dot(n, V)

	//Point Plane Distance Projected onto the Velocity
	if nDotRay == 0 {
		nDotRay = 0.0001
	}

	d := Vec.Dot(v0, n)
	k := (d) / (nDotRay)              //Project distance to velocity Vector
	p0 := Vec.Add(P, Vec.Scale(V, k)) //Projection to the plane
	dist := Vec.Length(Vec.Sub(P, p0))

	//Check the P1 is crossed current plane
	p1 := Vec.Add(P, Vec.Scale(V, dt))
	p10 := Vec.Sub(p1, p0)
	dotp10 := Vec.Dot(n, p10)

	//Point Crossed the plane in a time step. We don't care about the actual collision point
	//This needs to be scaled with velocity or time step needs to be decreased
	if (dotp10 > 0 && dv0 < 0) && dist < 0.05 {
		coord, collision := t.Barycentric(&P)
		return n, coord, collision
	} else {
		return n, Vec.Vec32{}, false
	}

}

//Given particle w/ velocity determine barycentric collisions and if a collision occurs
//This code should be ray triangle intersection code similar to ray tracing. Returns flipped normal
func (g *Mesh) Collision(P Vec.Vec32, V Vec.Vec32, dt float32) (Vec.Vec32, Vec.Vec32, bool) {

	VERTS := len(g.Vertexes)

	for i := 0; i < VERTS; i += 3 {
		triangle := InitTriangle(g.Vertexes[i], g.Vertexes[i+1], g.Vertexes[i+2])

		fN, coord, c0 := triangle.BarycentricCollision(P, V, dt)

		//Collision
		if c0 {
			return fN, coord, true
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

//Project XY, XZ, YZ - Plane must be
func (t *Triangle) Barycentric(p *Vec.Vec32) (Vec.Vec32, bool) {

	v0 := Vec.Sub(*t.Verts[1], *t.Verts[0])
	v1 := Vec.Sub(*t.Verts[2], *t.Verts[0])
	v2 := Vec.Sub(*p, *t.Verts[0])
	d00 := Vec.Dot(v0, v0)
	d01 := Vec.Dot(v0, v1)
	d11 := Vec.Dot(v1, v1)
	d20 := Vec.Dot(v2, v0)
	d21 := Vec.Dot(v2, v1)
	denom := d00*d11 - d01*d01
	u := (d11*d20 - d01*d21) / denom
	v := (d00*d21 - d01*d20) / denom
	w := 1.0 - v - u
	coord := Vec.Vec32{u, v, w}
	collision := false
	if u <= 1.0 && v <= 1.0 && w <= 1.0 && (u+v+w) <= 1.0 && u >= 0 && v >= 0 && w >= 0 {
		collision = true
	}

	return coord, collision

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
	Verts[0] = Vec.Vec32{x - p, y - q, z + s} //LFB
	Verts[1] = Vec.Vec32{x - p, y + q, z + s} //LFT
	Verts[2] = Vec.Vec32{x + p, y + q, z + s} //RFT

	Verts[3] = Vec.Vec32{x + p, y + q, z + s} //RFT
	Verts[4] = Vec.Vec32{x + p, y - q, z + s} //RFB
	Verts[5] = Vec.Vec32{x - p, y - q, z + s} //LFB

	//BACK FACE -Z
	Verts[6] = Vec.Vec32{x - p, y - q, z - s} //LBB
	Verts[7] = Vec.Vec32{x - p, y + q, z - s} //LBT
	Verts[8] = Vec.Vec32{x + p, y - q, z - s} //RBB

	Verts[9] = Vec.Vec32{x - p, y + q, z - s}  //LBT
	Verts[10] = Vec.Vec32{x + p, y + q, z - s} //RBT
	Verts[11] = Vec.Vec32{x + p, y - q, z - s} //RBB

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

	Verts[21] = Vec.Vec32{x + p, y + q, z - s} //RBT
	Verts[22] = Vec.Vec32{x + p, y + q, z + s} //RFT
	Verts[23] = Vec.Vec32{x - p, y + q, z + s} //LFT

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
