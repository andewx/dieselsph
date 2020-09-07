package geometry

import (
	"fmt"

	Fluid "diesel.com/diesel/fluid"
	Vec "diesel.com/diesel/vector"
)

//diesel geometry library - primarily for particle boundary collision detection
//may have to be extended for physical rendering properties
//This class will handle plane mesh object signed distance functions for particle collision
//Particle System runs from world coordinate intersections so no modelview / projection transforms

//These are for software object management and not yet being handled as single point buffer arrays for GPU
//Ideally the physics is handled by gl vertex shaders

type Triangle struct {
	Verts [3]*Vec.Vec32
}

type Mesh struct {
	Mesh    []Triangle
	Normals []Vec.Vec32
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

func InitMesh(triList []Triangle) Mesh {
	nMesh := Mesh{}
	nMesh.Mesh = triList
	nMesh.Normals = make([]Vec.Vec32, len(triList))
	for i := 0; i < len(triList); i++ {
		nMesh.Normals[i] = nMesh.Mesh[i].Normal()
	}

	return nMesh
}

func (tri *Triangle) Origin() *Triangle {
	V := Triangle{}
	V.Verts[0] = tri.Verts[0].Sub(*tri.Verts[0])
	V.Verts[1] = tri.Verts[1].Sub(*tri.Verts[0])
	V.Verts[2] = tri.Verts[2].Sub(*tri.Verts[0])

	return &V
}

func (tri *Triangle) Normal() Vec.Vec32 {
	N := Vec.Cross(Vec.Sub(*tri.Verts[1], *tri.Verts[0]), Vec.Sub(*tri.Verts[2], *tri.Verts[1]))
	return Vec.Normalize(N)
}

//Point Collision Test Function Wrapper
//Projects a point P onto the reference triangle and determines
//The Signed Distance and Whether the projection resulted in a barycentric
//Coordinate
func (t *Triangle) Collision(P *Vec.Vec32) (float32, bool) {

	//Measures if  a point projected into the triangles plane gives a barycentric coord
	_, Distance, isBarycentric := t.Barycentric(P)
	return Distance, isBarycentric

}

//Given particle w/ velocity determine barycentric collisions and if a collision occurs
func (g *Mesh) Collision(P *Fluid.Particle, dt float32) (Vec.Vec32, bool) {

	collision := false
	TRIS := len(g.Mesh)

	//We search each triangle for each particle, this needs to be improved
	//We could store triangles in their own spatial hash structure based on
	//Triangle Origin and Hash the Particle Position to find the comparison bucket
	for i := 0; i < TRIS; i++ {
		triangle := g.Mesh[i]
		nPos := Vec.Add(*P.GetPosition(), Vec.Scale(*P.GetVelocity(), dt))
		var dist float32
		var bary bool
		var ndist float32
		var nbary bool
		dist, bary = triangle.Collision(P.GetPosition())
		ndist, nbary = triangle.Collision(&nPos)

		//Barycentric with a Sign Change
		if bary == true && nbary == true {
			if dist-ndist > dist {
				collision = true
				normal := triangle.Normal()
				return normal, collision
			}
		}
	}

	return Vec.Vec32{}, collision
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
	fmt.Println("Point XY Plane Projection Coords: " + P.String())
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
	fmt.Println(Inv.String())
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
	const Verts = 3
	var BoxTris = make([]Triangle, 12)
	x := o[0]
	y := o[1]
	z := o[2]

	p := w / 2
	q := h / 2
	s := d / 2

	//FRONT FACE -Z
	BoxTris[0].Verts[0] = &Vec.Vec32{x - p, y - q, z - s} //LFB
	BoxTris[0].Verts[1] = &Vec.Vec32{x - p, y + q, z - s} //LFT
	BoxTris[0].Verts[2] = &Vec.Vec32{x + p, y - q, z - s} //RFB

	BoxTris[1].Verts[0] = &Vec.Vec32{x - p, y + q, z - s} //LFT
	BoxTris[1].Verts[1] = &Vec.Vec32{x + p, y + q, z - s} //RFT
	BoxTris[1].Verts[2] = &Vec.Vec32{x + p, y - q, z - s} //RFB

	//BACK FACE -Z
	BoxTris[2].Verts[0] = &Vec.Vec32{x - p, y - q, z + s} //LBB
	BoxTris[2].Verts[1] = &Vec.Vec32{x - p, y + q, z + s} //LBT
	BoxTris[2].Verts[2] = &Vec.Vec32{x + p, y - q, z + s} //RBB

	BoxTris[3].Verts[0] = &Vec.Vec32{x - p, y + q, z + s} //LBT
	BoxTris[3].Verts[1] = &Vec.Vec32{x + p, y + q, z + s} //RBT
	BoxTris[3].Verts[2] = &Vec.Vec32{x + p, y - q, z + s} //RBB

	//BOTTOM FACE -Y
	BoxTris[4].Verts[0] = &Vec.Vec32{x - p, y - q, z - s} //LFB
	BoxTris[4].Verts[1] = &Vec.Vec32{x - p, y - q, z + s} //LBB
	BoxTris[4].Verts[2] = &Vec.Vec32{x + p, y - q, z + s} //RBB

	BoxTris[5].Verts[0] = &Vec.Vec32{x - p, y - q, z + s} //LFB
	BoxTris[5].Verts[1] = &Vec.Vec32{x + p, y - q, z + s} //RFB
	BoxTris[5].Verts[2] = &Vec.Vec32{x + p, y - q, z - s} //RBB

	//Top FACE - Y
	BoxTris[6].Verts[0] = &Vec.Vec32{x - p, y + q, z - s} //LFT
	BoxTris[6].Verts[1] = &Vec.Vec32{x - p, y + q, z + s} //LBT
	BoxTris[6].Verts[2] = &Vec.Vec32{x + p, y + q, z + s} //RBT

	BoxTris[7].Verts[0] = &Vec.Vec32{x - p, y + q, z + s} //LFT
	BoxTris[7].Verts[1] = &Vec.Vec32{x + p, y + q, z + s} //RFT
	BoxTris[7].Verts[2] = &Vec.Vec32{x + p, y + q, z - s} //RBT

	//LEFT FACE - X
	BoxTris[8].Verts[0] = &Vec.Vec32{x - p, y + q, z + s} //LFT
	BoxTris[8].Verts[1] = &Vec.Vec32{x - p, y - q, z + s} //LFB
	BoxTris[8].Verts[2] = &Vec.Vec32{x - p, y - q, z + s} //LBB

	BoxTris[9].Verts[0] = &Vec.Vec32{x - p, y + q, z + s} //LFT
	BoxTris[9].Verts[1] = &Vec.Vec32{x - p, y + q, z - s} //LFB
	BoxTris[9].Verts[2] = &Vec.Vec32{x - p, y - q, z + s} //LBB

	//Right FACE - X
	BoxTris[10].Verts[0] = &Vec.Vec32{x + p, y + q, z + s} //LFT
	BoxTris[10].Verts[1] = &Vec.Vec32{x + p, y - q, z + s} //LFB
	BoxTris[10].Verts[2] = &Vec.Vec32{x + p, y - q, z - s} //LBB

	BoxTris[11].Verts[0] = &Vec.Vec32{x + p, y + q, z + s} //RFT
	BoxTris[11].Verts[1] = &Vec.Vec32{x + p, y + q, z - s} //RFB
	BoxTris[11].Verts[2] = &Vec.Vec32{x + p, y - q, z - s} //RBB

	boxMesh := InitMesh(BoxTris)
	return &boxMesh

}

//We  ill use the Displacement Vector Formulation In Order to Calculate Barycentric Coordinates
//This eliminates the need for Projecting Vectors
/*
func (tri * Triangle)Barycentric(p *Vec.Vec32)(*Vec.Vec32 ,float32){

  //Perform V0 Origin Transform
  T := tri.Origin()
  P := Vec.Normalize(p.Sub(*tri.Verts[0]))
  A := Vec.Normalize(T.Verts[0])
  B := Vec.Normalize(T.Verts[1])
  C := Vec.Normalize(T.Verts[2])
  //Project our Point into the Plane of our triangle
  N := Vec.Cross(*C,*B)
  PROJ :=Vec.Proj(*P,N)

  DISTANCE := P.Sub(PROJ).Length()
  //Dispalne Vectors in this space calculated as |PQ|^2 = (Q-P)*Q-P
  //Calculate Triangle Displacements
  a := A.Sub(*B)
  al := a.Dot(*a)
  b := B.Sub(*C)
  bl := b.Dot(*b)
  c := C.Sub(*A)
  cl := c.Dot(*c)
  //Point to Edge Displacements
  da := PROJ.Sub(*a)
  dal := da.Dot(*da)
  db := PROJ.Sub(*b)
  dbl := db.Dot(*db)
  dc := PROJ.Sub(*c)
  dcl := dc.Dot(*dc)
  //Form 3 x 3 matrix
  Mat3 := Vec.Mat3{}
  Mat3[0] = -cl
  Mat3[1] = cl
  Mat3[2] = bl-al
  Mat3[3] = -bl
  Mat3[4] = cl-al
  Mat3[5] = bl
  Mat3[6] = 1
  Mat3[7] = 1
  Mat3[8] = 1
  //Distance Vectors
  DVec := Vec.Vec32{dal-dbl, dal-dcl, 1.0}
  //Mat 3x 3 Inverse Calcula
  Inv := Mat3.Inverse()
  RVec,_ := Inv.CrossVec(&DVec)
  return  RVec, DISTANCE


}
*/
