package geometry

import (
	"diesel.com/diesel/vector"
	"fmt"
	"testing"
)

//Intuitive Geometry Coordinate Tests
func TestGeometry(t *testing.T) {
	V0 := vector.Vec32{-1, 0, 0}
	V1 := vector.Vec32{0, 0, -1}
	V2 := vector.Vec32{1, 0, 0}
	P1 := vector.Vec32{0, 0.5, -0.51}
	P2 := vector.Vec32{1, 0.5, -0.5}

	tTriangle := InitTriangle(V0, V1, V2)
	//Return Collision and result between each of the three points
	fmt.Printf("Testing Triangle Collisions\n")

	fmt.Printf("Transformed Triangle Coordinates\n")
	coords1, dist1, isCollision := tTriangle.Barycentric(&P1)

	if !isCollision {
		t.Errorf("Point 1 Should be a barycentric collision\n Point Triangle Distance Project %f\n", dist1)
		fmt.Printf("Barycentric Coordinates:%s\nDistance %f\n", coords1.String(), dist1)
	} else {
		fmt.Printf("Barycentric Coordinates:%s\nDistance %f\n", coords1.String(), dist1)
	}

	var meshVerts []vector.Vec32
	meshVerts = make([]vector.Vec32, 3)
	meshVerts[0] = V0
	meshVerts[1] = V1
	meshVerts[2] = V2

	G := InitMesh(meshVerts)
	vel := vector.Vec32{0, -1, 0}
	dist, P0, c0 := G.Collision(P1, vel, 0.1)
	dist2, P3, c1 := G.Collision(P2, vel, 0.1)

	i := 0
	for !c0 && i < 10 {
		npoint := vector.Add(P1, vector.Scale(vel, float32(i)*0.1))
		dist, P0, c0 = G.Collision(npoint, vel, 0.1)
		i++
	}

	i = 0
	for !c0 && i < 10 {
		npoint := vector.Add(P2, vector.Scale(vel, float32(i)*0.1))
		dist, P3, c0 = G.Collision(npoint, vel, 0.1)
		i++
	}

	if !c0 {
		t.Errorf("Point Error Collide:\nDistance %f\nP0: %f", dist, P0)
	}

	if c1 {
		t.Errorf("Point Error Collide:\nDistance %f\nP1: %f", dist2, P3)
	}

}
