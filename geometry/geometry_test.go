package geometry

import (
	"diesel.com/diesel/vector"
	"fmt"
	"testing"
)

//Intuitive Geometry Coordinate Tests
func TestGeometry(t *testing.T) {
	V0 := vector.Vec32{-1, 0, -2}
	V1 := vector.Vec32{0, 1, -2}
	V2 := vector.Vec32{1, 0, -2}
	P1 := vector.Vec32{0, 0.0, -1.5}
	P2 := vector.Vec32{1, 1, -1.5}

	tTriangle := InitTriangle(V0, V1, V2)
	//Return Collision and result between each of the three points
	fmt.Printf("Testing Triangle Collisions\n")

	fmt.Printf("Transformed Triangle Coordinates\n")
	coords1, isCollision := tTriangle.Barycentric(&P1)

	if !isCollision {
		t.Errorf("Point 1 Should be a barycentric collision\n\n")
		fmt.Printf("Barycentric Coordinates:%s\n", coords1.String())
	} else {
		fmt.Printf("Barycentric Coordinates:%s\n", coords1.String())
	}

	var meshVerts []vector.Vec32
	meshVerts = make([]vector.Vec32, 3)
	meshVerts[0] = V0
	meshVerts[1] = V1
	meshVerts[2] = V2

	G := InitMesh(meshVerts)
	vel := vector.Vec32{0, 0, -1}
	n1, P0, c0 := G.Collision(P1, vel, 0.1)
	n2, P3, c1 := G.Collision(P2, vel, 0.1)

	i := 0
	for !c0 && i < 10 {
		npoint := vector.Add(P1, vector.Scale(vel, float32(i)*0.1))
		n1, P0, c0 = G.Collision(npoint, vel, 0.1)
		i++
	}

	i = 0
	for !c1 && i < 10 {
		npoint := vector.Add(P2, vector.Scale(vel, float32(i)*0.1))
		n2, P3, c1 = G.Collision(npoint, vel, 0.1)
		i++
	}

	if !c0 {
		t.Errorf("Did not calc a collision:\nNormal %s\nCoord: %s", n1.String(), P0.String())
	} else {
		fmt.Printf("Collision: %sWith Normal: %s\n", P0.String(), n1.String())
	}

	if c1 {
		t.Errorf("Faulty Collision:\nNormal %s\nCoord: %s", n2.String(), P3.String())
	} else {
		fmt.Printf("Success no collision %s\n", n2.String())
	}

}
