package geometry

import (
	"fmt"
	"testing"

	"diesel.com/diesel/vector"
)

//Intuitive Geometry Coordinate Tests
func TestGeometry(t *testing.T) {
	V0 := vector.Vec32{0, 0, 0}
	V1 := vector.Vec32{-1, 0, -2}
	V2 := vector.Vec32{-1, 1, -1}
	P1 := vector.Vec32{-0.5, 0, -0.5}

	tTriangle := Triangle{}
	tTriangle.verts[0] = &V0
	tTriangle.verts[1] = &V1
	tTriangle.verts[2] = &V2

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

}
