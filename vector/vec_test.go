package vector

import (
	"fmt"
	"math"
	"testing"
)

//Vector module testing
func TestVecAdd(t *testing.T) {
	var x = Vec32{1.0, 1.0, 1.0}
	var y = Vec32{1, 1, 1}
	var eq = Vec32{2, 2, 2}

	if vecEquals(*x.Add(y), eq) {

	} else {
		t.Errorf("Vector Addition failed %f", x[0])
	}

}

//Vector module testing
func TestVecDot(t *testing.T) {
	var x = Vec32{1, 2, 3}
	var y = Vec32{1, 1, 1}
	var eq = float32(6.0)

	if Dot(x, y) == eq && x.Dot(y) == eq {

	} else {
		t.Errorf("Vector failed %f", x[0])
	}

}

func TestVector(t *testing.T) {
	x := NewVec32(2.0)
	y := NewDefaultVec32()
	z := NewDefaultVec32Array()
	g := NewVec32Array(10, 20)

	a := Vec32{2, 2, 2}
	b := Vec32{0, 0, 0}
	c := Vec32Array{make([]Vec32, 0, 64)}
	d := Vec32Array{make([]Vec32, 10, 20)}

	if !x.equals(a) && !y.equals(b) {
		t.Error()
	}

	if len(z.data) != len(c.data) {
		t.Error()
	}

	if len(g.data) != len(d.data) {
		t.Error()
	}

	if !vecEquals(Scale(a, 2.0), Vec32{4.0, 4.0, 4.0}) {
		t.Error()
	}
	if !vecEquals(Add(a, Vec32{2.0, 2.0, 2.0}), Vec32{4.0, 4.0, 4.0}) {
		t.Error()
	}

	if !isEpsilon(x.normalize().Length(), 1.0) {
		t.Errorf("Normalized vector error: A Length(): %f, %f, %f", x[0], x[1], x[2])
	}

	if !vecEquals(Cross(Vec32{-2, -2, -2}, Vec32{1, 2, 1}), Vec32{2, 0, -2}) {
		r := Cross(Vec32{-2, -2, -2}, Vec32{1, 2, 1})
		t.Errorf("Cross %f,%f,%f", r[0], r[1], r[2])
	}

	a = Vec32{2, 2, 2}

	if Length(a) != float32(math.Sqrt(12)) {
		t.Errorf("Error Length")
	}

	if a.Length() != float32(math.Sqrt(12)) {
		t.Errorf("Error Length")
	}

	a = Vec32{2, 2, 0}
	p := Vec32{0, 2, 0}
	r := Proj(a, p)
	h := ProjPlane(a, p)

	if !vecEquals(r, Vec32{0, 2, 0}) {
		t.Errorf("Error Projection %f %f %f", r[0], r[1], r[2])
	}

	if !vecEquals(h, Vec32{2, 0, 0}) {
		t.Errorf("Error Proj Plane  %f %f %f", h[0], h[1], h[2])
	}

	if !vecEquals(*a.Proj(p), Vec32{0, 2, 0}) {
		t.Errorf("Error Projection %f, %f, %f", a[0], a[1], a[2])
	}

	p = Vec32{1, -1, 0}
	o := Vec32{0, 1, 0}

	if !vecEquals(*p.Reflect(o), Vec32{1, 1, 0}) {
		t.Errorf("Error Reflection %f, %f, %f", p[0], p[1], p[2])
	}

}

func TestMatrix(t *testing.T) {
	A := Mat3{-2, 2, -3, -1, 1, 3, 2, 0, -1}
	B := Mat3{}
	C := Mat4{3, 2, 0, 1, 4, 0, 1, 2, 3, 0, 2, 1, 9, 2, 3, 1}
	D := Mat4{}

	a := Vec32{}
	b := Vec4{}

	id3 := Mat3{}
	id4 := Mat4{}

	var err error

	//Construct Mat 3
	for i := 0; i < MAT3; i++ {
		for j := 0; j < MAT3; j++ {
			index, e := mapN(i, j, MAT3)
			B[index] = float32(index)
			err = e
			if i == j {
				id3[index] = 1.0
			}
		}
		a[i] = float32(i)

	}

	//Construct Mat 4
	for i := 0; i < MAT4; i++ {
		for j := 0; j < MAT4; j++ {
			index, e := mapN(i, j, MAT4)
			D[index] = float32(index)
			err = e
			if i == j {
				id4[index] = 1.0
			}
		}
		b[i] = float32(i)
	}

	if err != nil {
		t.Errorf(err.Error())
	}

	det3 := A.Det()
	det4 := C.Det()
	m1, _ := id3.CrossMat(&A)
	m2, _ := id4.CrossMat(&C)
	c, _ := A.CrossVec(&a)
	d, _ := C.CrossVec(&b)

	fmt.Printf("Determinant Mat3: %f\n", det3)
	fmt.Printf("Determinant Mat4: %f\n", det4)
	fmt.Printf(m1.String())
	fmt.Printf(m2.String())

	fmt.Printf(c.String())
	fmt.Printf(d.String())

	//Compute Matrix Inverses
	invMat3 := A.Inverse()
	fmt.Printf("Inverse A Matrix 3x3\n")
	fmt.Printf(invMat3.String())

}

func BenchmarkVecOp(b *testing.B) {

	p := Vec32{1, -1, 0}
	o := Vec32{0, 1, 0}

	for i := 0; i < b.N; i++ {
		r := p.Add(o)
		Cross(*r, p)
		r.Proj(o)
		r.Add(o)
	}
}
