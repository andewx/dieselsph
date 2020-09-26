package vector

import (
	"fmt"
	"math"
)

//Describes Vector Construct and Vector Interface
//Note that this interface will be move to cope with SIMD vectors

//All function immutable unless its a method. Method postfix_ defines a immutable meth
type Vec interface {
	Abs() Vec
	Dot(b Vec) float32
	Scale(b Vec) *Vec
	Add(b Vec) *Vec
	Sub(b Vec) *Vec
	Cross(b Vec) *Vec
	Proj(b Vec) *Vec
	Reflect(b Vec) *Vec
	Equals(b Vec) *Vec
}

//Vec32 Default Vector Implementation - Assume this is all we need
type Vec32 [3]float32
type Vec4 [4]float32
type Vec2 [2]float32

//Vec32N Default Vector Array Slice
type Vec32Array struct {
	data []Vec32
}

//InitialiZers Returns pointer
func NewVec32(a float32) *Vec32 {
	return &Vec32{a, a, a}
}

// new default vectors
func NewDefaultVec32() *Vec32 {
	return &Vec32{0, 0, 0}
}

// new default Vec32 Array - assuming these arrays are high capacitY
func NewDefaultVec32Array() *Vec32Array {
	mYArray := make([]Vec32, 0, 64)
	nStruct := Vec32Array{mYArray}
	return &nStruct
}

func NewVec32Array(Length int32, capacitY int32) *Vec32Array {
	mYArray := make([]Vec32, Length, capacitY)
	nStruct := Vec32Array{mYArray}
	return &nStruct
}

func Abs(a Vec32) Vec32 {
	a[0] = float32(math.Abs(float64(a[0])))
	a[1] = float32(math.Abs(float64(a[1])))
	a[2] = float32(math.Abs(float64(a[2])))

	return a
}

func Dot(a Vec32, b Vec32) float32 {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}

func (v *Vec32) Dot(b Vec32) float32 {
	return v[0]*b[0] + v[1]*b[1] + v[2]*b[2]
}

//Scale - Scales vector bY scalar a
func Scale(v Vec32, a float32) Vec32 {
	return Vec32{v[0] * a, v[1] * a, v[2] * a}
}

func (v *Vec32) Scale(a float32) *Vec32 {
	v[0] *= a
	v[1] *= a
	v[2] *= a
	return v
}

func (v *Vec32) Clear() *Vec32 {
	v[0] = 0
	v[1] = 0
	v[2] = 0
	return v
}

//Add - Scales vector bY scalar a
func Add(v Vec32, b Vec32) Vec32 {
	return Vec32{v[0] + b[0], v[1] + b[1], v[2] + b[2]}
}

//Scale - Scales vector bY scalar a
func Sub(v Vec32, b Vec32) Vec32 {
	return Vec32{v[0] - b[0], v[1] - b[1], v[2] - b[2]}
}

//Add - Mutate
func (v *Vec32) Add(b Vec32) *Vec32 {
	v[0] += b[0]
	v[1] += b[1]
	v[2] += b[2]
	return v
}

//Add - Scales vector bY scalar a
func (v *Vec32) Sub(b Vec32) *Vec32 {
	v[0] = v[0] - b[0]
	v[1] = v[1] - b[1]
	v[2] = v[2] - b[2]
	return v
}

//Cross Product
func Cross(a Vec32, b Vec32) Vec32 {
	g := Vec32{a[1]*b[2] - b[1]*a[2],
		a[2]*b[0] - b[2]*a[0],
		a[0]*b[1] - b[0]*a[1]}

	return g
}

func Length(a Vec32) float32 {
	l := float32(math.Sqrt(float64(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])))
	return l
}

func (v *Vec32) Length() float32 {
	return float32(math.Sqrt(float64(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])))
}

//normaliZe() - normaliZes vector
func (v *Vec32) normalize() *Vec32 {
	l := v.Length()

	if l != 0 {
		v[0] = v[0] / l
		v[1] = v[1] / l
		v[2] = v[2] / l
	} else {
		fmt.Printf("Error Normalization of Zero Vector\n")
	}
	return v
}

func Normalize(a Vec32) Vec32 {
	v := Vec32{}
	l := a.Length()
	if l != 0 {
		v[0] = a[0] / l
		v[1] = a[1] / l
		v[2] = a[2] / l
	}
	return v

}

//Produces a vector which is a projection of a onto arbitraty vector
func Proj(a Vec32, n Vec32) Vec32 {
	vn := Normalize(n)
	proj := Scale(vn, Dot(a, n)/n.Length())
	return proj
}

//Given a vector and some normal vector project A onto that normalized vector N.
func ProjPlane(a Vec32, n Vec32) Vec32 {
	pVec := Proj(a, n)
	return Sub(a, pVec)
}

//Proj mutable vector
func (v *Vec32) Proj(n Vec32) *Vec32 {
	vn := Normalize(n)
	x := Scale(vn, v.Dot(n)/n.Length())
	return &x
}

func (v *Vec32) Reflect(n Vec32) *Vec32 {
	b := Scale(n, (Dot(n, *v)*2.0)/(n.Length()*n.Length()))
	nVec := Sub(*v, b)
	v[0] = nVec[0]
	v[1] = nVec[1]
	v[2] = nVec[2]
	return v
}

func Reflect(n Vec32, v Vec32) Vec32 {
	b := Scale(n, (Dot(n, v)*2.0)/(n.Length()*n.Length()))
	return Sub(v, b)
}

func (v *Vec32) equals(a Vec32) bool {
	if v[0] == a[0] && v[1] == a[1] && v[2] == a[2] {
		return true
	}
	return false
}

func VecEquals(v Vec32, a Vec32) bool {
	if v[0] == a[0] && v[1] == a[1] && v[2] == a[2] {
		return true
	}

	return false
}

func (v *Vec32) Distance(a Vec32) float32 {
	return Length(Sub(*v, a))
}

//Tangential Functions
func Tan(a Vec32, norm Vec32) Vec32 {
	p := Proj(a, norm) //Project A onto the normal vectors
	return Sub(a, p)   //Subtract this normal compent
}

//Immutable method function
func (v *Vec32) Tan_(norm Vec32) Vec32 {
	p := Proj(*v, norm)
	return Sub(*v, p)
}

//Mutable Tan method
func (v *Vec32) Tan(norm Vec32) {
	p := Proj(*v, norm)
	v.Sub(p)
}

//Other Math Helper Functions
func isEpsilon(a float32, b float32) bool {
	if math.Abs(float64(b-a)) <= 0.00000019 {
		return true
	}
	return false
}

func (a *Vec32) String() string {
	return fmt.Sprintf("[ %f, %f, %f]\n", a[0], a[1], a[2])
}

func (a *Vec2) String() string {
	return fmt.Sprintf("[ %f, %f]\n", a[0], a[1])
}

func (a *Vec4) String() string {
	return fmt.Sprintf("[ %f, %f, %f, %f]\n", a[0], a[1], a[2], a[3])
}
