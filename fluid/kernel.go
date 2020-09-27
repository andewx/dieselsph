package fluid

import V "diesel.com/diesel/vector"

const PI = 3.141592653589
const PI2 = PI * 2
const SQRT2PI = 2.50662827463

//Kernel Interface for Particle Integration - Kernels return Relative Weight of Integrator
type Kernel interface {
	F(distance float32) float32                  //F stands for inline () Function operator since we can't operator overload
	O1D(distance float32) float32                //First Order Derivative of Estimator
	O2D(distance float32) float32                //Second Order Derivative of Estimator
	Grad(distance float32, dir *V.Vec32) V.Vec32 //Gradient Vector given direction to center
}

//Guassian structure is a kernel: in literature the variable h holds the radial extent of the smoothed particle
//Powers of H will be held in an array
type GaussianKernel struct {
	H [5]float32
}

//Cubic Kernel is also known as a Spiky Kernel, Its 2nd Order Derivative is Linear // First order 2nd order poly
type CubicKernel struct {
	H [5]float32
}

func InitGaussian(radius float32) GaussianKernel {
	G := GaussianKernel{}
	G.H[0] = radius
	G.H[1] = radius * radius
	G.H[2] = radius * radius * radius
	G.H[3] = radius * radius * radius * radius
	G.H[4] = radius * radius * radius * radius * radius
	return G
}

func InitCubic(radius float32) CubicKernel {
	G := CubicKernel{}
	G.H[0] = radius
	G.H[1] = radius * radius
	G.H[2] = radius * radius * radius
	G.H[3] = radius * radius * radius * radius
	G.H[4] = radius * radius * radius * radius * radius

	return G
}

//--------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Guassian Kernel Operators
func (K *GaussianKernel) O1D(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	x := 1.0 - distance*distance/K.H[1]
	return -945.0 / (32.0 * PI * K.H[4]) * distance * x * x
}

func (K *GaussianKernel) O2D(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	x := distance * distance / K.H[1]
	return 945.0 / (32.0 * PI * K.H[4]) * (1 - x) * (3*x - 1)
}

func (K *GaussianKernel) Grad(distance float32, dir *V.Vec32) V.Vec32 {
	return V.Scale(*dir, -K.O1D(distance))
}

func (K *GaussianKernel) F(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	guassian := 315 / (64 * PI * K.H[2])
	x := 1.0 - (distance * distance / K.H[1])
	return guassian * x * x * x
}

//--------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Guassian Kernel Operators
func (K *CubicKernel) O1D(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	x := 1.0 - distance/K.H[0]
	return -45.0 / (PI * K.H[3]) * x * x
}

func (K *CubicKernel) O2D(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	x := 1.0 - distance/K.H[0]
	return 90.0 / (PI * K.H[4]) * x
}

func (K *CubicKernel) Grad(distance float32, dir *V.Vec32) V.Vec32 {
	return V.Scale(*dir, -K.O1D(distance))
}

func (K *CubicKernel) F(distance float32) float32 {
	if distance > K.H[0] {
		return 0.0
	}
	x := 1.0 - distance/K.H[0]
	return 15.0 / (PI * K.H[2]) * x * x * x
}
