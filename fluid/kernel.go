package fluid

import V "diesel.com/diesel/vector"
import "math"

const PI = 3.141592653589
const PI2 = PI * 2
const SQRT2PI = 2.50662827463
const SQRPI = 5.5860525258
const A = (4*PI*PI - 30)
const A0 = (2*PI*PI - 15)

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
	H  float32
	H0 float32
	A  float32
}

//Cubic Kernel is also known as a Spiky Kernel, Its 2nd Order Derivative is Linear // First order 2nd order poly
type CubicKernel struct {
	H [5]float32
}

//Standard Cubic Kernel
type BSplineKernel struct {
	H  float32
	H0 float32
	A  float32
	W0 float32
}

//--------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Guassian Kernel Operators
func (K *GaussianKernel) O1D(distance float32) float32 {
	r := distance / K.H0
	if r > 3.0 {
		return 0.0
	}

	r0 := r + (r * 0.0001)
	d := r0 - r

	return (K.F(r0) - K.F(r)) / d
}

func (K *GaussianKernel) O2D(distance float32) float32 {
	r := distance / K.H0
	if r > 3.0 {
		return 0.0
	}

	r0 := r + (r * 0.0001)
	d := r0 - r

	return -(K.O1D(r0) - K.O1D(r)) / d
}

func (K *GaussianKernel) Grad(distance float32, dir *V.Vec32) V.Vec32 {
	return V.Scale(*dir, -K.O1D(distance))
}

func (K *GaussianKernel) F(distance float32) float32 {
	r := distance / K.H0
	if r > 3.0 {
		return 0.0
	}
	q := float32(math.Exp(float64(-r * r)))
	return q
}

//Adjusts smoothing length estimate based on density profile which should be
//The Density profile over the target density p/p0. This needs to be passed in
//Adjusts a Base smoothing Length - If you wish to iteratively reduce the smoothing Length
//Then Additionally modify Kernel.H with the returned value
func (K *GaussianKernel) Adjust(densityRatio float32) float32 {
	p := float64(densityRatio)
	k := 5.0
	b := 0.25

	h := k*math.Exp(-p) + b
	K.H0 = K.H

	return float32(h)
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

//**-------------------CUBIC KERNEL BSPLINE----------------------//
func (K *BSplineKernel) F(x float32) float32 {

	r := x / K.H0

	if r > 2.0 {
		return 0.0
	}

	s := (2 - r)
	p := (1 - r)

	ret := K.A * 0.25 * s * s * s

	if r < 1.0 {
		ret = K.A * ((0.25 * s * s * s) - (p * p * p))
	}

	return ret
}

func (K *BSplineKernel) GetW0() float32 {
	w := K.F(0)
	return w
}

//1st order Differential
func (K *BSplineKernel) O1D(x float32) float32 {
	//Try the functional derivative
	r := x / K.H0
	q := (2 - r)
	p := (1 - r)

	if r > 2.0 {
		return 0.0
	}

	if r < 1.0 {
		return K.A * ((0.75 * q * q) - 3*(p*p))
	} else {
		return K.A * 0.75 * (q * q)
	}

}

//2nd Order Differential
func (K *BSplineKernel) O2D(x float32) float32 {

	//Try the functional derivative
	r := x / K.H0
	q := (2 - r)
	p := (1 - r)

	if r > 2.0 {
		return 0.0
	}

	if r < 1.0 {
		return K.A * ((1.5 * q) - 6*p)
	} else {
		return K.A * 1.5 * q
	}

}

//Adjusts smoothing length estimate based on density profile which should be
//The Density profile over the target density p/p0. This needs to be passed in
//Adjusts a Base smoothing Length - If you wish to iteratively reduce the smoothing Length
//Then Additionally modify Kernel.H with the returned value
func (K *BSplineKernel) Adjust(densityRatio float32) float32 {

	K.H0 = K.H

	return densityRatio
}

//gradient
func (K *BSplineKernel) Grad(x float32, dir *V.Vec32) V.Vec32 {
	return V.Scale(*dir, -K.O1D(x))
}

////////////ALLOCATION FUNCTION //////////////

func InitGaussian(radius float32) GaussianKernel {
	G := GaussianKernel{}
	G.H = radius
	G.H0 = radius
	G.A = float32(1 / (SQRPI * G.H * G.H * G.H))
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

func AllocBSplineKernel(h float32) BSplineKernel {
	bSpline := BSplineKernel{h, h, 0, 0}
	bSpline.A = 1 / (PI * h * h * h)
	bSpline.W0 = bSpline.GetW0()
	return bSpline
}
