package fluid

import (
	G "diesel.com/diesel/geometry"
	//	U "diesel.com/diesel/utils"
	V "diesel.com/diesel/vector"
	"fmt"
	"math"
)

//SPHFluid structure: Holds relevant fluid particles, spatial grid, meshes
//and animation context. Does not initialize the specific construction of the particle
//field to be studied.

//Will handle geometry export to Mistuba 3D Render and other formaTime.TS. IsoSurface
//Meshes will be exported to .obj file format.

//These particle positions will need to be passed to OpenGL Vertex Buffers
//Made accessible by GLFW and GOGL frameworks.
const MAX_PCI = 3 //Cubic Predictive Branch
const GRAV = -9.810435
const EOS_EXP = 1 //EOS Stiffness Parameter
const DENSITY_THREAD_RUN = 100
const DENSITY_THREAD_WAIT = 101

//SPHFluid - Is a PCISPH Fluid with Predictive Correction of Pressures
type SPHFluid struct {
	Sampler    *SPDSampler        //Sampler Interface
	Colliders  *G.Mesh            //Collider Triangle Meshes
	Mfp        *MassFluidParticle //Fluid Particle Descriptor
	ItrpKernel GaussianKernel     //Gaussian Kernel Typically
	GradKernel CubicKernel        //Cubic Kernel For Derivative Kernels
	Timer      Timer
	Count      int       //Count of particles
	Positions  []V.Vec32 //Particle pos
	Velocities []V.Vec32 //Particle vel
	Forces     []V.Vec32 //Particle
	Densities  []float32 //Densities
	Pressures  []float32 //Pressures
}

//MassFluidParticle - Fluid system particle properties extended to system
//mass is in kg / viscosity scalar coefficient / KernelRadius is the innerParticle
//boundary, ParticleRadius is utilized for surface reconstruction
type MassFluidParticle struct {
	Mass           float32
	Viscosity      float32
	KernelRadius   float32
	ParticleRadius float32
	TimeStep       float32
	SpeedSound     float32
	TargetDensity  float32
	EosExp         float32
}

type Timer struct {
	T        float64
	TS       float64
	TIMELAST float64
}

func (t *Timer) StepTime() {
	t.TIMELAST = t.T
	t.T = t.T + t.TS
}

//Fluid System Description - Used for boxed particle & collider mesh generation
type BoxFluidSystem struct {
	Origin      V.Vec32 //Box Origin //Typically Zero
	Width       float32 //Width Centered
	Height      float32 //Height Centered
	Depth       float32 //Depth Centered
	WidthCells  int     //Width cells (columns in width)
	HeightCells int     //Height cells (rows and column height)
	DepthCells  int     //Depth Cells (depth rows)
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//Initialize does heavy lifting of setting up the Grid Data and Computing Initial
//Particle Densities
func (fluid *SPHFluid) Initialize(init *BoxFluidSystem, mpf *MassFluidParticle, initialVelocity V.Vec32) {

	sampler := SPDSampler{}

	//Initialize Particles
	fluid.Count = init.WidthCells * init.HeightCells * init.DepthCells
	fluid.Mfp = mpf
	fluid.ItrpKernel = InitGaussian(mpf.KernelRadius)
	fluid.GradKernel = InitCubic(fluid.Mfp.KernelRadius)
	wStep := init.Width / float32(init.WidthCells)
	hStep := init.Height / float32(init.HeightCells)
	dStep := init.Depth / float32(init.DepthCells)
	minW := init.Origin[0] - (init.Width / 2)
	minH := init.Origin[1] - (init.Height / 2)
	minD := init.Origin[2] - (init.Depth / 2)

	//Initialize buffers //
	fluid.Positions = make([]V.Vec32, fluid.Count) //Must set positions
	fluid.Velocities = make([]V.Vec32, fluid.Count)
	fluid.Pressures = make([]float32, fluid.Count)
	fluid.Densities = make([]float32, fluid.Count)
	fluid.Forces = make([]V.Vec32, fluid.Count)

	for i := 0; i < init.WidthCells; i++ {
		for j := 0; j < init.HeightCells; j++ {
			for k := 0; k < init.DepthCells; k++ {
				if32 := float32(i)
				jf32 := float32(j)
				kf32 := float32(k)
				nPos := V.Vec32{float32(minW + wStep*if32), float32(minH + hStep*jf32), float32(minD + dStep*kf32)} //removed wStep , dSteh, hStep
				index := i*init.WidthCells*init.HeightCells + int(j*init.HeightCells) + int(k)

				if index%2 == 0 {
					nPos[0] = float32(minW + wStep/2 + (wStep)*if32)
					nPos[1] = float32(minH + hStep/2 + (hStep)*jf32)
					nPos[2] = float32(minD + dStep/2 + (dStep)*kf32)
				}

				fluid.Positions[index] = nPos
				fluid.Velocities[index] = initialVelocity
			}
		}
	} //End Particle Init

	fluid.Colliders = G.Box(init.Width, init.Height, init.Depth, init.Origin) //Initialize Collider Box
	//Initiate Sampler from particle data
	fluid.Sampler = sampler.SPHSampler(25, fluid.Positions)
	fluid.UpdateDensities() //Baseline density calculation
	//Time step dependent on propogation of particle collisions
	fluid.Timer.TS = float64((0.4 * fluid.Mfp.KernelRadius) / (fluid.Mfp.SpeedSound * fluid.Mfp.SpeedSound))

	//Update spherical fluid mass
	//fluid.Mfp.Mass = fluid.Mfp.Mass / ((4 / 3) * PI * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius)
}

//Updates Densities associated with each particle position with Gaussian Kernel
func (fluid *SPHFluid) UpdateDensities() {
	FIELD := fluid.Count

	//Compute Density Fields
	for i := 0; i < int(FIELD); i++ {
		//For Each Particle Calculate Kernel Based Summation

		samples := fluid.Sampler.GetSamples(fluid.Positions[i])
		mass := fluid.Mfp.Mass
		density := float32(0.0)
		for j := 0; j < len(samples); j++ {
			if samples[j] != i {
				dist := V.Length(V.Sub(fluid.Positions[i], fluid.Positions[samples[j]]))
				weight := float32(fluid.ItrpKernel.F(dist))
				density += (mass * weight)
			}
		}
		if !math.IsNaN(float64(density)) {
			fluid.Densities[i] = density
		}
	}
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) DensityGradient(i int) V.Vec32 {

	//For Each Particle Calculate Kernel Based Summation
	DensityGrad := V.Vec32{}
	samples := fluid.Sampler.GetSamples(fluid.Positions[i])
	//Spherical Density:
	mass := fluid.Mfp.Mass
	iDensity := fluid.Densities[i]

	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := fluid.Densities[samples[j]]
			dir := V.Sub(fluid.Positions[samples[j]], fluid.Positions[i])
			dist := V.Length(dir)
			dir = V.Scale(dir, 1/dist)
			grad := fluid.ItrpKernel.Grad(dist, &dir)
			estm := (mass / iDensity) + (mass / jDensity)
			DensityGrad.Add(V.Scale(grad, -estm))
		}
	}

	return DensityGrad
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Pressure(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.Sampler.GetSamples(fluid.Positions[i])
	mass := fluid.Mfp.Mass
	dens := fluid.Densities[i]
	msq := mass * mass
	F := float32(0.0)

	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := fluid.Densities[samples[j]]
			dir := V.Sub(fluid.Positions[samples[j]], fluid.Positions[i])
			dist := V.Length(dir)
			dir = V.Scale(dir, 1/dist) //Normalize
			grad := fluid.GradKernel.Grad(dist, &dir)
			F = (msq * ((fluid.Pressures[i] / dens * dens) + (fluid.Pressures[samples[j]] / jDensity * jDensity)))
			fluid.Forces[i].Add(V.Scale(grad, -F))
		}

	}

}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Viscosity(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.Sampler.GetSamples(fluid.Positions[i])
	mass := fluid.Mfp.Mass

	vi := fluid.Velocities[i]

	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			vj := fluid.Velocities[samples[j]]
			jDensity := fluid.Densities[samples[j]]

			//Check the Density
			if !math.IsNaN(float64(jDensity)) || jDensity != 0 {
				coeff_vec := V.Scale(V.Sub(vj, vi), 0.00091/jDensity)
				dist := fluid.Positions[i].Distance(fluid.Positions[samples[j]])
				lap := fluid.GradKernel.O2D(dist)
				force := V.Scale(coeff_vec, mass*mass*lap)
				fluid.Forces[i].Add(force)
			}
		}
	}

	return
}

//Updates particle system with accumalted External Force (I.E. Gravity)
func (fluid *SPHFluid) External(i int, f V.Vec32) {
	fluid.Forces[i].Add(f)
}

//Computes Pressure From A Given Equation of State which models incompressible flow
//Density vs State Density for Incompressible Fluid. Negative Pressure Scale usually can be
//set to 0. If there is a valid use for negative pressures (i.e. < target density) then add a Scaling
//factor to the negative pressure. typically < 1.0
func (fluid *SPHFluid) PressureEOS(i int, negativePressure float32) {
	//reference density : volumetric distribution of particle mass through
	r0 := fluid.Mfp.Mass
	eosExponent := fluid.Mfp.EosExp
	eosScale := (r0 * fluid.Mfp.SpeedSound * fluid.Mfp.SpeedSound) / eosExponent
	density := fluid.Densities[i]
	if density > 0 {
		p := (eosScale) * (float32(math.Pow(float64(density/r0-1.0), float64(eosExponent))))
		if p < 0 {
			p *= negativePressure //Negative Pressure Scaling
		}
		if !math.IsNaN(float64(p)) {
			fluid.Pressures[i] = p
		} else {
			fluid.Pressures[i] = 1.0
		}
	}
}

//Recurses collision calculation to deal with multi-collisions
//Initially called with e (exclusion face index set to -1). If this function
//Is not called this way then behavior is unspecified.
func (fluid *SPHFluid) Collide(index int, frames int, e int) {
	for frames <= 0 { //Multi-Boundary Projected Collision
		frames++
		normal, _, pos, collision, exclude := fluid.Colliders.Collision(fluid.Positions[index], fluid.Velocities[index], float32(fluid.Timer.TS), fluid.Mfp.ParticleRadius, e)
		if collision {
			v, f := fluid.CalcCollision(index, normal)
			fluid.Positions[index] = pos
			fluid.Forces[index] = f //keep external force constants?
			fluid.Velocities[index] = v
			fluid.Collide(index, frames, exclude)

		} else {
			return
		}
	}
	return
}

//Collision Calculations returns Velocity (Vec32), Force (Vec32) Momentum Vector
func (fluid *SPHFluid) CalcCollision(index int, norm V.Vec32) (V.Vec32, V.Vec32) {
	vel := fluid.Velocities[index]
	k_stiff := float32(0.45) //Restitution Coefficient. Further research req'd
	friction := float32(0.1)
	velN := V.Scale(norm, V.Dot(norm, vel))
	velTan := V.Sub(vel, velN)
	dtVN := V.Scale(velN, (-k_stiff - 1.0))
	velN = V.Scale(velN, -k_stiff)

	//Compute friction coefficients
	if V.Length(velTan) > 0.0 {
		fcomp := float64(1.0 - friction*V.Length(dtVN)/V.Length(velTan))
		frictionScale := float32(math.Max(fcomp, 0.0))
		velTan = V.Scale(velTan, frictionScale)
	}

	nVelocity := V.Add(velN, velTan)

	//Oppose particle Momentum Vector - THIS WORKED
	forceNormal := V.Scale(norm, V.Dot(norm, V.Scale(fluid.Velocities[index], fluid.Mfp.Mass)))

	return nVelocity, forceNormal
}

//Integrates the current particle forces and updates the velocity vector.
//Also updates the position of the particle. Clears all forces
//Utilizes MassFluidParticle description for Time.TS modifier.
func (fluid *SPHFluid) Update(index int) error {

	fluid.Forces[index].Scale(float32(fluid.Timer.TS) / fluid.Mfp.Mass)
	fluid.Velocities[index].Add(fluid.Forces[index])
	fluid.Positions[index].Add(fluid.Velocities[index])

	//Clear Particle Force State
	fluid.Forces[index] = V.Vec32{0.0, 0.0, 0.0}
	//	fluid.Densities[index] = fluid.Mfp.Mass

	return nil
}

//Main SPH fluid loop. Integrates all forces. Computes pressure from EOS and Resolves
//Collisions. Updates particle velocity and position then clears all forces
func (fluid *SPHFluid) Compute(threadStatus chan int, secondsAdvance float64) error {

	FLUID := fluid.Count
	GRAVITY := V.Vec32{0, fluid.Mfp.Mass * GRAV, 0}
	EXTERNAL := V.Vec32{}
	EXTERNAL.Add(GRAVITY)

	done := false

	fluid.Colliders.PrintNormals()

	for !done {

		for i := 0; i < FLUID; i++ {

			fluid.PressureEOS(i, 0.001) //Negative Pressure Scale 0
			fluid.Pressure(i)
			fluid.Viscosity(i)
			fluid.External(i, EXTERNAL)
			fluid.Collide(i, 0, -1) //Recursive Collider Function
			fluid.Update(i)

		}
		fluid.Timer.StepTime()

		threadStatus <- 20 //FLUID_THREAD_SYNCED SEND SIGNAL TO UPDATE
		fluid.Timer.T = 0.0
		fluid.Timer.TIMELAST = 0.0
		status := <-threadStatus //Block and Wait for Signal
		if status == 20 {        //FLUID THREAD RUN
			if math.IsNaN(float64(fluid.Positions[0][0])) {
				return fmt.Errorf("Fluid position diverged\nExiting")
			}
		}

	}
	return nil
}

//Asyncrohnously updates the densities
func (fluid *SPHFluid) ComputeDensities(densityStatus chan int) {
	done := false
	for !done {
		fluid.UpdateDensities()
		densityStatus <- DENSITY_THREAD_WAIT
		runStatus := <-densityStatus
		if runStatus != DENSITY_THREAD_RUN {
			done = true
		}
	}
}
