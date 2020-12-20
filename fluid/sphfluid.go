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

//Scalar Particle Interp Values - Add More if Needed
const DENS = 1
const PRESS = 2

//SPHFluid - Is a PCISPH Fluid with Predictive Correction of Pressures
type SPHFluid struct {
	Sampler    *VoxelArray        //Sampler Interface
	Colliders  *G.Mesh            //Collider Triangle Meshes
	Mfp        *MassFluidParticle //Fluid Particle Descriptor
	ItrpKernel BSplineKernel      //Gaussian Kernel Typically
	GradKernel BSplineKernel      //Cubic Kernel For Derivative Kernels
	Timer      Timer
	Count      int       //Count of particles
	Positions  []V.Vec32 //Particle pos
	Velocities []V.Vec32 //Particle vel
	Forces     []V.Vec32 //Particle
	Densities  []float32 //Densities
	Pressures  []float32 //Pressures
	Max_Force  float32   //Maximal Force Calc'd
	Max_Vel    float32   //Max Vel
}

//MassFluidParticle - Fluid system particle properties extended to system
//mass is in kg / viscosity scalar coefficient / KernelRadius is the innerParticle
//boundary, ParticleRadius is utilized for surface reconstruction
type MassFluidParticle struct {
	Mass           float32 // Mass Kg/m^3 (Target Density)
	Viscosity      float32
	KernelRadius   float32
	ParticleRadius float32
	SpeedSound     float32
	Density        float32
	EosExp         float32
}

func AllocMassFluidParticle(Density float32, ParticleRad float32, KernelRad float32) MassFluidParticle {
	f := MassFluidParticle{0, 0.00091, KernelRad, ParticleRad, 1480, Density, 7.0}
	f.Mass = (f.ParticleRadius * f.ParticleRadius * f.ParticleRadius) * f.Density
	return f
}

func (f *MassFluidParticle) SmoothedParticleMass() float32 {
	return f.Mass
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

//Updates Time Step Parameter Based on Fluid Max Force Calc
func (t *Timer) UpdateTimeDelta(fluid *SPHFluid, h float32) {
	tv := float64((0.4 * h) / (fluid.Mfp.SpeedSound))

	tf := 0.0
	if fluid.Max_Vel != 0 {
		tf = float64((h / (fluid.Max_Force)))
	}
	min := tv
	if tf < tv {
		min = tf
	}
	t.TS = min
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

func AllocBoxFluid(size float32, cells int) BoxFluidSystem {
	box := BoxFluidSystem{V.Vec32{0, 0, 0}, size, size, size, cells, cells, cells}
	return box
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//Initialize does heavy lifting of setting up the Grid Data and Computing Initial
//Particle Densities
func (fluid *SPHFluid) Initialize(init *BoxFluidSystem, mpf *MassFluidParticle, initialVelocity V.Vec32) {

	//Initialize Particles
	fluid.Count = init.WidthCells * init.HeightCells * init.DepthCells
	fluid.Mfp = mpf
	fluid.ItrpKernel = AllocBSplineKernel(fluid.Mfp.KernelRadius)
	fluid.GradKernel = AllocBSplineKernel(fluid.Mfp.KernelRadius)
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
	fluid.Sampler = AllocateVoxelStorage(fluid.Positions, 8, len(fluid.Positions), init.Width)
	fluid.Sampler.Update()
	fluid.UpdateDensities() //Baseline density calculation
	//Time step dependent on propogation of particle collisions
	tv := (fluid.Mfp.KernelRadius * 0.4) / (fluid.Mfp.SpeedSound * fluid.Mfp.SpeedSound)
	//Ignore FMAX Per Time Step Adjustment SQRT(h*m / FMAX) * 0.25 ; take MIN - Initial Step Small (initial shock)
	fluid.Timer.TS = float64(tv)
	fluid.Sampler.PrintStorageRequirements()

	//Update spherical fluid mass -- pack consisten fluid metric to space -> Mapping Density Transformation Scale
	//fluid.Mfp.Mass = fluid.Mfp.Mass / ((4 / 3) * PI * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius)
}

//Updates Densities associated with each particle position with Gaussian Kernel
func (fluid *SPHFluid) UpdateDensities() {
	FIELD := fluid.Count
	sphMass := fluid.Mfp.SmoothedParticleMass()

	//Compute Density Fields

	for i := 0; i < int(FIELD); i++ {
		//For Each Particle Calculate Kernel Based Summation

		samples := fluid.Sampler.GetSamples(i)
		mass := sphMass
		density := float32(0.0)
		fluid.ItrpKernel.Adjust(fluid.Densities[i] / fluid.Mfp.Density)
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
	samples := fluid.Sampler.GetSamples(i)
	//Spherical Density:
	mass := fluid.Mfp.Mass
	iDensity := fluid.Densities[i]
	fluid.GradKernel.Adjust(iDensity / fluid.Mfp.Density)
	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := fluid.Densities[samples[j]]
			dir := V.Sub(fluid.Positions[samples[j]], fluid.Positions[i])
			dist := V.Length(dir)
			dir = V.Scale(dir, 1/dist)
			grad := fluid.GradKernel.Grad(dist, &dir)
			estm := (mass / iDensity) + (mass / jDensity)
			DensityGrad.Add(V.Scale(grad, -estm))
		}
	}

	return DensityGrad
}

/* Gives weighted interpolation for quantity based on position and particle selection */
func (fluid *SPHFluid) Interpolate(position V.Vec32, value_type int) float32 {
	vIdx := fluid.Sampler.VoxelHash(position)
	samples := fluid.Sampler.GetSampleVoxels(vIdx)
	mass := fluid.Mfp.SmoothedParticleMass()
	interp := float32(0.0)

	for j := 0; j < len(samples); j++ {
		jDensity := fluid.Densities[samples[j]]
		dir := V.Sub(fluid.Positions[samples[j]], position)
		dist := V.Length(dir)
		if value_type == DENS {
			interp = interp + mass*fluid.ItrpKernel.F(dist)
		}
		if value_type == PRESS {
			value := fluid.Pressures[samples[j]]
			interp = interp + mass*(value/jDensity)*fluid.ItrpKernel.F(dist)
		}
		//Add other VALUES to interpolate here

	}

	return interp

}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Pressure(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.Sampler.GetSamples(i)
	mass := fluid.Mfp.SmoothedParticleMass()
	dens := fluid.Densities[i]
	msq := mass * mass
	F := float32(0.0)
	fluid.GradKernel.Adjust(dens / fluid.Mfp.Density)
	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := fluid.Densities[samples[j]]
			dir := V.Sub(fluid.Positions[samples[j]], fluid.Positions[i])
			dist := V.Length(dir)
			dir = V.Scale(dir, 1/dist) //Normalize
			grad := fluid.GradKernel.Grad(dist, &dir)
			F = (msq * ((fluid.Pressures[i] / dens * dens) + (fluid.Pressures[samples[j]] / jDensity * jDensity)))
			fluid.Forces[i].Add(V.Scale(grad, F))
			//Calculate Main Force Max

			FAbs := Abs(F)
			if FAbs > fluid.Max_Force {
				fluid.Max_Force = FAbs
			}

		}

	}

}

//Holy fuck go
func Abs(x float32) float32 {
	if x < 0 {
		x = -x
	}
	return x
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Viscosity(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.Sampler.GetSamples(i)
	mass := fluid.Mfp.SmoothedParticleMass()

	vi := fluid.Velocities[i]
	fluid.GradKernel.Adjust(fluid.Densities[i] / fluid.Mfp.Density)
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
	r0 := fluid.Mfp.Density * (fluid.Mfp.KernelRadius * fluid.Mfp.KernelRadius * fluid.Mfp.KernelRadius)
	eosExponent := fluid.Mfp.EosExp
	eosScale := (r0) / eosExponent
	density := fluid.Densities[i]
	if density > 0 {
		p := ((eosScale) / eosExponent) * float32(math.Pow(float64(density/r0-1.0), float64(eosExponent)))
		if p < 0 {
			p *= negativePressure //Negative Pressure Scaling
		}
		if !math.IsNaN(float64(p)) {
			fluid.Pressures[i] = p
		} else {
			fluid.Pressures[i] = 0.0
		}
	}

}

//Recurses collision calculation to deal with multi-collisions
//Initially called with e (exclusion face index set to -1). If this function
//Is not called this way then behavior is unspecified.
func (fluid *SPHFluid) Collide(index int) {
	normal, _, _, collision := fluid.Colliders.Collision(fluid.Positions[index], fluid.Velocities[index], float32(fluid.Timer.TS), fluid.Mfp.ParticleRadius)
	if collision {
		v, f := fluid.CalcCollision(index, normal)
		//	fluid.Positions[index] = pos
		fluid.Forces[index] = f //keep external force constants?
		fluid.Velocities[index] = v
	}
	return
}

//Collision Calculations returns Velocity (Vec32), Force (Vec32) Momentum Vector
func (fluid *SPHFluid) CalcCollision(index int, norm V.Vec32) (V.Vec32, V.Vec32) {
	vel := fluid.Velocities[index]
	k_stiff := float32(-0.2) //Restitution Coefficient. Further research req'd
	friction := float32(0.000001)
	velN := V.Scale(norm, V.Dot(norm, vel))
	velTan := V.Sub(vel, velN)
	dtVN := V.Scale(velN, (k_stiff - 1.0))
	velN = V.Scale(velN, k_stiff)

	//Compute friction coefficients
	if V.Length(velTan) > 0.0 {
		fcomp := float64(1.0 - friction*V.Length(dtVN)/V.Length(velTan))
		frictionScale := float32(math.Max(fcomp, 0.0))
		velTan = V.Scale(velTan, frictionScale)
	}

	nVelocity := V.Add(velN, velTan)

	//Oppose particle Momentum Vector - THIS WORKED
	forceNormal := V.Scale(velN, fluid.Mfp.Mass)

	return nVelocity, forceNormal
}

//Integrates the current particle forces and updates the velocity vector.
//Also updates the position of the particle. Clears all forces
//Utilizes MassFluidParticle description for Time.TS modifier.
func (fluid *SPHFluid) Update(index int) error {

	fluid.Forces[index].Scale(float32(fluid.Timer.TS) / fluid.Mfp.SmoothedParticleMass())
	fluid.Velocities[index].Add(fluid.Forces[index])
	fluid.Positions[index].Add(fluid.Velocities[index])

	if V.Length(fluid.Velocities[index]) > fluid.Max_Vel {
		fluid.Max_Vel = V.Length(fluid.Velocities[index])
	}

	if V.Length(fluid.Forces[index]) > fluid.Max_Force {
		fluid.Max_Force = V.Length(fluid.Forces[index])
	}
	//Clear Particle Force State
	fluid.Forces[index] = V.Vec32{0.0, 0.0, 0.0}
	//	fluid.Densities[index] = fluid.Mfp.Mass

	return nil
}

//Main SPH fluid loop. Integrates all forces. Computes pressure from EOS and Resolves
//Collisions. Updates particle velocity and position then clears all forces
func (fluid *SPHFluid) Compute(threadStatus chan int, secondsAdvance float64) error {

	FLUID := fluid.Count
	GRAVITY := V.Vec32{0, fluid.Mfp.SmoothedParticleMass() * GRAV, 0}
	EXTERNAL := V.Vec32{}
	EXTERNAL.Add(GRAVITY)

	done := false

	for !done {

		for i := 0; i < FLUID; i++ {
			fluid.PressureEOS(i, 0.1) //Negative Pressure Scale 0
			fluid.Pressure(i)
			fluid.Viscosity(i)
			fluid.External(i, EXTERNAL)
			fluid.Collide(i) //Recursive Collider Function
			fluid.Update(i)

		}
		//Step Timer and Update Timer Calculation based on FMAX
		fluid.Timer.StepTime()
		fluid.Timer.UpdateTimeDelta(fluid, fluid.Mfp.KernelRadius)
		fluid.Max_Force = 0
		fluid.Max_Vel = 0

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
