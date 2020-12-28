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
	Sampler            *VoxelArray        //Sampler Interface
	Colliders          *G.Mesh            //Collider Triangle Meshes
	Mfp                *MassFluidParticle //Fluid Particle Descriptor
	ItrpKernel         BSplineKernel      //Gaussian Kernel Typically
	GradKernel         BSplineKernel      //Cubic Kernel For Derivative Kernels
	Timer              Timer
	Count              int       //Count of particles
	Positions          []V.Vec32 //Particle pos
	Velocities         []V.Vec32 //Particle vel
	Forces             []V.Vec32 //Particle
	Densities          []float32 //Densities
	Pressures          []float32 //Pressures
	TempPositions      []V.Vec32 //Temp Positions
	TempVelocities     []V.Vec32 //Temp Velocities
	TempPressureForces []V.Vec32 //Temp Pressure Force
	Max_Force          float32   //Maximal Force Calc'd
	Max_Vel            float32   //Max Vel
	SampleAllNeighbors bool      //Full Sampling Method
	MaxDensityError    float32   //PCI Max Density Error
	MaxPCIIterations   int       //Max Iterations
	PCIDelta           float32   //PCI Delta
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

func AllocMassFluidParticle(Lattice BoxFluidSystem, Scale float32) MassFluidParticle {
	f := MassFluidParticle{1.0, 0.00091, 0.0, 0, 1480, 0.0, 1.0}
	f.Density = 1000.0
	f.ParticleRadius = ((Lattice.Depth * Scale) / float32(Lattice.DepthCells)) //For Diameter	f.Mass = f.GetVolume() * f.Density
	f.KernelRadius = 1.33 * f.ParticleRadius
	return f
}

func (m *MassFluidParticle) GetVolume() float32 {
	diam := 2 * m.ParticleRadius
	return diam * diam * diam
}

func (m *MassFluidParticle) GetKernelVolume() float32 {
	diam := 2 * 2 * m.KernelRadius
	return diam * diam * diam
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
func (t *Timer) UpdateTimeDelta(fluid *SPHFluid, h float32, forceRelative bool) {

	tv := float64(50*h) / float64(fluid.Mfp.SpeedSound)
	min := tv
	tf := 0.0
	//Max Acceleration SQRT
	if fluid.Max_Force != 0 && forceRelative {
		tf = 0.4 * float64(h) / float64(fluid.Max_Vel)

		if tf < tv {
			min = tf
		}
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

	minW := init.Origin[0] - (init.Width / 2)
	minH := init.Origin[1] - (init.Height / 2)
	minD := init.Origin[2] - (init.Depth / 2)
	fluid.SampleAllNeighbors = false
	pRadius := fluid.Mfp.KernelRadius * 2.0
	//Initialize buffers //
	fluid.Positions = make([]V.Vec32, fluid.Count) //Must set positions
	fluid.Velocities = make([]V.Vec32, fluid.Count)
	fluid.Pressures = make([]float32, fluid.Count)
	fluid.Densities = make([]float32, fluid.Count)
	fluid.Forces = make([]V.Vec32, fluid.Count)
	fluid.TempPositions = make([]V.Vec32, fluid.Count)
	fluid.TempVelocities = make([]V.Vec32, fluid.Count)
	fluid.TempPressureForces = make([]V.Vec32, fluid.Count)

	for i := 0; i < init.WidthCells; i++ {
		for j := 0; j < init.HeightCells; j++ {
			for k := 0; k < init.DepthCells; k++ {
				if32 := float32(i)
				jf32 := float32(j)
				kf32 := float32(k)
				nPos := V.Vec32{float32(minW + pRadius*if32), float32(minH + pRadius*jf32), float32(minD + pRadius*kf32)} //removed wStep , dSteh, hStep
				index := i*init.WidthCells*init.HeightCells + int(j*init.HeightCells) + int(k)

				fluid.Positions[index] = nPos
				fluid.Velocities[index] = initialVelocity
				fluid.Densities[index] = fluid.Mfp.Density
			}
		}
	} //End Particle Init

	fluid.Colliders = G.Box(init.Width, init.Height, init.Depth, init.Origin) //Initialize Collider Box
	//Initiate Sampler from particle data
	fluid.Sampler = AllocateVoxelStorage(fluid.Positions, 10, init.Width/2)
	fluid.Sampler.Samples = int(math.Cbrt(float64(fluid.Count)) * 4.0)
	fluid.Sampler.Update()
	fluid.UpdateDensities()
	fluid.Density0() //Computes Target Density Based on Initial Confguration Conditions

	//Scale Positions Slightly Out to Relax Initial Configuration
	/*
		for i := 0; i < fluid.Count; i++ {
			Trans0 := V.Add(fluid.Positions[i], V.Scale(init.Origin, -1))
			Scale1 := V.Scale(Trans0, 1+(4*fluid.Mfp.ParticleRadius))
			nPos := V.Add(Scale1, init.Origin)
			fluid.Positions[i] = nPos
		}
	*/

	//Baseline density calculation
	//Time step dependent on propogation of particle collisions
	tv := (fluid.Mfp.KernelRadius) / (fluid.Mfp.SpeedSound)
	//Ignore FMAX Per Time Step Adjustment SQRT(h*m / FMAX) * 0.25 ; take MIN - Initial Step Small (initial shock)
	fluid.Timer.TS = float64(tv)
	fluid.MaxDensityError = 0.01
	fluid.MaxPCIIterations = 3
	//	fluid.Sampler.PrintStorageRequirements()

	//Update spherical fluid mass -- pack consisten fluid metric to space -> Mapping Density Transformation Scale
	//fluid.Mfp.Mass = fluid.Mfp.Mass / ((4 / 3) * PI * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius * fluid.Mfp.ParticleRadius)
}

//Initialize densities to match sampling for initial grid
func (fluid *SPHFluid) Density0() float32 {
	avgDensity := fluid.Mfp.GetKernelVolume() * fluid.ItrpKernel.W0

	for i := 0; i < len(fluid.Densities); i++ {
		avgDensity = avgDensity + fluid.Densities[i]
	}
	avgDensity = avgDensity / float32(fluid.Count+1)
	fluid.PCIDelta = avgDensity
	return avgDensity
}

func (fluid *SPHFluid) GetSamples(i int) []int {
	if fluid.SampleAllNeighbors {
		return fluid.Sampler.GetAllNeighbors(i)
	} else {
		return fluid.Sampler.GetSamples(i)
	}
}

func (fluid *SPHFluid) ClearPCIPressure() {
	for i := 0; i < fluid.Count; i++ {
		fluid.TempPressureForces[i] = V.Vec32{}
	}
}

//PCI SPH -- Pressure Accumulation
func (fluid *SPHFluid) AccumulatePressureForce() {
	r0 := fluid.Mfp.GetKernelVolume() * fluid.Mfp.Density
	fluid.ClearPCIPressure()
	var ds []float32
	var densityErrors []float32
	ds = make([]float32, fluid.Count)
	densityErrors = make([]float32, fluid.Count)
	delta := fluid.PCIDelta
	maxDensityError := float32(0.0)

	for k := 0; k < fluid.MaxPCIIterations; k++ {
		//Predict Velocity and POSITION

		maxDensityError = 0.0
		for i := 0; i < fluid.Count; i++ {
			nVel := V.Scale(V.Add(fluid.Forces[i], fluid.TempPressureForces[i]), float32(fluid.Timer.TS)/fluid.Mfp.Mass)
			fluid.TempVelocities[i] = V.Add(fluid.Velocities[i], nVel)
			fluid.TempPositions[i] = V.Add(fluid.Positions[i], V.Scale(fluid.TempVelocities[i], float32(fluid.Timer.TS)))
		}
		//Resolve collisions
		for i := 0; i < fluid.Count; i++ {
			fluid.CollideTemp(i)
		}
		//Computer Pressure from density Error
		for i := 0; i < fluid.Count; i++ {
			samples := fluid.GetSamples(i)
			w := fluid.ItrpKernel.GetW0()

			for j := 0; j < len(samples); j++ {
				dist := V.Length(V.Sub(fluid.TempPositions[i], fluid.TempPositions[j]))
				w = w + fluid.ItrpKernel.F(dist)
			}

			density := fluid.Mfp.GetVolume() * w * fluid.Mfp.Density
			densityError := (density - r0)
			if densityError > maxDensityError {
				maxDensityError = densityError
			}
			pressure := delta * densityError

			/*
				if pressure < 0.0 {
					pressure = 0.0
					densityError = 0.0
				}

			*/

			fluid.Pressures[i] = fluid.Pressures[i] + pressure
			ds[i] = density
			densityErrors[i] = densityError

		}

		densityErrorRatio := maxDensityError / r0

		if densityErrorRatio < 0.0 {
			densityErrorRatio = -densityErrorRatio
		}

		if densityErrorRatio < fluid.MaxDensityError {
			return
		}

		//Compute Pressure Gradient force
		for i := 0; i < fluid.Count; i++ {
			fluid.accumulatePressureForce(i, ds, fluid.TempPositions, fluid.Pressures, fluid.TempPressureForces)
		}
	}

	//Accumulate Pressure Forces
	for i := 0; i < fluid.Count; i++ {
		fluid.Forces[i] = V.Add(fluid.Forces[i], fluid.TempPressureForces[i])
	}
}

//Standard Pressure Accumulation
func (fluid *SPHFluid) accumulatePressureForce(i int, densities []float32, positions []V.Vec32, pressures []float32, pressureForces []V.Vec32) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.GetSamples(i)
	dens := densities[i]
	F := float32(0.0)
	fluid.GradKernel.Adjust(dens / fluid.Mfp.Density)
	mq2 := -fluid.Mfp.Mass * fluid.Mfp.Mass
	accumGrad := V.Vec32{}
	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := densities[samples[j]]
			dir := V.Sub(positions[samples[j]], positions[i])
			dist := V.Length(dir)
			dir = V.Scale(dir, 1/dist) //Normalize
			grad := fluid.GradKernel.Grad(dist, &dir)

			if dist > 0 {
				F = ((pressures[i] / (dens * dens)) + (pressures[samples[j]] / (jDensity * jDensity)))
				accumGrad = V.Add(accumGrad, V.Scale(grad, -F))
			}
		}
	}

	pressureForces[i].Add(V.Scale(accumGrad, mq2))
	ForceMag := V.Length(pressureForces[i])
	if ForceMag > fluid.Max_Force {
		fluid.Max_Force = ForceMag
	}

}

//Updates Densities associated with each particle position with Gaussian Kernel
func (fluid *SPHFluid) UpdateDensities() {
	FIELD := fluid.Count

	//Compute Density Fields
	for i := 0; i < int(FIELD); i++ {
		//For Each Particle Calculate Kernel Based Summation

		samples := fluid.GetSamples(i)
		fluid.ItrpKernel.Adjust(fluid.Densities[i] / fluid.Mfp.Density)
		sum := fluid.ItrpKernel.GetW0()
		for j := 0; j < len(samples); j++ {
			if samples[j] != i {
				dist := V.Length(V.Sub(fluid.Positions[i], fluid.Positions[samples[j]]))
				if dist > 0 {
					sum += fluid.ItrpKernel.F(dist)
				}
			}
		}
		fluid.Densities[i] = fluid.Mfp.Mass * sum
	}
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) DensityGradient(i int) V.Vec32 {

	//For Each Particle Calculate Kernel Based Summation
	DensityGrad := V.Vec32{}
	samples := fluid.GetSamples(i)
	//Spherical Density:
	mass := fluid.Mfp.Mass
	iDensity := fluid.Densities[i]
	//	fluid.GradKernel.Adjust(iDensity / fluid.Mfp.Density)
	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			jDensity := fluid.Densities[samples[j]]
			dir := V.Sub(fluid.Positions[samples[j]], fluid.Positions[i])

			dist := V.Length(dir)

			if dist > 0 {
				dir = V.Scale(dir, 1/dist)
				grad := fluid.GradKernel.Grad(dist, &dir)
				estm := (mass / iDensity) + (mass / jDensity)
				DensityGrad.Add(V.Scale(grad, -estm))
			}
		}
	}

	return DensityGrad
}

/* Gives weighted interpolation for quantity based on position and particle selection */
func (fluid *SPHFluid) Interpolate(position V.Vec32, value_type int) float32 {
	samples := fluid.Sampler.GetSampleVoxels(position, 0)
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

	samples := fluid.GetSamples(i)
	dens := fluid.Densities[i]
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

			if dist > 0 {
				F = fluid.Mfp.Mass * ((fluid.Pressures[i] / (dens * dens)) + (fluid.Pressures[samples[j]] / (jDensity * jDensity)))
				if !math.IsNaN(float64(F)) {
					fluid.Forces[i].Add(V.Scale(grad, -F))
					if F > fluid.Max_Force {
						fluid.Max_Force = F
					}
				}
			}
		}
	}

}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Viscosity(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples := fluid.GetSamples(i)

	vi := fluid.Velocities[i]
	fluid.GradKernel.Adjust(fluid.Densities[i] / fluid.Mfp.Density)

	//Compute Viscosity
	for j := 0; j < len(samples); j++ {
		jIndex := samples[j]
		if jIndex != i {
			vj := fluid.Velocities[samples[j]]
			jDensity := fluid.Densities[samples[j]]
			coeff := float32(1.0)
			dist := V.Length(V.Sub(fluid.Positions[j], fluid.Positions[i]))

			lap := fluid.GradKernel.O2D(dist)
			vec := V.Scale(V.Sub(vj, vi), 1/jDensity)
			force := V.Scale(vec, fluid.Mfp.Mass*coeff*lap)
			if !math.IsNaN(float64(V.Length(force))) {
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
	r0 := fluid.Mfp.Density
	stiffness := float32(1.0) // Enforces the time step
	density := fluid.Densities[i]

	if density < r0 {
		density = r0
		fluid.Pressures[i] = 0.0
	} else {
		//Compute EOS
		//q := density / r0
		p := stiffness * (density - r0) //(float32(math.Pow(float64(q), float64(fluid.Mfp.EosExp))) - 1.0)
		if p < 0 {
			p *= negativePressure //Negative Pressure Scaling
		}
		if !math.IsNaN(float64(p)) {
			fluid.Pressures[i] = p
		}
	}

}

func (fluid *SPHFluid) CollideTemp(index int) {
	normal, _, p0, collision := fluid.Colliders.Collision(fluid.TempPositions[index], fluid.TempVelocities[index], float32(fluid.Timer.TS), fluid.Mfp.ParticleRadius)
	if collision {
		v, _ := fluid.CalcCollision(index, normal)
		//	fluid.Positions[index] = pos
		fluid.TempVelocities[index] = v
		fluid.TempPositions[index] = p0
	}
	return
}

//Recurses collision calculation to deal with multi-collisions
//Initially called with e (exclusion face index set to -1). If this function
//Is not called this way then behavior is unspecified.
func (fluid *SPHFluid) Collide(index int) {
	normal, _, p0, collision := fluid.Colliders.Collision(fluid.Positions[index], fluid.Velocities[index], float32(fluid.Timer.TS), fluid.Mfp.ParticleRadius)
	if collision {
		v, _ := fluid.CalcCollision(index, normal)
		//	fluid.Positions[index] = pos
		//fluid.Forces[index] = f //keep external force constants?
		fluid.Velocities[index] = v
		fluid.Positions[index] = p0
	}
	return
}

//Collision Calculations returns Velocity (Vec32), Force (Vec32) Momentum Vector
func (fluid *SPHFluid) CalcCollision(index int, norm V.Vec32) (V.Vec32, V.Vec32) {
	vel := fluid.Velocities[index]
	k_stiff := float32(-0.25) //Restitution Coefficient. Further research req'd
	friction := float32(0.01)
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
	forceNormal := V.Scale(velN, -fluid.Mfp.Mass)
	fluid.Forces[index] = forceNormal

	return nVelocity, forceNormal
}

//Integrates the current particle forces and updates the velocity vector.
//Also updates the position of the particle. Clears all forces
//Utilizes MassFluidParticle description for Time.TS modifier.
func (fluid *SPHFluid) Update(index int) error {

	if math.IsNaN(float64(V.Length(fluid.Forces[index]))) {
		return fmt.Errorf("Force NaN Divergence\n")
	}

	fluid.Forces[index].Scale(float32(fluid.Timer.TS) / fluid.Mfp.Mass)
	fluid.Velocities[index].Add(fluid.Forces[index]) //Normalize for varying delta velocity
	if !math.IsNaN(float64(V.Length(fluid.Velocities[index]))) {
		fluid.Positions[index].Add(V.Scale(fluid.Velocities[index], float32(fluid.Timer.TS)))
	} else {
		//	fluid.Velocities[index].Scale(0.0)
	}

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
	GRAVITY := V.Vec32{0, fluid.Mfp.Mass * GRAV, 0}
	EXTERNAL := V.Vec32{}
	EXTERNAL.Add(GRAVITY)

	done := false

	for !done {

		//Update All Densities
		fluid.UpdateDensities()

		//Compute Pressure field from Density
		for i := 0; i < FLUID; i++ {
			fluid.PressureEOS(i, 0.1) //Negative Pressure Scale 0
		}

		//fluid.AccumulatePressureForce() //PCI SPH Method

		for i := 0; i < FLUID; i++ {
			fluid.accumulatePressureForce(i, fluid.Densities, fluid.Positions, fluid.Pressures, fluid.Forces)
		}

		//Compute Particle Force Gradients and Add External Forces
		for i := 0; i < FLUID; i++ {
			//	fluid.Pressure(i)
			fluid.Viscosity(i)
			fluid.External(i, GRAVITY)
			fluid.Collide(i) //Recursive Collider Function
			fluid.Update(i)
		}
		//Step Timer and Update Timer Calculation based on FMAX
		fluid.Timer.StepTime()
		fluid.Timer.UpdateTimeDelta(fluid, fluid.Mfp.KernelRadius, true)
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
