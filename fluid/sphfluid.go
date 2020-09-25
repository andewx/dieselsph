package fluid

import (
	G "diesel.com/diesel/geometry"
	V "diesel.com/diesel/vector"
	Math "math"
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

//SPHFluid - Is a PCISPH Fluid with Predictive Correction of Pressures
type SPHFluid struct {
	SPHGrid    *SpatialHashGrid   //Spatial Hash Grid For Neighbor Particles
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
//mass is in kg / viscosity scalar coefficient / innerRadius is the innerParticle
//boundary, outerRadius is utilized for surface reconstruction
type MassFluidParticle struct {
	Mass          float32
	Viscosity     float32
	InnerRadius   float32
	OuterRadius   float32
	TimeStep      float32
	SpeedSound    float32
	TargetDensity float32
	EosExp        float32
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
func (fluid *SPHFluid) Initialize(init *BoxFluidSystem, mpf *MassFluidParticle) {

	//Initialize Particles
	fluid.Count = init.WidthCells * init.HeightCells * init.DepthCells
	fluid.Mfp = mpf
	fluid.ItrpKernel = InitGaussian(mpf.InnerRadius)
	fluid.GradKernel = InitCubic(fluid.Mfp.InnerRadius)
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

	//Spatial Acceleration Grid -- We're having an issue with the spatial hashing modulus due to rounding modulus error
	fluid.SPHGrid = AllocateGridUserDefined(init.Width, 7) //Constructs a cubic grid the 7 constant needs to be change

	//Initialize Particle Positions and Stuff
	for i := 0; i < init.WidthCells; i++ {
		for j := 0; j < init.HeightCells; j++ {
			for k := 0; k < init.DepthCells; k++ {
				if32 := float32(i)
				jf32 := float32(j)
				kf32 := float32(k)
				nPos := V.Vec32{float32(minW + wStep*if32), float32(minH + hStep*jf32), float32(minD + dStep*kf32)} //removed wStep , dSteh, hStep
				index := i*init.WidthCells*init.HeightCells + int(j*init.HeightCells) + int(k)
				fluid.Positions[index] = nPos
			}
		}
	} //End Particle Init

	fluid.Colliders = G.Box(init.Width, init.Height, init.Depth, init.Origin) //Initialize Collider Box

	//Allocates Particles to Spatial Hash Grid
	fluid.SPHGrid.Load(fluid.Positions)
	//Create Collider Mesh Box From List of triangles (12)
	fluid.UpdateDensities()
	//Time step dependent on propogation of particle collisions
	fluid.Timer.TS = 0.01 //(fluid.Mfp.InnerRadius * 0.4) / (fluid.Mfp.SpeedSound) //Time Step Per Iteration
}

//Updates Densities associated with each particle position with Gaussian Kernel
func (fluid *SPHFluid) UpdateDensities() {
	FIELD := fluid.Count
	//Compute Density Fieldsa
	for i := 0; i < int(FIELD); i++ {
		//For Each Particle Calculate Kernel Based Summation

		samples, nCollide, _ := fluid.SPHGrid.GetSamples(&fluid.Positions[i])

		mass := fluid.Mfp.Mass
		density := mass
		for j := 0; j < nCollide; j++ {
			dist := V.Length(V.Sub(fluid.Positions[i], fluid.Positions[samples[j].Index]))
			density += mass * fluid.ItrpKernel.F(dist)
		}

		fluid.Densities[i] = density
	}
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) DensityGradient(i int) V.Vec32 {

	//For Each Particle Calculate Kernel Based Summation
	DensityGrad := V.Vec32{}
	samples, nCount, _ := fluid.SPHGrid.GetSamples(&fluid.Positions[i])
	mass := fluid.Mfp.Mass
	iDensity := fluid.Densities[i]

	for j := 0; j < nCount; j++ {
		jDensity := fluid.Densities[samples[j].Index]
		dir := V.Sub(fluid.Positions[samples[j].Index], fluid.Positions[i])
		dist := V.Length(dir)
		dir = V.Scale(dir, 1/dist)
		grad := fluid.GradKernel.Grad(dist, &dir)
		estm := (mass / iDensity) + (mass / jDensity)
		DensityGrad.Add(*grad.Scale(estm)) //Mutation
	}

	return DensityGrad
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Pressure(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples, nCount, _ := fluid.SPHGrid.GetSamples(&fluid.Positions[i])
	mass := fluid.Mfp.Mass
	dens := fluid.Densities[i]
	msq := mass * mass
	F := float32(0.0)

	for j := 0; j < nCount; j++ {

		jDensity := fluid.Densities[samples[j].Index]
		dir := V.Sub(fluid.Positions[samples[j].Index], fluid.Positions[i])
		dist := V.Length(dir)
		dir = V.Scale(dir, 1/dist)
		grad := fluid.GradKernel.Grad(dist, &dir)
		F = F - (msq * ((fluid.Pressures[i] / dens * dens) + (fluid.Pressures[samples[j].Index] / jDensity * jDensity)))
		fluid.Forces[i].Add(*grad.Scale(F)) //Mutation
	}

}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Viscosity(i int) {

	//For Each Particle Calculate Kernel Based Summation

	samples, nCount, _ := fluid.SPHGrid.GetSamples(&fluid.Positions[i])
	mass := fluid.Mfp.Mass

	iDensity := fluid.Densities[i]
	v := 1.30 / iDensity //viscosity
	vi := fluid.Velocities[i]

	for j := 0; j < nCount; j++ {
		vj := fluid.Velocities[samples[j].Index]
		jDensity := fluid.Densities[samples[j].Index]

		coeff_vec := V.Scale(V.Sub(vj, vi), jDensity)
		dist := fluid.Positions[i].Distance(fluid.Positions[samples[j].Index])
		lap := fluid.GradKernel.O2D(dist)
		force := V.Scale(coeff_vec, v*mass*lap)
		fluid.Forces[i].Add(force)
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
	sos := fluid.Mfp.SpeedSound
	exp := fluid.Mfp.EosExp
	tgt := fluid.Mfp.TargetDensity
	density := fluid.Densities[i]
	eosScale := tgt * float32(Math.Sqrt(float64(sos))) / exp
	p := eosScale / exp * float32(Math.Pow(float64(tgt/density), float64(exp))-1.0)
	if p < 0 {
		p *= negativePressure //Negative Pressure Scaling
	}
	fluid.Pressures[i] = p
}

//Test Particles Against Possible Mesh Collisions given a position and velocity
func (fluid *SPHFluid) Collide(index int) {
	//Resolve Particle Collisions
	normal, collis := fluid.Colliders.Collision(&fluid.Positions[index], &fluid.Velocities[index], float32(fluid.Timer.TS))

	if collis == true {
		k_stiff := float32(0.85) //Restitution Coefficient. Further research req'd
		fluid.Velocities[index].Reflect(normal)
		fluid.Velocities[index].Scale(k_stiff)
	}
}

//Integrates the current particle forces and updates the velocity vector.
//Also updates the position of the particle. Clears all forces
//Utilizes MassFluidParticle description for Time.TS modifier.
func (fluid *SPHFluid) Update(index int) error {

	//Integrates fluid force
	fluid.Velocities[index].Add(*fluid.Forces[index].Scale(float32(fluid.Timer.TS) / fluid.Mfp.Mass))
	//Updates Position
	///TestSPH

	fluid.Positions[index].Add(*fluid.Velocities[index].Scale(float32(fluid.Timer.TS)))

	//Clear Particle Force State
	fluid.Forces[index][0] = float32(0.0)
	fluid.Forces[index][1] = float32(0.0)
	fluid.Forces[index][2] = float32(0.0)

	return nil
}

//Main SPH fluid loop. Integrates all forces. Computes pressure from EOS and Resolves
//Collisions. Updates particle velocity and position then clears all forces
func (fluid *SPHFluid) Compute(threadStatus chan int, secondsAdvance float64) {

	FLUID := fluid.Count
	GRAVITY := V.Vec32{0, GRAV, 0}
	EXTERNAL := V.Vec32{}
	EXTERNAL.Add(GRAVITY)

	done := false

	for !done {
		fluid.UpdateDensities()
		for i := 0; i < FLUID; i++ {
			fluid.PressureEOS(i, 0) //Negative Pressure Scale 0
			fluid.Pressure(i)
			fluid.Viscosity(i)
			fluid.External(i, EXTERNAL)
			//Resolve Mesh Collisions
			fluid.Collide(i)

			//Update Particles and resolve forces
			fluid.Update(i)
		}

		fluid.Timer.StepTime()

		//Has Time Advanced Sufficiently
		//If so send channel message so parent thread can update buffers
		if fluid.Timer.T >= secondsAdvance {
			threadStatus <- 20 //FLUID_THREAD_SYNCED CONST
			fluid.Timer.T = 0.0
			fluid.Timer.TIMELAST = 0.0
		}
	}

}
