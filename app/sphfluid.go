package app

import (
	F "diesel.com/diesel/fluid"
	G "diesel.com/diesel/geometry"
	V "diesel.com/diesel/vector"
	Math "math"
)

//SPHFluid structure: Holds relevant fluid particles, spatial grid, meshes
//and animation context. Does not initialize the specific construction of the particle
//field to be studied.

//Will handle geometry export to Mistuba 3D Render and other formats. IsoSurface
//Meshes will be exported to .obj file format.

//These particle positions will need to be passed to OpenGL Vertex Buffers
//Made accessible by GLFW and GOGL frameworks.
const MAX_PCI = 3 //Cubic Predictive Branch
const GRAV = -9.810435
const EOS_EXP = 1 //EOS Stiffness Parameter

//SPHFluid - Is a PCISPH Fluid with Predictive Correction of Pressures
type SPHFluid struct {
	SPHGrid             *F.SpatialHashGrid   //Spatial Hash Grid For Neighbor Particles
	SPHGridSearcher     *F.GridSearch        //Adds Functionality to the SPHGrid
	Particles           []*F.ParticleNode    //Particle System Array
	Colliders           *G.Mesh              //Collider Triangle Meshes
	FluidDescriptor     *F.MassFluidParticle //Fluid Particle Descriptor
	InterpolationKernel F.GaussianKernel     //Gaussian Kernel Typically
	GradientKernel      F.CubicKernel        //Cubic Kernel For Derivative Kernels
	Time                float32              //Animation Time
	TimeStep            float32              //Time Step
	TimeLast            float32              //Time Last
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//Initialize does heavy lifting of setting up the Grid Data and Computing Initial
//Particle Densities
func (fluid *SPHFluid) Initialize() {
	fluid.UpdateDensities()
	//Time step dependent on propogation of particle collisions
	fluid.FluidDescriptor.TimeStep = (fluid.FluidDescriptor.OuterRadius * fluid.FluidDescriptor.Mass) / fluid.FluidDescriptor.SpeedSound
	fluid.TimeStep = fluid.FluidDescriptor.TimeStep
}

//Updates Densities associated with each particle position with Gaussian Kernel
func (fluid *SPHFluid) UpdateDensities() {
	FIELD := len(fluid.Particles)
	//Compute Density Fieldsa
	for i := 0; i < FIELD; i++ {
		//For Each Particle Calculate Kernel Based Summation
		thisParticle := fluid.Particles[i]
		neighbors, _ := fluid.SPHGridSearcher.GetParticleColliders(thisParticle, fluid.FluidDescriptor)
		n := len(neighbors)
		mass := fluid.FluidDescriptor.Mass
		density := mass
		for j := 0; j < n; j++ {
			dist := V.Length(V.Sub(thisParticle.Particle.Position, neighbors[i].Particle.Position))
			density += mass * fluid.InterpolationKernel.F(dist)
		}
		thisParticle.Particle.Density = density
	}
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) DensityGradient(i int) V.Vec32 {

	//For Each Particle Calculate Kernel Based Summation
	DensityGrad := V.Vec32{}
	thisParticle := fluid.Particles[i]
	neighbors, _ := fluid.SPHGridSearcher.GetParticleColliders(thisParticle, fluid.FluidDescriptor)
	n := len(neighbors)
	mass := fluid.FluidDescriptor.Mass
	iDensity := thisParticle.Particle.Density

	for j := 0; j < n; j++ {
		p := neighbors[i].Particle
		jDensity := p.Density
		dir := V.Sub(p.Position, thisParticle.Particle.Position)
		dist := V.Length(dir)
		dir = V.Scale(dir, 1/dist)
		grad := fluid.GradientKernel.Grad(dist, &dir)
		estm := (mass / iDensity) + (mass / jDensity)
		DensityGrad.Add(*grad.Scale(estm)) //Mutation
	}

	return DensityGrad
}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Pressure(i int) {

	//For Each Particle Calculate Kernel Based Summation
	thisParticle := fluid.Particles[i]
	neighbors, _ := fluid.SPHGridSearcher.GetParticleColliders(thisParticle, fluid.FluidDescriptor)
	n := len(neighbors)
	mass := fluid.FluidDescriptor.Mass
	iDensity := thisParticle.Particle.Density

	for j := 0; j < n; j++ {
		p := neighbors[i].Particle
		jDensity := p.Density
		dir := V.Sub(p.Position, thisParticle.Particle.Position)
		dist := V.Length(dir)
		dir = V.Scale(dir, 1/dist)
		grad := fluid.GradientKernel.Grad(dist, &dir)
		estm := (mass*mass*(thisParticle.Particle.Pressure/iDensity*iDensity) + (p.Pressure / jDensity * jDensity))
		thisParticle.Particle.ForceApply(*grad.Scale(-estm)) //Mutation
	}

}

//Updates gradient associated with each particle position with Gaussian Kernel -- these should be
//Gradient Value Vectors
func (fluid *SPHFluid) Viscosity(i int) {

	//For Each Particle Calculate Kernel Based Summation
	thisParticle := fluid.Particles[i]
	neighbors, _ := fluid.SPHGridSearcher.GetParticleColliders(thisParticle, fluid.FluidDescriptor)
	n := len(neighbors)
	mass := fluid.FluidDescriptor.Mass

	iDensity := thisParticle.Particle.Density
	v := 1.30 / iDensity //viscosity
	vi := thisParticle.Particle.Velocity

	for j := 0; j < n; j++ {
		p := neighbors[i].Particle
		vj := p.Velocity
		jDensity := p.Density

		coeff_vec := V.Scale(V.Sub(vj, vi), jDensity)
		dist := thisParticle.Particle.Position.Distance(p.Position)
		lap := fluid.GradientKernel.O2D(dist)
		force := V.Scale(coeff_vec, v*mass*lap)
		thisParticle.Particle.ForceApply(force)
	}

	return
}

//Updates particle system with accumalted External Force (I.E. Gravity)
func (fluid *SPHFluid) External(i int, f V.Vec32) {
	fluid.Particles[i].Particle.ForceApply(f)
}

//Computes Pressure From A Given Equation of State which models incompressible flow
//We may use this pressure solution for an initialization state surely.
func (fluid *SPHFluid) PressureEOS(i int) {
	sos := fluid.FluidDescriptor.SpeedSound
	exp := fluid.FluidDescriptor.EosExp
	tgt := fluid.FluidDescriptor.TargetDensity
	eosScale := tgt * (sos * sos) * exp
	p := float64(eosScale) / float64(exp) * Math.Pow(float64(tgt/fluid.Particles[i].Particle.Density-1), float64(exp))
	if p < 0 {
		p *= 0.1 //Negative Pressure Scaling  - Make Const
	}
	fluid.Particles[i].Particle.Pressure = float32(p)
}

func (fluid *SPHFluid) Collide(p *F.Particle) {
	//Resolve Particle Collisions
	var normal V.Vec32
	var collis bool
	normal, collis = fluid.Colliders.Collision(p, fluid.FluidDescriptor.TimeStep)

	if collis == true {
		p.Collide(normal)
	}
}

//Iterates through the particle system applying force computation
func (fluid *SPHFluid) Compute() {
	FLUID := len(fluid.Particles)
	GRAVITY := V.Vec32{0, GRAV, 0}
	EXTERNAL := V.Vec32{}
	EXTERNAL.Add(GRAVITY)

	//Conditioning Loop
	fluid.UpdateDensities()

	//Fluid Properties
	for i := 0; i < FLUID; i++ {
		particle := fluid.Particles[i].Particle
		particle.Clear()
		fluid.PressureEOS(i)
		fluid.Viscosity(i)
		fluid.Pressure(i)
		fluid.External(i, EXTERNAL)

		//Resolve Mesh Collisions
		fluid.Collide(particle)

		//Update Particles
		particle.Update(*fluid.FluidDescriptor)
	}

	fluid.TimeLast = fluid.Time
	fluid.Time += fluid.TimeStep
}
