package fluid

//Particle Type properties include only Velocity and Position
//Particle Mass Viscosity and additional params will be handled by fluid description
import (
	"fmt"

	vector "diesel.com/diesel/vector"
)

//Particle Values to be interpolated will be handled from its own struct
type Particle struct {
	Force    vector.Vec32
	Density  float32
	Pressure float32
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

//Force - Accumulate the Force vector
func (h *Particle) ForceApply(f vector.Vec32) error {

	if &f != nil {
		h.Force.Add(f)
		return nil
	}

	return fmt.Errorf("Force Vector Memory Exception\n")
}

//Clears particle Force vector
func (h *Particle) Clear() {
	h.Force[0] = 0.0
	h.Force[1] = 0.0
	h.Force[2] = 0.0
}

//Integrates the current particle forces and updates the velocity vector.
//Also updates the position of the particle. Clears all forces
//Utilizes MassFluidParticle description for timestep modifier.
func (h *Particle) Update(fluid MassFluidParticle, position *vector.Vec32, velocity *vector.Vec32) error {

	//Check Velocity and Position vector memory Positions
	if &position == nil || velocity == nil {
		return fmt.Errorf("Error: Particle Velocity or Position Vectors Not Defined In Memory")
	}

	//Integrates fluid force
	velocity.Add(*h.Force.Scale(fluid.TimeStep / fluid.Mass))

	//Updates Position
	position.Add(velocity.Scale(fluid.TimeStep))
	//Clear Particle State
	h.Force.Clear()

	return nil
}

//Update particle based on collision with a restitution Velocity
//Particle tangential drag should deviate from perfect reflection
func (h *Particle) Collide(norm vector.Vec32, V *vector.Vec32) error {
	//Resitution Force + Normal Component + Tangential Component

	k_stiff := float32(0.85)
	V.Reflect(norm)
	V.Scale(k_stiff)

	return nil
}
