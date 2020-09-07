package fluid

//Particle Type properties include only Velocity and Position
//Particle Mass Viscosity and additional params will be handled by fluid description
import (
	"fmt"

	vector "diesel.com/diesel/vector"
)

//Particle Values to be interpolated will be handled from its own struct
type Particle struct {
	Velocity            vector.Vec32
	Position            vector.Vec32
	Force               vector.Vec32
	index               int
	Density             float32
	Pressure            float32
	SignedDistanceField []float32 //indexes sdf for each collision object
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

//Uodates Fluid Particle Position - However we need to resolve collisions
func (h *Particle) Update(fluid MassFluidParticle) error {

	//Check Velocity and Position vector memory Positions
	if &h.Velocity == nil || &h.Position == nil {
		return fmt.Errorf("Error: Particle Velocity or Position Vectors Not Defined In Memory")
	}

	h.Velocity.Add(*h.Force.Scale(1 / fluid.Mass).Scale(fluid.TimeStep))
	h.Position.Add(*h.Velocity.Scale(fluid.TimeStep))
	//Clear Particle State
	h.Force.Clear()

	return nil
}

func (h *Particle) Momentum(mass float32) (float32, error) {
	return h.Velocity.Length() * mass, nil
}

func (h *Particle) String() (string, error) {
	s := fmt.Sprintf("(Pos: %f, %f, %f) (Vel: %f, %f, %f)\n", h.Position[0], h.Position[1],
		h.Position[2], h.Velocity[0], h.Velocity[1], h.Velocity[2])

	return s, nil
}

func (h *Particle) GetPosition() *vector.Vec32 {
	return &h.Position
}

func (h *Particle) GetVelocity() *vector.Vec32 {
	return &h.Velocity
}

//Update particle based on collision with a restitution Velocity
//Particle tangential drag should deviate from perfect reflection
func (h *Particle) Collide(norm vector.Vec32) error {
	//Resitution Force + Normal Component + Tangential Component

	k_stiff := float32(0.85)
	h.Velocity.Reflect(norm)
	h.Velocity.Scale(k_stiff)

	return nil
}
