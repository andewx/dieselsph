package fluid

import (
	"fmt"
	rand "math/rand"
	"testing"

	vector "diesel.com/diesel/vector"
)

func TestParticle(t *testing.T) {
	const PARTICLES = 10
	var particleSet [10]ParticleNode
	var Err error
	var fluidProps = MassFluidParticle{0.01, 0.3, 0.02, 0.04, 0.05, 1500, 1, 1.4}

	rnd := rand.New(rand.NewSource(295275912632))

	for i := 0; i < PARTICLES; i++ {
		nVel := vector.Vec32{rnd.Float32() * 5.0, rnd.Float32() * 5.0, rnd.Float32() * 5.0}
		nPos := vector.Vec32{rnd.Float32() * 10.0, rnd.Float32() * 10, rnd.Float32() * 10.0}
		particle := Particle{nVel, nPos, vector.Vec32{0, 0, 0}, 0.0, 0.0, nil}
		particleSet[i] = ParticleNode{&particle, nil}
		//
	}

	//Update all particle

	for i := 0; i < PARTICLES; i++ {
		if Err != nil {
			t.Errorf(Err.Error()+" Update Particle Routine index %d", i)
			Err = nil
		}
		Err = particleSet[i].Particle.Update(fluidProps)
	}
}

//Fluid Spatial Grid Usage and Load Requires Extensive / Debugging /
// Performance Reporting

func TestGrid(t *testing.T) {
	//Create Enumerated Particle Domain with Spatial Dimensional Modulus with random particle generation

	const PARTICLES = 100
	var Err error
	var particleSet [PARTICLES]*ParticleNode
	rnd := rand.New(rand.NewSource(295275912632))
	var fluidProps = MassFluidParticle{0.01, 0.3, 0.02, 0.04, 0.05, 1500, 1, 1.4}

	for i := 0; i < PARTICLES; i++ {
		nVel := vector.Vec32{rnd.Float32() * 5.0, rnd.Float32() * 5.0, rnd.Float32() * 5.0}
		nPos := vector.Vec32{rnd.Float32() * 100.0, rnd.Float32() * 100, rnd.Float32() * 100.0}
		particle := Particle{nVel, nPos, vector.Vec32{0, 0, 0}, 0.0, 0.0, nil}
		particleSet[i] = &ParticleNode{&particle, nil}
	}

	pSet := particleSet[0:PARTICLES]
	sphGrid := CustomGrid(10, 10)
	Err = sphGrid.Load(pSet, PARTICLES)

	if Err != nil {
		t.Errorf("Particle Grid Construction Error:" + Err.Error())
	} else {
		fmt.Printf("SPH Grid Constructed\n")
	}

	//Select particles from Grid
	for i := 0; i < PARTICLES; i++ {
		pNode := particleSet[i]
		Err = sphGrid.findNode(pNode)
	}
	if Err != nil {
		t.Errorf(Err.Error())
	} else {
		fmt.Printf("All Particles Found\n")
	}

	//Get Particle Colliders for Each Particle
	gridSearcher := GridSearch{sphGrid}

	for i := 0; i < PARTICLES; i++ {
		gridSearcher.GetParticleColliders(particleSet[i], &fluidProps)
	}

	fmt.Printf("All Colliders Accessed\n")

}

func BenchmarkGridOperations(b *testing.B) {
	//Create Enumerated Particle Domain with Spatial Dimensional Modulus with random particle generation

	const PARTICLES = 10000
	var Err error
	var particleSet [PARTICLES]*ParticleNode
	rnd := rand.New(rand.NewSource(295275912632))
	var fluidProps = MassFluidParticle{0.01, 0.3, 0.02, 0.04, 0.05, 1500, 1, 1.4}

	for i := 0; i < PARTICLES; i++ {
		nVel := vector.Vec32{rnd.Float32() * 5.0, rnd.Float32() * 5.0, rnd.Float32() * 5.0}
		nPos := vector.Vec32{rnd.Float32() * 100.0, rnd.Float32() * 100, rnd.Float32() * 100.0}
		particle := Particle{nVel, nPos, vector.Vec32{0, 0, 0}, 0.0, 0.0, nil}
		particleSet[i] = &ParticleNode{&particle, nil}
	}

	pSet := particleSet[0:PARTICLES]
	sphGrid := CustomGrid(10, 10)
	Err = sphGrid.Load(pSet, PARTICLES)

	if Err != nil {
		b.Errorf("Particle Grid Construction Error:" + Err.Error())
	} else {
		fmt.Printf("SPH Grid Constructed\n")
	}

	//Select particles from Grid BENCH MARK LOOP
	//Per Particle Collider Grid Search 404NS WITH LINEAR RUNTIME GROWTH O(N) - O(NLOGN)
	for u := 0; u <= b.N/PARTICLES; u++ {
		for i := 0; i < PARTICLES; i++ {
			pNode := particleSet[i]
			Err = sphGrid.findNode(pNode)
		}
	}

	if Err != nil {
		b.Errorf(Err.Error())
	}
	for u := 0; u <= b.N/PARTICLES; u++ {
		//Get Particle Colliders for Each Particle
		gridSearcher := GridSearch{sphGrid}
		for i := 0; i < PARTICLES; i++ {
			gridSearcher.GetParticleColliders(particleSet[i], &fluidProps)
		}
	}
}
