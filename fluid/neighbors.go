package fluid

import vector "diesel.com/diesel/vector"
import "fmt"

//Particle Type properties include only velocity and position
//Particle Mass Viscosity and additional params will be handled by fluid description
//This is the heart of the SPH Calculation since it will determine boundary and collision on particles

//Radial Grid Search Return Valued
type NeighborGrid [7]SHGIndex

//Radial Grid Search Structure
type GridSearch struct {
	Grid *SpatialHashGrid
}

const NEIGHBORS = 7
const COLLIDER_COUNT = 10

//Returns NeighborGrid Index Structure for accessing Left-Right-Top-Bottom Potential Collider Positions
//For Now We Wrap The Indexes
func (shg *GridSearch) GetNeighborGrid(particleNode *ParticleNode) (*NeighborGrid, error) {
	initial := shg.Grid.Hash(particleNode.Particle)

	this := initial
	left := SHGIndex{0, 0, 0}
	right := SHGIndex{0, 0, 0}
	top := SHGIndex{0, 0, 0}
	bottom := SHGIndex{0, 0, 0}
	back := SHGIndex{0, 0, 0}
	front := SHGIndex{0, 0, 0}

	copyIndexes(initial, left)
	copyIndexes(initial, right)
	copyIndexes(initial, top)
	copyIndexes(initial, bottom)

	left[0] = left[0] - 1
	right[0] = right[0] + 1

	bottom[1] = bottom[1] - 1
	top[1] = top[1] + 1

	front[2] = front[2] - 1
	back[2] = back[2] + 1

	//Wrap collider Bounds // Check for boundary collision
	if left[0] <= 0 {
		left[0] = shg.Grid.dimensionality - 1
	}
	if right[0] >= shg.Grid.dimensionality-1 {
		right[0] = 0
	}
	if top[1] >= shg.Grid.dimensionality-1 {
		top[1] = 0
	}
	if bottom[1] <= 0 {
		bottom[1] = shg.Grid.dimensionality - 1
	}

	if back[2] >= shg.Grid.dimensionality-1 {
		back[2] = 0
	}
	if front[2] <= 0 {
		front[2] = shg.Grid.dimensionality - 1
	}

	nhGrid := NeighborGrid{this, left, right, top, bottom, back, front}

	return &nhGrid, nil
}

//Main Neighbor Grid Colliders Functionality
func (shg *GridSearch) GetParticleColliders(particleNode *ParticleNode, fluid *MassFluidParticle) ([]*ParticleNode, error) {

	neighbors, _ := shg.GetNeighborGrid(particleNode)
	colliders := make([]*ParticleNode, 10)
	count := 0

	for i := 0; i < NEIGHBORS; i++ {
		PNode := shg.Grid.getHash(neighbors[i])
		PNodeIter := NewParticleIter(PNode)

		if PNode == nil {
			return colliders, fmt.Errorf("Invalid Grid Location Accessed\n")
		}

		for PNodeIter.Next() {
			radialDistance := vector.Length(vector.Sub(PNode.Particle.Position, particleNode.Particle.Position))
			if radialDistance <= fluid.OuterRadius && PNodeIter.P != PNode && count < COLLIDER_COUNT {
				colliders[count] = PNodeIter.P
				count++
			}
		}
	}

	//Search all neighbords for particles within particle radius
	return colliders, nil
}
