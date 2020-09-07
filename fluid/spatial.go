package fluid

import (
	"fmt"

	vector "diesel.com/diesel/vector"
)

//Note an updated version of the spatial hash would use GPU Texture Memory

//Spatial Hashing Structure - Computes a Linked Hash List of Particle Positions
type ParticleNode struct {
	Particle *Particle
	link     *ParticleNode
}

type ParticleNodeIter struct {
	P       *ParticleNode
	e       error
	currIdx int
}

type SHGIndex [3]int

//Spatial Hash Grid - Stores a grid of ParticleNode entry point locations over a given range and fidelity
type SpatialHashGrid struct {

	//Vector Valued Attributes Affect the Mapping Function and May be updated
	origin         vector.Vec32        //origin - spatial origin used for grid mapping
	width          float32             //The width of the problem space
	cell           float32             //Cell resolution with regards to spatial domain
	dimensionality int                 //Dimensionality of the grid
	grid           [][][]*ParticleNode //Chained grid mapping Hash Vector
}

//AllocateGrid - Allocates default grid. 20 x 20 x 20 -- 15,625 Grid Locations
//Radial domain of 1.0 centered about origin (-10, 10) on all axis
func AllocateGrid() *SpatialHashGrid {
	sphGrid := SpatialHashGrid{*vector.NewVec32(0.0), 20, 1.0, 20, make([][][]*ParticleNode, 20)}
	//Initialize Dimensional grid
	for i := 0; i < sphGrid.dimensionality; i++ {
		sphGrid.grid[i] = make([][]*ParticleNode, sphGrid.dimensionality)
		for k := 0; k < sphGrid.dimensionality; k++ {
			sphGrid.grid[i][k] = make([]*ParticleNode, sphGrid.dimensionality)
		}
	}

	return &sphGrid
}

func copyIndexes(index SHGIndex, cpyIndex SHGIndex) error {
	cpyIndex[0] = index[0]
	cpyIndex[1] = index[1]
	cpyIndex[2] = index[2]
	return nil
}

func CustomGrid(width float32, dim int) *SpatialHashGrid {

	cellWidth := width / float32(dim)

	sphGrid := SpatialHashGrid{*vector.NewVec32(0.0), width, cellWidth, dim, make([][][]*ParticleNode, dim)}
	//Initialize Dimensional grid
	for i := 0; i < sphGrid.dimensionality; i++ {
		sphGrid.grid[i] = make([][]*ParticleNode, sphGrid.dimensionality)
		for k := 0; k < sphGrid.dimensionality; k++ {
			sphGrid.grid[i][k] = make([]*ParticleNode, sphGrid.dimensionality)
		}
	}

	return &sphGrid
}

//Returns Spatial Hash Index where index Range{0, N*N*N}
//Modulus of Dimensionality retains Locality Clustering
func (s *SpatialHashGrid) Hash(p *Particle) [3]int {
	vecPos := vector.Scale(vector.Abs(p.Position), 1/s.cell) //Scale By Resolution
	idx := SHGIndex{0, 0, 0}
	d := s.dimensionality
	idx[0] = int(vecPos[0]) % d
	idx[1] = int(vecPos[1]) % d
	idx[2] = int(vecPos[1]) % d
	return idx
}

func (s *SpatialHashGrid) Load(particleSystem []*ParticleNode, length int) error {
	if length <= 0 || length > len(particleSystem) {
		return fmt.Errorf("Cant Load in Particle Set with length <= 0")
	}
	for i := 0; i < length; i++ {
		s.InsertNode(particleSystem[i])
	}
	return nil
}

func (s *SpatialHashGrid) getHash(idx [3]int) *ParticleNode {
	return s.grid[idx[0]][idx[1]][idx[2]]
}

//Prints Particle Find
func (s *SpatialHashGrid) findNode(p *ParticleNode) error {
	pNode := p
	sphPosition := s.Hash(pNode.Particle)
	sphNode := s.getHash(sphPosition)
	found := false
	iter := NewParticleIter(sphNode)

	//Particle front of grid
	if sphNode == pNode {
		found = true
	}

	for iter.Next() && found != true {

		if iter.P == pNode {
			found = true
			break
		}

	}
	if found == false {
		s, _ := pNode.Particle.String()
		return fmt.Errorf("Particle " + s + " not found")
	}
	return nil
}

//Executes Iterator Callback function after accessing P
func (s *SpatialHashGrid) InsertNode(p *ParticleNode) error {

	gIdx := s.Hash(p.Particle)
	currNode := s.getHash(gIdx)

	if currNode == nil {
		s.grid[gIdx[0]][gIdx[1]][gIdx[2]] = p
		return nil
	}

	iter := NewParticleIter(currNode)

	for iter.Next() {
		if iter.P == p {
			return fmt.Errorf("Particle Exists in current Grid Position (%d, %d, %d)\n", gIdx[0], gIdx[1], gIdx[2])
		}

	}
	return iter.Insert(p)
}

//Particle Node iterator Functions -------------------------------------------
//----------------------------------------------------------------------------

//Takes care of collision list due to imperfect hashing
func (p *ParticleNodeIter) Insert(nextNode *ParticleNode) error {
	//Make sure Particle Node Address Isn't Referenced
	p.P.link = nextNode
	return nil
}

func NewParticleIter(node *ParticleNode) *ParticleNodeIter {
	var err error
	return &ParticleNodeIter{
		P:       node,
		e:       err,
		currIdx: 0,
	}
}

func (p *ParticleNodeIter) Value() *Particle {
	if p.P == nil {
		panic("Particle Node Iterator - Invalid Access of Particle Value")
	}
	return p.P.Particle
}

//Advances to next stored node - if node was not found this is reported in the error context
func (p *ParticleNodeIter) Next() bool {
	if p.P.link != nil {
		p.P = p.P.link
		p.currIdx++
		return true
	}
	return false
}
