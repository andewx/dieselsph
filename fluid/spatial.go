package fluid

import (
	"fmt"
	V "diesel.com/diesel/vector"
)

const NEIGHBORS = 7
const COLLIDER_SAMPLES = 10
const PARTICLE_SAMPLES = 20

//Spatial Hash Grid - Stores a Grid of IDNode entry point locations with a simple spatial hash
//See Hash Function Below (float32:Scale - Width of the data structure determines how much wrapping / int:Subiv how many times that width is divided for the lenght depth width of the cube)
type SpatialHashGrid struct {
	//V Valued Attributes Affect the Mapping Function and May be updated
	Scale          float32         //Scale positional wrapping
	Subdiv int                		 //Subdiv of the Grid
	Grid           [][][]IDNode 	//Chained Grid mapping Hash V
}

//-----------------------Utility Structs--------------------------------//

//IDNode stores a simple integer index. For this application indexes are equivlant to particle ID
type IDNode struct {
	Index		 int    //Value stored is the Array Index Data
	Link     *IDNode
}

//ID Node Iterator seeks and steps through node chains. We do not include Pre-Post processing
type IDNodeIter struct {
	P       *IDNode
	currIdx int
}

//Radial Grid Search Return Valued
type NeighborGrid [7][3]int

//----------------------------------------------------------------------//

//AllocateGrid - Allocates default Grid. 20 x 20 x 20 -- 15,625 Grid Locations
//Radial domain of 1.0 centered about origin (-10, 10) on all axis
func AllocateGrid() *SpatialHashGrid {
	sphGrid := SpatialHashGrid{*V.NewVec32(0.0), 20, 1.0, 20, make([][][]IDNode, 20)}
	//Initialize Dimensional Grid
	for i := 0; i < sphGrid.Subdiv; i++ {
		sphGrid.Grid[i] = make([][]IDNode, sphGrid.Subdiv)
		for k := 0; k < sphGrid.Subdiv; k++ {
			sphGrid.Grid[i][k] = make([]IDNode, sphGrid.Subdiv)
		}
	}

	return &sphGrid
}


//Grid Searching methods

//Returns NeighborGrid Index Structure for accessing Left-Right-Top-Bottom Potential Collider Positions
//Passes in V.Vec32:particle Position - for determing grid index point
func (shg *SpatialHashGrid) GetNeighborGrid(node [3]int) (*NeighborGrid, error) {
	initial := node
	this := initial
	left := [3]int{0, 0, 0}
	right := [3]int{0, 0, 0}
	top := [3]int{0, 0, 0}
	bottom := [3]int{0, 0, 0}
	back := [3]int{0, 0, 0}
	front := [3]int{0, 0, 0}

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
		left[0] = shg.Subdiv - 1
	}
	if right[0] >= shg.Subdiv-1 {
		right[0] = 0
	}
	if top[1] >= shg.Subdiv-1 {
		top[1] = 0
	}
	if bottom[1] <= 0 {
		bottom[1] = shg.Subdiv - 1
	}

	if back[2] >= shg.Subdiv-1 {
		back[2] = 0
	}
	if front[2] <= 0 {
		front[2] = shg.Subdiv - 1
	}

	nhGrid := NeighborGrid{this, left, right, top, bottom, back, front}

	return &nhGrid, nil
}

//Gathers all particles within a search radius and returns a list once 20 particles are found
//We need to expand the search for the neighbor grid if particles arent found and keep going until list is full
//Given a position, search for a particle set to sample nearby. For now just return a count of samples returned
//The beauty here is that we can just sample any position easily
func (shg *SpatialHashGrid) GetSamples(position *V.Vec32) ([]IDNode, int, error) {
	head := shg.Hash(position)
	neighbors, _ := shg.GetNeighborGrid(head)
	samples := make([]IDNode, PARTICLE_SAMPLES) //Currently Set at 20
  PNode := shg.getHash(head)
	count := 0

	//Iterate through the Neigbors grid lists and append IDs as they are found
	for i := 0; i < NEIGHBORS; i++ {
		PNode := shg.getHash(neighbors[i]) //Neighbors actually searchits own first index first
		if PNode != nil {
					PNodeIter := NewIter(PNode)
					for PNodeIter.Next() {
							samples[count++] = PNodeIter.P
						}
		}
		return samples, count, nil
	 }
}




//Allows us to copy and preserve some indexes when we check the neighbors.
func copyIndexes(index [3]int, cpyIndex [3]int) error {
	cpyIndex[0] = index[0]
	cpyIndex[1] = index[1]
	cpyIndex[2] = index[2]
	return nil
}


//Creates a custom storage Grid cube with specified int:Scale wrapping domains and  int:dim specifying  subdivisions in the  cube
func AllocateGridUserDefined(Scale float32, dim int) *SpatialHashGrid {

	cellScale := Scale / float32(dim)
	sphGrid := SpatialHashGrid{*V.NewVec32(0.0), Scale, cellScale, dim, make([][][]IDNode, dim)}
	//Initialize Dimensional Grid
	for i := 0; i < sphGrid.Subdiv; i++ {
		sphGrid.Grid[i] = make([][]IDNode, sphGrid.Subdiv)
		for k := 0; k < sphGrid.Subdiv; k++ {
			sphGrid.Grid[i][k] = make([]IDNode, sphGrid.Subdiv)
		}
	}

	return &sphGrid
}

//Returns Spatial Hash Index where index Range{0, N*N*N}
//Modulus of Subdiv retains Locality Clustering
func (s *SpatialHashGrid) Hash(p *V.Vec32) [3]int {
	vecPos := V.Scale(V.Abs(p.Position), 1/s.cell) //Scale By Resolution
	idx := [3]int
	d := s.Subdiv
	idx[0] = int(vecPos[0]) % d
	idx[1] = int(vecPos[1]) % d
	idx[2] = int(vecPos[1]) % d
	return idx
}

//Loads particle grid with particle system positional data by Inserting nodes Based
//On indexed positional data
func (s *SpatialHashGrid) Load(Positions []V.Vec32) error {
	if Positions == nil {
		return fmt.Errorf("Positions Don't Exist")
	}
	length := len(Positions)

	for i := 0; i < length; i++ {
		s.InsertNode(Positions[i],i)
	}
	return nil
}

func (s *SpatialHashGrid) getHash(idx [3]int) *IDNode {
	return s.Grid[idx[0]][idx[1]][idx[2]]
}

//Finds particle by its index location and position. This typically wont conern us with fluids
//But we added the function as a utility
func (s *SpatialHashGrid) findNode(index int, pos *V.Vec32) error {

	//Setup Node Iterator with a Grid Index
	sphPosition := s.Hash(pos)
	sphNode := s.getHash(sphPosition)
	found := false
	iter := NewIter(sphNode)

	if sphNode == nil{
		return fmt.Errorf("Particle Index wasn't found in grid location")
	}

	//Particle front of Grid
	if index == sphNode.index {
		found = true
	}

	for iter.Next() && found != true {

		if iter.P.Index == index {
			found = true
			break
		}

	}
	if found == false {
		return fmt.Errorf("Particle not found")
	}
	return nil
}

//Inserts a IDNode into the Spatial Hash Grid based on a given position. Index with that position is stored in the tree.
func (s *SpatialHashGrid) InsertNode(pos *V.Vec32, index int) error {

	gIdx := s.Hash(pos)
	currNode := s.getHash(gIdx)

	if currNode == nil {
		s.Grid[gIdx[0]][gIdx[1]][gIdx[2]] = IDNode{index,nil}
		return nil //End InsertNode
	}
	data := IDNode{index,nil}
	//Inserts Node In Beginning of List
	iter := NewIter(currNode)
	return iter.Insert(data)
}

//Inserts the node into the current node tree.Inserts and updates links
func (p *IDNodeIter) Insert(nextNode IDNode) error {
	//Make sure Particle Node Address Isn't Referenced
	tmpNode := p.P.Link
	p.P.Link = &nextNode
	p.P.Link = tmpNode
	return nil
}

func NewIter(node IDNode) IDNodeIter {
	var err error
	return &IDNodeIter{
		P:       &node,
		currIdx: 0,
	}
}

func (p *IDNodeIter) Value() int {
	if p.P == nil {
		panic("Particle Node Iterator - Particle Is Nil")
	}
	return p.P.Index
}

//Advances to next stored node - if node was not found this is reported in the error context
func (p *IDNodeIter) Next() bool {
	if p.P.Link != nil {
		p.P = p.P.Link
		p.currIdx++
		return true
	}
	return false
}


//Grid Searching Methods
