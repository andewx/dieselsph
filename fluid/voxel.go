//Voxel Array Based Storage Structure O1 Lookup / Edit Times Using Voxel Index Bucket Technique
//Voxel Array Runs an Update Thread To Continuously Edit Particle Position Indexes
package fluid

import (
	V "diesel.com/diesel/vector"
	"fmt"
)

const MAX_DIM = 15
const LOAD_FACTOR = 2.0
const VOXEL_SAMPLES = 30
const THREAD_ERROR = 404
const THREAD_RUN_SAMPLER = 60
const THREAD_WAIT_SAMPLER = 61

//Cubical Structure
type VRanges struct {
	Min       float32
	Max       float32
	Divisions int
	Buckets   int
	DivLength float32
	Particles int
}

//Storage Container For Particle Reference Indexes
type VoxelArray struct {
	PositionsRef            []V.Vec32
	VoxelDescriptor         VRanges
	ComponentStorageSizeMax int
	Voxel                   [][][][]int
	PVoxelIdx               ParticleVoxelArray
}

//Stores the X,Y,Z, POSITION For Particle References in the Grid
//Header Positions Will Return X,Y,Z,0 Quartet
type VoxelIndex struct {
	VoxIndex [4]int
}

//Neighbor Volume Look Structure - Nil Refs Out Of Bounds - Includes Center
type VoxelLookup struct {
	Indexes []VoxelIndex
}

//Particle Index Voxel Index Reference
type ParticleVoxelArray struct {
	Indexes []VoxelIndex
}

//Allocates 1.5 x Number Particles Voxel Based Array At A Resolution Not To Exceed MAX_DIM
//If A Particle Cannot Be Added To the Voxel Bucket It will attempt to insert in Neighbor Grids
//If No Positions Are Found In The Neighbor Grid The Particle Index Won't Be Added Or Updated
func AllocateVoxelStorage(positions []V.Vec32, res int, num_particles int, scale_grid float32) *VoxelArray {
	//Determine Bucket Header length
	buckets := res * res * res
	bucketLength := int(float32(num_particles/buckets)*float32(LOAD_FACTOR) + 1.0)

	//Set Main Voxel Headers
	VoxelStorage := VoxelArray{}
	VoxelStorage.PositionsRef = positions

	VDescrip := VRanges{}
	VDescrip.Min = -1.0 * scale_grid
	VDescrip.Max = 1.0 * scale_grid
	VDescrip.Divisions = res
	VDescrip.Buckets = bucketLength
	VDescrip.DivLength = (VDescrip.Max - VDescrip.Min) / float32(res)
	VoxelStorage.VoxelDescriptor = VDescrip
	VoxelStorage.PVoxelIdx = ParticleVoxelArray{make([]VoxelIndex, num_particles)}
	VoxelStorage.VoxelDescriptor.Particles = num_particles

	//Loop Construct Storage Structure
	VoxelStorage.Voxel = make([][][][]int, res)
	for i := 0; i < res; i++ {
		VoxelStorage.Voxel[i] = make([][][]int, res)
		for j := 0; j < res; j++ {
			VoxelStorage.Voxel[i][j] = make([][]int, res)
			for k := 0; k < res; k++ {
				VoxelStorage.Voxel[i][j][k] = make([]int, bucketLength)
			}
		}
	}

	//Nil All Voxel Refs
	for i := 0; i < res; i++ {
		for j := 0; j < res; j++ {
			for k := 0; k < res; k++ {
				for t := 0; t < VoxelStorage.VoxelDescriptor.Buckets; t++ {
					VoxelStorage.Voxel[i][j][k][t] = -1
				}
			}
		}
	}
	return &VoxelStorage
}

//Voxel Array Update
func (v *VoxelArray) Update() error {
	for i := 0; i < v.VoxelDescriptor.Particles; i++ {
		err := v.UpdateParticle(i)
		if err != nil {
			return err
		}
	}
	return nil
}

func (v *VoxelArray) UpdateParticle(pindex int) error {
	//Find POSITION

	refIndex := v.PVoxelIdx.Indexes[pindex]
	r := refIndex.VoxIndex[0]
	s := refIndex.VoxIndex[1]
	t := refIndex.VoxIndex[2]
	u := refIndex.VoxIndex[3]

	vIndex := v.VoxelHash(v.PositionsRef[pindex])
	x := vIndex.VoxIndex[0]
	y := vIndex.VoxIndex[1]
	z := vIndex.VoxIndex[2]

	//No Update
	if x == r && y == s && t == z {
		return nil
	}

	//Place particle in bucket if available
	for i := 0; i < v.VoxelDescriptor.Buckets; i++ {
		if v.Voxel[x][y][z][i] == -1 {
			v.Voxel[x][y][z][i] = pindex
			v.PVoxelIdx.Indexes[pindex] = VoxelIndex{[4]int{x, y, z, i}}
			v.Voxel[r][s][t][u] = -1
			return nil
		}
	}

	NeighborVoxels := v.VolumeLookup(vIndex)

	//Insert into Neighbor Voxel
	for i := 0; i < len(NeighborVoxels.Indexes); i++ {
		vxl := NeighborVoxels.Indexes[i]
		for j := 0; j < v.VoxelDescriptor.Buckets; j++ {
			p := vxl.VoxIndex[0]
			q := vxl.VoxIndex[1]
			w := vxl.VoxIndex[2]
			if v.Voxel[p][q][w][j] == -1 {
				v.Voxel[p][q][w][j] = pindex
				v.PVoxelIdx.Indexes[pindex] = VoxelIndex{[4]int{p, q, w, j}}
				v.Voxel[r][s][t][u] = -1
				return nil
			}
		}
	}

	//Particle Index Not Found
	return fmt.Errorf("Particle %d No Voxel Space: [%d][%d][%d]\n", pindex, x, y, z)

}

//Modulus Hashes A Position Into a Voxel Index Bucket - Lazy Hash Method
func (v *VoxelArray) VoxelHash(pos V.Vec32) VoxelIndex {
	x := pos[0]
	y := pos[1]
	z := pos[2]

	//Push Coordinates to Min Indexes if Outside Bounds
	if x < v.VoxelDescriptor.Min {
		x = v.VoxelDescriptor.Min
	}
	if y < v.VoxelDescriptor.Min {
		y = v.VoxelDescriptor.Min
	}
	if z < v.VoxelDescriptor.Min {
		z = v.VoxelDescriptor.Min
	}

	//Confine to Max Bounds
	if x > v.VoxelDescriptor.Max {
		x = v.VoxelDescriptor.Max
	}
	if y > v.VoxelDescriptor.Max {
		y = v.VoxelDescriptor.Max
	}
	if z > v.VoxelDescriptor.Max {
		z = v.VoxelDescriptor.Max
	}

	//Handle Negative Values For Modulus
	if x < 0 {
		x = -x + v.VoxelDescriptor.Max
	}

	if y < 0 {
		y = -y + v.VoxelDescriptor.Max
	}

	if z < 0 {
		z = -z + v.VoxelDescriptor.Max
	}

	//Hash Position Indexes
	x0 := int(x/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions
	y0 := int(y/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions
	z0 := int(z/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions

	VIndex := VoxelIndex{}
	VIndex.VoxIndex[0] = x0
	VIndex.VoxIndex[1] = y0
	VIndex.VoxIndex[2] = z0
	VIndex.VoxIndex[0] = 0

	return VIndex

}

func (v *VoxelArray) GetSamples(idx int) []int {
	vIndex := v.PVoxelIdx.Indexes[idx]
	return v.GetSampleVoxels(vIndex)
}

func (v *VoxelArray) GetSampleVoxels(pIndex VoxelIndex) []int {
	mNeighbors := v.VolumeLookup(pIndex)
	sampleIndexes := make([]int, VOXEL_SAMPLES)
	index := 0
	nLength := len(mNeighbors.Indexes)

	//Get Initial Samples From This Particle Volume
	for i := 0; i < v.VoxelDescriptor.Buckets && index < VOXEL_SAMPLES; i++ {
		if v.Voxel[pIndex.VoxIndex[0]][pIndex.VoxIndex[1]][pIndex.VoxIndex[2]][i] != -1 {
			sampleIndexes[index] = v.Voxel[pIndex.VoxIndex[0]][pIndex.VoxIndex[1]][pIndex.VoxIndex[2]][i]
			index++
		}
	}

	//Get Neighbor Samples
	for i := 0; i < nLength && index < VOXEL_SAMPLES; i++ {

		r := mNeighbors.Indexes[i].VoxIndex[0]
		s := mNeighbors.Indexes[i].VoxIndex[1]
		t := mNeighbors.Indexes[i].VoxIndex[2]
		for q := 0; q < v.VoxelDescriptor.Buckets && index < VOXEL_SAMPLES; q++ {
			if v.Voxel[r][s][t][q] != -1 {
				sampleIndexes[index] = v.Voxel[r][s][t][q]
				index++
			}
		}
	}

	return sampleIndexes
}

//Constructs Neihbor Voxel Structure Given VoxelIndex and Bounds
func (v *VoxelArray) VolumeLookup(vIndex VoxelIndex) VoxelLookup {

	n0 := 3
	n1 := 3
	n2 := 3
	x := vIndex.VoxIndex[0]
	y := vIndex.VoxIndex[1]
	z := vIndex.VoxIndex[2]

	//Check All Bounds
	i := x - 1
	j := y - 1
	k := z - 1

	i0 := x + 1
	j0 := y + 1
	k0 := z + 1

	//Check Bounds and Set Index Ranges
	if x-1 < 0 {
		i = x
		n0 = n0 - 1
	}
	if x+1 > v.VoxelDescriptor.Divisions {
		i0 = x
		n0 = n0 - 1
	}
	if y-1 < 0 {
		j = y
		n1 = n1 - 1
	}
	if y+1 > v.VoxelDescriptor.Divisions {
		j0 = j
		n1 = n1 - 1
	}
	if z-1 < 0 {
		k = z
		n2 = n2 - 1
	}
	if z+1 > v.VoxelDescriptor.Divisions {
		k0 = z
		n2 = n2 - 1
	}

	vxNeighbors := VoxelLookup{make([]VoxelIndex, n0*n1*n2)}

	//Set Neighbor Volume Indexes
	qIndex := 0
	for r := i; r < i0; r++ {
		for s := j; s < j0; s++ {
			for t := k; t < k0; t++ {
				if qIndex < n0*n1*n2 {
					vxNeighbors.Indexes[qIndex] = VoxelIndex{[4]int{r, s, t, 0}}
					qIndex++
				}
			}
		}
	}

	return vxNeighbors

}

//Thread Run
//Runs sampler interface run. Performs the core sampler maintence taskings
func (s VoxelArray) Run(status chan int) {
	done := false
	synced := true

	for !done {
		if synced {
			err := s.Update()
			synced = false

			if err != nil {
				fmt.Printf(err.Error() + "\nVoxel Update Failed\n")
			}
			status <- THREAD_WAIT_SAMPLER //Thread has been synced
			nStatus := <-status           //Wait on the next status update
			if nStatus == THREAD_RUN_SAMPLER {
				synced = true
			}
		}
	}
}

func (v *VoxelArray) PrintStorageRequirements() {
	particleBytes := v.VoxelDescriptor.Particles * 16
	cells := v.VoxelDescriptor.Divisions * v.VoxelDescriptor.Divisions * v.VoxelDescriptor.Divisions
	voxelIndexBytes := cells * 4 * v.VoxelDescriptor.Buckets
	totalBytes := particleBytes + voxelIndexBytes
	kilobytes := totalBytes / 1024

	fmt.Printf("Voxel Grid Storage: %dkB\nBucket Size: %d\nVoxel Cells: %d\n", kilobytes, v.VoxelDescriptor.Divisions, cells)

}
