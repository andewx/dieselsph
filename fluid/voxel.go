//Voxel Array Based Storage Structure O1 Lookup / Edit Times Using Voxel Index Bucket Technique
//Voxel Array Runs an Update Thread To Continuously Edit Particle Position Indexes
package fluid

import (
	V "diesel.com/diesel/vector"
	"fmt"
)

const MAX_DIM = 15
const LOAD_FACTOR = 2.5
const VOXEL_SAMPLES = 20
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
	Voxel                   [][]int
	PVoxelIdx               ParticleVoxelArray
	Utilized                int
}

//Stores the X,Y,Z, POSITION For Particle References in the Grid
//Header Positions Will Return X,Y,Z,0 Quartet
type VoxelIndex struct {
	VoxIndex [2]int
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
func AllocateVoxelStorage(positions []V.Vec32, res int, scale_grid float32) *VoxelArray {
	//Determine Bucket Header length
	buckets := res * res * res
	num_particles := len(positions)
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
	VoxelStorage.PVoxelIdx = ParticleVoxelArray{make([]VoxelIndex, len(positions))}
	VoxelStorage.VoxelDescriptor.Particles = num_particles
	//Loop Construct Storage Structure
	VoxelStorage.Voxel = make([][]int, res*res*res)
	for i := 0; i < res*res*res; i++ {
		VoxelStorage.Voxel[i] = make([]int, VDescrip.Buckets)
	}

	//Nil All Voxel Refs
	for i := 0; i < res*res*res; i++ {
		for t := 0; t < VDescrip.Buckets; t++ {
			VoxelStorage.Voxel[i][t] = -1
		}
	}

	//Set all Reference indexes to -1 For Initialization
	for i := 0; i < VoxelStorage.VoxelDescriptor.Particles; i++ {
		VoxelStorage.PVoxelIdx.Indexes[i].VoxIndex[0] = -1
		VoxelStorage.PVoxelIdx.Indexes[i].VoxIndex[1] = -1
	}

	return &VoxelStorage
}

func (v *VoxelIndex) IsNil() bool {
	if v.VoxIndex[0] == -1 {
		return true
	}
	return false
}

//Voxel Array Update
func (v *VoxelArray) Update() {
	v.Utilized = 0
	for i := 0; i < v.VoxelDescriptor.Particles; i++ {
		v.UpdateParticle(i)
	}
	if v.Utilization() < 0.90 {
		fmt.Printf("\nError Voxel Particles Exclusion: %f ", v.Utilization())
	}
}

func (v *VoxelArray) UpdateParticle(pindex int) {
	//Find POSITION

	refIndex := v.PVoxelIdx.Indexes[pindex]
	r := refIndex.VoxIndex[0]
	s := refIndex.VoxIndex[1]

	vIndex := v.VoxelHash(v.PositionsRef[pindex])
	x := vIndex.VoxIndex[0]

	//No Change In Particle Bucket
	if r == x && !refIndex.IsNil() {
		v.Utilized++
		return
	}

	//Place particle in bucket if available
	for i := 0; i < v.VoxelDescriptor.Buckets; i++ {
		if v.Voxel[x][i] == -1 {
			v.Voxel[x][i] = pindex
			v.PVoxelIdx.Indexes[pindex] = VoxelIndex{[2]int{x, i}}
			if !refIndex.IsNil() {
				v.Voxel[r][s] = -1
			}
			v.Utilized++
			return
		}
	}

	NeighborVoxels := v.VolumeLookup(v.PositionsRef[pindex])

	//Insert into Neighbor Voxel
	for i := 0; i < len(NeighborVoxels.Indexes); i++ {
		vxl := NeighborVoxels.Indexes[i]
		for j := 0; j < v.VoxelDescriptor.Buckets; j++ {
			p := vxl.VoxIndex[0]
			if v.Voxel[p][j] == -1 {
				v.Voxel[p][j] = pindex
				v.PVoxelIdx.Indexes[pindex] = VoxelIndex{[2]int{p, j}}
				if !refIndex.IsNil() {
					v.Voxel[r][s] = -1
				}
				v.Utilized++
				return
			}
		}
	}

	//Particle Index Not Found
	//return fmt.Errorf("Particle %d No Voxel Space: [%d][%d][%d]\n", pindex, x, y, z)

}

func (v *VoxelArray) Utilization() float32 {
	return float32(v.Utilized) / float32(v.VoxelDescriptor.Particles)
}

//Modulus Hashes A Position Into a Voxel Index Bucket - Lazy Hash Method
func (v *VoxelArray) VoxelHash(pos V.Vec32) VoxelIndex {
	x := pos[0]
	y := pos[1]
	z := pos[2]

	//Push Coordinates to Min Indexes if Outside Bounds - Handle Diff
	if x < v.VoxelDescriptor.Min {
		x = v.VoxelDescriptor.Min
	}
	if y < v.VoxelDescriptor.Min {
		y = v.VoxelDescriptor.Min
	}
	if z < v.VoxelDescriptor.Min {
		z = v.VoxelDescriptor.Min
	}

	//Handle Negative Values For Modulus
	x = x - v.VoxelDescriptor.Min

	y = y - v.VoxelDescriptor.Min

	z = z - v.VoxelDescriptor.Min

	//Hash Position Indexes
	x0 := int(x/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions
	y0 := int(y/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions
	z0 := int(z/v.VoxelDescriptor.DivLength) % v.VoxelDescriptor.Divisions

	//All Indexes are Positive
	if x0 < 0 {
		x0 = x0 + v.VoxelDescriptor.Divisions
	}
	if y0 < 0 {
		y0 = y0 + v.VoxelDescriptor.Divisions
	}
	if z0 < 0 {
		z0 = z0 + v.VoxelDescriptor.Divisions
	}

	VIndex := VoxelIndex{}
	VIndex.VoxIndex[0] = (z0*v.VoxelDescriptor.Divisions+y0)*v.VoxelDescriptor.Divisions + x0
	VIndex.VoxIndex[1] = 0

	return VIndex

}

func (v *VoxelArray) GetSamples(idx int) []int {
	return v.GetSampleVoxels(v.PositionsRef[idx])
}

//Gets Samples from position
func (v *VoxelArray) GetSampleVoxels(pos V.Vec32) []int {
	mNeighbors := v.VolumeLookup(pos)
	sampleIndexes := make([]int, VOXEL_SAMPLES)
	index := 0
	nLength := len(mNeighbors.Indexes)
	pindex := v.VoxelHash(pos)
	//Get Initial Samples From This Particle Volume
	for i := 0; i < v.VoxelDescriptor.Buckets && index < VOXEL_SAMPLES; i++ {
		sIndex := v.Voxel[pindex.VoxIndex[0]][i]
		if sIndex != -1 {
			sampleIndexes[index] = v.Voxel[pindex.VoxIndex[0]][i]
			index++
		}
	}

	//Get Neighbor Samples
	for i := 0; i < nLength && index < VOXEL_SAMPLES; i++ {

		r := mNeighbors.Indexes[i].VoxIndex[0]

		for q := 0; q < v.VoxelDescriptor.Buckets && index < VOXEL_SAMPLES; q++ {
			sIndex := v.Voxel[r][q]
			if sIndex != -1 {
				sampleIndexes[index] = v.Voxel[r][q]
				index++
			}
		}
	}

	return sampleIndexes
}

//Constructs Neighbor Voxels with position hashes
func (v *VoxelArray) VolumeLookup(pos V.Vec32) VoxelLookup {

	vxNeighbors := VoxelLookup{make([]VoxelIndex, 27)}
	i := 0
	//Set Neighbor Volume Indexes
	for x := -1; x < 2; x++ {
		for y := -1; y < 2; y++ {
			for z := -1; z < 2; z++ {
				nPos := V.Scale(pos, 1.0)
				addVec := V.Vec32{float32(x) * v.VoxelDescriptor.DivLength, float32(y) * v.VoxelDescriptor.DivLength, float32(z) * v.VoxelDescriptor.DivLength}
				nPos.Add(addVec)
				vxlHash := v.VoxelHash(nPos)
				vxNeighbors.Indexes[i] = vxlHash
				i++
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
			s.Update()
			synced = false
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
