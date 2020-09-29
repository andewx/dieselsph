package fluid

import V "diesel.com/diesel/vector"
import "math"
import "fmt"
import "unsafe"

const (
	THREAD_RUN_SAMPLER  = 60
	THREAD_WAIT_SAMPLER = 61
	THREAD_STOP_SAMPLER = 62
)

//Spatial Deviation Sampler maintains linear index buffer into spatial positions array
//Implements SPHSampler interface. Sampler analysis routine should be called from high level api
//stack to run concurrently with the system using the Sampler through its concurrent hook Run()
//Look for future implementation of Spatial Clustering Sampler to improve and extend this model
type SPDSampler struct {
	Std       float32   //Standard Deviation Global
	MDist     float32   //Mean Particle Distance from Origin
	MIdx      int       //Centroid Indexes
	Positions []V.Vec32 //spatial positions reference (does not modify)
	IdxPos    []int     //index positions
	S         int       //number samples
	P         int       //particles
	Occup     []uint64  //Compresses particle index occupancy list
	BigEndian bool      //Check the endianess
}

//Initiates Sampler  (samples is number samples)
func (s *SPDSampler) SPDSampler(samples int, particles []V.Vec32) {
	positions := len(particles)

	s.P = positions
	s.IdxPos = make([]int, positions)
	s.Positions = particles //make sure this is only a reference
	s.S = samples
	meanIdx := 0
	MeanDist := float32(0.0)
	occupints := float64(positions) / 64.0
	s.Occup = make([]uint64, int(math.Ceil(occupints)))

	for i := 0; i < positions; i++ {
		s.IdxPos[i] = i
		meanIdx++
		MeanDist += V.Length(s.Positions[i])
	}

	s.MDist = MeanDist / float32(positions)
	s.MIdx = meanIdx / 2

	meanStd := float32(0.0)
	for i := 0; i < positions; i++ {
		x := V.Length(s.Positions[i]) - s.MDist
		meanStd += x * x
	}

	s.Std = float32(math.Sqrt(float64(meanStd / float32(s.P))))

	s.BigEndian = s.CheckEndian()
	//Now set all the index positions based on spatial deviations
	//We have the problem that indexes must be unique so we don't lose and Indexes
	//From the mapping
	s.MapIndexes()

}

//Gets samples but produces clustering behavior of indexes by swapping
//indexes based on standard deviations of distances. Number of samples returned
//is not guaranteed
func (s *SPDSampler) GetSamples(v V.Vec32) []int {

	vIdx := s.MapDeviation(v)
	minIdx, maxIdx := s.GetSamplerIndexBounds(vIdx)
	count := maxIdx - minIdx
	samples := make([]int, maxIdx-minIdx)

	for i := 0; i < count; i++ {
		samples[i] = s.IdxPos[i+minIdx]
	}

	return samples
}

//Runs sampler interface run. Performs the core sampler maintence taskings
func (s *SPDSampler) Run(status chan int) {
	done := false
	threadSignal := <-status
	for !done {
		if threadSignal == THREAD_RUN_SAMPLER {
			s.MapIndexes()
			status <- THREAD_WAIT_SAMPLER //Signal Thread Status waiting
			if threadSignal == THREAD_STOP_SAMPLER {
				done = true
			}
		}
	}
}

//Maps a standard deviation to an index
func (s *SPDSampler) MapDeviation(v V.Vec32) int {
	vIdx := 0
	x := (V.Length(v) - s.MDist)
	dev := float32(math.Sqrt(float64((x * x) / float32(s.P))))
	oStep := (s.Std * 2) / float32(s.P)
	ltMean := true

	if V.Length(v) > s.MDist {
		ltMean = false
	}

	if ltMean {
		vIdx = s.MIdx - int(dev/oStep) //need to check if conversion floors the value
	} else {
		vIdx = s.MIdx + int(dev/oStep)
	}

	//Bounds check
	if vIdx < 0 {
		vIdx = 0
	}
	if vIdx >= s.P-1 {
		vIdx = s.P - 1
	}

	return vIdx
}

//Gets Sampler Index Bounds from a central index and sample count parameter
//returns min index, max index of the bounds
func (s *SPDSampler) GetSamplerIndexBounds(sampleIdx int) (int, int) {
	//Indexes should not wrap instead reposition the sampling index
	minSampleIdx := sampleIdx - (s.S / 2)
	maxSampleIdx := (sampleIdx + s.S/2)

	if minSampleIdx < 0 {
		minOffset := int(math.Abs(float64(minSampleIdx)))
		minSampleIdx = 0
		maxSampleIdx += minOffset
	}

	if maxSampleIdx >= s.P {
		maxOffset := maxSampleIdx - s.P
		maxSampleIdx = s.P - 1
		minSampleIdx += maxOffset
	}

	//Final check right sample - this code should not execute
	if minSampleIdx < 0 || maxSampleIdx >= s.P {
		minSampleIdx = sampleIdx
		maxSampleIdx = sampleIdx + s.S
		if maxSampleIdx >= s.P {
			maxSampleIdx = s.P - 1
		}
	}

	return minSampleIdx, maxSampleIdx

}

//Maps all position indexes based on reference positions. The occupancy uint64
//bit array tracks the occupancy status of an index if the bit is flagged
//then index is mapped to a location to right of. Index Bounds collisions cause
//A seek then to the left of the mapped index location
func (s *SPDSampler) MapIndexes() error {

	//Clear the bit arrays
	s.ClearOccupancy()

	for i := 0; i < len(s.Positions); i++ {
		v := s.Positions[i]
		vIdx := s.MapDeviation(v)
		ogIdx := vIdx //Maintain the original index
		//Check the bit occupancy and flag when location is valid
		b := s.GetIndexBit(vIdx)
		if !b {
			s.IdxPos[vIdx] = i
		} else {
			//Perform Positional Seek only seek len(Positions) times max
			locationFound := false
			seekLeft := false
			seeks := 0
			for !locationFound && seeks < s.P {

				//Check if we have hit any bounds
				bitAvail := false
				if !seekLeft {
					vIdx++
				} else {
					vIdx--
				}
				if vIdx >= s.P-1 {
					vIdx = ogIdx
					seekLeft = true
				}

				//See if the current bit is available
				bitAvail = s.GetIndexBit(vIdx)
				if bitAvail {
					s.IdxPos[vIdx] = i
					s.FlagIndexBit(vIdx)
					locationFound = true
				}
				seeks++

				if seeks > s.P {
					return fmt.Errorf("SPD Sampler Bit Location Not Found!\n")
				}

			} //End Bounds search
		} //End  Else Bit not available

	} //End Particle Array Loop
	return nil
} //End Function

func (s *SPDSampler) ClearOccupancy() {
	for i := 0; i < len(s.Occup); i++ {
		s.Occup[i] = 0
	}
}

//Flags
func (s *SPDSampler) FlagIndexBit(idx int) {
	//Create the masking uint at the required index
	mask := uint64(1)
	nmask := uint64(0)
	idx2buf := int(idx / s.P)           //floored value
	offset := (idx % 64)                //bit offset should be 0 - 64
	occupIndexValue := s.Occup[idx2buf] //current bit state

	if s.BigEndian {
		nmask = mask >> offset
	} else {
		nmask = mask << (64 - offset)
	}

	nIdxValue := occupIndexValue | nmask //OR'D
	s.Occup[idx2buf] = nIdxValue
	return
}

func (s *SPDSampler) GetIndexBit(idx int) bool {
	//Create the masking uint at the required index
	//Create the masking uint at the required index
	mask := uint64(1)
	nmask := uint64(0)
	idx2buf := int(idx / s.P)           //floored value
	offset := (idx % 64)                //bit offset should be 0 - 64
	occupIndexValue := s.Occup[idx2buf] //current bit state
	if s.BigEndian {
		nmask = mask >> offset
	} else {
		nmask = mask << (64 - offset)
	}
	val := occupIndexValue & nmask //AND'D
	if val == 1 {
		return true
	}
	return false
}

//Checks the byte order specifications of the system returns true if big endian
func (s *SPDSampler) CheckEndian() bool {
	var i int = 0x0100
	ptr := unsafe.Pointer(&i)
	if 0x01 == *(*byte)(ptr) {
		//Big Endian
		return true
	} else if 0x00 == *(*byte)(ptr) {
		//little endian
		return false
	}
	return false
}
