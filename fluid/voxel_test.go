package fluid

import "testing"
import "fmt"
import V "diesel.com/diesel/vector"
import "time"

//All Vars are metric - metric constants for water

func TestVoxel(t *testing.T) {
	fmt.Printf("\n---------TESTING VOXEL DATA STRUCTURE----------\n\n")
	var mfp = AllocMassFluidParticle(1000, 0.1, 0.4)                           //Main Fluid Component
	var boxfluid = BoxFluidSystem{V.Vec32{0, 0, 0}, 2.0, 2.0, 2.0, 15, 15, 15} //Box System Description
	var sphfluid = SPHFluid{}                                                  //Main Fluid Component
	sphfluid.Initialize(&boxfluid, &mfp, V.Vec32{0, 0, 0})

	VoxelGrid := AllocateVoxelStorage(sphfluid.Positions, 8, 15*15*15, 2.0)
	VoxelGrid.Update()

	//Check Thread Updates Run Update for 2.0 Seconds
	var threadUpdate chan int = make(chan int)

	go VoxelGrid.Run(threadUpdate)

	var timeLast time.Time = time.Now()

	for time.Now().Sub(timeLast).Seconds() < 1.00 {
		nStatus := <-threadUpdate //Wait on Message
		if nStatus == THREAD_WAIT_SAMPLER {
			threadUpdate <- THREAD_RUN_SAMPLER
		}
	}

	VoxelGrid.PrintStorageRequirements()
	fmt.Printf("Voxel Storage Allocated - Test Pass\n")

}
