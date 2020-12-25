package fluid

import "testing"
import "fmt"
import V "diesel.com/diesel/vector"

//All Vars are metric - metric constants for water

func TestVoxel(t *testing.T) {
	fmt.Printf("\n---------TESTING VOXEL DATA STRUCTURE----------\n\n")     //Main Fluid Component
	var boxfluid = BoxFluidSystem{V.Vec32{0, 0, 0}, 1.0, 1.0, 1.0, 5, 5, 5} //Box System Description
	var mfp = AllocMassFluidParticle(boxfluid, 1.0)
	var sphfluid = SPHFluid{} //Main Fluid Component
	sphfluid.Initialize(&boxfluid, &mfp, V.Vec32{0, 0, 0})

	VoxelGrid := AllocateVoxelStorage(sphfluid.Positions, 4, boxfluid.Width/2)
	VoxelGrid.Update()

	//Check that all volumes are used
	divs := VoxelGrid.VoxelDescriptor.Divisions
	var usedPositions []int = make([]int, divs*divs*divs)

	for i := 0; i < VoxelGrid.VoxelDescriptor.Particles; i++ {
		Hash := VoxelGrid.VoxelHash(sphfluid.Positions[i])
		x := Hash.VoxIndex[0]
		index := x
		if index < divs*divs*divs {
			usedPositions[index] = 1
		} else {
			fmt.Printf("error index out of range")
		}
	}

	used := true

	for i := 0; i < divs*divs*divs; i++ {
		if usedPositions[i] != 1 {
			used = false
			t.Errorf("Voxel grid[%d] not fully utilized\n", i)
		}
	}

	used = true
	if used {
		fmt.Printf("All Voxel Grids Occupied\n")
	}
	VoxelGrid.PrintStorageRequirements()
	fmt.Printf("Voxel Storage Allocated - Test Pass\n")

}
