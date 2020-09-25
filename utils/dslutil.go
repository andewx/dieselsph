package utils

import "unsafe"
import V "diesel.com/diesel/vector"
import "fmt"

//Application Specific Positional Data Transfer, Vec32 Array, Unsafe Access
//This is intended for a graphics memory buffer but may be streamed to any pointer from a go slice
func TransferPositionData(graphicsPtr unsafe.Pointer, posArray []V.Vec32, count int) error {
	//Convert graphicsPtr into an initial stream Vec32 *ptr.
	streamVecPtr := (*V.Vec32)(graphicsPtr)
	refVec := V.Vec32{}
	if count <= 0 || count >= len(posArray) {
		fmt.Printf("Size of positional data buffer transfer out of bounds: %d\n", count)
		return fmt.Errorf("Size of positional data buffer transfer out of bounds: %d\n", count)
	}

	if streamVecPtr == nil {
		return fmt.Errorf("No Valid Pointer to Graphics Memory Location\n")
		//or whatever location was the target
	}

	for i := 0; i < count; i++ {
		next := posArray[i]
		*streamVecPtr = next //is this a deep copy
		//Step streamVecPtr
		offset := unsafe.Sizeof(refVec) * uintptr(i+1)
		streamVecPtr = (*V.Vec32)(unsafe.Pointer(uintptr(graphicsPtr) + offset))
	}

	return nil
}

//Scales Position List Points Around an Origin
func ScalePositions(pos []V.Vec32, origin V.Vec32, scale float32) {

	//Complex Matrix Mult Didn't Work Completely with the Affine Portion Lol

	count := len(pos)

	for i := 0; i < count; i++ {
		v := pos[i]
		//Just Translate Normally
		v.Sub(origin)
		v.Scale(scale)
		v.Add(origin)

		pos[i] = v

	}

	return

}
