package utils

import "testing"
import V "diesel.com/diesel/vector"
import "unsafe"

func TestTransfer(t *testing.T) {
	var r = V.Vec32{1, 0, 0}
	var positionSlice = []V.Vec32{r, r, r, r, r, r, r, r, r, r, r, r, r, r, r, r, r, r, r, r}
	var v = V.Vec32{}
	uSlice := []V.Vec32{v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v}
	uPtr := unsafe.Pointer(&uSlice[0])

	TransferPositionData(uPtr, positionSlice, len(positionSlice)-1)

	for i := 0; i < len(positionSlice)-1; i++ {
		if !V.VecEquals(positionSlice[i], uSlice[i]) {
			t.Errorf("Improper Buffer Load at index %d\n", i)
		}
	}

}
