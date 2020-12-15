//Kernel Testing MapIndexes
package fluid

import "testing"
import "fmt"

func TestKernel(t *testing.T) {
	var cubicKernel CubicKernel = InitCubic(15.0)

	//Test Kernel Weights Given Distance
	var dist [6]float32

	dist[0] = 0.1
	dist[1] = 0.3
	dist[2] = 0.4
	dist[3] = 0.5
	dist[4] = 1.2
	dist[5] = 0.01

	for i := 0; i < 6; i++ {
		weight := cubicKernel.F(dist[i])
		fmt.Printf("F(%f) = %f\n", dist[i], weight)
	}

}
