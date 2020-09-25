package app

import (
	"testing"
)

//Testing SPH Routines with OpenGL Program Windo
func TestSPH(t *testing.T) {
	DefaultConfig := DefaultDslFl()
	FluidRenderer := InitRenderer(DefaultConfig)
	FluidRenderer.Run()
}
