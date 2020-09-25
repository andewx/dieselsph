package app

import (
	"testing"
)

//Testing SPH Routines with OpenGL Program Windo
func TestSPH(t *testing.T) {
	DefaultConfig := DefaultDslFl()

	//Application Thread Render
	RenderFluidGL(DefaultConfig)
}
