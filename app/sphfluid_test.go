package app

import (
	F "diesel.com/diesel/fluid"
	V "diesel.com/diesel/vector"
	"github.com/go-gl/glfw/v3.2/glfw"
	"runtime"
	"testing"
)

//Testing SPH Routines with OpenGL Program Window
func TestSPH(t *testing.T) {

	//// TODO:
	//1. Initialize Bounded Particle Setup (HIGHER LEVEL MODULE) - Implement New Modules (Particles / Grids / Etc)
	//2. Initialize Collider Geometry (Implement New Modules)
	//3. Export Particles & Geometry Into GL Buffer Objects for Rendering (Also Work on Fluid Iso Surfaces)
	//4. Texture Data Module
	//4. Bind GL Programs to Geometry & Render

	//Fluid Setup
	var mfp = F.MassFluidParticle{0.01, 0.3, 0.02, 0.04, 0.05, 1500, 1, 1.4}
	var boxfluid = BoxFluidSystem{V.Vec32{}, 10, 10, 10, 10, 10, 10}
	var sphfluid = SPHFluid{}

	sphfluid.Initialize(&boxfluid, &mfp)

	//Set OpenGL Windowing Context with GLFW and GO-GL Bindings
	glWindowProperties := AppWindow{500, 500, "Diesel Particle SPH"}
	runtime.LockOSThread() //OpenGL can only handle one thread context
	window := InitGLFW(&glWindowProperties)
	defer glfw.Terminate()
	program := InitOpenGL()

	//MAIN ---------------------
	//
	//--------------------------
	for !window.ShouldClose() {
		Draw(window, program)
	}
	//OpenGL Drawing Routine Done
}
