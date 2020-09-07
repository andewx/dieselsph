package app

import (
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
