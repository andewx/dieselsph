package app

import (
	F "diesel.com/diesel/fluid"
	V "diesel.com/diesel/vector"
	"github.com/go-gl/glfw/v3.2/glfw"
	"runtime"
	"testing"
	"time"
)

//Testing SPH Routines with OpenGL Program Windo
func TestSPH(t *testing.T) {

	//// TODO:
	//1. Initialize Bounded Particle Setup (HIGHER LEVEL MODULE) - Implement New Modules (Particles / Grids / Etc)
	//2. Initialize Collider Geometry (Implement New Modules)
	//3. Export Particles & Geometry Into GL Buffer Objects for Rendering (Also Work on Fluid Iso Surfaces
	//4. Texture Data Module
	//4. Bind GL Programs to Geometry & Rende

	//Fluid Setup
	var mfp = F.MassFluidParticle{0.001, 0.3, 0.1, 0.5, 0.1 / 1500, 1500, 1, 1.4} //Particle Mass Description
	var boxfluid = F.BoxFluidSystem{V.Vec32{0, 0, -5}, 2, 2, 2, 9, 9, 9}          //Box System Description
	var sphfluid = F.SPHFluid{}                                                   //Main Fluid Component

	sphfluid.Initialize(&boxfluid, &mfp)
	//Set OpenGL Windowing Context with GLFW and GO-GL Bindings
	glWindowProperties := AppWindow{1440, 800, "Diesel Particle SPH"}
	runtime.LockOSThread() //OpenGL can only handle one thread contex
	window := InitGLFW(&glWindowProperties)
	defer glfw.Terminate()
	dieselContext := InitOpenGL(&sphfluid)
	window.SetKeyCallback(ProcessInput)
	lastT := time.Now()
	seconds := 0.0 //Accumulates Seconds

	for !window.ShouldClose() {
		elapsed_s := lastT.Sub(time.Now()).Seconds()
		seconds += elapsed_s
		lastT = time.Now()

		sphfluid.Compute()
		Draw(window, &sphfluid, dieselContext, lastT)
	}
	//OpenGL Drawing Routine Don
}
