package app

//OpenGL Windowing Calls and Structs
import (
	"github.com/go-gl/gl/v4.1-core/gl" //go does some @latest found weird shit. If fixed go back to v4.1-core. Clean modcache if this doesn't work
	"github.com/go-gl/glfw/v3.2/glfw"
	"log"
)

type AppWindow struct {
	Width  int
	Height int
	Name   string
}

// initGlfw initializes glfw and returns a Window to use.
func InitGLFW(a *AppWindow) *glfw.Window {
	if err := glfw.Init(); err != nil {
		panic(err)
	}

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 4) // OR 2
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(a.Width, a.Height, a.Name, nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()

	return window
}

// initOpenGL initializes OpenGL and returns an intiialized program.
func InitOpenGL() uint32 {
	if err := gl.Init(); err != nil {
		panic(err)
	}
	version := gl.GoStr(gl.GetString(gl.VERSION))
	log.Println("OpenGL version", version)

	vtxFile := "../shaders/ParticleVTX.glsl"
	frgFile := "../shaders/ParticleFRG.glsl"

	prog := gl.CreateProgram()
	gl.LinkProgram(prog)
	return prog
}

func Draw(window *glfw.Window, program uint32, FluidScene *SPHFluid) {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(program)

	glfw.PollEvents()
	window.SwapBuffers()
}
