package app

//OpenGL Windowing Calls and Structs
import (
	"fmt"
	"github.com/go-gl/gl/v4.1-core/gl" //go does some @latest found weird shit. If fixed go back to v4.1-core. Clean modcache if this doesn't work
	"github.com/go-gl/glfw/v3.2/glfw"
	"io/ioutil"
	"log"
	"strings"
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
	checkError(err)
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

	//Load Shader Strings from files
	vtxFile := "../shaders/ParticleVTX.glsl"
	frgFile := "../shaders/ParticleFRG.glsl"

	//Easy Load Strings
	sourceVTX, err := ioutil.ReadFile(vtxFile)
	checkError(err)
	sourceFRG, err := ioutil.ReadFile(frgFile)
	checkError(err)

	vtxSHO, err := compileShader(string(sourceVTX), gl.VERTEX_SHADER)
	frgSHO, err := compileShader(string(sourceFRG), gl.FRAGMENT_SHADER)

	checkError(err)

	prog := gl.CreateProgram()
	gl.AttachShader(prog, vtxSHO)
	gl.AttachShader(prog, frgSHO)
	gl.LinkProgram(prog)
	return prog
}

func Draw(window *glfw.Window, program uint32, FluidScene *SPHFluid) {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(program)

	glfw.PollEvents()
	window.SwapBuffers()
}

func checkError(err error) {
	if err != nil {
		panic(err)
	}
}

func compileShader(source string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)

	csources, free := gl.Strs(source)
	gl.ShaderSource(shader, 1, csources, nil)
	free()
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))

		return 0, fmt.Errorf("failed to compile %v: %v", source, log)
	}

	return shader, nil
}
