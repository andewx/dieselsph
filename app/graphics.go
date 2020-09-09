package app

//OpenGL Windowing Calls and Structs
import (
	F "diesel.com/diesel/fluid"
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
func InitOpenGL(sph *F.SPHFluid) uint32 {

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

	//Handled by local function compile Shader
	vtxSHO, err := compileShader(string(sourceVTX), gl.VERTEX_SHADER)
	frgSHO, err := compileShader(string(sourceFRG), gl.FRAGMENT_SHADER)

	checkError(err)

	prog := gl.CreateProgram()
	gl.AttachShader(prog, vtxSHO)
	gl.AttachShader(prog, frgSHO)
	gl.LinkProgram(prog)

	return prog
}

func Draw(window *glfw.Window, program uint32, FluidScene *F.SPHFluid) {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(program)
	vao := MakeVAO(FluidScene)

	gl.PointSize(30.0)
	gl.BindVertexArray(vao)
	gl.DrawArrays(gl.POINTS, 0, int32(FluidScene.Count))
	glfw.PollEvents()
	window.SwapBuffers()
}

func MakeVAO(sph *F.SPHFluid) uint32 {
	//when we move to GPU compute this will be each attribute in a VBO
	var vbo [2]uint32 //we will pass both the positions and velocity of each particle
	var vao uint32

	//VBO/VAO handle
	gl.GenBuffers(2, &vbo[0])
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo[0])                                              //Just use pointer arithmetic i guess
	gl.BufferData(gl.ARRAY_BUFFER, sph.Count*4, gl.Ptr(sph.Positions), gl.DYNAMIC_DRAW) //float 32 (4 bytes)
	gl.VertexAttribPointer(0, 3, gl.FLOAT_VEC3, false, 0, nil)                          /* Coord data VBO[0], 3 Verts per coord */
	gl.EnableVertexAttribArray(0)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo[1])                                               //Enable and bind second attribute buffer
	gl.BufferData(gl.ARRAY_BUFFER, sph.Count*4, gl.Ptr(sph.Velocities), gl.DYNAMIC_DRAW) //float 32 (4 bytes)
	gl.VertexAttribPointer(1, 3, gl.FLOAT_VEC3, false, 0, nil)
	gl.EnableVertexAttribArray(1)

	return vao
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
