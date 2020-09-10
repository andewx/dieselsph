package app

//OpenGL Windowing Calls and Structs
import (
	F "diesel.com/diesel/fluid"
	V "diesel.com/diesel/vector"
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

type Camera struct {
	Pos   V.Vec32
	Front V.Vec32
	Up    V.Vec32
}

type DieselContext struct {
	Cam            *Camera
	PrgID          uint32
	VAO            uint32
	VBO            [2]uint32
	Model          *V.Mat4
	View           *V.Mat4
	Proj           *V.Mat4
	ModelShaderLoc int32
	ViewShaderLoc  int32
	ProjShaderLoc  int32
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
func InitOpenGL(sph *F.SPHFluid) *DieselContext {

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

	//Generate Diesel Context which includes cameras an stuff
	n := float32(-1.0)
	f := float32(-20.0)
	r := float32(20.0)
	l := float32(-20.0)
	t := float32(10.0)
	b := float32(-10.0)

	newCamera := Camera{V.Vec32{0, 0, 10.0}, V.Vec32{0, 0.0, -1.0}, V.Vec32{0, 1.0, 0}}
	model := V.Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} //identiy
	view := V.NewMat4(newCamera.Pos, newCamera.Front, newCamera.Up, V.Vec32{1.0, 1.0, 1.0})
	proj := V.ProjectionMatrix(l, r, t, b, n, f)
	dslContext := DieselContext{}
	dslContext.Cam = &newCamera
	dslContext.Model = &model
	dslContext.View = view
	dslContext.Proj = proj
	dslContext.PrgID = prog

	//have to fucking wrangle a utf-8 out of a string again
	dslContext.ModelShaderLoc = gl.GetUniformLocation(prog, StringToUTF8("model"))
	dslContext.ViewShaderLoc = gl.GetUniformLocation(prog, StringToUTF8("view"))
	dslContext.ProjShaderLoc = gl.GetUniformLocation(prog, StringToUTF8("proj"))

	//View/World matrix

	//VBO/VAO handle
	gl.GenBuffers(2, &dslContext.VBO[0])
	gl.GenVertexArrays(1, &dslContext.VAO)
	gl.BindVertexArray(dslContext.VAO)

	//Pass uint32
	return &dslContext
}

func StringToUTF8(str string) *uint8 {
	var ptr *uint8
	conv := []uint8(str)
	ptr = &conv[0]
	return ptr
}

func Draw(window *glfw.Window, FluidScene *F.SPHFluid, dslContext *DieselContext) {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(dslContext.PrgID)
	//Set uniforms
	gl.UniformMatrix4fv(dslContext.ModelShaderLoc, 1, false, &dslContext.Model[0])
	gl.UniformMatrix4fv(dslContext.ViewShaderLoc, 1, false, &dslContext.View[0])
	gl.UniformMatrix4fv(dslContext.ProjShaderLoc, 1, false, &dslContext.Proj[0])

	MakeVAO(FluidScene, dslContext)
	gl.PointSize(20.0)
	gl.BindVertexArray(dslContext.VAO)
	gl.DrawArrays(gl.POINTS, 0, int32(FluidScene.Count))
	glfw.PollEvents()
	window.SwapBuffers()
}

//Refreshes VBO object with positional data
func MakeVAO(sph *F.SPHFluid, dsl *DieselContext) {
	gl.EnableVertexAttribArray(0)
	gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])                                         //Just use pointer arithmetic i guess
	gl.BufferData(gl.ARRAY_BUFFER, sph.Count*4, gl.Ptr(sph.Positions), gl.STATIC_DRAW) //float 32 (4 bytes)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)                              /* Coord data VBO[0], 3 Verts per coord */
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

/*
func processInput(dsl *DieselContext) {
	cameraSpeed := 0.05 // adjust accordingly
	if glfw.GetKeyName(glfw.KEYW) == glfw.Press {
		dsl.Cam.Pos.Add(dsl.Cam.Front.Scale(cameraSpeed))
	}
	if glfw.GetKeyName(glfw.KEYS) == glfw.Press {
		dsl.Cam.Pos.Add(dsl.Cam.Front.Scale(-cameraSpeed))
	}
	if glfw.GetKeyName(glfw.KeyA) == glfw.Press {
		dsl.Cam.Pos.Add(V.Cross(dsl.Cam.Front, dsl.Cam.Up).Scale(cameraSpeed))
	}
	if glfw.GetKeyName(glfw.KeyD) == glfw.Press {
		dsl.Cam.Pos.Add(V.Cross(dsl.Cam.Front, dsl.Cam.Up).Scale(-cameraSpeed))
	}
}
*/
