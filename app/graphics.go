package app

//OpenGL Windowing Calls and Structs
import (
	F "diesel.com/diesel/fluid"
	V "diesel.com/diesel/vector"
	"fmt"
	"github.com/go-gl/gl/v4.1-core/gl" //go does some @latest found weird shit. If fixed go back to v4.1-core. Clean modcache if this doesn't wor
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
	Info  string
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
	Frames         int32
}

//Global Pointers :)))))))))))))) - Debugger Apparently Can't Handle This
var GlobalTrans *V.Vec32
var GlobalCamera *Camera

var (
	triangle = []float32{
		0, 0.5, 0, // top
		-0.5, -0.5, 0, // left
		0.5, -0.5, 0, // right
	}
)

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
	window.SetKeyCallback(ProcessInput)

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
	n := float32(1.1)
	f := float32(100.0)
	r := float32(20.0)
	l := float32(-20.0)
	t := float32(10.0)
	b := float32(-10.0)

	GlobalCamera = &Camera{V.Vec32{0, 0, 0}, V.Vec32{0, 0.0, 1.0}, V.Vec32{0, 1.0, 0}, "hi im a camera"}
	GlobalTrans = &V.Vec32{0, 0, 0}
	model := V.Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} //identity
	view := V.Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
	proj := V.ProjectionMatrix(l, r, t, b, n, f)
	dslContext := DieselContext{}
	dslContext.Cam = GlobalCamera
	dslContext.Model = &model
	dslContext.View = &view
	dslContext.Proj = &proj
	dslContext.PrgID = prog
	//have to fucking wrangle a utf-8 out of a string again
	dslContext.ModelShaderLoc = gl.GetUniformLocation(prog, gl.Str("model\x00"))
	dslContext.ViewShaderLoc = gl.GetUniformLocation(prog, gl.Str("view\x00"))
	dslContext.ProjShaderLoc = gl.GetUniformLocation(prog, gl.Str("projection\x00"))
	//View/World matrix

	//VBO/VAO handle

	MakeVAO(sph, &dslContext)
	fmt.Printf("Lets do this : )\n")
	//Pass uint32
	return &dslContext
}

func Draw(window *glfw.Window, sph *F.SPHFluid, dsl *DieselContext) {

	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(dsl.PrgID)
	//Set uniforms
	dsl.View.Translation(GlobalTrans) //Should accumulate Translationinto the View Matrix
	GlobalTrans[0] *= 0
	GlobalTrans[1] *= 0
	GlobalTrans[2] *= 0
	//Clear out translation Vector for smoothness
	if dsl.Frames%100 == 0 {
		gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])                         //Just use pointer arithmetic i guess
		gl.MapBufferRange(gl.ARRAY_BUFFER, 0, sph.Count, gl.MAP_WRITE_BIT) //
		gl.UnmapBuffer(gl.ARRAY_BUFFER)

	}

	gl.UniformMatrix4fv(dsl.ModelShaderLoc, 1, false, &dsl.Model[0])
	gl.UniformMatrix4fv(dsl.ViewShaderLoc, 1, false, &dsl.View[0])
	gl.UniformMatrix4fv(dsl.ProjShaderLoc, 1, false, &dsl.Proj[0])

	gl.PointSize(5.0)
	gl.BindVertexArray(dsl.VAO)
	gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])
	gl.DrawArrays(gl.POINTS, 0, int32(sph.Count)*3)
	glfw.PollEvents()
	window.SwapBuffers()

}

//Refreshes VBO object with positional da
func MakeVAO(sph *F.SPHFluid, dsl *DieselContext) {

	gl.GenBuffers(2, &dsl.VBO[0])
	gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])                                                  //Just use pointer arithmetic i guess
	gl.BufferData(gl.ARRAY_BUFFER, sph.Count*4*3, gl.Ptr(&sph.Positions[0][0]), gl.STREAM_DRAW) //float 32 (4 bytes)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)
	/* Coord data VBO[0], 3 Verts per coord */
	/*
		gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[1])
		gl.BufferData(gl.ARRAY_BUFFER, 4*len(triangle), gl.Ptr(triangle), gl.STATIC_DRAW)
		gl.EnableVertexAttribArray(0)
		gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)
	*/

	gl.GenVertexArrays(1, &dsl.VAO)
	gl.BindVertexArray(dsl.VAO)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)
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

func ProcessInput(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	cameraSpeed := float32(0.5) // adjust accordingly
	if key == glfw.KeyW {
		//Accumulates the translation Vector
		p := V.Vec32{0.0, 0.0, -1.0}
		GlobalTrans.Add(*p.Scale(cameraSpeed))
	}
	if key == glfw.KeyS {
		//Accumulates the translation Vector
		p := V.Vec32{0.0, 0.0, 1.0}
		GlobalTrans.Add(*p.Scale(cameraSpeed))
	}
	if key == glfw.KeyA {
		//Accumulates the translation Vector
		//Accumulates the translation Vector
		p := V.Vec32{-1.0, 0.0, 0.0}
		GlobalTrans.Add(*p.Scale(cameraSpeed))
	}
	if key == glfw.KeyD {
		//Accumulates the translation Vector
		p := V.Vec32{1.0, 0.0, 0.0}
		GlobalTrans.Add(*p.Scale(cameraSpeed))
	}
}
