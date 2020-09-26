package app

//OpenGL Windowing Calls and Structs
import (
	F "diesel.com/diesel/fluid"
	//U "diesel.com/diesel/utils"
	V "diesel.com/diesel/vector"
	"fmt"
	"github.com/go-gl/gl/v4.1-core/gl" //go does some@latest found weird shit. If fixed go back to v4.1-core. Clean modcache if this doesn't wor
	"github.com/go-gl/glfw/v3.2/glfw"
	"io/ioutil"
	"log"
	"strings"
	"time"
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
	PrgID          [2]uint32
	VAO            [2]uint32
	VBO            [2]uint32
	Model          *V.Mat4
	View           *V.Mat4
	Proj           *V.Mat4
	ModelShaderLoc [2]int32
	ViewShaderLoc  [2]int32
	ProjShaderLoc  [2]int32
	ModeLoc        int32
	Frames         int32
	GLFWindow      *glfw.Window
	VertexSRC      string
	FragSRC        string
}

//Global Pointers :)))))))))))))) - Debugger Apparently Can't Handle This
var GlobalTrans *V.Vec32
var GlobalCamera *Camera

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
func InitOpenGL(sph *F.SPHFluid) (*DieselContext, error) {

	if err := gl.Init(); err != nil {
		return nil, err
	}

	dslContext := DieselContext{}
	version := gl.GoStr(gl.GetString(gl.VERSION))
	log.Println("OpenGL version", version)

	//Load Shader Strings from files
	vtxFile := "../shaders/ParticleVTX.glsl"
	frgFile := "../shaders/ParticleFRG.glsl"

	//Easy Load Strings
	sourceVTX, err := ioutil.ReadFile(vtxFile)
	if checkError(err) {
		return nil, err
	}
	sourceFRG, err := ioutil.ReadFile(frgFile)
	if checkError(err) {
		return nil, err
	}

	dslContext.VertexSRC = string(sourceVTX)
	dslContext.FragSRC = string(sourceFRG)

	//Handled by local function compile Shader
	vtxSHO, err := compileShader(dslContext.VertexSRC, gl.VERTEX_SHADER)
	if checkError(err) {
		return nil, err
	}
	frgSHO, err := compileShader(dslContext.FragSRC, gl.FRAGMENT_SHADER)
	if checkError(err) {
		return nil, err
	}

	prog1 := gl.CreateProgram()
	gl.AttachShader(prog1, vtxSHO)
	gl.AttachShader(prog1, frgSHO)
	gl.LinkProgram(prog1)

	//Free up context Strings
	//dslContext.VertexSRC = ""
	//	dslContext.FragSRC = ""

	//Generate Diesel Context which includes cameras an stuff
	n := float32(1.3)
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

	dslContext.Cam = GlobalCamera
	dslContext.Model = &model
	dslContext.View = &view
	dslContext.Proj = &proj
	dslContext.PrgID[0] = prog1
	//have to fucking wrangle a utf-8 out of a string again
	dslContext.ModelShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("model\x00"))
	dslContext.ViewShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("view\x00"))
	dslContext.ProjShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("projection\x00"))
	dslContext.ProjShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("projection\x00"))
	dslContext.ModeLoc = gl.GetUniformLocation(prog1, gl.Str("mode\x00"))
	//View/World matrix

	//VBO/VAO handle

	MakeVAO(sph, &dslContext)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

	fmt.Printf("OpenGL Initiated\n")
	//Pass uint32
	return &dslContext, nil
}

func Draw(sph *F.SPHFluid, dsl *DieselContext, anim *AnimationTimer, interval float64, c chan int) {

	//Timer Function in Seconds
	//elapse_s := lastTime.Sub(time.Now()).Seconds()
	//fps := 1 / elapse_s

	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(dsl.PrgID[0])

	//Set uniforms
	dsl.View.Translation(GlobalTrans) //Should accumulate Translationinto the View Matrix
	GlobalTrans[0] *= 0
	GlobalTrans[1] *= 0
	GlobalTrans[2] *= 0

	//Sync Particle System With Thread
	if time.Now().Sub(anim.LastParticleSync).Seconds() >= interval {
		status := <-c //Fluid should be synced
		if status == FLUID_THREAD_SYNCED {
			gl.BindVertexArray(dsl.VAO[0])
			gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])
			gl.BufferSubData(gl.ARRAY_BUFFER, 0, int(sph.Count)*4*3, gl.Ptr(&sph.Positions[0][0]))
			gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)
		}
		anim.LastParticleSync = time.Now()

		//Clear and update Grid
		//	sph.SPHGrid.Clear()
		//	sph.SPHGrid.Load(sph.Positions)

		c <- FLUID_THREAD_WORKING //Signal the channel for Fluid to Continue
	}

	gl.UniformMatrix4fv(dsl.ModelShaderLoc[0], 1, false, &dsl.Model[0])
	gl.UniformMatrix4fv(dsl.ViewShaderLoc[0], 1, false, &dsl.View[0])
	gl.UniformMatrix4fv(dsl.ProjShaderLoc[0], 1, false, &dsl.Proj[0])
	gl.Uniform1i(dsl.ModeLoc, 0) // Draw opaque particles

	gl.PointSize(3.0)

	//Bind Particles & Draw
	gl.BindVertexArray(dsl.VAO[0])
	gl.DrawArrays(gl.POINTS, 0, int32(sph.Count))
	//Bind Collision Geometry & Draw
	gl.Uniform1i(dsl.ModeLoc, 1) //Transparent Lit Surface
	gl.BindVertexArray(dsl.VAO[1])

	gl.DrawArrays(gl.TRIANGLES, 0, int32(len(sph.Colliders.Vertexes)))
	glfw.PollEvents()
	dsl.GLFWindow.SwapBuffers()

}

//Refreshes VBO object with positional da
func MakeVAO(sph *F.SPHFluid, dsl *DieselContext) {

	//Generate Two Object Buffer References
	gl.GenBuffers(2, &dsl.VBO[0])

	//Generate two Separate VAO Contexts // Bind the First VAO
	gl.GenVertexArrays(2, &dsl.VAO[0])
	gl.BindVertexArray(dsl.VAO[0])
	//Bind Vertex Attributes To The Currently Bound Buffer
	//First bind the buffer as a context for the VAO
	gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[0])                                                   //Just use pointer arithmetic i guess
	gl.BufferData(gl.ARRAY_BUFFER, sph.Count*4*3, gl.Ptr(&sph.Positions[0][0]), gl.DYNAMIC_DRAW) //float 32 (4 bytes)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)

	//Now Bind All The Collision Buffer Geometry
	gl.BindVertexArray(dsl.VAO[1])
	gl.BindBuffer(gl.ARRAY_BUFFER, dsl.VBO[1])
	gl.BufferData(gl.ARRAY_BUFFER, 4*len(sph.Colliders.Vertexes)*3, gl.Ptr(&sph.Colliders.Vertexes[0][0]), gl.STATIC_DRAW)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, nil)

}

func checkError(err error) bool {
	if err != nil {
		fmt.Printf(err.Error())
		return true
	}
	return false
}

func compileShader(source string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)
	csources, free := gl.Strs(source)
	gl.ShaderSource(shader, 1, csources, nil)
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))
		free()
		return 0, fmt.Errorf("GLSL Shader failed to compile\n: %v", log)
	}

	return shader, nil
}

func ProcessInput(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	cameraSpeed := float32(0.1) // adjust accordingly
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
