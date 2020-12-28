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
	"math"
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
	Cam             *Camera
	PrgID           [2]uint32
	VAO             [2]uint32
	VBO             [2]uint32
	Model           *V.Mat4
	View            *V.Mat4
	Proj            *V.Mat4
	ModelShaderLoc  [2]int32
	ViewShaderLoc   [2]int32
	ProjShaderLoc   [2]int32
	RotShaderLoc    [4]int32
	RotOriginLoc    [4]int32
	Frames          int32
	GLFWindow       *glfw.Window
	VertexSRC       string
	FragSRC         string
	GeomVertexSRC   string
	GeomFragSRC     string
	RotX            *V.Mat3
	RotY            *V.Mat3
	RotOriginTrans  *V.Mat4 //Positive Translation
	RotOriginTrans0 *V.Mat4 //Negative Translation
	RotOrigin       *V.Vec32
}

//Global Pointers :)))))))))))))) - Debugger Apparently Can't Handle This
var GlobalTrans *V.Vec32
var RotateTime time.Time
var RotateTimeLast time.Time
var state_hold_mouse bool
var GlobalCamera *Camera
var xT float64
var yT float64
var RotAngleX float32
var RotAngleY float32
var Fps float64
var lastTime time.Time
var animationTime float64

// initGlfw initializes glfw and returns a Window to use.
func InitGLFW(a *AppWindow) *glfw.Window {
	if err := glfw.Init(); err != nil {
		return nil
	}

	state_hold_mouse = false
	RotateTime = time.Now()
	lastTime = time.Now()
	animationTime = 0.0

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 4) // OR 2
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(a.Width, a.Height, a.Name, nil, nil)
	checkError(err)
	window.MakeContextCurrent()
	window.SetKeyCallback(ProcessInput)
	window.SetMouseButtonCallback(ProcessMouse)
	window.SetCursorPosCallback(ProcessCursor)

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
	vtxFile := "../shaders/sph.vert.glsl"
	frgFile := "../shaders/sph.frag.glsl"
	geomVtxFile := "../shaders/geom.vert.glsl"
	geomFrgFile := "../shaders/geom.frag.glsl"

	//Easy Load Strings
	sourceVTX, err := ioutil.ReadFile(vtxFile)
	if checkError(err) {
		return nil, err
	}
	sourceFRG, err := ioutil.ReadFile(frgFile)
	if checkError(err) {
		return nil, err
	}

	geomVTX, err := ioutil.ReadFile(geomVtxFile)
	if checkError(err) {
		return nil, err
	}
	geomFRG, err := ioutil.ReadFile(geomFrgFile)
	if checkError(err) {
		return nil, err
	}

	dslContext.VertexSRC = string(sourceVTX) + "\x00"
	dslContext.FragSRC = string(sourceFRG) + "\x00"
	dslContext.GeomVertexSRC = string(geomVTX) + "\x00"
	dslContext.GeomFragSRC = string(geomFRG) + "\x00"

	//COMPILE GLSL SHADER OBJECTS
	vtxSHO, err := compileShader(dslContext.VertexSRC, gl.VERTEX_SHADER)
	if checkError(err) {
		return nil, err
	}
	frgSHO, err := compileShader(dslContext.FragSRC, gl.FRAGMENT_SHADER)
	if checkError(err) {
		return nil, err
	}
	gVtxSHO, err := compileShader(dslContext.VertexSRC, gl.VERTEX_SHADER)
	if checkError(err) {
		return nil, err
	}
	gFrgSHO, err := compileShader(dslContext.FragSRC, gl.FRAGMENT_SHADER)
	if checkError(err) {
		return nil, err
	}

	prog1 := gl.CreateProgram()
	gl.AttachShader(prog1, vtxSHO)
	gl.AttachShader(prog1, frgSHO)
	gl.LinkProgram(prog1)

	//Generate Diesel Context which includes cameras an stuff
	n := float32(1.0)
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
	rotX := V.Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}
	rotY := V.Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}
	rot0 := V.Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
	rot1 := V.Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
	rotOrigin := V.Vec32{0, 0, 0}

	dslContext.Cam = GlobalCamera
	dslContext.Model = &model
	dslContext.View = &view
	dslContext.Proj = &proj
	dslContext.PrgID[0] = prog1
	dslContext.RotX = &rotX
	dslContext.RotY = &rotY
	dslContext.RotOriginTrans = &rot0
	dslContext.RotOriginTrans0 = &rot1
	dslContext.RotOrigin = &rotOrigin

	dslContext.ModelShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("model\x00"))
	dslContext.ViewShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("view\x00"))
	dslContext.ProjShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("projection\x00"))
	dslContext.RotShaderLoc[0] = gl.GetUniformLocation(prog1, gl.Str("rotX\x00"))
	dslContext.RotShaderLoc[1] = gl.GetUniformLocation(prog1, gl.Str("rotY\x00"))
	dslContext.RotOriginLoc[0] = gl.GetUniformLocation(prog1, gl.Str("rotOriginTrans0\x00")) //Positive Trans
	dslContext.RotOriginLoc[1] = gl.GetUniformLocation(prog1, gl.Str("rotOriginTrans1\x00")) //Negative Translation

	prog2 := gl.CreateProgram()
	gl.AttachShader(prog2, gVtxSHO)
	gl.AttachShader(prog2, gFrgSHO)
	gl.LinkProgram(prog2)
	dslContext.PrgID[1] = prog2

	//Query Active Uniforms in Program 2
	sizeN := int32(0)
	maxLength := int32(0)
	gl.GetProgramiv(prog2, gl.ACTIVE_UNIFORM_MAX_LENGTH, &maxLength)
	gl.GetProgramiv(prog2, gl.ACTIVE_UNIFORMS, &sizeN)

	constructString := ""

	for i := 0; i < int(maxLength); i++ {
		constructString = constructString + " "
	}
	constructString = constructString + "\x00"

	charBuf := gl.Str(constructString)
	sizebuf := int32(0)
	sizeVar := int32(0)
	typeVar := uint32(0)
	for i := 0; i < 7; i++ {
		gl.GetActiveUniform(prog2, uint32(i), maxLength, &sizebuf, &sizeVar, &typeVar, charBuf)
		varName := gl.GoStr(charBuf)
		fmt.Printf("Uniform[%d]: %s\n", i, varName)
	}
	gl.UseProgram(prog2)
	dslContext.ModelShaderLoc[1] = gl.GetUniformLocation(prog2, gl.Str("model\x00"))
	dslContext.ViewShaderLoc[1] = gl.GetUniformLocation(prog2, gl.Str("view\x00"))
	dslContext.ProjShaderLoc[1] = gl.GetUniformLocation(prog2, gl.Str("projection\x00"))
	dslContext.RotShaderLoc[2] = gl.GetUniformLocation(prog2, gl.Str("rotX\x00"))
	dslContext.RotShaderLoc[3] = gl.GetUniformLocation(prog2, gl.Str("rotY\x00"))
	dslContext.RotOriginLoc[2] = gl.GetUniformLocation(prog2, gl.Str("rotOriginTrans0\x00")) //Positive Trans
	dslContext.RotOriginLoc[3] = gl.GetUniformLocation(prog2, gl.Str("rotOriginTrans1\x00")) //Negative Translation

	MakeVAO(sph, &dslContext)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

	return &dslContext, nil
}

func SetRotMatrix(dsl *DieselContext) {
	cosAX := float32(math.Cos(float64(RotAngleX)))
	sinAX := float32(math.Sin(float64(RotAngleX)))
	cosAY := float32(math.Cos(float64(RotAngleY)))
	sinAY := float32(math.Sin(float64(RotAngleY)))

	dsl.RotX[4] = cosAX
	dsl.RotX[5] = sinAX
	dsl.RotX[7] = -sinAX
	dsl.RotX[8] = cosAX

	dsl.RotY[0] = cosAY
	dsl.RotY[2] = -sinAY
	dsl.RotY[6] = sinAY
	dsl.RotY[8] = cosAY

}

func Draw(sph *F.SPHFluid, dsl *DieselContext, anim *AnimationTimer, interval float64, c chan int) {

	//FPS
	elapse_s := lastTime.Sub(time.Now()).Seconds()
	Fps = 1 / elapse_s
	animationTime = sph.Timer.T

	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.ClearColor(0.9, 0.9, 0.9, 1.0)

	//-------------SET GPU GLOBAL TRANSFORMATIONS--------------------------------//

	dsl.View.Translation(GlobalTrans) //Should accumulate Translationinto the View Matrix
	dsl.RotOriginTrans[12] = dsl.RotOrigin[0]
	dsl.RotOriginTrans[13] = dsl.RotOrigin[1]
	dsl.RotOriginTrans[14] = dsl.RotOrigin[2]
	dsl.RotOriginTrans0[12] = -dsl.RotOrigin[0]
	dsl.RotOriginTrans0[13] = -dsl.RotOrigin[1]
	dsl.RotOriginTrans0[14] = -dsl.RotOrigin[2]

	SetRotMatrix(dsl)
	GlobalTrans[0] *= 0
	GlobalTrans[1] *= 0
	GlobalTrans[2] *= 0

	gl.UseProgram(dsl.PrgID[0])
	gl.UniformMatrix4fv(dsl.ModelShaderLoc[0], 1, false, &dsl.Model[0])
	gl.UniformMatrix4fv(dsl.ViewShaderLoc[0], 1, false, &dsl.View[0])
	gl.UniformMatrix4fv(dsl.ProjShaderLoc[0], 1, false, &dsl.Proj[0])
	gl.UniformMatrix3fv(dsl.RotShaderLoc[0], 1, false, &dsl.RotX[0])
	gl.UniformMatrix3fv(dsl.RotShaderLoc[1], 1, false, &dsl.RotY[0])
	gl.UniformMatrix4fv(dsl.RotOriginLoc[0], 1, false, &dsl.RotOriginTrans[0])
	gl.UniformMatrix4fv(dsl.RotOriginLoc[1], 1, false, &dsl.RotOriginTrans0[0])

	gl.UseProgram(dsl.PrgID[1])

	gl.UniformMatrix4fv(dsl.ModelShaderLoc[1], 1, false, &dsl.Model[0])
	gl.UniformMatrix4fv(dsl.ViewShaderLoc[1], 1, false, &dsl.View[0])
	gl.UniformMatrix4fv(dsl.ProjShaderLoc[1], 1, false, &dsl.Proj[0])
	gl.UniformMatrix3fv(dsl.RotShaderLoc[2], 1, false, &dsl.RotX[0])
	gl.UniformMatrix3fv(dsl.RotShaderLoc[3], 1, false, &dsl.RotY[0])
	gl.UniformMatrix4fv(dsl.RotOriginLoc[2], 1, false, &dsl.RotOriginTrans[0])
	gl.UniformMatrix4fv(dsl.RotOriginLoc[3], 1, false, &dsl.RotOriginTrans0[0])

	//-------------DRAW STATIC GEOMETRY--------------------------------//

	gl.BindVertexArray(dsl.VAO[1])
	gl.DrawArrays(gl.LINES, 0, int32(len(sph.Colliders.Vertexes)))

	//--------------SPH PARTICLE DRAW---------------------------------------
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
		c <- FLUID_THREAD_WORKING //Signal the channel for Fluid to Continue
	}

	gl.UseProgram(dsl.PrgID[0])
	gl.PointSize(sph.Mfp.KernelRadius * 400)
	gl.BindVertexArray(dsl.VAO[0])
	gl.DrawArrays(gl.POINTS, 0, int32(sph.Count))

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
		return 0, fmt.Errorf("GLSL Shader failed to compile\n: %v", log)
	}
	free()
	return shader, nil
}

func ProcessInput(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	cameraSpeed := float32(0.009) // adjust accordingly - Just use framerate

	trans := cameraSpeed

	if key == glfw.KeyW {
		//Accumulates the translation Vector
		p := V.Vec32{0.0, 0.0, -1.0}
		GlobalTrans.Add(*p.Scale(trans))
	}
	if key == glfw.KeyS {
		//Accumulates the translation Vector
		p := V.Vec32{0.0, 0.0, 1.0}
		GlobalTrans.Add(*p.Scale(trans))
	}
	if key == glfw.KeyA {
		//Accumulates the translation Vector
		//Accumulates the translation Vector
		p := V.Vec32{-1.0, 0.0, 0.0}
		GlobalTrans.Add(*p.Scale(trans))
	}
	if key == glfw.KeyD {
		//Accumulates the translation Vector
		p := V.Vec32{1.0, 0.0, 0.0}
		GlobalTrans.Add(*p.Scale(trans))
	}

	if key == glfw.KeyUp {
		p := V.Vec32{0.0, -1.0, 0.0}
		GlobalTrans.Add(*p.Scale(trans))
	}
	if key == glfw.KeyDown {
		p := V.Vec32{0.0, 1.0, 0.0}
		GlobalTrans.Add(*p.Scale(trans))
	}
	if key == glfw.KeyTab {
		fmt.Printf("Current Simulation Time: %f\n", animationTime)
	}
}

//Set Mouse Callba j
func ProcessMouse(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
	if button == glfw.MouseButtonLeft && action == glfw.Press {
		//Save the raw xy postion intial and continuously update and poll
		if !state_hold_mouse {
			state_hold_mouse = true
			RotateTime = time.Now()
			RotateTimeLast = time.Now()

		} else {
			RotateTimeLast = time.Now()
		}

	}
	if button == glfw.MouseButtonLeft && action == glfw.Release {
		state_hold_mouse = false
		xT = 0
		yT = 0
	}
}

func ProcessCursor(w *glfw.Window, xPos float64, yPos float64) {

	if state_hold_mouse {
		dt := RotateTimeLast.Sub(time.Now()).Seconds()
		if !(xT == 0) && !(yT == 0) {
			xdt := (xT - xPos) / dt
			ydt := (yT - yPos) / dt
			RotAngleX += float32(ydt / 100)
			RotAngleY += float32(xdt / 100)
		}
		xT = xPos
		yT = yPos
	}
}
