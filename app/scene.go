package app

//Manages Fluid Scene Routine - Composes Concurrency Demands and Animation - Timing Settings
import (
	F "diesel.com/diesel/fluid"
	U "diesel.com/diesel/utils"
	V "diesel.com/diesel/vector"
	"github.com/go-gl/glfw/v3.2/glfw"
	"runtime"
	"time"
)

//Features List and Render Routines (Not Implemented Yet)
const (

	//Diesel Attribute Pointer Locations for VAO acce
	DSL_VERTEX   = 0
	DSL_INDICE   = 1
	DSL_NORMAL   = 2
	DSL_COLOR    = 3
	DSL_TEXCOORD = 4

	//Render Features
	PARTICLE_POINT              = 5
	PARTICLE_POINT_SPRITE       = 6
	PARTICLE_POINT_VOLUMETRIC   = 7
	COLLIDER_NO_RENDER          = 8
	COLLIDER_TRANSPARENT        = 9
	NORMALIZE_PARTICLE_POSITION = 10 //Normalizes Particle Position to the reference Grid Box so that Sph Grid Calculations are Predictable and Spread Even

	//Physics & Animation
	METRIC_SYS  = 11
	US_SYS      = 12
	DSL_RADIANS = 13
	DSL_DEGREES = 14

	STATE_PLAY  = 15
	STATE_PAUSE = 16

	//Logging // Text // Export
	KEYSTROKE_LOG = 17 //Keystroke Command Saves a Snapshot of the Buffer for Inspection
	EXPORT_PBRT   = 18 //Exports scene into a PBRT file for easy Render
	USE_KEYMAP    = 19 //Keymaps functions

	//Concurrency Sync
	FLUID_THREAD_SYNCED  = 20 //Fluid Thread Paused & Caught Up
	FLUID_THREAD_WORKING = 21 //Fluid Thread processing
	SHG_THREAD_SYNCED    = 22 // Spatial Hash Grid Thread Synced
	SHG_THREAD_WORKING   = 23 //Spatial Hash Grid Thread Working.

)

//Our configuration sturction for Initialization of the Renderer Duplicates Fluid Parameters so we can pass them
//from a user point of view
type DslFlConfig struct {
	CU           int     //Total Particles = ParticlesCU^3 (Cubic Root of Particle Size)
	MdlOrg       V.Vec32 //Model origin for initialization
	MdlW         float32 //Model dimensions w x h x d
	PrtlScale    float32 //Scale and Transform Initialized Particle Field towards origin (1 for no transform)
	GridInterval float64 //seconds Until SPH Grid is recalculated
	PrtlInterval float64 //seconds Until Particle CPU Vertex Buffer Is Updated
	PrtlMass     float32 //Particle Dynamics
	KrnlRadius   float32 //Particle Kernel Radius
	EOSDens      float32 //Particle State Density
	SOS          float32 //Particle Speed of Sound
	Viscos       float32 //Particle Viscosity
	DrwMde       int     //particle draw mode
	Samples      int     //particle samples to take.
	PCISamples   int     //PCI Correction Steps
}

//Seconds timer for animation
type AnimationTimer struct {
	AppStart         time.Time //Time Application Started Secionds
	CurrentTime      time.Time //Last Polled Time
	LastParticleSync time.Time //Last Time Particles Were Sync'd
	LastGridSync     time.Time //Last Time Grid Was synced.
}

//A Scene Graph Type Object Holds the Key Buffers. The Fluid Particle Buffer
//Is contained with in the fluid logic along with its collider box. This class
//Is a chance to add more graphical / collider world geometry and definitions
//And serve as a client to the GPU Graphics Side of House.
type DSLFluidRenderer struct {
	SPH          *F.SPHFluid  //Holds the key initialization
	Config       *DslFlConfig //Configuration Information for Initialization and Control
	Anim         *AnimationTimer
	GLSLMapInt   map[string]int
	GLSLMapVec3  map[string]V.Vec32
	GLSLMapMat4  map[string]V.Mat4
	GLSLMapFloat map[string]float32
	Context      *DieselContext
}

//All Vars are metric - metric constants for water
const (
	PSync           = 0.04166 //seconds (24fps update
	H20Mass         = 0.001   //kg/cm3
	H20Visc         = 1.0     //kg*(m/s)
	H20Kern         = .125    //Smoothing Kernel
	H20LiqDensity   = 0.001   //kg/cm^3
	SOS             = 10.01   //m/s (maximal information transfer) 1480 m/s with sound
	Particles       = 10      //15,625 Particles -- 1.9MB Positional Data Ram
	DefaultTimeStep = 0.1     //Evolution at Small Interval
	EOSGamma        = 7.3     //Equation of State Exponent Feature
	PCISamples      = 20
	SPHSamples      = 10
)

//Initializes Default Fluisd Structure During Initialization
func DefaultDslFl() *DslFlConfig {
	//Syncs at 24 Frames with a 60FPS runtime. 0.041 Seconds
	return &DslFlConfig{Particles, V.Vec32{0.0, 0.0, -5.0}, 2.0, 0.8, 5.0, PSync, H20Mass, H20Kern, H20LiqDensity, SOS, H20Visc, PARTICLE_POINT, SPHSamples, PCISamples}
}

func RenderFluidGL(config *DslFlConfig) error {

	//Fluid Setup
	var mfp = F.MassFluidParticle{config.PrtlMass, config.Viscos, config.KrnlRadius, config.KrnlRadius, DefaultTimeStep, config.SOS, config.EOSDens, EOSGamma}
	var boxfluid = F.BoxFluidSystem{config.MdlOrg, config.MdlW, config.MdlW, config.MdlW, config.CU, config.CU, config.CU} //Box System Description
	var sphfluid = F.SPHFluid{}                                                                                            //Main Fluid Component
	sphfluid.Initialize(&boxfluid, &mfp)
	var thisTimer = AnimationTimer{time.Now(), time.Now(), time.Now(), time.Now()}
	var thisFluid = DSLFluidRenderer{&sphfluid, DefaultDslFl(), &thisTimer, make(map[string]int), make(map[string]V.Vec32), make(map[string]V.Mat4), make(map[string]float32), nil}

	//Scale the Positions
	U.ScalePositions(thisFluid.SPH.Positions, thisFluid.Config.MdlOrg, thisFluid.Config.PrtlScale)
	//Set OpenGL Windowing Context with GLFW and GO-GL Bindings
	glWindowProperties := AppWindow{1440, 800, "Diesel Particle SPH"}

	//Perform OpenGL Setup..Need to Lock Thread for all OpenGL Context Calls
	runtime.LockOSThread()
	window := InitGLFW(&glWindowProperties)
	defer glfw.Terminate()
	dieselContext, err := InitOpenGL(thisFluid.SPH) //Bind buffers to Initialized Fluid Block.
	if checkError(err) {
		return err
	}
	dieselContext.GLFWindow = window
	thisFluid.Context = dieselContext
	window.SetKeyCallback(ProcessInput)
	thisFluid.Run()
	return nil
}

//Main Fluid Run Loop -- Must Run inside same thread as Initiation Function
func (this *DSLFluidRenderer) Run() {

	var threadStatus chan int = make(chan int)

	go this.SPH.Compute(threadStatus, this.Config.PrtlInterval)
	for !this.Context.GLFWindow.ShouldClose() {
		this.Anim.CurrentTime = time.Now()
		Draw(this.SPH, this.Context, this.Anim, this.Config.PrtlInterval, threadStatus)
	}
}
