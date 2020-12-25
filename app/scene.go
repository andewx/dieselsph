package app

//Manages Fluid Scene Routine - Composes Concurrency Demands and Animation - Timing Settings
import (
	F "diesel.com/diesel/fluid"
	U "diesel.com/diesel/utils"
	V "diesel.com/diesel/vector"
	"fmt"
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
	SPD_THREAD_SYNCED    = 61 // Spatial Hash Grid Thread Synced
	SPD_THREAD_RUN       = 60 //Spatial Hash Grid Thread Working.
	DENSITY_THREAD_RUN   = 100
	DENSITY_THREAD_WAIT  = 101
	ERROR                = 404
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
	PrtlMass     float32 //Particle Dynamics - Scaled Mass / Density
	PrtlRadius   float32 //Particle Radius
	KrnlRadius   float32 //Particle Kernel Radius
	EOSDens      float32 //Particle State Density
	SOS          float32 //Particle Speed of Sound
	Viscos       float32 //Particle Viscosity
	DrwMde       int     //particle draw mode
	Samples      int     //particle samples to take.
	PCISamples   int     //PCI Correction Steps
	InitialVel   V.Vec32 //
}

//Seconds timer for animation
type AnimationTimer struct {
	AppStart         time.Time //Time Application Started Secionds
	CurrentTime      time.Time //Last Polled Time
	LastParticleSync time.Time //Last Time Particles Were Sync'd
	LastSamplerSync  time.Time
	LastDensitySync  time.Time
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
	PSync             = 0.04166  //seconds (24fps update
	H20Mass           = 1000.0   //kg/m3 (metric mass vol)
	H20Visc           = 0.000091 //kg*(m/s)
	H20Kern           = 0.25     //Most important var - fine tune for fluid
	H20LiqDensity     = 1.0      //g/cm^3
	SOS               = 1400.0   //m/s (maximal information transfer) 1480 m/s with sounds
	Particles         = 20       //15,625 Particles -- 1.9MB Positional Data Ram
	DefaultTimeStep   = 0.00001  //Evolution at Small Interval
	EOSGamma          = 7        //Equation of State Exponent Feature
	PCISamples        = 10
	SPHSamples        = 10
	ParticleRadius    = 0.1 //Particle Radius (10 CM Particle)
	SPHScaleParticles = 0.5
)

//Cubic Volume :)
func GetVolume(radius float32) float32 {
	return (radius * radius * radius)
}

//Initializes Default Fluisd Structure During Initialization
func DefaultDslFl() *DslFlConfig {
	//Syncs at 24 Frames with a 60FPS runtime. 0.041 Seconds
	KParticleRad := float32(SPHScaleParticles / Particles)
	HKern := float32(KParticleRad * 18)
	return &DslFlConfig{Particles, V.Vec32{0.0, 0.0, -2.5}, 1.0, SPHScaleParticles, 5.0, PSync, H20Mass, KParticleRad, HKern, H20LiqDensity, SOS, H20Visc, PARTICLE_POINT, SPHSamples, PCISamples, V.Vec32{0, 0, 0.0}}
}

func RenderFluidGL(config *DslFlConfig) error {

	//Fluid Setup
	var boxfluid = F.BoxFluidSystem{config.MdlOrg, config.MdlW, config.MdlW, config.MdlW, config.CU, config.CU, config.CU}
	var mfp = F.AllocMassFluidParticle(boxfluid, SPHScaleParticles)
	var sphfluid = F.SPHFluid{} //Main Fluid Component
	sphfluid.Initialize(&boxfluid, &mfp, config.InitialVel)
	var thisTimer = AnimationTimer{time.Now(), time.Now(), time.Now(), time.Now(), time.Now()}
	var thisFluid = DSLFluidRenderer{&sphfluid, DefaultDslFl(), &thisTimer, make(map[string]int), make(map[string]V.Vec32), make(map[string]V.Mat4), make(map[string]float32), nil}

	//Scale the Positions
	U.ScalePositions(thisFluid.SPH.Positions, thisFluid.Config.MdlOrg, thisFluid.Config.PrtlScale)
	//Set OpenGL Windowing Context with GLFW and GO-GL Bindings
	glWindowProperties := AppWindow{1920, 1080, "Diesel Particle SPH"}
	//Perform OpenGL Setup..Need to Lock Thread fr all OpenGL Context Calls
	runtime.LockOSThread()
	window := InitGLFW(&glWindowProperties)
	if window == nil {
		return fmt.Errorf("Could not Initiate GLFW Context Window\n")
	}
	defer glfw.Terminate()
	dieselContext, err := InitOpenGL(thisFluid.SPH) //Bind buffers to Initialized Fluid Block.
	if checkError(err) {
		fmt.Printf("Could Not Initial OPENGL CONTEXT\n\n")
		return err
	}

	dieselContext.RotOrigin = &config.MdlOrg
	dieselContext.GLFWindow = window
	thisFluid.Context = dieselContext
	window.SetKeyCallback(ProcessInput)
	thisFluid.Run()
	return nil
}

//Main Fluid Run Loop -- Must Run inside same thread as Initiation Function
func (this *DSLFluidRenderer) Run() {

	var fluidStatus chan int = make(chan int)
	var samplerStatus chan int = make(chan int)
	var densityStatus chan int = make(chan int)
	spdinterval := float64(0.09) //Sync Every 2.0 Seconds
	densityinterval := float64(0.01)
	samplerError := false

	//Thread Most of the Fluid Routines // For Example Collision // Density Updates Etc
	go this.SPH.Compute(fluidStatus, this.Config.PrtlInterval)
	go this.SPH.ComputeDensities(densityStatus)
	go this.SPH.Sampler.Run(samplerStatus)

	this.Anim.LastSamplerSync = time.Now()
	this.Anim.LastDensitySync = time.Now()

	//Animation / GL Main Entry
	for !this.Context.GLFWindow.ShouldClose() {
		this.Anim.CurrentTime = time.Now()
		Draw(this.SPH, this.Context, this.Anim, this.Config.PrtlInterval, fluidStatus)

		//Update the Spatial Sampler Thread
		if time.Now().Sub(this.Anim.LastSamplerSync).Seconds() > spdinterval && !samplerError {
			sampleSyncStatus := <-samplerStatus
			if sampleSyncStatus == SPD_THREAD_SYNCED {
				samplerStatus <- SPD_THREAD_RUN
				this.Anim.LastSamplerSync = time.Now()
			}
			if sampleSyncStatus == ERROR {
				samplerError = true
			}

		}

		//Update the Density Thread
		if time.Now().Sub(this.Anim.LastDensitySync).Seconds() > densityinterval {
			densitySync := <-densityStatus
			if densitySync == DENSITY_THREAD_WAIT {
				densityStatus <- DENSITY_THREAD_RUN
				this.Anim.LastDensitySync = time.Now()
			}
		}

	} //End GL render loop
} //End func
