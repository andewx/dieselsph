package fluid

import "testing"
import V "diesel.com/diesel/vector"
import "fmt"

//All Vars are metric - metric constants for water
const (
	PSync           = 0.04166  //seconds (24fps update
	H20Mass         = 0.001    //kg/cm3
	H20Visc         = 0.000091 //kg*(m/s)
	H20Kern         = 1.3      //Smoothing Kernel
	H20LiqDensity   = 0.001    //kg/cm^3
	SOS             = 1400.0   //m/s (maximal information transfer) 1480 m/s with sounds
	Particles       = 15       //15,625 Particles -- 1.9MB Positional Data Ram
	DefaultTimeStep = 0.1      //Evolution at Small Interval
	EOSGamma        = 1.8      //Equation of State Exponent Feature
	PCISamples      = 20
	SPHSamples      = 10
)

func TestSPDSampler(t *testing.T) {
	var mfp = MassFluidParticle{H20Mass, H20Visc, H20Kern, H20Kern, PSync, SOS, H20Mass, EOSGamma}
	var boxfluid = BoxFluidSystem{V.Vec32{0, 0, 0}, 2.0, 2.0, 2.0, 5, 5, 5} //Box System Description
	var sphfluid = SPHFluid{}                                               //Main Fluid Component
	sphfluid.Initialize(&boxfluid, &mfp, V.Vec32{0, 0, 0})
	//Print All Indexes and Check that all Indexes are included
	//Every 5
	for i := 0; i < 5*5*5; i++ {
		fmt.Printf("%d: [%d]\n ", i, sphfluid.Sampler.IdxPos[i])
	}
	fmt.Println()
	fmt.Printf("Test Passed")
}
