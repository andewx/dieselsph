package fluid

import V "diesel.com/diesel/vector"

//SPH Sampler Interface Derives Particle Subset of N Samples from a Vec3 Spatial Position
type SPHSampler interface {

	//Number of samples to take + particle slice
	SPHSampler(samples int, particles []V.Vec32) *SPHSampler //Initiates Sampler  (samples is number samples)
	GetSamples(v V.Vec32) []int                              //Gets samples by index value slice
	Run(status chan int)                                     //Runs any internal thread
}
