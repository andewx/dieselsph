# dieselsph
Diesel Smoothed Particle Hydrodynamics Library. Using Go Language, Go-GL / GLFW frameworks. 

# README

This version of Diesel SPH is still in development and not yet considered releasable for application development. However you may clone and work on your own repository.

# OVERVIEW

Diesel SPH develops a langrangian fluid simulation environment with the GO Language environment. Go language was chosen for the ability to quickly build and test frameworks and applications, it's superior runtime performance, and easy multi-threading support. Note that all context calls to the GLFW/GL frameworks requires a single locked thread once the windowed application starts.

# DEPENDENCIES:

Uses <b>Go 1.8+</b>

Currently this version uses <b>GL V4.1 / and GLFW wV3.2.</b> In the future we may look at build contraints in Go Lang in order to allow different versions of go-gl to be invoked. 

# For OpenGL 4.1
$ go get -u github.com/go-gl/gl/v4.1-core/gl

# For GLFW 3.2
$ go get github.com/go-gl/glfw/v3.2/glfw


# TODO

1. Packages fluid and geometry requires further testing and validation
2. Create new package for high level fluid parameterization and scene building (composer)
3. OpenGL Tasks (Translate Simulation into OpenGL Environment) - Consider Using G3N Framework and Translate Fluid into GL Geometry
4. Export Scene for Lux Render / PBRT / Cycles Ray Tracing


