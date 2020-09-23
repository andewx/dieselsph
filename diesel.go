package diesel

//Declarative Interfaces
type App interface {
	pre() error
	run() error
	pos() error
}
