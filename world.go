package rigidbody

import (
	"errors"
	"math"
)

// 步进参数错误。
var ErrInvalidStepCount = errors.New("rigidbody: step count must be in [1, 120]")

const (
	// MaxBodies 每个世界最多刚体数。
	MaxBodies = 500
	// MaxStepsPerCall 单次步进调用最多推进的步数。
	MaxStepsPerCall = 120
	// FixedDT 固定步长（秒）。
	FixedDT = 1.0 / 60.0

	solverIterations = 10
	// restitutionThreshold 法向接近速度低于该值（m/s）时不反弹。
	restitutionThreshold = 1.0
)

// BodyState 刚体的对外可读状态。
type BodyState struct {
	ID         int
	Position   Vec2
	Angle      float64
	Velocity   Vec2
	AngularVel float64
}

// Contact 最近一步求解用到的接触。
type Contact struct {
	BodyA  int     // 较小的刚体 id
	BodyB  int     // 较大的刚体 id
	Normal Vec2    // 从 A 指向 B 的单位法线
	Depth  float64 // 穿透深度，刚好贴合为 0
	Points []Vec2  // 1 或 2 个世界坐标接触点
}

// World 是物理世界，状态全部在内存中。
type World struct {
	gravity  Vec2
	bodies   []*body
	nextID   int
	contacts []Contact                 // 最近一步的接触
	cache    map[pairKey][]cachedPoint // 热启动缓存
}

// NewWorld 创建世界。gravity 为零值 Vec2 时使用默认 (0, -9.81)。
func NewWorld(gravity Vec2) *World {
	if gravity == (Vec2{}) {
		gravity = Vec2{0, -9.81}
	}
	return &World{gravity: gravity}
}

// Gravity 返回世界重力向量。
func (w *World) Gravity() Vec2 { return w.gravity }

// AddBody 创建刚体，返回引擎分配的 id（从 0 开始递增）。
// 任一参数非法时返回错误，世界保持不变。
func (w *World) AddBody(p BodyParams) (int, error) {
	if len(w.bodies) >= MaxBodies {
		return 0, ErrTooManyBodies
	}
	if p.Restitution < 0 || p.Restitution > 1 || math.IsNaN(p.Restitution) {
		return 0, ErrInvalidRestitution
	}
	if p.Friction < 0 || p.Friction > 1 || math.IsNaN(p.Friction) {
		return 0, ErrInvalidFriction
	}
	shape, err := p.Shape.validate()
	if err != nil {
		return 0, err
	}
	b := &body{
		id:          w.nextID,
		shape:       shape,
		typ:         p.Type,
		pos:         p.Position,
		angle:       p.Angle,
		vel:         p.Velocity,
		angularVel:  p.AngularVel,
		restitution: p.Restitution,
		friction:    p.Friction,
	}
	if p.Type == Dynamic {
		if p.Mass < minMass || p.Mass > maxMass || math.IsNaN(p.Mass) {
			return 0, ErrInvalidMass
		}
		area, unitInertia := shape.massData()
		density := p.Mass / area
		b.mass = p.Mass
		b.invMass = 1 / p.Mass
		b.inertia = unitInertia * density
		b.invInertia = 1 / b.inertia
	}
	w.bodies = append(w.bodies, b)
	w.nextID++
	return b.id, nil
}

// Step 推进 n 步（1~120），超出范围返回错误且一步都不推进。
func (w *World) Step(n int) error {
	if n < 1 || n > MaxStepsPerCall {
		return ErrInvalidStepCount
	}
	for i := 0; i < n; i++ {
		w.stepOnce()
	}
	return nil
}

// BodyState 读取刚体状态。
func (w *World) BodyState(id int) (BodyState, error) {
	if id < 0 || id >= len(w.bodies) {
		return BodyState{}, ErrUnknownBody
	}
	b := w.bodies[id]
	return BodyState{
		ID:         b.id,
		Position:   b.pos,
		Angle:      b.angle,
		Velocity:   b.vel,
		AngularVel: b.angularVel,
	}, nil
}

// Bodies 返回全部刚体 id（按创建顺序）。
func (w *World) Bodies() []int {
	ids := make([]int, len(w.bodies))
	for i, b := range w.bodies {
		ids[i] = b.id
	}
	return ids
}

// Contacts 返回最近一步求解用到的全部接触。
func (w *World) Contacts() []Contact {
	out := make([]Contact, len(w.contacts))
	copy(out, w.contacts)
	return out
}
