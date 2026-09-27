package rigid2d

import (
	"errors"
	"math"
	"sort"
)

// 世界与步进限制。
const (
	MaxBodies       = 500
	DefaultDt       = 1.0 / 60.0
	MaxSolverIters  = 10
	MaxStepsPerCall = 120
)

var (
	ErrTooManyBodies = errors.New("rigid2d: world already has 500 bodies")
	ErrInvalidSteps  = errors.New("rigid2d: steps must be in [1, 120]")
	ErrBodyNotFound  = errors.New("rigid2d: body id not found")
)

// DefaultGravity 默认重力。
var DefaultGravity = Vec2{0, -9.81}

// World 物理世界，全部状态在内存中。
type World struct {
	Gravity  Vec2
	Dt       float64
	bodies   []*Body // 按 id 升序
	nextID   int
	arbiters map[arbiterKey]*arbiter
	// 最近一步的接触（按 (idA,idB) 排序）
	lastContacts []Contact
	// 每步缓存：世界坐标顶点与 AABB（复用底层数组，避免分配）
	cache      []bodyCache
	vertsBuf   []Vec2
	sortedKeys []arbiterKey
}

type arbiterKey struct{ a, b int }

// bodyCache 单个刚体的每步缓存。
type bodyCache struct {
	verts   []Vec2 // 多边形世界坐标顶点；圆为 nil
	aabbMin Vec2
	aabbMax Vec2
}

// NewWorld 创建世界，重力为默认值的副本。
func NewWorld() *World {
	return &World{
		Gravity:  DefaultGravity,
		Dt:       DefaultDt,
		arbiters: make(map[arbiterKey]*arbiter),
	}
}

// AddBody 校验并创建刚体，返回分配的 id。
// 任何参数不合法时返回错误，世界保持不变。
func (w *World) AddBody(def BodyDef) (int, error) {
	if len(w.bodies) >= MaxBodies {
		return 0, ErrTooManyBodies
	}
	if err := validateBodyDef(&def); err != nil {
		return 0, err
	}
	id := w.nextID
	b := newBody(id, &def)
	w.bodies = append(w.bodies, b)
	w.nextID++
	return id, nil
}

// BodyState 刚体状态快照。
type BodyState struct {
	ID       int
	Position Vec2
	Angle    float64
	LinVel   Vec2
	AngVel   float64
}

// GetBodyState 读回刚体状态。
func (w *World) GetBodyState(id int) (BodyState, error) {
	b := w.bodyByID(id)
	if b == nil {
		return BodyState{}, ErrBodyNotFound
	}
	return BodyState{
		ID:       b.ID,
		Position: b.Position,
		Angle:    b.Angle,
		LinVel:   b.LinVel,
		AngVel:   b.AngVel,
	}, nil
}

// BodyCount 当前刚体数。
func (w *World) BodyCount() int { return len(w.bodies) }

// States 返回全部刚体状态（按 id 升序）。
func (w *World) States() []BodyState {
	out := make([]BodyState, len(w.bodies))
	for i, b := range w.bodies {
		out[i] = BodyState{
			ID:       b.ID,
			Position: b.Position,
			Angle:    b.Angle,
			LinVel:   b.LinVel,
			AngVel:   b.AngVel,
		}
	}
	return out
}

// Contact 单个接触的只读快照。
type Contact struct {
	BodyA       int // 较小 id
	BodyB       int
	Normal      Vec2    // 从 BodyA 指向 BodyB 的单位法线
	Penetration float64 // 穿透深度，贴合为 0
	Points      []Vec2  // 1 或 2 个世界坐标接触点
}

// Contacts 返回最近一步用到的全部接触。
func (w *World) Contacts() []Contact {
	out := make([]Contact, len(w.lastContacts))
	copy(out, w.lastContacts)
	return out
}

func (w *World) bodyByID(id int) *Body {
	// bodies 按 id 升序且 id 连续递增
	if id >= 0 && id < len(w.bodies) && w.bodies[id].ID == id {
		return w.bodies[id]
	}
	i := sort.Search(len(w.bodies), func(i int) bool { return w.bodies[i].ID >= id })
	if i < len(w.bodies) && w.bodies[i].ID == id {
		return w.bodies[i]
	}
	return nil
}

// Step 推进 n 步（1..120），否则拒绝且一步都不推进。
func (w *World) Step(n int) error {
	if n < 1 || n > MaxStepsPerCall {
		return ErrInvalidSteps
	}
	for i := 0; i < n; i++ {
		w.stepOnce()
	}
	return nil
}

func (w *World) stepOnce() {
	dt := w.Dt
	invDt := 1 / dt

	// 0. 重建每步缓存（顶点/AABB）
	w.rebuildCache()

	// 1. 积分外力
	for _, b := range w.bodies {
		if b.Type != Dynamic {
			continue
		}
		b.LinVel = Add(b.LinVel, Mul(Add(w.Gravity, Mul(b.Force, b.InvMass)), dt))
		b.AngVel += dt * b.InvInertia * b.Torque
	}

	// 2. 碰撞检测，构建/更新 arbiter
	w.broadPhase()

	// 3. 预求解（含热启动）
	states := make([]*arbiterState, 0, len(w.arbiters))
	for _, key := range w.sortedKeys {
		arb := w.arbiters[key]
		states = append(states, arb.preStep(invDt))
	}

	// 4. 迭代求解
	for iter := 0; iter < MaxSolverIters; iter++ {
		for _, st := range states {
			st.applyImpulse()
		}
	}

	// 5. 积分位置
	for _, b := range w.bodies {
		if b.Type != Dynamic {
			continue
		}
		b.Position = Add(b.Position, Mul(b.LinVel, dt))
		b.Angle += dt * b.AngVel
		b.Force = Vec2{}
		b.Torque = 0
	}

	// 6. 记录本步接触
	w.recordContacts()
}

// rebuildCache 重新计算所有刚体的世界顶点与 AABB，复用底层数组。
func (w *World) rebuildCache() {
	const margin = 0.05
	n := len(w.bodies)
	if cap(w.cache) < n {
		w.cache = make([]bodyCache, n)
	}
	w.cache = w.cache[:n]
	total := 0
	for _, b := range w.bodies {
		if b.Shape.Type == ShapePolygon {
			total += len(b.Shape.Vertices)
		}
	}
	if cap(w.vertsBuf) < total {
		w.vertsBuf = make([]Vec2, total)
	}
	w.vertsBuf = w.vertsBuf[:total]
	off := 0
	for i, b := range w.bodies {
		c := &w.cache[i]
		if b.Shape.Type == ShapePolygon {
			nv := len(b.Shape.Vertices)
			c.verts = w.vertsBuf[off : off+nv : off+nv]
			off += nv
			r := NewRot(b.Angle)
			for j, v := range b.Shape.Vertices {
				c.verts[j] = Add(b.Position, r.MulV(v))
			}
			min := Vec2{math.MaxFloat64, math.MaxFloat64}
			max := Vec2{-math.MaxFloat64, -math.MaxFloat64}
			for _, v := range c.verts {
				min.X = math.Min(min.X, v.X)
				min.Y = math.Min(min.Y, v.Y)
				max.X = math.Max(max.X, v.X)
				max.Y = math.Max(max.Y, v.Y)
			}
			c.aabbMin = Vec2{min.X - margin, min.Y - margin}
			c.aabbMax = Vec2{max.X + margin, max.Y + margin}
		} else {
			c.verts = nil
			r := b.Shape.Radius + margin
			c.aabbMin = Vec2{b.Position.X - r, b.Position.Y - r}
			c.aabbMax = Vec2{b.Position.X + r, b.Position.Y + r}
		}
	}
}

func aabbOverlap(a, b *bodyCache) bool {
	return a.aabbMin.X <= b.aabbMax.X && a.aabbMax.X >= b.aabbMin.X &&
		a.aabbMin.Y <= b.aabbMax.Y && a.aabbMax.Y >= b.aabbMin.Y
}

func (w *World) broadPhase() {
	// 复用 map：先标记旧键失效，再逐对检测
	for key := range w.arbiters {
		w.arbiters[key].alive = false
	}
	for i := 0; i < len(w.bodies); i++ {
		for j := i + 1; j < len(w.bodies); j++ {
			a, b := w.bodies[i], w.bodies[j]
			if a.Type == Static && b.Type == Static {
				continue
			}
			if !aabbOverlap(&w.cache[i], &w.cache[j]) {
				continue
			}
			key := arbiterKey{a.ID, b.ID}
			cs := collide(a, b, w.cache[i].verts, w.cache[j].verts)
			if len(cs) == 0 {
				continue
			}
			arb, ok := w.arbiters[key]
			if !ok {
				arb = &arbiter{
					a: a, b: b,
					friction:    math.Sqrt(a.Friction * b.Friction),
					restitution: math.Max(a.Restitution, b.Restitution),
				}
				w.arbiters[key] = arb
			}
			arb.alive = true
			arb.update(cs)
		}
	}
	// 移除不再接触的 arbiter
	for key := range w.arbiters {
		if !w.arbiters[key].alive {
			delete(w.arbiters, key)
		}
	}
	// 重建排序键缓存
	w.sortedKeys = w.sortedKeys[:0]
	for k := range w.arbiters {
		w.sortedKeys = append(w.sortedKeys, k)
	}
	sort.Slice(w.sortedKeys, func(i, j int) bool {
		if w.sortedKeys[i].a != w.sortedKeys[j].a {
			return w.sortedKeys[i].a < w.sortedKeys[j].a
		}
		return w.sortedKeys[i].b < w.sortedKeys[j].b
	})
}

func (w *World) recordContacts() {
	w.lastContacts = w.lastContacts[:0]
	for _, key := range w.sortedKeys {
		arb := w.arbiters[key]
		c := Contact{
			BodyA:  key.a,
			BodyB:  key.b,
			Normal: arb.contacts[0].normal,
		}
		maxPen := 0.0
		for _, ct := range arb.contacts {
			pen := -ct.separation
			if pen > maxPen {
				maxPen = pen
			}
			c.Points = append(c.Points, ct.position)
		}
		c.Penetration = maxPen
		w.lastContacts = append(w.lastContacts, c)
	}
}
