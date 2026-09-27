package rigid2d

import (
	"errors"
	"math"
)

// BodyType 刚体类型。
type BodyType int

const (
	Static BodyType = iota
	Dynamic
)

var (
	ErrInvalidMass        = errors.New("rigid2d: dynamic body mass must be in [0.1, 10000]")
	ErrInvalidFriction    = errors.New("rigid2d: friction must be in [0, 1]")
	ErrInvalidRestitution = errors.New("rigid2d: restitution must be in [0, 1]")
	ErrInvalidTransform   = errors.New("rigid2d: position/angle must be finite")
)

const (
	minMass = 0.1
	maxMass = 10000.0
)

// BodyDef 创建刚体的参数。
type BodyDef struct {
	Type        BodyType
	Shape       Shape
	Position    Vec2 // 质心位置（世界坐标）
	Angle       float64
	Mass        float64 // 仅动态刚体使用
	Friction    float64
	Restitution float64
}

// Body 刚体。字段全部可序列化。
type Body struct {
	ID          int
	Type        BodyType
	Shape       Shape
	Position    Vec2
	Angle       float64
	LinVel      Vec2
	AngVel      float64
	Force       Vec2
	Torque      float64
	Mass        float64
	InvMass     float64
	Inertia     float64
	InvInertia  float64
	Friction    float64
	Restitution float64
}

func validateBodyDef(def *BodyDef) error {
	if math.IsNaN(def.Position.X) || math.IsNaN(def.Position.Y) ||
		math.IsInf(def.Position.X, 0) || math.IsInf(def.Position.Y, 0) ||
		math.IsNaN(def.Angle) || math.IsInf(def.Angle, 0) {
		return ErrInvalidTransform
	}
	if math.IsNaN(def.Friction) || def.Friction < 0 || def.Friction > 1 {
		return ErrInvalidFriction
	}
	if math.IsNaN(def.Restitution) || def.Restitution < 0 || def.Restitution > 1 {
		return ErrInvalidRestitution
	}
	if def.Type == Dynamic {
		if math.IsNaN(def.Mass) || math.IsInf(def.Mass, 0) ||
			def.Mass < minMass || def.Mass > maxMass {
			return ErrInvalidMass
		}
	}
	return nil
}

func newBody(id int, def *BodyDef) *Body {
	b := &Body{
		ID:          id,
		Type:        def.Type,
		Shape:       def.Shape,
		Position:    def.Position,
		Angle:       def.Angle,
		Friction:    def.Friction,
		Restitution: def.Restitution,
	}
	if def.Type == Dynamic {
		// 由形状面积推密度，使总质量等于指定质量
		unit := def.Shape.ComputeMass(1.0)
		density := def.Mass / unit.Mass
		md := def.Shape.ComputeMass(density)
		b.Mass = md.Mass
		b.InvMass = 1 / md.Mass
		b.Inertia = md.Inertia
		b.InvInertia = 1 / md.Inertia
	}
	return b
}
