package rigidbody

import "errors"

// BodyType 刚体类型。
type BodyType int

const (
	Static BodyType = iota
	Dynamic
)

// 刚体参数校验错误。
var (
	ErrInvalidMass        = errors.New("rigidbody: dynamic body mass must be in [0.1, 10000]")
	ErrInvalidRestitution = errors.New("rigidbody: restitution must be in [0, 1]")
	ErrInvalidFriction    = errors.New("rigidbody: friction must be in [0, 1]")
	ErrTooManyBodies      = errors.New("rigidbody: world supports at most 500 bodies")
	ErrUnknownBody        = errors.New("rigidbody: unknown body id")
)

const (
	minMass = 0.1
	maxMass = 10000.0
)

// BodyParams 创建刚体的参数。
type BodyParams struct {
	Shape       Shape
	Type        BodyType
	Position    Vec2    // 质心初始位置（世界坐标）
	Angle       float64 // 初始转角（弧度）
	Velocity    Vec2    // 初始线速度
	AngularVel  float64 // 初始角速度
	Mass        float64 // 动态刚体质量，静态忽略
	Restitution float64 // 弹性系数 [0,1]
	Friction    float64 // 摩擦系数 [0,1]
}

// body 是引擎内部的刚体状态。
type body struct {
	id          int
	shape       Shape // 已规范化：质心为原点、逆时针
	typ         BodyType
	pos         Vec2
	angle       float64
	vel         Vec2
	angularVel  float64
	mass        float64
	invMass     float64
	inertia     float64
	invInertia  float64
	restitution float64
	friction    float64
}

// worldVertices 返回世界坐标下的顶点（圆返回空）。
func (b *body) worldVertices() []Vec2 {
	if b.shape.Type == ShapeCircle {
		return nil
	}
	out := make([]Vec2, len(b.shape.Vertices))
	for i, v := range b.shape.Vertices {
		out[i] = v.Rotate(b.angle).Add(b.pos)
	}
	return out
}

// worldPoint 把局部坐标点变换到世界坐标。
func (b *body) worldPoint(p Vec2) Vec2 {
	return p.Rotate(b.angle).Add(b.pos)
}
