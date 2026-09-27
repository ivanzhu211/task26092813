package rigid2d

import "math"

// Vec2 二维向量，单位米（或米/秒等，视上下文）。
type Vec2 struct {
	X, Y float64
}

func Add(a, b Vec2) Vec2             { return Vec2{a.X + b.X, a.Y + b.Y} }
func Sub(a, b Vec2) Vec2             { return Vec2{a.X - b.X, a.Y - b.Y} }
func Mul(a Vec2, s float64) Vec2     { return Vec2{a.X * s, a.Y * s} }
func Neg(a Vec2) Vec2                { return Vec2{-a.X, -a.Y} }
func Dot(a, b Vec2) float64          { return a.X*b.X + a.Y*b.Y }
func Cross(a, b Vec2) float64        { return a.X*b.Y - a.Y*b.X }
func CrossSV(s float64, v Vec2) Vec2 { return Vec2{-s * v.Y, s * v.X} }
func CrossVS(v Vec2, s float64) Vec2 { return Vec2{s * v.Y, -s * v.X} }
func Len(a Vec2) float64             { return math.Hypot(a.X, a.Y) }

func Normalize(a Vec2) Vec2 {
	l := Len(a)
	if l == 0 {
		return Vec2{0, 0}
	}
	return Vec2{a.X / l, a.Y / l}
}

// Mat22 2x2 矩阵，用于转动惯量变换。
type Mat22 struct {
	Ex, Ey Vec2 // 列向量
}

func (m Mat22) MulV(v Vec2) Vec2 {
	return Vec2{m.Ex.X*v.X + m.Ey.X*v.Y, m.Ex.Y*v.X + m.Ey.Y*v.Y}
}

func (m Mat22) Add(o Mat22) Mat22 {
	return Mat22{Add(m.Ex, o.Ex), Add(m.Ey, o.Ey)}
}

func (m Mat22) Invert() Mat22 {
	a, b := m.Ex.X, m.Ey.X
	c, d := m.Ex.Y, m.Ey.Y
	det := a*d - b*c
	if det == 0 {
		return Mat22{}
	}
	inv := 1 / det
	return Mat22{
		Ex: Vec2{inv * d, -inv * c},
		Ey: Vec2{-inv * b, inv * a},
	}
}

// Rot 旋转矩阵（用 cos/sin 表示）。
type Rot struct {
	S, C float64
}

func NewRot(angle float64) Rot {
	return Rot{S: math.Sin(angle), C: math.Cos(angle)}
}

func (r Rot) MulV(v Vec2) Vec2 {
	return Vec2{r.C*v.X - r.S*v.Y, r.S*v.X + r.C*v.Y}
}

func (r Rot) MulTV(v Vec2) Vec2 {
	return Vec2{r.C*v.X + r.S*v.Y, -r.S*v.X + r.C*v.Y}
}
