package rigidbody

import "math"

// Vec2 是二维向量，单位米（或米/秒等，视上下文）。
type Vec2 struct {
	X, Y float64
}

func (a Vec2) Add(b Vec2) Vec2    { return Vec2{a.X + b.X, a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2    { return Vec2{a.X - b.X, a.Y - b.Y} }
func (a Vec2) Mul(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }

// Cross 返回 a×b 的 z 分量（标量）。
func (a Vec2) Cross(b Vec2) float64 { return a.X*b.Y - a.Y*b.X }

// CrossSV 返回标量 s 与向量 v 的叉积：(-s*v.Y, s*v.X)。
func CrossSV(s float64, v Vec2) Vec2 { return Vec2{-s * v.Y, s * v.X} }

func (a Vec2) Len() float64 { return math.Hypot(a.X, a.Y) }

// Norm 返回单位向量；零向量原样返回。
func (a Vec2) Norm() Vec2 {
	l := a.Len()
	if l == 0 {
		return a
	}
	return Vec2{a.X / l, a.Y / l}
}

// Perp 返回逆时针旋转 90° 的向量。
func (a Vec2) Perp() Vec2 { return Vec2{-a.Y, a.X} }

// RPerp 返回顺时针旋转 90° 的向量。
// 对逆时针多边形的有向边，RPerp 即外法线方向。
func (a Vec2) RPerp() Vec2 { return Vec2{a.Y, -a.X} }

// Rotate 将向量旋转 angle 弧度（逆时针为正）。
func (a Vec2) Rotate(angle float64) Vec2 {
	c, s := math.Cos(angle), math.Sin(angle)
	return Vec2{a.X*c - a.Y*s, a.X*s + a.Y*c}
}
