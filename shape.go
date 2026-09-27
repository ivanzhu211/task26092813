package rigidbody

import (
	"errors"
	"math"
)

// ShapeType 形状类型。
type ShapeType int

const (
	ShapeCircle ShapeType = iota
	ShapePolygon
)

// 形状与参数校验错误。
var (
	ErrTooFewVertices    = errors.New("rigidbody: polygon needs at least 3 vertices")
	ErrTooManyVertices   = errors.New("rigidbody: polygon supports at most 8 vertices")
	ErrCollinear         = errors.New("rigidbody: polygon has three collinear consecutive vertices")
	ErrDuplicateVertex   = errors.New("rigidbody: polygon has duplicate vertices")
	ErrZeroArea          = errors.New("rigidbody: polygon has zero area")
	ErrConcave           = errors.New("rigidbody: polygon must be convex")
	ErrNonPositiveRadius = errors.New("rigidbody: circle radius must be positive")
)

// Shape 描述刚体形状。多边形顶点为局部坐标，
// 引擎内部会平移到以面积质心为原点。
type Shape struct {
	Type     ShapeType
	Radius   float64 // 圆半径（Type == ShapeCircle）
	Vertices []Vec2  // 多边形顶点（Type == ShapePolygon），局部坐标
}

// Circle 构造圆形。
func Circle(radius float64) Shape {
	return Shape{Type: ShapeCircle, Radius: radius}
}

// Polygon 构造凸多边形（3~8 顶点，顺/逆时针均可）。
func Polygon(vertices []Vec2) Shape {
	return Shape{Type: ShapePolygon, Vertices: vertices}
}

// Box 构造以原点为中心、宽 w 高 h 的矩形。
func Box(w, h float64) Shape {
	hw, hh := w/2, h/2
	return Shape{Type: ShapePolygon, Vertices: []Vec2{
		{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh},
	}}
}

// validate 校验形状，返回规范化后的形状：
// 多边形顶点平移到以面积质心为原点，并统一为逆时针序。
func (s Shape) validate() (Shape, error) {
	if s.Type == ShapeCircle {
		if !(s.Radius > 0) || math.IsNaN(s.Radius) || math.IsInf(s.Radius, 0) {
			return s, ErrNonPositiveRadius
		}
		return s, nil
	}
	verts := s.Vertices
	if len(verts) < 3 {
		return s, ErrTooFewVertices
	}
	if len(verts) > 8 {
		return s, ErrTooManyVertices
	}
	for _, v := range verts {
		if math.IsNaN(v.X) || math.IsNaN(v.Y) || math.IsInf(v.X, 0) || math.IsInf(v.Y, 0) {
			return s, ErrZeroArea
		}
	}
	// 重复顶点（任意两点完全相同）。
	for i := 0; i < len(verts); i++ {
		for j := i + 1; j < len(verts); j++ {
			if verts[i] == verts[j] {
				return s, ErrDuplicateVertex
			}
		}
	}
	// 鞋带公式面积（带符号）。
	signedArea := 0.0
	n := len(verts)
	for i := 0; i < n; i++ {
		signedArea += verts[i].Cross(verts[(i+1)%n])
	}
	signedArea /= 2
	if signedArea == 0 {
		return s, ErrZeroArea
	}
	// 凸性与三点共线：所有连续边叉积必须与面积同号且非零。
	for i := 0; i < n; i++ {
		a := verts[i]
		b := verts[(i+1)%n]
		c := verts[(i+2)%n]
		cross := b.Sub(a).Cross(c.Sub(b))
		if cross == 0 {
			return s, ErrCollinear
		}
		if cross*signedArea < 0 {
			return s, ErrConcave
		}
	}
	// 统一为逆时针。
	out := make([]Vec2, n)
	if signedArea > 0 {
		copy(out, verts)
	} else {
		for i, v := range verts {
			out[n-1-i] = v
		}
	}
	// 面积质心。
	cx, cy := 0.0, 0.0
	for i := 0; i < n; i++ {
		c := out[i].Cross(out[(i+1)%n])
		cx += (out[i].X + out[(i+1)%n].X) * c
		cy += (out[i].Y + out[(i+1)%n].Y) * c
	}
	area := math.Abs(signedArea)
	centroid := Vec2{cx / (6 * area), cy / (6 * area)}
	for i := range out {
		out[i] = out[i].Sub(centroid)
	}
	return Shape{Type: ShapePolygon, Vertices: out}, nil
}

// massData 按均匀密度计算单位密度下的质量（面积）与绕质心转动惯量。
// 多边形要求已规范化为质心原点。
func (s Shape) massData() (mass float64, inertia float64) {
	if s.Type == ShapeCircle {
		m := math.Pi * s.Radius * s.Radius
		return m, 0.5 * m * s.Radius * s.Radius
	}
	verts := s.Vertices
	n := len(verts)
	area := 0.0
	inertia = 0.0
	for i := 0; i < n; i++ {
		p1 := verts[i]
		p2 := verts[(i+1)%n]
		cross := math.Abs(p1.Cross(p2))
		area += cross
		inertia += cross * (p1.Dot(p1) + p1.Dot(p2) + p2.Dot(p2))
	}
	area /= 2
	return area, inertia / 12
}
