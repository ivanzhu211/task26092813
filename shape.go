package rigid2d

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

var (
	ErrTooFewVertices  = errors.New("rigid2d: polygon needs at least 3 vertices")
	ErrTooManyVertices = errors.New("rigid2d: polygon allows at most 8 vertices")
	ErrDuplicateVertex = errors.New("rigid2d: polygon has duplicate vertices")
	ErrCollinear       = errors.New("rigid2d: polygon has three collinear consecutive vertices")
	ErrConcave         = errors.New("rigid2d: polygon is concave")
	ErrZeroArea        = errors.New("rigid2d: polygon has zero area")
	ErrInvalidRadius   = errors.New("rigid2d: circle radius must be > 0")
)

const (
	minPolygonVertices = 3
	maxPolygonVertices = 8
	vertexEpsilon      = 1e-9
)

// Shape 形状定义。多边形顶点为局部坐标（任意原点），
// 引擎内部会平移到以面积质心为原点。
type Shape struct {
	Type     ShapeType
	Radius   float64 // 圆：半径
	Vertices []Vec2  // 多边形：局部顶点（以质心为原点）
}

// NewCircle 创建圆形。
func NewCircle(radius float64) (Shape, error) {
	if !(radius > 0) || math.IsNaN(radius) || math.IsInf(radius, 0) {
		return Shape{}, ErrInvalidRadius
	}
	return Shape{Type: ShapeCircle, Radius: radius}, nil
}

// NewPolygon 创建凸多边形。顶点可顺/逆时针给出。
// 返回的形状顶点已平移到以面积质心为原点，并统一为逆时针序。
func NewPolygon(vertices []Vec2) (Shape, error) {
	if len(vertices) < minPolygonVertices {
		return Shape{}, ErrTooFewVertices
	}
	if len(vertices) > maxPolygonVertices {
		return Shape{}, ErrTooManyVertices
	}
	vs := make([]Vec2, len(vertices))
	copy(vs, vertices)
	for _, v := range vs {
		if math.IsNaN(v.X) || math.IsNaN(v.Y) || math.IsInf(v.X, 0) || math.IsInf(v.Y, 0) {
			return Shape{}, ErrZeroArea
		}
	}
	// 重复顶点
	for i := 0; i < len(vs); i++ {
		for j := i + 1; j < len(vs); j++ {
			if Len(Sub(vs[i], vs[j])) < vertexEpsilon {
				return Shape{}, ErrDuplicateVertex
			}
		}
	}
	// 面积（有向），同时检查共线
	area := signedArea(vs)
	if math.Abs(area) < vertexEpsilon {
		return Shape{}, ErrZeroArea
	}
	if area < 0 { // 统一为逆时针
		for i, j := 0, len(vs)-1; i < j; i, j = i+1, j-1 {
			vs[i], vs[j] = vs[j], vs[i]
		}
		area = -area
	}
	// 凸性与共线检查（逆时针下所有叉积必须严格为正）
	n := len(vs)
	for i := 0; i < n; i++ {
		a := vs[i]
		b := vs[(i+1)%n]
		c := vs[(i+2)%n]
		cross := Cross(Sub(b, a), Sub(c, b))
		if math.Abs(cross) < vertexEpsilon {
			return Shape{}, ErrCollinear
		}
		if cross < 0 {
			return Shape{}, ErrConcave
		}
	}
	// 面积质心
	centroid := polygonCentroid(vs)
	for i := range vs {
		vs[i] = Sub(vs[i], centroid)
	}
	return Shape{Type: ShapePolygon, Vertices: vs}, nil
}

func signedArea(vs []Vec2) float64 {
	var a float64
	n := len(vs)
	for i := 0; i < n; i++ {
		a += Cross(vs[i], vs[(i+1)%n])
	}
	return a * 0.5
}

func polygonCentroid(vs []Vec2) Vec2 {
	var cx, cy, a float64
	n := len(vs)
	for i := 0; i < n; i++ {
		p := vs[i]
		q := vs[(i+1)%n]
		w := Cross(p, q)
		a += w
		cx += (p.X + q.X) * w
		cy += (p.Y + q.Y) * w
	}
	a *= 0.5
	if a == 0 {
		return Vec2{}
	}
	f := 1 / (6 * a)
	return Vec2{cx * f, cy * f}
}

// MassData 均匀密度下的质量数据。
type MassData struct {
	Mass    float64
	Inertia float64 // 绕质心
}

// ComputeMass 按均匀密度计算质量与绕质心转动惯量。
func (s Shape) ComputeMass(density float64) MassData {
	switch s.Type {
	case ShapeCircle:
		m := density * math.Pi * s.Radius * s.Radius
		return MassData{Mass: m, Inertia: 0.5 * m * s.Radius * s.Radius}
	default:
		// 多边形：顶点以质心为原点、逆时针
		var area, inertia float64
		vs := s.Vertices
		n := len(vs)
		const inv3 = 1.0 / 3.0
		for i := 0; i < n; i++ {
			p := vs[i]
			q := vs[(i+1)%n]
			w := math.Abs(Cross(p, q))
			area += 0.5 * w
			intx2 := p.X*p.X + q.X*p.X + q.X*q.X
			inty2 := p.Y*p.Y + q.Y*p.Y + q.Y*q.Y
			inertia += (0.25 * inv3 * w) * (intx2 + inty2)
		}
		m := density * area
		return MassData{Mass: m, Inertia: density * inertia}
	}
}
