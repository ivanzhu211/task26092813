package rigidbody

import "math"

// featureID 标识接触点的特征（用于跨步匹配，做热启动）。
type featureID struct {
	refEdge int // 参考边索引
	incEdge int // 入射边索引
	flip    int // 参考面属于 b 时为 1
}

// contactPoint 是内部接触点。
type contactPoint struct {
	pos      Vec2 // 世界坐标
	depth    float64
	feature  featureID
	pn       float64 // 累计法向冲量
	pt       float64 // 累计切向冲量
	massN    float64 // 法向有效质量
	massT    float64 // 切向有效质量
	bias     float64 // 位置误差修正速度
	restBias float64 // 弹性偏置速度
	rA, rB   Vec2    // 接触点相对两质心的矢径
}

// manifold 是两个刚体间的碰撞流形（内部表示）。
type manifold struct {
	a, b    *body // 保证 a.id < b.id
	normal  Vec2  // 从 a 指向 b 的单位法线
	points  []contactPoint
	mu      float64 // 混合摩擦系数
	tangent Vec2    // 切向
}

// collide 检测两个刚体是否接触。静态-静态由调用方过滤。
func collide(a, b *body) (manifold, bool) {
	ca, cb := a.shape.Type, b.shape.Type
	var m manifold
	var ok bool
	switch {
	case ca == ShapeCircle && cb == ShapeCircle:
		m, ok = collideCircles(a, b)
	case ca == ShapePolygon && cb == ShapePolygon:
		m, ok = collidePolygons(a, b)
	case ca == ShapeCircle && cb == ShapePolygon:
		m, ok = collideCirclePolygon(a, b)
	default: // 多边形 vs 圆：交换后翻转法线
		m, ok = collideCirclePolygon(b, a)
		if ok {
			m.normal = m.normal.Mul(-1)
			m.a, m.b = a, b
		}
	}
	return m, ok
}

func collideCircles(a, b *body) (manifold, bool) {
	d := b.pos.Sub(a.pos)
	dist := d.Len()
	r := a.shape.Radius + b.shape.Radius
	if dist >= r {
		return manifold{}, false
	}
	n := Vec2{1, 0}
	if dist > 0 {
		n = d.Mul(1 / dist)
	}
	depth := r - dist
	// 两表面点的中点。
	pa := a.pos.Add(n.Mul(a.shape.Radius))
	pb := b.pos.Sub(n.Mul(b.shape.Radius))
	p := pa.Add(pb).Mul(0.5)
	return manifold{
		a: a, b: b, normal: n,
		points: []contactPoint{{pos: p, depth: depth}},
	}, true
}

// collideCirclePolygon 检测圆 a 与多边形 b，法线从 a 指向 b。
func collideCirclePolygon(circ, poly *body) (manifold, bool) {
	// 圆心到多边形局部坐标。
	cLocal := circ.pos.Sub(poly.pos).Rotate(-poly.angle)
	verts := poly.shape.Vertices
	n := len(verts)

	// 最大分离面。
	maxSep := math.Inf(-1)
	faceIdx := 0
	for i := 0; i < n; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%n]
		edge := v2.Sub(v1)
		normal := edge.RPerp().Norm() // 逆时针多边形的外法线
		s := normal.Dot(cLocal.Sub(v1))
		if s > maxSep {
			maxSep = s
			faceIdx = i
		}
	}
	r := circ.shape.Radius
	if maxSep > r {
		return manifold{}, false
	}

	var normalLocal Vec2
	var depth float64
	var pointLocal Vec2
	if maxSep <= 0 {
		// 圆心在多边形内部：最近面即 maxSep 面。
		v1 := verts[faceIdx]
		v2 := verts[(faceIdx+1)%n]
		normalLocal = v2.Sub(v1).RPerp().Norm()
		depth = r - maxSep
		pointLocal = cLocal.Sub(normalLocal.Mul(maxSep))
	} else {
		// 圆心在外部：求边界上最近点。
		best := Vec2{}
		bestDist := math.Inf(1)
		for i := 0; i < n; i++ {
			v1 := verts[i]
			v2 := verts[(i+1)%n]
			q := closestOnSegment(cLocal, v1, v2)
			d := q.Sub(cLocal).Len()
			if d < bestDist {
				bestDist = d
				best = q
			}
		}
		if bestDist >= r {
			return manifold{}, false
		}
		normalLocal = best.Sub(cLocal).Norm()
		depth = r - bestDist
		pointLocal = best
	}
	normal := normalLocal.Rotate(poly.angle)
	return manifold{
		a: circ, b: poly, normal: normal,
		points: []contactPoint{{
			pos:     poly.worldPoint(pointLocal),
			depth:   depth,
			feature: featureID{refEdge: faceIdx},
		}},
	}, true
}

func closestOnSegment(p, a, b Vec2) Vec2 {
	ab := b.Sub(a)
	t := 0.0
	if d := ab.Dot(ab); d > 0 {
		t = p.Sub(a).Dot(ab) / d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return a.Add(ab.Mul(t))
}

// maxSeparation 返回 b 相对 a 各面的最大分离量及对应面索引。
// 分离量 > 0 表示存在分离轴。
func maxSeparation(vertsA, vertsB []Vec2) (float64, int) {
	best := math.Inf(-1)
	bestIdx := 0
	na := len(vertsA)
	for i := 0; i < na; i++ {
		v1 := vertsA[i]
		v2 := vertsA[(i+1)%na]
		normal := v2.Sub(v1).RPerp().Norm()
		minB := math.Inf(1)
		for _, vb := range vertsB {
			if s := normal.Dot(vb.Sub(v1)); s < minB {
				minB = s
			}
		}
		if minB > best {
			best = minB
			bestIdx = i
		}
	}
	return best, bestIdx
}

// clipSegment 用平面 (normal·p <= offset) 裁剪线段，返回保留的点。
func clipSegment(normal Vec2, offset float64, in [2]Vec2) []Vec2 {
	var out []Vec2
	d0 := normal.Dot(in[0]) - offset
	d1 := normal.Dot(in[1]) - offset
	if d0 <= 0 {
		out = append(out, in[0])
	}
	if d1 <= 0 {
		out = append(out, in[1])
	}
	if d0*d1 < 0 {
		t := d0 / (d0 - d1)
		out = append(out, in[0].Add(in[1].Sub(in[0]).Mul(t)))
	}
	return out
}

// collidePolygons 用 SAT + 参考面裁剪检测两个凸多边形。
// 法线从 a 指向 b。
func collidePolygons(a, b *body) (manifold, bool) {
	vertsA := a.worldVertices()
	vertsB := b.worldVertices()

	sepA, faceA := maxSeparation(vertsA, vertsB)
	if sepA > 0 {
		return manifold{}, false
	}
	sepB, faceB := maxSeparation(vertsB, vertsA)
	if sepB > 0 {
		return manifold{}, false
	}

	// 选参考面：分离量较大者，带迟滞以稳定参考面选择。
	var refVerts, incVerts []Vec2
	var refEdge int
	var flip int
	if sepB > 0.95*sepA+0.001 {
		refVerts, incVerts = vertsB, vertsA
		refEdge = faceB
		flip = 1
	} else {
		refVerts, incVerts = vertsA, vertsB
		refEdge = faceA
		flip = 0
	}

	// 入射边：入射多边形上与参考法线最反向的面。
	nr := len(refVerts)
	rv1 := refVerts[refEdge]
	rv2 := refVerts[(refEdge+1)%nr]
	refNormal := rv2.Sub(rv1).RPerp().Norm()
	ni := len(incVerts)
	incEdge := 0
	minDot := math.Inf(1)
	for i := 0; i < ni; i++ {
		v1 := incVerts[i]
		v2 := incVerts[(i+1)%ni]
		n := v2.Sub(v1).RPerp().Norm()
		if d := refNormal.Dot(n); d < minDot {
			minDot = d
			incEdge = i
		}
	}
	incident := [2]Vec2{incVerts[incEdge], incVerts[(incEdge+1)%ni]}

	// 用参考边的两个侧面裁剪入射边。
	refTangent := rv2.Sub(rv1).Norm()
	// 侧面 1：过 rv1，法线 = -refTangent。
	pts := clipSegment(refTangent.Mul(-1), -refTangent.Dot(rv1), incident)
	if len(pts) < 2 {
		return manifold{}, false
	}
	// 侧面 2：过 rv2，法线 = refTangent。
	pts = clipSegment(refTangent, refTangent.Dot(rv2), [2]Vec2{pts[0], pts[1]})
	if len(pts) < 2 {
		return manifold{}, false
	}

	// 保留参考面以下的点。
	frontOffset := refNormal.Dot(rv1)
	var points []contactPoint
	for i, p := range pts {
		sep := refNormal.Dot(p) - frontOffset
		if sep <= 0 {
			points = append(points, contactPoint{
				pos:   p,
				depth: -sep,
				feature: featureID{
					refEdge: refEdge,
					incEdge: incEdge + i,
					flip:    flip,
				},
			})
		}
	}
	if len(points) == 0 {
		return manifold{}, false
	}

	normal := refNormal
	if flip == 1 {
		normal = normal.Mul(-1)
	}
	return manifold{a: a, b: b, normal: normal, points: points}, true
}
