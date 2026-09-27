package rigid2d

import "math"

// contact 内部接触点（求解器用）。
type contact struct {
	position   Vec2
	normal     Vec2 // 从 bodyA 指向 bodyB 的单位法线
	separation float64
	pn         float64 // 累计法向冲量
	pt         float64 // 累计切向冲量
	feature    uint64  // 用于跨步匹配（热启动）
}

func packFeature(inEdge, outEdge, refEdge int, flip bool) uint64 {
	f := uint64(uint8(inEdge)) | uint64(uint8(outEdge))<<8 | uint64(uint8(refEdge))<<16
	if flip {
		f |= 1 << 24
	}
	return f
}

// collide 计算两刚体间的接触，法线从 a 指向 b。
// va/vb 为预先算好的世界坐标顶点（圆为 nil）。
func collide(a, b *Body, va, vb []Vec2) []contact {
	switch {
	case a.Shape.Type == ShapeCircle && b.Shape.Type == ShapeCircle:
		return collideCircleCircle(a, b)
	case a.Shape.Type == ShapeCircle && b.Shape.Type == ShapePolygon:
		return collideCirclePoly(a, b)
	case a.Shape.Type == ShapePolygon && b.Shape.Type == ShapeCircle:
		cs := collideCirclePoly(b, a)
		for i := range cs {
			cs[i].normal = Neg(cs[i].normal)
		}
		return cs
	default:
		return collidePolyPoly(a, b, va, vb)
	}
}

func collideCircleCircle(a, b *Body) []contact {
	d := Sub(b.Position, a.Position)
	r := a.Shape.Radius + b.Shape.Radius
	d2 := Dot(d, d)
	if d2 > r*r {
		return nil
	}
	dist := math.Sqrt(d2)
	var n Vec2
	if dist > 1e-12 {
		n = Mul(d, 1/dist)
	} else {
		n = Vec2{1, 0}
	}
	sep := dist - r
	p := Add(a.Position, Mul(n, a.Shape.Radius+0.5*sep))
	return []contact{{position: p, normal: n, separation: sep}}
}

// collideCirclePoly 圆与多边形，法线从圆指向多边形。
func collideCirclePoly(circle, poly *Body) []contact {
	rp := NewRot(poly.Angle)
	verts := poly.Shape.Vertices
	n := len(verts)
	// 圆心到多边形局部坐标
	c := rp.MulTV(Sub(circle.Position, poly.Position))
	radius := circle.Shape.Radius

	// 找最大分离面
	maxSep := -math.MaxFloat64
	face := 0
	for i := 0; i < n; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%n]
		edge := Sub(v2, v1)
		normal := Normalize(Vec2{edge.Y, -edge.X}) // 外法线（CCW）
		s := Dot(normal, Sub(c, v1))
		if s > radius {
			return nil
		}
		if s > maxSep {
			maxSep = s
			face = i
		}
	}

	v1 := verts[face]
	v2 := verts[(face+1)%n]
	edge := Sub(v2, v1)
	faceNormal := Normalize(Vec2{edge.Y, -edge.X})

	var normalLocal Vec2
	var sep float64
	if maxSep < vertexEpsilon {
		// 圆心在多边形内或贴面
		normalLocal = faceNormal
		sep = maxSep - radius
	} else {
		u1 := Dot(Sub(c, v1), edge)
		u2 := Dot(Sub(c, v2), Neg(edge))
		switch {
		case u1 < 0: // 靠近 v1
			d := Sub(c, v1)
			dist := Len(d)
			if dist > radius {
				return nil
			}
			normalLocal = Mul(d, -1/dist)
			sep = dist - radius
		case u2 < 0: // 靠近 v2
			d := Sub(c, v2)
			dist := Len(d)
			if dist > radius {
				return nil
			}
			normalLocal = Mul(d, -1/dist)
			sep = dist - radius
		default: // 面区域
			normalLocal = Neg(faceNormal)
			sep = maxSep - radius
		}
	}
	normal := rp.MulV(normalLocal)
	p := Add(circle.Position, Mul(normal, radius+0.5*sep))
	return []contact{{position: p, normal: normal, separation: sep}}
}

type clipVertex struct {
	v               Vec2
	inEdge, outEdge int
}

func clipSegmentToLine(in [2]clipVertex, normal Vec2, offset float64, clipEdge int) []clipVertex {
	var out []clipVertex
	d0 := Dot(normal, in[0].v) - offset
	d1 := Dot(normal, in[1].v) - offset
	if d0 <= 0 {
		out = append(out, in[0])
	}
	if d1 <= 0 {
		out = append(out, in[1])
	}
	if d0*d1 < 0 {
		interp := d0 / (d0 - d1)
		v := Add(in[0].v, Mul(Sub(in[1].v, in[0].v), interp))
		var cv clipVertex
		cv.v = v
		if d0 > 0 {
			cv.inEdge = clipEdge
			cv.outEdge = in[0].outEdge
		} else {
			cv.outEdge = clipEdge
			cv.inEdge = in[0].inEdge
		}
		out = append(out, cv)
	}
	return out
}

// polyNormals 计算各边外法线，写入 buf 并返回前 n 个。
func polyNormals(verts []Vec2, buf *[maxPolygonVertices]Vec2) []Vec2 {
	n := len(verts)
	out := buf[:n]
	for i := 0; i < n; i++ {
		edge := Sub(verts[(i+1)%n], verts[i])
		out[i] = Normalize(Vec2{edge.Y, -edge.X})
	}
	return out
}

// findMaxSeparation p1 各面相对 p2 的最大分离距离及对应面。
func findMaxSeparation(p1 []Vec2, n1 []Vec2, p2 []Vec2) (float64, int) {
	bestSep := -math.MaxFloat64
	bestEdge := 0
	for i := range p1 {
		nv := n1[i]
		v := p1[i]
		minDot := math.MaxFloat64
		for _, v2 := range p2 {
			d := Dot(nv, Sub(v2, v))
			if d < minDot {
				minDot = d
			}
		}
		if minDot > bestSep {
			bestSep = minDot
			bestEdge = i
		}
	}
	return bestSep, bestEdge
}

func findIncidentEdge(refNormal Vec2, incPoly []Vec2, incNormals []Vec2) [2]clipVertex {
	minDot := math.MaxFloat64
	idx := 0
	for i, nv := range incNormals {
		d := Dot(refNormal, nv)
		if d < minDot {
			minDot = d
			idx = i
		}
	}
	return [2]clipVertex{
		{v: incPoly[idx], inEdge: idx, outEdge: -1},
		{v: incPoly[(idx+1)%len(incPoly)], inEdge: (idx + 1) % len(incPoly), outEdge: -1},
	}
}

// collidePolyPoly 参考面裁剪（box2d-lite 风格），最多 2 个接触点。
func collidePolyPoly(a, b *Body, va, vb []Vec2) []contact {
	var naBuf, nbBuf [maxPolygonVertices]Vec2
	na := polyNormals(va, &naBuf)
	nb := polyNormals(vb, &nbBuf)

	sepA, edgeA := findMaxSeparation(va, na, vb)
	if sepA > 0 {
		return nil
	}
	sepB, edgeB := findMaxSeparation(vb, nb, va)
	if sepB > 0 {
		return nil
	}

	const relTol = 0.95
	const absTol = 0.01

	var refPoly, incPoly []Vec2
	var refNormals []Vec2
	var incNormals []Vec2
	var refEdge int
	var flip bool
	if sepB > relTol*sepA+absTol {
		refPoly, refNormals, refEdge = vb, nb, edgeB
		incPoly = va
		incNormals = na
		flip = true
	} else {
		refPoly, refNormals, refEdge = va, na, edgeA
		incPoly = vb
		incNormals = nb
	}

	incident := findIncidentEdge(refNormals[refEdge], incPoly, incNormals)

	rv1 := refPoly[refEdge]
	rv2 := refPoly[(refEdge+1)%len(refPoly)]
	refFaceNormal := refNormals[refEdge]
	refC := Dot(refFaceNormal, rv1)
	sideNormal := Normalize(Sub(rv2, rv1))
	negSide := -Dot(sideNormal, rv1)
	posSide := Dot(sideNormal, rv2)

	clip1 := clipSegmentToLine(incident, Neg(sideNormal), negSide, refEdge)
	if len(clip1) < 2 {
		return nil
	}
	var clip1Arr [2]clipVertex
	copy(clip1Arr[:], clip1)
	clip2 := clipSegmentToLine(clip1Arr, sideNormal, posSide, (refEdge+1)%len(refPoly))
	if len(clip2) < 2 {
		return nil
	}

	normal := refFaceNormal
	if flip {
		normal = Neg(normal)
	}

	var out []contact
	for _, cp := range clip2 {
		sep := Dot(refFaceNormal, cp.v) - refC
		if sep <= 0 {
			out = append(out, contact{
				position:   cp.v,
				normal:     normal,
				separation: sep,
				feature:    packFeature(cp.inEdge, cp.outEdge, refEdge, flip),
			})
		}
	}
	return out
}
