package rigidbody

import "math"

const (
	// baumgarte 位置误差修正系数。
	baumgarte = 0.2
	// slop 允许的残余穿透（米）。
	slop = 0.005
)

// pairKey 标识一对刚体（a < b）。
type pairKey struct{ a, b int }

// cachedPoint 跨步保存的接触点热启动数据。
type cachedPoint struct {
	feature featureID
	pn, pt  float64
}

// aabb 粗略包围盒。
type aabb struct {
	min, max Vec2
}

func (b *body) aabb() aabb {
	if b.shape.Type == ShapeCircle {
		r := Vec2{b.shape.Radius, b.shape.Radius}
		return aabb{b.pos.Sub(r), b.pos.Add(r)}
	}
	verts := b.worldVertices()
	min, max := verts[0], verts[0]
	for _, v := range verts[1:] {
		if v.X < min.X {
			min.X = v.X
		}
		if v.Y < min.Y {
			min.Y = v.Y
		}
		if v.X > max.X {
			max.X = v.X
		}
		if v.Y > max.Y {
			max.Y = v.Y
		}
	}
	return aabb{min, max}
}

func (a aabb) overlaps(b aabb) bool {
	return a.min.X <= b.max.X && b.min.X <= a.max.X &&
		a.min.Y <= b.max.Y && b.min.Y <= a.max.Y
}

// stepOnce 推进世界一个固定步长。
func (w *World) stepOnce() {
	dt := FixedDT

	// 1. 积分外力（重力）。
	for _, b := range w.bodies {
		if b.typ == Dynamic {
			b.vel = b.vel.Add(w.gravity.Mul(dt))
		}
	}

	// 2. 碰撞检测（按 id 顺序遍历，保证确定性）。
	boxes := make([]aabb, len(w.bodies))
	for i, b := range w.bodies {
		boxes[i] = b.aabb()
	}
	var manifolds []manifold
	for i := 0; i < len(w.bodies); i++ {
		for j := i + 1; j < len(w.bodies); j++ {
			a, b := w.bodies[i], w.bodies[j]
			if a.typ == Static && b.typ == Static {
				continue
			}
			if !boxes[i].overlaps(boxes[j]) {
				continue
			}
			if m, ok := collide(a, b); ok {
				manifolds = append(manifolds, m)
			}
		}
	}

	// 3. 预计算约束 + 热启动。
	for mi := range manifolds {
		m := &manifolds[mi]
		a, b := m.a, m.b
		e := math.Max(a.restitution, b.restitution)
		mu := math.Sqrt(a.friction * b.friction)
		tangent := Vec2{m.normal.Y, -m.normal.X}
		key := pairKey{a.id, b.id}
		cached := w.cache[key]
		for pi := range m.points {
			p := &m.points[pi]
			// 热启动：按特征匹配上一步的累计冲量。
			for _, cp := range cached {
				if cp.feature == p.feature {
					p.pn = cp.pn
					p.pt = cp.pt
					break
				}
			}
			p.rA = p.pos.Sub(a.pos)
			p.rB = p.pos.Sub(b.pos)
			rnA := p.rA.Cross(m.normal)
			rnB := p.rB.Cross(m.normal)
			kn := a.invMass + b.invMass + a.invInertia*rnA*rnA + b.invInertia*rnB*rnB
			p.massN = 1 / kn
			rtA := p.rA.Cross(tangent)
			rtB := p.rB.Cross(tangent)
			kt := a.invMass + b.invMass + a.invInertia*rtA*rtA + b.invInertia*rtB*rtB
			p.massT = 1 / kt
			// Baumgarte 位置修正。
			p.bias = baumgarte / dt * math.Max(0, p.depth-slop)
			// 弹性：接近速度超过阈值才反弹。
			vn := relVel(a, b, p).Dot(m.normal)
			if vn < -restitutionThreshold {
				if rb := -e * vn; rb > p.bias {
					p.bias = rb
				}
			}
			// 应用累计冲量（热启动）。
			imp := m.normal.Mul(p.pn).Add(tangent.Mul(p.pt))
			applyImpulse(a, b, p, imp)
		}
		m.mu = mu
		m.tangent = tangent
	}

	// 4. 迭代求解速度约束。
	for iter := 0; iter < solverIterations; iter++ {
		for mi := range manifolds {
			m := &manifolds[mi]
			a, b := m.a, m.b
			for pi := range m.points {
				p := &m.points[pi]
				rv := relVel(a, b, p)
				// 法向。
				vn := rv.Dot(m.normal)
				dPn := p.massN * (p.bias - vn)
				pn0 := p.pn
				p.pn = math.Max(pn0+dPn, 0)
				applyImpulse(a, b, p, m.normal.Mul(p.pn-pn0))
				// 切向摩擦。
				rv = relVel(a, b, p)
				vt := rv.Dot(m.tangent)
				dPt := p.massT * (-vt)
				maxPt := m.mu * p.pn
				pt0 := p.pt
				p.pt = clamp(pt0+dPt, -maxPt, maxPt)
				applyImpulse(a, b, p, m.tangent.Mul(p.pt-pt0))
			}
		}
	}

	// 5. 积分位置。
	for _, b := range w.bodies {
		if b.typ == Static {
			continue
		}
		b.pos = b.pos.Add(b.vel.Mul(dt))
		b.angle += b.angularVel * dt
	}

	// 6. 更新热启动缓存与对外接触列表。
	w.cache = make(map[pairKey][]cachedPoint, len(manifolds))
	w.contacts = w.contacts[:0]
	for _, m := range manifolds {
		pts := make([]cachedPoint, len(m.points))
		cps := make([]Vec2, len(m.points))
		depth := 0.0
		for i, p := range m.points {
			pts[i] = cachedPoint{feature: p.feature, pn: p.pn, pt: p.pt}
			cps[i] = p.pos
			if p.depth > depth {
				depth = p.depth
			}
		}
		w.cache[pairKey{m.a.id, m.b.id}] = pts
		w.contacts = append(w.contacts, Contact{
			BodyA:  m.a.id,
			BodyB:  m.b.id,
			Normal: m.normal,
			Depth:  depth,
			Points: cps,
		})
	}
}

// relVel 返回接触点处 b 相对 a 的速度。
func relVel(a, b *body, p *contactPoint) Vec2 {
	return b.vel.Add(CrossSV(b.angularVel, p.rB)).
		Sub(a.vel.Add(CrossSV(a.angularVel, p.rA)))
}

// applyImpulse 对接触点施加冲量（a 负向，b 正向）。
func applyImpulse(a, b *body, p *contactPoint, imp Vec2) {
	a.vel = a.vel.Sub(imp.Mul(a.invMass))
	a.angularVel -= a.invInertia * p.rA.Cross(imp)
	b.vel = b.vel.Add(imp.Mul(b.invMass))
	b.angularVel += b.invInertia * p.rB.Cross(imp)
}

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
