package rigid2d

import "math"

// 求解器常数（box2d-lite 风格）。
const (
	baumgarteBeta   = 0.2
	penetrationSlop = 0.005
	maxBiasVelocity = 10.0
	restitutionSlop = 1.0 // 法向接近速度低于 1m/s 不反弹
)

// arbiter 一对刚体间的接触约束。
type arbiter struct {
	a, b        *Body // a.ID < b.ID
	contacts    []contact
	friction    float64
	restitution float64
	alive       bool // 瞬态：本步 broadPhase 是否仍接触
}

// update 用新碰撞结果刷新接触点，保留匹配特征的累计冲量（热启动）。
func (arb *arbiter) update(newContacts []contact) {
	var oldBuf [4]contact
	old := oldBuf[:copy(oldBuf[:], arb.contacts)]
	merged := arb.contacts[:0]
	for _, nc := range newContacts {
		nc.pn, nc.pt = 0, 0
		for _, oc := range old {
			if oc.feature == nc.feature {
				nc.pn = oc.pn
				nc.pt = oc.pt
				break
			}
		}
		merged = append(merged, nc)
	}
	arb.contacts = merged
}

// contactConstraint 预计算的求解数据。
type contactConstraint struct {
	c           *contact
	r1, r2      Vec2
	massNormal  float64
	massTangent float64
	bias        float64
	normal      Vec2
}

type arbiterState struct {
	arb *arbiter
	cs  []contactConstraint
}

// preStep 预计算质量阵与偏置，并施加累计冲量（热启动）。
func (arb *arbiter) preStep(invDt float64) *arbiterState {
	a, b := arb.a, arb.b
	st := &arbiterState{arb: arb}
	for i := range arb.contacts {
		c := &arb.contacts[i]
		r1 := Sub(c.position, a.Position)
		r2 := Sub(c.position, b.Position)

		rn1 := Cross(r1, c.normal)
		rn2 := Cross(r2, c.normal)
		kn := a.InvMass + b.InvMass + a.InvInertia*rn1*rn1 + b.InvInertia*rn2*rn2
		massNormal := 0.0
		if kn != 0 {
			massNormal = 1 / kn
		}
		tangent := CrossVS(c.normal, 1.0)
		rt1 := Cross(r1, tangent)
		rt2 := Cross(r2, tangent)
		kt := a.InvMass + b.InvMass + a.InvInertia*rt1*rt1 + b.InvInertia*rt2*rt2
		massTangent := 0.0
		if kt != 0 {
			massTangent = 1 / kt
		}

		// Baumgarte 位置偏置
		bias := -baumgarteBeta * invDt * math.Min(0, c.separation+penetrationSlop)
		bias = math.Max(bias, -maxBiasVelocity)

		// 弹性偏置：接近速度低于阈值不反弹
		dv := Add(Sub(b.LinVel, a.LinVel), Sub(CrossSV(b.AngVel, r2), CrossSV(a.AngVel, r1)))
		vn := Dot(dv, c.normal)
		if vn < -restitutionSlop {
			restBias := -arb.restitution * vn
			if restBias > bias {
				bias = restBias
			}
		}

		st.cs = append(st.cs, contactConstraint{
			c: c, r1: r1, r2: r2,
			massNormal: massNormal, massTangent: massTangent,
			bias: bias, normal: c.normal,
		})

		// 热启动：施加累计冲量
		p := Add(Mul(c.normal, c.pn), Mul(tangent, c.pt))
		a.LinVel = Sub(a.LinVel, Mul(p, a.InvMass))
		a.AngVel -= a.InvInertia * Cross(r1, p)
		b.LinVel = Add(b.LinVel, Mul(p, b.InvMass))
		b.AngVel += b.InvInertia * Cross(r2, p)
	}
	return st
}

// applyImpulse 执行一轮法向+切向冲量求解。
func (st *arbiterState) applyImpulse() {
	arb := st.arb
	a, b := arb.a, arb.b
	for i := range st.cs {
		cc := &st.cs[i]
		c := cc.c
		n := cc.normal

		// 法向冲量
		dv := Add(Sub(b.LinVel, a.LinVel), Sub(CrossSV(b.AngVel, cc.r2), CrossSV(a.AngVel, cc.r1)))
		vn := Dot(dv, n)
		dPn := cc.massNormal * (-vn + cc.bias)
		pn0 := c.pn
		c.pn = math.Max(pn0+dPn, 0)
		dPn = c.pn - pn0
		p := Mul(n, dPn)
		a.LinVel = Sub(a.LinVel, Mul(p, a.InvMass))
		a.AngVel -= a.InvInertia * Cross(cc.r1, p)
		b.LinVel = Add(b.LinVel, Mul(p, b.InvMass))
		b.AngVel += b.InvInertia * Cross(cc.r2, p)

		// 切向冲量（摩擦）
		tangent := CrossVS(n, 1.0)
		dv = Add(Sub(b.LinVel, a.LinVel), Sub(CrossSV(b.AngVel, cc.r2), CrossSV(a.AngVel, cc.r1)))
		vt := Dot(dv, tangent)
		dPt := cc.massTangent * (-vt)
		maxPt := arb.friction * c.pn
		pt0 := c.pt
		c.pt = math.Max(-maxPt, math.Min(pt0+dPt, maxPt))
		dPt = c.pt - pt0
		p = Mul(tangent, dPt)
		a.LinVel = Sub(a.LinVel, Mul(p, a.InvMass))
		a.AngVel -= a.InvInertia * Cross(cc.r1, p)
		b.LinVel = Add(b.LinVel, Mul(p, b.InvMass))
		b.AngVel += b.InvInertia * Cross(cc.r2, p)
	}
}
