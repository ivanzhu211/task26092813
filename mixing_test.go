package rigid2d

import (
	"math"
	"testing"
)

// 摩擦取两者乘积的平方根，弹性取两者较大值。
func TestMaterialMixing(t *testing.T) {
	w := NewWorld()
	a, _ := w.AddBody(BodyDef{
		Type: Dynamic, Shape: boxShape(t, 0.5, 0.5), Mass: 1,
		Position: Vec2{0, 0.5}, Friction: 0.4, Restitution: 0.1,
	})
	b, _ := w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.9, Restitution: 0.3,
	})
	if err := w.Step(1); err != nil {
		t.Fatal(err)
	}
	arb := w.arbiters[arbiterKey{a, b}]
	if arb == nil {
		t.Fatal("expected arbiter")
	}
	wantF := math.Sqrt(0.4 * 0.9)
	if math.Abs(arb.friction-wantF) > 1e-12 {
		t.Fatalf("friction=%f want sqrt(0.36)=%f", arb.friction, wantF)
	}
	if arb.restitution != 0.3 {
		t.Fatalf("restitution=%f want max=0.3", arb.restitution)
	}
}

// 法向接近速度低于 1m/s 不反弹。
func TestRestitutionThreshold(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Restitution: 0.9,
	})
	circ, _ := NewCircle(0.5)
	// 落地速度约 0.9m/s（< 1m/s 阈值），弹性 0.9 也不应反弹
	id, _ := w.AddBody(BodyDef{
		Type: Dynamic, Shape: circ, Position: Vec2{0, 0.53},
		Mass: 1, Restitution: 0.9,
	})
	maxVy := 0.0
	touched := false
	for i := 0; i < 120; i++ {
		if err := w.Step(1); err != nil {
			t.Fatal(err)
		}
		st, _ := w.GetBodyState(id)
		if len(w.Contacts()) > 0 {
			touched = true
		}
		if touched && st.LinVel.Y > maxVy {
			maxVy = st.LinVel.Y
		}
	}
	if !touched {
		t.Fatal("circle never touched ground")
	}
	// 若弹性生效，反弹速度约 0.89m/s；仅位置修正时不超过 ~0.1
	if maxVy > 0.3 {
		t.Fatalf("bounced with impact speed < 1m/s, maxVy=%f", maxVy)
	}
}
