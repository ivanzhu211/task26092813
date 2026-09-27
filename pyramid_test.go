package rigid2d

import (
	"testing"
)

// 55 个箱子、底层 10 个的金字塔，10 秒不塌。
func TestPyramid55(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 100, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.8,
	})
	var ids []int
	y := 0.5
	for row := 0; row < 10; row++ {
		count := 10 - row
		x0 := -float64(count-1) * 0.5 * 1.05
		for i := 0; i < count; i++ {
			id, _ := w.AddBody(BodyDef{
				Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
				Position: Vec2{x0 + float64(i)*1.05, y},
				Mass:     1, Friction: 0.8,
			})
			ids = append(ids, id)
		}
		y += 1.001
	}
	if len(ids) != 55 {
		t.Fatalf("expected 55 boxes, got %d", len(ids))
	}
	// 跑 1 秒，记录基准
	stepN(t, w, 60)
	base := make(map[int]Vec2)
	for _, id := range ids {
		st, _ := w.GetBodyState(id)
		base[id] = st.Position
	}
	// 再跑 9 秒
	stepN(t, w, 540)
	for _, id := range ids {
		st, _ := w.GetBodyState(id)
		d := Len(Sub(st.Position, base[id]))
		if d > 0.1 {
			t.Fatalf("box %d moved %v m after t=1s (pos=%v base=%v)", id, d, st.Position, base[id])
		}
	}
	maxDrift := 0.0
	for _, id := range ids {
		st, _ := w.GetBodyState(id)
		d := Len(Sub(st.Position, base[id]))
		if d > maxDrift {
			maxDrift = d
		}
	}
	t.Logf("max drift after 1s: %f", maxDrift)
}

// 两个箱子叠放，质量比 10:1，重的在上，5 秒穿透不超 0.02m。
func TestHeavyOnLight(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.5,
	})
	w.AddBody(BodyDef{
		Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
		Position: Vec2{0, 0.5}, Mass: 1, Friction: 0.5,
	})
	w.AddBody(BodyDef{
		Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
		Position: Vec2{0, 1.501}, Mass: 10, Friction: 0.5,
	})
	stepN(t, w, 300)
	for _, c := range w.Contacts() {
		if c.Penetration > 0.02 {
			t.Fatalf("penetration %v between %d and %d", c.Penetration, c.BodyA, c.BodyB)
		}
	}
	maxPen := 0.0
	for _, c := range w.Contacts() {
		if c.Penetration > maxPen {
			maxPen = c.Penetration
		}
	}
	t.Logf("max penetration: %f", maxPen)
}
