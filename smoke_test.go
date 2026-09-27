package rigid2d

import (
	"math"
	"testing"
)

func boxShape(t testing.TB, halfW, halfH float64) Shape {
	s, err := NewPolygon([]Vec2{
		{-halfW, -halfH}, {halfW, -halfH}, {halfW, halfH}, {-halfW, halfH},
	})
	if err != nil {
		panic(err)
	}
	return s
}

func stepN(t *testing.T, w *World, n int) {
	t.Helper()
	for n > 0 {
		k := n
		if k > MaxStepsPerCall {
			k = MaxStepsPerCall
		}
		if err := w.Step(k); err != nil {
			t.Fatal(err)
		}
		n -= k
	}
}

func TestBoxFallsAndRests(t *testing.T) {
	w := NewWorld()
	ground, _ := w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.6,
	})
	box, _ := w.AddBody(BodyDef{
		Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
		Position: Vec2{0, 5}, Mass: 1, Friction: 0.6,
	})
	stepN(t, w, 600)
	st, _ := w.GetBodyState(box)
	if math.Abs(st.Position.Y-0.5) > 0.02 {
		t.Fatalf("box should rest at y≈0.5, got %v", st.Position.Y)
	}
	if Len(st.LinVel) > 0.05 {
		t.Fatalf("box should be at rest, vel=%v", st.LinVel)
	}
	_ = ground
	t.Logf("rest pos=%v angle=%v vel=%v", st.Position, st.Angle, st.LinVel)
}

func TestStackOf10(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.6,
	})
	ids := make([]int, 10)
	for i := 0; i < 10; i++ {
		id, _ := w.AddBody(BodyDef{
			Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
			Position: Vec2{0, 0.5 + float64(i)*1.001},
			Mass:     1, Friction: 0.6,
		})
		ids[i] = id
	}
	stepN(t, w, 600)
	top, _ := w.GetBodyState(ids[9])
	t.Logf("top box pos=%v angle=%v vel=%v", top.Position, top.Angle, top.LinVel)
	if math.Abs(top.Position.X) > 0.1 {
		t.Fatalf("top box drifted horizontally: x=%v", top.Position.X)
	}
	for _, id := range ids {
		st, _ := w.GetBodyState(id)
		if math.Abs(st.Angle) > 0.05 {
			t.Fatalf("box %d rotated: %v", id, st.Angle)
		}
		if Len(st.LinVel) > 0.05 {
			t.Fatalf("box %d moving: %v", id, st.LinVel)
		}
	}
	for _, c := range w.Contacts() {
		if c.Penetration > 0.02 {
			t.Fatalf("penetration %v between %d and %d", c.Penetration, c.BodyA, c.BodyB)
		}
	}
}
