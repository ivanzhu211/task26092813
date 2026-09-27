package rigidbody

import (
	"math"
	"testing"
)

const groundFriction = 0.6

func addGround(t *testing.T, w *World) {
	t.Helper()
	addBox(t, w, BodyParams{
		Shape: Box(100, 1), Type: Static, Position: Vec2{0, -0.5},
		Friction: groundFriction,
	})
}

// TestStackOfTenBoxes 10 个 1m 箱子竖直叠放 10 秒。
func TestStackOfTenBoxes(t *testing.T) {
	w := NewWorld(Vec2{})
	addGround(t, w)
	const n = 10
	for i := 0; i < n; i++ {
		addBox(t, w, BodyParams{
			Shape: Box(1, 1), Type: Dynamic, Mass: 1,
			Position: Vec2{0, 0.5 + float64(i)*1.0005},
			Friction: 0.6,
		})
	}
	// 10 秒 = 600 步。
	for s := 0; s < 5; s++ {
		if err := w.Step(120); err != nil {
			t.Fatal(err)
		}
	}
	top, err := w.BodyState(n) // 地面 id=0，最上面的箱子 id=n
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(top.Position.X) > 0.1 {
		t.Fatalf("top box horizontal drift %.4f m > 0.1 m", math.Abs(top.Position.X))
	}
	for i := 1; i <= n; i++ {
		st, _ := w.BodyState(i)
		if math.Abs(st.Angle) > 0.05 {
			t.Fatalf("box %d angle %.4f rad > 0.05", i, math.Abs(st.Angle))
		}
		if v := st.Velocity.Len(); v > 0.05 {
			t.Fatalf("box %d speed %.4f m/s > 0.05", i, v)
		}
	}
	for _, c := range w.Contacts() {
		if c.Depth > 0.02 {
			t.Fatalf("contact %d-%d depth %.4f m > 0.02", c.BodyA, c.BodyB, c.Depth)
		}
	}
}

// TestPyramid 55 个箱子搭底层 10 个的金字塔，10 秒不塌。
func TestPyramid(t *testing.T) {
	w := NewWorld(Vec2{})
	addGround(t, w)
	const base = 10
	const spacing = 1.05
	y := 0.5
	var ids []int
	for row := 0; row < base; row++ {
		count := base - row
		x0 := -float64(count-1) * spacing / 2
		for i := 0; i < count; i++ {
			id := addBox(t, w, BodyParams{
				Shape: Box(1, 1), Type: Dynamic, Mass: 1,
				Position: Vec2{x0 + float64(i)*spacing, y},
				Friction: 0.6,
			})
			ids = append(ids, id)
		}
		y += 1.0005
	}
	if len(ids) != 55 {
		t.Fatalf("expected 55 boxes, got %d", len(ids))
	}
	// 跑 1 秒，记录基准位置。
	if err := w.Step(60); err != nil {
		t.Fatal(err)
	}
	basePos := make(map[int]Vec2)
	for _, id := range ids {
		st, _ := w.BodyState(id)
		basePos[id] = st.Position
	}
	// 再跑 9 秒。
	for s := 0; s < 9; s++ {
		if err := w.Step(60); err != nil {
			t.Fatal(err)
		}
	}
	for _, id := range ids {
		st, _ := w.BodyState(id)
		d := st.Position.Sub(basePos[id]).Len()
		if d > 0.1 {
			t.Fatalf("box %d moved %.4f m from 1s baseline > 0.1 m", id, d)
		}
	}
}

// TestHeavyOnLight 两个箱子叠放，质量比 10:1，重的在上，5 秒穿透不超标。
func TestHeavyOnLight(t *testing.T) {
	w := NewWorld(Vec2{})
	addGround(t, w)
	addBox(t, w, BodyParams{
		Shape: Box(1, 1), Type: Dynamic, Mass: 1,
		Position: Vec2{0, 0.5}, Friction: 0.5,
	})
	addBox(t, w, BodyParams{
		Shape: Box(1, 1), Type: Dynamic, Mass: 10,
		Position: Vec2{0, 1.5005}, Friction: 0.5,
	})
	for s := 0; s < 5; s++ {
		if err := w.Step(60); err != nil {
			t.Fatal(err)
		}
		for _, c := range w.Contacts() {
			if c.Depth > 0.02 {
				t.Fatalf("second %d: contact %d-%d depth %.4f > 0.02",
					s, c.BodyA, c.BodyB, c.Depth)
			}
		}
	}
}
