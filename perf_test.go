package rigidbody

import (
	"testing"
	"time"
)

// TestPerformance500Bodies 500 个刚体大部分互相接触，平均每步 ≤16ms。
func TestPerformance500Bodies(t *testing.T) {
	w := NewWorld(Vec2{})
	// 静态地面。
	addBox(t, w, BodyParams{
		Shape: Box(200, 1), Type: Static, Position: Vec2{0, -0.5},
		Friction: 0.6,
	})
	// 499 个动态箱子紧密堆叠成互相接触的多层墙。
	count := 0
	cols := 25
	for row := 0; row < 20 && count < 499; row++ {
		for col := 0; col < cols && count < 499; col++ {
			x := float64(col-cols/2) * 1.0
			y := 0.5 + float64(row)*1.0
			addBox(t, w, BodyParams{
				Shape: Box(1, 1), Type: Dynamic, Mass: 1,
				Position: Vec2{x, y}, Friction: 0.6,
			})
			count++
		}
	}
	if len(w.Bodies()) != MaxBodies {
		t.Fatalf("expected %d bodies, got %d", MaxBodies, len(w.Bodies()))
	}
	// 预热 30 步让接触稳定。
	if err := w.Step(30); err != nil {
		t.Fatal(err)
	}
	// 计时 120 步。
	start := time.Now()
	if err := w.Step(120); err != nil {
		t.Fatal(err)
	}
	avg := time.Since(start) / 120
	t.Logf("avg step time: %v", avg)
	if avg > 16*time.Millisecond {
		t.Fatalf("avg step time %v > 16ms", avg)
	}
}
