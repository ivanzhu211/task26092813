package rigid2d

import (
	"testing"
	"time"
)

// 500 个刚体、大部分互相接触时，平均每步 ≤ 16ms。
func TestPerf500Bodies(t *testing.T) {
	if testing.Short() {
		t.Skip("short mode")
	}
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 100, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.6,
	})
	// 25 列 × 20 层 = 500 个动态箱子中的 499 个，紧密堆叠
	count := 0
	for row := 0; row < 20; row++ {
		for col := 0; col < 25; col++ {
			if count >= 499 {
				break
			}
			w.AddBody(BodyDef{
				Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
				Position: Vec2{float64(col-12) * 1.002, 0.5 + float64(row)*1.002},
				Mass:     1, Friction: 0.6,
			})
			count++
		}
	}
	if w.BodyCount() != 500 {
		t.Fatalf("bodies=%d", w.BodyCount())
	}
	// 先沉降 2 秒，让大部分箱子进入接触
	stepN(t, w, 120)
	// 计时 120 步
	start := time.Now()
	if err := w.Step(120); err != nil {
		t.Fatal(err)
	}
	elapsed := time.Since(start)
	avg := elapsed / 120
	t.Logf("avg step time: %v (contacts=%d)", avg, len(w.Contacts()))
	if avg > 16*time.Millisecond {
		t.Fatalf("avg step %v exceeds 16ms", avg)
	}
}
