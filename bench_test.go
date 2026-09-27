package rigid2d

import "testing"

func BenchmarkStep500(b *testing.B) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(b, 100, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.6,
	})
	count := 0
	for row := 0; row < 20; row++ {
		for col := 0; col < 25; col++ {
			if count >= 499 {
				break
			}
			w.AddBody(BodyDef{
				Type: Dynamic, Shape: boxShape(b, 0.5, 0.5),
				Position: Vec2{float64(col-12) * 1.002, 0.5 + float64(row)*1.002},
				Mass:     1, Friction: 0.6,
			})
			count++
		}
	}
	for i := 0; i < 120; i++ {
		w.Step(1)
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w.Step(1)
	}
}
