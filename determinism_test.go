package rigid2d

import "testing"

// 搭建一个含圆、箱子、斜坡的混合场景。
func buildScene(t *testing.T) *World {
	t.Helper()
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 100, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.7, Restitution: 0.1,
	})
	// 一摞箱子
	for i := 0; i < 5; i++ {
		w.AddBody(BodyDef{
			Type: Dynamic, Shape: boxShape(t, 0.5, 0.5),
			Position: Vec2{0.01 * float64(i), 0.5 + float64(i)*1.001},
			Mass:     1 + float64(i), Friction: 0.6, Restitution: 0.2,
		})
	}
	// 几个圆
	circ, err := NewCircle(0.4)
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		w.AddBody(BodyDef{
			Type: Dynamic, Shape: circ,
			Position: Vec2{3 + float64(i)*0.85, 3 + float64(i)},
			Mass:     2, Friction: 0.4, Restitution: 0.5,
		})
	}
	return w
}

func statesEqual(a, b []BodyState) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// 同样操作序列在两个世界里跑，状态逐位相同。
func TestDeterminismTwoWorlds(t *testing.T) {
	w1 := buildScene(t)
	w2 := buildScene(t)
	stepN(t, w1, 300)
	stepN(t, w2, 300)
	if !statesEqual(w1.States(), w2.States()) {
		t.Fatal("two identical runs diverged")
	}
}

// 一次推进 120 步与分 120 次推进 1 步，结果逐位相同。
func TestDeterminismChunked(t *testing.T) {
	w1 := buildScene(t)
	w2 := buildScene(t)
	if err := w1.Step(120); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 120; i++ {
		if err := w2.Step(1); err != nil {
			t.Fatal(err)
		}
	}
	if !statesEqual(w1.States(), w2.States()) {
		t.Fatal("Step(120) != 120x Step(1)")
	}
}

// 存档-恢复后继续跑，与不中断逐位相同。
func TestDeterminismSaveRestore(t *testing.T) {
	w1 := buildScene(t)
	stepN(t, w1, 100)
	snap := w1.Snapshot()
	// 恢复后继续
	w2, err := Restore(snap)
	if err != nil {
		t.Fatal(err)
	}
	stepN(t, w1, 200)
	stepN(t, w2, 200)
	if !statesEqual(w1.States(), w2.States()) {
		t.Fatal("restored run diverged from uninterrupted run")
	}
	// 再存一次，快照也应逐位相同
	snap2a := w1.Snapshot()
	snap2b := w2.Snapshot()
	if string(snap2a) != string(snap2b) {
		t.Fatal("snapshots after continued run differ")
	}
}

// 通过文件保存/加载恢复。
func TestDeterminismFileRoundTrip(t *testing.T) {
	w1 := buildScene(t)
	stepN(t, w1, 77)
	path := t.TempDir() + "/save.json"
	if err := w1.SaveToFile(path); err != nil {
		t.Fatal(err)
	}
	w2, err := LoadFromFile(path)
	if err != nil {
		t.Fatal(err)
	}
	stepN(t, w1, 150)
	stepN(t, w2, 150)
	if !statesEqual(w1.States(), w2.States()) {
		t.Fatal("file-restored run diverged")
	}
}
