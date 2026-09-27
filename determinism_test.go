package rigidbody

import (
	"math"
	"path/filepath"
	"testing"
)

// buildScene 构造一个包含箱子堆、圆、斜坡的混合场景。
func buildScene(t *testing.T) *World {
	t.Helper()
	w := NewWorld(Vec2{})
	// 地面。
	addBox(t, w, BodyParams{
		Shape: Box(40, 1), Type: Static, Position: Vec2{0, -0.5},
		Friction: 0.6, Restitution: 0.1,
	})
	// 一摞箱子。
	for i := 0; i < 5; i++ {
		addBox(t, w, BodyParams{
			Shape: Box(1, 1), Type: Dynamic, Mass: 1 + float64(i),
			Position: Vec2{-5, 0.5 + float64(i)*1.001},
			Friction: 0.5, Restitution: 0.1,
		})
	}
	// 几个圆。
	for i := 0; i < 3; i++ {
		addBox(t, w, BodyParams{
			Shape: Circle(0.5), Type: Dynamic, Mass: 2,
			Position: Vec2{3 + float64(i)*1.2, 0.5 + float64(i)*0.3},
			Friction: 0.4, Restitution: 0.6,
		})
	}
	// 斜置的静态箱子。
	addBox(t, w, BodyParams{
		Shape: Box(6, 0.5), Type: Static, Position: Vec2{8, 2},
		Angle: 0.3, Friction: 0.3,
	})
	// 掉落的箱子。
	addBox(t, w, BodyParams{
		Shape: Box(0.8, 0.8), Type: Dynamic, Mass: 3,
		Position: Vec2{8, 5}, Velocity: Vec2{1, 0}, AngularVel: 0.5,
		Friction: 0.5, Restitution: 0.3,
	})
	return w
}

// stateBits 提取全部刚体状态的原始位表示。
func stateBits(t *testing.T, w *World) []uint64 {
	t.Helper()
	var bits []uint64
	for _, id := range w.Bodies() {
		st, err := w.BodyState(id)
		if err != nil {
			t.Fatal(err)
		}
		bits = append(bits,
			math.Float64bits(st.Position.X), math.Float64bits(st.Position.Y),
			math.Float64bits(st.Angle),
			math.Float64bits(st.Velocity.X), math.Float64bits(st.Velocity.Y),
			math.Float64bits(st.AngularVel),
		)
	}
	return bits
}

func requireSameBits(t *testing.T, a, b []uint64, what string) {
	t.Helper()
	if len(a) != len(b) {
		t.Fatalf("%s: length %d vs %d", what, len(a), len(b))
	}
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("%s: differ at word %d: %x vs %x", what, i, a[i], b[i])
		}
	}
}

func TestDeterminismTwoWorlds(t *testing.T) {
	w1 := buildScene(t)
	w2 := buildScene(t)
	for step := 0; step < 10; step++ {
		if err := w1.Step(30); err != nil {
			t.Fatal(err)
		}
		if err := w2.Step(30); err != nil {
			t.Fatal(err)
		}
	}
	requireSameBits(t, stateBits(t, w1), stateBits(t, w2), "two worlds")
}

func TestDeterminismStepSplitting(t *testing.T) {
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
	requireSameBits(t, stateBits(t, w1), stateBits(t, w2), "120x1 vs 1x120")
}

func TestDeterminismSaveRestore(t *testing.T) {
	path := filepath.Join(t.TempDir(), "save.bin")

	// 不中断的基准运行：300 + 300 步。
	ref := buildScene(t)
	if err := ref.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := ref.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := ref.Step(60); err != nil {
		t.Fatal(err)
	}
	if err := ref.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := ref.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := ref.Step(60); err != nil {
		t.Fatal(err)
	}

	// 中断运行：300 步后存档，恢复后继续 300 步。
	w := buildScene(t)
	if err := w.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := w.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := w.Step(60); err != nil {
		t.Fatal(err)
	}
	if err := w.Save(path); err != nil {
		t.Fatal(err)
	}
	w = nil

	got, err := Load(path)
	if err != nil {
		t.Fatal(err)
	}
	if err := got.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := got.Step(120); err != nil {
		t.Fatal(err)
	}
	if err := got.Step(60); err != nil {
		t.Fatal(err)
	}
	requireSameBits(t, stateBits(t, ref), stateBits(t, got), "save/restore")
}

func TestSaveRestoreStateIdentical(t *testing.T) {
	path := filepath.Join(t.TempDir(), "save.bin")
	w := buildScene(t)
	if err := w.Step(77); err != nil {
		t.Fatal(err)
	}
	before := stateBits(t, w)
	if err := w.Save(path); err != nil {
		t.Fatal(err)
	}
	got, err := Load(path)
	if err != nil {
		t.Fatal(err)
	}
	requireSameBits(t, before, stateBits(t, got), "save/load roundtrip")
}

func TestLoadCorrupt(t *testing.T) {
	dir := t.TempDir()
	// 不存在的文件。
	if _, err := Load(filepath.Join(dir, "nope")); err == nil {
		t.Fatal("expected error for missing file")
	}
	// 垃圾内容。
	p := filepath.Join(dir, "bad.bin")
	if err := writeFile(p, []byte("garbage")); err != nil {
		t.Fatal(err)
	}
	if _, err := Load(p); err == nil {
		t.Fatal("expected error for garbage file")
	}
	// 截断的合法存档。
	w := buildScene(t)
	good := filepath.Join(dir, "good.bin")
	if err := w.Save(good); err != nil {
		t.Fatal(err)
	}
	data, err := readFile(good)
	if err != nil {
		t.Fatal(err)
	}
	trunc := filepath.Join(dir, "trunc.bin")
	if err := writeFile(trunc, data[:len(data)/2]); err != nil {
		t.Fatal(err)
	}
	if _, err := Load(trunc); err == nil {
		t.Fatal("expected error for truncated file")
	}
}
