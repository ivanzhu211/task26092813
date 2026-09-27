package rigidbody

import (
	"math"
	"testing"
)

const g = 9.81

// slopeWorld 构造 30° 静态斜坡世界，返回世界与沿坡向下单位向量。
func slopeWorld(t *testing.T, friction float64) (*World, Vec2) {
	t.Helper()
	w := NewWorld(Vec2{})
	angle := math.Pi / 6 // 30°
	addBox(t, w, BodyParams{
		Shape: Box(60, 1), Type: Static, Position: Vec2{0, 0},
		Angle: angle, Friction: friction,
	})
	// 坡面逆时针抬升，沿坡向下为局部 x 轴反方向。
	down := Vec2{-math.Cos(angle), -math.Sin(angle)}
	return w, down
}

// TestInclineHighFriction 摩擦 0.8 时箱子在 30° 斜坡上 5 秒滑动不超过 0.05m。
func TestInclineHighFriction(t *testing.T) {
	w, down := slopeWorld(t, 0.8)
	angle := math.Pi / 6
	normal := Vec2{-math.Sin(angle), math.Cos(angle)} // 坡面外法线
	start := normal.Mul(0.5 + 0.51)                   // 半厚0.5 + 箱子半边0.5 + 0.01 间隙
	id := addBox(t, w, BodyParams{
		Shape: Box(1, 1), Type: Dynamic, Mass: 1,
		Position: start, Angle: angle, Friction: 0.8,
	})
	if err := w.Step(60); err != nil { // 先落稳
		t.Fatal(err)
	}
	st0, _ := w.BodyState(id)
	s0 := st0.Position.Dot(down)
	for i := 0; i < 4; i++ {
		if err := w.Step(60); err != nil {
			t.Fatal(err)
		}
	}
	st1, _ := w.BodyState(id)
	s1 := st1.Position.Dot(down)
	if d := s1 - s0; d > 0.05 {
		t.Fatalf("slid %.4f m along slope in ~4s > 0.05 m", d)
	}
}

// TestInclineLowFriction 摩擦 0.2 时沿坡加速度 ≈ g(sin30° − 0.2·cos30°)。
func TestInclineLowFriction(t *testing.T) {
	w, down := slopeWorld(t, 0.2)
	angle := math.Pi / 6
	normal := Vec2{-math.Sin(angle), math.Cos(angle)}
	start := normal.Mul(0.5 + 0.51)
	id := addBox(t, w, BodyParams{
		Shape: Box(1, 1), Type: Dynamic, Mass: 1,
		Position: start, Angle: angle, Friction: 0.2,
	})
	if err := w.Step(60); err != nil { // 落稳，进入滑动状态
		t.Fatal(err)
	}
	st1, _ := w.BodyState(id)
	if err := w.Step(60); err != nil {
		t.Fatal(err)
	}
	st2, _ := w.BodyState(id)
	a := (st2.Velocity.Dot(down) - st1.Velocity.Dot(down)) / 1.0
	want := g * (math.Sin(angle) - 0.2*math.Cos(angle))
	if math.Abs(a-want)/want > 0.05 {
		t.Fatalf("acceleration %.4f, want %.4f ±5%%", a, want)
	}
}

// TestRollingDisk 实心圆盘在 30° 斜坡纯滚动，质心加速度 ≈ (2/3)g·sin30°。
func TestRollingDisk(t *testing.T) {
	w, down := slopeWorld(t, 0.8)
	angle := math.Pi / 6
	normal := Vec2{-math.Sin(angle), math.Cos(angle)}
	const r = 0.5
	start := normal.Mul(0.5 + r + 0.01)
	id := addBox(t, w, BodyParams{
		Shape: Circle(r), Type: Dynamic, Mass: 1,
		Position: start, Friction: 0.8,
	})
	if err := w.Step(60); err != nil { // 落稳并进入纯滚动
		t.Fatal(err)
	}
	st1, _ := w.BodyState(id)
	if err := w.Step(60); err != nil {
		t.Fatal(err)
	}
	st2, _ := w.BodyState(id)
	a := (st2.Velocity.Dot(down) - st1.Velocity.Dot(down)) / 1.0
	want := (2.0 / 3.0) * g * math.Sin(angle)
	if math.Abs(a-want)/want > 0.05 {
		t.Fatalf("acceleration %.4f, want %.4f ±5%%", a, want)
	}
}

// TestBounce 弹性 0.5 的圆从底部离地 5m 落下，反弹最高点 ≈ 1.25m ±10%。
func TestBounce(t *testing.T) {
	w := NewWorld(Vec2{})
	addBox(t, w, BodyParams{
		Shape: Box(100, 1), Type: Static, Position: Vec2{0, -0.5},
		Restitution: 0, Friction: 0.5,
	})
	const r = 0.5
	const dropBottom = 5.0
	id := addBox(t, w, BodyParams{
		Shape: Circle(r), Type: Dynamic, Mass: 1,
		Position: Vec2{0, dropBottom + r}, Restitution: 0.5, Friction: 0.5,
	})
	// 跑到第一次反弹后的最高点。
	maxBottom := 0.0
	prevVy := 0.0
	bounced := false
	for i := 0; i < 600; i++ {
		if err := w.Step(1); err != nil {
			t.Fatal(err)
		}
		st, _ := w.BodyState(id)
		bottom := st.Position.Y - r
		if st.Velocity.Y < 0 && prevVy >= 0 && bounced {
			// 到达最高点。
			break
		}
		if st.Velocity.Y > 0 {
			bounced = true
		}
		if bounced && bottom > maxBottom {
			maxBottom = bottom
		}
		prevVy = st.Velocity.Y
	}
	want := 1.25
	if math.Abs(maxBottom-want)/want > 0.10 {
		t.Fatalf("rebound height %.4f m, want %.4f ±10%%", maxBottom, want)
	}
}
