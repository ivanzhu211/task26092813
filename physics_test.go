package rigid2d

import (
	"math"
	"testing"
)

const slopeAngle = math.Pi / 6 // 30°

// 在 30° 斜坡上放一个刚体，返回世界与刚体 id。
// 斜坡为绕原点旋转 30° 的大箱子，刚体放在坡面中上段。
func slopeScene(t *testing.T, shape Shape, mass, friction float64) (*World, int) {
	t.Helper()
	w := NewWorld()
	rampHalf := Vec2{50, 0.5}
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, rampHalf.X, rampHalf.Y),
		Position: Vec2{0, 0}, Angle: slopeAngle, Friction: friction,
	})
	// 坡面法线与上坡方向
	n := Vec2{-math.Sin(slopeAngle), math.Cos(slopeAngle)}
	up := Vec2{math.Cos(slopeAngle), math.Sin(slopeAngle)}
	// 坡面上某点：沿上坡方向偏移 10m，再沿法线抬高
	surfPoint := Add(Mul(up, 10), Mul(n, rampHalf.Y))
	var lift float64
	if shape.Type == ShapeCircle {
		lift = shape.Radius
	} else {
		// 箱子底面与坡面平行，质心沿法线距坡面 0.5m，留 1mm 间隙
		lift = 0.5 + 0.001
	}
	pos := Add(surfPoint, Mul(n, lift))
	id, err := w.AddBody(BodyDef{
		Type: Dynamic, Shape: shape, Position: pos,
		Angle: slopeAngle, Mass: mass, Friction: friction,
	})
	if err != nil {
		t.Fatal(err)
	}
	return w, id
}

func downSlope() Vec2 {
	return Vec2{-math.Cos(slopeAngle), -math.Sin(slopeAngle)}
}

// μ=0.8：静摩擦保持不动，5 秒滑动不超过 0.05m。
func TestSlopeHighFrictionSticks(t *testing.T) {
	w, id := slopeScene(t, boxShape(t, 0.5, 0.5), 1, 0.8)
	start, _ := w.GetBodyState(id)
	stepN(t, w, 300)
	end, _ := w.GetBodyState(id)
	slide := Dot(Sub(end.Position, start.Position), downSlope())
	t.Logf("slide distance: %f", slide)
	if math.Abs(slide) > 0.05 {
		t.Fatalf("box slid %f m with μ=0.8", slide)
	}
}

// μ=0.2：沿坡加速度 ≈ g(sin30° − 0.2cos30°)，误差 5% 以内。
func TestSlopeLowFrictionSlides(t *testing.T) {
	w, id := slopeScene(t, boxShape(t, 0.5, 0.5), 1, 0.2)
	// 先滑 0.5 秒进入稳定滑动，再测 2 秒内的速度增量
	stepN(t, w, 30)
	s0, _ := w.GetBodyState(id)
	stepN(t, w, 120)
	s1, _ := w.GetBodyState(id)
	dt := 2.0
	a := (Dot(s1.LinVel, downSlope()) - Dot(s0.LinVel, downSlope())) / dt
	want := 9.81 * (math.Sin(slopeAngle) - 0.2*math.Cos(slopeAngle))
	t.Logf("a=%f want=%f", a, want)
	if math.Abs(a-want)/want > 0.05 {
		t.Fatalf("acceleration %f, want %f (±5%%)", a, want)
	}
}

// 实心圆盘纯滚动：质心加速度 ≈ (2/3)g·sin30°，误差 5% 以内。
func TestDiskRollsDownSlope(t *testing.T) {
	disk, err := NewCircle(0.5)
	if err != nil {
		t.Fatal(err)
	}
	w, id := slopeScene(t, disk, 1, 0.8)
	// 进入稳定滚动后测 2 秒速度增量
	stepN(t, w, 30)
	s0, _ := w.GetBodyState(id)
	stepN(t, w, 120)
	s1, _ := w.GetBodyState(id)
	a := (Dot(s1.LinVel, downSlope()) - Dot(s0.LinVel, downSlope())) / 2.0
	want := (2.0 / 3.0) * 9.81 * math.Sin(slopeAngle)
	t.Logf("a=%f want=%f angVel=%f", a, want, s1.AngVel)
	if math.Abs(a-want)/want > 0.05 {
		t.Fatalf("rolling acceleration %f, want %f (±5%%)", a, want)
	}
}

// 弹性 0.5 的圆从底部离地 5m 落到弹性 0 的地面，反弹最高点 1.25m ±10%。
func TestCircleBounce(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{
		Type: Static, Shape: boxShape(t, 50, 0.5),
		Position: Vec2{0, -0.5}, Friction: 0.5, Restitution: 0,
	})
	circ, _ := NewCircle(0.5)
	id, _ := w.AddBody(BodyDef{
		Type: Dynamic, Shape: circ, Position: Vec2{0, 5.5},
		Mass: 1, Friction: 0.5, Restitution: 0.5,
	})
	// 逐步跟踪：接触后底部离地高度的最大值
	maxH := 0.0
	bounced := false
	for i := 0; i < 300; i++ {
		if err := w.Step(1); err != nil {
			t.Fatal(err)
		}
		st, _ := w.GetBodyState(id)
		if !bounced && st.LinVel.Y > 0 {
			bounced = true
		}
		if bounced {
			h := st.Position.Y - 0.5
			if h > maxH {
				maxH = h
			}
		}
	}
	t.Logf("max rebound height: %f", maxH)
	if math.Abs(maxH-1.25)/1.25 > 0.10 {
		t.Fatalf("rebound height %f, want 1.25 ±10%%", maxH)
	}
}
