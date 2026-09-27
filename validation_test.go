package rigid2d

import (
	"errors"
	"testing"
)

func TestPolygonRejections(t *testing.T) {
	// 顶点太少
	if _, err := NewPolygon([]Vec2{{0, 0}, {1, 0}}); !errors.Is(err, ErrTooFewVertices) {
		t.Fatalf("too few: %v", err)
	}
	// 顶点太多（9 个）
	many := make([]Vec2, 9)
	for i := range many {
		many[i] = Vec2{float64(i % 3), float64(i / 3)}
	}
	if _, err := NewPolygon(many); !errors.Is(err, ErrTooManyVertices) {
		t.Fatalf("too many: %v", err)
	}
	// 重复顶点
	if _, err := NewPolygon([]Vec2{{0, 0}, {1, 0}, {1, 0}}); !errors.Is(err, ErrDuplicateVertex) {
		t.Fatalf("duplicate: %v", err)
	}
	// 三点共线
	if _, err := NewPolygon([]Vec2{{0, 0}, {1, 0}, {2, 0}}); !errors.Is(err, ErrZeroArea) {
		t.Fatalf("collinear degenerate: %v", err)
	}
	if _, err := NewPolygon([]Vec2{{0, 0}, {1, 0}, {2, 0}, {1, 1}}); !errors.Is(err, ErrCollinear) {
		t.Fatalf("collinear consecutive: %v", err)
	}
	// 凹多边形
	if _, err := NewPolygon([]Vec2{{0, 0}, {2, 0}, {2, 2}, {1, 0.5}, {0, 2}}); !errors.Is(err, ErrConcave) {
		t.Fatalf("concave: %v", err)
	}
	// 面积为零
	if _, err := NewPolygon([]Vec2{{1, 1}, {1, 1}, {1, 1}}); !errors.Is(err, ErrDuplicateVertex) {
		t.Fatalf("zero area dup: %v", err)
	}
	// 合法：逆时针
	if _, err := NewPolygon([]Vec2{{0, 0}, {1, 0}, {1, 1}, {0, 1}}); err != nil {
		t.Fatalf("ccw square should pass: %v", err)
	}
	// 合法：顺时针
	if _, err := NewPolygon([]Vec2{{0, 0}, {0, 1}, {1, 1}, {1, 0}}); err != nil {
		t.Fatalf("cw square should pass: %v", err)
	}
	// 三角形、八边形
	tri, err := NewPolygon([]Vec2{{0, 0}, {4, 0}, {0, 3}})
	if err != nil {
		t.Fatalf("triangle: %v", err)
	}
	// 质心应平移到原点：三角形质心 (4/3, 1)
	c := Vec2{}
	for _, v := range tri.Vertices {
		c = Add(c, v)
	}
	c = Mul(c, 1.0/3.0)
	if Len(c) > 1e-9 {
		t.Fatalf("centroid not at origin: %v", c)
	}
}

func TestBodyRejection(t *testing.T) {
	w := NewWorld()
	box := boxShape(t, 0.5, 0.5)
	cases := []BodyDef{
		{Type: Dynamic, Shape: box, Mass: 0.05, Friction: 0.5},              // 质量太小
		{Type: Dynamic, Shape: box, Mass: 10001, Friction: 0.5},             // 质量太大
		{Type: Dynamic, Shape: box, Mass: 1, Friction: -0.1},                // 摩擦越界
		{Type: Dynamic, Shape: box, Mass: 1, Friction: 1.1},                 // 摩擦越界
		{Type: Dynamic, Shape: box, Mass: 1, Friction: 0.5, Restitution: 2}, // 弹性越界
	}
	for i, def := range cases {
		before := w.BodyCount()
		if _, err := w.AddBody(def); err == nil {
			t.Fatalf("case %d should be rejected", i)
		}
		if w.BodyCount() != before {
			t.Fatalf("case %d: world mutated on rejection", i)
		}
	}
	// 合法创建
	if _, err := w.AddBody(BodyDef{Type: Dynamic, Shape: box, Mass: 1, Friction: 0.5}); err != nil {
		t.Fatalf("valid body rejected: %v", err)
	}
	// id 从 0 递增
	id2, _ := w.AddBody(BodyDef{Type: Static, Shape: box})
	if id2 != 1 {
		t.Fatalf("ids should increment from 0, got %d", id2)
	}
}

func TestMaxBodies(t *testing.T) {
	w := NewWorld()
	box := boxShape(t, 0.5, 0.5)
	for i := 0; i < MaxBodies; i++ {
		if _, err := w.AddBody(BodyDef{Type: Static, Shape: box, Position: Vec2{float64(i) * 10, 0}}); err != nil {
			t.Fatalf("body %d: %v", i, err)
		}
	}
	if _, err := w.AddBody(BodyDef{Type: Static, Shape: box}); !errors.Is(err, ErrTooManyBodies) {
		t.Fatalf("501st body should be rejected: %v", err)
	}
	if w.BodyCount() != MaxBodies {
		t.Fatalf("world mutated on rejection")
	}
}

func TestStepLimits(t *testing.T) {
	w := NewWorld()
	w.AddBody(BodyDef{Type: Dynamic, Shape: boxShape(t, 0.5, 0.5), Mass: 1, Position: Vec2{0, 10}})
	if err := w.Step(0); !errors.Is(err, ErrInvalidSteps) {
		t.Fatalf("0 steps: %v", err)
	}
	if err := w.Step(121); !errors.Is(err, ErrInvalidSteps) {
		t.Fatalf("121 steps: %v", err)
	}
	// 一步都没推进
	st, _ := w.GetBodyState(0)
	if st.Position.Y != 10 {
		t.Fatalf("rejected Step call still advanced the world")
	}
	if err := w.Step(1); err != nil {
		t.Fatalf("1 step should pass: %v", err)
	}
	if err := w.Step(120); err != nil {
		t.Fatalf("120 steps should pass: %v", err)
	}
}

// 接触读回格式：id 小的在前、法线方向、点数规则。
func TestContactFormat(t *testing.T) {
	w := NewWorld()
	// 地面（后创建，id 更大）
	boxID, _ := w.AddBody(BodyDef{Type: Dynamic, Shape: boxShape(t, 0.5, 0.5), Mass: 1, Position: Vec2{0, 0.5}})
	groundID, _ := w.AddBody(BodyDef{Type: Static, Shape: boxShape(t, 50, 0.5), Position: Vec2{0, -0.5}, Friction: 0.5})
	stepN(t, w, 120)
	cs := w.Contacts()
	if len(cs) == 0 {
		t.Fatal("expected contacts")
	}
	for _, c := range cs {
		if c.BodyA >= c.BodyB {
			t.Fatalf("BodyA should be smaller id: %+v", c)
		}
		if c.BodyA != boxID || c.BodyB != groundID {
			t.Fatalf("unexpected pair %+v", c)
		}
		// 平边贴平边：2 个接触点
		if len(c.Points) != 2 {
			t.Fatalf("box-on-box should give 2 points, got %d", len(c.Points))
		}
		// 法线从 box 指向 ground，应向下
		if c.Normal.Y >= 0 {
			t.Fatalf("normal should point from box to ground: %+v", c.Normal)
		}
	}
	// 圆贴地面：1 个接触点
	w2 := NewWorld()
	circ, _ := NewCircle(0.5)
	w2.AddBody(BodyDef{Type: Dynamic, Shape: circ, Mass: 1, Position: Vec2{0, 0.5}})
	w2.AddBody(BodyDef{Type: Static, Shape: boxShape(t, 50, 0.5), Position: Vec2{0, -0.5}, Friction: 0.5})
	stepN(t, w2, 120)
	for _, c := range w2.Contacts() {
		if len(c.Points) != 1 {
			t.Fatalf("circle contact should give 1 point, got %d", len(c.Points))
		}
	}
}
