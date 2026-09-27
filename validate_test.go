package rigidbody

import (
	"errors"
	"testing"
)

func addBox(t *testing.T, w *World, p BodyParams) int {
	t.Helper()
	id, err := w.AddBody(p)
	if err != nil {
		t.Fatalf("AddBody failed: %v", err)
	}
	return id
}

func validParams() BodyParams {
	return BodyParams{
		Shape: Box(1, 1), Type: Dynamic, Mass: 1,
		Restitution: 0.2, Friction: 0.5,
	}
}

func TestPolygonRejections(t *testing.T) {
	cases := []struct {
		name string
		vert []Vec2
		want error
	}{
		{"too few", []Vec2{{0, 0}, {1, 0}}, ErrTooFewVertices},
		{"too many", []Vec2{{0, 0}, {1, 0}, {2, 0.5}, {3, 1}, {3, 2}, {2, 3}, {1, 3}, {0, 2}, {-0.5, 1}}, ErrTooManyVertices},
		{"duplicate", []Vec2{{0, 0}, {1, 0}, {1, 0}, {0, 1}}, ErrDuplicateVertex},
		{"duplicate non-adjacent", []Vec2{{0, 0}, {2, 0}, {2, 2}, {0, 0}, {0, 2}}, ErrDuplicateVertex},
		{"zero area", []Vec2{{0, 0}, {0, 0}, {0, 0}}, ErrDuplicateVertex},
		{"collinear", []Vec2{{0, 0}, {1, 0}, {2, 0}, {1, 1}}, ErrCollinear},
		{"concave", []Vec2{{0, 0}, {2, 0}, {2, 2}, {1, 0.5}, {0, 2}}, ErrConcave},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := NewWorld(Vec2{})
			p := validParams()
			p.Shape = Polygon(tc.vert)
			_, err := w.AddBody(p)
			if !errors.Is(err, tc.want) {
				t.Fatalf("got %v, want %v", err, tc.want)
			}
			if len(w.Bodies()) != 0 {
				t.Fatal("world must stay unchanged on rejection")
			}
		})
	}
}

func TestZeroAreaPolygon(t *testing.T) {
	w := NewWorld(Vec2{})
	p := validParams()
	// 蝴蝶结形：无重复顶点、无连续三点共线，但鞋带面积为 0。
	p.Shape = Polygon([]Vec2{{0, 0}, {1, 1}, {1, 0}, {0, 1}})
	_, err := w.AddBody(p)
	if !errors.Is(err, ErrZeroArea) {
		t.Fatalf("got %v, want ErrZeroArea", err)
	}
}

func TestPolygonAcceptsBothWindings(t *testing.T) {
	ccw := []Vec2{{0, 0}, {2, 0}, {2, 1}, {0, 1}}
	cw := []Vec2{{0, 0}, {0, 1}, {2, 1}, {2, 0}}
	for i, v := range [][]Vec2{ccw, cw} {
		w := NewWorld(Vec2{})
		p := validParams()
		p.Shape = Polygon(v)
		if _, err := w.AddBody(p); err != nil {
			t.Fatalf("winding %d rejected: %v", i, err)
		}
	}
}

func TestPolygonCentroidNormalization(t *testing.T) {
	w := NewWorld(Vec2{})
	p := validParams()
	p.Shape = Polygon([]Vec2{{10, 10}, {12, 10}, {12, 12}, {10, 12}})
	p.Position = Vec2{5, 5}
	id := addBox(t, w, p)
	st, _ := w.BodyState(id)
	if st.Position != (Vec2{5, 5}) {
		t.Fatalf("position should be the centroid: %v", st.Position)
	}
	// 自由落体时质心直线下落，不应因局部坐标偏移而转动。
	if err := w.Step(30); err != nil {
		t.Fatal(err)
	}
	st, _ = w.BodyState(id)
	if st.Angle != 0 || st.Velocity.X != 0 {
		t.Fatalf("unexpected rotation or horizontal drift: %+v", st)
	}
}

func TestBodyParamRejections(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*BodyParams)
		want   error
	}{
		{"mass too small", func(p *BodyParams) { p.Mass = 0.09 }, ErrInvalidMass},
		{"mass too big", func(p *BodyParams) { p.Mass = 10000.1 }, ErrInvalidMass},
		{"mass zero", func(p *BodyParams) { p.Mass = 0 }, ErrInvalidMass},
		{"mass NaN", func(p *BodyParams) { p.Mass = nan() }, ErrInvalidMass},
		{"restitution negative", func(p *BodyParams) { p.Restitution = -0.01 }, ErrInvalidRestitution},
		{"restitution too big", func(p *BodyParams) { p.Restitution = 1.01 }, ErrInvalidRestitution},
		{"friction negative", func(p *BodyParams) { p.Friction = -0.01 }, ErrInvalidFriction},
		{"friction too big", func(p *BodyParams) { p.Friction = 1.01 }, ErrInvalidFriction},
		{"bad circle", func(p *BodyParams) { p.Shape = Circle(0) }, ErrNonPositiveRadius},
		{"negative circle", func(p *BodyParams) { p.Shape = Circle(-1) }, ErrNonPositiveRadius},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := NewWorld(Vec2{})
			p := validParams()
			tc.mutate(&p)
			_, err := w.AddBody(p)
			if !errors.Is(err, tc.want) {
				t.Fatalf("got %v, want %v", err, tc.want)
			}
			if len(w.Bodies()) != 0 {
				t.Fatal("world must stay unchanged on rejection")
			}
		})
	}
}

func TestMassBoundariesAccepted(t *testing.T) {
	for _, m := range []float64{0.1, 10000} {
		w := NewWorld(Vec2{})
		p := validParams()
		p.Mass = m
		if _, err := w.AddBody(p); err != nil {
			t.Fatalf("mass %v rejected: %v", m, err)
		}
	}
}

func TestBodyLimit(t *testing.T) {
	w := NewWorld(Vec2{})
	p := validParams()
	p.Type = Static
	for i := 0; i < MaxBodies; i++ {
		if _, err := w.AddBody(p); err != nil {
			t.Fatalf("body %d rejected: %v", i, err)
		}
	}
	if _, err := w.AddBody(p); !errors.Is(err, ErrTooManyBodies) {
		t.Fatalf("got %v, want ErrTooManyBodies", err)
	}
	if len(w.Bodies()) != MaxBodies {
		t.Fatal("world must stay unchanged on rejection")
	}
}

func TestIDsAssignedInOrder(t *testing.T) {
	w := NewWorld(Vec2{})
	p := validParams()
	for i := 0; i < 10; i++ {
		id, _ := w.AddBody(p)
		if id != i {
			t.Fatalf("got id %d, want %d", id, i)
		}
	}
}

func TestStepCountLimits(t *testing.T) {
	w := NewWorld(Vec2{})
	addBox(t, w, validParams())
	for _, n := range []int{0, -1, 121, 1000} {
		if err := w.Step(n); !errors.Is(err, ErrInvalidStepCount) {
			t.Fatalf("Step(%d): got %v", n, err)
		}
	}
	// 拒绝后一步都不推进：位置不变。
	st, _ := w.BodyState(0)
	if st.Position != (Vec2{}) || st.Velocity != (Vec2{}) {
		t.Fatalf("world advanced on rejected step: %+v", st)
	}
	for _, n := range []int{1, 120} {
		if err := w.Step(n); err != nil {
			t.Fatalf("Step(%d) rejected: %v", n, err)
		}
	}
}

func TestUnknownBody(t *testing.T) {
	w := NewWorld(Vec2{})
	if _, err := w.BodyState(0); !errors.Is(err, ErrUnknownBody) {
		t.Fatalf("got %v", err)
	}
}

func nan() float64 {
	z := 0.0
	return z / z
}
