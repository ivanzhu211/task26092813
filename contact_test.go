package rigidbody

import (
	"math"
	"testing"
)

// TestContactFlatFace 平边贴平边给 2 个接触点，法线从 id 小者指向大者。
func TestContactFlatFace(t *testing.T) {
	w := NewWorld(Vec2{})
	addBox(t, w, BodyParams{Shape: Box(10, 1), Type: Static, Position: Vec2{0, -0.5}, Friction: 0.5})
	addBox(t, w, BodyParams{Shape: Box(1, 1), Type: Dynamic, Mass: 1, Position: Vec2{0, 0.5}, Friction: 0.5})
	if err := w.Step(10); err != nil {
		t.Fatal(err)
	}
	cs := w.Contacts()
	if len(cs) != 1 {
		t.Fatalf("expected 1 contact, got %d", len(cs))
	}
	c := cs[0]
	if c.BodyA != 0 || c.BodyB != 1 {
		t.Fatalf("ids not ordered: %d, %d", c.BodyA, c.BodyB)
	}
	if len(c.Points) != 2 {
		t.Fatalf("flat face contact should have 2 points, got %d", len(c.Points))
	}
	if math.Abs(c.Normal.X) > 1e-9 || c.Normal.Y < 0.999 {
		t.Fatalf("normal should point up (ground -> box): %v", c.Normal)
	}
	if c.Depth < 0 || c.Depth > 0.02 {
		t.Fatalf("resting depth %.5f out of [0, 0.02]", c.Depth)
	}
}

// TestContactCircle 涉及圆的接触给 1 个接触点。
func TestContactCircle(t *testing.T) {
	w := NewWorld(Vec2{})
	addBox(t, w, BodyParams{Shape: Box(10, 1), Type: Static, Position: Vec2{0, -0.5}, Friction: 0.5})
	addBox(t, w, BodyParams{Shape: Circle(0.5), Type: Dynamic, Mass: 1, Position: Vec2{0, 0.5}, Friction: 0.5})
	if err := w.Step(10); err != nil {
		t.Fatal(err)
	}
	cs := w.Contacts()
	if len(cs) != 1 {
		t.Fatalf("expected 1 contact, got %d", len(cs))
	}
	if len(cs[0].Points) != 1 {
		t.Fatalf("circle contact should have 1 point, got %d", len(cs[0].Points))
	}
}

// TestContactNormalDirection 法线始终从 id 小的刚体指向 id 大的。
func TestContactNormalDirection(t *testing.T) {
	w := NewWorld(Vec2{})
	// 先创建圆（id 小），后创建地面（id 大）：法线应向下。
	addBox(t, w, BodyParams{Shape: Circle(0.5), Type: Dynamic, Mass: 1, Position: Vec2{0, 0.5}, Friction: 0.5})
	addBox(t, w, BodyParams{Shape: Box(10, 1), Type: Static, Position: Vec2{0, -0.5}, Friction: 0.5})
	if err := w.Step(10); err != nil {
		t.Fatal(err)
	}
	cs := w.Contacts()
	if len(cs) != 1 {
		t.Fatalf("expected 1 contact, got %d", len(cs))
	}
	c := cs[0]
	if c.BodyA != 0 || c.BodyB != 1 {
		t.Fatalf("ids not ordered: %d, %d", c.BodyA, c.BodyB)
	}
	if c.Normal.Y > -0.999 {
		t.Fatalf("normal should point down (circle -> ground): %v", c.Normal)
	}
	if l := c.Normal.Len(); math.Abs(l-1) > 1e-9 {
		t.Fatalf("normal not unit length: %v", l)
	}
}

// TestStaticStaticNoContact 静态刚体之间不产生接触。
func TestStaticStaticNoContact(t *testing.T) {
	w := NewWorld(Vec2{})
	addBox(t, w, BodyParams{Shape: Box(1, 1), Type: Static, Position: Vec2{0, 0}})
	addBox(t, w, BodyParams{Shape: Box(1, 1), Type: Static, Position: Vec2{0, 0.5}})
	if err := w.Step(1); err != nil {
		t.Fatal(err)
	}
	if n := len(w.Contacts()); n != 0 {
		t.Fatalf("static-static should not collide, got %d contacts", n)
	}
}
