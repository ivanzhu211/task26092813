package rigid2d

import (
	"encoding/json"
	"math"
	"os"
	"sort"
)

// 序列化格式：JSON。float64 经 encoding/json 最短表示可逐位往返。

type worldSnapshot struct {
	Gravity  Vec2              `json:"gravity"`
	Dt       float64           `json:"dt"`
	NextID   int               `json:"nextId"`
	Bodies   []bodySnapshot    `json:"bodies"`
	Arbiters []arbiterSnapshot `json:"arbiters"`
}

type bodySnapshot struct {
	ID          int      `json:"id"`
	Type        BodyType `json:"type"`
	Shape       Shape    `json:"shape"`
	Position    Vec2     `json:"position"`
	Angle       float64  `json:"angle"`
	LinVel      Vec2     `json:"linVel"`
	AngVel      float64  `json:"angVel"`
	Force       Vec2     `json:"force"`
	Torque      float64  `json:"torque"`
	Mass        float64  `json:"mass"`
	InvMass     float64  `json:"invMass"`
	Inertia     float64  `json:"inertia"`
	InvInertia  float64  `json:"invInertia"`
	Friction    float64  `json:"friction"`
	Restitution float64  `json:"restitution"`
}

type arbiterSnapshot struct {
	A        int               `json:"a"`
	B        int               `json:"b"`
	Contacts []contactSnapshot `json:"contacts"`
}

type contactSnapshot struct {
	Feature uint64  `json:"feature"`
	Pn      float64 `json:"pn"`
	Pt      float64 `json:"pt"`
}

// Snapshot 导出世界完整状态（含求解器热启动数据）。
func (w *World) Snapshot() []byte {
	snap := worldSnapshot{
		Gravity: w.Gravity,
		Dt:      w.Dt,
		NextID:  w.nextID,
	}
	for _, b := range w.bodies {
		snap.Bodies = append(snap.Bodies, bodySnapshot{
			ID: b.ID, Type: b.Type, Shape: b.Shape,
			Position: b.Position, Angle: b.Angle,
			LinVel: b.LinVel, AngVel: b.AngVel,
			Force: b.Force, Torque: b.Torque,
			Mass: b.Mass, InvMass: b.InvMass,
			Inertia: b.Inertia, InvInertia: b.InvInertia,
			Friction: b.Friction, Restitution: b.Restitution,
		})
	}
	for _, key := range sortedKeysOf(w.arbiters) {
		arb := w.arbiters[key]
		as := arbiterSnapshot{A: key.a, B: key.b}
		for _, c := range arb.contacts {
			as.Contacts = append(as.Contacts, contactSnapshot{
				Feature: c.feature, Pn: c.pn, Pt: c.pt,
			})
		}
		snap.Arbiters = append(snap.Arbiters, as)
	}
	data, err := json.Marshal(snap)
	if err != nil {
		panic(err) // 内部状态必然可序列化
	}
	return data
}

// sortedKeysOf 返回按 (a,b) 升序的 arbiter 键，保证序列化输出确定。
func sortedKeysOf(m map[arbiterKey]*arbiter) []arbiterKey {
	keys := make([]arbiterKey, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].a != keys[j].a {
			return keys[i].a < keys[j].a
		}
		return keys[i].b < keys[j].b
	})
	return keys
}

// Restore 从快照数据重建世界。
func Restore(data []byte) (*World, error) {
	var snap worldSnapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return nil, err
	}
	w := NewWorld()
	w.Gravity = snap.Gravity
	w.Dt = snap.Dt
	w.nextID = snap.NextID
	for _, bs := range snap.Bodies {
		b := &Body{
			ID: bs.ID, Type: bs.Type, Shape: bs.Shape,
			Position: bs.Position, Angle: bs.Angle,
			LinVel: bs.LinVel, AngVel: bs.AngVel,
			Force: bs.Force, Torque: bs.Torque,
			Mass: bs.Mass, InvMass: bs.InvMass,
			Inertia: bs.Inertia, InvInertia: bs.InvInertia,
			Friction: bs.Friction, Restitution: bs.Restitution,
		}
		w.bodies = append(w.bodies, b)
	}
	for _, as := range snap.Arbiters {
		a := w.bodyByID(as.A)
		b := w.bodyByID(as.B)
		if a == nil || b == nil {
			continue
		}
		arb := &arbiter{
			a: a, b: b,
			friction:    sqrtFriction(a, b),
			restitution: maxRestitution(a, b),
		}
		for _, cs := range as.Contacts {
			arb.contacts = append(arb.contacts, contact{
				feature: cs.Feature, pn: cs.Pn, pt: cs.Pt,
			})
		}
		w.arbiters[arbiterKey{as.A, as.B}] = arb
	}
	return w, nil
}

func sqrtFriction(a, b *Body) float64 {
	return math.Sqrt(a.Friction * b.Friction)
}

func maxRestitution(a, b *Body) float64 {
	if a.Restitution > b.Restitution {
		return a.Restitution
	}
	return b.Restitution
}

// SaveToFile 把世界状态写入本地文件。
func (w *World) SaveToFile(path string) error {
	return os.WriteFile(path, w.Snapshot(), 0o644)
}

// LoadFromFile 从本地文件恢复世界。
func LoadFromFile(path string) (*World, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	return Restore(data)
}
