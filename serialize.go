package rigidbody

import (
	"bytes"
	"encoding/binary"
	"errors"
	"math"
	"os"
)

// ErrCorruptSave 存档数据损坏或格式不符。
var ErrCorruptSave = errors.New("rigidbody: corrupt or incompatible save data")

// saveMagic 格式标识与版本。
var saveMagic = []byte{'R', 'B', 'P', 'H', 1}

// writer 确定性二进制写入（小端序）。
type writer struct {
	buf bytes.Buffer
}

func (w *writer) u64(v uint64) {
	var b [8]byte
	binary.LittleEndian.PutUint64(b[:], v)
	w.buf.Write(b[:])
}

func (w *writer) f64(v float64) { w.u64(math.Float64bits(v)) }

func (w *writer) vec(v Vec2) {
	w.f64(v.X)
	w.f64(v.Y)
}

// reader 确定性二进制读取。
type reader struct {
	data []byte
	off  int
}

func (r *reader) u64() (uint64, error) {
	if r.off+8 > len(r.data) {
		return 0, ErrCorruptSave
	}
	v := binary.LittleEndian.Uint64(r.data[r.off:])
	r.off += 8
	return v, nil
}

func (r *reader) f64() (float64, error) {
	v, err := r.u64()
	return math.Float64frombits(v), err
}

func (r *reader) vec() (Vec2, error) {
	x, err := r.f64()
	if err != nil {
		return Vec2{}, err
	}
	y, err := r.f64()
	if err != nil {
		return Vec2{}, err
	}
	return Vec2{x, y}, nil
}

// Save 把世界状态写入本地文件（原子写入：临时文件 + 重命名）。
func (w *World) Save(path string) error {
	wr := &writer{}
	wr.buf.Write(saveMagic)
	wr.vec(w.gravity)
	wr.u64(uint64(w.nextID))
	wr.u64(uint64(len(w.bodies)))
	for _, b := range w.bodies {
		wr.u64(uint64(b.id))
		wr.u64(uint64(b.typ))
		wr.u64(uint64(b.shape.Type))
		wr.f64(b.shape.Radius)
		wr.u64(uint64(len(b.shape.Vertices)))
		for _, v := range b.shape.Vertices {
			wr.vec(v)
		}
		wr.vec(b.pos)
		wr.f64(b.angle)
		wr.vec(b.vel)
		wr.f64(b.angularVel)
		wr.f64(b.mass)
		wr.f64(b.restitution)
		wr.f64(b.friction)
	}
	// 热启动缓存：按 (a,b) 排序写入保证确定性。
	keys := make([]pairKey, 0, len(w.cache))
	for k := range w.cache {
		keys = append(keys, k)
	}
	sortPairKeys(keys)
	wr.u64(uint64(len(keys)))
	for _, k := range keys {
		wr.u64(uint64(k.a))
		wr.u64(uint64(k.b))
		pts := w.cache[k]
		wr.u64(uint64(len(pts)))
		for _, p := range pts {
			wr.u64(uint64(uint32(int32(p.feature.refEdge))))
			wr.u64(uint64(uint32(int32(p.feature.incEdge))))
			wr.u64(uint64(uint32(int32(p.feature.flip))))
			wr.f64(p.pn)
			wr.f64(p.pt)
		}
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, wr.buf.Bytes(), 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// Load 从本地文件恢复世界。
func Load(path string) (*World, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	r := &reader{data: data}
	if len(data) < len(saveMagic) || !bytes.Equal(data[:len(saveMagic)], saveMagic) {
		return nil, ErrCorruptSave
	}
	r.off = len(saveMagic)

	gravity, err := r.vec()
	if err != nil {
		return nil, err
	}
	nextID, err := r.u64()
	if err != nil {
		return nil, err
	}
	numBodies, err := r.u64()
	if err != nil {
		return nil, err
	}
	if numBodies > MaxBodies {
		return nil, ErrCorruptSave
	}
	w := &World{gravity: gravity, nextID: int(nextID)}
	for i := uint64(0); i < numBodies; i++ {
		b, err := readBody(r)
		if err != nil {
			return nil, err
		}
		w.bodies = append(w.bodies, b)
	}
	// 缓存。
	numPairs, err := r.u64()
	if err != nil {
		return nil, err
	}
	if numPairs > 1<<20 {
		return nil, ErrCorruptSave
	}
	w.cache = make(map[pairKey][]cachedPoint, numPairs)
	for i := uint64(0); i < numPairs; i++ {
		a, err := r.u64()
		if err != nil {
			return nil, err
		}
		bb, err := r.u64()
		if err != nil {
			return nil, err
		}
		np, err := r.u64()
		if err != nil {
			return nil, err
		}
		if np > 2 {
			return nil, ErrCorruptSave
		}
		pts := make([]cachedPoint, 0, np)
		for j := uint64(0); j < np; j++ {
			re, err := r.u64()
			if err != nil {
				return nil, err
			}
			ie, err := r.u64()
			if err != nil {
				return nil, err
			}
			fl, err := r.u64()
			if err != nil {
				return nil, err
			}
			pn, err := r.f64()
			if err != nil {
				return nil, err
			}
			pt, err := r.f64()
			if err != nil {
				return nil, err
			}
			pts = append(pts, cachedPoint{
				feature: featureID{
					refEdge: int(int32(uint32(re))),
					incEdge: int(int32(uint32(ie))),
					flip:    int(int32(uint32(fl))),
				},
				pn: pn,
				pt: pt,
			})
		}
		w.cache[pairKey{int(a), int(bb)}] = pts
	}
	if r.off != len(data) {
		return nil, ErrCorruptSave
	}
	return w, nil
}

func readBody(r *reader) (*body, error) {
	id, err := r.u64()
	if err != nil {
		return nil, err
	}
	typ, err := r.u64()
	if err != nil {
		return nil, err
	}
	if typ > uint64(Dynamic) {
		return nil, ErrCorruptSave
	}
	shapeType, err := r.u64()
	if err != nil {
		return nil, err
	}
	radius, err := r.f64()
	if err != nil {
		return nil, err
	}
	numVerts, err := r.u64()
	if err != nil {
		return nil, err
	}
	if numVerts > 8 {
		return nil, ErrCorruptSave
	}
	verts := make([]Vec2, 0, numVerts)
	for i := uint64(0); i < numVerts; i++ {
		v, err := r.vec()
		if err != nil {
			return nil, err
		}
		verts = append(verts, v)
	}
	pos, err := r.vec()
	if err != nil {
		return nil, err
	}
	angle, err := r.f64()
	if err != nil {
		return nil, err
	}
	vel, err := r.vec()
	if err != nil {
		return nil, err
	}
	angularVel, err := r.f64()
	if err != nil {
		return nil, err
	}
	mass, err := r.f64()
	if err != nil {
		return nil, err
	}
	restitution, err := r.f64()
	if err != nil {
		return nil, err
	}
	friction, err := r.f64()
	if err != nil {
		return nil, err
	}
	if ShapeType(shapeType) == ShapeCircle && numVerts != 0 {
		return nil, ErrCorruptSave
	}
	if ShapeType(shapeType) == ShapePolygon && numVerts < 3 {
		return nil, ErrCorruptSave
	}
	b := &body{
		id:          int(id),
		typ:         BodyType(typ),
		shape:       Shape{Type: ShapeType(shapeType), Radius: radius, Vertices: verts},
		pos:         pos,
		angle:       angle,
		vel:         vel,
		angularVel:  angularVel,
		restitution: restitution,
		friction:    friction,
	}
	if b.typ == Dynamic {
		// 与 AddBody 相同的推导路径，保证逐位一致。
		area, unitInertia := b.shape.massData()
		density := mass / area
		b.mass = mass
		b.invMass = 1 / mass
		b.inertia = unitInertia * density
		b.invInertia = 1 / b.inertia
	}
	return b, nil
}

func sortPairKeys(keys []pairKey) {
	// 插入排序，键数量小且要求确定性。
	for i := 1; i < len(keys); i++ {
		for j := i; j > 0; j-- {
			a, b := keys[j-1], keys[j]
			if a.a < b.a || (a.a == b.a && a.b < b.b) {
				break
			}
			keys[j-1], keys[j] = keys[j], keys[j-1]
		}
	}
}
