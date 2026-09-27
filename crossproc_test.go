package rigidbody

import (
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// TestHelperProcess 不是真正的测试：作为子进程入口，生成存档后退出。
func TestHelperProcess(t *testing.T) {
	if os.Getenv("GO_HELPER_PROCESS") != "1" {
		return
	}
	path := os.Getenv("GO_HELPER_SAVE")
	w := buildScene(t)
	if err := w.Step(120); err != nil {
		os.Exit(2)
	}
	if err := w.Step(120); err != nil {
		os.Exit(2)
	}
	if err := w.Step(60); err != nil {
		os.Exit(2)
	}
	if err := w.Save(path); err != nil {
		os.Exit(2)
	}
	os.Exit(0)
}

// TestDeterminismCrossProcess 存档由另一个进程生成，本进程恢复后继续跑，
// 结果与本进程不中断运行逐位相同。
func TestDeterminismCrossProcess(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cross.bin")

	// 子进程：跑 300 步并存档。
	cmd := exec.Command(os.Args[0], "-test.run", "^TestHelperProcess$")
	cmd.Env = append(os.Environ(),
		"GO_HELPER_PROCESS=1",
		"GO_HELPER_SAVE="+path,
	)
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("helper process failed: %v\n%s", err, out)
	}

	// 本进程：恢复后再跑 300 步。
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

	// 基准：本进程不中断跑 600 步。
	ref := buildScene(t)
	for i := 0; i < 5; i++ {
		if err := ref.Step(120); err != nil {
			t.Fatal(err)
		}
	}
	requireSameBits(t, stateBits(t, ref), stateBits(t, got), "cross-process restore")
}
