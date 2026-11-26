"""통로 가두기 실험용 keepout 마스크: 경기장의 마스크에 막을 네모를 더 칠해 새 마스크를 쓴다.

전역 계획이 다른 통로로 돌아가지 못하게 막아, 장애물을 아는 경로가 한 통로 안에 남는지 보는 실험에 쓴다.
정적 마스크라 한 경로(벤치의 O_IN -> M_E)에만 맞는다.

사용: python aisle_lock_mask.py <경기장 마스크 yaml> <출력 폴더> [x0,y0,x1,y1 ...]
네모를 주지 않으면 v7의 남쪽 통로를 막는다. 출력은 <출력 폴더>/aisle_lock.yaml과 aisle_lock.pgm이다.
"""
import pathlib
import re
import sys

import numpy as np
import yaml

FREE, BLOCKED = 254, 0
# v7에서 남쪽 진열대 줄(x 0.70~1.58, y 0.36~0.41)의 남쪽: 가운데 통로를 돌아가는 유일한 길이다
V7_SOUTH_AISLE = (0.70, 0.0, 1.58, 0.36)


# ---- 마스크 그림 ----
def load_pgm(path) -> np.ndarray:
    """P5(8비트) pgm을 (행, 열) 배열로 읽는다. 행 0이 맨 위다."""
    raw = pathlib.Path(path).read_bytes()
    # 최댓값 뒤의 공백 한 바이트까지가 헤더다. 그 뒤는 공백 바이트여도 픽셀이다
    head = re.match(rb"P5\s+(\d+)\s+(\d+)\s+255\s", raw)
    if not head:
        raise ValueError(f"{path}: not an 8-bit P5 pgm")
    w, h = int(head.group(1)), int(head.group(2))
    data = raw[head.end():]
    if len(data) != w * h:
        raise ValueError(f"{path}: {len(data)} pixel bytes, expected {w * h}")
    return np.frombuffer(data, dtype=np.uint8).reshape(h, w).copy()


def save_pgm(path, grid: np.ndarray) -> None:
    h, w = grid.shape
    pathlib.Path(path).write_bytes(b"P5\n%d %d\n255\n" % (w, h) + grid.astype(np.uint8).tobytes())


def paint(grid: np.ndarray, origin, resolution: float, rect) -> np.ndarray:
    """칸의 중심이 네모 (x0, y0, x1, y1) 안에 든 칸을 막은 새 배열. origin은 맨 아래 왼쪽 칸의 모서리다."""
    x0, y0, x1, y1 = rect
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f"need x0 < x1 and y0 < y1, got {rect}")
    h, w = grid.shape
    xs = origin[0] + (np.arange(w) + 0.5) * resolution
    ys = origin[1] + (h - 1 - np.arange(h) + 0.5) * resolution
    inside = ((ys >= y0) & (ys < y1))[:, None] & ((xs >= x0) & (xs < x1))[None, :]
    if not inside.any():
        raise ValueError(f"rectangle {rect} covers no cell of the mask")
    out = grid.copy()
    out[inside] = BLOCKED
    return out


# ---- 파일 ----
def write(base_yaml, out_dir, rects) -> pathlib.Path:
    """경기장 마스크에 네모들을 칠한 마스크를 out_dir에 쓰고 그 yaml 경로를 돌려준다."""
    base_yaml, out_dir = pathlib.Path(base_yaml), pathlib.Path(out_dir)
    meta = yaml.safe_load(base_yaml.read_text(encoding="utf-8"))
    grid = load_pgm(base_yaml.parent / meta["image"])
    for rect in rects:
        grid = paint(grid, meta["origin"][:2], float(meta["resolution"]), rect)
    out_dir.mkdir(parents=True, exist_ok=True)
    save_pgm(out_dir / "aisle_lock.pgm", grid)
    lines = [f"# 통로 가두기 실험용. {base_yaml.name}에 막은 네모: {[list(r) for r in rects]}",
             "image: aisle_lock.pgm"]
    lines += [f"{k}: {meta[k]}" for k in ("mode", "resolution", "origin", "negate", "occupied_thresh",
                                          "free_thresh")]
    out = out_dir / "aisle_lock.yaml"
    out.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return out


if __name__ == "__main__":
    rects = [tuple(float(v) for v in a.split(",")) for a in sys.argv[3:]] or [V7_SOUTH_AISLE]
    print("wrote", write(sys.argv[1], sys.argv[2], rects), "blocked", rects)
