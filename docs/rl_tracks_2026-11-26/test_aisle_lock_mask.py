import aisle_lock_mask as alm
import numpy as np
import pytest


def test_paint_blocks_the_cells_whose_centre_is_inside_the_rectangle():
    # 4 x 3 칸, 1 cm, 원점 (0, 0). 행 0이 맨 위(y가 가장 큰 줄)다
    grid = np.full((3, 4), alm.FREE, dtype=np.uint8)
    out = alm.paint(grid, (0.0, 0.0), 0.01, (0.01, 0.0, 0.03, 0.02))
    assert out.tolist() == [[alm.FREE] * 4,
                            [alm.FREE, alm.BLOCKED, alm.BLOCKED, alm.FREE],
                            [alm.FREE, alm.BLOCKED, alm.BLOCKED, alm.FREE]]
    assert grid.min() == alm.FREE, "the input grid is left alone"


def test_paint_keeps_cells_that_were_already_blocked():
    grid = np.full((2, 2), alm.FREE, dtype=np.uint8)
    grid[0, 0] = alm.BLOCKED
    out = alm.paint(grid, (0.0, 0.0), 0.01, (0.01, 0.0, 0.02, 0.01))
    assert out.tolist() == [[alm.BLOCKED, alm.FREE], [alm.FREE, alm.BLOCKED]]


def test_paint_rejects_a_rectangle_that_blocks_nothing():
    grid = np.full((2, 2), alm.FREE, dtype=np.uint8)
    with pytest.raises(ValueError, match="no cell"):
        alm.paint(grid, (0.0, 0.0), 0.01, (0.5, 0.5, 0.6, 0.6))
    with pytest.raises(ValueError, match="x0 < x1"):
        alm.paint(grid, (0.0, 0.0), 0.01, (0.02, 0.0, 0.01, 0.01))


def test_load_pgm_keeps_a_first_pixel_that_is_a_whitespace_byte(tmp_path):
    grid = np.array([[10, 32], [9, 254]], dtype=np.uint8)  # 10은 줄바꿈, 32는 빈칸, 9는 탭 바이트다
    alm.save_pgm(tmp_path / "w.pgm", grid)
    assert alm.load_pgm(tmp_path / "w.pgm").tolist() == grid.tolist()


def test_load_pgm_rejects_a_file_with_the_wrong_number_of_pixels(tmp_path):
    (tmp_path / "s.pgm").write_bytes(b"P5\n2 2\n255\n\xfe\xfe")
    with pytest.raises(ValueError, match="pixel"):
        alm.load_pgm(tmp_path / "s.pgm")
    (tmp_path / "t.pgm").write_bytes(b"P2\n1 1\n255\n\xfe")
    with pytest.raises(ValueError, match="P5"):
        alm.load_pgm(tmp_path / "t.pgm")


def test_write_makes_a_mask_the_map_server_can_read(tmp_path):
    base = tmp_path / "base"
    base.mkdir()
    grid = np.full((4, 6), alm.FREE, dtype=np.uint8)
    grid[0, :] = alm.BLOCKED
    alm.save_pgm(base / "arena_keepout.pgm", grid)
    (base / "arena_keepout.yaml").write_text(
        "image: arena_keepout.pgm\nmode: trinary\nresolution: 0.01\norigin: [-0.02, -0.01, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
    out = alm.write(base / "arena_keepout.yaml", tmp_path / "out", [(0.0, 0.0, 0.02, 0.01)])
    assert out == tmp_path / "out" / "aisle_lock.yaml"
    text = out.read_text(encoding="utf-8")
    assert "image: aisle_lock.pgm" in text and "origin: [-0.02, -0.01, 0.0]" in text
    made = alm.load_pgm(tmp_path / "out" / "aisle_lock.pgm")
    want = grid.copy()
    want[2, 2:4] = alm.BLOCKED  # y 0.00~0.01은 아래에서 둘째 줄, x 0.00~0.02는 셋째·넷째 칸
    assert made.tolist() == want.tolist()


def test_the_default_rectangle_closes_the_south_aisle_of_v7_and_nothing_else():
    import pathlib
    maps = pathlib.Path(__file__).resolve().parents[2] / "sim/fleet_gz_sim/maps"
    grid = alm.load_pgm(maps / "arena_v7_keepout.pgm")
    out = alm.paint(grid, (-0.15, -0.15), 0.01, alm.V7_SOUTH_AISLE)

    def blocked(x, y):
        return out[out.shape[0] - 1 - int((y + 0.15) / 0.01), int((x + 0.15) / 0.01)] == alm.BLOCKED

    assert blocked(1.0, 0.18) and blocked(0.71, 0.01) and blocked(1.57, 0.35)
    # 가운데 통로, 진열대 틈의 입구, 넓은 공간, 동쪽 끝은 그대로 열려 있다
    assert not blocked(1.0, 0.55) and not blocked(1.14, 0.38)
    assert not blocked(0.49, 0.2) and not blocked(1.645, 0.2)
    assert (out != grid).sum() == 88 * 36
