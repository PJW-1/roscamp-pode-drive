"""gz.launch.py가 배치와 시나리오에서 정하는 것. ROS 없이 돈다.
배치(layout)별 구역 지도와 월드, Pode 시작 자리, 돌릴 수 없는 시나리오를 정한다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml
from fleet_core.fleet_map import KNOWN_LAYOUTS, load_fleet_map
from fleet_core.fleet_map.model import SHARED

DEFAULT_LAYOUT = "v7"
ARENA_FAULTS = ("low_battery",)  # 경기장에서도 일으킬 수 있는 로봇 고장(sim_stand_in)


class ScenarioUnfit(RuntimeError):
    """Gazebo 경기장에서 돌릴 수 없는 시나리오."""


def layout_of(value: str) -> str:
    if value not in KNOWN_LAYOUTS:
        raise ScenarioUnfit(f"layout must be one of {', '.join(KNOWN_LAYOUTS)}, got {value!r}")
    return value


def scenario_path(value: str, *shares: str | Path) -> Path:
    """파일 경로이거나, 주어진 패키지 share의 scenarios 폴더에 있는 이름(앞 share가 먼저)."""
    path = Path(value)
    if path.is_file():
        return path
    name = value if value.endswith(".yaml") else value + ".yaml"
    folders = [Path(share) / "scenarios" for share in shares]
    for folder in folders:
        if (folder / name).is_file():
            return folder / name
    where = ", ".join(str(folder) for folder in folders)
    raise ScenarioUnfit(f"scenario {value!r} is nowhere: not a file and not in {where}")


def obstacles_path(value: str, share: str | Path) -> str:
    """gz.launch.py의 obstacles 인자. 비면 장애물 없음, 아니면 파일이나 share/obstacles의 이름."""
    from fleet_gz_sim.obstacles import ScheduleError
    if not value:
        return ""
    if Path(value).is_file():
        return value
    name = value if value.endswith(".yaml") else value + ".yaml"
    path = Path(share) / "obstacles" / name
    if not path.is_file():
        raise ScheduleError(f"obstacle schedule {value!r} is nowhere: not a file and not in "
                            f"{path.parent}")
    return str(path)


@dataclass(frozen=True)
class Arena:
    """한 배치의 Gazebo 월드, Nav2 지도 둘, keepout 마스크. layout은 관제 map_file 값이기도 하다."""
    layout: str
    world: str
    nav_map: str
    nav_map_fine: str
    keepout_mask: str


# 배치별 기본 갈래(v7은 2.5 cm 지도)
DEFAULT_NAV_VARIANT = {"v7": "a"}
BASE_NAV_VARIANT = "base"  # 갈래 없는 5 cm 지도(비교용)


def arena_for(layout: str, gz_share: str | Path, nav_map_variant: str = "") -> Arena:
    """nav_map_variant: 비면 배치 기본(DEFAULT_NAV_VARIANT, 없으면 5 cm 지도), base면 5 cm 지도.
    아니면 arena_maps.NAV_VARIANTS의 갈래 지도(maps/<경기장>_<갈래>.yaml)로 nav_map만 바꾼다.
    1 cm 지도와 keepout 마스크는 그대로."""
    from fleet_gz_sim.arena import NAV_VARIANT_LAYOUTS
    from fleet_gz_sim.arena_maps import NAV_VARIANTS
    name = load_fleet_map(layout_of(layout)).name
    if nav_map_variant and nav_map_variant not in (*NAV_VARIANTS, BASE_NAV_VARIANT):
        raise ScenarioUnfit(f"nav_map_variant must be empty, {BASE_NAV_VARIANT} or one of"
                            f" {', '.join(NAV_VARIANTS)}, got {nav_map_variant!r}")
    if nav_map_variant == BASE_NAV_VARIANT:
        nav_map_variant = ""
    elif not nav_map_variant:
        nav_map_variant = DEFAULT_NAV_VARIANT.get(layout, "")
    if nav_map_variant and layout not in NAV_VARIANT_LAYOUTS:
        raise ScenarioUnfit(f"nav_map_variant {nav_map_variant!r} exists only for layout"
                            f" {', '.join(NAV_VARIANT_LAYOUTS)}, not {layout!r}")
    gz = Path(gz_share)
    nav_map = f"{name}_{nav_map_variant}" if nav_map_variant else name
    found = Arena(layout, str(gz / "worlds" / f"{name}.sdf"), str(gz / "maps" / f"{nav_map}.yaml"),
                  str(gz / "maps" / f"{name}_fine.yaml"),
                  str(gz / "maps" / f"{name}_keepout.yaml"))
    missing = [p for p in (found.world, found.nav_map, found.nav_map_fine, found.keepout_mask)
               if not Path(p).is_file()]
    if missing:
        raise ScenarioUnfit(f"layout {layout!r} has no {', '.join(missing)} "
                            f"(python3 -m fleet_gz_sim.arena --layout {layout})")
    return found


def read_scenario(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ScenarioUnfit(f"{Path(path).name} is not a mapping")
    return data


def scenario_layout(data: dict) -> str:
    """시나리오의 map(배치 이름). 없으면 v7."""
    return layout_of(str(data.get("map") or DEFAULT_LAYOUT))


def _faults(data: dict, seed: int | None) -> list[dict]:
    from fake_robots.scenario import expand_random
    return [f for f in expand_random(data, seed).get("faults") or [] if isinstance(f, dict)]


def check_fits(data: dict, name: str, layout: str, seed: int | None = None) -> None:
    """배치가 맞고 로봇 고장이 없어야 한다. random 블록은 seed로 풀어 본다(None이면 파일의 seed)."""
    from fake_robots.scenario import ROBOT_FAULTS
    wants = scenario_layout(data)
    if wants != layout:
        raise ScenarioUnfit(f"{name} is for layout {wants!r}, the launch runs {layout!r}")
    faults = sorted({str(f.get("type")) for f in _faults(data, seed)
                     if f.get("type") in ROBOT_FAULTS and f.get("type") not in ARENA_FAULTS})
    if faults:
        raise ScenarioUnfit(f"{name} has robot faults {faults}; only fake podes can make them")


@dataclass(frozen=True)
class StandInInputs:
    """sim_stand_in이 시나리오에서 쓰는 것: 도킹 시간과 고장(dict, fake_robots.battery가 읽는다)."""
    dock_s: float
    faults: list


def stand_in_inputs(path, seed: int | None = None) -> StandInInputs:
    data = read_scenario(path)
    dock = data.get("dock") or {}
    return StandInInputs(float(dock.get("duration_s", 3.0)), _faults(data, seed))


def has_low_battery(data: dict, seed: int | None = None) -> bool:
    return any(f.get("type") == "low_battery" for f in _faults(data, seed))


def start_nodes(site, data: dict) -> dict[str, str]:
    """로봇마다 시작 지점. start가 없으면 아직 안 쓴 주차 지점을 앞에서부터."""
    robots = data.get("robots") or {}
    if not isinstance(robots, dict) or not robots:
        raise ScenarioUnfit("the scenario names no robots")
    given = {robot: str((cfg or {}).get("start") or "") for robot, cfg in robots.items()}
    free = [p for p in site.parking if p not in given.values()]
    out = {}
    for robot, start in given.items():
        if not start:
            if not free:
                raise ScenarioUnfit(f"no parking node left for {robot}")
            start = free.pop(0)
        if start not in site.nodes:
            raise ScenarioUnfit(f"{robot} starts at {start!r}, which is not a node of {site.name}")
        out[robot] = start
    if len(set(out.values())) != len(out):
        raise ScenarioUnfit(f"two robots start on one node: {out}")
    return out


def shelf_end_boxes(shelves, band: float) -> list[tuple[float, float, float, float]]:
    """진열대마다 긴 축 두 끝면을 band만큼 사방으로 넓힌 사각형 둘(낮은 끝, 높은 끝)."""
    boxes = []
    for x0, y0, x1, y1 in shelves:
        if x1 - x0 >= y1 - y0:
            ends = ((x0, y0, x0, y1), (x1, y0, x1, y1))
        else:
            ends = ((x0, y0, x1, y0), (x0, y1, x1, y1))
        boxes += [tuple(round(v, 6) for v in (a - band, b - band, c + band, d + band))
                  for a, b, c, d in ends]
    return boxes


AISLE_MAX_M = 0.45  # 마주 보는 두 면 사이가 이 이하면 통로로 본다(fleet_core.tuck과 같다)
AISLE_MIN_LEN_M = 0.25  # 이보다 짧은 면(충전 칸 칸막이)은 통로를 만들지 않는다


def aisle_strips(shelves, bounds, band: float, max_gap: float = AISLE_MAX_M,
                 min_len: float = AISLE_MIN_LEN_M) -> list[tuple[float, float, float, float]]:
    """진열대 긴 면과 그 면이 마주 보는 벽·진열대 사이가 max_gap 이하인 통로 사각형들.
    두 면을 band만큼 넘겨 잡고, 같은 폭으로 이어지는 통로는 진열대 사이 틈(max_gap 이하)을 넘어 하나로 잇는다."""
    bx0, by0, bx1, by1 = bounds
    rects = []
    for i, (x0, y0, x1, y1) in enumerate(shelves):
        along_x = x1 - x0 >= y1 - y0
        if (x1 - x0 if along_x else y1 - y0) < min_len:
            continue
        # 긴 축을 a, 통로 폭 축을 c로 본다
        a0, a1, c0, c1 = (x0, x1, y0, y1) if along_x else (y0, y1, x0, x1)
        wall_lo, wall_hi = (by0, by1) if along_x else (bx0, bx1)
        below, above = [wall_lo], [wall_hi]
        for j, other in enumerate(shelves):
            if j == i:
                continue
            o0, o1, p0, p1 = ((other[0], other[2], other[1], other[3]) if along_x
                              else (other[1], other[3], other[0], other[2]))
            if o1 <= a0 or o0 >= a1:
                continue
            if p1 <= c0:
                below.append(p1)
            if p0 >= c1:
                above.append(p0)
        for lo, hi in ((max(below), c0), (c1, min(above))):
            if 1e-9 < hi - lo <= max_gap:
                rects.append((along_x, a0, a1, lo - band, hi + band))
    # 같은 폭의 통로끼리 긴 축 순서로 잇는다
    merged = []
    for along_x, lo, hi, a0, a1 in sorted((r[0], round(r[3], 6), round(r[4], 6), r[1], r[2])
                                          for r in rects):
        last = merged[-1] if merged else None
        if last is not None and last[:3] == (along_x, lo, hi) and a0 - last[4] <= max_gap:
            merged[-1] = (along_x, lo, hi, last[3], max(last[4], a1))
        else:
            merged.append((along_x, lo, hi, a0, a1))
    out = [tuple(round(v, 6) for v in ((a0, lo, a1, hi) if along_x else (lo, a0, hi, a1)))
           for along_x, lo, hi, a0, a1 in merged]
    # 다른 띠 안에 든 띠는 뺀다
    return sorted(r for r in out
                  if not any(o != r and o[0] <= r[0] and o[1] <= r[1] and o[2] >= r[2] and o[3] >= r[3]
                             for o in out))


def narrow_strips(site, band: float = 0.05, aisles: bool = False) -> str:
    """스캔 거르개의 좁은 띠를 "x0,y0,x1,y1; ..." 글로 만든다.
    넓은 구역이 아닌 구역 사각형 뒤에 진열대 끝면 상자를 붙이고, 벽에 닿는 변은 band만큼 벽 너머로 넓힌다.
    aisles면 진열대 옆 좁은 통로(aisle_strips)도 붙인다.
    """
    bx0, by0, bx1, by1 = site.bounds()

    def past_walls(rect):
        x0, y0, x1, y1 = rect
        return tuple(round(v, 6) for v in (
            x0 - band if x0 <= bx0 + 1e-9 else x0, y0 - band if y0 <= by0 + 1e-9 else y0,
            x1 + band if x1 >= bx1 - 1e-9 else x1, y1 + band if y1 >= by1 - 1e-9 else y1))

    rects = sorted(past_walls(rect) for zone in site.zones.values() if zone.kind != SHARED
                   for rect in zone.rects)
    rects += shelf_end_boxes(site.sim_shelves, band)
    if aisles:
        rects += aisle_strips(site.sim_shelves, site.bounds(), band)
    return "; ".join(",".join(f"{v:g}" for v in rect) for rect in rects)


def start_poses(site, starts: dict[str, str]) -> dict[str, tuple[float, float, float]]:
    """로봇마다 (x, y, yaw). yaw는 지점의 yaw, 없으면 0."""
    poses = {}
    for robot, node in starts.items():
        x, y = site.node_pose(node)
        yaw = site.nodes[node].yaw
        poses[robot] = (x, y, 0.0 if yaw is None else yaw)
    return poses
