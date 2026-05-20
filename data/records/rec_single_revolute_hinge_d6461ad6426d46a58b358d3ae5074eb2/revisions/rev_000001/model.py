from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_LENGTH = 0.90
LEAF_WIDTH = 0.048
LEAF_THICKNESS = 0.0032
BARREL_RADIUS = 0.0062
PIN_RADIUS = 0.0024
PIN_CLEARANCE = 0.0006
PIN_HEAD_RADIUS = 0.009
PIN_HEAD_HEIGHT = 0.0035
PIN_HEAD_STANDOFF = 0.0008
PIN_TAIL = 0.0025
LEAF_CLEARANCE = 0.0004
WEB_BRIDGE = 0.005
WEB_INNER_RADIUS = 0.004
KNUCKLE_COUNT = 7
KNUCKLE_GAP = 0.0
OPEN_LIMIT = 1.60


def _box_between(x0: float, x1: float, y_size: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(abs(x1 - x0), y_size, z1 - z0)
        .translate(((x0 + x1) / 2.0, 0.0, (z0 + z1) / 2.0))
    )


def _cylinder_segment(radius: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((0.0, 0.0, z0))


def _knuckle_ranges() -> list[tuple[float, float]]:
    segment_length = (HINGE_LENGTH - KNUCKLE_GAP * (KNUCKLE_COUNT - 1)) / KNUCKLE_COUNT
    ranges: list[tuple[float, float]] = []
    z_cursor = -HINGE_LENGTH / 2.0
    for _ in range(KNUCKLE_COUNT):
        z0 = z_cursor
        z1 = z0 + segment_length
        ranges.append((z0, z1))
        z_cursor = z1 + KNUCKLE_GAP
    return ranges


def _leaf_panel(side: int, knuckle_ranges: list[tuple[float, float]]) -> cq.Workplane:
    if side < 0:
        plate_x0 = -BARREL_RADIUS - LEAF_CLEARANCE - LEAF_WIDTH
        plate_x1 = -BARREL_RADIUS - LEAF_CLEARANCE
        web_x0 = -BARREL_RADIUS - LEAF_CLEARANCE
        web_x1 = -BARREL_RADIUS + WEB_BRIDGE
    else:
        plate_x0 = BARREL_RADIUS + LEAF_CLEARANCE
        plate_x1 = BARREL_RADIUS + LEAF_CLEARANCE + LEAF_WIDTH
        web_x0 = BARREL_RADIUS - WEB_BRIDGE
        web_x1 = BARREL_RADIUS + LEAF_CLEARANCE

    panel = _box_between(plate_x0, plate_x1, LEAF_THICKNESS, -HINGE_LENGTH / 2.0, HINGE_LENGTH / 2.0)
    for z0, z1 in knuckle_ranges:
        panel = panel.union(_box_between(web_x0, web_x1, LEAF_THICKNESS, z0, z1))
    return panel


def _leaf_plate(side: int) -> cq.Workplane:
    if side < 0:
        plate_x0 = -BARREL_RADIUS - LEAF_CLEARANCE - LEAF_WIDTH
        plate_x1 = -BARREL_RADIUS - LEAF_CLEARANCE
    else:
        plate_x0 = BARREL_RADIUS + LEAF_CLEARANCE
        plate_x1 = BARREL_RADIUS + LEAF_CLEARANCE + LEAF_WIDTH
    return _box_between(plate_x0, plate_x1, LEAF_THICKNESS, -HINGE_LENGTH / 2.0, HINGE_LENGTH / 2.0)


def _leaf_webs(side: int, knuckle_ranges: list[tuple[float, float]]) -> cq.Workplane:
    if side < 0:
        web_x0 = -BARREL_RADIUS - LEAF_CLEARANCE
        web_x1 = -WEB_INNER_RADIUS
    else:
        web_x0 = WEB_INNER_RADIUS
        web_x1 = BARREL_RADIUS + LEAF_CLEARANCE

    webs = None
    for z0, z1 in knuckle_ranges:
        web = _box_between(web_x0, web_x1, LEAF_THICKNESS, z0, z1)
        webs = web if webs is None else webs.union(web)
    return webs


def _root_barrel(knuckle_ranges: list[tuple[float, float]]) -> cq.Workplane:
    barrel = None
    for z0, z1 in knuckle_ranges:
        segment = _cylinder_segment(BARREL_RADIUS, z0, z1)
        barrel = segment if barrel is None else barrel.union(segment)

    neck = _cylinder_segment(
        PIN_RADIUS,
        HINGE_LENGTH / 2.0,
        HINGE_LENGTH / 2.0 + PIN_HEAD_STANDOFF,
    )
    head = _cylinder_segment(
        PIN_HEAD_RADIUS,
        HINGE_LENGTH / 2.0 + PIN_HEAD_STANDOFF,
        HINGE_LENGTH / 2.0 + PIN_HEAD_STANDOFF + PIN_HEAD_HEIGHT,
    )

    return barrel.union(neck).union(head)


def _swing_barrel(knuckle_ranges: list[tuple[float, float]]) -> cq.Workplane:
    barrel = None
    for z0, z1 in knuckle_ranges:
        outer = _cylinder_segment(BARREL_RADIUS, z0, z1)
        hole = _cylinder_segment(PIN_RADIUS + PIN_CLEARANCE, z0 - 0.001, z1 + 0.001)
        segment = outer.cut(hole)
        barrel = segment if barrel is None else barrel.union(segment)
    return barrel


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_door_hinge")

    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.43, 0.45, 0.48, 1.0))

    all_ranges = _knuckle_ranges()
    root_ranges = [knuckle for idx, knuckle in enumerate(all_ranges) if idx % 2 == 0]
    swing_ranges = [knuckle for idx, knuckle in enumerate(all_ranges) if idx % 2 == 1]

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(_leaf_plate(-1), "fixed_leaf_leaf"),
        material=satin_steel,
        name="leaf",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_leaf_webs(-1, root_ranges), "fixed_leaf_webs"),
        material=satin_steel,
        name="webs",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_root_barrel(root_ranges), "fixed_leaf_barrel"),
        material=darker_steel,
        name="barrel",
    )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        mesh_from_cadquery(_leaf_plate(1), "swing_leaf_leaf"),
        material=satin_steel,
        name="leaf",
    )
    swing_leaf.visual(
        mesh_from_cadquery(_leaf_webs(1, swing_ranges), "swing_leaf_webs"),
        material=satin_steel,
        name="webs",
    )
    swing_leaf.visual(
        mesh_from_cadquery(_swing_barrel(swing_ranges), "swing_leaf_barrel"),
        material=darker_steel,
        name="barrel",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=swing_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    fixed_leaf = object_model.get_part("fixed_leaf")
    swing_leaf = object_model.get_part("swing_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    limits = hinge.motion_limits
    ctx.check(
        "hinge axis follows the barrel",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge limit matches a service-door swing",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.4 <= limits.upper <= 1.7,
        details=f"limits={limits}",
    )

    rest_panel_aabb = None
    open_panel_aabb = None

    with ctx.pose({hinge: 0.0}):
        ctx.expect_origin_distance(
            fixed_leaf,
            swing_leaf,
            axes="xy",
            max_dist=1e-6,
            name="leaf origins stay coaxial with the pin",
        )
        ctx.expect_gap(
            swing_leaf,
            fixed_leaf,
            axis="x",
            positive_elem="leaf",
            negative_elem="leaf",
            min_gap=0.012,
            max_gap=0.015,
            name="flat leaves sit on opposite sides of the barrel",
        )
        ctx.expect_overlap(
            fixed_leaf,
            swing_leaf,
            axes="z",
            elem_a="barrel",
            elem_b="barrel",
            min_overlap=0.63,
            name="alternating knuckles share one long barrel span",
        )
        ctx.expect_contact(
            fixed_leaf,
            swing_leaf,
            elem_a="barrel",
            elem_b="barrel",
            name="resting knuckles stay physically stacked together",
        )
        rest_panel_aabb = ctx.part_element_world_aabb(swing_leaf, elem="leaf")

    with ctx.pose({hinge: 1.20}):
        ctx.expect_overlap(
            fixed_leaf,
            swing_leaf,
            axes="z",
            elem_a="barrel",
            elem_b="barrel",
            min_overlap=0.63,
            name="open leaf stays supported along the barrel axis",
        )
        ctx.expect_contact(
            fixed_leaf,
            swing_leaf,
            elem_a="barrel",
            elem_b="barrel",
            name="opened knuckles remain nested on the barrel",
        )
        open_panel_aabb = ctx.part_element_world_aabb(swing_leaf, elem="leaf")

    rest_center = _aabb_center(rest_panel_aabb)
    open_center = _aabb_center(open_panel_aabb)
    ctx.check(
        "swing leaf rotates outward around the hinge pin",
        rest_center is not None
        and open_center is not None
        and open_center[1] > rest_center[1] + 0.025
        and open_center[0] < rest_center[0] - 0.01,
        details=f"rest_center={rest_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
