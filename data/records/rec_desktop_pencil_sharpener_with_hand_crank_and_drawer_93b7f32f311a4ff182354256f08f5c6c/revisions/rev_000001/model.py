from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_LENGTH = 0.130
HOUSING_WIDTH = 0.090
HOUSING_HEIGHT = 0.118

DRAWER_BOTTOM = 0.018
DRAWER_TRAVEL = 0.038
DRAWER_DEPTH = 0.062
DRAWER_WIDTH = 0.060
DRAWER_HEIGHT = 0.038
DRAWER_SEAT_X = HOUSING_LENGTH * 0.5
DRAWER_SEAT_Z = DRAWER_BOTTOM + 0.001

PENCIL_ENTRY_Z = 0.078
CRANK_X = -0.008
CRANK_Z = 0.074


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    top_crown = (
        cq.Workplane("XY")
        .box(0.082, 0.068, 0.018, centered=(True, True, False))
        .translate((0.004, 0.0, HOUSING_HEIGHT - 0.010))
    )
    front_hump = (
        cq.Workplane("XY")
        .box(0.028, 0.064, 0.022, centered=(True, True, False))
        .translate((0.044, 0.0, 0.057))
    )
    crank_boss = (
        cq.Workplane("XZ")
        .workplane(offset=HOUSING_WIDTH * 0.5)
        .center(CRANK_X, CRANK_Z)
        .circle(0.016)
        .extrude(0.012)
    )
    spindle_cap = (
        cq.Workplane("XZ")
        .workplane(offset=-(HOUSING_WIDTH * 0.5 + 0.008))
        .center(CRANK_X, CRANK_Z)
        .circle(0.010)
        .extrude(0.008)
    )
    housing = body.union(top_crown).union(front_hump).union(crank_boss).union(spindle_cap)

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.074, 0.070, 0.046, centered=(True, True, False))
        .translate((HOUSING_LENGTH * 0.5 - 0.035, 0.0, DRAWER_BOTTOM))
    )
    chip_chute = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.032, centered=(True, True, False))
        .translate((0.038, 0.0, 0.039))
    )
    internal_cavity = (
        cq.Workplane("YZ")
        .workplane(offset=HOUSING_LENGTH * 0.5 + 0.002)
        .center(0.0, PENCIL_ENTRY_Z)
        .circle(0.0135)
        .extrude(-0.032)
    )
    entry_bore = (
        cq.Workplane("YZ")
        .workplane(offset=HOUSING_LENGTH * 0.5 + 0.010)
        .center(0.0, PENCIL_ENTRY_Z)
        .circle(0.0055)
        .extrude(-0.052)
    )
    entry_counterbore = (
        cq.Workplane("YZ")
        .workplane(offset=HOUSING_LENGTH * 0.5 + 0.010)
        .center(0.0, PENCIL_ENTRY_Z)
        .circle(0.0086)
        .extrude(-0.006)
    )
    crank_bore = (
        cq.Workplane("XZ")
        .workplane(offset=HOUSING_WIDTH * 0.5 + 0.014)
        .center(CRANK_X, CRANK_Z)
        .circle(0.0070)
        .extrude(-0.028)
    )

    return housing.cut(drawer_cavity).cut(chip_chute).cut(internal_cavity).cut(entry_bore).cut(
        entry_counterbore
    ).cut(crank_bore)


def _build_drawer_pan_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(DRAWER_DEPTH, DRAWER_WIDTH, DRAWER_HEIGHT, centered=(True, True, False))
        .translate((-DRAWER_DEPTH * 0.5, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(DRAWER_DEPTH - 0.005, DRAWER_WIDTH - 0.005, DRAWER_HEIGHT, centered=(True, True, False))
        .translate((-(DRAWER_DEPTH * 0.5) - 0.0005, 0.0, 0.0025))
    )
    return outer.cut(inner)


def _build_clamp_bracket_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.058, 0.056, 0.006)
    left_cheek = cq.Workplane("XY").box(0.044, 0.004, 0.050).translate((0.0, 0.026, -0.028))
    right_cheek = cq.Workplane("XY").box(0.044, 0.004, 0.050).translate((0.0, -0.026, -0.028))
    front_jaw = cq.Workplane("XY").box(0.008, 0.048, 0.020).translate((0.025, 0.0, -0.042))
    hole = cq.Workplane("XY").circle(0.0065).extrude(0.020, both=True)

    return plate.union(left_cheek).union(right_cheek).union(front_jaw).cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_pencil_sharpener")

    cast_burgundy = model.material("cast_burgundy", rgba=(0.47, 0.10, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing_shell"),
        material=cast_burgundy,
        name="housing_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_DEPTH, DRAWER_WIDTH, 0.003)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, 0.0, 0.0015)),
        material=dark_steel,
        name="drawer_body",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.003, 0.032)),
        origin=Origin(
            xyz=(-DRAWER_DEPTH * 0.5, DRAWER_WIDTH * 0.5 - 0.0015, 0.016),
        ),
        material=dark_steel,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.003, 0.032)),
        origin=Origin(
            xyz=(-DRAWER_DEPTH * 0.5, -(DRAWER_WIDTH * 0.5 - 0.0015), 0.016),
        ),
        material=dark_steel,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.003, DRAWER_WIDTH - 0.006, 0.032)),
        origin=Origin(xyz=(-DRAWER_DEPTH + 0.0015, 0.0, 0.016)),
        material=dark_steel,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.008, 0.074, 0.050)),
        origin=Origin(xyz=(0.004, 0.0, 0.025)),
        material=cast_burgundy,
        name="drawer_face",
    )
    drawer.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(0.010, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="drawer_stem",
    )
    drawer.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.019, 0.0, 0.022)),
        material=black_plastic,
        name="drawer_knob",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0052, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="crank_axle",
    )
    crank.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="crank_hub",
    )
    _add_member(
        crank,
        (0.0, 0.024, 0.0),
        (0.030, 0.037, 0.0),
        radius=0.0040,
        material=machined_steel,
        name="crank_arm",
    )
    _add_member(
        crank,
        (0.030, 0.037, 0.0),
        (0.041, 0.043, 0.019),
        radius=0.0040,
        material=machined_steel,
        name="crank_offset",
    )
    crank.visual(
        Cylinder(radius=0.0032, length=0.024),
        origin=Origin(xyz=(0.041, 0.043, 0.018)),
        material=machined_steel,
        name="grip_spindle",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.022),
        origin=Origin(xyz=(0.041, 0.043, 0.018)),
        material=black_plastic,
        name="crank_grip",
    )

    clamp_bracket = model.part("clamp_bracket")
    clamp_bracket.visual(
        mesh_from_cadquery(_build_clamp_bracket_shape(), "clamp_bracket"),
        material=dark_steel,
        name="clamp_bracket",
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.0052, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=machined_steel,
        name="screw_shaft",
    )
    clamp_screw.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=machined_steel,
        name="screw_collar",
    )
    clamp_screw.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=machined_steel,
        name="pressure_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=machined_steel,
        name="screw_neck",
    )
    clamp_screw.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=black_plastic,
        name="screw_knob",
    )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(DRAWER_SEAT_X, 0.0, DRAWER_SEAT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.12, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(CRANK_X, HOUSING_WIDTH * 0.5 + 0.008, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "housing_to_clamp_bracket",
        ArticulationType.FIXED,
        parent=housing,
        child=clamp_bracket,
        origin=Origin(xyz=(-0.030, 0.0, -0.003)),
    )
    model.articulation(
        "clamp_bracket_to_clamp_screw",
        ArticulationType.CONTINUOUS,
        parent=clamp_bracket,
        child=clamp_screw,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    clamp_bracket = object_model.get_part("clamp_bracket")
    clamp_screw = object_model.get_part("clamp_screw")

    drawer_slide = object_model.get_articulation("housing_to_drawer")

    ctx.expect_gap(
        housing,
        clamp_bracket,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="clamp bracket mounts directly under the housing",
    )
    ctx.expect_within(
        drawer,
        housing,
        axes="yz",
        inner_elem="drawer_body",
        outer_elem="housing_shell",
        margin=0.004,
        name="drawer body stays centered in the housing opening",
    )
    ctx.expect_origin_gap(
        crank,
        housing,
        axis="y",
        min_gap=0.048,
        max_gap=0.055,
        name="hand crank sits on the housing side",
    )
    ctx.expect_overlap(
        clamp_screw,
        clamp_bracket,
        axes="xy",
        elem_a="screw_shaft",
        elem_b="clamp_bracket",
        min_overlap=0.008,
        name="clamp screw stays aligned with the bracket throat",
    )

    rest_pos = ctx.part_world_position(drawer)
    upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({drawer_slide: upper}):
            ctx.expect_within(
                drawer,
                housing,
                axes="yz",
                inner_elem="drawer_body",
                outer_elem="housing_shell",
                margin=0.004,
                name="extended drawer remains guided by the housing rails",
            )
            ctx.expect_overlap(
                drawer,
                housing,
                axes="x",
                elem_a="drawer_body",
                elem_b="housing_shell",
                min_overlap=0.020,
                name="extended drawer keeps retained insertion in the housing",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.025,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
