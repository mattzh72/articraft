from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


POST_RADIUS = 0.12
POST_HEIGHT = 2.35
HINGE_AXIS_X = 0.16
LOWER_HINGE_Z = 0.26
HINGE_SPACING = 1.02

GATE_WIDTH = 3.20
GATE_HEIGHT = 1.20
GATE_THICKNESS = 0.05
STILE_WIDTH = 0.12
RAIL_HEIGHT = 0.09


def _timber(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    wood,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=wood,
        name=name,
    )


def _midpoint(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_swing_gate")

    post_wood = model.material("post_wood", rgba=(0.50, 0.38, 0.24, 1.0))
    gate_wood = model.material("gate_wood", rgba=(0.64, 0.48, 0.30, 1.0))
    steel = model.material("hinge_steel", rgba=(0.30, 0.31, 0.33, 1.0))

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT * 0.5 - 0.20)),
        material=post_wood,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS * 0.92, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT - 0.16)),
        material=post_wood,
        name="post_cap",
    )
    post.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(
            xyz=(0.12, -0.044, LOWER_HINGE_Z + 0.045),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=steel,
        name="lower_pintle_pin",
    )
    post.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(
            xyz=(0.12, -0.044, LOWER_HINGE_Z + HINGE_SPACING - 0.005),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=steel,
        name="upper_pintle_pin",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT * 0.5 - 0.20)),
    )

    hinge_spine = model.part("hinge_spine")
    hinge_spine.inertial = Inertial.from_geometry(
        Box((0.07, 0.03, HINGE_SPACING + 0.04)),
        mass=8.0,
        origin=Origin(xyz=(0.035, -0.045, HINGE_SPACING * 0.5)),
    )
    hinge_spine.visual(
        Box((0.03, 0.02, HINGE_SPACING + 0.04)),
        origin=Origin(xyz=(0.045, -0.044, HINGE_SPACING * 0.5)),
        material=steel,
        name="spine_strap",
    )
    hinge_spine.visual(
        Box((0.09, 0.02, 0.09)),
        origin=Origin(xyz=(0.045, -0.044, 0.045)),
        material=steel,
        name="lower_hinge_tab",
    )
    hinge_spine.visual(
        Box((0.09, 0.02, 0.09)),
        origin=Origin(xyz=(0.045, -0.044, HINGE_SPACING - 0.005)),
        material=steel,
        name="upper_hinge_tab",
    )

    gate_panel = model.part("gate_panel")
    gate_panel.inertial = Inertial.from_geometry(
        Box((GATE_WIDTH, 0.10, GATE_HEIGHT)),
        mass=55.0,
        origin=Origin(xyz=(GATE_WIDTH * 0.5, 0.0, -0.51)),
    )

    top_edge = 0.09
    bottom_edge = top_edge - GATE_HEIGHT
    upper_axis_z = 0.0
    lower_axis_z = -HINGE_SPACING

    gate_panel.visual(
        Box((STILE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        origin=Origin(xyz=(STILE_WIDTH * 0.5, 0.0, (top_edge + bottom_edge) * 0.5)),
        material=gate_wood,
        name="hinge_stile",
    )
    _timber(
        gate_panel,
        (STILE_WIDTH, GATE_THICKNESS, GATE_HEIGHT),
        (GATE_WIDTH - STILE_WIDTH * 0.5, 0.0, (top_edge + bottom_edge) * 0.5),
        wood=gate_wood,
        name="latch_stile",
    )
    _timber(
        gate_panel,
        (GATE_WIDTH - 2.0 * STILE_WIDTH, GATE_THICKNESS, RAIL_HEIGHT),
        (GATE_WIDTH * 0.5, 0.0, top_edge - RAIL_HEIGHT * 0.5),
        wood=gate_wood,
        name="top_rail",
    )
    _timber(
        gate_panel,
        (GATE_WIDTH - 2.0 * STILE_WIDTH, GATE_THICKNESS, RAIL_HEIGHT),
        (GATE_WIDTH * 0.5, 0.0, bottom_edge + RAIL_HEIGHT * 0.5),
        wood=gate_wood,
        name="bottom_rail",
    )

    for index, rail_z in enumerate((-0.39, -0.73), start=1):
        _timber(
            gate_panel,
            (GATE_WIDTH - 2.0 * STILE_WIDTH, 0.045, 0.075),
            (GATE_WIDTH * 0.5, 0.0, rail_z),
            wood=gate_wood,
            name=f"mid_rail_{index}",
        )

    _timber(
        gate_panel,
        (0.09, 0.045, 1.01),
        (1.62, 0.0, -0.515),
        wood=gate_wood,
        name="center_upright",
    )

    brace_start = (0.18, bottom_edge + 0.11)
    brace_end = (GATE_WIDTH - 0.16, top_edge - 0.10)
    brace_center_x, brace_center_z = _midpoint(brace_start, brace_end)
    brace_run = brace_end[0] - brace_start[0]
    brace_rise = brace_end[1] - brace_start[1]
    brace_length = sqrt(brace_run * brace_run + brace_rise * brace_rise)
    brace_pitch = -atan2(brace_rise, brace_run)
    _timber(
        gate_panel,
        (brace_length, 0.042, 0.09),
        (brace_center_x, 0.0, brace_center_z),
        wood=gate_wood,
        name="diagonal_brace",
        rpy=(0.0, brace_pitch, 0.0),
    )

    gate_panel.visual(
        Cylinder(radius=0.032, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, upper_axis_z)),
        material=steel,
        name="upper_hinge_barrel",
    )
    gate_panel.visual(
        Cylinder(radius=0.032, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, lower_axis_z)),
        material=steel,
        name="lower_hinge_barrel",
    )
    gate_panel.visual(
        Box((0.22, 0.012, 0.10)),
        origin=Origin(xyz=(0.11, -0.028, upper_axis_z)),
        material=steel,
        name="upper_hinge_strap",
    )
    gate_panel.visual(
        Box((0.22, 0.012, 0.10)),
        origin=Origin(xyz=(0.11, -0.028, lower_axis_z)),
        material=steel,
        name="lower_hinge_strap",
    )

    hinge_limits = MotionLimits(
        effort=80.0,
        velocity=1.2,
        lower=0.0,
        upper=1.25,
    )

    model.articulation(
        "lower_pintle_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=hinge_spine,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, LOWER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "upper_pintle_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_spine,
        child=gate_panel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_SPACING)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
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
    post = object_model.get_part("post")
    hinge_spine = object_model.get_part("hinge_spine")
    gate_panel = object_model.get_part("gate_panel")
    lower_hinge = object_model.get_articulation("lower_pintle_hinge")
    upper_hinge = object_model.get_articulation("upper_pintle_hinge")

    lower_axis = lower_hinge.axis
    upper_axis = upper_hinge.axis
    upper_origin = upper_hinge.origin.xyz
    lower_limits = lower_hinge.motion_limits
    upper_limits = upper_hinge.motion_limits

    ctx.check(
        "dual pintle hinges stay vertically aligned",
        lower_axis == (0.0, 0.0, 1.0)
        and upper_axis == (0.0, 0.0, 1.0)
        and abs(upper_origin[0]) < 1e-9
        and abs(upper_origin[1]) < 1e-9
        and abs(upper_origin[2] - HINGE_SPACING) < 1e-9
        and lower_limits is not None
        and upper_limits is not None
        and lower_limits.lower == 0.0
        and upper_limits.lower == 0.0
        and abs((lower_limits.upper or 0.0) - (upper_limits.upper or 0.0)) < 1e-9,
        details=(
            f"lower_axis={lower_axis}, upper_axis={upper_axis}, "
            f"upper_origin={upper_origin}, "
            f"lower_limits={lower_limits}, upper_limits={upper_limits}"
        ),
    )

    ctx.expect_gap(
        gate_panel,
        post,
        axis="x",
        min_gap=0.005,
        positive_elem="hinge_stile",
        negative_elem="post_shaft",
        name="closed gate clears the timber post",
    )

    rest_aabb = ctx.part_world_aabb(gate_panel)
    with ctx.pose({lower_hinge: 0.95, upper_hinge: 0.95}):
        open_aabb = ctx.part_world_aabb(gate_panel)

    opens_clear = (
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > rest_aabb[1][1] + 1.8
        and open_aabb[0][0] < rest_aabb[0][0] - 1.0
    )
    ctx.check(
        "gate panel swings broadly outward when both pintles turn",
        opens_clear,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    ctx.expect_contact(
        hinge_spine,
        gate_panel,
        elem_a="upper_hinge_tab",
        elem_b="upper_hinge_strap",
        contact_tol=0.004,
        name="upper hinge hardware stays visually mounted",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
