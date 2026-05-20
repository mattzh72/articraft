from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


FRONT_HEIGHT = 2.18
FRONT_BOTTOM_HALF_WIDTH = 0.36
FRONT_TOP_HALF_WIDTH = 0.13
FRONT_STILE_ANGLE = math.atan(
    (FRONT_BOTTOM_HALF_WIDTH - FRONT_TOP_HALF_WIDTH) / FRONT_HEIGHT
)

REAR_LEG_OPEN = 0.43
BRACE_OPEN = 0.82


def _front_half_width_at_z(z: float) -> float:
    clamped_z = max(0.0, min(FRONT_HEIGHT, z))
    t = clamped_z / FRONT_HEIGHT
    return FRONT_BOTTOM_HALF_WIDTH + (FRONT_TOP_HALF_WIDTH - FRONT_BOTTOM_HALF_WIDTH) * t


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchard_picking_ladder")

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    tread_gray = model.material("tread_gray", rgba=(0.54, 0.56, 0.59, 1.0))
    hardware = model.material("hardware", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.036, 0.040, 2.22)),
        origin=Origin(
            xyz=(0.0, -(FRONT_BOTTOM_HALF_WIDTH + FRONT_TOP_HALF_WIDTH) * 0.5, 1.11),
            rpy=(-FRONT_STILE_ANGLE, 0.0, 0.0),
        ),
        material=aluminum,
        name="left_stile",
    )
    front_frame.visual(
        Box((0.036, 0.040, 2.22)),
        origin=Origin(
            xyz=(0.0, (FRONT_BOTTOM_HALF_WIDTH + FRONT_TOP_HALF_WIDTH) * 0.5, 1.11),
            rpy=(FRONT_STILE_ANGLE, 0.0, 0.0),
        ),
        material=aluminum,
        name="right_stile",
    )

    rung_zs = (0.34, 0.60, 0.86, 1.12, 1.38, 1.64, 1.90)
    for index, rung_z in enumerate(rung_zs, start=1):
        rung_width = 2.0 * _front_half_width_at_z(rung_z) + 0.070
        front_frame.visual(
            Box((0.110, rung_width, 0.028)),
            origin=Origin(xyz=(0.031, 0.0, rung_z)),
            material=tread_gray,
            name=f"rung_{index:02d}",
        )

    front_frame.visual(
        Box((0.080, 0.360, 0.080)),
        origin=Origin(xyz=(0.025, 0.0, 2.18)),
        material=aluminum,
        name="top_head",
    )
    front_frame.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.015, -0.030, 2.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_barrel_left",
    )
    front_frame.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.015, 0.030, 2.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_barrel_right",
    )

    front_frame.visual(
        Box((0.040, 0.460, 0.050)),
        origin=Origin(xyz=(0.004, 0.0, 1.30)),
        material=aluminum,
        name="brace_cross_tie",
    )
    front_frame.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.020, 0.039, 1.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="brace_mount_barrel_lower",
    )
    front_frame.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.020, 0.091, 1.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="brace_mount_barrel_upper",
    )

    front_frame.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.005, -FRONT_BOTTOM_HALF_WIDTH, 0.020)),
        material=rubber,
        name="left_foot",
    )
    front_frame.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.005, FRONT_BOTTOM_HALF_WIDTH, 0.020)),
        material=rubber,
        name="right_foot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.160, 0.790, 2.22)),
        mass=8.2,
        origin=Origin(xyz=(0.012, 0.0, 1.11)),
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.040, 0.035, 2.14)),
        origin=Origin(xyz=(-0.055, 0.0, -1.07)),
        material=aluminum,
        name="rear_stile",
    )
    rear_leg.visual(
        Box((0.040, 0.080, 0.140)),
        origin=Origin(xyz=(-0.055, 0.0, -0.070)),
        material=aluminum,
        name="rear_head",
    )
    rear_leg.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(-0.034, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_barrel",
    )
    rear_leg.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.080, 0.065, -1.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="brace_lug",
    )
    rear_leg.visual(
        Box((0.020, 0.075, 0.050)),
        origin=Origin(xyz=(-0.075, 0.0325, -1.72)),
        material=hardware,
        name="brace_lug_bracket",
    )
    rear_leg.visual(
        Box((0.090, 0.060, 0.040)),
        origin=Origin(xyz=(-0.060, 0.0, -2.10)),
        material=rubber,
        name="rear_foot",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 2.14)),
        mass=3.1,
        origin=Origin(xyz=(-0.055, 0.0, -1.07)),
    )

    brace_strut = model.part("brace_strut")
    brace_strut.visual(
        Box((0.028, 0.046, 0.960)),
        origin=Origin(xyz=(-0.040, 0.0, -0.480)),
        material=aluminum,
        name="brace_bar",
    )
    brace_strut.visual(
        Box((0.032, 0.050, 0.080)),
        origin=Origin(xyz=(-0.036, 0.0, -0.040)),
        material=aluminum,
        name="brace_knuckle",
    )
    brace_strut.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="brace_hinge_barrel",
    )
    brace_strut.visual(
        Box((0.040, 0.054, 0.100)),
        origin=Origin(xyz=(-0.040, 0.0, -0.930)),
        material=aluminum,
        name="brace_lock_head",
    )
    brace_strut.visual(
        Box((0.024, 0.014, 0.120)),
        origin=Origin(xyz=(-0.040, 0.022, -0.935)),
        material=aluminum,
        name="brace_yoke_outer",
    )
    brace_strut.visual(
        Box((0.024, 0.014, 0.120)),
        origin=Origin(xyz=(-0.040, -0.022, -0.935)),
        material=aluminum,
        name="brace_yoke_inner",
    )
    brace_strut.inertial = Inertial.from_geometry(
        Box((0.080, 0.100, 0.980)),
        mass=1.2,
        origin=Origin(xyz=(-0.040, 0.0, -0.490)),
    )

    model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(-0.015, 0.0, 2.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=0.62,
        ),
    )
    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=brace_strut,
        origin=Origin(xyz=(-0.020, 0.065, 1.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
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

    front_frame = object_model.get_part("front_frame")
    rear_leg = object_model.get_part("rear_leg")
    brace_strut = object_model.get_part("brace_strut")
    rear_leg_hinge = object_model.get_articulation("rear_leg_hinge")
    brace_hinge = object_model.get_articulation("brace_hinge")

    ctx.check(
        "orchard ladder parts present",
        all(part is not None for part in (front_frame, rear_leg, brace_strut)),
        details="Expected front frame, rear leg, and folding brace strut parts.",
    )
    ctx.check(
        "rear leg hinge uses backward opening limits",
        rear_leg_hinge.motion_limits is not None
        and rear_leg_hinge.motion_limits.lower == 0.0
        and rear_leg_hinge.motion_limits.upper is not None
        and rear_leg_hinge.motion_limits.upper >= 0.55,
        details=f"limits={rear_leg_hinge.motion_limits}",
    )
    ctx.check(
        "brace hinge has folding travel",
        brace_hinge.motion_limits is not None
        and brace_hinge.motion_limits.lower == 0.0
        and brace_hinge.motion_limits.upper is not None
        and brace_hinge.motion_limits.upper >= 1.0,
        details=f"limits={brace_hinge.motion_limits}",
    )

    with ctx.pose({rear_leg_hinge: 0.0, brace_hinge: 0.0}):
        rear_foot_rest = _aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot"))
        brace_head_rest = _aabb_center(
            ctx.part_element_world_aabb(brace_strut, elem="brace_lock_head")
        )

    with ctx.pose({rear_leg_hinge: REAR_LEG_OPEN, brace_hinge: BRACE_OPEN}):
        rear_foot_open = _aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot"))
        brace_head_open = _aabb_center(
            ctx.part_element_world_aabb(brace_strut, elem="brace_lock_head")
        )
        brace_lug_open = _aabb_center(ctx.part_element_world_aabb(rear_leg, elem="brace_lug"))

    ctx.check(
        "rear leg opens backward",
        rear_foot_rest is not None
        and rear_foot_open is not None
        and rear_foot_open[0] < rear_foot_rest[0] - 0.55
        and rear_foot_open[2] > rear_foot_rest[2] + 0.15,
        details=f"rear_foot_rest={rear_foot_rest}, rear_foot_open={rear_foot_open}",
    )
    ctx.check(
        "brace swings out toward rear leg",
        brace_head_rest is not None
        and brace_head_open is not None
        and brace_head_open[0] < brace_head_rest[0] - 0.55
        and brace_head_open[2] > brace_head_rest[2] + 0.20,
        details=f"brace_head_rest={brace_head_rest}, brace_head_open={brace_head_open}",
    )
    ctx.check(
        "brace lock aligns with rear leg lug when opened",
        brace_head_open is not None
        and brace_lug_open is not None
        and abs(brace_head_open[1] - brace_lug_open[1]) <= 0.03
        and math.hypot(
            brace_head_open[0] - brace_lug_open[0],
            brace_head_open[2] - brace_lug_open[2],
        )
        <= 0.09,
        details=f"brace_head_open={brace_head_open}, brace_lug_open={brace_lug_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
