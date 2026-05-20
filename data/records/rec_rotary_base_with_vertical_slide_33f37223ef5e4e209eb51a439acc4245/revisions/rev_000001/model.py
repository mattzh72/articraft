from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


PLATE_HEIGHT = 0.72
PLATE_WIDTH = 0.34
PLATE_THICKNESS = 0.018

FOOT_DEPTH = 0.11
FOOT_WIDTH = 0.18
FOOT_HEIGHT = 0.028

GUIDE_Y = 0.06
GUIDE_DEPTH = 0.024
GUIDE_WIDTH = 0.072
GUIDE_HEIGHT = 0.46
GUIDE_Z0 = 0.20
PLATE_FRONT_X = PLATE_THICKNESS / 2.0
GUIDE_CENTER_X = (PLATE_THICKNESS / 2.0) + (GUIDE_DEPTH / 2.0)
GUIDE_FRONT_X = PLATE_FRONT_X + GUIDE_DEPTH

ROTARY_Y = -0.075
ROTARY_Z = 0.16
ROTARY_MOUNT_DEPTH = 0.036
ROTARY_MOUNT_WIDTH = 0.118
ROTARY_MOUNT_HEIGHT = 0.094
ROTARY_ORIGIN_X = PLATE_FRONT_X + ROTARY_MOUNT_DEPTH

PRISMATIC_HOME_Z = 0.31
SLIDE_UPPER = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_rotary_column_module")

    model.material("frame_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("carriage_silver", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_HEIGHT / 2.0)),
        material="frame_gray",
        name="side_plate",
    )
    frame.visual(
        Box((FOOT_DEPTH, FOOT_WIDTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(PLATE_FRONT_X + (FOOT_DEPTH / 2.0), 0.0, FOOT_HEIGHT / 2.0)),
        material="frame_gray",
        name="base_foot",
    )
    frame.visual(
        Box((GUIDE_DEPTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_Y, GUIDE_Z0 + (GUIDE_HEIGHT / 2.0))),
        material="frame_gray",
        name="guide_rail",
    )
    frame.visual(
        Box((GUIDE_DEPTH + 0.024, 0.120, 0.040)),
        origin=Origin(
            xyz=(
                PLATE_FRONT_X + ((GUIDE_DEPTH + 0.024) / 2.0),
                GUIDE_Y,
                GUIDE_Z0 + GUIDE_HEIGHT + 0.020,
            )
        ),
        material="frame_gray",
        name="guide_cap",
    )
    frame.visual(
        Box((ROTARY_MOUNT_DEPTH, ROTARY_MOUNT_WIDTH, ROTARY_MOUNT_HEIGHT)),
        origin=Origin(
            xyz=(PLATE_FRONT_X + (ROTARY_MOUNT_DEPTH / 2.0), ROTARY_Y, ROTARY_Z)
        ),
        material="frame_gray",
        name="rotary_mount",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.12, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(0.040, 0.0, PLATE_HEIGHT / 2.0)),
    )

    rotary_base = model.part("rotary_base")
    rotary_base.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="hub",
    )
    rotary_base.visual(
        Cylinder(radius=0.064, length=0.012),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="turntable",
    )
    rotary_base.visual(
        Box((0.020, 0.040, 0.080)),
        origin=Origin(xyz=(0.048, -0.020, 0.040)),
        material="machined_aluminum",
        name="support_web",
    )
    rotary_base.visual(
        Box((0.018, 0.036, 0.120)),
        origin=Origin(xyz=(0.060, -0.030, 0.085)),
        material="machined_aluminum",
        name="riser_arm",
    )
    rotary_base.visual(
        Box((0.026, 0.060, 0.080)),
        origin=Origin(xyz=(0.076, -0.040, 0.120)),
        material="machined_aluminum",
        name="tool_frame",
    )
    rotary_base.visual(
        Box((0.012, 0.120, 0.180)),
        origin=Origin(xyz=(0.090, -0.055, 0.095)),
        material="machined_aluminum",
        name="rotary_fixture",
    )
    rotary_base.inertial = Inertial.from_geometry(
        Box((0.10, 0.14, 0.20)),
        mass=4.2,
        origin=Origin(xyz=(0.050, -0.030, 0.070)),
    )

    carriage = model.part("carriage_faceplate")
    carriage.visual(
        Box((0.036, 0.090, 0.100)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material="carriage_silver",
        name="bearing_block",
    )
    carriage.visual(
        Box((0.020, 0.056, 0.140)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material="carriage_silver",
        name="neck_block",
    )
    carriage.visual(
        Box((0.012, 0.160, 0.200)),
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        material="carriage_silver",
        name="faceplate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.074, 0.160, 0.180)),
        mass=4.5,
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_rotary",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rotary_base,
        origin=Origin(xyz=(ROTARY_ORIGIN_X, ROTARY_Y, ROTARY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.2,
            upper=1.4,
            effort=55.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(GUIDE_FRONT_X, GUIDE_Y, PRISMATIC_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_UPPER,
            effort=850.0,
            velocity=0.30,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(ax - bx) <= tol for ax, bx in zip(a, b))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotary_base = object_model.get_part("rotary_base")
    carriage = object_model.get_part("carriage_faceplate")
    rotary_joint = object_model.get_articulation("frame_to_rotary")
    slide_joint = object_model.get_articulation("frame_to_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected parts present",
        {part.name for part in object_model.parts} == {"frame", "rotary_base", "carriage_faceplate"},
        details=f"found parts: {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "rotary articulation axis and limits",
        _vec_close(rotary_joint.axis, (1.0, 0.0, 0.0))
        and rotary_joint.motion_limits is not None
        and rotary_joint.motion_limits.lower == -1.2
        and rotary_joint.motion_limits.upper == 1.4,
        details=(
            f"axis={rotary_joint.axis}, "
            f"limits={rotary_joint.motion_limits.lower if rotary_joint.motion_limits else None}, "
            f"{rotary_joint.motion_limits.upper if rotary_joint.motion_limits else None}"
        ),
    )
    ctx.check(
        "prismatic articulation axis and limits",
        _vec_close(slide_joint.axis, (0.0, 0.0, 1.0))
        and slide_joint.motion_limits is not None
        and slide_joint.motion_limits.lower == 0.0
        and slide_joint.motion_limits.upper == SLIDE_UPPER,
        details=(
            f"axis={slide_joint.axis}, "
            f"limits={slide_joint.motion_limits.lower if slide_joint.motion_limits else None}, "
            f"{slide_joint.motion_limits.upper if slide_joint.motion_limits else None}"
        ),
    )

    ctx.expect_contact(frame, rotary_base, name="rotary base seats on side-plate boss")
    ctx.expect_contact(frame, carriage, name="carriage bears on upright guide")

    with ctx.pose({slide_joint: 0.0}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: SLIDE_UPPER}):
        upper_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(frame, carriage, name="carriage stays guided at upper travel")

    if lower_pos is not None and upper_pos is not None:
        ctx.check(
            "carriage translates vertically",
            abs(upper_pos[0] - lower_pos[0]) <= 1e-6
            and abs(upper_pos[1] - lower_pos[1]) <= 1e-6
            and abs((upper_pos[2] - lower_pos[2]) - SLIDE_UPPER) <= 1e-6,
            details=f"lower={lower_pos}, upper={upper_pos}",
        )
    else:
        ctx.fail("carriage translates vertically", "missing world positions for carriage")

    with ctx.pose({rotary_joint: 0.0}):
        rotary_rest_aabb = ctx.part_element_world_aabb(rotary_base, elem="rotary_fixture")
    with ctx.pose({rotary_joint: 1.0}):
        rotary_turned_aabb = ctx.part_element_world_aabb(rotary_base, elem="rotary_fixture")
        ctx.expect_contact(frame, rotary_base, name="rotary base stays seated when turned")

    if rotary_rest_aabb is not None and rotary_turned_aabb is not None:
        rest_center = _aabb_center(rotary_rest_aabb)
        turned_center = _aabb_center(rotary_turned_aabb)
        ctx.check(
            "rotary fixture swings around x-axis",
            abs(turned_center[1] - rest_center[1]) > 0.03 and abs(turned_center[2] - rest_center[2]) > 0.04,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )
    else:
        ctx.fail("rotary fixture swings around x-axis", "missing world AABB for rotary fixture")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
