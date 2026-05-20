from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import uuid

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


FRAME_WIDTH = 0.60
FRAME_HEIGHT = 1.20
FRAME_DEPTH = 0.04
STILE_WIDTH = 0.06
RAIL_HEIGHT = 0.09

LOUVER_COUNT = 8
LOUVER_HEIGHT = 0.11
LOUVER_DEPTH = 0.012
PIVOT_RADIUS = 0.0045
PIVOT_LENGTH = 0.006
PIVOT_EMBED = 0.002

OPENING_WIDTH = FRAME_WIDTH - (2.0 * STILE_WIDTH)
OPENING_HEIGHT = FRAME_HEIGHT - (2.0 * RAIL_HEIGHT)
LOUVER_LENGTH = OPENING_WIDTH - (2.0 * (PIVOT_LENGTH - PIVOT_EMBED))
LOUVER_PITCH = OPENING_HEIGHT / LOUVER_COUNT
LOUVER_LIMIT = 1.15

def _louver_center_z(index: int) -> float:
    return RAIL_HEIGHT + (LOUVER_PITCH * (index + 0.5))


def _build_louver_blade_mesh(name: str):
    profile = superellipse_profile(
        LOUVER_DEPTH,
        LOUVER_HEIGHT,
        exponent=2.0,
        segments=40,
    )
    left_section = [(-LOUVER_LENGTH / 2.0, y, z) for (y, z) in profile]
    right_section = [(LOUVER_LENGTH / 2.0, y, z) for (y, z) in profile]
    blade_geometry = section_loft([left_section, right_section])
    return mesh_from_geometry(blade_geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter")

    painted_wood = model.material(
        "painted_wood",
        rgba=(0.93, 0.93, 0.89, 1.0),
    )
    pivot_metal = model.material(
        "pivot_metal",
        rgba=(0.72, 0.73, 0.75, 1.0),
    )

    blade_mesh = _build_louver_blade_mesh(f"plantation_louver_blade_{uuid.uuid4().hex}")

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                (-FRAME_WIDTH / 2.0) + (STILE_WIDTH / 2.0),
                0.0,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material=painted_wood,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                (FRAME_WIDTH / 2.0) - (STILE_WIDTH / 2.0),
                0.0,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material=painted_wood,
        name="right_stile",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
        material=painted_wood,
        name="bottom_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - (RAIL_HEIGHT / 2.0))),
        material=painted_wood,
        name="top_rail",
    )

    pivot_center_x = (LOUVER_LENGTH / 2.0) + (PIVOT_LENGTH / 2.0) - PIVOT_EMBED

    for index in range(LOUVER_COUNT):
        louver = model.part(f"louver_{index + 1}")
        louver.visual(
            blade_mesh,
            origin=Origin(),
            material=painted_wood,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_LENGTH),
            origin=Origin(
                xyz=(-pivot_center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pivot_metal,
            name="left_pivot",
        )
        louver.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_LENGTH),
            origin=Origin(
                xyz=(pivot_center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pivot_metal,
            name="right_pivot",
        )

        model.articulation(
            f"frame_to_louver_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, 0.0, _louver_center_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=-LOUVER_LIMIT,
                upper=LOUVER_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    louvers = [object_model.get_part(f"louver_{index + 1}") for index in range(LOUVER_COUNT)]
    joints = [
        object_model.get_articulation(f"frame_to_louver_{index + 1}")
        for index in range(LOUVER_COUNT)
    ]

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

    frame_aabb = ctx.part_world_aabb(frame)
    frame_size_ok = (
        frame_aabb is not None
        and abs((frame_aabb[1][0] - frame_aabb[0][0]) - FRAME_WIDTH) < 1e-6
        and abs((frame_aabb[1][2] - frame_aabb[0][2]) - FRAME_HEIGHT) < 1e-6
    )
    ctx.check(
        "frame_overall_size",
        frame_size_ok,
        details=(
            f"Expected frame size {(FRAME_WIDTH, FRAME_HEIGHT)} "
            f"but got {frame_aabb}."
        ),
    )

    for index, (louver, joint) in enumerate(zip(louvers, joints), start=1):
        ctx.check(
            f"louver_{index}_exists",
            louver is not None,
            details=f"Missing louver_{index}.",
        )
        axis = tuple(joint.axis)
        axis_ok = (
            len(axis) == 3
            and abs(axis[0] - 1.0) < 1e-9
            and abs(axis[1]) < 1e-9
            and abs(axis[2]) < 1e-9
        )
        ctx.check(
            f"frame_to_louver_{index}_axis",
            axis_ok,
            details=f"Expected x-axis rotation for louver_{index}, got {axis}.",
        )

        limits = joint.motion_limits
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -1.0
            and limits.upper >= 1.0
        )
        ctx.check(
            f"frame_to_louver_{index}_limits",
            limits_ok,
            details=f"Unexpected motion limits for {joint.name}: {limits}.",
        )

        ctx.expect_contact(
            louver,
            frame,
            elem_a="left_pivot",
            elem_b="left_stile",
            name=f"louver_{index}_left_pivot_contact",
        )
        ctx.expect_contact(
            louver,
            frame,
            elem_a="right_pivot",
            elem_b="right_stile",
            name=f"louver_{index}_right_pivot_contact",
        )

    ctx.expect_gap(
        louvers[0],
        frame,
        axis="z",
        min_gap=0.006,
        max_gap=0.015,
        positive_elem="blade",
        negative_elem="bottom_rail",
        name="bottom_rail_to_first_louver_gap",
    )
    ctx.expect_gap(
        frame,
        louvers[-1],
        axis="z",
        min_gap=0.006,
        max_gap=0.015,
        positive_elem="top_rail",
        negative_elem="blade",
        name="last_louver_to_top_rail_gap",
    )

    for lower, upper in zip(louvers[:-1], louvers[1:]):
        upper_index = louvers.index(upper) + 1
        lower_index = louvers.index(lower) + 1
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.012,
            max_gap=0.025,
            positive_elem="blade",
            negative_elem="blade",
            name=f"louver_{lower_index}_to_louver_{upper_index}_blade_gap",
        )

    for index, (louver, joint) in enumerate(zip(louvers, joints), start=1):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue

        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(
                louver,
                frame,
                elem_a="left_pivot",
                elem_b="left_stile",
                name=f"louver_{index}_lower_left_pivot_contact",
            )
            ctx.expect_contact(
                louver,
                frame,
                elem_a="right_pivot",
                elem_b="right_stile",
                name=f"louver_{index}_lower_right_pivot_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"frame_to_louver_{index}_lower_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"frame_to_louver_{index}_lower_no_floating")

        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(
                louver,
                frame,
                elem_a="left_pivot",
                elem_b="left_stile",
                name=f"louver_{index}_upper_left_pivot_contact",
            )
            ctx.expect_contact(
                louver,
                frame,
                elem_a="right_pivot",
                elem_b="right_stile",
                name=f"louver_{index}_upper_right_pivot_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"frame_to_louver_{index}_upper_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"frame_to_louver_{index}_upper_no_floating")

    with ctx.pose({joint: joint.motion_limits.lower for joint in joints if joint.motion_limits is not None}):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_louvers_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="all_louvers_lower_no_floating")

    with ctx.pose({joint: joint.motion_limits.upper for joint in joints if joint.motion_limits is not None}):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_louvers_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="all_louvers_upper_no_floating")

    alternating_pose = {}
    for index, joint in enumerate(joints):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        alternating_pose[joint] = limits.upper if index % 2 == 0 else limits.lower

    with ctx.pose(alternating_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="alternating_louver_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="alternating_louver_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
