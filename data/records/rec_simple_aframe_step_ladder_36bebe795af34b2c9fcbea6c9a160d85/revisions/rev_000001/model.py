from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    tread_mat = model.material("tread", rgba=(0.82, 0.82, 0.84, 1.0))
    top_cap_mat = model.material("top_cap", rgba=(0.12, 0.12, 0.14, 1.0))

    ladder_height = 0.93
    front_foot_y = 0.25
    rear_foot_y = 0.30
    half_width = 0.225

    front_rail_size = (0.030, 0.018, math.hypot(front_foot_y, ladder_height))
    rear_rail_size = (0.026, 0.016, math.hypot(rear_foot_y, ladder_height))

    front_rail_roll = math.atan2(front_foot_y, ladder_height)
    rear_rail_roll = -math.atan2(rear_foot_y, ladder_height)

    front_frame = model.part("front_frame")
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        front_frame.visual(
            Box(front_rail_size),
            origin=Origin(
                xyz=(x_sign * half_width, front_foot_y * 0.5, ladder_height * 0.5),
                rpy=(front_rail_roll, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_front_rail",
        )

    front_frame.visual(
        Box((0.485, 0.110, 0.060)),
        origin=Origin(xyz=(0.0, 0.055, ladder_height - 0.030)),
        material=top_cap_mat,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.100, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, ladder_height - 0.030)),
        material=top_cap_mat,
        name="front_hinge_lug",
    )

    tread_width = 2.0 * (half_width - front_rail_size[0] * 0.5)
    tread_depth = 0.100
    tread_thickness = 0.030
    tread_heights = (0.23, 0.46, 0.69)
    for index, z in enumerate(tread_heights, start=1):
        y = front_foot_y * (1.0 - z / ladder_height)
        front_frame.visual(
            Box((tread_width, tread_depth, tread_thickness)),
            origin=Origin(xyz=(0.0, y, z)),
            material=tread_mat,
            name=f"tread_{index}",
        )

    rear_frame = model.part("rear_frame")
    rear_rail_x = half_width - 0.020
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        rear_frame.visual(
            Box(rear_rail_size),
            origin=Origin(
                xyz=(x_sign * rear_rail_x, -rear_foot_y * 0.5, -ladder_height * 0.5),
                rpy=(rear_rail_roll, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_rear_rail",
        )

    rear_frame.visual(
        Box((2.0 * rear_rail_x, 0.028, 0.068)),
        origin=Origin(xyz=(0.0, -0.024, -0.034)),
        material=top_cap_mat,
        name="rear_hinge_head",
    )
    rear_frame.visual(
        Box((2.0 * rear_rail_x, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, -0.180, -0.560)),
        material=aluminum,
        name="rear_lower_brace",
    )

    model.articulation(
        "front_to_rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.020, ladder_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=-0.08,
            upper=0.40,
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
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("front_to_rear_hinge")

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    tread_details = []
    tread_ok = True
    for tread_name in ("tread_1", "tread_2", "tread_3"):
        tread_aabb = ctx.part_element_world_aabb(front_frame, elem=tread_name)
        if tread_aabb is None:
            tread_ok = False
            tread_details.append(f"{tread_name}=missing")
            continue
        (min_x, min_y, min_z), (max_x, max_y, max_z) = tread_aabb
        width = max_x - min_x
        depth = max_y - min_y
        thickness = max_z - min_z
        tread_details.append(
            f"{tread_name}: width={width:.3f}, depth={depth:.3f}, thickness={thickness:.3f}"
        )
        tread_ok = tread_ok and width >= 0.38 and depth >= 0.09 and thickness >= 0.025
    ctx.check(
        "front frame carries three wide climbing treads",
        tread_ok,
        details="; ".join(tread_details),
    )

    limits = hinge.motion_limits
    ctx.check(
        "rear hinge has a usable folding range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.05
        and limits.upper >= 0.35,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        open_brace = ctx.part_element_world_aabb(rear_frame, elem="rear_lower_brace")
        low_tread = ctx.part_element_world_aabb(front_frame, elem="tread_1")

    open_brace_center = center_from_aabb(open_brace)
    low_tread_center = center_from_aabb(low_tread)
    ctx.check(
        "rear support frame opens behind the climbing side",
        open_brace_center is not None
        and low_tread_center is not None
        and open_brace_center[1] < low_tread_center[1] - 0.20,
        details=f"rear_lower_brace_center={open_brace_center}, tread_1_center={low_tread_center}",
    )

    with ctx.pose({hinge: 0.30}):
        folded_brace = ctx.part_element_world_aabb(rear_frame, elem="rear_lower_brace")

    folded_brace_center = center_from_aabb(folded_brace)
    ctx.check(
        "rear support frame swings forward when folded",
        open_brace_center is not None
        and folded_brace_center is not None
        and folded_brace_center[1] > open_brace_center[1] + 0.14,
        details=f"open_center={open_brace_center}, folded_center={folded_brace_center}",
    )

    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="front_hinge_lug",
        elem_b="rear_hinge_head",
        reason="The simplified top-hinge visuals stand in for interleaved hinge hardware around the same revolute axis.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
