from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_SIDE_X = 0.27
FRAME_SIDE_WIDTH = 0.06
FRAME_SIDE_DEPTH = 0.40
FRAME_SIDE_HEIGHT = 0.24
FRAME_SIDE_CENTER_Z = 0.18

FRAME_CROSS_WIDTH = 0.48
FRAME_CROSS_DEPTH = 0.06
FRAME_CROSS_HEIGHT = 0.06
FRAME_CROSS_Z = 0.03
FRAME_CROSS_Y = 0.18

FRAME_TRAY_WIDTH = 0.34
FRAME_TRAY_DEPTH = 0.32
FRAME_TRAY_HEIGHT = 0.012
FRAME_TRAY_Z = 0.066

FRAME_CAP_WIDTH = 0.08
FRAME_CAP_DEPTH = 0.34
FRAME_CAP_HEIGHT = 0.02
FRAME_CAP_Z = 0.31

FRAME_GUIDE_X = 0.25
FRAME_GUIDE_WIDTH = 0.035
FRAME_GUIDE_DEPTH = 0.30
FRAME_GUIDE_HEIGHT = 0.01
FRAME_GUIDE_Z = 0.325
FRAME_GUIDE_TOP_Z = FRAME_GUIDE_Z + FRAME_GUIDE_HEIGHT / 2.0

BRIDGE_TRUCK_X = 0.25
BRIDGE_TRUCK_WIDTH = 0.085
BRIDGE_TRUCK_DEPTH = 0.09
BRIDGE_TRUCK_HEIGHT = 0.022

BRIDGE_RISER_WIDTH = 0.06
BRIDGE_RISER_DEPTH = 0.07
BRIDGE_RISER_HEIGHT = 0.06
BRIDGE_RISER_Z = 0.052

BRIDGE_BEAM_WIDTH = 0.47
BRIDGE_BEAM_DEPTH = 0.09
BRIDGE_BEAM_HEIGHT = 0.08
BRIDGE_BEAM_Z = 0.122

BRIDGE_PLATE_WIDTH = 0.34
BRIDGE_PLATE_DEPTH = 0.028
BRIDGE_PLATE_HEIGHT = 0.12
BRIDGE_PLATE_Y = -0.059
BRIDGE_PLATE_Z = 0.103

BRIDGE_RAIL_WIDTH = 0.26
BRIDGE_RAIL_DEPTH = 0.012
BRIDGE_RAIL_HEIGHT = 0.018
BRIDGE_RAIL_Y = -0.079
BRIDGE_RAIL_UPPER_Z = 0.132
BRIDGE_RAIL_LOWER_Z = 0.068

BRIDGE_COVER_WIDTH = 0.16
BRIDGE_COVER_DEPTH = 0.06
BRIDGE_COVER_HEIGHT = 0.04
BRIDGE_COVER_Z = 0.182

BRIDGE_TRAVEL = 0.10

CROSS_SLIDE_SHOE_WIDTH = 0.10
CROSS_SLIDE_SHOE_DEPTH = 0.016
CROSS_SLIDE_SHOE_HEIGHT = 0.026
CROSS_SLIDE_SHOE_Y = -0.008
CROSS_SLIDE_SHOE_OFFSET_Z = 0.032

CROSS_SLIDE_SUPPORT_WIDTH = 0.14
CROSS_SLIDE_SUPPORT_DEPTH = 0.028
CROSS_SLIDE_SUPPORT_HEIGHT = 0.13
CROSS_SLIDE_SUPPORT_Y = -0.03

CROSS_SLIDE_CAP_WIDTH = 0.10
CROSS_SLIDE_CAP_DEPTH = 0.045
CROSS_SLIDE_CAP_HEIGHT = 0.05
CROSS_SLIDE_CAP_Y = -0.05
CROSS_SLIDE_CAP_Z = 0.09

CROSS_SLIDE_RAM_WIDTH = 0.09
CROSS_SLIDE_RAM_DEPTH = 0.055
CROSS_SLIDE_RAM_HEIGHT = 0.08
CROSS_SLIDE_RAM_Y = -0.07
CROSS_SLIDE_RAM_Z = -0.09

CROSS_SLIDE_TOOL_CARRIER_WIDTH = 0.07
CROSS_SLIDE_TOOL_CARRIER_DEPTH = 0.045
CROSS_SLIDE_TOOL_CARRIER_HEIGHT = 0.03
CROSS_SLIDE_TOOL_CARRIER_Y = -0.078
CROSS_SLIDE_TOOL_CARRIER_Z = -0.145

CROSS_SLIDE_TOOL_FACE_WIDTH = 0.042
CROSS_SLIDE_TOOL_FACE_DEPTH = 0.026
CROSS_SLIDE_TOOL_FACE_HEIGHT = 0.008
CROSS_SLIDE_TOOL_FACE_Y = -0.078
CROSS_SLIDE_TOOL_FACE_Z = -0.164

CROSS_SLIDE_TRAVEL = 0.06

BRIDGE_TO_CROSS_SLIDE_ORIGIN = (0.0, -0.085, 0.10)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _axis_tuple(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(float(value) for value in axis)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_gantry")

    model.material("frame_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("dark_trim", rgba=(0.10, 0.11, 0.13, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("service_orange", rgba=(0.91, 0.50, 0.18, 1.0))

    lower_frame = model.part("lower_frame")
    _add_box(
        lower_frame,
        (FRAME_SIDE_WIDTH, FRAME_SIDE_DEPTH, FRAME_SIDE_HEIGHT),
        (-FRAME_SIDE_X, 0.0, FRAME_SIDE_CENTER_Z),
        "frame_charcoal",
        "left_side_frame",
    )
    _add_box(
        lower_frame,
        (FRAME_SIDE_WIDTH, FRAME_SIDE_DEPTH, FRAME_SIDE_HEIGHT),
        (FRAME_SIDE_X, 0.0, FRAME_SIDE_CENTER_Z),
        "frame_charcoal",
        "right_side_frame",
    )
    _add_box(
        lower_frame,
        (FRAME_CROSS_WIDTH, FRAME_CROSS_DEPTH, FRAME_CROSS_HEIGHT),
        (0.0, -FRAME_CROSS_Y, FRAME_CROSS_Z),
        "frame_charcoal",
        "front_crossmember",
    )
    _add_box(
        lower_frame,
        (FRAME_CROSS_WIDTH, FRAME_CROSS_DEPTH, FRAME_CROSS_HEIGHT),
        (0.0, FRAME_CROSS_Y, FRAME_CROSS_Z),
        "frame_charcoal",
        "rear_crossmember",
    )
    _add_box(
        lower_frame,
        (FRAME_TRAY_WIDTH, FRAME_TRAY_DEPTH, FRAME_TRAY_HEIGHT),
        (0.0, 0.0, FRAME_TRAY_Z),
        "dark_trim",
        "service_tray",
    )
    _add_box(
        lower_frame,
        (FRAME_CAP_WIDTH, FRAME_CAP_DEPTH, FRAME_CAP_HEIGHT),
        (-FRAME_SIDE_X, 0.0, FRAME_CAP_Z),
        "machined_aluminum",
        "left_cap",
    )
    _add_box(
        lower_frame,
        (FRAME_CAP_WIDTH, FRAME_CAP_DEPTH, FRAME_CAP_HEIGHT),
        (FRAME_SIDE_X, 0.0, FRAME_CAP_Z),
        "machined_aluminum",
        "right_cap",
    )
    _add_box(
        lower_frame,
        (FRAME_GUIDE_WIDTH, FRAME_GUIDE_DEPTH, FRAME_GUIDE_HEIGHT),
        (-FRAME_GUIDE_X, 0.0, FRAME_GUIDE_Z),
        "rail_steel",
        "left_guide_pad",
    )
    _add_box(
        lower_frame,
        (FRAME_GUIDE_WIDTH, FRAME_GUIDE_DEPTH, FRAME_GUIDE_HEIGHT),
        (FRAME_GUIDE_X, 0.0, FRAME_GUIDE_Z),
        "rail_steel",
        "right_guide_pad",
    )

    moving_bridge = model.part("moving_bridge")
    _add_box(
        moving_bridge,
        (BRIDGE_TRUCK_WIDTH, BRIDGE_TRUCK_DEPTH, BRIDGE_TRUCK_HEIGHT),
        (-BRIDGE_TRUCK_X, 0.0, BRIDGE_TRUCK_HEIGHT / 2.0),
        "dark_trim",
        "left_truck",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_TRUCK_WIDTH, BRIDGE_TRUCK_DEPTH, BRIDGE_TRUCK_HEIGHT),
        (BRIDGE_TRUCK_X, 0.0, BRIDGE_TRUCK_HEIGHT / 2.0),
        "dark_trim",
        "right_truck",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_RISER_WIDTH, BRIDGE_RISER_DEPTH, BRIDGE_RISER_HEIGHT),
        (-BRIDGE_TRUCK_X, 0.0, BRIDGE_RISER_Z),
        "machined_aluminum",
        "left_riser",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_RISER_WIDTH, BRIDGE_RISER_DEPTH, BRIDGE_RISER_HEIGHT),
        (BRIDGE_TRUCK_X, 0.0, BRIDGE_RISER_Z),
        "machined_aluminum",
        "right_riser",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_BEAM_WIDTH, BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_HEIGHT),
        (0.0, 0.0, BRIDGE_BEAM_Z),
        "machined_aluminum",
        "bridge_beam",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_PLATE_WIDTH, BRIDGE_PLATE_DEPTH, BRIDGE_PLATE_HEIGHT),
        (0.0, BRIDGE_PLATE_Y, BRIDGE_PLATE_Z),
        "machined_aluminum",
        "slide_plate",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_RAIL_WIDTH, BRIDGE_RAIL_DEPTH, BRIDGE_RAIL_HEIGHT),
        (0.0, BRIDGE_RAIL_Y, BRIDGE_RAIL_UPPER_Z),
        "rail_steel",
        "upper_slide_rail",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_RAIL_WIDTH, BRIDGE_RAIL_DEPTH, BRIDGE_RAIL_HEIGHT),
        (0.0, BRIDGE_RAIL_Y, BRIDGE_RAIL_LOWER_Z),
        "rail_steel",
        "lower_slide_rail",
    )
    _add_box(
        moving_bridge,
        (BRIDGE_COVER_WIDTH, BRIDGE_COVER_DEPTH, BRIDGE_COVER_HEIGHT),
        (0.0, 0.0, BRIDGE_COVER_Z),
        "dark_trim",
        "bridge_cover",
    )

    cross_slide = model.part("cross_slide")
    _add_box(
        cross_slide,
        (CROSS_SLIDE_SHOE_WIDTH, CROSS_SLIDE_SHOE_DEPTH, CROSS_SLIDE_SHOE_HEIGHT),
        (0.0, CROSS_SLIDE_SHOE_Y, CROSS_SLIDE_SHOE_OFFSET_Z),
        "rail_steel",
        "upper_shoe",
    )
    _add_box(
        cross_slide,
        (CROSS_SLIDE_SHOE_WIDTH, CROSS_SLIDE_SHOE_DEPTH, CROSS_SLIDE_SHOE_HEIGHT),
        (0.0, CROSS_SLIDE_SHOE_Y, -CROSS_SLIDE_SHOE_OFFSET_Z),
        "rail_steel",
        "lower_shoe",
    )
    _add_box(
        cross_slide,
        (CROSS_SLIDE_SUPPORT_WIDTH, CROSS_SLIDE_SUPPORT_DEPTH, CROSS_SLIDE_SUPPORT_HEIGHT),
        (0.0, CROSS_SLIDE_SUPPORT_Y, 0.0),
        "machined_aluminum",
        "support_plate",
    )
    _add_box(
        cross_slide,
        (CROSS_SLIDE_CAP_WIDTH, CROSS_SLIDE_CAP_DEPTH, CROSS_SLIDE_CAP_HEIGHT),
        (0.0, CROSS_SLIDE_CAP_Y, CROSS_SLIDE_CAP_Z),
        "dark_trim",
        "carriage_cap",
    )
    _add_box(
        cross_slide,
        (CROSS_SLIDE_RAM_WIDTH, CROSS_SLIDE_RAM_DEPTH, CROSS_SLIDE_RAM_HEIGHT),
        (0.0, CROSS_SLIDE_RAM_Y, CROSS_SLIDE_RAM_Z),
        "machined_aluminum",
        "short_ram",
    )
    _add_box(
        cross_slide,
        (
            CROSS_SLIDE_TOOL_CARRIER_WIDTH,
            CROSS_SLIDE_TOOL_CARRIER_DEPTH,
            CROSS_SLIDE_TOOL_CARRIER_HEIGHT,
        ),
        (0.0, CROSS_SLIDE_TOOL_CARRIER_Y, CROSS_SLIDE_TOOL_CARRIER_Z),
        "dark_trim",
        "tool_carrier",
    )
    _add_box(
        cross_slide,
        (
            CROSS_SLIDE_TOOL_FACE_WIDTH,
            CROSS_SLIDE_TOOL_FACE_DEPTH,
            CROSS_SLIDE_TOOL_FACE_HEIGHT,
        ),
        (0.0, CROSS_SLIDE_TOOL_FACE_Y, CROSS_SLIDE_TOOL_FACE_Z),
        "service_orange",
        "tool_face",
    )

    model.articulation(
        "lower_frame_to_bridge",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=moving_bridge,
        origin=Origin(xyz=(0.0, 0.0, FRAME_GUIDE_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.30,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=moving_bridge,
        child=cross_slide,
        origin=Origin(xyz=BRIDGE_TO_CROSS_SLIDE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=-CROSS_SLIDE_TRAVEL,
            upper=CROSS_SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    moving_bridge = object_model.get_part("moving_bridge")
    cross_slide = object_model.get_part("cross_slide")
    bridge_joint = object_model.get_articulation("lower_frame_to_bridge")
    cross_joint = object_model.get_articulation("bridge_to_cross_slide")

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
        "gantry_joint_types_and_axes",
        bridge_joint.articulation_type == ArticulationType.PRISMATIC
        and cross_joint.articulation_type == ArticulationType.PRISMATIC
        and _axis_tuple(bridge_joint.axis) == (0.0, 1.0, 0.0)
        and _axis_tuple(cross_joint.axis) == (1.0, 0.0, 0.0),
        "Expected orthogonal Y-then-X prismatic joints for the bridge and cross-slide.",
    )
    ctx.expect_contact(
        moving_bridge,
        lower_frame,
        name="bridge_is_supported_by_lower_frame",
    )
    ctx.expect_contact(
        cross_slide,
        moving_bridge,
        name="cross_slide_is_supported_by_bridge",
    )

    with ctx.pose({bridge_joint: bridge_joint.motion_limits.lower}):
        bridge_low = ctx.part_world_position(moving_bridge)
    with ctx.pose({bridge_joint: bridge_joint.motion_limits.upper}):
        bridge_high = ctx.part_world_position(moving_bridge)
    bridge_delta = (
        bridge_high[0] - bridge_low[0],
        bridge_high[1] - bridge_low[1],
        bridge_high[2] - bridge_low[2],
    )
    ctx.check(
        "bridge_motion_runs_along_y_only",
        bridge_delta[1] > 0.19 and abs(bridge_delta[0]) < 1e-6 and abs(bridge_delta[2]) < 1e-6,
        f"Bridge motion delta was {bridge_delta!r}; expected positive Y travel only.",
    )

    with ctx.pose({cross_joint: cross_joint.motion_limits.lower}):
        cross_low = ctx.part_world_position(cross_slide)
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        cross_high = ctx.part_world_position(cross_slide)
    cross_delta = (
        cross_high[0] - cross_low[0],
        cross_high[1] - cross_low[1],
        cross_high[2] - cross_low[2],
    )
    ctx.check(
        "cross_slide_motion_runs_along_x_only",
        cross_delta[0] > 0.11 and abs(cross_delta[1]) < 1e-6 and abs(cross_delta[2]) < 1e-6,
        f"Cross-slide motion delta was {cross_delta!r}; expected positive X travel only.",
    )

    carrier_size = _aabb_size(ctx.part_element_world_aabb(cross_slide, elem="tool_carrier"))
    tool_face_size = _aabb_size(ctx.part_element_world_aabb(cross_slide, elem="tool_face"))
    ctx.check(
        "tool_support_hardware_is_visibly_larger_than_tool_face",
        carrier_size is not None
        and tool_face_size is not None
        and carrier_size[0] >= tool_face_size[0] + 0.02
        and carrier_size[1] >= tool_face_size[1] + 0.015,
        (
            "Tool carrier must remain visibly larger than the tool face; "
            f"carrier={carrier_size!r}, tool_face={tool_face_size!r}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
