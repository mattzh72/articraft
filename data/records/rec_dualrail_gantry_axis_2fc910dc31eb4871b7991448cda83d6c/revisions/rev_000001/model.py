from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_LENGTH = 0.72
FRAME_WIDTH = 0.43
FRAME_HEIGHT = 0.29
TRACK_Y = 0.185
TRACK_LENGTH = 0.60
TRACK_WIDTH = 0.022
TRACK_HEIGHT = 0.010

BRIDGE_TRAVEL = 0.17
BRIDGE_TRUCK_SIZE = (0.11, 0.07, 0.06)
BRIDGE_BEAM_SIZE = (0.10, 0.31, 0.07)

SLIDE_TRAVEL = 0.055
CARRIAGE_SIZE = (0.082, 0.18, 0.16)
TOOL_FACE_SIZE = (0.008, 0.052, 0.050)


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _make_frame_body() -> cq.Workplane:
    frame = _box((0.68, 0.06, 0.28), (0.0, TRACK_Y, 0.14))
    frame = frame.union(_box((0.68, 0.06, 0.28), (0.0, -TRACK_Y, 0.14)))
    for x_sign in (-1.0, 1.0):
        frame = frame.union(_box((0.06, 0.31, 0.05), (x_sign * 0.29, 0.0, 0.05)))
        frame = frame.union(_box((0.09, 0.09, 0.018), (x_sign * 0.29, 0.18, 0.009)))
        frame = frame.union(_box((0.09, 0.09, 0.018), (x_sign * 0.29, -0.18, 0.009)))
    return frame


def _make_bridge_body() -> cq.Workplane:
    bridge = _box((0.06, 0.05, 0.10), (0.0, TRACK_Y - 0.03, 0.11))
    bridge = bridge.union(_box((0.06, 0.05, 0.10), (0.0, -TRACK_Y + 0.03, 0.11)))
    bridge = bridge.union(_box(BRIDGE_BEAM_SIZE, (0.0, 0.0, 0.195)))
    bridge = bridge.union(_box((0.028, 0.26, 0.13), (0.050, 0.0, 0.110)))
    bridge = bridge.union(_box((0.055, 0.12, 0.05), (0.0, 0.0, 0.245)))
    return bridge


def _make_cross_slide_body() -> cq.Workplane:
    slide = _box(CARRIAGE_SIZE, (0.0, 0.0, 0.0))
    slide = slide.union(_box((0.095, 0.10, 0.085), (0.082, 0.0, -0.010)))
    slide = slide.union(_box((0.040, 0.12, 0.055), (0.112, 0.0, -0.050)))
    return slide


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_gantry")

    model.material("frame_gray", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("bridge_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("rail_steel", rgba=(0.27, 0.30, 0.34, 1.0))
    model.material("slide_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("tool_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("accent_blue", rgba=(0.14, 0.38, 0.72, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(_make_frame_body(), "lower_frame_body"),
        material="frame_gray",
        name="frame_body",
    )
    lower_frame.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, TRACK_Y, FRAME_HEIGHT - TRACK_HEIGHT / 2.0)),
        material="rail_steel",
        name="left_track",
    )
    lower_frame.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, -TRACK_Y, FRAME_HEIGHT - TRACK_HEIGHT / 2.0)),
        material="rail_steel",
        name="right_track",
    )
    lower_frame.inertial = Inertial.from_geometry(
        Box((FRAME_LENGTH, FRAME_WIDTH, FRAME_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    moving_bridge = model.part("moving_bridge")
    moving_bridge.visual(
        mesh_from_cadquery(_make_bridge_body(), "moving_bridge_body"),
        material="bridge_gray",
        name="bridge_body",
    )
    moving_bridge.visual(
        Box(BRIDGE_TRUCK_SIZE),
        origin=Origin(xyz=(0.0, TRACK_Y, BRIDGE_TRUCK_SIZE[2] / 2.0)),
        material="rail_steel",
        name="left_truck",
    )
    moving_bridge.visual(
        Box(BRIDGE_TRUCK_SIZE),
        origin=Origin(xyz=(0.0, -TRACK_Y, BRIDGE_TRUCK_SIZE[2] / 2.0)),
        material="rail_steel",
        name="right_truck",
    )
    moving_bridge.visual(
        Box((0.014, 0.24, 0.018)),
        origin=Origin(xyz=(0.070, 0.0, 0.135)),
        material="rail_steel",
        name="upper_slide_rail",
    )
    moving_bridge.visual(
        Box((0.014, 0.24, 0.018)),
        origin=Origin(xyz=(0.070, 0.0, 0.055)),
        material="rail_steel",
        name="lower_slide_rail",
    )
    moving_bridge.inertial = Inertial.from_geometry(
        Box((0.12, 0.44, 0.25)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        mesh_from_cadquery(_make_cross_slide_body(), "cross_slide_body"),
        material="slide_gray",
        name="carriage_body",
    )
    cross_slide.visual(
        Box(TOOL_FACE_SIZE),
        origin=Origin(xyz=(0.136, 0.0, -0.050)),
        material="accent_blue",
        name="tool_face",
    )
    cross_slide.inertial = Inertial.from_geometry(
        Box((0.19, 0.18, 0.18)),
        mass=6.0,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_bridge",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=moving_bridge,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.55,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=moving_bridge,
        child=cross_slide,
        origin=Origin(xyz=(0.118, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=420.0,
            velocity=0.35,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    moving_bridge = object_model.get_part("moving_bridge")
    cross_slide = object_model.get_part("cross_slide")
    bridge_joint = object_model.get_articulation("frame_to_bridge")
    slide_joint = object_model.get_articulation("bridge_to_cross_slide")

    def _extents(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (max_x - min_x, max_y - min_y, max_z - min_z)

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

    ctx.expect_contact(
        moving_bridge,
        lower_frame,
        elem_a="left_truck",
        elem_b="left_track",
        name="left bridge truck is seated on the left track",
    )
    ctx.expect_contact(
        moving_bridge,
        lower_frame,
        elem_a="right_truck",
        elem_b="right_track",
        name="right bridge truck is seated on the right track",
    )
    ctx.expect_contact(
        cross_slide,
        moving_bridge,
        elem_a="carriage_body",
        elem_b="upper_slide_rail",
        name="cross-slide carriage bears on the upper bridge rail",
    )
    ctx.expect_contact(
        cross_slide,
        moving_bridge,
        elem_a="carriage_body",
        elem_b="lower_slide_rail",
        name="cross-slide carriage bears on the lower bridge rail",
    )

    ctx.check(
        "both motion axes are prismatic",
        bridge_joint.joint_type == ArticulationType.PRISMATIC
        and slide_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"bridge={bridge_joint.joint_type}, slide={slide_joint.joint_type}",
    )
    axis_dot = sum(a * b for a, b in zip(bridge_joint.axis, slide_joint.axis))
    ctx.check(
        "bridge and cross-slide axes are orthogonal",
        abs(axis_dot) < 1e-6,
        details=f"bridge_axis={bridge_joint.axis}, slide_axis={slide_joint.axis}, dot={axis_dot}",
    )

    rest_bridge_pos = ctx.part_world_position(moving_bridge)
    with ctx.pose({bridge_joint: bridge_joint.motion_limits.upper}):
        bridge_upper_pos = ctx.part_world_position(moving_bridge)
        ctx.expect_overlap(
            moving_bridge,
            lower_frame,
            axes="x",
            elem_a="left_truck",
            elem_b="left_track",
            min_overlap=0.10,
            name="left truck retains track overlap at max bridge travel",
        )
        ctx.expect_overlap(
            moving_bridge,
            lower_frame,
            axes="x",
            elem_a="right_truck",
            elem_b="right_track",
            min_overlap=0.10,
            name="right truck retains track overlap at max bridge travel",
        )
    ctx.check(
        "bridge translates along +x",
        rest_bridge_pos is not None
        and bridge_upper_pos is not None
        and bridge_upper_pos[0] > rest_bridge_pos[0] + 0.10
        and abs(bridge_upper_pos[1] - rest_bridge_pos[1]) < 1e-6
        and abs(bridge_upper_pos[2] - rest_bridge_pos[2]) < 1e-6,
        details=f"rest={rest_bridge_pos}, upper={bridge_upper_pos}",
    )

    rest_slide_pos = ctx.part_world_position(cross_slide)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        slide_upper_pos = ctx.part_world_position(cross_slide)
        ctx.expect_overlap(
            cross_slide,
            moving_bridge,
            axes="y",
            elem_a="carriage_body",
            elem_b="upper_slide_rail",
            min_overlap=0.12,
            name="cross-slide carriage remains engaged on the upper rail at max travel",
        )
        ctx.expect_overlap(
            cross_slide,
            moving_bridge,
            axes="y",
            elem_a="carriage_body",
            elem_b="lower_slide_rail",
            min_overlap=0.12,
            name="cross-slide carriage remains engaged on the lower rail at max travel",
        )
    ctx.check(
        "cross-slide translates along +y",
        rest_slide_pos is not None
        and slide_upper_pos is not None
        and slide_upper_pos[1] > rest_slide_pos[1] + 0.04
        and abs(slide_upper_pos[0] - rest_slide_pos[0]) < 1e-6
        and abs(slide_upper_pos[2] - rest_slide_pos[2]) < 1e-6,
        details=f"rest={rest_slide_pos}, upper={slide_upper_pos}",
    )

    carriage_extents = _extents(ctx.part_element_world_aabb(cross_slide, elem="carriage_body"))
    tool_face_extents = _extents(ctx.part_element_world_aabb(cross_slide, elem="tool_face"))
    ctx.check(
        "cross-slide support hardware stays visibly larger than the tool face",
        carriage_extents is not None
        and tool_face_extents is not None
        and carriage_extents[1] >= tool_face_extents[1] * 2.5
        and carriage_extents[2] >= tool_face_extents[2] * 2.5,
        details=f"carriage={carriage_extents}, tool_face={tool_face_extents}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
