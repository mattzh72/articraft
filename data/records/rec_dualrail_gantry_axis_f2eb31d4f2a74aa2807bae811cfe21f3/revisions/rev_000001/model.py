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


RAIL_LEN = 1.20
RAIL_SPACING = 0.70
RAIL_WIDTH = 0.10
RAIL_HEIGHT = 0.10
GUIDE_LEN = 1.08
GUIDE_WIDTH = 0.05
GUIDE_HEIGHT = 0.02
BRACE_LENGTH = RAIL_SPACING + RAIL_WIDTH
BRACE_THICKNESS = 0.08
BRACE_HEIGHT = 0.08
BRACE_X_OFFSET = 0.56
FOOT_LENGTH = 0.18
FOOT_WIDTH = 0.18
FOOT_HEIGHT = 0.04
FOOT_X_OFFSET = 0.50

PAD_LENGTH = 0.20
PAD_WIDTH = GUIDE_WIDTH
PAD_HEIGHT = 0.012
CARRIAGE_DEPTH = 0.20
CARRIAGE_WIDTH = 0.16
CARRIAGE_HEIGHT = 0.38
BEAM_DEPTH = 0.14
BEAM_LENGTH = RAIL_SPACING + CARRIAGE_WIDTH
BEAM_HEIGHT = 0.10
BEAM_BOTTOM_Z = PAD_HEIGHT + CARRIAGE_HEIGHT
BEAM_CENTER_Z = BEAM_BOTTOM_Z + BEAM_HEIGHT / 2.0
SLIDE_THICKNESS = 0.02
SLIDE_LENGTH = 0.62
SLIDE_HEIGHT = 0.08
SLIDE_CENTER_X = BEAM_DEPTH / 2.0 + SLIDE_THICKNESS / 2.0
SLIDE_FRONT_X = BEAM_DEPTH / 2.0 + SLIDE_THICKNESS

TOOL_CARRIAGE_DEPTH = 0.08
TOOL_CARRIAGE_WIDTH = 0.11
TOOL_CARRIAGE_HEIGHT = 0.10
TOOL_CARRIAGE_CENTER_Z = -(SLIDE_HEIGHT / 2.0 + TOOL_CARRIAGE_HEIGHT / 2.0)
MOTOR_MOUNT_DEPTH = 0.04
MOTOR_MOUNT_WIDTH = 0.07
MOTOR_MOUNT_HEIGHT = 0.04
MOTOR_MOUNT_CENTER_Z = (
    TOOL_CARRIAGE_CENTER_Z + TOOL_CARRIAGE_HEIGHT / 2.0 + MOTOR_MOUNT_HEIGHT / 2.0
)
TOOL_PLATE_DEPTH = 0.025
TOOL_PLATE_WIDTH = 0.10
TOOL_PLATE_HEIGHT = 0.28
TOOL_PLATE_CENTER_X = TOOL_CARRIAGE_DEPTH + TOOL_PLATE_DEPTH / 2.0
TOOL_PLATE_CENTER_Z = -0.22
SPINDLE_BLOCK_DEPTH = 0.04
SPINDLE_BLOCK_WIDTH = 0.07
SPINDLE_BLOCK_HEIGHT = 0.07
SPINDLE_BLOCK_CENTER_X = TOOL_CARRIAGE_DEPTH + TOOL_PLATE_DEPTH + SPINDLE_BLOCK_DEPTH / 2.0
SPINDLE_BLOCK_CENTER_Z = -0.25
TOOL_NOSE_RADIUS = 0.016
TOOL_NOSE_LENGTH = 0.16
TOOL_NOSE_CENTER_X = SPINDLE_BLOCK_CENTER_X
TOOL_NOSE_CENTER_Z = -0.365
MOTOR_RADIUS = 0.03
MOTOR_LENGTH = 0.08

BRIDGE_TRAVEL = 0.41
TOOL_TRAVEL = 0.20


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rail_body(y_center: float) -> cq.Workplane:
    rail = _box((0.0, y_center, RAIL_HEIGHT / 2.0), (RAIL_LEN, RAIL_WIDTH, RAIL_HEIGHT))
    shoulder = _box(
        (0.0, y_center, RAIL_HEIGHT - 0.012),
        (RAIL_LEN * 0.96, RAIL_WIDTH * 0.72, 0.016),
    )
    return rail.union(shoulder)


def _guide_strip(y_center: float) -> cq.Workplane:
    return _box(
        (0.0, y_center, RAIL_HEIGHT + GUIDE_HEIGHT / 2.0),
        (GUIDE_LEN, GUIDE_WIDTH, GUIDE_HEIGHT),
    )


def _base_brace(x_center: float) -> cq.Workplane:
    return _box(
        (x_center, 0.0, BRACE_HEIGHT / 2.0 + 0.04),
        (BRACE_THICKNESS, BRACE_LENGTH, BRACE_HEIGHT),
    )


def _foot_pad(x_center: float, y_center: float) -> cq.Workplane:
    return _box(
        (x_center, y_center, FOOT_HEIGHT / 2.0),
        (FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
    )


def _carriage_shell() -> cq.Workplane:
    housing = _box(
        (0.0, 0.0, PAD_HEIGHT + CARRIAGE_HEIGHT / 2.0),
        (CARRIAGE_DEPTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT),
    )
    side_window = _box(
        (0.0, 0.0, PAD_HEIGHT + CARRIAGE_HEIGHT * 0.56),
        (CARRIAGE_DEPTH + 0.004, CARRIAGE_WIDTH * 0.62, CARRIAGE_HEIGHT * 0.42),
    )
    return housing.cut(side_window)


def _tool_nose() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(TOOL_NOSE_RADIUS)
        .extrude(TOOL_NOSE_LENGTH)
        .translate(
            (
                TOOL_NOSE_CENTER_X,
                0.0,
                TOOL_NOSE_CENTER_Z - TOOL_NOSE_LENGTH / 2.0,
            )
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_machine_gantry_axis")

    base_paint = model.material("base_paint", rgba=(0.30, 0.33, 0.36, 1.0))
    guide_metal = model.material("guide_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    bridge_paint = model.material("bridge_paint", rgba=(0.93, 0.46, 0.14, 1.0))
    truck_paint = model.material("truck_paint", rgba=(0.22, 0.24, 0.28, 1.0))
    tool_metal = model.material("tool_metal", rgba=(0.60, 0.62, 0.66, 1.0))

    base = model.part("base_frame")
    y_left = -RAIL_SPACING / 2.0
    y_right = RAIL_SPACING / 2.0

    base.visual(
        mesh_from_cadquery(_rail_body(y_left), "left_rail_body"),
        material=base_paint,
        name="left_rail_body",
    )
    base.visual(
        mesh_from_cadquery(_rail_body(y_right), "right_rail_body"),
        material=base_paint,
        name="right_rail_body",
    )
    base.visual(
        mesh_from_cadquery(_guide_strip(y_left), "left_guide_strip"),
        material=guide_metal,
        name="left_guide_strip",
    )
    base.visual(
        mesh_from_cadquery(_guide_strip(y_right), "right_guide_strip"),
        material=guide_metal,
        name="right_guide_strip",
    )
    base.visual(
        mesh_from_cadquery(_base_brace(-BRACE_X_OFFSET), "front_base_brace"),
        material=base_paint,
        name="front_base_brace",
    )
    base.visual(
        mesh_from_cadquery(_base_brace(BRACE_X_OFFSET), "rear_base_brace"),
        material=base_paint,
        name="rear_base_brace",
    )
    for x_center in (-FOOT_X_OFFSET, FOOT_X_OFFSET):
        for y_center, name in (
            (y_left, f"foot_{'front' if x_center < 0 else 'rear'}_left"),
            (y_right, f"foot_{'front' if x_center < 0 else 'rear'}_right"),
        ):
            base.visual(
                mesh_from_cadquery(_foot_pad(x_center, y_center), name),
                material=base_paint,
                name=name,
            )

    bridge = model.part("bridge_carriage")
    carriage_core = _carriage_shell()
    bridge.visual(
        mesh_from_cadquery(
            carriage_core.translate((0.0, y_left, 0.0)),
            "left_bridge_carriage",
        ),
        material=bridge_paint,
        name="left_bridge_carriage",
    )
    bridge.visual(
        mesh_from_cadquery(
            carriage_core.translate((0.0, y_right, 0.0)),
            "right_bridge_carriage",
        ),
        material=bridge_paint,
        name="right_bridge_carriage",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box((0.0, 0.0, PAD_HEIGHT / 2.0), (PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT)).translate(
                (0.0, y_left, 0.0)
            ),
            "left_glide_pad",
        ),
        material=guide_metal,
        name="left_glide_pad",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box((0.0, 0.0, PAD_HEIGHT / 2.0), (PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT)).translate(
                (0.0, y_right, 0.0)
            ),
            "right_glide_pad",
        ),
        material=guide_metal,
        name="right_glide_pad",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box((0.0, 0.0, BEAM_CENTER_Z), (BEAM_DEPTH, BEAM_LENGTH, BEAM_HEIGHT)),
            "cross_beam",
        ),
        material=bridge_paint,
        name="cross_beam",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box((SLIDE_CENTER_X, 0.0, BEAM_CENTER_Z), (SLIDE_THICKNESS, SLIDE_LENGTH, SLIDE_HEIGHT)),
            "beam_slide",
        ),
        material=guide_metal,
        name="beam_slide",
    )

    truck = model.part("tool_truck")
    truck.visual(
        mesh_from_cadquery(
            _box(
                (TOOL_CARRIAGE_DEPTH / 2.0, 0.0, TOOL_CARRIAGE_CENTER_Z),
                (TOOL_CARRIAGE_DEPTH, TOOL_CARRIAGE_WIDTH, TOOL_CARRIAGE_HEIGHT),
            ),
            "carriage_body",
        ),
        material=truck_paint,
        name="carriage_body",
    )
    truck.visual(
        mesh_from_cadquery(
            _box(
                (TOOL_CARRIAGE_DEPTH / 2.0, 0.0, MOTOR_MOUNT_CENTER_Z),
                (MOTOR_MOUNT_DEPTH, MOTOR_MOUNT_WIDTH, MOTOR_MOUNT_HEIGHT),
            ),
            "motor_mount",
        ),
        material=truck_paint,
        name="motor_mount",
    )
    truck.visual(
        mesh_from_cadquery(
            _box(
                (
                    TOOL_PLATE_CENTER_X,
                    0.0,
                    TOOL_PLATE_CENTER_Z,
                ),
                (TOOL_PLATE_DEPTH, TOOL_PLATE_WIDTH, TOOL_PLATE_HEIGHT),
            ),
            "tool_plate",
        ),
        material=truck_paint,
        name="tool_plate",
    )
    truck.visual(
        mesh_from_cadquery(
            _box(
                (SPINDLE_BLOCK_CENTER_X, 0.0, SPINDLE_BLOCK_CENTER_Z),
                (SPINDLE_BLOCK_DEPTH, SPINDLE_BLOCK_WIDTH, SPINDLE_BLOCK_HEIGHT),
            ),
            "spindle_block",
        ),
        material=tool_metal,
        name="spindle_block",
    )
    truck.visual(
        mesh_from_cadquery(
            cq.Workplane("YZ")
            .circle(MOTOR_RADIUS)
            .extrude(MOTOR_LENGTH)
            .translate(
                (
                    0.0,
                    0.0,
                    MOTOR_MOUNT_CENTER_Z + MOTOR_MOUNT_HEIGHT / 2.0 + MOTOR_RADIUS - 0.002,
                )
            ),
            "motor_can",
        ),
        material=tool_metal,
        name="motor_can",
    )
    truck.visual(
        mesh_from_cadquery(_tool_nose(), "tool_nose"),
        material=tool_metal,
        name="tool_nose",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT + GUIDE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.80,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )

    model.articulation(
        "bridge_to_tool",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(SLIDE_FRONT_X, 0.0, BEAM_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=-TOOL_TRAVEL,
            upper=TOOL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge_carriage")
    truck = object_model.get_part("tool_truck")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    tool_slide = object_model.get_articulation("bridge_to_tool")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="sampled_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check("base_frame_present", base.name == "base_frame", "Missing base frame part.")
    ctx.check("bridge_carriage_present", bridge.name == "bridge_carriage", "Missing bridge carriage part.")
    ctx.check("tool_truck_present", truck.name == "tool_truck", "Missing tool truck part.")
    ctx.check(
        "bridge_prismatic_axis_along_x",
        tuple(round(v, 6) for v in bridge_slide.axis) == (1.0, 0.0, 0.0),
        f"Expected x-axis bridge motion, got {bridge_slide.axis}.",
    )
    ctx.check(
        "tool_prismatic_axis_along_y",
        tuple(round(v, 6) for v in tool_slide.axis) == (0.0, 1.0, 0.0),
        f"Expected y-axis tool motion, got {tool_slide.axis}.",
    )

    bridge_limits = bridge_slide.motion_limits
    tool_limits = tool_slide.motion_limits
    ctx.check(
        "bridge_travel_is_machine_scale",
        bridge_limits is not None
        and bridge_limits.lower is not None
        and bridge_limits.upper is not None
        and bridge_limits.lower <= -0.40
        and bridge_limits.upper >= 0.40,
        "Bridge should traverse most of the rail length.",
    )
    ctx.check(
        "tool_travel_is_machine_scale",
        tool_limits is not None
        and tool_limits.lower is not None
        and tool_limits.upper is not None
        and tool_limits.lower <= -0.19
        and tool_limits.upper >= 0.19,
        "Tool truck should travel across a meaningful portion of the beam.",
    )

    with ctx.pose({bridge_slide: 0.0, tool_slide: 0.0}):
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            positive_elem="left_glide_pad",
            negative_elem="left_guide_strip",
            max_gap=0.001,
            max_penetration=0.0,
            name="left_glide_contacts_left_guide_center",
        )
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            positive_elem="right_glide_pad",
            negative_elem="right_guide_strip",
            max_gap=0.001,
            max_penetration=0.0,
            name="right_glide_contacts_right_guide_center",
        )
        ctx.expect_gap(
            truck,
            bridge,
            axis="x",
            positive_elem="carriage_body",
            negative_elem="beam_slide",
            max_gap=0.001,
            max_penetration=0.0,
            name="truck_carriage_seats_on_beam_slide_center",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="left_glide_pad",
            outer_elem="left_guide_strip",
            margin=0.0,
            name="left_glide_within_left_guide_center",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="right_glide_pad",
            outer_elem="right_guide_strip",
            margin=0.0,
            name="right_glide_within_right_guide_center",
        )
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            inner_elem="carriage_body",
            outer_elem="beam_slide",
            margin=0.0,
            name="truck_within_beam_slide_center",
        )

    if bridge_limits is not None and bridge_limits.lower is not None and bridge_limits.upper is not None:
        for label, bridge_pos in (("lower", bridge_limits.lower), ("upper", bridge_limits.upper)):
            with ctx.pose({bridge_slide: bridge_pos, tool_slide: 0.0}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"bridge_{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"bridge_{label}_no_floating")
                ctx.expect_gap(
                    bridge,
                    base,
                    axis="z",
                    positive_elem="left_glide_pad",
                    negative_elem="left_guide_strip",
                    max_gap=0.001,
                    max_penetration=0.0,
                    name=f"left_glide_contacts_left_guide_{label}",
                )
                ctx.expect_gap(
                    bridge,
                    base,
                    axis="z",
                    positive_elem="right_glide_pad",
                    negative_elem="right_guide_strip",
                    max_gap=0.001,
                    max_penetration=0.0,
                    name=f"right_glide_contacts_right_guide_{label}",
                )
                ctx.expect_within(
                    bridge,
                    base,
                    axes="x",
                    inner_elem="left_glide_pad",
                    outer_elem="left_guide_strip",
                    margin=0.0,
                    name=f"left_glide_within_left_guide_{label}",
                )
                ctx.expect_within(
                    bridge,
                    base,
                    axes="x",
                    inner_elem="right_glide_pad",
                    outer_elem="right_guide_strip",
                    margin=0.0,
                    name=f"right_glide_within_right_guide_{label}",
                )

    if tool_limits is not None and tool_limits.lower is not None and tool_limits.upper is not None:
        for label, tool_pos in (("lower", tool_limits.lower), ("upper", tool_limits.upper)):
            with ctx.pose({bridge_slide: 0.0, tool_slide: tool_pos}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"tool_{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"tool_{label}_no_floating")
                ctx.expect_gap(
                    truck,
                    bridge,
                    axis="x",
                    positive_elem="carriage_body",
                    negative_elem="beam_slide",
                    max_gap=0.001,
                    max_penetration=0.0,
                    name=f"truck_carriage_seats_on_beam_slide_{label}",
                )
                ctx.expect_within(
                    truck,
                    bridge,
                    axes="y",
                    inner_elem="carriage_body",
                    outer_elem="beam_slide",
                    margin=0.0,
                    name=f"truck_within_beam_slide_{label}",
                )

        for bridge_pos in (bridge_limits.lower if bridge_limits else 0.0, bridge_limits.upper if bridge_limits else 0.0):
            for tool_pos in (tool_limits.lower, tool_limits.upper):
                with ctx.pose({bridge_slide: bridge_pos, tool_slide: tool_pos}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name=f"corner_pose_bridge_{bridge_pos:.3f}_tool_{tool_pos:.3f}_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(
                        name=f"corner_pose_bridge_{bridge_pos:.3f}_tool_{tool_pos:.3f}_no_floating"
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
