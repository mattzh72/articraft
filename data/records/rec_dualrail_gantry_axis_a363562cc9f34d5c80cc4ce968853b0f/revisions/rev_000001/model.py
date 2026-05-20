from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_LENGTH = 1.40
BASE_RAIL_CENTER_Y = 0.27
BASE_SIDE_BEAM_WIDTH = 0.11
BASE_SIDE_BEAM_HEIGHT = 0.09
BASE_CROSS_MEMBER_LENGTH = 0.20
BASE_CROSS_MEMBER_WIDTH = 0.70
BASE_CROSS_MEMBER_HEIGHT = 0.08
BASE_FOOT_LENGTH = 0.16
BASE_FOOT_WIDTH = 0.12
BASE_FOOT_HEIGHT = 0.02

RAIL_LENGTH = 1.24
RAIL_WIDTH = 0.03
RAIL_HEIGHT = 0.018
BASE_RAIL_TOP_Z = BASE_SIDE_BEAM_HEIGHT + RAIL_HEIGHT

BRIDGE_TRUCK_LENGTH = 0.18
BRIDGE_TRUCK_WIDTH = 0.09
BRIDGE_TRUCK_HEIGHT = 0.064
BRIDGE_BEAM_LENGTH = 0.19
BRIDGE_BEAM_WIDTH = 0.68
BRIDGE_BEAM_HEIGHT = 0.09
BRIDGE_BEAM_CENTER_Z = 0.135
BRIDGE_CHEEK_LENGTH = 0.17
BRIDGE_CHEEK_WIDTH = 0.055
BRIDGE_CHEEK_HEIGHT = 0.11
BRIDGE_CHEEK_CENTER_Z = 0.086
BRIDGE_RAIL_LENGTH = 0.36
BRIDGE_RAIL_WIDTH = 0.03
BRIDGE_RAIL_HEIGHT = 0.018
BRIDGE_RAIL_CENTER_X = 0.045
BRIDGE_RAIL_CENTER_Z = (
    BRIDGE_BEAM_CENTER_Z + (BRIDGE_BEAM_HEIGHT / 2.0) + (BRIDGE_RAIL_HEIGHT / 2.0)
)
BRIDGE_RAIL_TOP_Z = BRIDGE_RAIL_CENTER_Z + (BRIDGE_RAIL_HEIGHT / 2.0)

TOOL_SADDLE_LENGTH = 0.22
TOOL_SADDLE_WIDTH = 0.12
TOOL_SADDLE_HEIGHT = 0.056
TOOL_SADDLE_CENTER_X = 0.04
TOOL_BODY_LENGTH = 0.10
TOOL_BODY_WIDTH = 0.09
TOOL_BODY_HEIGHT = 0.12
TOOL_BODY_CENTER_Z = -0.052
TOOL_BODY_CENTER_X = 0.147
TOOL_FLANGE_RADIUS = 0.026
TOOL_FLANGE_LENGTH = 0.024
TOOL_FLANGE_CENTER_Z = -0.128
TOOL_NOSE_RADIUS = 0.014
TOOL_NOSE_LENGTH = 0.075
TOOL_NOSE_CENTER_Z = -0.1775

BRIDGE_TRAVEL = 0.42
TOOL_TRAVEL = 0.12


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _build_base_frame_shape() -> cq.Workplane:
    frame = _box_shape(
        (BASE_LENGTH, BASE_SIDE_BEAM_WIDTH, BASE_SIDE_BEAM_HEIGHT),
        (0.0, BASE_RAIL_CENTER_Y, BASE_SIDE_BEAM_HEIGHT / 2.0),
    )
    frame = frame.union(
        _box_shape(
            (BASE_LENGTH, BASE_SIDE_BEAM_WIDTH, BASE_SIDE_BEAM_HEIGHT),
            (0.0, -BASE_RAIL_CENTER_Y, BASE_SIDE_BEAM_HEIGHT / 2.0),
        )
    )

    for x in (-0.54, 0.54):
        frame = frame.union(
            _box_shape(
                (
                    BASE_CROSS_MEMBER_LENGTH,
                    BASE_CROSS_MEMBER_WIDTH,
                    BASE_CROSS_MEMBER_HEIGHT,
                ),
                (x, 0.0, BASE_CROSS_MEMBER_HEIGHT / 2.0),
            )
        )

    for x in (-0.56, 0.56):
        for y in (-BASE_RAIL_CENTER_Y, BASE_RAIL_CENTER_Y):
            frame = frame.union(
                _box_shape(
                    (BASE_FOOT_LENGTH, BASE_FOOT_WIDTH, BASE_FOOT_HEIGHT),
                    (x, y, BASE_FOOT_HEIGHT / 2.0),
                )
            )

    return frame


def _build_bridge_structure_shape() -> cq.Workplane:
    structure = _box_shape(
        (BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_WIDTH, BRIDGE_BEAM_HEIGHT),
        (0.0, 0.0, BRIDGE_BEAM_CENTER_Z),
    )
    structure = structure.union(
        _box_shape(
            (BRIDGE_CHEEK_LENGTH, BRIDGE_CHEEK_WIDTH, BRIDGE_CHEEK_HEIGHT),
            (0.0, BASE_RAIL_CENTER_Y, BRIDGE_CHEEK_CENTER_Z),
        )
    )
    structure = structure.union(
        _box_shape(
            (BRIDGE_CHEEK_LENGTH, BRIDGE_CHEEK_WIDTH, BRIDGE_CHEEK_HEIGHT),
            (0.0, -BASE_RAIL_CENTER_Y, BRIDGE_CHEEK_CENTER_Z),
        )
    )
    return structure


def _build_tool_body_shape() -> cq.Workplane:
    body = _box_shape(
        (TOOL_BODY_LENGTH, TOOL_BODY_WIDTH, TOOL_BODY_HEIGHT),
        (0.0, 0.0, TOOL_BODY_CENTER_Z),
    )
    body = body.union(_box_shape((0.08, 0.065, 0.03), (0.0, 0.0, -0.118)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry")

    model.material("frame_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rail_steel", rgba=(0.33, 0.35, 0.39, 1.0))
    model.material("bridge_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("truck_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("tool_light", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("tool_steel", rgba=(0.48, 0.50, 0.54, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "gantry_base_frame"),
        material="frame_gray",
        name="base_frame",
    )
    base.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, BASE_RAIL_CENTER_Y, BASE_SIDE_BEAM_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="rail_left",
    )
    base.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -BASE_RAIL_CENTER_Y, BASE_SIDE_BEAM_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="rail_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_CROSS_MEMBER_WIDTH, BASE_RAIL_TOP_Z)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_build_bridge_structure_shape(), "gantry_bridge_structure"),
        material="bridge_dark",
        name="bridge_structure",
    )
    bridge.visual(
        Box((BRIDGE_TRUCK_LENGTH, BRIDGE_TRUCK_WIDTH, BRIDGE_TRUCK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, BASE_RAIL_CENTER_Y, BRIDGE_TRUCK_HEIGHT / 2.0)
        ),
        material="truck_black",
        name="truck_left",
    )
    bridge.visual(
        Box((BRIDGE_TRUCK_LENGTH, BRIDGE_TRUCK_WIDTH, BRIDGE_TRUCK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -BASE_RAIL_CENTER_Y, BRIDGE_TRUCK_HEIGHT / 2.0)
        ),
        material="truck_black",
        name="truck_right",
    )
    bridge.visual(
        Box((BRIDGE_RAIL_WIDTH, BRIDGE_RAIL_LENGTH, BRIDGE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(BRIDGE_RAIL_CENTER_X, 0.0, BRIDGE_RAIL_CENTER_Z)
        ),
        material="rail_steel",
        name="bridge_rail_front",
    )
    bridge.visual(
        Box((BRIDGE_RAIL_WIDTH, BRIDGE_RAIL_LENGTH, BRIDGE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(-BRIDGE_RAIL_CENTER_X, 0.0, BRIDGE_RAIL_CENTER_Z)
        ),
        material="rail_steel",
        name="bridge_rail_rear",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_WIDTH, BRIDGE_RAIL_TOP_Z)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_RAIL_TOP_Z / 2.0)),
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((TOOL_SADDLE_LENGTH, TOOL_SADDLE_WIDTH, TOOL_SADDLE_HEIGHT)),
        origin=Origin(xyz=(TOOL_SADDLE_CENTER_X, 0.0, TOOL_SADDLE_HEIGHT / 2.0)),
        material="tool_light",
        name="tool_saddle",
    )
    tool_slide.visual(
        mesh_from_cadquery(_build_tool_body_shape(), "gantry_tool_body"),
        origin=Origin(xyz=(TOOL_BODY_CENTER_X, 0.0, 0.0)),
        material="tool_light",
        name="tool_body",
    )
    tool_slide.visual(
        Cylinder(radius=TOOL_FLANGE_RADIUS, length=TOOL_FLANGE_LENGTH),
        origin=Origin(xyz=(TOOL_BODY_CENTER_X, 0.0, TOOL_FLANGE_CENTER_Z)),
        material="tool_steel",
        name="tool_flange",
    )
    tool_slide.visual(
        Cylinder(radius=TOOL_NOSE_RADIUS, length=TOOL_NOSE_LENGTH),
        origin=Origin(xyz=(TOOL_BODY_CENTER_X, 0.0, TOOL_NOSE_CENTER_Z)),
        material="tool_steel",
        name="tool_nose",
    )
    tool_slide.inertial = Inertial.from_geometry(
        Box((TOOL_SADDLE_LENGTH, TOOL_SADDLE_WIDTH, 0.24)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=900.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "bridge_to_tool",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=tool_slide,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TOOL_TRAVEL,
            upper=TOOL_TRAVEL,
            effort=220.0,
            velocity=0.25,
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

    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    tool_slide = object_model.get_part("tool_slide")
    base_to_bridge = object_model.get_articulation("base_to_bridge")
    bridge_to_tool = object_model.get_articulation("bridge_to_tool")

    rail_left = base.get_visual("rail_left")
    rail_right = base.get_visual("rail_right")
    truck_left = bridge.get_visual("truck_left")
    truck_right = bridge.get_visual("truck_right")
    bridge_rail_front = bridge.get_visual("bridge_rail_front")
    bridge_rail_rear = bridge.get_visual("bridge_rail_rear")
    tool_saddle = tool_slide.get_visual("tool_saddle")

    ctx.check(
        "bridge prismatic axis runs along base length",
        tuple(base_to_bridge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={base_to_bridge.axis}",
    )
    ctx.check(
        "tool slide prismatic axis runs across bridge",
        tuple(bridge_to_tool.axis) == (0.0, 1.0, 0.0),
        details=f"axis={bridge_to_tool.axis}",
    )

    with ctx.pose({base_to_bridge: 0.0, bridge_to_tool: 0.0}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=truck_left,
            elem_b=rail_left,
            name="left truck sits on left rail",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=truck_right,
            elem_b=rail_right,
            name="right truck sits on right rail",
        )
        ctx.expect_contact(
            tool_slide,
            bridge,
            elem_a=tool_saddle,
            elem_b=bridge_rail_front,
            name="tool saddle sits on front bridge rail",
        )
        ctx.expect_contact(
            tool_slide,
            bridge,
            elem_a=tool_saddle,
            elem_b=bridge_rail_rear,
            name="tool saddle sits on rear bridge rail",
        )
        bridge_rest = ctx.part_world_position(bridge)
        tool_rest = ctx.part_world_position(tool_slide)

    with ctx.pose({base_to_bridge: BRIDGE_TRAVEL}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a=truck_left,
            elem_b=rail_left,
            min_overlap=0.08,
            name="left truck retains insertion at max bridge travel",
        )
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a=truck_right,
            elem_b=rail_right,
            min_overlap=0.08,
            name="right truck retains insertion at max bridge travel",
        )
        bridge_extended = ctx.part_world_position(bridge)

    ctx.check(
        "positive bridge travel moves bridge along +x",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.20,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )

    with ctx.pose({bridge_to_tool: TOOL_TRAVEL}):
        ctx.expect_overlap(
            tool_slide,
            bridge,
            axes="y",
            elem_a=tool_saddle,
            elem_b=bridge_rail_front,
            min_overlap=0.08,
            name="tool saddle retains engagement on front bridge rail",
        )
        ctx.expect_overlap(
            tool_slide,
            bridge,
            axes="y",
            elem_a=tool_saddle,
            elem_b=bridge_rail_rear,
            min_overlap=0.08,
            name="tool saddle retains engagement on rear bridge rail",
        )
        tool_extended = ctx.part_world_position(tool_slide)

    ctx.check(
        "positive tool travel moves saddle across bridge along +y",
        tool_rest is not None
        and tool_extended is not None
        and tool_extended[1] > tool_rest[1] + 0.06,
        details=f"rest={tool_rest}, extended={tool_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
