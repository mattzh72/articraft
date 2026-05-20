from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 1.20
BASE_SIDE_CENTER_X = 0.39
BASE_BEAM_WIDTH = 0.14
BASE_BEAM_HEIGHT = 0.10
BASE_CROSS_TIE_WIDTH = 0.92
BASE_CROSS_TIE_DEPTH = 0.10
BASE_CROSS_TIE_Z = 0.05

RAIL_PEDESTAL_LENGTH = 1.08
RAIL_PEDESTAL_WIDTH = 0.10
RAIL_PEDESTAL_HEIGHT = 0.12
RAIL_STRIP_LENGTH = 1.04
RAIL_STRIP_WIDTH = 0.08
RAIL_STRIP_HEIGHT = 0.03
RAIL_TOP_Z = BASE_BEAM_HEIGHT + RAIL_PEDESTAL_HEIGHT + RAIL_STRIP_HEIGHT

BRIDGE_FOOT_DEPTH = 0.18
BRIDGE_FOOT_WIDTH = 0.11
BRIDGE_FOOT_HEIGHT = 0.07
BRIDGE_UPRIGHT_WIDTH = 0.08
BRIDGE_UPRIGHT_DEPTH = 0.12
BRIDGE_UPRIGHT_HEIGHT = 0.75
BRIDGE_BEAM_LENGTH = 0.76
BRIDGE_BEAM_DEPTH = 0.18
BRIDGE_BEAM_HEIGHT = 0.16
BRIDGE_BEAM_BOTTOM_Z = BRIDGE_FOOT_HEIGHT + BRIDGE_UPRIGHT_HEIGHT
BRIDGE_BEAM_TOP_Z = BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT
BRIDGE_GUIDE_LENGTH = 0.56
BRIDGE_GUIDE_DEPTH = 0.025
BRIDGE_GUIDE_HEIGHT = 0.02
BRIDGE_GUIDE_Y = 0.045
BRIDGE_GUIDE_TOP_Z = BRIDGE_BEAM_TOP_Z + BRIDGE_GUIDE_HEIGHT

RUNNER_SHOE_LENGTH = 0.16
RUNNER_SHOE_DEPTH = BRIDGE_GUIDE_DEPTH
RUNNER_SHOE_HEIGHT = BRIDGE_GUIDE_HEIGHT
RUNNER_TOP_BLOCK_LENGTH = 0.18
RUNNER_TOP_BLOCK_DEPTH = 0.18
RUNNER_TOP_BLOCK_HEIGHT = 0.05
RUNNER_HEADER_LENGTH = 0.14
RUNNER_HEADER_DEPTH = 0.05
RUNNER_HEADER_HEIGHT = 0.08
RUNNER_GUIDE_BAR_WIDTH = 0.02
RUNNER_GUIDE_BAR_DEPTH = 0.05
RUNNER_GUIDE_BAR_HEIGHT = 0.42
RUNNER_GUIDE_BAR_CENTER_Y = 0.115
RUNNER_GUIDE_BAR_CENTER_X = 0.05
RUNNER_GUIDE_BAR_CENTER_Z = -0.21

SLIDE_CARRIAGE_WIDTH = 0.08
SLIDE_CARRIAGE_DEPTH = 0.05
SLIDE_CARRIAGE_HEIGHT = 0.20
SLIDE_FRONT_PLATE_WIDTH = 0.10
SLIDE_FRONT_PLATE_DEPTH = 0.05
SLIDE_FRONT_PLATE_HEIGHT = 0.34
SLIDE_TOOL_ARM_WIDTH = 0.06
SLIDE_TOOL_ARM_DEPTH = 0.13
SLIDE_TOOL_ARM_HEIGHT = 0.06
SLIDE_TOOL_PLATE_WIDTH = 0.16
SLIDE_TOOL_PLATE_DEPTH = 0.10
SLIDE_TOOL_PLATE_HEIGHT = 0.03
SLIDE_NOSE_RADIUS = 0.02
SLIDE_NOSE_LENGTH = 0.06

BRIDGE_TRAVEL = 0.34
RUNNER_TRAVEL = 0.18
SLIDE_TRAVEL = 0.20


def _add_box(part, size, xyz, *, material: str, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, *, material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_lift_portal_module")

    model.material("frame_gray", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("rail_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("bridge_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("runner_blue", rgba=(0.18, 0.42, 0.76, 1.0))
    model.material("slide_light", rgba=(0.86, 0.87, 0.89, 1.0))
    model.material("tool_steel", rgba=(0.46, 0.48, 0.52, 1.0))

    base = model.part("base")
    _add_box(
        base,
        (BASE_BEAM_WIDTH, BASE_LENGTH, BASE_BEAM_HEIGHT),
        (-BASE_SIDE_CENTER_X, 0.0, BASE_BEAM_HEIGHT / 2.0),
        material="frame_gray",
        name="left_base_beam",
    )
    _add_box(
        base,
        (BASE_BEAM_WIDTH, BASE_LENGTH, BASE_BEAM_HEIGHT),
        (BASE_SIDE_CENTER_X, 0.0, BASE_BEAM_HEIGHT / 2.0),
        material="frame_gray",
        name="right_base_beam",
    )
    _add_box(
        base,
        (BASE_CROSS_TIE_WIDTH, BASE_CROSS_TIE_DEPTH, BASE_BEAM_HEIGHT),
        (0.0, 0.5 * BASE_LENGTH - 0.05, BASE_CROSS_TIE_Z),
        material="frame_gray",
        name="front_tie_beam",
    )
    _add_box(
        base,
        (BASE_CROSS_TIE_WIDTH, BASE_CROSS_TIE_DEPTH, BASE_BEAM_HEIGHT),
        (0.0, -0.5 * BASE_LENGTH + 0.05, BASE_CROSS_TIE_Z),
        material="frame_gray",
        name="rear_tie_beam",
    )
    _add_box(
        base,
        (RAIL_PEDESTAL_WIDTH, RAIL_PEDESTAL_LENGTH, RAIL_PEDESTAL_HEIGHT),
        (
            -BASE_SIDE_CENTER_X,
            0.0,
            BASE_BEAM_HEIGHT + RAIL_PEDESTAL_HEIGHT / 2.0,
        ),
        material="frame_gray",
        name="left_rail_pedestal",
    )
    _add_box(
        base,
        (RAIL_PEDESTAL_WIDTH, RAIL_PEDESTAL_LENGTH, RAIL_PEDESTAL_HEIGHT),
        (
            BASE_SIDE_CENTER_X,
            0.0,
            BASE_BEAM_HEIGHT + RAIL_PEDESTAL_HEIGHT / 2.0,
        ),
        material="frame_gray",
        name="right_rail_pedestal",
    )
    _add_box(
        base,
        (RAIL_STRIP_WIDTH, RAIL_STRIP_LENGTH, RAIL_STRIP_HEIGHT),
        (
            -BASE_SIDE_CENTER_X,
            0.0,
            BASE_BEAM_HEIGHT + RAIL_PEDESTAL_HEIGHT + RAIL_STRIP_HEIGHT / 2.0,
        ),
        material="rail_steel",
        name="left_rail_strip",
    )
    _add_box(
        base,
        (RAIL_STRIP_WIDTH, RAIL_STRIP_LENGTH, RAIL_STRIP_HEIGHT),
        (
            BASE_SIDE_CENTER_X,
            0.0,
            BASE_BEAM_HEIGHT + RAIL_PEDESTAL_HEIGHT + RAIL_STRIP_HEIGHT / 2.0,
        ),
        material="rail_steel",
        name="right_rail_strip",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_CROSS_TIE_WIDTH, BASE_LENGTH, RAIL_TOP_Z)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z / 2.0)),
    )

    bridge = model.part("portal_bridge")
    _add_box(
        bridge,
        (BRIDGE_FOOT_WIDTH, BRIDGE_FOOT_DEPTH, BRIDGE_FOOT_HEIGHT),
        (-BASE_SIDE_CENTER_X, 0.0, BRIDGE_FOOT_HEIGHT / 2.0),
        material="bridge_dark",
        name="left_bridge_foot",
    )
    _add_box(
        bridge,
        (BRIDGE_FOOT_WIDTH, BRIDGE_FOOT_DEPTH, BRIDGE_FOOT_HEIGHT),
        (BASE_SIDE_CENTER_X, 0.0, BRIDGE_FOOT_HEIGHT / 2.0),
        material="bridge_dark",
        name="right_bridge_foot",
    )
    _add_box(
        bridge,
        (BRIDGE_UPRIGHT_WIDTH, BRIDGE_UPRIGHT_DEPTH, BRIDGE_UPRIGHT_HEIGHT),
        (
            -BASE_SIDE_CENTER_X,
            0.0,
            BRIDGE_FOOT_HEIGHT + BRIDGE_UPRIGHT_HEIGHT / 2.0,
        ),
        material="bridge_dark",
        name="left_upright",
    )
    _add_box(
        bridge,
        (BRIDGE_UPRIGHT_WIDTH, BRIDGE_UPRIGHT_DEPTH, BRIDGE_UPRIGHT_HEIGHT),
        (
            BASE_SIDE_CENTER_X,
            0.0,
            BRIDGE_FOOT_HEIGHT + BRIDGE_UPRIGHT_HEIGHT / 2.0,
        ),
        material="bridge_dark",
        name="right_upright",
    )
    _add_box(
        bridge,
        (BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_HEIGHT),
        (
            0.0,
            0.0,
            BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT / 2.0,
        ),
        material="bridge_dark",
        name="bridge_crossbeam",
    )
    _add_box(
        bridge,
        (BRIDGE_GUIDE_LENGTH, BRIDGE_GUIDE_DEPTH, BRIDGE_GUIDE_HEIGHT),
        (
            0.0,
            -BRIDGE_GUIDE_Y,
            BRIDGE_BEAM_TOP_Z + BRIDGE_GUIDE_HEIGHT / 2.0,
        ),
        material="rail_steel",
        name="bridge_left_guide",
    )
    _add_box(
        bridge,
        (BRIDGE_GUIDE_LENGTH, BRIDGE_GUIDE_DEPTH, BRIDGE_GUIDE_HEIGHT),
        (
            0.0,
            BRIDGE_GUIDE_Y,
            BRIDGE_BEAM_TOP_Z + BRIDGE_GUIDE_HEIGHT / 2.0,
        ),
        material="rail_steel",
        name="bridge_right_guide",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.90, BRIDGE_FOOT_DEPTH, BRIDGE_GUIDE_TOP_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_GUIDE_TOP_Z / 2.0)),
    )

    runner = model.part("center_runner")
    _add_box(
        runner,
        (RUNNER_SHOE_LENGTH, RUNNER_SHOE_DEPTH, RUNNER_SHOE_HEIGHT),
        (0.0, -BRIDGE_GUIDE_Y, RUNNER_SHOE_HEIGHT / 2.0),
        material="runner_blue",
        name="runner_left_shoe",
    )
    _add_box(
        runner,
        (RUNNER_SHOE_LENGTH, RUNNER_SHOE_DEPTH, RUNNER_SHOE_HEIGHT),
        (0.0, BRIDGE_GUIDE_Y, RUNNER_SHOE_HEIGHT / 2.0),
        material="runner_blue",
        name="runner_right_shoe",
    )
    _add_box(
        runner,
        (RUNNER_TOP_BLOCK_LENGTH, RUNNER_TOP_BLOCK_DEPTH, RUNNER_TOP_BLOCK_HEIGHT),
        (
            0.0,
            0.0,
            RUNNER_TOP_BLOCK_HEIGHT / 2.0,
        ),
        material="runner_blue",
        name="runner_top_block",
    )
    _add_box(
        runner,
        (RUNNER_HEADER_LENGTH, RUNNER_HEADER_DEPTH, RUNNER_HEADER_HEIGHT),
        (
            0.0,
            RUNNER_GUIDE_BAR_CENTER_Y,
            -RUNNER_HEADER_HEIGHT / 2.0,
        ),
        material="runner_blue",
        name="runner_header",
    )
    _add_box(
        runner,
        (RUNNER_GUIDE_BAR_WIDTH, RUNNER_GUIDE_BAR_DEPTH, RUNNER_GUIDE_BAR_HEIGHT),
        (
            -RUNNER_GUIDE_BAR_CENTER_X,
            RUNNER_GUIDE_BAR_CENTER_Y,
            RUNNER_GUIDE_BAR_CENTER_Z,
        ),
        material="rail_steel",
        name="left_slide_guide",
    )
    _add_box(
        runner,
        (RUNNER_GUIDE_BAR_WIDTH, RUNNER_GUIDE_BAR_DEPTH, RUNNER_GUIDE_BAR_HEIGHT),
        (
            RUNNER_GUIDE_BAR_CENTER_X,
            RUNNER_GUIDE_BAR_CENTER_Y,
            RUNNER_GUIDE_BAR_CENTER_Z,
        ),
        material="rail_steel",
        name="right_slide_guide",
    )
    runner.inertial = Inertial.from_geometry(
        Box((RUNNER_TOP_BLOCK_LENGTH, RUNNER_TOP_BLOCK_DEPTH, 0.52)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    slide = model.part("lift_slide")
    _add_box(
        slide,
        (SLIDE_CARRIAGE_WIDTH, SLIDE_CARRIAGE_DEPTH, SLIDE_CARRIAGE_HEIGHT),
        (0.0, 0.0, -0.18),
        material="slide_light",
        name="slide_carriage",
    )
    _add_box(
        slide,
        (
            SLIDE_FRONT_PLATE_WIDTH,
            SLIDE_FRONT_PLATE_DEPTH,
            SLIDE_FRONT_PLATE_HEIGHT,
        ),
        (
            0.0,
            SLIDE_CARRIAGE_DEPTH / 2.0 + SLIDE_FRONT_PLATE_DEPTH / 2.0,
            -0.20,
        ),
        material="slide_light",
        name="slide_front_plate",
    )
    _add_box(
        slide,
        (SLIDE_TOOL_ARM_WIDTH, SLIDE_TOOL_ARM_DEPTH, SLIDE_TOOL_ARM_HEIGHT),
        (
            0.0,
            0.14,
            -0.385,
        ),
        material="tool_steel",
        name="tool_arm",
    )
    _add_box(
        slide,
        (SLIDE_TOOL_PLATE_WIDTH, SLIDE_TOOL_PLATE_DEPTH, SLIDE_TOOL_PLATE_HEIGHT),
        (
            0.0,
            0.205,
            -0.385,
        ),
        material="tool_steel",
        name="tool_plate",
    )
    _add_cylinder(
        slide,
        SLIDE_NOSE_RADIUS,
        SLIDE_NOSE_LENGTH,
        (
            0.0,
            0.205,
            -0.43,
        ),
        material="tool_steel",
        name="tool_nose",
    )
    slide.inertial = Inertial.from_geometry(
        Box((SLIDE_TOOL_PLATE_WIDTH, 0.26, 0.45)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.10, -0.22)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=850.0,
            velocity=0.7,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_runner",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=runner,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=420.0,
            velocity=0.5,
            lower=-RUNNER_TRAVEL,
            upper=RUNNER_TRAVEL,
        ),
    )
    model.articulation(
        "runner_to_lift_slide",
        ArticulationType.PRISMATIC,
        parent=runner,
        child=slide,
        origin=Origin(xyz=(0.0, RUNNER_GUIDE_BAR_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
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
    bridge = object_model.get_part("portal_bridge")
    runner = object_model.get_part("center_runner")
    slide = object_model.get_part("lift_slide")

    base_to_bridge = object_model.get_articulation("base_to_bridge")
    bridge_to_runner = object_model.get_articulation("bridge_to_runner")
    runner_to_slide = object_model.get_articulation("runner_to_lift_slide")

    ctx.expect_contact(
        bridge,
        base,
        name="bridge sits on the grounded side rails",
    )
    ctx.expect_contact(
        runner,
        bridge,
        name="runner sits on the portal bridge guides",
    )
    ctx.expect_contact(
        slide,
        runner,
        name="lift slide remains guided by the center runner",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({base_to_bridge: BRIDGE_TRAVEL}):
        ctx.expect_contact(
            bridge,
            base,
            name="bridge stays supported at full bridge travel",
        )
        bridge_extended = ctx.part_world_position(bridge)

    ctx.check(
        "bridge moves along +Y",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[1] > bridge_rest[1] + 0.25,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )

    runner_rest = ctx.part_world_position(runner)
    with ctx.pose({bridge_to_runner: RUNNER_TRAVEL}):
        ctx.expect_contact(
            runner,
            bridge,
            name="runner stays supported at full runner travel",
        )
        runner_extended = ctx.part_world_position(runner)

    ctx.check(
        "runner moves along +X",
        runner_rest is not None
        and runner_extended is not None
        and runner_extended[0] > runner_rest[0] + 0.12,
        details=f"rest={runner_rest}, extended={runner_extended}",
    )

    slide_rest = ctx.part_world_position(slide)
    with ctx.pose({runner_to_slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            slide,
            runner,
            name="slide stays guided at full lift stroke",
        )
        slide_extended = ctx.part_world_position(slide)

    ctx.check(
        "lift slide moves downward",
        slide_rest is not None
        and slide_extended is not None
        and slide_extended[2] < slide_rest[2] - 0.15,
        details=f"rest={slide_rest}, extended={slide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
