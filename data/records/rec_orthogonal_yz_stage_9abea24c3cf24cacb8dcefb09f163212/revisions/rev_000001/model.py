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


WALL_HEIGHT = 1.55
WALL_WIDTH = 1.10
WALL_THICKNESS = 0.06
BASE_DEPTH = 0.28
BASE_WIDTH = 0.96
BASE_HEIGHT = 0.08

RAIL_LENGTH = 0.84
RAIL_DEPTH = 0.022
RAIL_HEIGHT = 0.04
LOWER_RAIL_Z = 0.76
UPPER_RAIL_Z = 1.08

CARRIAGE_SHOE_DEPTH = 0.03
CARRIAGE_TRAVEL = 0.56
CARRIAGE_HOME_Y = -0.28
CARRIAGE_JOINT_X = WALL_THICKNESS / 2 + RAIL_DEPTH + CARRIAGE_SHOE_DEPTH / 2
CARRIAGE_JOINT_Z = (LOWER_RAIL_Z + UPPER_RAIL_Z) / 2

VERTICAL_TRAVEL = 0.23
SLIDER_JOINT_X = 0.318
SLIDER_HOME_Z = 0.125


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _build_wall_frame() -> cq.Workplane:
    wall_panel = _box(
        (WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT),
        (0.0, 0.0, WALL_HEIGHT / 2),
    ).edges("|Z").fillet(0.012)
    base_plinth = _box(
        (BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT),
        (0.11, 0.0, BASE_HEIGHT / 2),
    ).edges("|Z").fillet(0.014)
    top_cap = _box((0.10, 0.30, 0.12), (0.02, 0.0, WALL_HEIGHT - 0.09)).edges("|Z").fillet(0.01)
    center_rib = _box((0.05, 0.18, 0.80), (0.0, 0.0, 0.82)).edges("|Z").fillet(0.008)
    return _union_all(wall_panel, base_plinth, top_cap, center_rib)


def _build_wall_rails() -> cq.Workplane:
    lower_rail = _box((RAIL_DEPTH, RAIL_LENGTH, RAIL_HEIGHT), (0.041, 0.0, LOWER_RAIL_Z)).edges("|Y").fillet(0.006)
    upper_rail = _box((RAIL_DEPTH, RAIL_LENGTH, RAIL_HEIGHT), (0.041, 0.0, UPPER_RAIL_Z)).edges("|Y").fillet(0.006)

    stop_blocks: list[cq.Workplane] = []
    for rail_z in (LOWER_RAIL_Z, UPPER_RAIL_Z):
        for rail_y in (-RAIL_LENGTH / 2, RAIL_LENGTH / 2):
            stop_blocks.append(_box((0.03, 0.045, 0.065), (0.055, rail_y, rail_z)).edges("|Z").fillet(0.004))

    return _union_all(lower_rail, upper_rail, *stop_blocks)


def _build_carriage_shoes() -> cq.Workplane:
    lower_shoe = _box((0.03, 0.12, 0.07), (0.0, 0.0, -0.16)).edges("|Y").fillet(0.005)
    upper_shoe = _box((0.03, 0.12, 0.07), (0.0, 0.0, 0.16)).edges("|Y").fillet(0.005)
    return _union_all(lower_shoe, upper_shoe)


def _build_carriage_body() -> cq.Workplane:
    spine = _box((0.07, 0.12, 0.44), (0.035, 0.0, 0.0)).edges("|Z").fillet(0.01)
    beam = _box((0.20, 0.10, 0.08), (0.145, 0.0, 0.0)).edges("|Z").fillet(0.008)

    stage_body = _box((0.07, 0.12, 0.44), (0.255, 0.0, 0.0)).edges("|Z").fillet(0.008)
    stage_pocket = _box((0.022, 0.058, 0.31), (0.279, 0.0, 0.0))
    stage_body = stage_body.cut(stage_pocket)

    top_cap = _box((0.09, 0.14, 0.03), (0.255, 0.0, 0.235)).edges("|Y").fillet(0.004)
    bottom_cap = _box((0.08, 0.10, 0.03), (0.255, 0.0, -0.225)).edges("|Y").fillet(0.004)
    lower_spar = _box((0.10, 0.08, 0.06), (0.18, 0.0, -0.11)).edges("|Z").fillet(0.006)

    left_guide = _box((0.008, 0.018, 0.30), (0.294, -0.036, 0.0))
    right_guide = _box((0.008, 0.018, 0.30), (0.294, 0.036, 0.0))

    return _union_all(
        spine,
        beam,
        stage_body,
        top_cap,
        bottom_cap,
        lower_spar,
        left_guide,
        right_guide,
    )


def _build_slider_body() -> cq.Workplane:
    carriage_block = _box((0.04, 0.095, 0.13), (0.0, 0.0, 0.0)).edges("|Z").fillet(0.006)
    nose = _box((0.055, 0.06, 0.07), (0.03, 0.0, -0.02)).edges("|Z").fillet(0.005)
    tool_plate = _box((0.10, 0.10, 0.012), (0.028, 0.0, -0.078)).edges("|Y").fillet(0.003)
    lower_nose = _box((0.04, 0.04, 0.05), (0.042, 0.0, -0.05)).edges("|Z").fillet(0.004)
    return _union_all(carriage_block, nose, tool_plate, lower_nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_yz_positioning_module")

    frame_gray = model.material("frame_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    slider_silver = model.material("slider_silver", rgba=(0.84, 0.85, 0.87, 1.0))

    wall = model.part("wall")
    wall.visual(
        mesh_from_cadquery(_build_wall_frame(), "wall_frame"),
        material=frame_gray,
        name="wall_frame",
    )
    wall.visual(
        mesh_from_cadquery(_build_wall_rails(), "wall_rails"),
        material=rail_steel,
        name="wall_rails",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shoes(), "carriage_shoes"),
        material=rail_steel,
        name="carriage_shoes",
    )
    carriage.visual(
        mesh_from_cadquery(_build_carriage_body(), "carriage_body"),
        material=carriage_gray,
        name="carriage_body",
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_build_slider_body(), "slider_body"),
        material=slider_silver,
        name="slider_body",
    )

    model.articulation(
        "wall_to_carriage",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_JOINT_X, CARRIAGE_HOME_Y, CARRIAGE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.45,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_slider",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slider,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, SLIDER_HOME_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.30,
            lower=0.0,
            upper=VERTICAL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall")
    carriage = object_model.get_part("carriage")
    slider = object_model.get_part("slider")
    lateral_slide = object_model.get_articulation("wall_to_carriage")
    vertical_slide = object_model.get_articulation("carriage_to_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_part_contains_disconnected_geometry_islands(
        name="all visible carriage and slider features are physically connected"
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "horizontal stage joint is prismatic +Y",
        lateral_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(lateral_slide.axis) == (0.0, 1.0, 0.0),
        details=f"got type={lateral_slide.articulation_type}, axis={lateral_slide.axis}",
    )
    ctx.check(
        "vertical stage joint is prismatic -Z",
        vertical_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(vertical_slide.axis) == (0.0, 0.0, -1.0),
        details=f"got type={vertical_slide.articulation_type}, axis={vertical_slide.axis}",
    )

    ctx.expect_contact(wall, carriage, name="carriage rides on wall rails")
    ctx.expect_contact(carriage, slider, name="slider is supported by carriage stage")

    with ctx.pose({lateral_slide: CARRIAGE_TRAVEL, vertical_slide: VERTICAL_TRAVEL}):
        ctx.expect_contact(wall, carriage, name="carriage stays guided at full lateral reach")
        ctx.expect_contact(carriage, slider, name="slider stays guided at full downward reach")
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at full working reach")

    with ctx.pose({lateral_slide: 0.0}):
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({lateral_slide: CARRIAGE_TRAVEL}):
        carriage_far = ctx.part_world_position(carriage)

    carriage_shift_ok = (
        carriage_home is not None
        and carriage_far is not None
        and abs(carriage_far[0] - carriage_home[0]) < 1e-6
        and abs(carriage_far[1] - carriage_home[1] - CARRIAGE_TRAVEL) < 1e-6
        and abs(carriage_far[2] - carriage_home[2]) < 1e-6
    )
    ctx.check(
        "carriage motion is lateral only",
        carriage_shift_ok,
        details=f"home={carriage_home}, shifted={carriage_far}",
    )

    with ctx.pose({vertical_slide: 0.0}):
        slider_home = ctx.part_world_position(slider)
    with ctx.pose({vertical_slide: VERTICAL_TRAVEL}):
        slider_low = ctx.part_world_position(slider)

    slider_shift_ok = (
        slider_home is not None
        and slider_low is not None
        and abs(slider_low[0] - slider_home[0]) < 1e-6
        and abs(slider_low[1] - slider_home[1]) < 1e-6
        and abs(slider_low[2] - slider_home[2] + VERTICAL_TRAVEL) < 1e-6
    )
    ctx.check(
        "slider motion is vertical only",
        slider_shift_ok,
        details=f"home={slider_home}, lowered={slider_low}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
