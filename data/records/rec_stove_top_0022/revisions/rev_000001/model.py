from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


CABINET_WIDTH = 1.02
CABINET_DEPTH = 0.58
CABINET_HEIGHT = 0.78
PANEL_THICKNESS = 0.018
FRAME_THICKNESS = 0.018
BACK_THICKNESS = 0.010
TOP_RAIL_DEPTH = 0.060

COUNTER_WIDTH = 1.26
COUNTER_DEPTH = 0.70
COUNTER_THICKNESS = 0.040
COUNTER_OPENING_WIDTH = 0.82
COUNTER_OPENING_DEPTH = 0.48
COUNTER_OPENING_Y = -0.035

COOKTOP_WIDTH = 0.84
COOKTOP_DEPTH = 0.48
COOKTOP_TOP_THICKNESS = 0.004
COOKTOP_FLOOR_THICKNESS = 0.003
COOKTOP_RECESS_FLOOR_Z = -0.016
COOKTOP_WALL_THICKNESS = 0.008
COOKTOP_CHASSIS_WIDTH = 0.800
COOKTOP_CHASSIS_DEPTH = 0.452

CONTROL_PANEL_WIDTH = 0.74
CONTROL_PANEL_HEIGHT = 0.052
CONTROL_PANEL_THICKNESS = 0.008
PANEL_SLOT_HEIGHT = 0.016

BUTTON_TRAVEL = 0.006
BUTTON_HEIGHT = 0.012
UPPER_BUTTON_WIDTH = 0.066
LOWER_BUTTON_WIDTH = 0.076

DOOR_THICKNESS = 0.018
DOOR_GAP = 0.004
DOOR_BOTTOM = 0.070
DOOR_TOP = 0.690
DOOR_HEIGHT = DOOR_TOP - DOOR_BOTTOM
DOOR_WIDTH = (CABINET_WIDTH - (3.0 * DOOR_GAP)) / 2.0
DOOR_SIDE_CLEARANCE = 0.016
DOOR_LEAF_WIDTH = DOOR_WIDTH - DOOR_SIDE_CLEARANCE
HINGE_RADIUS = 0.008
HINGE_BARREL_RADIUS = 0.011


def _rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0):
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 32,
):
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _add_row_fillers(
    part,
    *,
    slots,
    z_center: float,
    row_height: float,
    depth: float,
    material,
    prefix: str,
):
    half_width = CONTROL_PANEL_WIDTH * 0.5
    cursor = -half_width
    segment_index = 0
    for x_center, width in sorted(slots):
        left_edge = x_center - width * 0.5
        if left_edge > cursor:
            span_width = left_edge - cursor
            part.visual(
                Box((span_width, depth, row_height)),
                origin=Origin(xyz=((cursor + left_edge) * 0.5, 0.0, z_center)),
                material=material,
                name=f"{prefix}_{segment_index}",
            )
            segment_index += 1
        cursor = x_center + width * 0.5
    if cursor < half_width:
        span_width = half_width - cursor
        part.visual(
            Box((span_width, depth, row_height)),
            origin=Origin(xyz=((cursor + half_width) * 0.5, 0.0, z_center)),
            material=material,
            name=f"{prefix}_{segment_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_island_cabinet", assets=ASSETS)

    painted_charcoal = model.material("painted_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.11, 0.12, 0.95))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.24, 1.0))
    button_grey = model.material("button_grey", rgba=(0.70, 0.71, 0.73, 1.0))
    stone = model.material("stone", rgba=(0.84, 0.83, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.68, 0.70, 1.0))

    carcass = model.part("cabinet_carcass")
    carcass.visual(
        Box((FRAME_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + FRAME_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=painted_charcoal,
        name="left_side",
    )
    carcass.visual(
        Box((FRAME_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - FRAME_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=painted_charcoal,
        name="right_side",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * FRAME_THICKNESS), BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0 - BACK_THICKNESS / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=painted_charcoal,
        name="back_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * FRAME_THICKNESS), CABINET_DEPTH - BACK_THICKNESS, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -BACK_THICKNESS / 2.0,
                PANEL_THICKNESS / 2.0,
            )
        ),
        material=painted_charcoal,
        name="bottom_panel",
    )
    carcass.visual(
        Box((FRAME_THICKNESS, CABINET_DEPTH - TOP_RAIL_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + FRAME_THICKNESS * 1.5,
                -TOP_RAIL_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                CABINET_HEIGHT - PANEL_THICKNESS / 2.0,
            )
        ),
        material=graphite,
        name="left_top_rail",
    )
    carcass.visual(
        Box((FRAME_THICKNESS, CABINET_DEPTH - TOP_RAIL_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - FRAME_THICKNESS * 1.5,
                -TOP_RAIL_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                CABINET_HEIGHT - PANEL_THICKNESS / 2.0,
            )
        ),
        material=graphite,
        name="right_top_rail",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * FRAME_THICKNESS), TOP_RAIL_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0 - TOP_RAIL_DEPTH / 2.0,
                CABINET_HEIGHT - PANEL_THICKNESS / 2.0,
            )
        ),
        material=graphite,
        name="rear_top_rail",
    )
    carcass.visual(
        Box((0.020, CABINET_DEPTH - BACK_THICKNESS - 0.040, CABINET_HEIGHT - 0.120)),
        origin=Origin(
            xyz=(
                0.0,
                -0.020,
                (CABINET_HEIGHT - 0.120) / 2.0 + 0.060,
            )
        ),
        material=painted_charcoal,
        name="center_stile",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 0.080, 0.060, 0.080)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH / 2.0 + 0.070, 0.040)),
        material=satin_black,
        name="toe_kick",
    )
    carcass.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + DOOR_GAP,
                -CABINET_DEPTH / 2.0 - 0.001,
                DOOR_BOTTOM + DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="left_hinge_pin",
    )
    carcass.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - DOOR_GAP,
                -CABINET_DEPTH / 2.0 - 0.001,
                DOOR_BOTTOM + DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="right_hinge_pin",
    )
    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    counter_outer = _rect_profile(COUNTER_WIDTH, COUNTER_DEPTH)
    counter_opening = _rect_profile(
        COUNTER_OPENING_WIDTH,
        COUNTER_OPENING_DEPTH,
        cx=0.0,
        cy=COUNTER_OPENING_Y,
    )
    countertop_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            counter_outer,
            [counter_opening],
            height=COUNTER_THICKNESS,
            center=False,
        ),
        ASSETS.mesh_path("countertop_with_cooktop_cutout.obj"),
    )
    countertop = model.part("countertop")
    countertop.visual(countertop_mesh, material=stone, name="counter_slab")
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_THICKNESS / 2.0)),
    )

    zone_specs = [
        (-0.220, -0.115, 0.088),
        (-0.220, 0.110, 0.080),
        (0.000, 0.010, 0.072),
        (0.205, 0.108, 0.076),
        (0.205, -0.112, 0.084),
    ]
    cooktop_holes = [_circle_profile(radius, cx=x, cy=y, segments=40) for x, y, radius in zone_specs]
    cooktop_top_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(COOKTOP_WIDTH, COOKTOP_DEPTH),
            cooktop_holes,
            height=COOKTOP_TOP_THICKNESS,
            center=False,
        ),
        ASSETS.mesh_path("cooktop_top_plate.obj"),
    )
    cooktop = model.part("cooktop_body")
    cooktop.visual(cooktop_top_mesh, material=dark_glass, name="top_plate")
    cooktop.visual(
        Box(
            (
                COOKTOP_CHASSIS_WIDTH - (2.0 * COOKTOP_WALL_THICKNESS),
                COOKTOP_CHASSIS_DEPTH - (2.0 * COOKTOP_WALL_THICKNESS),
                COOKTOP_FLOOR_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                COOKTOP_RECESS_FLOOR_Z - COOKTOP_FLOOR_THICKNESS / 2.0,
            )
        ),
        material=matte_black,
        name="recess_floor",
    )
    cooktop.visual(
        Box(
            (
                COOKTOP_WALL_THICKNESS,
                COOKTOP_CHASSIS_DEPTH,
                abs(COOKTOP_RECESS_FLOOR_Z),
            )
        ),
        origin=Origin(
            xyz=(
                -COOKTOP_CHASSIS_WIDTH / 2.0 + COOKTOP_WALL_THICKNESS / 2.0,
                0.0,
                COOKTOP_RECESS_FLOOR_Z / 2.0,
            )
        ),
        material=graphite,
        name="left_wall",
    )
    cooktop.visual(
        Box(
            (
                COOKTOP_WALL_THICKNESS,
                COOKTOP_CHASSIS_DEPTH,
                abs(COOKTOP_RECESS_FLOOR_Z),
            )
        ),
        origin=Origin(
            xyz=(
                COOKTOP_CHASSIS_WIDTH / 2.0 - COOKTOP_WALL_THICKNESS / 2.0,
                0.0,
                COOKTOP_RECESS_FLOOR_Z / 2.0,
            )
        ),
        material=graphite,
        name="right_wall",
    )
    cooktop.visual(
        Box(
            (
                COOKTOP_CHASSIS_WIDTH - (2.0 * COOKTOP_WALL_THICKNESS),
                COOKTOP_WALL_THICKNESS,
                abs(COOKTOP_RECESS_FLOOR_Z),
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_CHASSIS_DEPTH / 2.0 + COOKTOP_WALL_THICKNESS / 2.0,
                COOKTOP_RECESS_FLOOR_Z / 2.0,
            )
        ),
        material=graphite,
        name="front_wall",
    )
    cooktop.visual(
        Box(
            (
                COOKTOP_CHASSIS_WIDTH - (2.0 * COOKTOP_WALL_THICKNESS),
                COOKTOP_WALL_THICKNESS,
                abs(COOKTOP_RECESS_FLOOR_Z),
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                COOKTOP_CHASSIS_DEPTH / 2.0 - COOKTOP_WALL_THICKNESS / 2.0,
                COOKTOP_RECESS_FLOOR_Z / 2.0,
            )
        ),
        material=graphite,
        name="rear_wall",
    )
    cooktop.visual(
        Box((0.700, 0.010, 0.032)),
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_CHASSIS_DEPTH / 2.0 + 0.010,
                -0.022,
            )
        ),
        material=graphite,
        name="front_panel_ledge",
    )
    for index, (x_pos, y_pos, radius) in enumerate(zone_specs):
        cooktop.visual(
            Cylinder(radius=radius * 0.92, length=0.0015),
            origin=Origin(
                xyz=(
                    x_pos,
                    y_pos,
                    COOKTOP_RECESS_FLOOR_Z - 0.00075,
                )
            ),
            material=satin_black,
            name=f"zone_{index + 1}",
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.050)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
    )

    panel_holes = [
        (-0.255, 0.011, UPPER_BUTTON_WIDTH),
        (-0.085, 0.011, UPPER_BUTTON_WIDTH),
        (0.085, 0.011, UPPER_BUTTON_WIDTH),
        (0.255, 0.011, UPPER_BUTTON_WIDTH),
        (-0.128, -0.013, LOWER_BUTTON_WIDTH),
        (0.128, -0.013, LOWER_BUTTON_WIDTH),
    ]
    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=graphite,
        name="top_strip",
    )
    control_panel.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=graphite,
        name="middle_strip",
    )
    control_panel.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
        material=graphite,
        name="bottom_strip",
    )
    control_panel.visual(
        Box((CONTROL_PANEL_WIDTH - 0.040, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.004)),
        material=graphite,
        name="top_return_flange",
    )
    _add_row_fillers(
        control_panel,
        slots=[(x_pos, width) for x_pos, _, width in panel_holes[:4]],
        z_center=-0.011,
        row_height=BUTTON_HEIGHT,
        depth=CONTROL_PANEL_THICKNESS,
        material=graphite,
        prefix="upper_row_filler",
    )
    _add_row_fillers(
        control_panel,
        slots=[(x_pos, width) for x_pos, _, width in panel_holes[4:]],
        z_center=-0.035,
        row_height=BUTTON_HEIGHT,
        depth=CONTROL_PANEL_THICKNESS,
        material=graphite,
        prefix="lower_row_filler",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((CONTROL_PANEL_WIDTH, 0.020, 0.060)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.008, 0.006)),
    )

    button_parts = []
    for index, (x_pos, z_pos, width) in enumerate(panel_holes, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((width, 0.009, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, -0.0005, 0.0)),
            material=button_grey,
            name="button_body",
        )
        button.inertial = Inertial.from_geometry(
            Box((width, 0.009, BUTTON_HEIGHT)),
            mass=0.05,
            origin=Origin(xyz=(0.0, -0.0005, 0.0)),
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, z_pos - 0.022)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )
        button_parts.append(button)

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_LEAF_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH / 2.0,
                -DOOR_THICKNESS / 2.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=painted_charcoal,
        name="door_leaf",
    )
    left_door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(0.026, -0.011, DOOR_HEIGHT / 2.0)),
        material=steel,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.042, 0.012, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.026,
                -0.011,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="hinge_web",
    )
    left_door.visual(
        Box((0.018, 0.030, 0.180)),
        origin=Origin(
            xyz=(
                DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH - 0.055,
                -0.024,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_LEAF_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=8.0,
        origin=Origin(
            xyz=(
                DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH / 2.0,
                -DOOR_THICKNESS / 2.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_LEAF_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH / 2.0),
                -DOOR_THICKNESS / 2.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=painted_charcoal,
        name="door_leaf",
    )
    right_door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(-0.026, -0.011, DOOR_HEIGHT / 2.0)),
        material=steel,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.042, 0.012, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.026,
                -0.011,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="hinge_web",
    )
    right_door.visual(
        Box((0.018, 0.030, 0.180)),
        origin=Origin(
            xyz=(
                -(DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH - 0.055),
                -0.024,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=steel,
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_LEAF_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=8.0,
        origin=Origin(
            xyz=(
                -(DOOR_SIDE_CLEARANCE + DOOR_LEAF_WIDTH / 2.0),
                -DOOR_THICKNESS / 2.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
    )

    model.articulation(
        "carcass_to_countertop",
        ArticulationType.FIXED,
        parent=carcass,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT)),
    )
    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(
            xyz=(
                0.0,
                COUNTER_OPENING_Y,
                COUNTER_THICKNESS - COOKTOP_TOP_THICKNESS - 0.003,
            )
        ),
    )
    model.articulation(
        "cooktop_to_control_panel",
        ArticulationType.FIXED,
        parent=cooktop,
        child=control_panel,
        origin=Origin(
            xyz=(
                0.0,
                -0.241,
                -0.039,
            )
        ),
    )
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + DOOR_GAP,
                -CABINET_DEPTH / 2.0 - 0.001,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-1.90,
            upper=0.0,
        ),
    )
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - DOOR_GAP,
                -CABINET_DEPTH / 2.0 - 0.001,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    carcass = object_model.get_part("cabinet_carcass")
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop_body")
    control_panel = object_model.get_part("control_panel")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    left_pin = carcass.get_visual("left_hinge_pin")
    right_pin = carcass.get_visual("right_hinge_pin")
    left_barrel = left_door.get_visual("hinge_barrel")
    right_barrel = right_door.get_visual("hinge_barrel")

    ctx.allow_overlap(
        left_door,
        carcass,
        elem_a=left_barrel,
        elem_b=left_pin,
        reason="Left door hinge barrel intentionally wraps the fixed hinge pin to represent the pivot.",
    )
    ctx.allow_overlap(
        right_door,
        carcass,
        elem_a=right_barrel,
        elem_b=right_pin,
        reason="Right door hinge barrel intentionally wraps the fixed hinge pin to represent the pivot.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(countertop, carcass, name="countertop_resting_on_carcass")
    ctx.expect_contact(cooktop, countertop, name="cooktop_supported_by_countertop")
    ctx.expect_contact(
        control_panel,
        cooktop,
        contact_tol=0.0015,
        name="control_panel_attached_to_cooktop",
    )

    ctx.expect_gap(
        countertop,
        carcass,
        axis="z",
        min_gap=0.0,
        max_gap=0.040,
        name="countertop_above_cabinet_height",
    )
    ctx.expect_gap(
        control_panel,
        left_door,
        axis="z",
        min_gap=0.020,
        name="control_panel_clears_doors",
    )
    ctx.expect_overlap(
        cooktop,
        countertop,
        axes="xy",
        min_overlap=0.40,
        name="cooktop_sits_under_counter_cutout",
    )

    button_specs = [
        ("button_1", "control_panel_to_button_1"),
        ("button_2", "control_panel_to_button_2"),
        ("button_3", "control_panel_to_button_3"),
        ("button_4", "control_panel_to_button_4"),
        ("button_5", "control_panel_to_button_5"),
        ("button_6", "control_panel_to_button_6"),
    ]
    for part_name, joint_name in button_specs:
        button = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        axis_ok = tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0)
        ctx.check(f"{joint_name}_axis_is_inward_y", axis_ok, details=f"axis={joint.axis}")
        ctx.expect_contact(button, control_panel, name=f"{part_name}_closed_contact")
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
                ctx.expect_contact(button, control_panel, name=f"{part_name}_lower_captured")
                closed_pos = ctx.part_world_position(button)
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")
                ctx.expect_contact(button, control_panel, name=f"{part_name}_upper_captured")
                pressed_pos = ctx.part_world_position(button)
            if closed_pos is not None and pressed_pos is not None:
                ctx.check(
                    f"{joint_name}_moves_inward",
                    pressed_pos[1] > closed_pos[1] + 0.004,
                    details=f"closed_y={closed_pos[1]:.4f}, pressed_y={pressed_pos[1]:.4f}",
                )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left_hinge_axis_vertical",
        tuple(round(value, 6) for value in left_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        tuple(round(value, 6) for value in right_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={right_hinge.axis}",
    )
    ctx.expect_contact(left_door, carcass, contact_tol=0.003, name="left_door_hinge_contact_closed")
    ctx.expect_contact(right_door, carcass, contact_tol=0.003, name="right_door_hinge_contact_closed")

    left_closed_center = None
    left_open_center = None
    if left_limits is not None and left_limits.lower is not None and left_limits.upper is not None:
        with ctx.pose({left_hinge: left_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_closed_no_floating")
            left_closed_center = _aabb_center(ctx.part_world_aabb(left_door))
        with ctx.pose({left_hinge: left_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
            ctx.expect_contact(left_door, carcass, contact_tol=0.003, name="left_door_hinge_contact_open")
            left_open_center = _aabb_center(ctx.part_world_aabb(left_door))
    if left_closed_center is not None and left_open_center is not None:
        ctx.check(
            "left_door_swings_outward",
            left_open_center[1] < left_closed_center[1] - 0.10,
            details=f"closed_center={left_closed_center}, open_center={left_open_center}",
        )

    right_closed_center = None
    right_open_center = None
    if right_limits is not None and right_limits.lower is not None and right_limits.upper is not None:
        with ctx.pose({right_hinge: right_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_closed_no_floating")
            right_closed_center = _aabb_center(ctx.part_world_aabb(right_door))
        with ctx.pose({right_hinge: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
            ctx.expect_contact(right_door, carcass, contact_tol=0.003, name="right_door_hinge_contact_open")
            right_open_center = _aabb_center(ctx.part_world_aabb(right_door))
    if right_closed_center is not None and right_open_center is not None:
        ctx.check(
            "right_door_swings_outward",
            right_open_center[1] < right_closed_center[1] - 0.10,
            details=f"closed_center={right_closed_center}, open_center={right_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
