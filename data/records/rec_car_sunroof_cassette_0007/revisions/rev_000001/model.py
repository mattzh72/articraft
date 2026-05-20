from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

CASSETTE_WIDTH = 0.92
CASSETTE_LENGTH = 1.70
TRAY_HEIGHT = 0.020

OPENING_WIDTH = 0.74
OPENING_LENGTH = 0.60
OPENING_CENTER_Y = -0.22
OPENING_FRONT_Y = OPENING_CENTER_Y - OPENING_LENGTH / 2.0
OPENING_REAR_Y = OPENING_CENTER_Y + OPENING_LENGTH / 2.0

SIDE_GUTTER_WIDTH = (CASSETTE_WIDTH - OPENING_WIDTH) / 2.0
FRONT_HEADER_LENGTH = OPENING_FRONT_Y + CASSETTE_LENGTH / 2.0
REAR_STORAGE_LENGTH = CASSETTE_LENGTH / 2.0 - OPENING_REAR_Y

FRAME_WIDTH = 0.018
FRAME_HEIGHT = 0.008
FRAME_Z = TRAY_HEIGHT + FRAME_HEIGHT / 2.0

GUIDE_X = 0.335
RAIL_CENTER_Y = 0.15
RAIL_LENGTH = 1.28
RAIL_SUPPORT_WIDTH = 0.080
RAIL_SUPPORT_HEIGHT = 0.004
RAIL_SUPPORT_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT / 2.0

GUIDE_BASE_WIDTH = 0.022
GUIDE_BASE_HEIGHT = 0.004
GUIDE_BASE_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT + GUIDE_BASE_HEIGHT / 2.0
GUIDE_RIB_WIDTH = 0.016
GUIDE_RIB_HEIGHT = 0.004
GUIDE_RIB_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT + GUIDE_BASE_HEIGHT + GUIDE_RIB_HEIGHT / 2.0
GUIDE_WALL_THICKNESS = 0.003
GUIDE_WALL_HEIGHT = 0.006
GUIDE_WALL_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT + GUIDE_WALL_HEIGHT / 2.0

PANEL_WIDTH = 0.82
PANEL_LENGTH = 0.69
PANEL_CLOSED_CENTER_Y = OPENING_CENTER_Y
PANEL_TRAVEL = 0.72
PANEL_CORNER_RADIUS = 0.070
SHOE_WIDTH = 0.016
SHOE_LENGTH = 0.46
SHOE_CENTER_Y = 0.05
SHOE_HEIGHT = 0.004
SHOE_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT + GUIDE_BASE_HEIGHT + GUIDE_RIB_HEIGHT + SHOE_HEIGHT / 2.0

BOTTOM_GLASS_THICKNESS = 0.003
INTERLAYER_THICKNESS = 0.0012
TOP_GLASS_THICKNESS = 0.003
BOTTOM_GLASS_Z = TRAY_HEIGHT + RAIL_SUPPORT_HEIGHT + GUIDE_BASE_HEIGHT + GUIDE_RIB_HEIGHT + SHOE_HEIGHT + BOTTOM_GLASS_THICKNESS / 2.0
INTERLAYER_Z = (
    TRAY_HEIGHT
    + RAIL_SUPPORT_HEIGHT
    + GUIDE_BASE_HEIGHT
    + GUIDE_RIB_HEIGHT
    + SHOE_HEIGHT
    + BOTTOM_GLASS_THICKNESS
    + INTERLAYER_THICKNESS / 2.0
)
TOP_GLASS_Z = (
    TRAY_HEIGHT
    + RAIL_SUPPORT_HEIGHT
    + GUIDE_BASE_HEIGHT
    + GUIDE_RIB_HEIGHT
    + SHOE_HEIGHT
    + BOTTOM_GLASS_THICKNESS
    + INTERLAYER_THICKNESS
    + TOP_GLASS_THICKNESS / 2.0
)
FRIT_THICKNESS = 0.0008
FRIT_Z = (
    TRAY_HEIGHT
    + RAIL_SUPPORT_HEIGHT
    + GUIDE_BASE_HEIGHT
    + GUIDE_RIB_HEIGHT
    + SHOE_HEIGHT
    + BOTTOM_GLASS_THICKNESS
    + INTERLAYER_THICKNESS
    + TOP_GLASS_THICKNESS
    + FRIT_THICKNESS / 2.0
)


def _add_box_visual(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _rounded_panel_mesh(width: float, length: float, thickness: float, radius: float, mesh_name: str):
    profile = rounded_rect_profile(width, length, radius, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True, cap=True, closed=True),
        ASSETS.mesh_path(mesh_name),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laminated_glass_sunroof_cassette", assets=ASSETS)

    tray_black = model.material("tray_black", rgba=(0.12, 0.13, 0.14, 1.0))
    aluminium = model.material("brushed_aluminium", rgba=(0.74, 0.76, 0.79, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    glass_lower = model.material("glass_lower", rgba=(0.54, 0.70, 0.77, 0.22))
    glass_upper = model.material("glass_upper", rgba=(0.62, 0.80, 0.87, 0.30))
    laminate = model.material("laminate", rgba=(0.10, 0.12, 0.14, 0.42))
    frit_black = model.material("frit_black", rgba=(0.04, 0.04, 0.05, 0.88))

    cassette = model.part("cassette")
    _add_box_visual(
        cassette,
        "left_gutter",
        (SIDE_GUTTER_WIDTH, CASSETTE_LENGTH, TRAY_HEIGHT),
        (-CASSETTE_WIDTH / 2.0 + SIDE_GUTTER_WIDTH / 2.0, 0.0, TRAY_HEIGHT / 2.0),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "right_gutter",
        (SIDE_GUTTER_WIDTH, CASSETTE_LENGTH, TRAY_HEIGHT),
        (CASSETTE_WIDTH / 2.0 - SIDE_GUTTER_WIDTH / 2.0, 0.0, TRAY_HEIGHT / 2.0),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "front_header",
        (CASSETTE_WIDTH, FRONT_HEADER_LENGTH, TRAY_HEIGHT),
        (
            0.0,
            -CASSETTE_LENGTH / 2.0 + FRONT_HEADER_LENGTH / 2.0,
            TRAY_HEIGHT / 2.0,
        ),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "rear_storage",
        (CASSETTE_WIDTH, REAR_STORAGE_LENGTH, TRAY_HEIGHT),
        (
            0.0,
            OPENING_REAR_Y + REAR_STORAGE_LENGTH / 2.0,
            TRAY_HEIGHT / 2.0,
        ),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "left_rail_support",
        (RAIL_SUPPORT_WIDTH, RAIL_LENGTH, RAIL_SUPPORT_HEIGHT),
        (-GUIDE_X, RAIL_CENTER_Y, RAIL_SUPPORT_Z),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "right_rail_support",
        (RAIL_SUPPORT_WIDTH, RAIL_LENGTH, RAIL_SUPPORT_HEIGHT),
        (GUIDE_X, RAIL_CENTER_Y, RAIL_SUPPORT_Z),
        tray_black,
    )
    _add_box_visual(
        cassette,
        "left_frame_strip",
        (FRAME_WIDTH, OPENING_LENGTH, FRAME_HEIGHT),
        (-(OPENING_WIDTH / 2.0 + FRAME_WIDTH / 2.0), OPENING_CENTER_Y, FRAME_Z),
        aluminium,
    )
    _add_box_visual(
        cassette,
        "right_frame_strip",
        (FRAME_WIDTH, OPENING_LENGTH, FRAME_HEIGHT),
        ((OPENING_WIDTH / 2.0 + FRAME_WIDTH / 2.0), OPENING_CENTER_Y, FRAME_Z),
        aluminium,
    )
    _add_box_visual(
        cassette,
        "front_frame_strip",
        (OPENING_WIDTH, FRAME_WIDTH, FRAME_HEIGHT),
        (0.0, OPENING_FRONT_Y + FRAME_WIDTH / 2.0, FRAME_Z),
        aluminium,
    )
    _add_box_visual(
        cassette,
        "rear_frame_strip",
        (OPENING_WIDTH, FRAME_WIDTH, FRAME_HEIGHT),
        (0.0, OPENING_REAR_Y - FRAME_WIDTH / 2.0, FRAME_Z),
        aluminium,
    )
    cassette.inertial = Inertial.from_geometry(
        Box((CASSETTE_WIDTH, CASSETTE_LENGTH, 0.05)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    left_guide_rail = model.part("left_guide_rail")
    _add_box_visual(
        left_guide_rail,
        "guide_base",
        (GUIDE_BASE_WIDTH, RAIL_LENGTH, GUIDE_BASE_HEIGHT),
        (0.0, 0.0, GUIDE_BASE_Z),
        guide_dark,
    )
    _add_box_visual(
        left_guide_rail,
        "guide_wall_left",
        (GUIDE_WALL_THICKNESS, RAIL_LENGTH, GUIDE_WALL_HEIGHT),
        (-(GUIDE_BASE_WIDTH / 2.0 - GUIDE_WALL_THICKNESS / 2.0), 0.0, GUIDE_WALL_Z),
        guide_dark,
    )
    _add_box_visual(
        left_guide_rail,
        "guide_wall_right",
        (GUIDE_WALL_THICKNESS, RAIL_LENGTH, GUIDE_WALL_HEIGHT),
        ((GUIDE_BASE_WIDTH / 2.0 - GUIDE_WALL_THICKNESS / 2.0), 0.0, GUIDE_WALL_Z),
        guide_dark,
    )
    _add_box_visual(
        left_guide_rail,
        "guide_rib",
        (GUIDE_RIB_WIDTH, RAIL_LENGTH, GUIDE_RIB_HEIGHT),
        (0.0, 0.0, GUIDE_RIB_Z),
        guide_dark,
    )
    left_guide_rail.inertial = Inertial.from_geometry(
        Box((GUIDE_BASE_WIDTH, RAIL_LENGTH, 0.02)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    right_guide_rail = model.part("right_guide_rail")
    _add_box_visual(
        right_guide_rail,
        "guide_base",
        (GUIDE_BASE_WIDTH, RAIL_LENGTH, GUIDE_BASE_HEIGHT),
        (0.0, 0.0, GUIDE_BASE_Z),
        guide_dark,
    )
    _add_box_visual(
        right_guide_rail,
        "guide_wall_left",
        (GUIDE_WALL_THICKNESS, RAIL_LENGTH, GUIDE_WALL_HEIGHT),
        (-(GUIDE_BASE_WIDTH / 2.0 - GUIDE_WALL_THICKNESS / 2.0), 0.0, GUIDE_WALL_Z),
        guide_dark,
    )
    _add_box_visual(
        right_guide_rail,
        "guide_wall_right",
        (GUIDE_WALL_THICKNESS, RAIL_LENGTH, GUIDE_WALL_HEIGHT),
        ((GUIDE_BASE_WIDTH / 2.0 - GUIDE_WALL_THICKNESS / 2.0), 0.0, GUIDE_WALL_Z),
        guide_dark,
    )
    _add_box_visual(
        right_guide_rail,
        "guide_rib",
        (GUIDE_RIB_WIDTH, RAIL_LENGTH, GUIDE_RIB_HEIGHT),
        (0.0, 0.0, GUIDE_RIB_Z),
        guide_dark,
    )
    right_guide_rail.inertial = Inertial.from_geometry(
        Box((GUIDE_BASE_WIDTH, RAIL_LENGTH, 0.02)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    glass_panel = model.part("glass_panel")
    panel_bottom_mesh = _rounded_panel_mesh(
        PANEL_WIDTH,
        PANEL_LENGTH,
        BOTTOM_GLASS_THICKNESS,
        PANEL_CORNER_RADIUS,
        "sunroof_bottom_glass.obj",
    )
    panel_interlayer_mesh = _rounded_panel_mesh(
        PANEL_WIDTH - 0.008,
        PANEL_LENGTH - 0.008,
        INTERLAYER_THICKNESS,
        PANEL_CORNER_RADIUS - 0.004,
        "sunroof_interlayer.obj",
    )
    panel_top_mesh = _rounded_panel_mesh(
        PANEL_WIDTH,
        PANEL_LENGTH,
        TOP_GLASS_THICKNESS,
        PANEL_CORNER_RADIUS,
        "sunroof_top_glass.obj",
    )
    glass_panel.visual(
        panel_bottom_mesh,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_GLASS_Z)),
        material=glass_lower,
        name="bottom_glass",
    )
    glass_panel.visual(
        panel_interlayer_mesh,
        origin=Origin(xyz=(0.0, 0.0, INTERLAYER_Z)),
        material=laminate,
        name="interlayer",
    )
    glass_panel.visual(
        panel_top_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_GLASS_Z)),
        material=glass_upper,
        name="top_glass",
    )
    _add_box_visual(
        glass_panel,
        "left_frit",
        (0.028, PANEL_LENGTH - 0.050, FRIT_THICKNESS),
        (-(PANEL_WIDTH / 2.0 - 0.014), 0.0, FRIT_Z),
        frit_black,
    )
    _add_box_visual(
        glass_panel,
        "right_frit",
        (0.028, PANEL_LENGTH - 0.050, FRIT_THICKNESS),
        (PANEL_WIDTH / 2.0 - 0.014, 0.0, FRIT_Z),
        frit_black,
    )
    _add_box_visual(
        glass_panel,
        "front_frit",
        (PANEL_WIDTH - 0.056, 0.028, FRIT_THICKNESS),
        (0.0, -(PANEL_LENGTH / 2.0 - 0.014), FRIT_Z),
        frit_black,
    )
    _add_box_visual(
        glass_panel,
        "rear_frit",
        (PANEL_WIDTH - 0.056, 0.028, FRIT_THICKNESS),
        (0.0, PANEL_LENGTH / 2.0 - 0.014, FRIT_Z),
        frit_black,
    )
    _add_box_visual(
        glass_panel,
        "left_carriage",
        (SHOE_WIDTH, SHOE_LENGTH, SHOE_HEIGHT),
        (-GUIDE_X, SHOE_CENTER_Y, SHOE_Z),
        guide_dark,
    )
    _add_box_visual(
        glass_panel,
        "right_carriage",
        (SHOE_WIDTH, SHOE_LENGTH, SHOE_HEIGHT),
        (GUIDE_X, SHOE_CENTER_Y, SHOE_Z),
        guide_dark,
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_LENGTH, 0.045)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
    )

    model.articulation(
        "cassette_to_left_guide_rail",
        ArticulationType.FIXED,
        parent=cassette,
        child=left_guide_rail,
        origin=Origin(xyz=(-GUIDE_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "cassette_to_right_guide_rail",
        ArticulationType.FIXED,
        parent=cassette,
        child=right_guide_rail,
        origin=Origin(xyz=(GUIDE_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=glass_panel,
        origin=Origin(xyz=(0.0, PANEL_CLOSED_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=0.0,
            upper=PANEL_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cassette = object_model.get_part("cassette")
    left_guide_rail = object_model.get_part("left_guide_rail")
    right_guide_rail = object_model.get_part("right_guide_rail")
    glass_panel = object_model.get_part("glass_panel")
    panel_slide = object_model.get_articulation("panel_slide")

    left_rail_support = cassette.get_visual("left_rail_support")
    right_rail_support = cassette.get_visual("right_rail_support")
    left_frame_strip = cassette.get_visual("left_frame_strip")
    right_frame_strip = cassette.get_visual("right_frame_strip")
    front_frame_strip = cassette.get_visual("front_frame_strip")
    rear_frame_strip = cassette.get_visual("rear_frame_strip")
    rear_storage = cassette.get_visual("rear_storage")

    left_guide_base = left_guide_rail.get_visual("guide_base")
    left_guide_rib = left_guide_rail.get_visual("guide_rib")
    right_guide_base = right_guide_rail.get_visual("guide_base")
    right_guide_rib = right_guide_rail.get_visual("guide_rib")

    bottom_glass = glass_panel.get_visual("bottom_glass")
    top_glass = glass_panel.get_visual("top_glass")
    left_carriage = glass_panel.get_visual("left_carriage")
    right_carriage = glass_panel.get_visual("right_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_guide_rail,
        cassette,
        elem_a=left_guide_base,
        elem_b=left_rail_support,
        name="left_guide_rail_mounted_to_cassette",
    )
    ctx.expect_contact(
        right_guide_rail,
        cassette,
        elem_a=right_guide_base,
        elem_b=right_rail_support,
        name="right_guide_rail_mounted_to_cassette",
    )
    ctx.expect_origin_distance(
        glass_panel,
        cassette,
        axes="x",
        max_dist=0.001,
        name="panel_is_centered_laterally_in_cassette",
    )

    closed_position = None
    open_position = None
    limits = panel_slide.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({panel_slide: limits.lower}):
            closed_position = ctx.part_world_position(glass_panel)
            ctx.fail_if_parts_overlap_in_current_pose(name="panel_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="panel_slide_lower_no_floating")
            ctx.expect_within(cassette, glass_panel, axes="xy", inner_elem=left_frame_strip, outer_elem=top_glass)
            ctx.expect_within(cassette, glass_panel, axes="xy", inner_elem=right_frame_strip, outer_elem=top_glass)
            ctx.expect_within(cassette, glass_panel, axes="xy", inner_elem=front_frame_strip, outer_elem=top_glass)
            ctx.expect_within(cassette, glass_panel, axes="xy", inner_elem=rear_frame_strip, outer_elem=top_glass)
            ctx.expect_within(
                glass_panel,
                left_guide_rail,
                axes="xy",
                inner_elem=left_carriage,
                outer_elem=left_guide_base,
                name="left_carriage_within_left_guide_at_rest",
            )
            ctx.expect_within(
                glass_panel,
                right_guide_rail,
                axes="xy",
                inner_elem=right_carriage,
                outer_elem=right_guide_base,
                name="right_carriage_within_right_guide_at_rest",
            )
            ctx.expect_contact(
                glass_panel,
                left_guide_rail,
                elem_a=left_carriage,
                elem_b=left_guide_rib,
                name="left_carriage_seated_on_left_guide",
            )
            ctx.expect_contact(
                glass_panel,
                right_guide_rail,
                elem_a=right_carriage,
                elem_b=right_guide_rib,
                name="right_carriage_seated_on_right_guide",
            )
            ctx.expect_gap(
                glass_panel,
                cassette,
                axis="z",
                min_gap=0.007,
                max_gap=0.009,
                positive_elem=bottom_glass,
                negative_elem=front_frame_strip,
                name="closed_glass_sits_just_above_front_frame",
            )

        with ctx.pose({panel_slide: limits.upper}):
            open_position = ctx.part_world_position(glass_panel)
            ctx.fail_if_parts_overlap_in_current_pose(name="panel_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="panel_slide_upper_no_floating")
            ctx.expect_within(
                glass_panel,
                left_guide_rail,
                axes="xy",
                inner_elem=left_carriage,
                outer_elem=left_guide_base,
                name="left_carriage_within_left_guide_open",
            )
            ctx.expect_within(
                glass_panel,
                right_guide_rail,
                axes="xy",
                inner_elem=right_carriage,
                outer_elem=right_guide_base,
                name="right_carriage_within_right_guide_open",
            )
            ctx.expect_contact(
                glass_panel,
                left_guide_rail,
                elem_a=left_carriage,
                elem_b=left_guide_rib,
                name="left_carriage_stays_seated_open",
            )
            ctx.expect_contact(
                glass_panel,
                right_guide_rail,
                elem_a=right_carriage,
                elem_b=right_guide_rib,
                name="right_carriage_stays_seated_open",
            )
            ctx.expect_within(
                glass_panel,
                cassette,
                axes="xy",
                inner_elem=top_glass,
                outer_elem=rear_storage,
                name="open_panel_stows_over_rear_storage",
            )
            ctx.expect_gap(
                glass_panel,
                cassette,
                axis="y",
                min_gap=0.05,
                positive_elem=bottom_glass,
                negative_elem=rear_frame_strip,
                name="open_panel_front_edge_clears_rear_frame",
            )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    if closed_position is not None and open_position is not None:
        delta_y = open_position[1] - closed_position[1]
        delta_x = abs(open_position[0] - closed_position[0])
        delta_z = abs(open_position[2] - closed_position[2])
        ctx.check(
            "panel_slide_travels_fully_rearward",
            abs(delta_y - PANEL_TRAVEL) <= 0.005 and delta_x <= 0.001 and delta_z <= 0.001,
            details=(
                f"expected rearward travel {PANEL_TRAVEL:.3f} m with no lateral or vertical drift, "
                f"got dx={delta_x:.4f}, dy={delta_y:.4f}, dz={delta_z:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
