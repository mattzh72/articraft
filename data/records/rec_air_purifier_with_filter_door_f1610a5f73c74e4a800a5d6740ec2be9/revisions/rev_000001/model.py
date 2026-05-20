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


OUTER_DEPTH = 0.140
OUTER_WIDTH = 0.330
OUTER_HEIGHT = 0.480
WALL_T = 0.005
BACK_T = 0.006
FRONT_T = 0.008

COVER_TH = 0.006
COVER_HEIGHT = 0.468
HINGE_AXIS_X = (OUTER_DEPTH * 0.5) + 0.004
HINGE_AXIS_Y = (OUTER_WIDTH * 0.5) + 0.003
HINGE_RADIUS = 0.005
HINGE_LEN = 0.035

TRAY_TRAVEL = 0.075


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_air_purifier")

    wall_bracket_gray = model.material("wall_bracket_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    vent_white = model.material("vent_white", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    filter_green = model.material("filter_green", rgba=(0.43, 0.58, 0.47, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.57, 0.60, 0.63, 1.0))
    status_blue = model.material("status_blue", rgba=(0.42, 0.67, 0.88, 0.85))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.012, 0.180, 0.300)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=wall_bracket_gray,
        name="back_plate",
    )
    wall_bracket.visual(
        Box((0.0285, 0.070, 0.180)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=wall_bracket_gray,
        name="center_spine",
    )
    wall_bracket.visual(
        Box((0.024, 0.148, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, 0.108)),
        material=wall_bracket_gray,
        name="upper_hook_bar",
    )
    wall_bracket.visual(
        Box((0.024, 0.060, 0.018)),
        origin=Origin(xyz=(0.018, -0.046, -0.108)),
        material=wall_bracket_gray,
        name="lower_pad_left",
    )
    wall_bracket.visual(
        Box((0.024, 0.060, 0.018)),
        origin=Origin(xyz=(0.018, 0.046, -0.108)),
        material=wall_bracket_gray,
        name="lower_pad_right",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.032, 0.190, 0.310)),
        mass=0.9,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((BACK_T, 0.322, 0.472)),
        origin=Origin(xyz=(-0.067, 0.0, 0.0)),
        material=shell_white,
        name="back_panel",
    )
    housing.visual(
        Box((0.135, WALL_T, OUTER_HEIGHT)),
        origin=Origin(xyz=(0.0025, -0.1625, 0.0)),
        material=shell_white,
        name="left_wall",
    )
    housing.visual(
        Box((0.135, WALL_T, OUTER_HEIGHT)),
        origin=Origin(xyz=(0.0025, 0.1625, 0.0)),
        material=shell_white,
        name="right_wall",
    )
    housing.visual(
        Box((0.135, 0.321, WALL_T)),
        origin=Origin(xyz=(0.0025, 0.0, 0.2375)),
        material=shell_white,
        name="top_wall",
    )
    housing.visual(
        Box((0.135, 0.321, WALL_T)),
        origin=Origin(xyz=(0.0025, 0.0, -0.2375)),
        material=shell_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((FRONT_T, 0.314, 0.032)),
        origin=Origin(xyz=(0.066, 0.0, 0.219)),
        material=shell_white,
        name="front_top_lip",
    )
    housing.visual(
        Box((FRONT_T, 0.314, 0.040)),
        origin=Origin(xyz=(0.066, 0.0, -0.215)),
        material=shell_white,
        name="front_bottom_lip",
    )
    housing.visual(
        Box((FRONT_T, 0.016, 0.430)),
        origin=Origin(xyz=(0.066, -0.157, 0.0)),
        material=shell_white,
        name="front_left_lip",
    )
    housing.visual(
        Box((FRONT_T, 0.012, 0.430)),
        origin=Origin(xyz=(0.066, 0.159, 0.0)),
        material=shell_white,
        name="front_right_lip",
    )
    housing.visual(
        Box((0.050, 0.180, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.243)),
        material=vent_white,
        name="top_exhaust_cap",
    )
    housing.visual(
        Box((0.100, 0.012, 0.012)),
        origin=Origin(xyz=(0.005, -0.132, -0.229)),
        material=wall_bracket_gray,
        name="left_slide_rail",
    )
    housing.visual(
        Box((0.100, 0.012, 0.012)),
        origin=Origin(xyz=(0.005, 0.132, -0.229)),
        material=wall_bracket_gray,
        name="right_slide_rail",
    )
    housing.visual(
        Box((0.014, 0.016, 0.032)),
        origin=Origin(xyz=(0.067, 0.176, 0.174)),
        material=hinge_metal,
        name="upper_hinge_leaf",
    )
    housing.visual(
        Box((0.014, 0.016, 0.032)),
        origin=Origin(xyz=(0.067, 0.176, -0.174)),
        material=hinge_metal,
        name="lower_hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LEN),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.1725)),
        material=hinge_metal,
        name="upper_housing_knuckle",
    )
    housing.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LEN),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, -0.1725)),
        material=hinge_metal,
        name="lower_housing_knuckle",
    )
    housing.inertial = Inertial.from_geometry(
        Box((OUTER_DEPTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=4.8,
        origin=Origin(),
    )

    cover = model.part("front_cover")
    cover.visual(
        Box((COVER_TH, 0.015, COVER_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=vent_white,
        name="right_stile",
    )
    cover.visual(
        Box((COVER_TH, 0.016, COVER_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.307, 0.0)),
        material=vent_white,
        name="left_stile",
    )
    cover.visual(
        Box((COVER_TH, 0.310, 0.024)),
        origin=Origin(xyz=(0.0, -0.160, 0.222)),
        material=vent_white,
        name="top_rail",
    )
    cover.visual(
        Box((COVER_TH, 0.310, 0.038)),
        origin=Origin(xyz=(0.0, -0.160, -0.215)),
        material=vent_white,
        name="bottom_rail",
    )
    cover.visual(
        Box((0.004, 0.278, 0.090)),
        origin=Origin(xyz=(0.001, -0.160, -0.155)),
        material=vent_white,
        name="lower_panel",
    )
    for index, z_pos in enumerate((-0.070, -0.030, 0.010, 0.050, 0.090, 0.130, 0.170)):
        cover.visual(
            Box((0.004, 0.278, 0.010)),
            origin=Origin(xyz=(0.001, -0.160, z_pos)),
            material=vent_white,
            name=f"grille_slat_{index}",
        )
    cover.visual(
        Box((0.004, 0.072, 0.020)),
        origin=Origin(xyz=(0.001, -0.200, 0.201)),
        material=status_blue,
        name="status_strip",
    )
    cover.visual(
        Box((0.014, 0.016, 0.056)),
        origin=Origin(xyz=(0.007, -0.313, -0.180)),
        material=hinge_metal,
        name="pull_tab",
    )
    cover.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=hinge_metal,
        name="upper_cover_knuckle",
    )
    cover.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.1375)),
        material=hinge_metal,
        name="lower_cover_knuckle",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.016, 0.320, COVER_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.004, -0.160, 0.0)),
    )

    tray = model.part("filter_tray")
    tray.visual(
        Box((0.088, 0.012, 0.012)),
        origin=Origin(xyz=(0.001, -0.132, 0.0)),
        material=tray_gray,
        name="left_runner",
    )
    tray.visual(
        Box((0.088, 0.012, 0.012)),
        origin=Origin(xyz=(0.001, 0.132, 0.0)),
        material=tray_gray,
        name="right_runner",
    )
    tray.visual(
        Box((0.096, 0.300, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.010)),
        material=tray_gray,
        name="base_plate",
    )
    tray.visual(
        Box((0.094, 0.008, 0.036)),
        origin=Origin(xyz=(0.001, -0.146, 0.024)),
        material=tray_gray,
        name="left_side",
    )
    tray.visual(
        Box((0.094, 0.008, 0.036)),
        origin=Origin(xyz=(0.001, 0.146, 0.024)),
        material=tray_gray,
        name="right_side",
    )
    tray.visual(
        Box((0.006, 0.300, 0.034)),
        origin=Origin(xyz=(-0.045, 0.0, 0.023)),
        material=tray_gray,
        name="rear_wall",
    )
    tray.visual(
        Box((0.012, 0.304, 0.040)),
        origin=Origin(xyz=(0.054, 0.0, 0.020)),
        material=tray_gray,
        name="front_pull",
    )
    tray.visual(
        Box((0.086, 0.280, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, 0.026)),
        material=filter_green,
        name="filter_media",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.102, 0.304, 0.042)),
        mass=0.7,
        origin=Origin(xyz=(0.004, 0.0, 0.021)),
    )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=wall_bracket,
        child=housing,
        origin=Origin(xyz=(0.104, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_filter_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(-0.010, 0.0, -0.217)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    cover = object_model.get_part("front_cover")
    tray = object_model.get_part("filter_tray")

    cover_hinge = object_model.get_articulation("housing_to_cover")
    tray_slide = object_model.get_articulation("housing_to_filter_tray")

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
        housing,
        wall_bracket,
        elem_a="back_panel",
        elem_b="center_spine",
        name="housing mounts onto the wall bracket spine",
    )
    ctx.expect_contact(
        cover,
        housing,
        elem_a="upper_cover_knuckle",
        elem_b="upper_housing_knuckle",
        name="upper hinge knuckle stays seated",
    )
    ctx.expect_contact(
        cover,
        housing,
        elem_a="lower_cover_knuckle",
        elem_b="lower_housing_knuckle",
        name="lower hinge knuckle stays seated",
    )
    ctx.expect_contact(
        tray,
        housing,
        elem_a="left_runner",
        elem_b="left_slide_rail",
        name="filter tray left runner sits on its guide rail",
    )
    ctx.expect_contact(
        tray,
        housing,
        elem_a="right_runner",
        elem_b="right_slide_rail",
        name="filter tray right runner sits on its guide rail",
    )
    ctx.expect_gap(
        cover,
        housing,
        axis="x",
        positive_elem="top_rail",
        negative_elem="front_top_lip",
        max_gap=0.003,
        max_penetration=0.0,
        name="closed cover sits nearly flush with the front frame",
    )
    ctx.expect_within(
        tray,
        housing,
        axes="yz",
        inner_elem="filter_media",
        margin=0.020,
        name="filter tray stays laterally centered within the housing envelope",
    )
    ctx.expect_gap(
        tray,
        housing,
        axis="x",
        positive_elem="filter_media",
        negative_elem="back_panel",
        min_gap=0.008,
        max_gap=0.020,
        name="filter media clears the rear housing panel",
    )

    with ctx.pose({cover_hinge: 1.20}):
        ctx.expect_gap(
            cover,
            housing,
            axis="x",
            positive_elem="pull_tab",
            negative_elem="front_left_lip",
            min_gap=0.100,
            name="cover free edge swings clear when opened",
        )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_overlap(
            tray,
            housing,
            axes="x",
            elem_a="left_runner",
            elem_b="left_slide_rail",
            min_overlap=0.025,
            name="extended tray retains left runner engagement",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="x",
            elem_a="right_runner",
            elem_b="right_slide_rail",
            min_overlap=0.025,
            name="extended tray retains right runner engagement",
        )
        extended_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "filter tray slides outward from the housing base",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.060,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
