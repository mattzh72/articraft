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
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


CURB_DEPTH = 0.122
CURB_WIDTH = 0.162
CURB_HEIGHT = 0.018

TOWER_DEPTH = 0.110
TOWER_WIDTH = 0.150
TOWER_HEIGHT = 0.156
TOWER_Z = CURB_HEIGHT + TOWER_HEIGHT / 2.0

INNER_DEPTH = 0.084
INNER_WIDTH = 0.124
INNER_HEIGHT = 0.138
INNER_X = -0.001
INNER_Z = 0.100

OPENING_DEPTH = 0.036
OPENING_WIDTH = 0.094
OPENING_HEIGHT = 0.044
OPENING_X = 0.042
OPENING_Z = 0.114

FRAME_DEPTH = 0.0065
FRAME_OUTER_WIDTH = 0.112
FRAME_OUTER_HEIGHT = 0.060
FRAME_SIDE_THICK = 0.009
FRAME_BAR_THICK = 0.008
FRAME_X = TOWER_DEPTH / 2.0 + FRAME_DEPTH / 2.0

ROOF_SLOT_DEPTH = 0.082
ROOF_SLOT_WIDTH = 0.118
ROOF_SLOT_HEIGHT = 0.030
ROOF_SLOT_X = 0.008
ROOF_SLOT_Z = CURB_HEIGHT + TOWER_HEIGHT - 0.009

ROOF_RIM_DEPTH = 0.008
ROOF_RIM_WIDTH = 0.008
ROOF_RIM_HEIGHT = 0.006
ROOF_RIM_Z = CURB_HEIGHT + TOWER_HEIGHT - ROOF_RIM_HEIGHT / 2.0
ROOF_RIM_FRONT_X = ROOF_SLOT_X + ROOF_SLOT_DEPTH / 2.0 + ROOF_RIM_DEPTH / 2.0
ROOF_RIM_BACK_X = ROOF_SLOT_X - ROOF_SLOT_DEPTH / 2.0 - ROOF_RIM_DEPTH / 2.0
ROOF_RIM_SIDE_Y = ROOF_SLOT_WIDTH / 2.0 + ROOF_RIM_WIDTH / 2.0

HINGE_X = ROOF_RIM_BACK_X
HINGE_Z = CURB_HEIGHT + TOWER_HEIGHT + 0.0065
HINGE_RADIUS = 0.0055
BODY_KNUCKLE_LENGTH = 0.028
FLAP_KNUCKLE_LENGTH = 0.036
BODY_KNUCKLE_Y = 0.032

FLAP_DEPTH = 0.100
FLAP_WIDTH = 0.156
FLAP_THICKNESS = 0.004


def _translated_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> BoxGeometry:
    return BoxGeometry(size).translate(*center)


def _tower_shell_mesh():
    outer = _translated_box((TOWER_DEPTH, TOWER_WIDTH, TOWER_HEIGHT), (0.0, 0.0, TOWER_Z))
    inner = _translated_box((INNER_DEPTH, INNER_WIDTH, INNER_HEIGHT), (INNER_X, 0.0, INNER_Z))
    shell = boolean_difference(outer, inner)
    opening = _translated_box((OPENING_DEPTH, OPENING_WIDTH, OPENING_HEIGHT), (OPENING_X, 0.0, OPENING_Z))
    shell = boolean_difference(shell, opening)
    roof_slot = _translated_box((ROOF_SLOT_DEPTH, ROOF_SLOT_WIDTH, ROOF_SLOT_HEIGHT), (ROOF_SLOT_X, 0.0, ROOF_SLOT_Z))
    return boolean_difference(shell, roof_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower_premium", assets=ASSETS)

    graphite_matte = model.material("graphite_matte", rgba=(0.18, 0.20, 0.22, 1.0))
    curb_matte = model.material("curb_matte", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.69, 0.71, 0.73, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.30, 0.32, 0.35, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.58, 0.61, 1.0))

    tower = model.part("tower_body")
    tower.visual(
        Box((CURB_DEPTH, CURB_WIDTH, CURB_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CURB_HEIGHT / 2.0)),
        material=curb_matte,
        name="curb",
    )
    tower.visual(
        mesh_from_geometry(_tower_shell_mesh(), ASSETS.mesh_path("tower_shell.obj")),
        material=graphite_matte,
        name="tower_shell",
    )

    frame_center_z = OPENING_Z
    top_bar_z = frame_center_z + FRAME_OUTER_HEIGHT / 2.0 - FRAME_BAR_THICK / 2.0
    bottom_bar_z = frame_center_z - FRAME_OUTER_HEIGHT / 2.0 + FRAME_BAR_THICK / 2.0
    side_bar_y = FRAME_OUTER_WIDTH / 2.0 - FRAME_SIDE_THICK / 2.0

    tower.visual(
        Box((FRAME_DEPTH, FRAME_OUTER_WIDTH, FRAME_BAR_THICK)),
        origin=Origin(xyz=(FRAME_X, 0.0, top_bar_z)),
        material=satin_aluminum,
        name="frame_top",
    )
    tower.visual(
        Box((FRAME_DEPTH, FRAME_OUTER_WIDTH, FRAME_BAR_THICK)),
        origin=Origin(xyz=(FRAME_X, 0.0, bottom_bar_z)),
        material=satin_aluminum,
        name="frame_bottom",
    )
    tower.visual(
        Box((FRAME_DEPTH, FRAME_SIDE_THICK, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(FRAME_X, side_bar_y, frame_center_z)),
        material=satin_aluminum,
        name="frame_right",
    )
    tower.visual(
        Box((FRAME_DEPTH, FRAME_SIDE_THICK, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(FRAME_X, -side_bar_y, frame_center_z)),
        material=satin_aluminum,
        name="frame_left",
    )

    band_height = 0.008
    band_z = CURB_HEIGHT + TOWER_HEIGHT - 0.012
    tower.visual(
        Box((0.004, TOWER_WIDTH - 0.010, band_height)),
        origin=Origin(xyz=(TOWER_DEPTH / 2.0 + 0.002, 0.0, band_z)),
        material=satin_charcoal,
        name="top_break_front",
    )
    tower.visual(
        Box((0.004, TOWER_WIDTH - 0.010, band_height)),
        origin=Origin(xyz=(-TOWER_DEPTH / 2.0 - 0.002, 0.0, band_z)),
        material=satin_charcoal,
        name="top_break_back",
    )
    tower.visual(
        Box((TOWER_DEPTH - 0.010, 0.004, band_height)),
        origin=Origin(xyz=(0.0, TOWER_WIDTH / 2.0 + 0.002, band_z)),
        material=satin_charcoal,
        name="top_break_right",
    )
    tower.visual(
        Box((TOWER_DEPTH - 0.010, 0.004, band_height)),
        origin=Origin(xyz=(0.0, -TOWER_WIDTH / 2.0 - 0.002, band_z)),
        material=satin_charcoal,
        name="top_break_left",
    )
    tower.visual(
        Box((ROOF_RIM_DEPTH, ROOF_SLOT_WIDTH + 0.004, ROOF_RIM_HEIGHT)),
        origin=Origin(xyz=(ROOF_RIM_FRONT_X, 0.0, ROOF_RIM_Z)),
        material=satin_aluminum,
        name="roof_rim_front",
    )
    tower.visual(
        Box((ROOF_RIM_DEPTH, ROOF_SLOT_WIDTH + 0.004, ROOF_RIM_HEIGHT)),
        origin=Origin(xyz=(ROOF_RIM_BACK_X, 0.0, ROOF_RIM_Z)),
        material=satin_aluminum,
        name="roof_rim_back",
    )
    tower.visual(
        Box((ROOF_SLOT_DEPTH, ROOF_RIM_WIDTH, ROOF_RIM_HEIGHT)),
        origin=Origin(xyz=(ROOF_SLOT_X, ROOF_RIM_SIDE_Y, ROOF_RIM_Z)),
        material=satin_aluminum,
        name="roof_rim_right",
    )
    tower.visual(
        Box((ROOF_SLOT_DEPTH, ROOF_RIM_WIDTH, ROOF_RIM_HEIGHT)),
        origin=Origin(xyz=(ROOF_SLOT_X, -ROOF_RIM_SIDE_Y, ROOF_RIM_Z)),
        material=satin_aluminum,
        name="roof_rim_left",
    )

    tower.visual(
        Box((0.020, 0.030, 0.004)),
        origin=Origin(xyz=(HINGE_X, -BODY_KNUCKLE_Y, HINGE_Z - HINGE_RADIUS - 0.002)),
        material=satin_charcoal,
        name="hinge_pad_left",
    )
    tower.visual(
        Box((0.020, 0.030, 0.004)),
        origin=Origin(xyz=(HINGE_X, BODY_KNUCKLE_Y, HINGE_Z - HINGE_RADIUS - 0.002)),
        material=satin_charcoal,
        name="hinge_pad_right",
    )
    tower.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_X, -BODY_KNUCKLE_Y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="body_knuckle_left",
    )
    tower.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_X, BODY_KNUCKLE_Y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="body_knuckle_right",
    )
    tower.inertial = Inertial.from_geometry(
        Box((CURB_DEPTH, CURB_WIDTH, CURB_HEIGHT + TOWER_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, (CURB_HEIGHT + TOWER_HEIGHT) / 2.0)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Box((FLAP_DEPTH, FLAP_WIDTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(0.058, 0.0, 0.003)),
        material=satin_charcoal,
        name="flap_panel",
    )
    flap.visual(
        Box((0.072, 0.004, 0.016)),
        origin=Origin(xyz=(0.056, FLAP_WIDTH / 2.0 - 0.0015, -0.004)),
        material=satin_charcoal,
        name="flap_skirt_right",
    )
    flap.visual(
        Box((0.072, 0.004, 0.016)),
        origin=Origin(xyz=(0.056, -FLAP_WIDTH / 2.0 + 0.0015, -0.004)),
        material=satin_charcoal,
        name="flap_skirt_left",
    )
    flap.visual(
        Box((0.012, FLAP_WIDTH - 0.016, 0.010)),
        origin=Origin(xyz=(0.094, 0.0, 0.001)),
        material=satin_aluminum,
        name="flap_front_lip",
    )
    flap.visual(
        Box((0.024, 0.046, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=satin_aluminum,
        name="flap_hinge_leaf",
    )
    flap.visual(
        Cylinder(radius=HINGE_RADIUS, length=FLAP_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="flap_knuckle",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_DEPTH, FLAP_WIDTH, 0.024)),
        mass=0.95,
        origin=Origin(xyz=(0.054, 0.0, 0.000)),
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower_body")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_flap")

    tower_shell = tower.get_visual("tower_shell")
    frame_top = tower.get_visual("frame_top")
    frame_bottom = tower.get_visual("frame_bottom")
    frame_left = tower.get_visual("frame_left")
    frame_right = tower.get_visual("frame_right")
    roof_rim_front = tower.get_visual("roof_rim_front")
    roof_rim_back = tower.get_visual("roof_rim_back")
    roof_rim_left = tower.get_visual("roof_rim_left")
    roof_rim_right = tower.get_visual("roof_rim_right")
    body_knuckle_left = tower.get_visual("body_knuckle_left")
    body_knuckle_right = tower.get_visual("body_knuckle_right")
    flap_panel = flap.get_visual("flap_panel")
    flap_front_lip = flap.get_visual("flap_front_lip")
    flap_knuckle = flap.get_visual("flap_knuckle")
    flap_leaf = flap.get_visual("flap_hinge_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "tower_features_present",
        all(
            feature is not None
            for feature in (
                tower_shell,
                frame_top,
                frame_bottom,
                frame_left,
                frame_right,
                roof_rim_front,
                roof_rim_back,
                roof_rim_left,
                roof_rim_right,
                body_knuckle_left,
                body_knuckle_right,
            )
        ),
        "tower body is missing one or more framed opening or hinge visuals",
    )
    ctx.check(
        "flap_features_present",
        all(feature is not None for feature in (flap_panel, flap_front_lip, flap_knuckle, flap_leaf)),
        "weather flap is missing one or more panel or hinge visuals",
    )
    ctx.check(
        "hinge_axis_is_rear_pivot",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, -1.0, 0.0),
        f"expected a rear-pivot hinge axis, got {hinge.axis}",
    )
    ctx.check(
        "hinge_limits_cover_weather_flap_motion",
        hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-6
        and 0.95 <= hinge.motion_limits.upper <= 1.10,
        f"unexpected hinge limits: {hinge.motion_limits}",
    )
    ctx.expect_origin_distance(flap, tower, axes="y", max_dist=1e-6, name="flap_centered_on_tower")
    ctx.expect_contact(
        flap,
        tower,
        elem_a=flap_knuckle,
        elem_b=body_knuckle_left,
        contact_tol=5e-4,
        name="left_hinge_knuckle_contacts",
    )
    ctx.expect_contact(
        flap,
        tower,
        elem_a=flap_knuckle,
        elem_b=body_knuckle_right,
        contact_tol=5e-4,
        name="right_hinge_knuckle_contacts",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            tower,
            axes="y",
            min_overlap=0.105,
            elem_a=flap_panel,
            elem_b=roof_rim_front,
            name="closed_flap_spans_roof_outlet",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            min_gap=0.001,
            max_gap=0.006,
            positive_elem=flap_front_lip,
            negative_elem=roof_rim_front,
            name="closed_flap_lip_clears_roof_rim",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            min_gap=0.004,
            max_gap=0.014,
            positive_elem=flap_panel,
            negative_elem=tower_shell,
            name="closed_flap_panel_sits_above_shell",
        )

    with ctx.pose({hinge: 0.78}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            min_gap=0.040,
            positive_elem=flap_front_lip,
            negative_elem=roof_rim_front,
            name="open_flap_creates_exhaust_gap",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="y",
            min_overlap=0.105,
            elem_a=flap_panel,
            elem_b=roof_rim_front,
            name="open_flap_stays_laterally_registered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
