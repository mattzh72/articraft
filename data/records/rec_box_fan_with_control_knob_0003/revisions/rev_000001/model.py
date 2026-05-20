from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/tmp")
        return "/tmp"


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())

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

HOUSING_W = 0.46
HOUSING_H = 0.30
HOUSING_D = 0.11
SHELL_T = 0.012

GRILLE_OUTER_W = 0.366
GRILLE_OUTER_H = 0.256
GRILLE_CENTER_Z = -0.010
GRILLE_FRAME_W = 0.012
GRILLE_DEPTH = 0.009
SHROUD_DEPTH = 0.016
SHROUD_FRAME_W = 0.016
SHROUD_OUTER_W = 0.336
SHROUD_OUTER_H = 0.230
SHROUD_CENTER_Y = 0.037

SIDE_MARGIN_W = (HOUSING_W - GRILLE_OUTER_W) / 2.0
FRONT_BAND_D = 0.010
TOP_FRONT_BAND_H = 0.042
BOTTOM_FRONT_BAND_H = 0.028
SLOT_H = 0.046

RAIL_LENGTH = 0.105
OUTER_RAIL_DEPTH = 0.022
OUTER_RAIL_HEIGHT = 0.022
OUTER_RAIL_WALL = 0.003
INNER_RAIL_DEPTH = 0.016
INNER_RAIL_HEIGHT = 0.016
INNER_RAIL_WALL = 0.0025
RAIL_Z = 0.104

PANEL_W = 0.112
PANEL_D = 0.085
PANEL_H = 0.192

TOP_SURFACE_Z = HOUSING_H / 2.0
FRONT_SURFACE_Y = HOUSING_D / 2.0


def _box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder_y(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_x_channel(
    part,
    prefix: str,
    x_center: float,
    z_center: float,
    *,
    length: float,
    depth: float,
    height: float,
    wall: float,
    material=None,
) -> None:
    _box(
        part,
        f"{prefix}_web",
        (length, wall, height),
        (x_center, -depth / 2.0 + wall / 2.0, z_center),
        material=material,
    )
    _box(
        part,
        f"{prefix}_roof",
        (length, depth, wall),
        (x_center, 0.0, z_center + height / 2.0 - wall / 2.0),
        material=material,
    )
    _box(
        part,
        f"{prefix}_floor",
        (length, depth, wall),
        (x_center, 0.0, z_center - height / 2.0 + wall / 2.0),
        material=material,
    )


def _build_side_extension(part, side: str, material) -> None:
    sign = 1.0 if side == "right" else -1.0
    _add_x_channel(
        part,
        f"{side}_inner_top",
        sign * (RAIL_LENGTH / 2.0),
        RAIL_Z,
        length=RAIL_LENGTH,
        depth=INNER_RAIL_DEPTH,
        height=INNER_RAIL_HEIGHT,
        wall=INNER_RAIL_WALL,
        material=material,
    )
    _add_x_channel(
        part,
        f"{side}_inner_bottom",
        sign * (RAIL_LENGTH / 2.0),
        -RAIL_Z,
        length=RAIL_LENGTH,
        depth=INNER_RAIL_DEPTH,
        height=INNER_RAIL_HEIGHT,
        wall=INNER_RAIL_WALL,
        material=material,
    )
    _box(
        part,
        "panel_sheet",
        (PANEL_W, PANEL_D, PANEL_H),
        (sign * (PANEL_W / 2.0), 0.0, 0.0),
        material=material,
    )
    _box(
        part,
        "panel_face_trim",
        (0.008, PANEL_D, PANEL_H),
        (sign * (PANEL_W - 0.004), 0.0, 0.0),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    housing_white = model.material("housing_white", rgba=(0.90, 0.92, 0.93, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.24, 0.27, 0.31, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.72, 0.75, 0.78, 1.0))
    selector_beige = model.material("selector_beige", rgba=(0.86, 0.83, 0.77, 1.0))

    housing = model.part("housing")
    _box(housing, "top_shell", (HOUSING_W, HOUSING_D, SHELL_T), (0.0, 0.0, HOUSING_H / 2.0 - SHELL_T / 2.0), material=housing_white)
    _box(housing, "bottom_shell", (HOUSING_W, HOUSING_D, SHELL_T), (0.0, 0.0, -HOUSING_H / 2.0 + SHELL_T / 2.0), material=housing_white)
    _box(
        housing,
        "left_side_wall",
        (SHELL_T, HOUSING_D, HOUSING_H - 2.0 * SHELL_T),
        (-HOUSING_W / 2.0 + SHELL_T / 2.0, 0.0, 0.0),
        material=housing_white,
    )
    _box(
        housing,
        "right_side_wall",
        (SHELL_T, HOUSING_D, HOUSING_H - 2.0 * SHELL_T),
        (HOUSING_W / 2.0 - SHELL_T / 2.0, 0.0, 0.0),
        material=housing_white,
    )
    _box(
        housing,
        "front_top_band",
        (HOUSING_W, FRONT_BAND_D, TOP_FRONT_BAND_H),
        (0.0, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, 0.129),
        material=housing_white,
    )
    _box(
        housing,
        "front_bottom_band",
        (HOUSING_W, FRONT_BAND_D, BOTTOM_FRONT_BAND_H),
        (0.0, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, -0.136),
        material=housing_white,
    )
    slot_center_x = GRILLE_OUTER_W / 2.0 + SIDE_MARGIN_W / 2.0
    _box(
        housing,
        "left_slot_upper",
        (SIDE_MARGIN_W, FRONT_BAND_D, 0.085),
        (-slot_center_x, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, 0.0655),
        material=housing_white,
    )
    _box(
        housing,
        "left_slot_lower",
        (SIDE_MARGIN_W, FRONT_BAND_D, 0.099),
        (-slot_center_x, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, -0.0725),
        material=housing_white,
    )
    _box(
        housing,
        "right_slot_upper",
        (SIDE_MARGIN_W, FRONT_BAND_D, 0.085),
        (slot_center_x, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, 0.0655),
        material=housing_white,
    )
    _box(
        housing,
        "right_slot_lower",
        (SIDE_MARGIN_W, FRONT_BAND_D, 0.099),
        (slot_center_x, FRONT_SURFACE_Y - FRONT_BAND_D / 2.0, -0.0725),
        material=housing_white,
    )
    _box(
        housing,
        "left_slot_seat",
        (0.018, 0.012, 0.046),
        (-slot_center_x, 0.039, 0.0),
        material=housing_white,
    )
    _box(
        housing,
        "right_slot_seat",
        (0.018, 0.012, 0.046),
        (slot_center_x, 0.039, 0.0),
        material=housing_white,
    )
    _box(
        housing,
        "shroud_top",
        (SHROUD_OUTER_W, SHROUD_DEPTH, SHROUD_FRAME_W),
        (0.0, SHROUD_CENTER_Y, SHROUD_OUTER_H / 2.0 - SHROUD_FRAME_W / 2.0),
        material=housing_white,
    )
    _box(
        housing,
        "shroud_bottom",
        (SHROUD_OUTER_W, SHROUD_DEPTH, SHROUD_FRAME_W),
        (0.0, SHROUD_CENTER_Y, -SHROUD_OUTER_H / 2.0 + SHROUD_FRAME_W / 2.0),
        material=housing_white,
    )
    _box(
        housing,
        "shroud_left",
        (SHROUD_FRAME_W, SHROUD_DEPTH, SHROUD_OUTER_H - 2.0 * SHROUD_FRAME_W),
        (-SHROUD_OUTER_W / 2.0 + SHROUD_FRAME_W / 2.0, SHROUD_CENTER_Y, 0.0),
        material=housing_white,
    )
    _box(
        housing,
        "shroud_right",
        (SHROUD_FRAME_W, SHROUD_DEPTH, SHROUD_OUTER_H - 2.0 * SHROUD_FRAME_W),
        (SHROUD_OUTER_W / 2.0 - SHROUD_FRAME_W / 2.0, SHROUD_CENTER_Y, 0.0),
        material=housing_white,
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        rail_center_x = sign * (HOUSING_W / 2.0 + RAIL_LENGTH / 2.0 - 0.005)
        _add_x_channel(
            housing,
            f"{side}_outer_top",
            rail_center_x,
            RAIL_Z,
            length=RAIL_LENGTH,
            depth=OUTER_RAIL_DEPTH,
            height=OUTER_RAIL_HEIGHT,
            wall=OUTER_RAIL_WALL,
            material=aluminum,
        )
        _add_x_channel(
            housing,
            f"{side}_outer_bottom",
            rail_center_x,
            -RAIL_Z,
            length=RAIL_LENGTH,
            depth=OUTER_RAIL_DEPTH,
            height=OUTER_RAIL_HEIGHT,
            wall=OUTER_RAIL_WALL,
            material=aluminum,
        )
    _box(housing, "rear_motor_post", (0.014, 0.018, HOUSING_H - 2.0 * SHELL_T), (0.0, -0.030, 0.0), material=grille_dark)
    _cylinder_y(housing, "bearing_block", 0.016, 0.028, (0.0, -0.032, 0.0), material=grille_dark)
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    controls = model.part("control_bank")
    _box(controls, "bank_base", (0.18, 0.050, 0.008), (0.0, 0.0, 0.004), material=selector_beige)
    _box(controls, "bank_front_lip", (0.18, 0.010, 0.006), (0.0, 0.020, 0.011), material=selector_beige)
    selector_xs = (-0.052, -0.017, 0.018, 0.053)
    selector_heights = (0.012, 0.010, 0.014, 0.010)
    for index, (x_pos, tab_h) in enumerate(zip(selector_xs, selector_heights), start=1):
        _box(
            controls,
            f"selector_{index}",
            (0.018, 0.024, tab_h),
            (x_pos, 0.000, 0.008 + tab_h / 2.0),
            material=selector_beige,
        )
    controls.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.025)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
    )

    grille = model.part("front_grille")
    _box(grille, "frame_top", (GRILLE_OUTER_W, GRILLE_DEPTH, GRILLE_FRAME_W), (0.0, GRILLE_DEPTH / 2.0, GRILLE_OUTER_H / 2.0 - GRILLE_FRAME_W / 2.0), material=grille_dark)
    _box(grille, "frame_bottom", (GRILLE_OUTER_W, GRILLE_DEPTH, GRILLE_FRAME_W), (0.0, GRILLE_DEPTH / 2.0, -GRILLE_OUTER_H / 2.0 + GRILLE_FRAME_W / 2.0), material=grille_dark)
    _box(grille, "frame_left", (GRILLE_FRAME_W, GRILLE_DEPTH, GRILLE_OUTER_H - 2.0 * GRILLE_FRAME_W), (-GRILLE_OUTER_W / 2.0 + GRILLE_FRAME_W / 2.0, GRILLE_DEPTH / 2.0, 0.0), material=grille_dark)
    _box(grille, "frame_right", (GRILLE_FRAME_W, GRILLE_DEPTH, GRILLE_OUTER_H - 2.0 * GRILLE_FRAME_W), (GRILLE_OUTER_W / 2.0 - GRILLE_FRAME_W / 2.0, GRILLE_DEPTH / 2.0, 0.0), material=grille_dark)
    slat_span_h = GRILLE_OUTER_H - 2.0 * GRILLE_FRAME_W
    for index, x_pos in enumerate((-0.130, -0.095, -0.060, -0.025, 0.010, 0.045, 0.080, 0.115), start=1):
        _box(grille, f"vertical_slat_{index}", (0.005, 0.004, slat_span_h), (x_pos, 0.0045, 0.0), material=grille_dark)
    for name, z_pos in (
        ("crossbar_top", 0.086),
        ("crossbar_upper_mid", 0.043),
        ("crossbar_mid", 0.0),
        ("crossbar_lower_mid", -0.043),
        ("crossbar_bottom", -0.086),
    ):
        _box(grille, name, (GRILLE_OUTER_W - 2.0 * GRILLE_FRAME_W, 0.004, 0.004), (0.0, 0.0045, z_pos), material=grille_dark)
    _box(grille, "left_tab", (0.032, 0.012, 0.040), (-0.199, -0.004, 0.0), material=grille_dark)
    _box(grille, "right_tab", (0.032, 0.012, 0.040), (0.199, -0.004, 0.0), material=grille_dark)
    grille.inertial = Inertial.from_geometry(
        Box((GRILLE_OUTER_W, 0.012, GRILLE_OUTER_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_panel = model.part("left_extension_panel")
    _build_side_extension(left_panel, "left", aluminum)
    left_panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_D, PANEL_H)),
        mass=0.55,
        origin=Origin(xyz=(-PANEL_W / 2.0, 0.0, 0.0)),
    )

    right_panel = model.part("right_extension_panel")
    _build_side_extension(right_panel, "right", aluminum)
    right_panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_D, PANEL_H)),
        mass=0.55,
        origin=Origin(xyz=(PANEL_W / 2.0, 0.0, 0.0)),
    )

    rotor = model.part("blade_assembly")
    _cylinder_y(rotor, "shaft", 0.008, 0.026, (0.0, 0.013, 0.0), material=blade_gray)
    _cylinder_y(rotor, "hub_shell", 0.030, 0.020, (0.0, 0.032, 0.0), material=blade_gray)
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        _box(
            rotor,
            f"blade_{index}",
            (0.112, 0.006, 0.034),
            (0.0, 0.031, 0.0),
            rpy=(0.0, angle, 0.0),
            material=blade_gray,
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.060),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_controls",
        ArticulationType.FIXED,
        parent=housing,
        child=controls,
        origin=Origin(xyz=(0.0, 0.018, TOP_SURFACE_Z)),
    )
    model.articulation(
        "housing_to_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(0.0, FRONT_SURFACE_Y, GRILLE_CENTER_Z)),
    )
    model.articulation(
        "housing_to_left_panel",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_panel,
        origin=Origin(xyz=(-HOUSING_W / 2.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=0.08),
    )
    model.articulation(
        "housing_to_right_panel",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_panel,
        origin=Origin(xyz=(HOUSING_W / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=0.08),
    )
    model.articulation(
        "housing_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=16.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root="/tmp")
    housing = object_model.get_part("housing")
    controls = object_model.get_part("control_bank")
    grille = object_model.get_part("front_grille")
    left_panel = object_model.get_part("left_extension_panel")
    right_panel = object_model.get_part("right_extension_panel")
    rotor = object_model.get_part("blade_assembly")

    controls_joint = object_model.get_articulation("housing_to_controls")
    grille_joint = object_model.get_articulation("housing_to_grille")
    left_slide = object_model.get_articulation("housing_to_left_panel")
    right_slide = object_model.get_articulation("housing_to_right_panel")
    rotor_spin = object_model.get_articulation("housing_to_blade_assembly")

    top_shell = housing.get_visual("top_shell")
    front_top_band = housing.get_visual("front_top_band")
    left_side_wall = housing.get_visual("left_side_wall")
    right_side_wall = housing.get_visual("right_side_wall")
    left_slot_seat = housing.get_visual("left_slot_seat")
    right_slot_seat = housing.get_visual("right_slot_seat")
    left_outer_top_roof = housing.get_visual("left_outer_top_roof")
    left_outer_bottom_roof = housing.get_visual("left_outer_bottom_roof")
    right_outer_top_roof = housing.get_visual("right_outer_top_roof")
    right_outer_bottom_roof = housing.get_visual("right_outer_bottom_roof")
    bearing_block = housing.get_visual("bearing_block")
    shroud_top = housing.get_visual("shroud_top")

    bank_base = controls.get_visual("bank_base")
    selector_1 = controls.get_visual("selector_1")
    selector_4 = controls.get_visual("selector_4")

    frame_top = grille.get_visual("frame_top")
    left_tab = grille.get_visual("left_tab")
    right_tab = grille.get_visual("right_tab")
    crossbar_mid = grille.get_visual("crossbar_mid")

    left_sheet = left_panel.get_visual("panel_sheet")
    right_sheet = right_panel.get_visual("panel_sheet")
    left_inner_top_roof = left_panel.get_visual("left_inner_top_roof")
    left_inner_bottom_roof = left_panel.get_visual("left_inner_bottom_roof")
    right_inner_top_roof = right_panel.get_visual("right_inner_top_roof")
    right_inner_bottom_roof = right_panel.get_visual("right_inner_bottom_roof")

    shaft = rotor.get_visual("shaft")
    hub_shell = rotor.get_visual("hub_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.075)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        controls,
        housing,
        axis="z",
        positive_elem=bank_base,
        negative_elem=top_shell,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        controls,
        housing,
        axes="xy",
        elem_a=bank_base,
        elem_b=top_shell,
        min_overlap=0.04,
    )
    ctx.expect_within(controls, housing, axes="xy", inner_elem=selector_1, outer_elem=top_shell)
    ctx.expect_within(controls, housing, axes="xy", inner_elem=selector_4, outer_elem=top_shell)

    ctx.expect_gap(
        grille,
        housing,
        axis="y",
        positive_elem=frame_top,
        negative_elem=front_top_band,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        grille,
        housing,
        axis="y",
        positive_elem=frame_top,
        negative_elem=shroud_top,
        min_gap=0.009,
    )
    ctx.expect_overlap(grille, housing, axes="xz", min_overlap=0.20)
    ctx.expect_gap(
        grille,
        housing,
        axis="y",
        positive_elem=left_tab,
        negative_elem=left_slot_seat,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(grille, housing, elem_a=left_tab, elem_b=left_slot_seat)
    ctx.expect_gap(
        grille,
        housing,
        axis="y",
        positive_elem=right_tab,
        negative_elem=right_slot_seat,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(grille, housing, elem_a=right_tab, elem_b=right_slot_seat)
    ctx.expect_overlap(grille, housing, axes="xz", elem_a=left_tab, elem_b=left_slot_seat, min_overlap=0.010)
    ctx.expect_overlap(grille, housing, axes="xz", elem_a=right_tab, elem_b=right_slot_seat, min_overlap=0.010)

    ctx.expect_gap(
        right_panel,
        housing,
        axis="x",
        positive_elem=right_sheet,
        negative_elem=right_side_wall,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        housing,
        left_panel,
        axis="x",
        positive_elem=left_side_wall,
        negative_elem=left_sheet,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(right_panel, housing, axes="yz", elem_a=right_sheet, elem_b=right_side_wall, min_overlap=0.080)
    ctx.expect_overlap(left_panel, housing, axes="yz", elem_a=left_sheet, elem_b=left_side_wall, min_overlap=0.080)

    ctx.expect_gap(
        rotor,
        housing,
        axis="y",
        positive_elem=shaft,
        negative_elem=bearing_block,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(rotor, housing, elem_a=shaft, elem_b=bearing_block)
    ctx.expect_origin_distance(rotor, housing, axes="xz", max_dist=0.001)
    ctx.expect_gap(
        grille,
        rotor,
        axis="y",
        positive_elem=crossbar_mid,
        negative_elem=hub_shell,
        min_gap=0.020,
    )
    ctx.expect_overlap(rotor, grille, axes="xz", elem_a=hub_shell, min_overlap=0.050)

    with ctx.pose({left_slide: 0.060, right_slide: 0.060}):
        ctx.expect_gap(
            right_panel,
            housing,
            axis="x",
            positive_elem=right_sheet,
            negative_elem=right_side_wall,
            min_gap=0.055,
        )
        ctx.expect_gap(
            housing,
            left_panel,
            axis="x",
            positive_elem=left_side_wall,
            negative_elem=left_sheet,
            min_gap=0.055,
        )
        ctx.expect_gap(
            housing,
            right_panel,
            axis="z",
            positive_elem=right_outer_top_roof,
            negative_elem=right_inner_top_roof,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_contact(housing, right_panel, elem_a=right_outer_top_roof, elem_b=right_inner_top_roof)
        ctx.expect_gap(
            housing,
            right_panel,
            axis="z",
            positive_elem=right_outer_bottom_roof,
            negative_elem=right_inner_bottom_roof,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_contact(housing, right_panel, elem_a=right_outer_bottom_roof, elem_b=right_inner_bottom_roof)
        ctx.expect_gap(
            housing,
            left_panel,
            axis="z",
            positive_elem=left_outer_top_roof,
            negative_elem=left_inner_top_roof,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_contact(housing, left_panel, elem_a=left_outer_top_roof, elem_b=left_inner_top_roof)
        ctx.expect_gap(
            housing,
            left_panel,
            axis="z",
            positive_elem=left_outer_bottom_roof,
            negative_elem=left_inner_bottom_roof,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_contact(housing, left_panel, elem_a=left_outer_bottom_roof, elem_b=left_inner_bottom_roof)

    with ctx.pose({rotor_spin: 1.3}):
        ctx.expect_gap(
            grille,
            rotor,
            axis="y",
            positive_elem=crossbar_mid,
            negative_elem=hub_shell,
            min_gap=0.020,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
