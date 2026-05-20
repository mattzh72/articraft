from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


KEY_PITCH = 0.019
KEY_GAP = 0.0025
KEY_DEPTH = 0.018
KEY_LOWER_HEIGHT = 0.0050
KEY_TOP_HEIGHT = 0.0018
KEY_ARM_DEPTH = 0.010
KEY_ARM_HEIGHT = 0.010
KEY_ARM_Y = KEY_DEPTH * 0.32
KEY_ARM_Z = -0.0025
KEY_TAB_DEPTH = 0.004
KEY_TAB_HEIGHT = 0.0015
KEY_TAB_Y = KEY_DEPTH * 0.39
KEY_TAB_Z = -0.00825

CASE_WIDTH = 0.350
CASE_DEPTH = 0.146
CASE_FRONT_HEIGHT = 0.021
CASE_SIDE_HEIGHT = 0.024
CASE_REAR_HEIGHT = 0.029
TOP_RING_Z = 0.0215

ROW_TOTAL_UNITS = 13.75
MAIN_CLUSTER_CENTER_X = 0.010
MAIN_LEFT_EDGE = MAIN_CLUSTER_CENTER_X - (ROW_TOTAL_UNITS * KEY_PITCH * 0.5)
MACRO_COLUMN_CENTER_X = MAIN_LEFT_EDGE - 0.019

ROW_SPECS = (
    {
        "name": "number",
        "y": 0.040,
        "z": 0.0258,
        "macro": None,
        "main": (
            ("esc_key", 1.0),
            ("num_1_key", 1.0),
            ("num_2_key", 1.0),
            ("num_3_key", 1.0),
            ("num_4_key", 1.0),
            ("num_5_key", 1.0),
            ("num_6_key", 1.0),
            ("num_7_key", 1.0),
            ("num_8_key", 1.0),
            ("num_9_key", 1.0),
            ("num_0_key", 1.0),
            ("minus_key", 1.0),
            ("backspace_key", 1.75),
        ),
    },
    {
        "name": "upper_alpha",
        "y": 0.021,
        "z": 0.0250,
        "macro": ("macro_1_key", 1.0),
        "main": (
            ("tab_key", 1.5),
            ("q_key", 1.0),
            ("w_key", 1.0),
            ("e_key", 1.0),
            ("r_key", 1.0),
            ("t_key", 1.0),
            ("y_key", 1.0),
            ("u_key", 1.0),
            ("i_key", 1.0),
            ("o_key", 1.0),
            ("p_key", 1.0),
            ("l_bracket_key", 1.0),
            ("backslash_key", 1.25),
        ),
    },
    {
        "name": "home",
        "y": 0.002,
        "z": 0.0242,
        "macro": ("macro_2_key", 1.0),
        "main": (
            ("caps_key", 1.75),
            ("a_key", 1.0),
            ("s_key", 1.0),
            ("d_key", 1.0),
            ("f_key", 1.0),
            ("g_key", 1.0),
            ("h_key", 1.0),
            ("j_key", 1.0),
            ("k_key", 1.0),
            ("l_key", 1.0),
            ("semicolon_key", 1.0),
            ("enter_key", 2.0),
        ),
    },
    {
        "name": "lower_alpha",
        "y": -0.017,
        "z": 0.0234,
        "macro": ("macro_3_key", 1.0),
        "main": (
            ("left_shift_key", 2.25),
            ("z_key", 1.0),
            ("x_key", 1.0),
            ("c_key", 1.0),
            ("v_key", 1.0),
            ("b_key", 1.0),
            ("n_key", 1.0),
            ("m_key", 1.0),
            ("comma_key", 1.0),
            ("period_key", 1.0),
            ("right_shift_key", 1.5),
            ("arrow_up_key", 1.0),
        ),
    },
    {
        "name": "bottom",
        "y": -0.036,
        "z": 0.0227,
        "macro": ("macro_4_key", 1.0),
        "main": (
            ("left_ctrl_key", 1.25),
            ("left_alt_key", 1.25),
            ("left_meta_key", 1.25),
            ("space_key", 3.25),
            ("right_meta_key", 1.25),
            ("fn_key", 1.25),
            ("right_ctrl_key", 1.25),
            ("arrow_left_key", 1.0),
            ("arrow_down_key", 1.0),
            ("arrow_right_key", 1.0),
        ),
    },
)

KNOB_CENTER_X = 0.123
KNOB_CENTER_Y = 0.024
KNOB_JOINT_Z = 0.026

FOOT_HINGE_Y = 0.049
FOOT_HINGE_Z = 0.0
LEFT_FOOT_X = -0.108
RIGHT_FOOT_X = 0.108


def _units_to_cap_width(units: float) -> float:
    return (units * KEY_PITCH) - KEY_GAP


def _row_key_centers(items: tuple[tuple[str, float], ...], *, left_edge: float) -> list[tuple[str, float, float]]:
    centers: list[tuple[str, float, float]] = []
    cursor = left_edge
    for key_name, units in items:
        center_x = cursor + (units * KEY_PITCH * 0.5)
        centers.append((key_name, center_x, units))
        cursor += units * KEY_PITCH
    return centers


def _top_bezel_mesh():
    outer = rounded_rect_profile(CASE_WIDTH, CASE_DEPTH, 0.012, corner_segments=10)
    inner = rounded_rect_profile(CASE_WIDTH - 0.022, CASE_DEPTH - 0.022, 0.008, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            0.003,
            cap=True,
            center=True,
            closed=True,
        ),
        "compact_programming_keyboard_top_bezel",
    )


def _add_key_part(
    model: ArticulatedObject,
    case,
    *,
    part_name: str,
    center_x: float,
    center_y: float,
    rest_z: float,
    units: float,
    cap_material,
    switch_material,
) -> None:
    cap_width = _units_to_cap_width(units)
    arm_width = 0.004 if units <= 1.5 else min(0.008, cap_width * 0.34)
    tab_width = 0.010 if units <= 1.5 else min(0.026, cap_width * 0.55)

    key_part = model.part(part_name)
    key_part.visual(
        Box((cap_width, KEY_DEPTH, KEY_LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, KEY_LOWER_HEIGHT * 0.5)),
        material=cap_material,
        name="cap_lower",
    )
    key_part.visual(
        Box((max(cap_width - 0.0035, cap_width * 0.82), KEY_DEPTH - 0.003, KEY_TOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, KEY_LOWER_HEIGHT + (KEY_TOP_HEIGHT * 0.5))),
        material=cap_material,
        name="cap_top",
    )
    key_part.visual(
        Box((arm_width, KEY_ARM_DEPTH, KEY_ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, KEY_ARM_Y, KEY_ARM_Z)),
        material=switch_material,
        name="rear_arm",
    )
    key_part.visual(
        Box((tab_width, KEY_TAB_DEPTH, KEY_TAB_HEIGHT)),
        origin=Origin(xyz=(0.0, KEY_TAB_Y, KEY_TAB_Z)),
        material=switch_material,
        name="retainer_tab",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((cap_width, KEY_DEPTH, 0.014)),
        mass=0.008 + (0.004 * units),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        f"case_to_{part_name}",
        ArticulationType.PRISMATIC,
        parent=case,
        child=key_part,
        origin=Origin(xyz=(center_x, center_y, rest_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.05,
            lower=0.0,
            upper=0.0032 if units <= 1.25 else 0.0028,
        ),
    )


def _key_material_for_name(name: str, *, standard, macro, arrow):
    if name.startswith("macro_"):
        return macro
    if name.startswith("arrow_"):
        return arrow
    return standard


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_programming_keyboard")

    case_shell = model.material("case_shell", rgba=(0.16, 0.17, 0.18, 1.0))
    case_trim = model.material("case_trim", rgba=(0.09, 0.10, 0.11, 1.0))
    key_standard = model.material("key_standard", rgba=(0.84, 0.86, 0.88, 1.0))
    key_macro = model.material("key_macro", rgba=(0.38, 0.45, 0.49, 1.0))
    key_arrow = model.material("key_arrow", rgba=(0.74, 0.77, 0.80, 1.0))
    switch_dark = model.material("switch_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    shaft_metal = model.material("shaft_metal", rgba=(0.60, 0.61, 0.64, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.19, 0.20, 0.21, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=case_shell,
        name="bottom_plate",
    )
    case.visual(
        Box((CASE_WIDTH, 0.008, CASE_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.0, -(CASE_DEPTH * 0.5) + 0.004, CASE_FRONT_HEIGHT * 0.5)),
        material=case_shell,
        name="front_wall",
    )
    case.visual(
        Box((CASE_WIDTH, 0.008, CASE_REAR_HEIGHT)),
        origin=Origin(xyz=(0.0, (CASE_DEPTH * 0.5) - 0.004, CASE_REAR_HEIGHT * 0.5)),
        material=case_shell,
        name="rear_wall",
    )
    case.visual(
        Box((0.008, CASE_DEPTH, CASE_SIDE_HEIGHT)),
        origin=Origin(xyz=(-(CASE_WIDTH * 0.5) + 0.004, 0.0, CASE_SIDE_HEIGHT * 0.5)),
        material=case_shell,
        name="left_wall",
    )
    case.visual(
        Box((0.008, CASE_DEPTH, CASE_SIDE_HEIGHT)),
        origin=Origin(xyz=((CASE_WIDTH * 0.5) - 0.004, 0.0, CASE_SIDE_HEIGHT * 0.5)),
        material=case_shell,
        name="right_wall",
    )
    case.visual(
        _top_bezel_mesh(),
        origin=Origin(xyz=(0.0, 0.0, TOP_RING_Z)),
        material=case_trim,
        name="top_bezel",
    )
    case.visual(
        Box((CASE_WIDTH - 0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.064, 0.019)),
        material=case_trim,
        name="front_lip",
    )
    case.visual(
        Box((CASE_WIDTH - 0.020, 0.018, 0.005)),
        origin=Origin(xyz=(0.0, 0.057, 0.0235)),
        material=case_shell,
        name="rear_deck",
    )
    case.visual(
        Box((CASE_WIDTH - 0.016, 0.106, 0.002)),
        origin=Origin(xyz=(0.0, -0.002, 0.011)),
        material=case_trim,
        name="key_cavity_floor",
    )
    case.visual(
        Box((0.004, 0.102, 0.018)),
        origin=Origin(xyz=(-0.128, -0.001, 0.013)),
        material=case_trim,
        name="macro_gutter_wall",
    )
    case.visual(
        Box((0.004, 0.054, 0.017)),
        origin=Origin(xyz=(0.088, -0.028, 0.0125)),
        material=case_trim,
        name="arrow_cluster_divider",
    )
    case.visual(
        Box((0.060, 0.010, 0.004)),
        origin=Origin(xyz=(0.124, 0.009, 0.023)),
        material=case_shell,
        name="knob_shelf_front",
    )
    case.visual(
        Box((0.060, 0.038, 0.004)),
        origin=Origin(xyz=(0.124, 0.050, 0.023)),
        material=case_shell,
        name="knob_shelf_back",
    )
    case.visual(
        Box((0.016, 0.024, 0.004)),
        origin=Origin(xyz=(0.099, 0.023, 0.023)),
        material=case_shell,
        name="knob_shelf_left",
    )
    case.visual(
        Box((0.016, 0.024, 0.004)),
        origin=Origin(xyz=(0.149, 0.023, 0.023)),
        material=case_shell,
        name="knob_shelf_right",
    )
    case.visual(
        Box((0.024, 0.003, 0.006)),
        origin=Origin(xyz=(KNOB_CENTER_X, KNOB_CENTER_Y - 0.007, 0.026)),
        material=case_trim,
        name="knob_pod_front",
    )
    case.visual(
        Box((0.024, 0.003, 0.006)),
        origin=Origin(xyz=(KNOB_CENTER_X, KNOB_CENTER_Y + 0.007, 0.026)),
        material=case_trim,
        name="knob_pod_back",
    )
    case.visual(
        Box((0.003, 0.014, 0.006)),
        origin=Origin(xyz=(KNOB_CENTER_X - 0.0095, KNOB_CENTER_Y, 0.026)),
        material=case_trim,
        name="knob_pod_left",
    )
    case.visual(
        Box((0.003, 0.014, 0.006)),
        origin=Origin(xyz=(KNOB_CENTER_X + 0.0095, KNOB_CENTER_Y, 0.026)),
        material=case_trim,
        name="knob_pod_right",
    )
    case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_REAR_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, CASE_REAR_HEIGHT * 0.5)),
    )

    key_specs: list[dict[str, float | str]] = []
    for row in ROW_SPECS:
        row_y = row["y"]
        row_z = row["z"]
        if row["macro"] is not None:
            macro_name, macro_units = row["macro"]
            key_specs.append(
                {
                    "name": macro_name,
                    "x": MACRO_COLUMN_CENTER_X,
                    "y": row_y,
                    "z": row_z,
                    "units": macro_units,
                }
            )
        for key_name, center_x, units in _row_key_centers(row["main"], left_edge=MAIN_LEFT_EDGE):
            key_specs.append({"name": key_name, "x": center_x, "y": row_y, "z": row_z, "units": units})

        case.visual(
            Box((CASE_WIDTH - 0.016, 0.004, 0.0025)),
            origin=Origin(xyz=(0.0, row_y + KEY_TAB_Y, row_z - 0.00625)),
            material=case_trim,
            name=f"{row['name']}_retainer_bar",
        )

    for key in key_specs:
        key_name = str(key["name"])
        _add_key_part(
            model,
            case,
            part_name=key_name,
            center_x=float(key["x"]),
            center_y=float(key["y"]),
            rest_z=float(key["z"]),
            units=float(key["units"]),
            cap_material=_key_material_for_name(
                key_name,
                standard=key_standard,
                macro=key_macro,
                arrow=key_arrow,
            ),
            switch_material=switch_dark,
        )

    media_knob = model.part("media_knob")
    media_knob.visual(
        Cylinder(radius=0.0035, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shaft_metal,
        name="shaft",
    )
    media_knob.visual(
        Cylinder(radius=0.0082, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=shaft_metal,
        name="retaining_clip",
    )
    media_knob.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=knob_dark,
        name="knob_skirt",
    )
    media_knob.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=knob_dark,
        name="knob_cap",
    )
    media_knob.visual(
        Box((0.003, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0105, 0.0, 0.0235)),
        material=shaft_metal,
        name="indicator_bar",
    )
    media_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.020),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    model.articulation(
        "case_to_media_knob",
        ArticulationType.REVOLUTE,
        parent=case,
        child=media_knob,
        origin=Origin(xyz=(KNOB_CENTER_X, KNOB_CENTER_Y, KNOB_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    for foot_name, foot_x in (("rear_foot_left", LEFT_FOOT_X), ("rear_foot_right", RIGHT_FOOT_X)):
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.003, length=0.018),
            origin=Origin(xyz=(0.0, 0.001, -0.003), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=foot_dark,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.020, 0.022, 0.004)),
            origin=Origin(xyz=(0.0, 0.011, -0.002)),
            material=foot_dark,
            name="foot_blade",
        )
        foot.visual(
            Box((0.018, 0.006, 0.002)),
            origin=Origin(xyz=(0.0, 0.020, -0.003)),
            material=rubber_black,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.020, 0.024, 0.006)),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.010, -0.002)),
        )
        model.articulation(
            f"case_to_{foot_name}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(foot_x, FOOT_HINGE_Y, FOOT_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.12,
                velocity=2.5,
                lower=-1.15,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    macro_2_key = object_model.get_part("macro_2_key")
    a_key = object_model.get_part("a_key")
    arrow_up_key = object_model.get_part("arrow_up_key")
    arrow_down_key = object_model.get_part("arrow_down_key")
    media_knob = object_model.get_part("media_knob")
    rear_foot_left = object_model.get_part("rear_foot_left")
    rear_foot_right = object_model.get_part("rear_foot_right")

    sample_key_joint = object_model.get_articulation("case_to_a_key")
    media_knob_joint = object_model.get_articulation("case_to_media_knob")
    left_foot_joint = object_model.get_articulation("case_to_rear_foot_left")
    right_foot_joint = object_model.get_articulation("case_to_rear_foot_right")

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

    ctx.check(
        "sample_key_prismatic_vertical",
        sample_key_joint.joint_type == ArticulationType.PRISMATIC
        and tuple(round(value, 3) for value in sample_key_joint.axis) == (0.0, 0.0, -1.0),
        f"expected vertical prismatic key travel, got {sample_key_joint.joint_type} axis={sample_key_joint.axis}",
    )
    ctx.check(
        "media_knob_revolves_on_local_axis",
        media_knob_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(round(value, 3) for value in media_knob_joint.axis) == (0.0, 0.0, 1.0),
        f"expected knob revolute z-axis, got {media_knob_joint.joint_type} axis={media_knob_joint.axis}",
    )
    ctx.check(
        "rear_feet_hinge_on_x_axis",
        left_foot_joint.joint_type == ArticulationType.REVOLUTE
        and right_foot_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(round(value, 3) for value in left_foot_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(round(value, 3) for value in right_foot_joint.axis) == (1.0, 0.0, 0.0),
        f"left axis={left_foot_joint.axis}, right axis={right_foot_joint.axis}",
    )

    ctx.expect_contact(macro_2_key, case, name="macro_key_is_captured_by_case")
    ctx.expect_contact(arrow_down_key, case, name="arrow_key_is_captured_by_case")
    ctx.expect_contact(rear_foot_left, case, name="left_rear_foot_stows_against_case")
    ctx.expect_contact(rear_foot_right, case, name="right_rear_foot_stows_against_case")
    ctx.expect_origin_gap(a_key, macro_2_key, axis="x", min_gap=0.030, max_gap=0.070, name="macro_column_sits_left_of_alpha_keys")
    ctx.expect_origin_gap(media_knob, arrow_up_key, axis="y", min_gap=0.025, max_gap=0.060, name="media_knob_above_arrow_cluster")
    ctx.expect_origin_distance(media_knob, arrow_down_key, axes="x", max_dist=0.030, name="media_knob_centered_over_arrow_cluster")

    with ctx.pose({media_knob_joint: 1.6}):
        ctx.expect_contact(media_knob, case, name="media_knob_remains_clipped_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
