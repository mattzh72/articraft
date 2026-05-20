from __future__ import annotations

import os as _bootstrap_os

try:
    _bootstrap_os.getcwd()
except FileNotFoundError:
    _bootstrap_os.chdir("/")

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _profile_loop(profile, z: float):
    return [(x, y, z) for x, y in profile]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _phone_shell_mesh(
    *,
    name: str,
    width: float,
    length: float,
    thickness: float,
    edge_radius: float,
    face_inset: float,
):
    face_profile = rounded_rect_profile(
        width - face_inset,
        length - face_inset,
        max(edge_radius - (face_inset * 0.45), edge_radius * 0.75),
        corner_segments=10,
    )
    mid_profile = rounded_rect_profile(
        width,
        length,
        edge_radius,
        corner_segments=10,
    )
    return _save_mesh(
        name,
        LoftGeometry(
            [
                _profile_loop(face_profile, -(thickness * 0.5)),
                _profile_loop(mid_profile, 0.0),
                _profile_loop(face_profile, thickness * 0.5),
            ],
            cap=True,
            closed=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_flip_phone", assets=ASSETS)

    gloss_black = model.material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.20, 0.22, 1.0))
    button_gray = model.material("button_gray", rgba=(0.26, 0.27, 0.30, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.15, 0.18, 0.95))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.88, 1.0))

    body_width = 0.057
    body_length = 0.094
    body_thickness = 0.0088
    body_edge_radius = 0.0074
    face_inset = 0.0018
    rear_offset = 0.0052
    shell_center_y = -(rear_offset + (body_length * 0.5))
    closed_gap = 0.0022
    shell_center_z = (closed_gap * 0.5) + (body_thickness * 0.5)
    lower_interface_z = -(closed_gap * 0.5)
    upper_interface_z = closed_gap * 0.5

    hinge_radius = 0.0044
    side_knuckle_length = 0.014
    center_barrel_length = 0.022
    hinge_clearance = 0.0
    chrome_pin_length = 0.0022
    pin_embed = 0.0006
    knuckle_center_x = (center_barrel_length * 0.5) + hinge_clearance + (side_knuckle_length * 0.5)
    pin_center_x = (
        (center_barrel_length * 0.5)
        + hinge_clearance
        + side_knuckle_length
        + (chrome_pin_length * 0.5)
        - pin_embed
    )

    shell_mesh = _phone_shell_mesh(
        name="flip_phone_shell.obj",
        width=body_width,
        length=body_length,
        thickness=body_thickness,
        edge_radius=body_edge_radius,
        face_inset=face_inset,
    )

    key_height = 0.0010
    key_center_z = lower_interface_z + (key_height * 0.5)
    trim_height = 0.0006
    keypad_plate_z = lower_interface_z + (trim_height * 0.5)
    screen_bezel_height = 0.0008
    screen_bezel_z = upper_interface_z + (screen_bezel_height * 0.5)
    screen_glass_height = 0.0005
    screen_glass_z = upper_interface_z + 0.00045

    lower = model.part("lower_half")
    lower.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, shell_center_y, -shell_center_z)),
        material=gloss_black,
        name="lower_shell",
    )
    lower.visual(
        Box((0.043, 0.068, trim_height)),
        origin=Origin(xyz=(0.0, -0.056, keypad_plate_z)),
        material=satin_black,
        name="keypad_plate",
    )
    lower.visual(
        Box((0.044, 0.0068, 0.0050)),
        origin=Origin(xyz=(0.0, -0.0022, -0.0026)),
        material=gloss_black,
        name="lower_hinge_bridge",
    )
    lower.visual(
        Cylinder(radius=0.0105, length=0.0010),
        origin=Origin(xyz=(0.0, -0.026, key_center_z), rpy=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="nav_ring",
    )
    lower.visual(
        Cylinder(radius=0.0042, length=0.0012),
        origin=Origin(xyz=(0.0, -0.026, lower_interface_z + 0.0006)),
        material=button_gray,
        name="nav_select",
    )
    lower.visual(
        Box((0.010, 0.005, key_height)),
        origin=Origin(xyz=(-0.012, -0.013, key_center_z)),
        material=button_gray,
        name="soft_key_left",
    )
    lower.visual(
        Box((0.010, 0.005, key_height)),
        origin=Origin(xyz=(0.012, -0.013, key_center_z)),
        material=button_gray,
        name="soft_key_right",
    )
    lower.visual(
        Box((0.010, 0.005, key_height)),
        origin=Origin(xyz=(-0.012, -0.038, key_center_z)),
        material=button_gray,
        name="call_key",
    )
    lower.visual(
        Box((0.010, 0.005, key_height)),
        origin=Origin(xyz=(0.012, -0.038, key_center_z)),
        material=button_gray,
        name="end_key",
    )
    for row_index, y in enumerate((-0.053, -0.0645, -0.076, -0.0875)):
        for column_index, x in enumerate((-0.0115, 0.0, 0.0115)):
            lower.visual(
                Box((0.0084, 0.0072, 0.00095)),
                origin=Origin(xyz=(x, y, lower_interface_z + 0.000475)),
                material=button_gray,
                name=f"number_key_{row_index}_{column_index}",
            )
    lower.visual(
        Box((0.010, 0.0014, 0.0005)),
        origin=Origin(xyz=(0.0, -0.091, lower_interface_z + 0.00025)),
        material=charcoal,
        name="microphone_slot",
    )
    lower.visual(
        Cylinder(radius=hinge_radius, length=side_knuckle_length),
        origin=Origin(
            xyz=(-knuckle_center_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=gloss_black,
        name="left_hinge_knuckle",
    )
    lower.visual(
        Cylinder(radius=hinge_radius, length=side_knuckle_length),
        origin=Origin(
            xyz=(knuckle_center_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=gloss_black,
        name="right_hinge_knuckle",
    )
    lower.visual(
        Cylinder(radius=0.0019, length=chrome_pin_length),
        origin=Origin(
            xyz=(-pin_center_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=chrome,
        name="left_chrome_pin",
    )
    lower.visual(
        Cylinder(radius=0.0019, length=chrome_pin_length),
        origin=Origin(
            xyz=(pin_center_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=chrome,
        name="right_chrome_pin",
    )
    lower.inertial = Inertial.from_geometry(
        Box((body_width, body_length + rear_offset + hinge_radius, body_thickness + hinge_radius)),
        mass=0.09,
        origin=Origin(
            xyz=(
                0.0,
                -0.049,
                -0.003,
            )
        ),
    )

    upper = model.part("upper_half")
    upper.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, shell_center_y, shell_center_z)),
        material=gloss_black,
        name="upper_shell",
    )
    upper.visual(
        Box((0.039, 0.061, screen_bezel_height)),
        origin=Origin(xyz=(0.0, -0.050, screen_bezel_z)),
        material=satin_black,
        name="screen_bezel",
    )
    upper.visual(
        Box((0.034, 0.0068, 0.0050)),
        origin=Origin(xyz=(0.0, -0.0022, 0.0026)),
        material=gloss_black,
        name="upper_hinge_bridge",
    )
    upper.visual(
        Box((0.033, 0.053, screen_glass_height)),
        origin=Origin(xyz=(0.0, -0.050, screen_glass_z)),
        material=dark_glass,
        name="screen_glass",
    )
    upper.visual(
        Box((0.015, 0.0022, 0.0005)),
        origin=Origin(xyz=(0.0, -0.017, upper_interface_z + 0.00025)),
        material=charcoal,
        name="earpiece_slot",
    )
    upper.visual(
        Cylinder(radius=hinge_radius, length=center_barrel_length),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=gloss_black,
        name="center_hinge_barrel",
    )
    upper.inertial = Inertial.from_geometry(
        Box((body_width, body_length + rear_offset + hinge_radius, body_thickness + hinge_radius)),
        mass=0.08,
        origin=Origin(
            xyz=(
                0.0,
                -0.049,
                0.003,
            )
        ),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=0.0,
            upper=2.97,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    flip_hinge = object_model.get_articulation("flip_hinge")

    lower_shell = lower.get_visual("lower_shell")
    keypad_plate = lower.get_visual("keypad_plate")
    left_knuckle = lower.get_visual("left_hinge_knuckle")
    right_knuckle = lower.get_visual("right_hinge_knuckle")
    left_pin = lower.get_visual("left_chrome_pin")
    right_pin = lower.get_visual("right_chrome_pin")

    upper_shell = upper.get_visual("upper_shell")
    screen_glass = upper.get_visual("screen_glass")
    center_barrel = upper.get_visual("center_hinge_barrel")

    limits = flip_hinge.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "flip_hinge_has_realistic_limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and 2.8 <= limits.upper <= 3.05,
        "Flip hinge should close flat at 0 rad and open to roughly 160-175 degrees.",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({flip_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flip_hinge_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="flip_hinge_closed_no_floating")

            ctx.expect_within(
                upper,
                lower,
                axes="xy",
                inner_elem=upper_shell,
                outer_elem=lower_shell,
                name="closed_upper_matches_lower_outline",
            )
            ctx.expect_within(
                lower,
                upper,
                axes="xy",
                inner_elem=lower_shell,
                outer_elem=upper_shell,
                name="closed_lower_matches_upper_outline",
            )
            ctx.expect_overlap(
                upper,
                lower,
                axes="xy",
                min_overlap=0.050,
                elem_a=upper_shell,
                elem_b=lower_shell,
                name="closed_halves_share_plan_footprint",
            )
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.0020,
                max_gap=0.0024,
                positive_elem=upper_shell,
                negative_elem=lower_shell,
                name="closed_clamshell_interior_gap",
            )
            ctx.expect_overlap(
                lower,
                upper,
                axes="yz",
                min_overlap=0.0080,
                elem_a=left_knuckle,
                elem_b=center_barrel,
                name="left_knuckle_is_coaxial_with_center_barrel",
            )
            ctx.expect_overlap(
                lower,
                upper,
                axes="yz",
                min_overlap=0.0080,
                elem_a=right_knuckle,
                elem_b=center_barrel,
                name="right_knuckle_is_coaxial_with_center_barrel",
            )
            ctx.expect_contact(
                lower,
                upper,
                elem_a=left_knuckle,
                elem_b=center_barrel,
                name="left_knuckle_contacts_center_barrel",
            )
            ctx.expect_contact(
                lower,
                upper,
                elem_a=right_knuckle,
                elem_b=center_barrel,
                name="right_knuckle_contacts_center_barrel",
            )
            ctx.expect_overlap(
                lower,
                upper,
                axes="yz",
                min_overlap=0.0036,
                elem_a=left_pin,
                elem_b=center_barrel,
                name="left_chrome_pin_aligns_with_hinge_axis",
            )
            ctx.expect_overlap(
                lower,
                upper,
                axes="yz",
                min_overlap=0.0036,
                elem_a=right_pin,
                elem_b=center_barrel,
                name="right_chrome_pin_aligns_with_hinge_axis",
            )
            ctx.expect_overlap(
                upper,
                lower,
                axes="x",
                min_overlap=0.032,
                elem_a=screen_glass,
                elem_b=keypad_plate,
                name="screen_and_keypad_share_phone_width",
            )

        with ctx.pose({flip_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flip_hinge_open_no_overlap")
            ctx.fail_if_isolated_parts(name="flip_hinge_open_no_floating")
            ctx.expect_gap(
                upper,
                lower,
                axis="y",
                min_gap=0.0075,
                max_gap=0.0115,
                positive_elem=upper_shell,
                negative_elem=lower_shell,
                name="opened_halves_separate_around_hinge",
            )
            ctx.expect_overlap(
                upper,
                lower,
                axes="x",
                min_overlap=0.050,
                elem_a=upper_shell,
                elem_b=lower_shell,
                name="opened_halves_keep_matching_width",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
