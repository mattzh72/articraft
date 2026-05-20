from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_geometry(
    width: float,
    length: float,
    radius: float,
    thickness: float,
):
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, length, radius, corner_segments=8),
        thickness,
        cap=True,
        closed=True,
    )


def _rounded_ring_geometry(
    outer_width: float,
    outer_length: float,
    outer_radius: float,
    wall: float,
    height: float,
):
    inner_width = outer_width - (2.0 * wall)
    inner_length = outer_length - (2.0 * wall)
    inner_radius = max(outer_radius - (wall * 0.8), wall * 0.6)
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_length, outer_radius, corner_segments=8),
        [rounded_rect_profile(inner_width, inner_length, inner_radius, corner_segments=8)],
        height=height,
        cap=True,
        center=False,
        closed=True,
    )


def _shell_mesh(
    name: str,
    *,
    outer_width: float,
    outer_length: float,
    outer_height: float,
    wall: float,
    floor_thickness: float,
    radius: float,
):
    shell = _rounded_plate_geometry(outer_width, outer_length, radius, floor_thickness)
    wall_ring = _rounded_ring_geometry(
        outer_width,
        outer_length,
        radius,
        wall,
        outer_height - floor_thickness,
    )
    wall_ring.translate(0.0, 0.0, floor_thickness)
    shell.merge(wall_ring)
    return _save_mesh(name, shell)


def _keypad_mesh(name: str):
    keypad = _rounded_plate_geometry(0.046, 0.074, 0.0055, 0.0012)

    def add_key(cx: float, cy: float, width: float, length: float, height: float = 0.0018) -> None:
        key = _rounded_plate_geometry(width, length, min(width, length) * 0.22, height)
        key.translate(cx, cy, 0.0012)
        keypad.merge(key)

    for row_y in (0.002, -0.012, -0.026, -0.040):
        for column_x in (-0.013, 0.0, 0.013):
            add_key(column_x, row_y, 0.010, 0.0072)

    add_key(-0.013, 0.018, 0.012, 0.0075)
    add_key(0.013, 0.018, 0.012, 0.0075)
    add_key(0.0, 0.020, 0.018, 0.010, 0.0016)
    add_key(0.0, 0.008, 0.009, 0.009, 0.0016)
    add_key(-0.013, 0.030, 0.0115, 0.0058, 0.0016)
    add_key(0.013, 0.030, 0.0115, 0.0058, 0.0016)
    add_key(0.0, -0.053, 0.020, 0.0035, 0.0010)

    return _save_mesh(name, keypad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_clamshell_flip_phone")

    body_silver = model.material("body_silver", rgba=(0.58, 0.60, 0.64, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    keypad_rubber = model.material("keypad_rubber", rgba=(0.12, 0.13, 0.15, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.23, 0.52, 0.60, 0.72))

    lower_width = 0.056
    lower_length = 0.104
    lower_height = 0.024
    lower_wall = 0.0024
    lower_floor = 0.0030
    lower_radius = 0.009

    upper_width = 0.056
    upper_length = 0.082
    upper_height = 0.013
    upper_wall = 0.0023
    upper_floor = 0.0025
    upper_radius = 0.009

    hinge_radius = 0.0032
    hinge_span = 0.054
    hinge_axis_z = 0.019
    upper_bottom_local_z = 0.0005 - hinge_axis_z
    knuckle_length = hinge_span / 5.0
    knuckle_centers = [
        (-hinge_span * 0.5) + (knuckle_length * 0.5) + (knuckle_length * index)
        for index in range(5)
    ]

    lower_shell = _shell_mesh(
        "retro_phone_lower_shell",
        outer_width=lower_width,
        outer_length=lower_length,
        outer_height=lower_height,
        wall=lower_wall,
        floor_thickness=lower_floor,
        radius=lower_radius,
    )
    upper_shell = _shell_mesh(
        "retro_phone_upper_shell",
        outer_width=upper_width,
        outer_length=upper_length,
        outer_height=upper_height,
        wall=upper_wall,
        floor_thickness=upper_floor,
        radius=upper_radius,
    )
    keypad_mesh = _keypad_mesh("retro_phone_keypad_mat")
    display_plate_mesh = _save_mesh(
        "retro_phone_display_plate",
        _rounded_plate_geometry(0.044, 0.064, 0.0045, 0.0012),
    )

    lower_body = model.part("lower_body")
    lower_body.visual(
        lower_shell,
        origin=Origin(xyz=(0.0, -lower_length * 0.5, 0.0)),
        material=body_silver,
        name="lower_shell",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((lower_width, lower_length, lower_height + 0.004)),
        mass=0.115,
        origin=Origin(xyz=(0.0, -lower_length * 0.5, (lower_height + 0.004) * 0.5)),
    )

    lower_knuckle_index = 0
    for segment_index in (0, 2, 4):
        lower_body.visual(
            Cylinder(radius=hinge_radius, length=knuckle_length),
            origin=Origin(
                xyz=(knuckle_centers[segment_index], 0.0, hinge_axis_z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=hinge_metal,
            name=f"lower_knuckle_{lower_knuckle_index}",
        )
        lower_knuckle_index += 1

    keypad_plate = model.part("keypad_plate")
    keypad_plate.visual(
        keypad_mesh,
        material=keypad_rubber,
        name="keypad_surface",
    )
    keypad_plate.inertial = Inertial.from_geometry(
        Box((0.046, 0.074, 0.0032)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
    )

    upper_cover = model.part("upper_cover")
    upper_cover.visual(
        upper_shell,
        origin=Origin(xyz=(0.0, upper_length * 0.5, upper_bottom_local_z)),
        material=body_silver,
        name="upper_shell",
    )
    upper_cover.inertial = Inertial.from_geometry(
        Box((upper_width, upper_length, upper_height + 0.004)),
        mass=0.075,
        origin=Origin(
            xyz=(
                0.0,
                upper_length * 0.5,
                upper_bottom_local_z + ((upper_height + 0.004) * 0.5),
            )
        ),
    )

    upper_knuckle_index = 0
    for segment_index in (1, 3):
        upper_cover.visual(
            Cylinder(radius=hinge_radius, length=knuckle_length),
            origin=Origin(
                xyz=(knuckle_centers[segment_index], 0.0, 0.0),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=hinge_metal,
            name=f"upper_knuckle_{upper_knuckle_index}",
        )
        upper_knuckle_index += 1
    upper_cover.visual(
        Box((0.044, 0.0056, 0.0060)),
        origin=Origin(xyz=(0.0, 0.0028, -0.0028)),
        material=hinge_metal,
        name="upper_hinge_bridge",
    )

    display_module = model.part("display_module")
    display_module.visual(
        display_plate_mesh,
        material=trim_graphite,
        name="display_plate",
    )
    display_module.visual(
        Box((0.032, 0.038, 0.0010)),
        origin=Origin(xyz=(0.0, -0.004, 0.0017)),
        material=screen_glass,
        name="screen_glass",
    )
    display_module.visual(
        Box((0.018, 0.003, 0.0008)),
        origin=Origin(xyz=(0.0, 0.022, 0.0016)),
        material=trim_graphite,
        name="earpiece_slot",
    )
    display_module.inertial = Inertial.from_geometry(
        Box((0.044, 0.064, 0.003)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )

    model.articulation(
        "lower_to_keypad",
        ArticulationType.FIXED,
        parent=lower_body,
        child=keypad_plate,
        origin=Origin(xyz=(0.0, -0.049, lower_floor)),
    )
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_cover,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=6.0, lower=0.0, upper=3.05),
    )
    model.articulation(
        "upper_to_display",
        ArticulationType.FIXED,
        parent=upper_cover,
        child=display_module,
        origin=Origin(xyz=(0.0, 0.043, upper_bottom_local_z + upper_floor)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    keypad_plate = object_model.get_part("keypad_plate")
    upper_cover = object_model.get_part("upper_cover")
    display_module = object_model.get_part("display_module")
    flip_hinge = object_model.get_articulation("flip_hinge")

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

    limits = flip_hinge.motion_limits
    ctx.check("phone_parts_present", True)
    ctx.check(
        "flip_hinge_axis_is_widthwise",
        tuple(round(value, 6) for value in flip_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"flip hinge axis was {flip_hinge.axis}",
    )
    ctx.check(
        "flip_hinge_limits_are_phone_like",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 3.0,
        details=f"flip hinge limits were {limits}",
    )

    ctx.expect_contact(
        keypad_plate,
        lower_body,
        elem_a="keypad_surface",
        elem_b="lower_shell",
        contact_tol=1e-5,
        name="keypad_seated_in_lower_body",
    )
    ctx.expect_within(
        keypad_plate,
        lower_body,
        axes="xy",
        margin=0.002,
        inner_elem="keypad_surface",
        outer_elem="lower_shell",
        name="keypad_within_lower_body_footprint",
    )
    ctx.expect_contact(
        display_module,
        upper_cover,
        elem_a="display_plate",
        elem_b="upper_shell",
        contact_tol=1e-5,
        name="display_plate_seated_in_upper_cover",
    )
    ctx.expect_within(
        display_module,
        upper_cover,
        axes="xy",
        margin=0.002,
        inner_elem="display_plate",
        outer_elem="upper_shell",
        name="display_plate_within_upper_cover_footprint",
    )

    with ctx.pose({flip_hinge: 0.0}):
        ctx.expect_contact(
            lower_body,
            upper_cover,
            elem_a="lower_knuckle_1",
            elem_b="upper_knuckle_0",
            contact_tol=1e-5,
            name="piano_hinge_knuckles_touch",
        )
        ctx.expect_gap(
            upper_cover,
            lower_body,
            axis="y",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            min_gap=0.0,
            max_gap=0.003,
            name="open_halves_meet_cleanly_at_hinge_line",
        )

    with ctx.pose({flip_hinge: 3.0}):
        ctx.expect_overlap(
            upper_cover,
            lower_body,
            axes="xy",
            min_overlap=0.045,
            elem_a="upper_shell",
            elem_b="lower_shell",
            name="closed_cover_overlaps_lower_body_footprint",
        )
        ctx.expect_gap(
            upper_cover,
            lower_body,
            axis="z",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            max_penetration=0.0,
            max_gap=0.008,
            name="closed_shells_clear_without_interpenetration",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
