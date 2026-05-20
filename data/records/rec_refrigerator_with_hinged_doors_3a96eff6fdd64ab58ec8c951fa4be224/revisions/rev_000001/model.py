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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_loop(
    width: float,
    height: float,
    radius: float,
    *,
    y: float,
    x_center: float = 0.0,
    z_center: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y, z + z_center)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _door_shell_mesh(
    *,
    width: float,
    height: float,
    hinge_offset: float,
    front_radius: float,
    front_y: float = -0.088,
    mid_y: float = -0.044,
    back_y: float = -0.006,
) -> object:
    x_center = hinge_offset - (width * 0.5)
    back_radius = min(front_radius * 0.72, (width - 0.030) * 0.24, (height - 0.030) * 0.24)
    mid_radius = min(front_radius * 0.88, (width - 0.012) * 0.25, (height - 0.012) * 0.25)
    return section_loft(
        [
            _xz_loop(
                width - 0.028,
                height - 0.028,
                back_radius,
                y=back_y,
                x_center=x_center,
                corner_segments=12,
            ),
            _xz_loop(
                width - 0.010,
                height - 0.010,
                mid_radius,
                y=mid_y,
                x_center=x_center,
                corner_segments=12,
            ),
            _xz_loop(
                width,
                height,
                front_radius,
                y=front_y,
                x_center=x_center,
                corner_segments=12,
            ),
        ]
    )


def _hinge_ring_mesh(*, inner_radius: float, outer_radius: float, length: float, segments: int = 32):
    half_length = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _add_handle(
    part,
    *,
    prefix: str,
    x: float,
    z_center: float,
    bar_length: float,
    mount_span: float,
    chrome,
) -> None:
    bar_y = -0.124
    standoff_y = -0.107
    pad_y = -0.089
    part.visual(
        Cylinder(radius=0.016, length=bar_length),
        origin=Origin(xyz=(x, bar_y, z_center)),
        material=chrome,
        name=f"{prefix}_handle_bar",
    )
    for index, dz in enumerate((-mount_span * 0.5, mount_span * 0.5)):
        part.visual(
            Cylinder(radius=0.009, length=0.036),
            origin=Origin(
                xyz=(x, standoff_y, z_center + dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"{prefix}_handle_standoff_{index}",
        )
        part.visual(
            Box((0.038, 0.008, 0.032)),
            origin=Origin(xyz=(x, pad_y, z_center + dz)),
            material=chrome,
            name=f"{prefix}_handle_pad_{index}",
        )


def _add_retro_door(
    model: ArticulatedObject,
    *,
    part_name: str,
    shell_name: str,
    width: float,
    height: float,
    front_radius: float,
    liner_height: float,
    hinge_ring_name_top: str,
    hinge_ring_name_bottom: str,
    handle_length: float,
    handle_mount_span: float,
    body_color,
    interior_color,
    gasket_color,
    chrome,
    shell_mass: float,
) -> object:
    part = model.part(part_name)
    hinge_offset = -0.020
    x_center = hinge_offset - (width * 0.5)

    shell_mesh = _save_mesh(
        shell_name,
        _door_shell_mesh(
            width=width,
            height=height,
            hinge_offset=hinge_offset,
            front_radius=front_radius,
        ),
    )
    part.visual(shell_mesh, material=body_color, name="door_shell")
    part.visual(
        Box((width - 0.112, 0.014, liner_height)),
        origin=Origin(xyz=(x_center - 0.006, -0.012, 0.0)),
        material=interior_color,
        name="door_liner",
    )
    part.visual(
        Box((0.020, 0.016, height - 0.024)),
        origin=Origin(xyz=(-0.024, -0.008, 0.0)),
        material=body_color,
        name="hinge_rail",
    )

    ring_inner_radius = 0.0105
    ring_outer_radius = 0.0150
    ring_length = 0.034
    ring_z = (height * 0.5) - (0.060 if height < 0.50 else 0.090)
    ring_mesh = _save_mesh(
        f"{part_name}_hinge_ring",
        _hinge_ring_mesh(
            inner_radius=ring_inner_radius,
            outer_radius=ring_outer_radius,
            length=ring_length,
            segments=32,
        ),
    )
    part.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, ring_z)),
        material=chrome,
        name=hinge_ring_name_top,
    )
    part.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, -ring_z)),
        material=chrome,
        name=hinge_ring_name_bottom,
    )

    _add_handle(
        part,
        prefix=part_name,
        x=x_center - (width * 0.35),
        z_center=0.0,
        bar_length=handle_length,
        mount_span=handle_mount_span,
        chrome=chrome,
    )

    part.inertial = Inertial.from_geometry(
        Box((width, 0.105, height)),
        mass=shell_mass,
        origin=Origin(xyz=(x_center * 0.82, -0.048, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_two_door_refrigerator")

    body_mint = model.material("body_mint", rgba=(0.58, 0.77, 0.74, 1.0))
    warm_white = model.material("warm_white", rgba=(0.95, 0.94, 0.89, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.97, 0.95, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.56, 0.59, 0.61, 1.0))
    dial_red = model.material("dial_red", rgba=(0.76, 0.16, 0.12, 1.0))
    shadow_gray = model.material("shadow_gray", rgba=(0.24, 0.26, 0.29, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((0.72, 0.74, 1.78)),
        mass=88.0,
        origin=Origin(xyz=(0.0, 0.37, 0.89)),
    )

    cabinet.visual(
        Box((0.68, 0.03, 1.70)),
        origin=Origin(xyz=(0.0, 0.725, 0.85)),
        material=body_mint,
        name="back_shell",
    )
    cabinet.visual(
        Box((0.03, 0.61, 1.58)),
        origin=Origin(xyz=(-0.345, 0.435, 0.89)),
        material=body_mint,
        name="left_side_shell",
    )
    cabinet.visual(
        Box((0.03, 0.61, 1.58)),
        origin=Origin(xyz=(0.345, 0.435, 0.89)),
        material=body_mint,
        name="right_side_shell",
    )
    cabinet.visual(
        Box((0.52, 0.64, 0.03)),
        origin=Origin(xyz=(0.0, 0.420, 1.765)),
        material=body_mint,
        name="top_shell_flat",
    )
    cabinet.visual(
        Cylinder(radius=0.095, length=0.52),
        origin=Origin(xyz=(0.0, 0.095, 1.685), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_mint,
        name="top_front_roll",
    )
    cabinet.visual(
        Cylinder(radius=0.065, length=1.58),
        origin=Origin(xyz=(-0.28, 0.095, 0.89)),
        material=body_mint,
        name="left_front_column",
    )
    cabinet.visual(
        Cylinder(radius=0.065, length=1.58),
        origin=Origin(xyz=(0.28, 0.095, 0.89)),
        material=body_mint,
        name="right_front_column",
    )
    cabinet.visual(
        Box((0.56, 0.66, 0.03)),
        origin=Origin(xyz=(0.0, 0.41, 0.015)),
        material=body_mint,
        name="base_shell",
    )
    cabinet.visual(
        Cylinder(radius=0.08, length=0.56),
        origin=Origin(xyz=(0.0, 0.08, 0.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_mint,
        name="base_front_roll",
    )
    cabinet.visual(
        Box((0.60, 0.60, 0.02)),
        origin=Origin(xyz=(0.0, 0.42, 0.10)),
        material=liner_white,
        name="fresh_floor",
    )
    cabinet.visual(
        Box((0.60, 0.02, 1.54)),
        origin=Origin(xyz=(0.0, 0.71, 0.87)),
        material=liner_white,
        name="back_liner",
    )
    cabinet.visual(
        Box((0.02, 0.60, 1.54)),
        origin=Origin(xyz=(-0.30, 0.42, 0.87)),
        material=liner_white,
        name="left_liner",
    )
    cabinet.visual(
        Box((0.02, 0.60, 1.54)),
        origin=Origin(xyz=(0.30, 0.42, 0.87)),
        material=liner_white,
        name="right_liner",
    )
    cabinet.visual(
        Box((0.60, 0.60, 0.02)),
        origin=Origin(xyz=(0.0, 0.42, 1.65)),
        material=liner_white,
        name="ceiling_liner",
    )
    cabinet.visual(
        Box((0.60, 0.58, 0.028)),
        origin=Origin(xyz=(0.0, 0.41, 1.28)),
        material=warm_white,
        name="freezer_divider",
    )
    cabinet.visual(
        Box((0.54, 0.012, 0.05)),
        origin=Origin(xyz=(0.0, 0.006, 0.13)),
        material=chrome,
        name="toe_kick_trim",
    )

    hinge_axis_x = 0.355
    hinge_axis_y = -0.006
    ring_inner_radius = 0.0105
    pin_radius = ring_inner_radius * math.cos(math.pi / 32.0)
    pin_length = 0.040
    for pin_name, pin_z in (
        ("upper_hinge_pin_top", 1.62),
        ("upper_hinge_pin_bottom", 1.38),
        ("lower_hinge_pin_top", 1.16),
        ("lower_hinge_pin_bottom", 0.16),
    ):
        cabinet.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, pin_z)),
            material=chrome,
            name=pin_name,
        )
        cabinet.visual(
            Box((0.024, 0.048, 0.048)),
            origin=Origin(xyz=(0.344, 0.040, pin_z)),
            material=shadow_gray,
            name=f"{pin_name}_bracket",
        )
        cabinet.visual(
            Box((0.022, 0.020, 0.006)),
            origin=Origin(xyz=(0.350, 0.014, pin_z + 0.023)),
            material=shadow_gray,
            name=f"{pin_name}_cap",
        )

    top_trim = model.part("top_trim")
    top_trim.visual(
        Box((0.43, 0.06, 0.055)),
        material=warm_white,
        name="trim_panel",
    )
    top_trim.visual(
        Box((0.17, 0.008, 0.020)),
        origin=Origin(xyz=(0.09, -0.026, 0.0)),
        material=chrome,
        name="dial_escutcheon",
    )
    top_trim.inertial = Inertial.from_geometry(
        Box((0.43, 0.06, 0.055)),
        mass=0.8,
    )

    upper_door = _add_retro_door(
        model,
        part_name="upper_freezer_door",
        shell_name="upper_freezer_shell",
        width=0.74,
        height=0.36,
        front_radius=0.085,
        liner_height=0.27,
        hinge_ring_name_top="upper_hinge_ring_top",
        hinge_ring_name_bottom="upper_hinge_ring_bottom",
        handle_length=0.22,
        handle_mount_span=0.15,
        body_color=body_mint,
        interior_color=liner_white,
        gasket_color=gasket_gray,
        chrome=chrome,
        shell_mass=8.5,
    )

    lower_door = _add_retro_door(
        model,
        part_name="lower_fresh_food_door",
        shell_name="lower_fresh_food_shell",
        width=0.74,
        height=1.18,
        front_radius=0.092,
        liner_height=1.02,
        hinge_ring_name_top="lower_hinge_ring_top",
        hinge_ring_name_bottom="lower_hinge_ring_bottom",
        handle_length=0.60,
        handle_mount_span=0.42,
        body_color=body_mint,
        interior_color=liner_white,
        gasket_color=gasket_gray,
        chrome=chrome,
        shell_mass=17.5,
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="dial_knob",
    )
    temperature_dial.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_hub",
    )
    temperature_dial.visual(
        Box((0.005, 0.006, 0.018)),
        origin=Origin(xyz=(0.017, -0.020, 0.0)),
        material=dial_red,
        name="dial_pointer",
    )
    temperature_dial.inertial = Inertial.from_geometry(
        Box((0.060, 0.024, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
    )

    model.articulation(
        "cabinet_to_top_trim",
        ArticulationType.FIXED,
        parent=cabinet,
        child=top_trim,
        origin=Origin(xyz=(0.0, 0.09, 1.285)),
    )
    model.articulation(
        "cabinet_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 1.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.5,
            lower=0.0,
            upper=2.20,
        ),
    )
    model.articulation(
        "cabinet_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=38.0,
            velocity=1.4,
            lower=0.0,
            upper=2.20,
        ),
    )
    model.articulation(
        "top_trim_to_temperature_dial",
        ArticulationType.REVOLUTE,
        parent=top_trim,
        child=temperature_dial,
        origin=Origin(xyz=(0.09, -0.03, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=4.0,
            lower=-2.40,
            upper=2.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    top_trim = object_model.get_part("top_trim")
    upper_door = object_model.get_part("upper_freezer_door")
    lower_door = object_model.get_part("lower_fresh_food_door")
    temperature_dial = object_model.get_part("temperature_dial")

    upper_hinge = object_model.get_articulation("cabinet_to_upper_door")
    lower_hinge = object_model.get_articulation("cabinet_to_lower_door")
    dial_joint = object_model.get_articulation("top_trim_to_temperature_dial")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        cabinet,
        upper_door,
        elem_a="upper_hinge_pin_top",
        elem_b="upper_hinge_ring_top",
        reason="Upper freezer door hinge pin is intentionally clipped through its barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        upper_door,
        elem_a="upper_hinge_pin_bottom",
        elem_b="upper_hinge_ring_bottom",
        reason="Upper freezer door lower barrel remains clipped on its hinge pin.",
    )
    ctx.allow_overlap(
        cabinet,
        lower_door,
        elem_a="lower_hinge_pin_top",
        elem_b="lower_hinge_ring_top",
        reason="Lower fresh-food door upper barrel is intentionally retained by the hinge pin.",
    )
    ctx.allow_overlap(
        cabinet,
        lower_door,
        elem_a="lower_hinge_pin_bottom",
        elem_b="lower_hinge_ring_bottom",
        reason="Lower fresh-food door lower barrel is intentionally retained by the hinge pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "upper_door_axis_is_vertical",
        tuple(round(value, 3) for value in upper_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {upper_hinge.axis}",
    )
    ctx.check(
        "lower_door_axis_is_vertical",
        tuple(round(value, 3) for value in lower_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {lower_hinge.axis}",
    )
    ctx.check(
        "dial_axis_is_local_y",
        tuple(round(value, 3) for value in dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected local Y dial axis, got {dial_joint.axis}",
    )
    ctx.check(
        "door_limits_open_outward",
        (
            upper_hinge.motion_limits is not None
            and lower_hinge.motion_limits is not None
            and upper_hinge.motion_limits.lower == 0.0
            and lower_hinge.motion_limits.lower == 0.0
            and upper_hinge.motion_limits.upper is not None
            and lower_hinge.motion_limits.upper is not None
            and upper_hinge.motion_limits.upper >= 2.0
            and lower_hinge.motion_limits.upper >= 2.0
        ),
        details="Door joints should swing from closed at 0 rad to a broad open angle.",
    )

    ctx.expect_contact(top_trim, cabinet, name="top_trim_fixed_to_cabinet")
    ctx.expect_contact(temperature_dial, top_trim, name="temperature_dial_mounted_in_trim")
    ctx.expect_within(
        temperature_dial,
        top_trim,
        axes="xz",
        margin=0.03,
        name="temperature_dial_stays_within_trim_face",
    )

    ctx.expect_contact(
        upper_door,
        cabinet,
        elem_a="upper_hinge_ring_top",
        elem_b="upper_hinge_pin_top",
        name="upper_freezer_door_top_hinge_supported",
    )
    ctx.expect_contact(
        upper_door,
        cabinet,
        elem_a="upper_hinge_ring_bottom",
        elem_b="upper_hinge_pin_bottom",
        name="upper_freezer_door_bottom_hinge_supported",
    )
    ctx.expect_contact(
        lower_door,
        cabinet,
        elem_a="lower_hinge_ring_top",
        elem_b="lower_hinge_pin_top",
        name="lower_fresh_food_door_top_hinge_supported",
    )
    ctx.expect_contact(
        lower_door,
        cabinet,
        elem_a="lower_hinge_ring_bottom",
        elem_b="lower_hinge_pin_bottom",
        name="lower_fresh_food_door_bottom_hinge_supported",
    )
    ctx.expect_origin_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.75,
        max_gap=0.95,
        name="freezer_door_stacked_above_fresh_food_door",
    )

    with ctx.pose({upper_hinge: 1.20, lower_hinge: 1.15}):
        ctx.fail_if_parts_overlap_in_current_pose(name="doors_open_pose_clearance")
        ctx.expect_contact(
            upper_door,
            cabinet,
            elem_a="upper_hinge_ring_top",
            elem_b="upper_hinge_pin_top",
            name="upper_door_stays_clipped_when_open",
        )
        ctx.expect_contact(
            lower_door,
            cabinet,
            elem_a="lower_hinge_ring_bottom",
            elem_b="lower_hinge_pin_bottom",
            name="lower_door_stays_clipped_when_open",
        )

    with ctx.pose({dial_joint: 1.90}):
        ctx.expect_contact(
            temperature_dial,
            top_trim,
            name="dial_remains_seated_while_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
