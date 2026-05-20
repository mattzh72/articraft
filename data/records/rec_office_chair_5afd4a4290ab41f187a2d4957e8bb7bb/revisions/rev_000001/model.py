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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loop_xz(
    width: float,
    height: float,
    *,
    y: float,
    z_center: float = 0.0,
    radius: float | None = None,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width, height) * 0.20
    corner = min(corner, width * 0.45, height * 0.45)
    return [
        (x, y, z_center + z)
        for x, z in rounded_rect_profile(
            width,
            height,
            corner,
            corner_segments=corner_segments,
        )
    ]


def _loop_xy(
    width: float,
    depth: float,
    *,
    z: float,
    y_center: float = 0.0,
    radius: float | None = None,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width, depth) * 0.20
    corner = min(corner, width * 0.45, depth * 0.45)
    return [
        (x, y_center + y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            corner,
            corner_segments=corner_segments,
        )
    ]


def _seat_cushion_mesh():
    return _save_mesh(
        "executive_chair_seat_cushion",
        section_loft(
            [
                _loop_xz(0.52, 0.060, y=0.235, z_center=0.040, radius=0.022),
                _loop_xz(0.55, 0.090, y=0.060, z_center=0.055, radius=0.028),
                _loop_xz(0.52, 0.080, y=-0.060, z_center=0.055, radius=0.026),
                _loop_xz(0.49, 0.060, y=-0.195, z_center=0.045, radius=0.020),
            ]
        ),
    )


def _backrest_mesh():
    return _save_mesh(
        "executive_chair_backrest",
        section_loft(
            [
                _loop_xy(0.250, 0.075, z=0.065, y_center=-0.020, radius=0.022),
                _loop_xy(0.360, 0.105, z=0.245, y_center=-0.048, radius=0.030),
                _loop_xy(0.450, 0.110, z=0.495, y_center=-0.085, radius=0.034),
                _loop_xy(0.360, 0.095, z=0.690, y_center=-0.120, radius=0.026),
            ]
        ),
    )


def _arm_pad_mesh(name: str):
    return _save_mesh(
        name,
        section_loft(
            [
                _loop_xz(0.074, 0.036, y=0.015, z_center=0.098, radius=0.014),
                _loop_xz(0.080, 0.044, y=0.115, z_center=0.103, radius=0.017),
                _loop_xz(0.074, 0.040, y=0.230, z_center=0.101, radius=0.015),
            ]
        ),
    )


def _star_leg_mesh(name: str):
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            [
                (0.085, 0.000, 0.074),
                (0.200, 0.000, 0.063),
                (0.310, 0.000, 0.052),
                (0.388, 0.000, 0.046),
            ],
            profile=rounded_rect_profile(0.074, 0.028, 0.012, corner_segments=8),
            samples_per_segment=12,
            cap_profile=True,
        ),
    )


def _gas_outer_shell():
    outer_profile = [
        (0.046, 0.000),
        (0.044, 0.030),
        (0.038, 0.120),
        (0.030, 0.190),
    ]
    inner_profile = [
        (0.022, 0.006),
        (0.022, 0.040),
        (0.022, 0.150),
        (0.022, 0.184),
    ]
    return _save_mesh(
        "executive_chair_gas_outer_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _axis_tuple(axis) -> tuple[float, float, float]:
    return tuple(float(value) for value in axis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_office_chair")

    base_black = model.material("base_black", rgba=(0.11, 0.12, 0.13, 1.0))
    shell_black = model.material("shell_black", rgba=(0.17, 0.18, 0.20, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.82, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.50, 0.53, 1.0))

    seat_cushion_mesh = _seat_cushion_mesh()
    backrest_mesh = _backrest_mesh()
    left_arm_pad_mesh = _arm_pad_mesh("executive_chair_left_arm_pad")
    right_arm_pad_mesh = _arm_pad_mesh("executive_chair_right_arm_pad")
    arm_support_mesh = _save_mesh(
        "executive_chair_arm_support_tube",
        tube_from_spline_points(
            [
                (0.0, 0.000, 0.000),
                (0.0, 0.050, 0.045),
                (0.0, 0.145, 0.072),
                (0.0, 0.205, 0.078),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    star_leg_mesh = _star_leg_mesh("executive_chair_star_leg")
    gas_outer_mesh = _gas_outer_shell()

    star_base = model.part("star_base")
    star_base.visual(
        Cylinder(radius=0.090, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=base_black,
        name="hub_drum",
    )
    star_base.visual(
        Cylinder(radius=0.054, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=base_black,
        name="hub_spine",
    )
    for leg_index in range(5):
        angle = (2.0 * math.pi * leg_index) / 5.0
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        star_base.visual(
            star_leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=base_black,
            name=f"leg_{leg_index}",
        )
        star_base.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(
                xyz=(0.376 * direction_x, 0.376 * direction_y, 0.054),
            ),
            material=trim_dark,
            name=f"caster_socket_{leg_index}",
        )
    star_base.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.16)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    gas_outer = model.part("gas_outer")
    gas_outer.visual(
        gas_outer_mesh,
        material=shell_black,
        name="outer_shell",
    )
    gas_outer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.190),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    gas_inner = model.part("gas_inner")
    gas_inner.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=trim_dark,
        name="guide_sleeve",
    )
    gas_inner.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=chrome,
        name="chrome_piston",
    )
    gas_inner.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.257)),
        material=trim_dark,
        name="swivel_plate",
    )
    gas_inner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.291),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.1455)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_dark,
        name="swivel_socket",
    )
    seat_assembly.visual(
        Box((0.180, 0.165, 0.045)),
        origin=Origin(xyz=(0.0, -0.020, 0.032)),
        material=shell_black,
        name="tilt_mechanism",
    )
    seat_assembly.visual(
        Box((0.100, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, -0.120, 0.029)),
        material=shell_black,
        name="rear_mechanism_case",
    )
    seat_assembly.visual(
        Box((0.470, 0.420, 0.015)),
        origin=Origin(xyz=(0.0, 0.010, 0.024)),
        material=trim_dark,
        name="seat_tray",
    )
    seat_assembly.visual(
        seat_cushion_mesh,
        material=cushion_black,
        name="seat_cushion",
    )
    seat_assembly.visual(
        Box((0.230, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, -0.190, 0.070)),
        material=shell_black,
        name="back_bridge",
    )
    seat_assembly.visual(
        Box((0.032, 0.028, 0.050)),
        origin=Origin(xyz=(-0.108, -0.217, 0.108)),
        material=shell_black,
        name="back_hinge_left",
    )
    seat_assembly.visual(
        Box((0.032, 0.028, 0.050)),
        origin=Origin(xyz=(0.108, -0.217, 0.108)),
        material=shell_black,
        name="back_hinge_right",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        side_x = 0.240 * side_sign
        inner_x = 0.208 * side_sign
        outer_x = 0.272 * side_sign
        seat_assembly.visual(
            Box((0.030, 0.220, 0.050)),
            origin=Origin(xyz=(side_x, -0.035, 0.070)),
            material=shell_black,
            name=f"{side_name}_arm_side_rail",
        )
        seat_assembly.visual(
            Box((0.030, 0.060, 0.090)),
            origin=Origin(xyz=(side_x, -0.135, 0.105)),
            material=shell_black,
            name=f"{side_name}_arm_upright",
        )
        seat_assembly.visual(
            Box((0.034, 0.028, 0.050)),
            origin=Origin(xyz=(inner_x, -0.150, 0.145)),
            material=shell_black,
            name=f"{side_name}_arm_inner_cheek",
        )
        seat_assembly.visual(
            Box((0.034, 0.028, 0.050)),
            origin=Origin(xyz=(outer_x, -0.150, 0.145)),
            material=shell_black,
            name=f"{side_name}_arm_outer_cheek",
        )
    seat_assembly.inertial = Inertial.from_geometry(
        Box((0.560, 0.510, 0.160)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.000, 0.070)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.012, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="pivot_tube",
    )
    backrest.visual(
        Box((0.180, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.012, 0.055)),
        material=trim_dark,
        name="lower_spine",
    )
    backrest.visual(
        Box((0.082, 0.040, 0.480)),
        origin=Origin(xyz=(0.0, -0.102, 0.310)),
        material=shell_black,
        name="rear_spine",
    )
    backrest.visual(
        backrest_mesh,
        material=cushion_black,
        name="backrest_cushion",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.520, 0.130, 0.720)),
        mass=5.2,
        origin=Origin(xyz=(0.0, -0.080, 0.360)),
    )

    left_armrest = model.part("left_armrest")
    left_armrest.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    left_armrest.visual(
        arm_support_mesh,
        material=trim_dark,
        name="support_tube",
    )
    left_armrest.visual(
        left_arm_pad_mesh,
        material=cushion_black,
        name="arm_pad",
    )
    left_armrest.inertial = Inertial.from_geometry(
        Box((0.090, 0.320, 0.140)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.150, 0.080)),
    )

    right_armrest = model.part("right_armrest")
    right_armrest.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    right_armrest.visual(
        arm_support_mesh,
        material=trim_dark,
        name="support_tube",
    )
    right_armrest.visual(
        right_arm_pad_mesh,
        material=cushion_black,
        name="arm_pad",
    )
    right_armrest.inertial = Inertial.from_geometry(
        Box((0.090, 0.320, 0.140)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.150, 0.080)),
    )

    model.articulation(
        "base_to_gas_outer",
        ArticulationType.FIXED,
        parent=star_base,
        child=gas_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
    )
    model.articulation(
        "gas_height",
        ArticulationType.PRISMATIC,
        parent=gas_outer,
        child=gas_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.20,
            lower=0.0,
            upper=0.090,
        ),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas_inner,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=4.0),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat_assembly,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.225, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=0.420,
        ),
    )
    model.articulation(
        "left_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat_assembly,
        child=left_armrest,
        origin=Origin(xyz=(0.270, -0.150, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.150,
        ),
    )
    model.articulation(
        "right_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat_assembly,
        child=right_armrest,
        origin=Origin(xyz=(-0.270, -0.150, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.150,
        ),
    )

    for caster_index in range(5):
        angle = (2.0 * math.pi * caster_index) / 5.0
        caster_x = 0.376 * math.cos(angle)
        caster_y = 0.376 * math.sin(angle)

        fork = model.part(f"caster_fork_{caster_index}")
        fork.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=trim_dark,
            name="swivel_head",
        )
        fork.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=trim_dark,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.014, 0.008, 0.036)),
            origin=Origin(xyz=(-0.007, 0.0, -0.024)),
            material=trim_dark,
            name="support_post",
        )
        fork.visual(
            Cylinder(radius=0.004, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim_dark,
            name="axle_boss",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.018, 0.010, 0.052)),
            mass=0.18,
            origin=Origin(xyz=(-0.004, 0.0, -0.018)),
        )

        wheel = model.part(f"caster_wheel_{caster_index}")
        wheel.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hub_core",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="left_wheel",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="right_wheel",
        )
        wheel.inertial = Inertial.from_geometry(
            Box((0.032, 0.034, 0.032)),
            mass=0.22,
            origin=Origin(),
        )

        model.articulation(
            f"caster_swivel_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=star_base,
            child=fork,
            origin=Origin(xyz=(caster_x, caster_y, 0.042), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )
        model.articulation(
            f"caster_wheel_spin_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.020, 0.0, -0.040)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=24.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
    star_base = object_model.get_part("star_base")
    gas_outer = object_model.get_part("gas_outer")
    gas_inner = object_model.get_part("gas_inner")
    seat_assembly = object_model.get_part("seat_assembly")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")

    base_to_gas_outer = object_model.get_articulation("base_to_gas_outer")
    gas_height = object_model.get_articulation("gas_height")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_recline = object_model.get_articulation("back_recline")
    left_arm_flip = object_model.get_articulation("left_arm_flip")
    right_arm_flip = object_model.get_articulation("right_arm_flip")

    ctx.allow_overlap(
        gas_outer,
        star_base,
        reason="The gas-lift shroud is captured inside the star-base hub socket.",
    )
    ctx.allow_overlap(
        gas_inner,
        gas_outer,
        reason="The telescoping gas-lift sleeve nests concentrically inside the outer shroud.",
    )
    ctx.allow_overlap(
        gas_inner,
        star_base,
        reason="The lower gas-lift guide remains retained inside the base hub.",
    )
    ctx.allow_overlap(
        left_armrest,
        seat_assembly,
        elem_a="hinge_barrel",
        elem_b="left_arm_outer_cheek",
        reason="The left armrest hinge barrel sits inside the rear arm bracket cheek.",
    )
    ctx.allow_overlap(
        right_armrest,
        seat_assembly,
        elem_a="hinge_barrel",
        elem_b="right_arm_outer_cheek",
        reason="The right armrest hinge barrel sits inside the rear arm bracket cheek.",
    )
    ctx.allow_overlap(
        left_armrest,
        seat_assembly,
        elem_a="support_tube",
        elem_b="left_arm_outer_cheek",
        reason="The left arm support tube passes through the enclosed arm bracket cheek.",
    )
    ctx.allow_overlap(
        right_armrest,
        seat_assembly,
        elem_a="support_tube",
        elem_b="right_arm_outer_cheek",
        reason="The right arm support tube passes through the enclosed arm bracket cheek.",
    )
    for caster_index in range(5):
        ctx.allow_overlap(
            f"caster_fork_{caster_index}",
            star_base,
            elem_a="swivel_stem",
            elem_b=f"leg_{caster_index}",
            reason="Each caster stem is captured inside the leg-end socket, represented as a solid leg shell.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    for parent, child, name in (
        (star_base, gas_outer, "base_to_outer_column_contact"),
        (gas_outer, gas_inner, "outer_to_inner_column_contact"),
        (gas_inner, seat_assembly, "inner_column_to_seat_contact"),
        (seat_assembly, backrest, "seat_to_backrest_contact"),
        (seat_assembly, left_armrest, "seat_to_left_armrest_contact"),
        (seat_assembly, right_armrest, "seat_to_right_armrest_contact"),
    ):
        ctx.expect_contact(child, parent, name=name)

    ctx.expect_origin_distance(
        gas_outer,
        star_base,
        axes="xy",
        max_dist=0.001,
        name="pedestal_centered_on_base",
    )
    ctx.expect_origin_distance(
        seat_assembly,
        gas_inner,
        axes="xy",
        max_dist=0.001,
        name="seat_centered_on_gas_column",
    )
    ctx.expect_origin_gap(
        seat_assembly,
        star_base,
        axis="z",
        min_gap=0.30,
        max_gap=0.45,
        name="seat_height_above_base",
    )
    ctx.expect_origin_gap(
        seat_assembly,
        backrest,
        axis="y",
        min_gap=0.18,
        max_gap=0.28,
        name="backrest_mounts_behind_seat",
    )
    ctx.expect_origin_distance(
        left_armrest,
        right_armrest,
        axes="x",
        min_dist=0.45,
        max_dist=0.55,
        name="armrests_span_seat_width",
    )

    def expect_joint(
        joint_name: str,
        expected_type,
        expected_axis: tuple[float, float, float],
    ) -> None:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_definition",
            joint.articulation_type == expected_type
            and _axis_tuple(joint.axis) == expected_axis,
            (
                f"Expected {joint_name} to be {expected_type} on axis "
                f"{expected_axis}, got type={joint.articulation_type}, axis={joint.axis}"
            ),
        )

    expect_joint("gas_height", ArticulationType.PRISMATIC, (0.0, 0.0, 1.0))
    expect_joint("seat_swivel", ArticulationType.CONTINUOUS, (0.0, 0.0, 1.0))
    expect_joint("back_recline", ArticulationType.REVOLUTE, (1.0, 0.0, 0.0))
    expect_joint("left_arm_flip", ArticulationType.REVOLUTE, (1.0, 0.0, 0.0))
    expect_joint("right_arm_flip", ArticulationType.REVOLUTE, (1.0, 0.0, 0.0))

    for caster_index in range(5):
        fork = object_model.get_part(f"caster_fork_{caster_index}")
        wheel = object_model.get_part(f"caster_wheel_{caster_index}")
        swivel = object_model.get_articulation(f"caster_swivel_{caster_index}")
        spin = object_model.get_articulation(f"caster_wheel_spin_{caster_index}")

        ctx.expect_contact(fork, star_base, name=f"caster_fork_{caster_index}_mounted")
        ctx.expect_contact(wheel, fork, name=f"caster_wheel_{caster_index}_mounted")
        ctx.expect_origin_distance(
            fork,
            star_base,
            axes="xy",
            min_dist=0.33,
            max_dist=0.38,
            name=f"caster_fork_{caster_index}_at_leg_tip",
        )
        ctx.check(
            f"caster_swivel_{caster_index}_definition",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and _axis_tuple(swivel.axis) == (0.0, 0.0, 1.0),
            f"caster swivel {caster_index} should be continuous about +z",
        )
        ctx.check(
            f"caster_wheel_spin_{caster_index}_definition",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and _axis_tuple(spin.axis) == (0.0, 1.0, 0.0),
            f"caster wheel spin {caster_index} should be continuous about +y",
        )

    seat_rest = ctx.part_world_position(seat_assembly)
    back_rest_aabb = ctx.part_world_aabb(backrest)
    left_arm_rest_aabb = ctx.part_world_aabb(left_armrest)
    right_arm_rest_aabb = ctx.part_world_aabb(right_armrest)

    ctx.check(
        "base_to_gas_outer_fixed_joint",
        base_to_gas_outer.articulation_type == ArticulationType.FIXED,
        "The outer gas shell should be fixed into the star base hub.",
    )

    with ctx.pose({gas_height: 0.090}):
        seat_raised = ctx.part_world_position(seat_assembly)
        ctx.expect_contact(gas_inner, gas_outer, name="gas_column_stays_clipped_in_outer_shell")
        ctx.expect_contact(seat_assembly, gas_inner, name="seat_stays_seated_on_inner_column")
        ctx.check(
            "gas_lift_raises_seat",
            seat_rest is not None
            and seat_raised is not None
            and seat_raised[2] > seat_rest[2] + 0.08,
            "Height adjustment should raise the seat substantially.",
        )

    with ctx.pose({seat_swivel: math.pi / 2.0}):
        seat_swiveled = ctx.part_world_position(seat_assembly)
        ctx.expect_contact(seat_assembly, gas_inner, name="seat_swivel_keeps_swivel_plate_engaged")
        ctx.check(
            "seat_swivel_keeps_pedestal_stack",
            seat_rest is not None
            and seat_swiveled is not None
            and abs(seat_swiveled[0] - seat_rest[0]) < 1e-6
            and abs(seat_swiveled[1] - seat_rest[1]) < 1e-6
            and abs(seat_swiveled[2] - seat_rest[2]) < 1e-6,
            "Seat swivel should stay centered on the gas column.",
        )

    with ctx.pose({back_recline: math.radians(24.0)}):
        back_reclined_aabb = ctx.part_world_aabb(backrest)
        ctx.expect_contact(backrest, seat_assembly, name="backrest_recline_keeps_hinge_contact")
        ctx.check(
            "backrest_reclines_rearward",
            back_rest_aabb is not None
            and back_reclined_aabb is not None
            and back_reclined_aabb[0][1] < back_rest_aabb[0][1] - 0.03,
            "Reclining should move the backrest rearward.",
        )

    with ctx.pose({left_arm_flip: math.radians(78.0), right_arm_flip: math.radians(78.0)}):
        left_arm_up_aabb = ctx.part_world_aabb(left_armrest)
        right_arm_up_aabb = ctx.part_world_aabb(right_armrest)
        ctx.expect_contact(left_armrest, seat_assembly, name="left_armrest_keeps_hinge_contact")
        ctx.expect_contact(right_armrest, seat_assembly, name="right_armrest_keeps_hinge_contact")
        ctx.check(
            "armrests_flip_up",
            left_arm_rest_aabb is not None
            and right_arm_rest_aabb is not None
            and left_arm_up_aabb is not None
            and right_arm_up_aabb is not None
            and left_arm_up_aabb[1][2] > left_arm_rest_aabb[1][2] + 0.10
            and right_arm_up_aabb[1][2] > right_arm_rest_aabb[1][2] + 0.10,
            "Both armrests should rotate upward into a stowed position.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_armrests_flipped_up")

    first_swivel = object_model.get_articulation("caster_swivel_0")
    first_spin = object_model.get_articulation("caster_wheel_spin_0")
    with ctx.pose({first_swivel: math.radians(35.0), first_spin: 1.1}):
        ctx.expect_contact("caster_fork_0", star_base, name="caster_swivel_retains_mount_contact")
        ctx.expect_contact("caster_wheel_0", "caster_fork_0", name="caster_wheel_retains_axle_contact")

    with ctx.pose(
        {
            gas_height: 0.090,
            back_recline: math.radians(24.0),
            left_arm_flip: math.radians(78.0),
            right_arm_flip: math.radians(78.0),
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_extended_operating_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
