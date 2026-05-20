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
    tube_from_spline_points,
)


def _section_at_y(
    y: float,
    width: float,
    thickness: float,
    z_center: float,
    *,
    corner: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for x, z in rounded_rect_profile(
            width,
            thickness,
            corner,
            corner_segments=corner_segments,
        )
    ]


def _section_at_z(
    z: float,
    width: float,
    depth: float,
    y_center: float,
    *,
    corner: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            corner,
            corner_segments=corner_segments,
        )
    ]


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _caster_tire_mesh():
    outer_profile = [
        (0.012, -0.014),
        (0.018, -0.011),
        (0.026, -0.0095),
        (0.029, -0.0075),
        (0.029, 0.0075),
        (0.026, 0.0095),
        (0.018, 0.011),
        (0.012, 0.014),
    ]
    inner_profile = [
        (0.005, -0.014),
        (0.005, 0.014),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ).rotate_y(math.pi / 2.0),
        "office_chair_caster_tire_shell_v2",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair")

    nylon_black = model.material("nylon_black", rgba=(0.10, 0.11, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.16, 1.0))
    charcoal_mesh = model.material("charcoal_mesh", rgba=(0.21, 0.22, 0.24, 1.0))
    cushion_grey = model.material("cushion_grey", rgba=(0.29, 0.30, 0.32, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    seat_cushion_mesh = mesh_from_geometry(
        section_loft(
            [
                _section_at_y(-0.22, 0.40, 0.055, 0.040, corner=0.016),
                _section_at_y(-0.06, 0.50, 0.085, 0.048, corner=0.026),
                _section_at_y(0.12, 0.52, 0.095, 0.045, corner=0.028),
                _section_at_y(0.24, 0.46, 0.075, 0.037, corner=0.024),
            ]
        ),
        "office_chair_seat_cushion",
    )
    back_panel_mesh = mesh_from_geometry(
        section_loft(
            [
                _section_at_z(0.05, 0.26, 0.030, -0.004, corner=0.014),
                _section_at_z(0.20, 0.34, 0.060, -0.014, corner=0.028),
                _section_at_z(0.42, 0.32, 0.055, -0.030, corner=0.026),
                _section_at_z(0.58, 0.24, 0.040, -0.050, corner=0.020),
            ]
        ),
        "office_chair_back_panel",
    )
    headrest_mesh = mesh_from_geometry(
        section_loft(
            [
                _section_at_y(-0.055, 0.20, 0.052, 0.045, corner=0.016),
                _section_at_y(0.000, 0.28, 0.082, 0.056, corner=0.026),
                _section_at_y(0.055, 0.24, 0.060, 0.047, corner=0.020),
            ]
        ),
        "office_chair_headrest_pad",
    )
    caster_tire_mesh = _caster_tire_mesh()
    left_back_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.18, 0.000, 0.015),
                (0.18, -0.010, 0.180),
                (0.17, -0.030, 0.420),
                (0.13, -0.050, 0.600),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "office_chair_left_back_rail",
    )
    right_back_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _mirror_x(
                [
                    (0.18, 0.000, 0.015),
                    (0.18, -0.010, 0.180),
                    (0.17, -0.030, 0.420),
                    (0.13, -0.050, 0.600),
                ]
            ),
            radius=0.012,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "office_chair_right_back_rail",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=nylon_black,
        name="center_hub",
    )
    bell_shell = LatheGeometry.from_shell_profiles(
        [
            (0.074, 0.000),
            (0.068, 0.010),
            (0.054, 0.032),
            (0.039, 0.056),
            (0.028, 0.070),
        ],
        [
            (0.044, 0.006),
            (0.040, 0.014),
            (0.032, 0.032),
            (0.026, 0.056),
            (0.021, 0.070),
        ],
        segments=56,
    )
    base.visual(
        mesh_from_geometry(bell_shell, "office_chair_bell_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=matte_black,
        name="bell_shell",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=matte_black,
        name="column_bushing",
    )

    leg_mid_radius = 0.170
    leg_tip_radius = 0.305
    for index in range(5):
        angle = math.pi / 2.0 + index * 2.0 * math.pi / 5.0
        mid_x = leg_mid_radius * math.cos(angle)
        mid_y = leg_mid_radius * math.sin(angle)
        tip_x = leg_tip_radius * math.cos(angle)
        tip_y = leg_tip_radius * math.sin(angle)
        base.visual(
            Box((0.270, 0.045, 0.018)),
            origin=Origin(xyz=(mid_x, mid_y, 0.100), rpy=(0.0, 0.0, angle)),
            material=nylon_black,
            name=f"leg_{index}",
        )
        base.visual(
            Box((0.052, 0.062, 0.008)),
            origin=Origin(xyz=(tip_x, tip_y, 0.101), rpy=(0.0, 0.0, angle)),
            material=nylon_black,
            name=f"foot_pad_{index}",
        )
        base.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(tip_x, tip_y, 0.093)),
            material=matte_black,
            name=f"caster_socket_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.720, 0.720, 0.180)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    seat_support = model.part("seat_support")
    seat_support.visual(
        Cylinder(radius=0.024, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_metal,
        name="gas_lift_shaft",
    )
    seat_support.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_metal,
        name="upper_column",
    )
    seat_support.visual(
        Box((0.180, 0.160, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=matte_black,
        name="tilt_housing",
    )
    seat_support.visual(
        Box((0.080, 0.190, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, 0.112)),
        material=matte_black,
        name="tilt_backbone",
    )
    seat_support.inertial = Inertial.from_geometry(
        Box((0.200, 0.190, 0.180)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.260, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=matte_black,
        name="seat_mount_board",
    )
    seat.visual(
        Box((0.440, 0.360, 0.024)),
        origin=Origin(xyz=(0.0, 0.015, 0.020)),
        material=nylon_black,
        name="seat_shell",
    )
    seat.visual(
        Box((0.220, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, -0.185, 0.006)),
        material=nylon_black,
        name="rear_mount_pad",
    )
    seat.visual(
        seat_cushion_mesh,
        material=cushion_grey,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.540, 0.500, 0.105)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.020, 0.050)),
    )

    rear_mechanism = model.part("rear_mechanism")
    rear_mechanism.visual(
        Box((0.220, 0.080, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=matte_black,
        name="mechanism_mount_plate",
    )
    rear_mechanism.visual(
        Box((0.022, 0.040, 0.090)),
        origin=Origin(xyz=(-0.085, -0.060, 0.045)),
        material=matte_black,
        name="left_recline_cheek",
    )
    rear_mechanism.visual(
        Box((0.022, 0.040, 0.090)),
        origin=Origin(xyz=(0.085, -0.060, 0.045)),
        material=matte_black,
        name="right_recline_cheek",
    )
    rear_mechanism.visual(
        Box((0.180, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.070, 0.099)),
        material=matte_black,
        name="hinge_saddle",
    )
    rear_mechanism.visual(
        Cylinder(radius=0.011, length=0.160),
        origin=Origin(xyz=(0.0, -0.070, 0.099), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_cross_tube",
    )
    rear_mechanism.inertial = Inertial.from_geometry(
        Box((0.240, 0.090, 0.120)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.010, 0.045)),
    )

    back = model.part("back")
    back.visual(
        Box((0.180, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="back_hinge_block",
    )
    back.visual(
        left_back_rail_mesh,
        material=nylon_black,
        name="left_back_rail",
    )
    back.visual(
        right_back_rail_mesh,
        material=nylon_black,
        name="right_back_rail",
    )
    back.visual(
        Cylinder(radius=0.012, length=0.260),
        origin=Origin(xyz=(0.0, -0.050, 0.600), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nylon_black,
        name="top_back_bar",
    )
    back.visual(
        back_panel_mesh,
        material=charcoal_mesh,
        name="back_panel",
    )
    back.visual(
        Box((0.060, 0.034, 0.620)),
        origin=Origin(xyz=(0.0, -0.022, 0.319)),
        material=matte_black,
        name="center_back_spine",
    )
    back.visual(
        Box((0.070, 0.028, 0.430)),
        origin=Origin(xyz=(-0.145, -0.024, 0.245)),
        material=matte_black,
        name="left_panel_web",
    )
    back.visual(
        Box((0.070, 0.028, 0.430)),
        origin=Origin(xyz=(0.145, -0.024, 0.245)),
        material=matte_black,
        name="right_panel_web",
    )
    back.visual(
        Box((0.100, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, -0.050, 0.610)),
        material=matte_black,
        name="headrest_mount_block",
    )
    back.inertial = Inertial.from_geometry(
        Box((0.400, 0.120, 0.650)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.025, 0.315)),
    )

    headrest_bracket = model.part("headrest_bracket")
    headrest_bracket.visual(
        Box((0.100, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="bracket_base",
    )
    headrest_bracket.visual(
        Cylinder(radius=0.009, length=0.110),
        origin=Origin(xyz=(-0.040, -0.006, 0.075)),
        material=satin_metal,
        name="left_bracket_post",
    )
    headrest_bracket.visual(
        Cylinder(radius=0.009, length=0.110),
        origin=Origin(xyz=(0.040, -0.006, 0.075)),
        material=satin_metal,
        name="right_bracket_post",
    )
    headrest_bracket.visual(
        Cylinder(radius=0.009, length=0.100),
        origin=Origin(xyz=(0.0, -0.015, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="top_bridge",
    )
    headrest_bracket.visual(
        Box((0.090, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.015, 0.141)),
        material=matte_black,
        name="pivot_pad",
    )
    headrest_bracket.inertial = Inertial.from_geometry(
        Box((0.120, 0.050, 0.150)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.010, 0.080)),
    )

    headrest = model.part("headrest")
    headrest.visual(
        Box((0.090, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="headrest_mount_block",
    )
    headrest.visual(
        headrest_mesh,
        material=cushion_grey,
        name="headrest_pad",
    )
    headrest.inertial = Inertial.from_geometry(
        Box((0.300, 0.140, 0.090)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "base_to_seat_support",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat_support,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "seat_support_to_seat",
        ArticulationType.CONTINUOUS,
        parent=seat_support,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "seat_to_rear_mechanism",
        ArticulationType.FIXED,
        parent=seat,
        child=rear_mechanism,
        origin=Origin(xyz=(0.0, -0.185, 0.0)),
    )
    model.articulation(
        "rear_mechanism_to_back",
        ArticulationType.REVOLUTE,
        parent=rear_mechanism,
        child=back,
        origin=Origin(xyz=(0.0, -0.070, 0.109)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=0.52,
        ),
    )
    model.articulation(
        "back_to_headrest_bracket",
        ArticulationType.FIXED,
        parent=back,
        child=headrest_bracket,
        origin=Origin(xyz=(0.0, -0.050, 0.620)),
    )
    model.articulation(
        "headrest_bracket_to_headrest",
        ArticulationType.REVOLUTE,
        parent=headrest_bracket,
        child=headrest,
        origin=Origin(xyz=(0.0, -0.015, 0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.35,
            upper=0.30,
        ),
    )

    for index in range(5):
        angle = math.pi / 2.0 + index * 2.0 * math.pi / 5.0
        tip_x = leg_tip_radius * math.cos(angle)
        tip_y = leg_tip_radius * math.sin(angle)

        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=matte_black,
            name="fork_top_cap",
        )
        fork.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=satin_metal,
            name="fork_stem",
        )
        fork.visual(
            Box((0.048, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material=matte_black,
            name="fork_crown",
        )
        fork.visual(
            Box((0.004, 0.012, 0.038)),
            origin=Origin(xyz=(-0.024, 0.0, -0.056)),
            material=matte_black,
            name="left_fork_arm",
        )
        fork.visual(
            Box((0.004, 0.012, 0.038)),
            origin=Origin(xyz=(0.024, 0.0, -0.056)),
            material=matte_black,
            name="right_fork_arm",
        )
        fork.visual(
            Cylinder(radius=0.004, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, -0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name="axle_shaft",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.050, 0.030, 0.085)),
            mass=0.12,
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            caster_tire_mesh,
            material=wheel_rubber,
            name="caster_tire",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nylon_black,
            name="wheel_hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.029, length=0.028),
            mass=0.16,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"base_to_caster_fork_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(
                xyz=(tip_x, tip_y, 0.089),
                rpy=(0.0, 0.0, angle - math.pi / 2.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=12.0,
            ),
        )
        model.articulation(
            f"caster_fork_{index}_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.068)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=25.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat_support = object_model.get_part("seat_support")
    seat = object_model.get_part("seat")
    rear_mechanism = object_model.get_part("rear_mechanism")
    back = object_model.get_part("back")
    headrest_bracket = object_model.get_part("headrest_bracket")
    headrest = object_model.get_part("headrest")

    base_to_seat_support = object_model.get_articulation("base_to_seat_support")
    seat_support_to_seat = object_model.get_articulation("seat_support_to_seat")
    rear_mechanism_to_back = object_model.get_articulation("rear_mechanism_to_back")
    headrest_bracket_to_headrest = object_model.get_articulation("headrest_bracket_to_headrest")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    for index in range(5):
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="axle_shaft",
            elem_b="wheel_hub",
            reason="Wheel hub visually surrounds the fixed axle shaft without modeling the internal bore.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "major office chair parts present",
        len(object_model.parts) == 17,
        f"expected 17 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "seat height axis is vertical",
        base_to_seat_support.axis == (0.0, 0.0, 1.0),
        f"unexpected base_to_seat_support axis: {base_to_seat_support.axis}",
    )
    ctx.check(
        "seat swivel is continuous vertical",
        seat_support_to_seat.articulation_type == ArticulationType.CONTINUOUS
        and seat_support_to_seat.axis == (0.0, 0.0, 1.0),
        f"unexpected seat swivel: type={seat_support_to_seat.articulation_type}, axis={seat_support_to_seat.axis}",
    )
    ctx.check(
        "back recline is transverse",
        rear_mechanism_to_back.axis == (1.0, 0.0, 0.0),
        f"unexpected back recline axis: {rear_mechanism_to_back.axis}",
    )
    ctx.check(
        "headrest hinge is transverse",
        headrest_bracket_to_headrest.axis == (1.0, 0.0, 0.0),
        f"unexpected headrest hinge axis: {headrest_bracket_to_headrest.axis}",
    )

    ctx.expect_contact(
        seat_support,
        base,
        elem_a="gas_lift_shaft",
        elem_b="column_bushing",
        name="gas lift seated in base bushing",
    )
    ctx.expect_contact(
        seat,
        seat_support,
        elem_a="seat_mount_board",
        elem_b="tilt_housing",
        name="seat mounted to tilt housing",
    )
    ctx.expect_contact(
        rear_mechanism,
        seat,
        elem_a="mechanism_mount_plate",
        elem_b="rear_mount_pad",
        name="rear mechanism bolted to seat shell",
    )
    ctx.expect_gap(
        back,
        rear_mechanism,
        axis="z",
        positive_elem="back_hinge_block",
        negative_elem="hinge_saddle",
        min_gap=0.0,
        max_gap=0.001,
        name="back seated on recline hinge saddle",
    )
    ctx.expect_contact(
        headrest_bracket,
        back,
        elem_a="bracket_base",
        elem_b="headrest_mount_block",
        name="headrest bracket mounted to back frame",
    )
    ctx.expect_contact(
        headrest,
        headrest_bracket,
        elem_a="headrest_mount_block",
        elem_b="pivot_pad",
        name="headrest pad captured on bracket hinge",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        min_gap=0.16,
        name="seat clears five-star base",
    )

    for index in range(5):
        fork = object_model.get_part(f"caster_fork_{index}")
        wheel = object_model.get_part(f"caster_wheel_{index}")
        swivel = object_model.get_articulation(f"base_to_caster_fork_{index}")
        spin = object_model.get_articulation(f"caster_fork_{index}_to_wheel_{index}")

        ctx.expect_contact(
            fork,
            base,
            elem_a="fork_top_cap",
            elem_b=f"caster_socket_{index}",
            name=f"caster fork {index} seated in socket",
        )
        ctx.expect_contact(
            wheel,
            fork,
            name=f"caster wheel {index} captured by fork",
        )
        ctx.expect_origin_distance(
            wheel,
            fork,
            axes="xy",
            max_dist=0.001,
            name=f"caster wheel {index} aligned on fork centerline",
        )
        ctx.expect_gap(
            fork,
            wheel,
            axis="z",
            positive_elem="fork_crown",
            negative_elem="caster_tire",
            min_gap=0.001,
            max_gap=0.004,
            name=f"caster wheel {index} clears fork crown",
        )
        ctx.check(
            f"caster swivel {index} axis",
            swivel.axis == (0.0, 0.0, 1.0),
            f"unexpected swivel axis for caster {index}: {swivel.axis}",
        )
        ctx.check(
            f"caster wheel {index} spin axis",
            spin.axis == (1.0, 0.0, 0.0),
            f"unexpected wheel spin axis for caster {index}: {spin.axis}",
        )

    seat_rest = ctx.part_world_position(seat)
    back_rest_aabb = ctx.part_world_aabb(back)
    headrest_rest_aabb = ctx.part_world_aabb(headrest)
    headrest_rest_pos = ctx.part_world_position(headrest)
    back_rest_pos = ctx.part_world_position(back)
    assert seat_rest is not None
    assert back_rest_aabb is not None
    assert headrest_rest_aabb is not None
    assert headrest_rest_pos is not None
    assert back_rest_pos is not None

    ctx.check(
        "headrest sits above back frame",
        headrest_rest_pos[2] > back_rest_pos[2] + 0.18,
        f"headrest z={headrest_rest_pos[2]:.3f}, back z={back_rest_pos[2]:.3f}",
    )

    with ctx.pose({base_to_seat_support: 0.100}):
        seat_high = ctx.part_world_position(seat)
        assert seat_high is not None
        ctx.check(
            "seat height adjustment raises chair",
            seat_high[2] > seat_rest[2] + 0.095,
            f"seat rest z={seat_rest[2]:.3f}, raised z={seat_high[2]:.3f}",
        )

    with ctx.pose({rear_mechanism_to_back: 0.42}):
        back_reclined_aabb = ctx.part_world_aabb(back)
        assert back_reclined_aabb is not None
        ctx.check(
            "back recline moves top rearward",
            back_reclined_aabb[0][1] < back_rest_aabb[0][1] - 0.020,
            f"rest min y={back_rest_aabb[0][1]:.3f}, reclined min y={back_reclined_aabb[0][1]:.3f}",
        )

    with ctx.pose({headrest_bracket_to_headrest: 0.24}):
        headrest_tilted_aabb = ctx.part_world_aabb(headrest)
        assert headrest_tilted_aabb is not None
        ctx.check(
            "headrest tilt moves pad rearward",
            headrest_tilted_aabb[0][1] < headrest_rest_aabb[0][1] - 0.010,
            f"rest min y={headrest_rest_aabb[0][1]:.3f}, tilted min y={headrest_tilted_aabb[0][1]:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
