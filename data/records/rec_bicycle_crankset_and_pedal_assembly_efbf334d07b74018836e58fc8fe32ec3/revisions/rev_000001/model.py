from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

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
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int) -> list[tuple[float, float]]:
    return [
        (radius * cos((tau * index) / segments), radius * sin((tau * index) / segments))
        for index in range(segments)
    ]


def _xy_section(
    z_pos: float,
    *,
    thickness_x: float,
    width_y: float,
    corner_radius: float,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_offset, z_pos)
        for x, y in rounded_rect_profile(thickness_x, width_y, corner_radius, corner_segments=6)
    ]


def _yz_section(
    x_pos: float,
    *,
    width_y: float,
    height_z: float,
    corner_radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z_center + z)
        for y, z in rounded_rect_profile(width_y, height_z, corner_radius, corner_segments=6)
    ]


def _build_crank_arm_mesh() -> object:
    return section_loft(
        [
            _xy_section(
                0.000,
                thickness_x=0.020,
                width_y=0.040,
                corner_radius=0.006,
                y_offset=0.000,
            ),
            _xy_section(
                -0.050,
                thickness_x=0.017,
                width_y=0.028,
                corner_radius=0.004,
                y_offset=0.003,
            ),
            _xy_section(
                -0.112,
                thickness_x=0.015,
                width_y=0.023,
                corner_radius=0.003,
                y_offset=0.007,
            ),
            _xy_section(
                -0.170,
                thickness_x=0.016,
                width_y=0.030,
                corner_radius=0.004,
                y_offset=0.011,
            ),
        ]
    )


def _build_housing_meshes() -> tuple[object, object]:
    upper = section_loft(
        [
            _yz_section(
                -0.075,
                width_y=0.098,
                height_z=0.060,
                corner_radius=0.012,
                z_center=0.126,
            ),
            _yz_section(
                0.000,
                width_y=0.108,
                height_z=0.064,
                corner_radius=0.013,
                z_center=0.126,
            ),
            _yz_section(
                0.075,
                width_y=0.098,
                height_z=0.060,
                corner_radius=0.012,
                z_center=0.126,
            ),
        ]
    )
    lower = section_loft(
        [
            _yz_section(
                -0.072,
                width_y=0.090,
                height_z=0.074,
                corner_radius=0.013,
                z_center=0.038,
            ),
            _yz_section(
                0.000,
                width_y=0.108,
                height_z=0.084,
                corner_radius=0.015,
                z_center=0.036,
            ),
            _yz_section(
                0.072,
                width_y=0.090,
                height_z=0.074,
                corner_radius=0.013,
                z_center=0.038,
            ),
        ]
    )
    return upper, lower


def _build_chainring_mesh(
    *,
    outer_radius: float = 0.086,
    tooth_root_radius: float = 0.079,
    inner_radius: float = 0.052,
    teeth: int = 34,
) -> object:
    outer_profile: list[tuple[float, float]] = []
    for index in range(teeth * 2):
        angle = (tau * index) / (teeth * 2)
        radius = outer_radius if index % 2 == 0 else tooth_root_radius
        outer_profile.append((radius * cos(angle), radius * sin(angle)))

    hole_profiles = [_circle_profile(inner_radius, segments=44)]
    bolt_circle = 0.045
    bolt_hole_radius = 0.0044
    for bolt_index in range(5):
        angle = (tau * bolt_index) / 5.0 + pi / 10.0
        cx = bolt_circle * cos(angle)
        cy = bolt_circle * sin(angle)
        hole_profiles.append(
            [
                (
                    cx + bolt_hole_radius * cos((tau * sample) / 18),
                    cy + bolt_hole_radius * sin((tau * sample) / 18),
                )
                for sample in range(18)
            ]
        )

    return ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=0.004,
        center=True,
    ).rotate_y(pi / 2.0)


def _build_pedal_platform_mesh() -> object:
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.108, 0.082, 0.012, corner_segments=8),
        [rounded_rect_profile(0.072, 0.046, 0.010, corner_segments=8)],
        height=0.012,
        center=True,
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ebike_mid_motor_crankset")

    housing_black = model.material("housing_black", rgba=(0.14, 0.15, 0.16, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pedal_alloy = model.material("pedal_alloy", rgba=(0.69, 0.71, 0.74, 1.0))
    pedal_rubber = model.material("pedal_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.150, 0.108, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=housing_black,
        name="upper_housing",
    )
    housing.visual(
        Box((0.118, 0.094, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=housing_black,
        name="lower_housing",
    )
    housing.visual(
        Box((0.150, 0.020, 0.068)),
        origin=Origin(xyz=(0.0, 0.036, 0.082)),
        material=housing_black,
        name="front_bridge",
    )
    housing.visual(
        Box((0.150, 0.020, 0.068)),
        origin=Origin(xyz=(0.0, -0.036, 0.082)),
        material=housing_black,
        name="rear_bridge",
    )
    housing.visual(
        Box((0.110, 0.048, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=housing_black,
        name="skid_plate",
    )
    housing.visual(
        Box((0.036, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.046, 0.169)),
        material=housing_black,
        name="top_mount",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.180, 0.115, 0.160)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    spindle = model.part("spindle_assembly")
    spindle.visual(
        Cylinder(radius=0.014, length=0.174),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="spindle_axle",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(-0.081, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="left_spacer",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.081, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="right_spacer",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.186),
        mass=1.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="spindle_eye",
    )
    left_crank.visual(
        _save_mesh("left_crank_arm", _build_crank_arm_mesh()),
        material=forged_steel,
        name="arm_body",
    )
    left_crank.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.011, -0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="pedal_eye",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.020, 0.050, 0.188)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.006, -0.085)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="spindle_eye",
    )
    right_crank.visual(
        _save_mesh("right_crank_arm", _build_crank_arm_mesh()),
        material=forged_steel,
        name="arm_body",
    )
    right_crank.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.011, -0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="pedal_eye",
    )
    right_crank.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="spider_hub",
    )
    spider_angles = (0.0, 2.0 * pi / 5.0, 4.0 * pi / 5.0, 6.0 * pi / 5.0, 8.0 * pi / 5.0)
    for index, angle in enumerate(spider_angles):
        right_crank.visual(
            Box((0.004, 0.008, 0.070)),
            origin=Origin(
                xyz=(0.014, 0.0, 0.035),
                rpy=(angle, 0.0, 0.0),
            ),
            material=forged_steel,
            name=f"spider_arm_{index}",
        )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.026, 0.150, 0.188)),
        mass=1.2,
        origin=Origin(xyz=(0.010, 0.0, -0.060)),
    )

    chainring = model.part("chainring")
    chainring.visual(
        _save_mesh("single_chainring", _build_chainring_mesh()),
        material=chainring_black,
        name="chainring_shell",
    )
    chainring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.004),
        mass=0.42,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        pedal = model.part(f"{side_name}_pedal")
        pedal.visual(
            Cylinder(radius=0.0055, length=0.028),
            origin=Origin(xyz=(side_sign * 0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=forged_steel,
            name="stub_spindle",
        )
        pedal.visual(
            Cylinder(radius=0.0115, length=0.050),
            origin=Origin(xyz=(side_sign * 0.052, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=pedal_alloy,
            name="center_barrel",
        )
        pedal.visual(
            _save_mesh(f"{side_name}_platform_pedal", _build_pedal_platform_mesh()),
            origin=Origin(xyz=(side_sign * 0.056, 0.0, 0.0)),
            material=pedal_alloy,
            name="platform_frame",
        )
        pedal.visual(
            Box((0.010, 0.086, 0.008)),
            origin=Origin(xyz=(side_sign * 0.056, 0.0, 0.020)),
            material=pedal_alloy,
            name="upper_crossbar",
        )
        pedal.visual(
            Box((0.010, 0.086, 0.008)),
            origin=Origin(xyz=(side_sign * 0.056, 0.0, -0.020)),
            material=pedal_alloy,
            name="lower_crossbar",
        )
        pedal.visual(
            Box((0.006, 0.060, 0.026)),
            origin=Origin(xyz=(side_sign * 0.056, 0.0, 0.0)),
            material=pedal_rubber,
            name="grip_pad",
        )
        pedal.inertial = Inertial.from_geometry(
            Box((0.090, 0.100, 0.028)),
            mass=0.38,
            origin=Origin(xyz=(side_sign * 0.056, 0.0, 0.0)),
        )

    model.articulation(
        "housing_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=14.0),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.096, 0.0, 0.0), rpy=(pi, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
    )
    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank,
        child=chainring,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child="left_pedal",
        origin=Origin(xyz=(0.0, 0.011, -0.170)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child="right_pedal",
        origin=Origin(xyz=(0.0, 0.011, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    spindle = object_model.get_part("spindle_assembly")
    left_crank = object_model.get_part("left_crank")
    right_crank = object_model.get_part("right_crank")
    chainring = object_model.get_part("chainring")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    crank_spin = object_model.get_articulation("housing_to_spindle")
    left_pedal_spin = object_model.get_articulation("left_crank_to_left_pedal")
    right_pedal_spin = object_model.get_articulation("right_crank_to_right_pedal")

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
        "crank spindle articulation is lateral continuous rotation",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(crank_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={crank_spin.articulation_type}, axis={crank_spin.axis}",
    )
    ctx.check(
        "pedals spin on outward stub axles",
        tuple(left_pedal_spin.axis) == (-1.0, 0.0, 0.0) and tuple(right_pedal_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_pedal_spin.axis}, right={right_pedal_spin.axis}",
    )

    ctx.expect_contact(
        spindle,
        housing,
        elem_a="right_spacer",
        name="right spindle spacer seats against the motor housing",
    )
    ctx.expect_contact(
        spindle,
        housing,
        elem_a="left_spacer",
        name="left spindle spacer seats against the motor housing",
    )
    ctx.expect_contact(
        chainring,
        right_crank,
        name="chainring is bolted flush to the right crank spider",
    )
    ctx.expect_contact(
        left_pedal,
        left_crank,
        name="left pedal spindle seats against the left crank eye",
    )
    ctx.expect_contact(
        right_pedal,
        right_crank,
        name="right pedal spindle seats against the right crank eye",
    )
    ctx.expect_overlap(
        chainring,
        right_crank,
        axes="yz",
        min_overlap=0.020,
        name="chainring overlaps the spider footprint in the ring plane",
    )

    rest_left = ctx.part_world_position(left_pedal)
    rest_right = ctx.part_world_position(right_pedal)
    spindle_pos = ctx.part_world_position(spindle)
    ctx.check(
        "crank arms start 180 degrees opposed",
        rest_left is not None
        and rest_right is not None
        and spindle_pos is not None
        and rest_left[2] > spindle_pos[2] + 0.14
        and rest_right[2] < spindle_pos[2] - 0.14,
        details=f"left={rest_left}, right={rest_right}, spindle={spindle_pos}",
    )

    with ctx.pose({crank_spin: 1.2}):
        raised_right = ctx.part_world_position(right_pedal)
        raised_chainring = ctx.part_world_position(chainring)

    ctx.check(
        "positive crank rotation lifts the right pedal",
        rest_right is not None and raised_right is not None and raised_right[2] > rest_right[2] + 0.10,
        details=f"rest={rest_right}, raised={raised_right}",
    )
    ctx.check(
        "chainring rotates with the right crank on the drive side",
        raised_chainring is not None and raised_chainring[0] > 0.10,
        details=f"chainring_position={raised_chainring}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
