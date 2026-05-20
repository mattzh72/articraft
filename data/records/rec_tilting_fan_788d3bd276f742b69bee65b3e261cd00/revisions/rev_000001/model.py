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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _fan_blade_section(
    radius: float,
    chord: float,
    sweep_x: float,
    camber_y: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    return [
        (-0.48 * chord + sweep_x, -0.95 * half_t, radius),
        (0.02 * chord + sweep_x, 0.85 * half_t + camber_y, radius),
        (0.52 * chord + sweep_x, 0.18 * half_t + camber_y, radius),
        (-0.06 * chord + sweep_x, -0.85 * half_t, radius),
    ]


def _pattern_about_x(base_geom: MeshGeometry, count: int) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_x(index * math.tau / count))
    return patterned


def _fan_guard_cage() -> MeshGeometry:
    cage = TorusGeometry(
        radius=0.148,
        tube=0.0055,
        radial_segments=16,
        tubular_segments=56,
    ).rotate_y(math.pi / 2.0).translate(0.095, 0.0, 0.0)

    for index in range(8):
        angle = index * math.tau / 8.0
        c = math.cos(angle)
        s = math.sin(angle)
        rib = tube_from_spline_points(
            [
                (0.095, 0.148 * c, 0.148 * s),
                (0.118, 0.152 * c, 0.152 * s),
                (0.145, 0.155 * c, 0.155 * s),
            ],
            radius=0.0032,
            samples_per_segment=8,
            radial_segments=10,
            cap_ends=True,
            up_hint=(1.0, 0.0, 0.0),
        )
        cage.merge(rib)

    for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
        c = math.cos(angle)
        s = math.sin(angle)
        strut = tube_from_spline_points(
            [
                (0.052, 0.053 * c, 0.053 * s),
                (0.070, 0.092 * c, 0.092 * s),
                (0.095, 0.143 * c, 0.143 * s),
            ],
            radius=0.0048,
            samples_per_segment=8,
            radial_segments=12,
            cap_ends=True,
            up_hint=(1.0, 0.0, 0.0),
        )
        cage.merge(strut)

    return cage


def _fan_rotor_blades() -> MeshGeometry:
    blade = section_loft(
        [
            _fan_blade_section(0.032, 0.032, 0.004, 0.000, 0.008),
            _fan_blade_section(0.074, 0.070, 0.014, 0.008, 0.006),
            _fan_blade_section(0.118, 0.088, 0.026, 0.014, 0.0045),
        ]
    )
    return _pattern_about_x(blade, 4)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    body_white = model.material("body_white", rgba=(0.90, 0.91, 0.93, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.13, 0.14, 0.15, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.76, 0.79, 1.0))
    smoky_blade = model.material("smoky_blade", rgba=(0.52, 0.56, 0.60, 0.72))

    base = model.part("base")
    base.visual(
        Box((0.250, 0.180, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=body_white,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(-0.020, 0.0, 0.050)),
        material=body_white,
        name="pedestal_skirt",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.194),
        origin=Origin(xyz=(-0.045, 0.0, 0.165)),
        material=graphite,
        name="pedestal_column",
    )
    base.visual(
        Box((0.040, 0.104, 0.090)),
        origin=Origin(xyz=(-0.060, 0.0, 0.245)),
        material=body_white,
        name="yoke_block",
    )
    base.visual(
        Box((0.040, 0.060, 0.080)),
        origin=Origin(xyz=(-0.070, 0.0, 0.292)),
        material=body_white,
        name="yoke_stem",
    )
    base.visual(
        Box((0.085, 0.024, 0.120)),
        origin=Origin(xyz=(-0.038, 0.125, 0.330)),
        material=body_white,
        name="left_bracket",
    )
    base.visual(
        Box((0.085, 0.024, 0.120)),
        origin=Origin(xyz=(-0.038, -0.125, 0.330)),
        material=body_white,
        name="right_bracket",
    )
    base.visual(
        Box((0.038, 0.228, 0.055)),
        origin=Origin(xyz=(-0.078, 0.0, 0.334)),
        material=body_white,
        name="rear_bridge",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(
            xyz=(0.000, 0.118, 0.330),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="left_pivot_boss",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(
            xyz=(0.000, -0.118, 0.330),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="right_pivot_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.250, 0.228, 0.400)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, -0.034),
                    (0.038, -0.030),
                    (0.085, -0.008),
                    (0.098, 0.025),
                    (0.090, 0.048),
                    (0.060, 0.070),
                    (0.0, 0.072),
                ],
                segments=72,
            ).rotate_y(math.pi / 2.0),
            "fan_motor_housing",
        ),
        material=body_white,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_neck",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(
            xyz=(0.000, 0.094, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(
            xyz=(0.000, -0.094, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="right_trunnion",
    )
    head.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.155,
                tube=0.0065,
                radial_segments=18,
                tubular_segments=60,
            ).rotate_y(math.pi / 2.0).translate(0.145, 0.0, 0.0),
            "fan_front_guard_ring",
        ),
        material=metal,
        name="front_guard_ring",
    )
    head.visual(
        mesh_from_geometry(_fan_guard_cage(), "fan_guard_cage"),
        material=metal,
        name="guard_cage",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.320, 0.240, 0.240)),
        mass=1.2,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.030, length=0.038),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_collar",
    )
    rotor.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, -0.016),
                    (0.010, -0.012),
                    (0.022, -0.006),
                    (0.030, 0.006),
                    (0.016, 0.014),
                    (0.0, 0.018),
                ],
                segments=56,
            ).rotate_y(math.pi / 2.0).translate(0.015, 0.0, 0.0),
            "fan_spinner",
        ),
        material=metal,
        name="spinner",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), start=1):
        rotor.visual(
            Box((0.010, 0.105, 0.042)),
            origin=Origin(
                xyz=(0.010, 0.060 * math.cos(angle), 0.060 * math.sin(angle)),
                rpy=(angle, 0.0, math.radians(14.0)),
            ),
            material=smoky_blade,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.060),
        mass=0.22,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.093, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=32.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    with ctx.pose({tilt: 0.0}):
        ctx.expect_origin_distance(
            rotor,
            head,
            axes="yz",
            max_dist=0.001,
            name="rotor stays centered on the head axle",
        )
        ctx.expect_gap(
            head,
            rotor,
            axis="x",
            positive_elem="front_guard_ring",
            negative_elem="spinner",
            min_gap=0.012,
            max_gap=0.060,
            name="spinner sits behind the front guard",
        )
        ctx.expect_overlap(
            head,
            rotor,
            axes="yz",
            elem_a="front_guard_ring",
            min_overlap=0.220,
            name="rotor spans most of the guard opening",
        )

    rest_front = ctx.part_element_world_aabb(head, elem="front_guard_ring")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_front = ctx.part_element_world_aabb(head, elem="front_guard_ring")
    if rest_front is not None and tilted_front is not None:
        rest_center = tuple((rest_front[0][i] + rest_front[1][i]) * 0.5 for i in range(3))
        tilted_center = tuple((tilted_front[0][i] + tilted_front[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "positive tilt raises the fan head",
            tilted_center[2] > rest_center[2] + 0.06,
            details=f"rest_center={rest_center}, tilted_center={tilted_center}",
        )
    else:
        ctx.fail(
            "positive tilt raises the fan head",
            f"missing guard ring AABB data: rest={rest_front}, tilted={tilted_front}",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            rotor,
            head,
            axes="yz",
            max_dist=0.001,
            name="rotor stays centered while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
