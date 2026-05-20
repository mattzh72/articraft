from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    body_white = model.material("body_white", rgba=(0.90, 0.91, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    metal = model.material("metal", rgba=(0.67, 0.69, 0.73, 1.0))
    blade_smoke = model.material("blade_smoke", rgba=(0.25, 0.28, 0.32, 0.58))

    def merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def cylinder_x(radius: float, length: float, x_center: float) -> MeshGeometry:
        return CylinderGeometry(radius=radius, height=length, radial_segments=32).rotate_y(
            math.pi / 2.0
        ).translate(x_center, 0.0, 0.0)

    def cylinder_y(radius: float, length: float, y_center: float) -> MeshGeometry:
        return CylinderGeometry(radius=radius, height=length, radial_segments=32).rotate_x(
            math.pi / 2.0
        ).translate(0.0, y_center, 0.0)

    def ring_x(radius: float, tube: float, x_center: float) -> MeshGeometry:
        return TorusGeometry(
            radius=radius,
            tube=tube,
            radial_segments=16,
            tubular_segments=56,
        ).rotate_y(math.pi / 2.0).translate(x_center, 0.0, 0.0)

    def point_on_x_ring(x_pos: float, radius: float, angle: float) -> tuple[float, float, float]:
        return (x_pos, radius * math.cos(angle), radius * math.sin(angle))

    def build_yoke_mesh() -> MeshGeometry:
        yoke = tube_from_spline_points(
            [
                (-0.004, 0.110, 0.265),
                (-0.036, 0.102, 0.258),
                (-0.056, 0.070, 0.230),
                (-0.064, 0.000, 0.192),
                (-0.056, -0.070, 0.230),
                (-0.036, -0.102, 0.258),
                (-0.004, -0.110, 0.265),
            ],
            radius=0.0085,
            samples_per_segment=16,
            radial_segments=18,
        )
        yoke.merge(
            tube_from_spline_points(
                [
                    (-0.004, 0.000, 0.205),
                    (-0.026, 0.000, 0.202),
                    (-0.050, 0.000, 0.196),
                    (-0.064, 0.000, 0.192),
                ],
                radius=0.008,
                samples_per_segment=12,
                radial_segments=16,
            )
        )
        return yoke

    def build_head_motor_mesh() -> MeshGeometry:
        housing = merge_geometries(
            cylinder_x(radius=0.060, length=0.060, x_center=0.050),
            cylinder_x(radius=0.042, length=0.032, x_center=0.020),
            cylinder_y(radius=0.011, length=0.192, y_center=0.0).translate(-0.004, 0.0, 0.0),
        )
        for side in (1.0, -1.0):
            housing.merge(
                cylinder_y(radius=0.011, length=0.018, y_center=0.104 * side).translate(
                    0.007, 0.0, 0.0
                )
            )
            housing.merge(
                tube_from_spline_points(
                    [
                        (0.010, 0.083 * side, 0.0),
                        (0.016, 0.080 * side, 0.0),
                        (0.024, 0.072 * side, 0.0),
                        (0.034, 0.058 * side, 0.0),
                    ],
                    radius=0.0075,
                    samples_per_segment=10,
                    radial_segments=16,
                )
            )
        return housing

    def build_guard_mesh() -> MeshGeometry:
        cage = merge_geometries(
            ring_x(radius=0.108, tube=0.0032, x_center=0.002),
            ring_x(radius=0.111, tube=0.0032, x_center=0.038),
            ring_x(radius=0.114, tube=0.0032, x_center=0.074),
            ring_x(radius=0.018, tube=0.0026, x_center=0.074),
        )

        for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            front_spoke = tube_from_spline_points(
                [
                    point_on_x_ring(0.074, 0.018, angle),
                    point_on_x_ring(0.072, 0.056, angle),
                    point_on_x_ring(0.074, 0.111, angle),
                ],
                radius=0.0023,
                samples_per_segment=8,
                radial_segments=12,
            )
            cage.merge(front_spoke)

        support_angles = (
            math.pi / 4.0,
            3.0 * math.pi / 4.0,
            5.0 * math.pi / 4.0,
            7.0 * math.pi / 4.0,
        )
        for angle in support_angles:
            rear_support = tube_from_spline_points(
                [
                    point_on_x_ring(-0.010, 0.054, angle),
                    point_on_x_ring(-0.002, 0.076, angle),
                    point_on_x_ring(0.002, 0.108, angle),
                ],
                radius=0.0038,
                samples_per_segment=10,
                radial_segments=14,
            )
            cage.merge(rear_support)

        for angle in [index * math.tau / 10.0 for index in range(10)]:
            longitudinal_rib = tube_from_spline_points(
                [
                    point_on_x_ring(0.002, 0.108, angle),
                    point_on_x_ring(0.038, 0.111, angle),
                    point_on_x_ring(0.074, 0.114, angle),
                ],
                radius=0.0025,
                samples_per_segment=10,
                radial_segments=12,
            )
            cage.merge(longitudinal_rib)

        return cage

    def blade_sections() -> list[list[tuple[float, float, float]]]:
        return [
            [
                (-0.012, 0.024, -0.012),
                (0.018, 0.024, -0.003),
                (0.013, 0.024, 0.016),
                (-0.012, 0.024, 0.010),
            ],
            [
                (-0.002, 0.068, -0.038),
                (0.030, 0.068, -0.014),
                (0.021, 0.068, 0.046),
                (-0.004, 0.068, 0.022),
            ],
            [
                (0.008, 0.098, -0.026),
                (0.024, 0.098, -0.011),
                (0.017, 0.098, 0.030),
                (0.009, 0.098, 0.015),
            ],
        ]

    def build_rotor_mesh() -> MeshGeometry:
        hub = merge_geometries(
            cylinder_x(radius=0.020, length=0.020, x_center=-0.008),
            cylinder_x(radius=0.034, length=0.008, x_center=0.000),
            SphereGeometry(radius=0.014, width_segments=24, height_segments=16)
            .scale(1.35, 1.0, 1.0)
            .translate(0.010, 0.0, 0.0),
        )
        hub.merge(
            ConeGeometry(radius=0.016, height=0.018, radial_segments=32)
            .rotate_y(math.pi / 2.0)
            .translate(0.020, 0.0, 0.0)
        )

        blade = section_loft(blade_sections())
        blade = blade.rotate_x(math.radians(-18.0)).translate(0.002, 0.0, 0.0)
        for angle in (0.0, math.tau / 3.0, 2.0 * math.tau / 3.0):
            hub.merge(blade.copy().rotate_x(angle))
        return hub

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.112, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=body_white,
        name="base_pan",
    )
    base.visual(
        Cylinder(radius=0.054, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=body_white,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=metal,
        name="stem",
    )
    base.visual(
        mesh_from_geometry(build_yoke_mesh(), "fan_yoke"),
        material=body_white,
        name="yoke_frame",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.30),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(build_head_motor_mesh(), "fan_head_motor"),
        material=body_white,
        name="motor_housing",
    )
    head.visual(
        mesh_from_geometry(build_guard_mesh(), "fan_guard_cage"),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=metal,
        name="guard_cage",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.195, 0.235, 0.235)),
        mass=1.2,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(build_rotor_mesh(), "fan_rotor"),
        material=blade_smoke,
        name="blade_set",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.052),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    tilt_joint = model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=math.radians(-18.0),
            upper=math.radians(28.0),
        ),
    )

    rotor_joint = model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt_joint = object_model.get_articulation("base_to_head")

    ctx.expect_origin_distance(
        head,
        rotor,
        axes="yz",
        max_dist=1e-6,
        name="rotor stays centered on the head axis",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="blade_set",
        outer_elem="guard_cage",
        margin=0.0,
        name="rotor remains inside the guard footprint",
    )
    ctx.expect_origin_distance(
        base,
        rotor,
        axes="y",
        max_dist=1e-6,
        name="rotor stays centered on the support centerline",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="blade_set",
        negative_elem="motor_housing",
        min_gap=0.002,
        name="rotor clears the motor housing",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        tilted_pos = ctx.part_world_position(rotor)

    ctx.check(
        "positive tilt raises the fan center",
        rest_pos is not None
        and tilted_pos is not None
        and tilted_pos[2] > rest_pos[2] + 0.01,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
