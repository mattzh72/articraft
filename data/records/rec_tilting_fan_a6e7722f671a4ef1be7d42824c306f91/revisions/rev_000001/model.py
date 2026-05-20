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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    base_dark = model.material("base_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    shell_light = model.material("shell_light", rgba=(0.82, 0.84, 0.86, 1.0))
    guard_dark = model.material("guard_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.72, 0.75, 0.78, 1.0))

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.24, 0.17, 0.045), 0.028),
        "desk_fan_base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=base_dark,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.123), rpy=(0.0, 0.0, 0.0)),
        material=base_dark,
        name="pedestal_post",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=base_dark,
        name="pedestal_collar",
    )

    base.visual(
        Box((0.024, 0.016, 0.094)),
        origin=Origin(xyz=(-0.030, -0.114, 0.261)),
        material=base_dark,
        name="left_yoke_arm",
    )
    base.visual(
        Box((0.024, 0.016, 0.094)),
        origin=Origin(xyz=(-0.030, 0.114, 0.261)),
        material=base_dark,
        name="right_yoke_arm",
    )
    base.visual(
        Box((0.028, 0.092, 0.012)),
        origin=Origin(xyz=(-0.016, -0.068, 0.220)),
        material=base_dark,
        name="left_yoke_brace",
    )
    base.visual(
        Box((0.028, 0.092, 0.012)),
        origin=Origin(xyz=(-0.016, 0.068, 0.220)),
        material=base_dark,
        name="right_yoke_brace",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(-0.030, -0.122, 0.308), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=base_dark,
        name="left_yoke_cap",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(-0.030, 0.122, 0.308), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=base_dark,
        name="right_yoke_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.17, 0.34)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.078, length=0.10),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_light,
        name="rear_housing",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_light,
        name="front_neck",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_light,
        name="pivot_shaft",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, -0.088, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_light,
        name="left_pivot_boss",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.088, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_light,
        name="right_pivot_boss",
    )
    head.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.104,
                tube=0.0042,
                radial_segments=16,
                tubular_segments=48,
            ).rotate_y(math.pi / 2.0).translate(0.098, 0.0, 0.0),
            "desk_fan_front_guard_ring",
        ),
        material=guard_dark,
        name="front_guard_ring",
    )
    head.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.098,
                tube=0.0036,
                radial_segments=16,
                tubular_segments=48,
            ).rotate_y(math.pi / 2.0).translate(0.050, 0.0, 0.0),
            "desk_fan_back_guard_ring",
        ),
        material=guard_dark,
        name="back_guard_ring",
    )
    rear_spoke_angles = [math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0]
    for index, angle in enumerate(rear_spoke_angles):
        head.visual(
            Cylinder(radius=0.0028, length=0.074),
            origin=Origin(
                xyz=(0.050, 0.065 * math.cos(angle), 0.065 * math.sin(angle)),
                rpy=(angle - math.pi / 2.0, 0.0, 0.0),
            ),
            material=guard_dark,
            name=f"rear_guard_spoke_{index + 1}",
        )
        support_strut = tube_from_spline_points(
            [
                (0.004, 0.067 * math.cos(angle), 0.067 * math.sin(angle)),
                (0.028, 0.081 * math.cos(angle), 0.081 * math.sin(angle)),
                (0.050, 0.095 * math.cos(angle), 0.095 * math.sin(angle)),
            ],
            radius=0.0032,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        )
        head.visual(
            mesh_from_geometry(support_strut, f"desk_fan_guard_support_{index + 1}"),
            material=guard_dark,
            name=f"guard_support_strut_{index + 1}",
        )

    depth_strut_angles = [math.pi / 6.0, math.pi / 2.0, 5.0 * math.pi / 6.0, 7.0 * math.pi / 6.0, 3.0 * math.pi / 2.0, 11.0 * math.pi / 6.0]
    for index, angle in enumerate(depth_strut_angles):
        head.visual(
            Cylinder(radius=0.0035, length=0.056),
            origin=Origin(
                xyz=(0.074, 0.100 * math.cos(angle), 0.100 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=guard_dark,
            name=f"guard_depth_strut_{index + 1}",
        )

    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.18)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_dark,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_dark,
        name="rear_spindle",
    )
    rotor.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_dark,
        name="hub_cap",
    )

    def blade_section(y_pos: float, x_bias: float, z_scale: float) -> list[tuple[float, float, float]]:
        return [
            (-0.003 + x_bias, y_pos, -0.012 * z_scale),
            (0.010 + x_bias, y_pos, -0.003 * z_scale),
            (0.006 + x_bias, y_pos, 0.013 * z_scale),
            (-0.006 + x_bias, y_pos, 0.004 * z_scale),
        ]

    blade_base = section_loft(
        [
            blade_section(0.000, 0.000, 1.00),
            blade_section(0.036, 0.004, 1.18),
            blade_section(0.072, 0.008, 0.88),
        ]
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            mesh_from_geometry(blade_base.copy().rotate_x(angle), f"desk_fan_blade_{index + 1}"),
            origin=Origin(xyz=(0.004, 0.0, 0.0)),
            material=rotor_gray,
            name=f"blade_{index + 1}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.026),
        mass=0.18,
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-18.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")

    ctx.expect_gap(
        head,
        base,
        axis="y",
        positive_elem="left_pivot_boss",
        negative_elem="left_yoke_cap",
        min_gap=0.004,
        max_gap=0.020,
        name="left pivot boss clears left bracket",
    )
    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="right_yoke_cap",
        negative_elem="right_pivot_boss",
        min_gap=0.004,
        max_gap=0.020,
        name="right pivot boss clears right bracket",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        outer_elem="front_guard_ring",
        margin=0.0,
        name="rotor stays inside guard footprint",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="hub_body",
        negative_elem="front_neck",
        min_gap=0.006,
        name="rotor hub clears front neck",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_pt, max_pt) = aabb
        return tuple((min_pt[i] + max_pt[i]) * 0.5 for i in range(3))

    rest_center = aabb_center(ctx.part_element_world_aabb(head, elem="front_guard_ring"))
    with ctx.pose({tilt: math.radians(24.0)}):
        tilted_center = aabb_center(ctx.part_element_world_aabb(head, elem="front_guard_ring"))

    ctx.check(
        "head tilts upward",
        rest_center is not None
        and tilted_center is not None
        and tilted_center[2] > rest_center[2] + 0.025,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
