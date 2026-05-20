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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    dark_body = model.material("dark_body", color=(0.20, 0.22, 0.24))
    charcoal = model.material("charcoal", color=(0.13, 0.14, 0.15))
    blade_grey = model.material("blade_grey", color=(0.74, 0.76, 0.79))
    steel = model.material("steel", color=(0.62, 0.66, 0.70))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.12, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=charcoal,
        name="foot",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=dark_body,
        name="stem",
    )
    base.visual(
        Box((0.045, 0.045, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=dark_body,
        name="stem_cap",
    )
    base.visual(
        Box((0.050, 0.182, 0.024)),
        origin=Origin(xyz=(-0.030, 0.0, 0.180)),
        material=dark_body,
        name="yoke_crossbar",
    )
    base.visual(
        Box((0.126, 0.018, 0.024)),
        origin=Origin(xyz=(0.033, 0.081, 0.180)),
        material=dark_body,
        name="left_bracket",
    )
    base.visual(
        Box((0.126, 0.018, 0.024)),
        origin=Origin(xyz=(0.033, -0.081, 0.180)),
        material=dark_body,
        name="right_bracket",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(
            xyz=(0.096, 0.081, 0.180),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_body,
        name="left_pivot_cap",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(
            xyz=(0.096, -0.081, 0.180),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_body,
        name="right_pivot_cap",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.060, length=0.068),
        origin=Origin(
            xyz=(-0.038, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_body,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(
            xyz=(-0.012, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="motor_face",
    )
    guard_frame_geom = TorusGeometry(radius=0.108, tube=0.0075).rotate_y(math.pi / 2.0)
    guard_frame_geom.translate(0.046, 0.0, 0.0)
    for name, points in (
        (
            "guard_support_upper_left",
            [(-0.004, 0.030, 0.030), (0.016, 0.050, 0.050), (0.046, 0.074, 0.074)],
        ),
        (
            "guard_support_upper_right",
            [(-0.004, -0.030, 0.030), (0.016, -0.050, 0.050), (0.046, -0.074, 0.074)],
        ),
        (
            "guard_support_lower_right",
            [(-0.004, -0.030, -0.030), (0.016, -0.050, -0.050), (0.046, -0.074, -0.074)],
        ),
        (
            "guard_support_lower_left",
            [(-0.004, 0.030, -0.030), (0.016, 0.050, -0.050), (0.046, 0.074, -0.074)],
        ),
    ):
        guard_frame_geom.merge(
            tube_from_spline_points(
                points,
                radius=0.004,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )
    head.visual(
        mesh_from_geometry(guard_frame_geom, "guard_frame"),
        material=steel,
        name="guard_frame",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(
            xyz=(-0.004, 0.060, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(
            xyz=(-0.004, -0.060, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(
            xyz=(0.010, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hub",
    )
    for index, roll in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            Box((0.004, 0.064, 0.028)),
            origin=Origin(
                xyz=(0.018, 0.039, 0.0),
                rpy=(roll, 0.26, 0.0),
            ),
            material=blade_grey,
            name=f"blade_{index}",
        )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.096, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.45,
        ),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    head_tilt = object_model.get_articulation("head_tilt")

    def elem_center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.expect_contact(
        base,
        head,
        elem_a="left_pivot_cap",
        elem_b="left_trunnion",
        name="left trunnion bears on left pivot cap",
    )
    ctx.expect_contact(
        base,
        head,
        elem_a="right_pivot_cap",
        elem_b="right_trunnion",
        name="right trunnion bears on right pivot cap",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        outer_elem="guard_frame",
        name="rotor stays inside the guard ring silhouette",
    )

    rest_guard_z = elem_center_z("head", "guard_frame")
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        up_guard_z = elem_center_z("head", "guard_frame")
    with ctx.pose({head_tilt: head_tilt.motion_limits.lower}):
        down_guard_z = elem_center_z("head", "guard_frame")

    ctx.check(
        "positive head tilt raises the front guard",
        rest_guard_z is not None
        and up_guard_z is not None
        and up_guard_z > rest_guard_z + 0.008,
        details=f"rest_guard_z={rest_guard_z}, up_guard_z={up_guard_z}",
    )
    ctx.check(
        "negative head tilt lowers the front guard",
        rest_guard_z is not None
        and down_guard_z is not None
        and down_guard_z < rest_guard_z - 0.008,
        details=f"rest_guard_z={rest_guard_z}, down_guard_z={down_guard_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
