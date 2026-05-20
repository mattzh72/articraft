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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_arc_reading_lamp")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.67, 0.58, 0.39, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.91, 0.86, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.38)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )
    base.visual(
        Box((0.22, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder_black,
        name="base_plate",
    )
    base.visual(
        Box((0.16, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=graphite,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.29),
        origin=Origin(xyz=(0.0, 0.0, 0.163)),
        material=powder_black,
        name="post_column",
    )
    base.visual(
        Box((0.050, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.317)),
        material=graphite,
        name="post_cap",
    )
    base.visual(
        Box((0.024, 0.052, 0.012)),
        origin=Origin(xyz=(0.004, 0.0, 0.320)),
        material=graphite,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.030, 0.012, 0.052)),
        origin=Origin(xyz=(0.006, 0.032, 0.352)),
        material=graphite,
        name="yoke_left",
    )
    base.visual(
        Box((0.030, 0.012, 0.052)),
        origin=Origin(xyz=(0.006, -0.032, 0.352)),
        material=graphite,
        name="yoke_right",
    )

    boom_arm = model.part("boom_arm")
    boom_arm.inertial = Inertial.from_geometry(
        Box((0.68, 0.08, 0.28)),
        mass=0.85,
        origin=Origin(xyz=(0.34, 0.0, 0.13)),
    )
    boom_arm.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="root_hinge_barrel",
    )
    boom_arm.visual(
        Box((0.036, 0.046, 0.026)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=powder_black,
        name="root_mount",
    )
    arm_tube = tube_from_spline_points(
        [
            (0.018, 0.0, 0.000),
            (0.110, 0.0, 0.070),
            (0.285, 0.0, 0.195),
            (0.485, 0.0, 0.215),
            (0.600, 0.0, 0.125),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    boom_arm.visual(
        _mesh("boom_arm_tube", arm_tube),
        material=satin_brass,
        name="arm_tube",
    )
    end_bracket_origin = Origin(xyz=(0.616, 0.0, 0.109), rpy=(0.0, 0.45, 0.0))
    boom_arm.visual(
        Box((0.020, 0.052, 0.012)),
        origin=Origin(xyz=(0.592, 0.0, 0.121), rpy=(0.0, 0.45, 0.0)),
        material=powder_black,
        name="arm_end_mount",
    )
    boom_arm.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(0.609, 0.024, 0.113), rpy=(0.0, 0.45, 0.0)),
        material=powder_black,
        name="end_bracket_left",
    )
    boom_arm.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(0.609, -0.024, 0.113), rpy=(0.0, 0.45, 0.0)),
        material=powder_black,
        name="end_bracket_right",
    )

    shade = model.part("shade")
    shade.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.08)),
        mass=0.32,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="shade_hinge_barrel",
    )
    shade.visual(
        Box((0.026, 0.036, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=powder_black,
        name="shade_support",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="socket_housing",
    )
    shade_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.021, 0.000),
            (0.027, 0.016),
            (0.040, 0.068),
            (0.055, 0.118),
        ],
        inner_profile=[
            (0.015, 0.000),
            (0.020, 0.014),
            (0.032, 0.066),
            (0.047, 0.116),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)
    shade.visual(
        _mesh("shade_shell", shade_shell),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=powder_black,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_white,
        name="lamp_aperture_ring",
    )

    model.articulation(
        "base_to_boom",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.65,
            upper=0.95,
        ),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom_arm,
        child=shade,
        origin=end_bracket_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=-0.75,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    boom_arm = object_model.get_part("boom_arm")
    shade = object_model.get_part("shade")
    base_to_boom = object_model.get_articulation("base_to_boom")
    boom_to_shade = object_model.get_articulation("boom_to_shade")

    ctx.expect_contact(
        boom_arm,
        base,
        elem_a="root_hinge_barrel",
        elem_b="yoke_left",
        contact_tol=2e-4,
        name="boom hinge barrel seats against left yoke cheek",
    )
    ctx.expect_contact(
        shade,
        boom_arm,
        elem_a="shade_hinge_barrel",
        elem_b="end_bracket_left",
        contact_tol=2e-4,
        name="shade hinge barrel seats against left end bracket cheek",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="x",
        min_gap=0.50,
        name="shade reaches well forward of the weighted base",
    )

    rest_end = _aabb_center_z(ctx.part_element_world_aabb(boom_arm, elem="arm_end_mount"))
    with ctx.pose({base_to_boom: 0.55}):
        raised_end = _aabb_center_z(ctx.part_element_world_aabb(boom_arm, elem="arm_end_mount"))
    ctx.check(
        "boom arm raises upward on positive hinge motion",
        rest_end is not None and raised_end is not None and raised_end > rest_end + 0.12,
        details=f"rest_end_z={rest_end}, raised_end_z={raised_end}",
    )

    with ctx.pose({base_to_boom: 0.35, boom_to_shade: -0.30}):
        shade_up = _aabb_center_z(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({base_to_boom: 0.35, boom_to_shade: 0.40}):
        shade_down = _aabb_center_z(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "positive shade tilt angles aim the shade downward",
        shade_up is not None and shade_down is not None and shade_down < shade_up - 0.015,
        details=f"shade_up_z={shade_up}, shade_down_z={shade_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
