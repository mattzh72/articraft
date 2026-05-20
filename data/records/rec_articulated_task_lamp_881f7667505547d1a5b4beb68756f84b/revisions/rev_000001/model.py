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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_bankers_desk_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.14, 0.13, 0.12, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.74, 0.61, 0.28, 1.0))
    green_glass = model.material("green_glass", rgba=(0.13, 0.42, 0.24, 0.68))

    base_assembly = model.part("base_assembly")
    base_profile = [
        (0.0, 0.000),
        (0.092, 0.000),
        (0.102, 0.004),
        (0.102, 0.008),
        (0.096, 0.015),
        (0.089, 0.022),
        (0.036, 0.028),
        (0.0, 0.028),
    ]
    base_assembly.visual(
        _mesh("bankers_lamp_base_shell", LatheGeometry(base_profile, segments=72)),
        material=cast_iron,
        name="base_shell",
    )
    base_assembly.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=aged_brass,
        name="base_collar",
    )
    base_assembly.visual(
        Cylinder(radius=0.011, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.1105)),
        material=aged_brass,
        name="main_post",
    )
    base_assembly.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=aged_brass,
        name="post_cap",
    )
    base_assembly.visual(
        Box((0.018, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=aged_brass,
        name="post_head",
    )
    base_assembly.visual(
        Box((0.010, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.013, 0.203)),
        material=aged_brass,
        name="post_fork_left",
    )
    base_assembly.visual(
        Box((0.010, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.013, 0.203)),
        material=aged_brass,
        name="post_fork_right",
    )
    base_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.102, length=0.214),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="shoulder_barrel",
    )
    arm.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.021, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="shoulder_hub",
    )
    arm.visual(
        _mesh(
            "bankers_lamp_arm_tube",
            tube_from_spline_points(
                [
                    (-0.004, 0.0, 0.000),
                    (0.050, 0.0, 0.010),
                    (0.142, 0.0, 0.028),
                    (0.194, 0.0, 0.018),
                ],
                radius=0.0075,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=aged_brass,
        name="arm_tube",
    )
    arm.visual(
        Box((0.014, 0.022, 0.018)),
        origin=Origin(xyz=(0.197, 0.0, 0.018)),
        material=aged_brass,
        name="shade_knuckle",
    )
    arm.visual(
        Box((0.010, 0.004, 0.024)),
        origin=Origin(xyz=(0.209, 0.012, 0.018)),
        material=aged_brass,
        name="shade_fork_left",
    )
    arm.visual(
        Box((0.010, 0.004, 0.024)),
        origin=Origin(xyz=(0.209, -0.012, 0.018)),
        material=aged_brass,
        name="shade_fork_right",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.235, 0.032, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.112, 0.0, 0.016)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="pivot_barrel",
    )
    shade.visual(
        _mesh(
            "bankers_lamp_shade_neck",
            tube_from_spline_points(
                [
                    (0.000, 0.0, 0.000),
                    (0.014, 0.0, -0.014),
                    (0.024, 0.0, -0.030),
                ],
                radius=0.0055,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=aged_brass,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.032, 0.0, -0.030)),
        material=aged_brass,
        name="shade_cap",
    )
    shade_glass_outer = [
        (0.028, 0.000),
        (0.044, -0.006),
        (0.062, -0.018),
        (0.078, -0.036),
        (0.086, -0.056),
        (0.088, -0.072),
        (0.080, -0.086),
    ]
    shade_glass_inner = [
        (0.022, -0.001),
        (0.038, -0.007),
        (0.056, -0.019),
        (0.072, -0.036),
        (0.080, -0.056),
        (0.082, -0.071),
        (0.074, -0.082),
    ]
    shade.visual(
        _mesh(
            "bankers_lamp_shade_glass",
            LatheGeometry.from_shell_profiles(
                shade_glass_outer,
                shade_glass_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
        ),
        origin=Origin(xyz=(0.040, 0.0, -0.035)),
        material=green_glass,
        name="shade_glass",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.110),
        mass=0.55,
        origin=Origin(xyz=(0.040, 0.0, -0.060)),
    )

    model.articulation(
        "post_to_arm",
        ArticulationType.REVOLUTE,
        parent=base_assembly,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.40,
            upper=0.65,
        ),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.215, 0.0, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_assembly = object_model.get_part("base_assembly")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    arm_joint = object_model.get_articulation("post_to_arm")
    shade_joint = object_model.get_articulation("arm_to_shade")

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.expect_origin_gap(
        arm,
        base_assembly,
        axis="z",
        min_gap=0.17,
        max_gap=0.23,
        name="arm pivot sits on top of the post",
    )

    with ctx.pose({arm_joint: 0.0, shade_joint: 0.0}):
        arm_pos = ctx.part_world_position(arm)
        shade_pos = ctx.part_world_position(shade)
        ctx.check(
            "shade hangs forward of the post",
            arm_pos is not None
            and shade_pos is not None
            and shade_pos[0] > arm_pos[0] + 0.18
            and shade_pos[2] > 0.18,
            details=f"arm_pos={arm_pos}, shade_pos={shade_pos}",
        )

    with ctx.pose({arm_joint: 0.0, shade_joint: 0.0}):
        rest_pos = ctx.part_world_position(shade)
    with ctx.pose({arm_joint: math.radians(32.0), shade_joint: 0.0}):
        raised_pos = ctx.part_world_position(shade)
    ctx.check(
        "arm joint raises the lamp head",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.05,
        details=f"rest_pos={rest_pos}, raised_pos={raised_pos}",
    )

    with ctx.pose({arm_joint: 0.12, shade_joint: -0.35}):
        tilt_up_center = elem_center(shade, "shade_glass")
    with ctx.pose({arm_joint: 0.12, shade_joint: 0.45}):
        tilt_down_center = elem_center(shade, "shade_glass")
    ctx.check(
        "shade hinge tilts the glass downward",
        tilt_up_center is not None
        and tilt_down_center is not None
        and tilt_down_center[2] < tilt_up_center[2] - 0.015
        and tilt_down_center[0] < tilt_up_center[0] - 0.02,
        details=f"tilt_up_center={tilt_up_center}, tilt_down_center={tilt_down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
