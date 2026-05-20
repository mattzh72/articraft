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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _build_column_tube():
    return tube_from_spline_points(
        [
            (0.000, 0.000, 0.028),
            (0.000, 0.000, 0.260),
            (0.018, 0.000, 0.600),
            (0.094, 0.000, 0.975),
            (0.222, 0.000, 1.245),
            (0.310, 0.000, 1.355),
        ],
        radius=0.022,
        samples_per_segment=18,
        radial_segments=22,
    )


def _build_shade_shell():
    outer_profile = [
        (0.018, -0.010),
        (0.040, 0.000),
        (0.076, 0.028),
        (0.110, 0.072),
        (0.122, 0.106),
    ]
    inner_profile = [
        (0.009, -0.004),
        (0.031, 0.004),
        (0.066, 0.028),
        (0.098, 0.070),
        (0.111, 0.104),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _build_counter_arm():
    return tube_from_spline_points(
        [
            (-0.044, -0.025, 0.000),
            (-0.105, -0.025, 0.016),
            (-0.190, -0.025, 0.048),
            (-0.252, -0.025, 0.074),
            (-0.286, -0.025, 0.086),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    enamel_cream = model.material("enamel_cream", rgba=(0.86, 0.84, 0.78, 1.0))
    warm_light = model.material("warm_light", rgba=(0.98, 0.88, 0.62, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.190, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="weighted_disc",
    )
    base.visual(
        Cylinder(radius=0.148, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_black,
        name="top_cap",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=brushed_steel,
        name="column_socket",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.066),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brushed_steel,
        name="lower_sleeve",
    )
    column.visual(
        _save_mesh(_build_column_tube(), "pharmacy_column_tube"),
        material=brushed_steel,
        name="curved_column",
    )
    column.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(
            xyz=(0.310, 0.000, 1.355),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="tip_spindle",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.400, 0.080, 1.420)),
        mass=5.5,
        origin=Origin(xyz=(0.160, 0.0, 0.710)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.025, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="shade_knuckle",
    )
    shade.visual(
        Box((0.056, 0.014, 0.022)),
        origin=Origin(xyz=(0.031, 0.025, 0.000)),
        material=satin_black,
        name="shade_stem",
    )
    shade.visual(
        _save_mesh(_build_shade_shell(), "pharmacy_shade_shell"),
        origin=Origin(
            xyz=(0.072, 0.025, 0.004),
            rpy=(0.0, math.pi / 2.0 + math.radians(16.0), 0.0),
        ),
        material=enamel_cream,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(
            xyz=(0.082, 0.025, -0.004),
            rpy=(0.0, math.pi / 2.0 + math.radians(16.0), 0.0),
        ),
        material=satin_black,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.110, 0.025, -0.010)),
        material=warm_light,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.220, 0.160, 0.180)),
        mass=1.2,
        origin=Origin(xyz=(0.095, 0.025, -0.005)),
    )

    counter_arm = model.part("counter_arm")
    counter_arm.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.025, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="arm_knuckle",
    )
    counter_arm.visual(
        Box((0.050, 0.014, 0.018)),
        origin=Origin(xyz=(-0.028, -0.025, 0.000)),
        material=satin_black,
        name="arm_stem",
    )
    counter_arm.visual(
        _save_mesh(_build_counter_arm(), "pharmacy_counter_arm"),
        material=brushed_steel,
        name="arm_tube",
    )
    counter_arm.visual(
        Cylinder(radius=0.032, length=0.090),
        origin=Origin(
            xyz=(-0.325, -0.025, 0.086),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="counterweight",
    )
    counter_arm.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(
            xyz=(-0.376, -0.025, 0.086),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="counterweight_cap",
    )
    counter_arm.inertial = Inertial.from_geometry(
        Box((0.400, 0.090, 0.170)),
        mass=2.5,
        origin=Origin(xyz=(-0.210, -0.025, 0.050)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
    )
    model.articulation(
        "column_to_shade",
        ArticulationType.REVOLUTE,
        parent=column,
        child=shade,
        origin=Origin(xyz=(0.310, 0.0, 1.355)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.70,
        ),
    )
    model.articulation(
        "column_to_counter_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=counter_arm,
        origin=Origin(xyz=(0.310, 0.0, 1.355)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-0.25,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    shade = object_model.get_part("shade")
    counter_arm = object_model.get_part("counter_arm")
    shade_tilt = object_model.get_articulation("column_to_shade")
    arm_tilt = object_model.get_articulation("column_to_counter_arm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    assert tuple(shade_tilt.axis) == (0.0, 1.0, 0.0)
    assert tuple(arm_tilt.axis) == (0.0, 1.0, 0.0)
    assert shade_tilt.motion_limits is not None
    assert arm_tilt.motion_limits is not None
    assert shade_tilt.motion_limits.lower is not None and shade_tilt.motion_limits.upper is not None
    assert arm_tilt.motion_limits.lower is not None and arm_tilt.motion_limits.upper is not None
    assert shade_tilt.motion_limits.lower < 0.0 < shade_tilt.motion_limits.upper
    assert arm_tilt.motion_limits.lower < 0.0 < arm_tilt.motion_limits.upper

    ctx.expect_contact(column, base)
    ctx.expect_contact(shade, column)
    ctx.expect_contact(counter_arm, column)
    ctx.expect_gap(shade, base, axis="z", min_gap=1.15, positive_elem="shade_shell")
    ctx.expect_gap(
        shade,
        counter_arm,
        axis="x",
        min_gap=0.22,
        positive_elem="shade_shell",
        negative_elem="counterweight",
    )

    shade_rest = ctx.part_element_world_aabb(shade, elem="shade_shell")
    counter_rest = ctx.part_element_world_aabb(counter_arm, elem="counterweight")
    assert shade_rest is not None
    assert counter_rest is not None
    assert shade_rest[1][0] > 0.46
    assert counter_rest[0][0] < -0.03

    with ctx.pose({shade_tilt: 0.45}):
        shade_low = ctx.part_element_world_aabb(shade, elem="shade_shell")
        assert shade_low is not None
        assert shade_low[0][2] < shade_rest[0][2] - 0.05
        ctx.expect_contact(shade, column)

    with ctx.pose({arm_tilt: 0.35}):
        counter_high = ctx.part_element_world_aabb(counter_arm, elem="counterweight")
        assert counter_high is not None
        assert counter_high[1][2] > counter_rest[1][2] + 0.04
        ctx.expect_contact(counter_arm, column)

    with ctx.pose({shade_tilt: 0.45, arm_tilt: 0.35}):
        ctx.expect_gap(
            shade,
            counter_arm,
            axis="x",
            min_gap=0.18,
            positive_elem="shade_shell",
            negative_elem="counterweight",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
