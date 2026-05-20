from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailing_arm_torsion_beam_axle")

    dark_steel = model.material("dark_blasted_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    black_coating = model.material("black_epoxy_coating", rgba=(0.01, 0.012, 0.014, 1.0))
    rubber = model.material("black_rubber_bushing", rgba=(0.005, 0.005, 0.004, 1.0))
    bare_steel = model.material("machined_bearing_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    rotor_iron = model.material("brake_rotor_cast_iron", rgba=(0.34, 0.34, 0.32, 1.0))
    weld_heat = model.material("weld_heat_tint", rgba=(0.18, 0.12, 0.07, 1.0))

    beam = model.part("torsion_beam")
    beam.visual(
        Cylinder(radius=0.045, length=1.32),
        origin=Origin(xyz=(0.0, 0.0, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_tube",
    )
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        y = side * 0.735
        beam.visual(
            Box((0.18, 0.018, 0.18)),
            origin=Origin(xyz=(0.0, y - side * 0.075, 0.55)),
            material=dark_steel,
            name=f"{side_name}_clevis_inner_plate",
        )
        beam.visual(
            Box((0.18, 0.018, 0.18)),
            origin=Origin(xyz=(0.0, y + side * 0.075, 0.55)),
            material=dark_steel,
            name=f"{side_name}_clevis_outer_plate",
        )
        beam.visual(
            Box((0.18, 0.168, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.645)),
            material=dark_steel,
            name="left_clevis_top_bridge" if side_name == "left" else "right_clevis_top_bridge",
        )
        beam.visual(
            Box((0.12, 0.168, 0.020)),
            origin=Origin(xyz=(0.025, y, 0.455)),
            material=dark_steel,
            name=f"{side_name}_clevis_bottom_tie",
        )
        beam.visual(
            Cylinder(radius=0.058, length=0.018),
            origin=Origin(xyz=(0.0, y - side * 0.075, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"{side_name}_pivot_washer_inner",
        )
        beam.visual(
            Cylinder(radius=0.058, length=0.018),
            origin=Origin(xyz=(0.0, y + side * 0.075, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"{side_name}_pivot_washer_outer",
        )

    arm_centerline = [
        (0.015, 0.0, -0.010),
        (-0.120, 0.0, -0.032),
        (-0.520, 0.0, -0.052),
        (-0.900, 0.0, -0.048),
        (-1.020, 0.0, -0.030),
    ]

    def add_trailing_arm(side_name: str, side: float):
        arm = model.part(f"{side_name}_arm")
        arm_tube = tube_from_spline_points(
            arm_centerline,
            radius=0.037,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        )
        arm.visual(
            mesh_from_geometry(arm_tube, f"{side_name}_pressed_trailing_arm"),
            material=black_coating,
            name="pressed_arm",
        )
        arm.visual(
            Cylinder(radius=0.052, length=0.120),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="pivot_bushing",
        )
        arm.visual(
            Cylinder(radius=0.066, length=0.035),
            origin=Origin(xyz=(0.0, -side * 0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="inner_bushing_sleeve",
        )
        arm.visual(
            Cylinder(radius=0.066, length=0.035),
            origin=Origin(xyz=(0.0, side * 0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="outer_bushing_sleeve",
        )
        arm.visual(
            Box((0.21, 0.110, 0.040)),
            origin=Origin(xyz=(-0.105, 0.0, -0.034)),
            material=black_coating,
            name="pivot_gusset",
        )
        arm.visual(
            Box((0.115, 0.080, 0.120)),
            origin=Origin(xyz=(-1.020, 0.0, -0.030)),
            material=dark_steel,
            name="hub_carrier",
        )
        arm.visual(
            Cylinder(radius=0.060, length=0.025),
            origin=Origin(xyz=(-1.020, side * 0.017, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="bearing_boss",
        )
        arm.visual(
            Box((0.070, 0.150, 0.030)),
            origin=Origin(xyz=(-0.985, 0.0, 0.038)),
            material=weld_heat,
            name="welded_hub_seam",
        )
        return arm

    def add_hub(side_name: str, side: float):
        hub = model.part(f"{side_name}_hub")
        hub.visual(
            Cylinder(radius=0.168, length=0.018),
            origin=Origin(xyz=(0.0, side * 0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rotor_iron,
            name="rotor",
        )
        hub.visual(
            Cylinder(radius=0.052, length=0.040),
            origin=Origin(xyz=(0.0, side * 0.061, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="bearing_snout",
        )
        hub.visual(
            Cylinder(radius=0.090, length=0.055),
            origin=Origin(xyz=(0.0, side * 0.110, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="hub_flange",
        )
        hub.visual(
            Cylinder(radius=0.047, length=0.066),
            origin=Origin(xyz=(0.0, side * 0.150, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="center_cap",
        )
        hub.visual(
            Cylinder(radius=0.074, length=0.012),
            origin=Origin(xyz=(0.0, side * 0.144, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name="bolt_circle_pad",
        )
        return hub

    left_arm = add_trailing_arm("left", 1.0)
    right_arm = add_trailing_arm("right", -1.0)
    left_hub = add_hub("left", 1.0)
    right_hub = add_hub("right", -1.0)

    model.articulation(
        "left_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_arm,
        origin=Origin(xyz=(0.0, 0.735, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=1.5, lower=-0.32, upper=0.38),
    )
    model.articulation(
        "right_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_arm,
        origin=Origin(xyz=(0.0, -0.735, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=1.5, lower=-0.32, upper=0.38),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_hub,
        origin=Origin(xyz=(-1.020, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=50.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_hub,
        origin=Origin(xyz=(-1.020, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=50.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("torsion_beam")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_pivot = object_model.get_articulation("left_arm_pivot")
    right_pivot = object_model.get_articulation("right_arm_pivot")
    left_spin = object_model.get_articulation("left_hub_spin")
    right_spin = object_model.get_articulation("right_hub_spin")

    def axis_is_transverse(joint) -> bool:
        return tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)

    for joint in (left_pivot, right_pivot):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a limited transverse revolute pivot",
            joint.articulation_type == ArticulationType.REVOLUTE
            and axis_is_transverse(joint)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    for joint in (left_spin, right_spin):
        ctx.check(
            f"{joint.name} is a continuous wheel-hub rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS and axis_is_transverse(joint),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_within(
        left_arm,
        beam,
        axes="y",
        inner_elem="pivot_bushing",
        outer_elem="left_clevis_top_bridge",
        margin=0.001,
        name="left pivot bushing is retained between clevis plates",
    )
    ctx.expect_within(
        right_arm,
        beam,
        axes="y",
        inner_elem="pivot_bushing",
        outer_elem="right_clevis_top_bridge",
        margin=0.001,
        name="right pivot bushing is retained between clevis plates",
    )

    ctx.expect_gap(
        left_hub,
        left_arm,
        axis="y",
        positive_elem="bearing_snout",
        negative_elem="bearing_boss",
        min_gap=0.002,
        max_gap=0.018,
        name="left rotating hub snout sits just outboard of arm bearing",
    )
    ctx.expect_overlap(
        left_hub,
        left_arm,
        axes="xz",
        elem_a="bearing_snout",
        elem_b="bearing_boss",
        min_overlap=0.080,
        name="left hub bearing aligns with carrier bore",
    )
    ctx.expect_gap(
        right_arm,
        right_hub,
        axis="y",
        positive_elem="bearing_boss",
        negative_elem="bearing_snout",
        min_gap=0.002,
        max_gap=0.018,
        name="right rotating hub snout sits just outboard of arm bearing",
    )
    ctx.expect_overlap(
        right_hub,
        right_arm,
        axes="xz",
        elem_a="bearing_snout",
        elem_b="bearing_boss",
        min_overlap=0.080,
        name="right hub bearing aligns with carrier bore",
    )

    rest_left = ctx.part_world_position(left_hub)
    rest_right = ctx.part_world_position(right_hub)
    with ctx.pose({left_pivot: 0.38, right_pivot: 0.38}):
        raised_left = ctx.part_world_position(left_hub)
        raised_right = ctx.part_world_position(right_hub)
    with ctx.pose({left_pivot: -0.32, right_pivot: -0.32}):
        drooped_left = ctx.part_world_position(left_hub)
        drooped_right = ctx.part_world_position(right_hub)

    ctx.check(
        "positive trailing-arm pivot raises both hub centers",
        rest_left is not None
        and rest_right is not None
        and raised_left is not None
        and raised_right is not None
        and raised_left[2] > rest_left[2] + 0.20
        and raised_right[2] > rest_right[2] + 0.20,
        details=f"rest_left={rest_left}, raised_left={raised_left}, rest_right={rest_right}, raised_right={raised_right}",
    )
    ctx.check(
        "negative trailing-arm pivot droops both hub centers",
        rest_left is not None
        and rest_right is not None
        and drooped_left is not None
        and drooped_right is not None
        and drooped_left[2] < rest_left[2] - 0.20
        and drooped_right[2] < rest_right[2] - 0.20,
        details=f"rest_left={rest_left}, drooped_left={drooped_left}, rest_right={rest_right}, drooped_right={drooped_right}",
    )

    return ctx.report()


object_model = build_object_model()
