from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_serial_elbow_arm")

    cast = model.material("cast_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    blue = model.material("blue_anodized_link", rgba=(0.08, 0.20, 0.36, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = model.material("dark_ribbed_metal", rgba=(0.07, 0.08, 0.09, 1.0))
    rubber = model.material("matte_black_pad", rgba=(0.015, 0.015, 0.014, 1.0))

    # The root is a broad bolted, grounded shoulder yoke.  The two side
    # bearing cheeks leave a clear central slot for the rotating upper-link
    # collar, rather than hiding the revolute joint inside an overlapping solid.
    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Box((0.58, 0.60, 0.06)),
        origin=Origin(xyz=(0.02, 0.0, 0.03)),
        material=cast,
        name="base_plate",
    )
    shoulder_housing.visual(
        Box((0.25, 0.42, 0.29)),
        origin=Origin(xyz=(-0.04, 0.0, 0.195)),
        material=cast,
        name="cast_pedestal",
    )
    for y, suffix in ((0.205, "pos"), (-0.205, "neg")):
        shoulder_housing.visual(
            Box((0.16, 0.09, 0.26)),
            origin=Origin(xyz=(-0.02, y, 0.385)),
            material=cast,
            name=f"shoulder_cheek_{suffix}",
        )
        shoulder_housing.visual(
            Cylinder(radius=0.145, length=0.10),
            origin=Origin(xyz=(0.0, y, 0.46), rpy=(-pi / 2, 0.0, 0.0)),
            material=cast,
            name=f"shoulder_boss_{suffix}",
        )
        shoulder_housing.visual(
            Cylinder(radius=0.100, length=0.035),
            origin=Origin(
                xyz=(0.0, y + (0.067 if y > 0.0 else -0.067), 0.46),
                rpy=(-pi / 2, 0.0, 0.0),
            ),
            material=steel,
            name=f"shoulder_cap_{suffix}",
        )
    for x in (-0.20, 0.24):
        for y in (-0.23, 0.23):
            shoulder_housing.visual(
                Cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(x, y, 0.063)),
                material=steel,
                name=f"base_bolt_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    # Upper link frame: origin at the shoulder axis.  It is long and deep,
    # with an I-beam style ribbed cross-section and a much broader elbow yoke.
    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.105, length=0.270),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="shoulder_collar",
    )
    for y, suffix in ((0.123, "pos"), (-0.123, "neg")):
        upper_link.visual(
            Cylinder(radius=0.123, length=0.025),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"shoulder_face_{suffix}",
        )
    upper_link.visual(
        Box((0.50, 0.10, 0.13)),
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        material=blue,
        name="upper_beam",
    )
    for z, suffix in ((0.075, "top"), (-0.075, "bottom")):
        upper_link.visual(
            Box((0.45, 0.13, 0.030)),
            origin=Origin(xyz=(0.36, 0.0, z)),
            material=blue,
            name=f"upper_rib_{suffix}",
        )
    for y, suffix in ((0.066, "pos"), (-0.066, "neg")):
        upper_link.visual(
            Box((0.46, 0.040, 0.16)),
            origin=Origin(xyz=(0.35, y, 0.0)),
            material=dark,
            name=f"upper_side_rib_{suffix}",
        )
        upper_link.visual(
            Box((0.22, 0.060, 0.12)),
            origin=Origin(xyz=(0.64, y + (0.048 if y > 0.0 else -0.048), 0.0)),
            material=blue,
            name=f"elbow_cheek_{suffix}",
        )
        upper_link.visual(
            Cylinder(radius=0.145, length=0.080),
            origin=Origin(
                xyz=(0.75, y + (0.095 if y > 0.0 else -0.095), 0.0),
                rpy=(-pi / 2, 0.0, 0.0),
            ),
            material=blue,
            name=f"elbow_boss_{suffix}",
        )
        upper_link.visual(
            Cylinder(radius=0.105, length=0.030),
            origin=Origin(
                xyz=(0.75, y + (0.150 if y > 0.0 else -0.150), 0.0),
                rpy=(-pi / 2, 0.0, 0.0),
            ),
            material=steel,
            name=f"elbow_cap_{suffix}",
        )

    # Shorter forearm: wider and flatter than the upper link, with a large
    # rectangular end pad.  Its frame is the elbow trunnion center.
    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.100, length=0.168),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="elbow_trunnion",
    )
    forearm.visual(
        Box((0.34, 0.16, 0.070)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=cast,
        name="forearm_spine",
    )
    for y, suffix in ((0.083, "pos"), (-0.083, "neg")):
        forearm.visual(
            Box((0.28, 0.035, 0.11)),
            origin=Origin(xyz=(0.25, y, 0.0)),
            material=dark,
            name=f"forearm_side_rib_{suffix}",
        )
    forearm.visual(
        Box((0.12, 0.24, 0.10)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=cast,
        name="pad_plate",
    )
    forearm.visual(
        Box((0.016, 0.265, 0.120)),
        origin=Origin(xyz=(0.526, 0.0, 0.0)),
        material=rubber,
        name="rubber_face",
    )

    model.articulation(
        "shoulder_revolute",
        ArticulationType.REVOLUTE,
        parent=shoulder_housing,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=1.4, lower=0.0, upper=1.05),
    )
    model.articulation(
        "elbow_revolute",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        # A built-in pitch makes the rest pose visibly bent downward while the
        # axis remains parallel to the shoulder's horizontal bearing axis.
        origin=Origin(xyz=(0.75, 0.0, 0.0), rpy=(0.0, 0.60, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.7, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_part("shoulder_housing")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder_joint = object_model.get_articulation("shoulder_revolute")
    elbow_joint = object_model.get_articulation("elbow_revolute")

    ctx.check(
        "parallel horizontal revolute axes",
        shoulder_joint.axis == (0.0, -1.0, 0.0)
        and elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder_joint.axis}, elbow_axis={elbow_joint.axis}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0}):
        ctx.expect_gap(
            shoulder,
            upper,
            axis="y",
            positive_elem="shoulder_boss_pos",
            negative_elem="shoulder_face_pos",
            min_gap=0.003,
            name="positive shoulder yoke clearance",
        )
        ctx.expect_gap(
            upper,
            shoulder,
            axis="y",
            positive_elem="shoulder_face_neg",
            negative_elem="shoulder_boss_neg",
            min_gap=0.003,
            name="negative shoulder yoke clearance",
        )
        ctx.expect_gap(
            upper,
            forearm,
            axis="y",
            positive_elem="elbow_boss_pos",
            negative_elem="elbow_trunnion",
            min_gap=0.020,
            name="positive elbow fork clearance",
        )
        ctx.expect_gap(
            forearm,
            upper,
            axis="y",
            positive_elem="elbow_trunnion",
            negative_elem="elbow_boss_neg",
            min_gap=0.020,
            name="negative elbow fork clearance",
        )
        ctx.expect_gap(
            forearm,
            shoulder,
            axis="z",
            positive_elem="pad_plate",
            negative_elem="base_plate",
            min_gap=0.050,
            name="rest pad clears grounded base",
        )

    shoulder_rest = ctx.part_element_world_aabb(upper, elem="elbow_boss_pos")
    with ctx.pose({shoulder_joint: 1.05, elbow_joint: 0.0}):
        shoulder_raised = ctx.part_element_world_aabb(upper, elem="elbow_boss_pos")
        ctx.expect_gap(
            upper,
            shoulder,
            axis="z",
            positive_elem="upper_beam",
            negative_elem="cast_pedestal",
            min_gap=0.030,
            name="raised shoulder link clears pedestal",
        )
    ctx.check(
        "shoulder motion lifts elbow hub",
        shoulder_rest is not None
        and shoulder_raised is not None
        and shoulder_raised[1][2] > shoulder_rest[1][2] + 0.18,
        details=f"rest={shoulder_rest}, raised={shoulder_raised}",
    )

    pad_rest = ctx.part_element_world_aabb(forearm, elem="pad_plate")
    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.65}):
        pad_straight = ctx.part_element_world_aabb(forearm, elem="pad_plate")
        ctx.expect_gap(
            forearm,
            upper,
            axis="y",
            positive_elem="elbow_trunnion",
            negative_elem="elbow_boss_neg",
            min_gap=0.020,
            name="straightened elbow keeps fork clearance",
        )
    ctx.check(
        "elbow motion raises end pad",
        pad_rest is not None
        and pad_straight is not None
        and pad_straight[0][2] > pad_rest[0][2] + 0.10,
        details=f"rest={pad_rest}, straightened={pad_straight}",
    )

    return ctx.report()


object_model = build_object_model()
