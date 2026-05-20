from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 36):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy - radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _translate_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _capsule_profile(length: float, radius: float, segments: int = 24):
    points = []
    for i in range(segments + 1):
        angle = -0.5 * math.pi + math.pi * i / segments
        points.append((length + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = 0.5 * math.pi + math.pi * i / segments
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collapsible_service_arm")

    graphite = model.material("graphite_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    dark = model.material("black_polymer", rgba=(0.01, 0.012, 0.014, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.63, 0.64, 0.61, 1.0))
    amber = model.material("amber_service_plate", rgba=(0.94, 0.58, 0.14, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.013, 1.0))

    arm0_len = 0.34
    arm1_len = 0.28
    arm0_width = 0.070
    arm1_width = 0.064
    plate_thickness = 0.010
    hinge_hole = 0.015
    pin_radius = 0.009
    shoulder_z = 0.025
    level_step = 0.022

    base = model.part("base")
    base_outer = _translate_profile(rounded_rect_profile(0.180, 0.120, 0.014), dx=-0.055)
    base_holes = [
        _circle_profile(x, y, 0.0065, 28)
        for x in (-0.105, -0.035)
        for y in (-0.040, 0.040)
    ]
    base.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(base_outer, base_holes, 0.012, center=True),
            "base_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="base_plate",
    )
    bolt_positions = [(-0.105, -0.040), (-0.105, 0.040), (-0.035, -0.040), (-0.035, 0.040)]
    for index, (x, y) in enumerate(bolt_positions):
        base.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, y, 0.014)),
            material=brushed,
            name=f"bolt_head_{index}",
        )
    base.visual(
        Cylinder(radius=0.020, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=brushed,
        name="shoulder_boss",
    )
    base.visual(
        Cylinder(radius=pin_radius, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed,
        name="shoulder_pin",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=brushed,
        name="shoulder_cap",
    )

    arm_0 = model.part("arm_0")
    arm_0.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _capsule_profile(arm0_len, arm0_width * 0.5),
                [_circle_profile(0.0, 0.0, hinge_hole), _circle_profile(arm0_len, 0.0, hinge_hole)],
                plate_thickness,
                center=True,
            ),
            "arm_0_plate",
        ),
        origin=Origin(),
        material=graphite,
        name="arm_plate",
    )
    arm_0.visual(
        Box((arm0_len - 0.110, 0.018, 0.006)),
        origin=Origin(xyz=(arm0_len * 0.5, 0.0, 0.0075)),
        material=dark,
        name="cable_cover",
    )
    arm_0.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(arm0_len, 0.0, 0.009)),
        material=brushed,
        name="elbow_boss",
    )
    arm_0.visual(
        Cylinder(radius=pin_radius, length=0.033),
        origin=Origin(xyz=(arm0_len, 0.0, 0.0205)),
        material=brushed,
        name="elbow_pin",
    )
    arm_0.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(arm0_len, 0.0, 0.034)),
        material=brushed,
        name="elbow_cap",
    )

    arm_1 = model.part("arm_1")
    arm_1.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _capsule_profile(arm1_len, arm1_width * 0.5),
                [_circle_profile(0.0, 0.0, hinge_hole), _circle_profile(arm1_len, 0.0, hinge_hole)],
                plate_thickness,
                center=True,
            ),
            "arm_1_plate",
        ),
        origin=Origin(),
        material=graphite,
        name="arm_plate",
    )
    arm_1.visual(
        Box((arm1_len - 0.100, 0.016, 0.006)),
        origin=Origin(xyz=(arm1_len * 0.5, 0.0, 0.0075)),
        material=dark,
        name="cable_cover",
    )
    arm_1.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(arm1_len, 0.0, 0.009)),
        material=brushed,
        name="wrist_boss",
    )
    arm_1.visual(
        Cylinder(radius=pin_radius, length=0.033),
        origin=Origin(xyz=(arm1_len, 0.0, 0.0205)),
        material=brushed,
        name="wrist_pin",
    )
    arm_1.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(arm1_len, 0.0, 0.034)),
        material=brushed,
        name="wrist_cap",
    )

    service_plate = model.part("service_plate")
    service_outer = _translate_profile(rounded_rect_profile(0.130, 0.090, 0.012), dx=0.055)
    service_holes = [
        _circle_profile(0.0, 0.0, hinge_hole),
        _circle_profile(0.078, -0.026, 0.005, 24),
        _circle_profile(0.078, 0.026, 0.005, 24),
    ]
    service_plate.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(service_outer, service_holes, plate_thickness, center=True),
            "service_plate",
        ),
        origin=Origin(),
        material=amber,
        name="mount_plate",
    )
    service_plate.visual(
        Box((0.060, 0.045, 0.008)),
        origin=Origin(xyz=(0.074, 0.0, 0.0085)),
        material=rubber,
        name="tool_pad",
    )
    service_plate.visual(
        Box((0.045, 0.010, 0.010)),
        origin=Origin(xyz=(0.098, 0.0, 0.017)),
        material=dark,
        name="cable_clamp",
    )

    model.articulation(
        "base_to_arm_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_0,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "arm_0_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=arm_1,
        origin=Origin(xyz=(arm0_len, 0.0, level_step), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=2.55),
    )
    model.articulation(
        "arm_1_to_service_plate",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=service_plate,
        origin=Origin(xyz=(arm1_len, 0.0, level_step), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.20, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    service_plate = object_model.get_part("service_plate")
    shoulder = object_model.get_articulation("base_to_arm_0")
    elbow = object_model.get_articulation("arm_0_to_arm_1")
    wrist = object_model.get_articulation("arm_1_to_service_plate")

    ctx.allow_overlap(
        base,
        arm_0,
        elem_a="shoulder_pin",
        elem_b="arm_plate",
        reason="The shoulder screw is intentionally captured through the proximal hinge bore.",
    )
    ctx.allow_overlap(
        arm_0,
        arm_1,
        elem_a="elbow_pin",
        elem_b="arm_plate",
        reason="The elbow screw intentionally passes through the upper link bore as the hinge shaft.",
    )
    ctx.allow_overlap(
        arm_1,
        service_plate,
        elem_a="wrist_pin",
        elem_b="mount_plate",
        reason="The wrist screw intentionally passes through the service-plate bore as the hinge shaft.",
    )

    ctx.check(
        "serial hinge chain",
        len(object_model.articulations) == 3,
        details=f"articulations={len(object_model.articulations)}",
    )
    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        ctx.expect_gap(
            arm_0,
            base,
            axis="z",
            positive_elem="arm_plate",
            negative_elem="base_plate",
            min_gap=0.006,
            max_gap=0.012,
            name="shoulder plate clears base plate",
        )
        ctx.expect_gap(
            arm_1,
            arm_0,
            axis="z",
            positive_elem="arm_plate",
            negative_elem="arm_plate",
            min_gap=0.008,
            max_gap=0.016,
            name="folded links stack without rubbing",
        )
        ctx.expect_gap(
            service_plate,
            arm_1,
            axis="z",
            positive_elem="mount_plate",
            negative_elem="arm_plate",
            min_gap=0.008,
            max_gap=0.016,
            name="service plate stacks above second link",
        )
        ctx.expect_overlap(
            arm_1,
            arm_0,
            axes="xy",
            elem_a="arm_plate",
            elem_b="arm_plate",
            min_overlap=0.050,
            name="folded links nest compactly",
        )
        ctx.expect_within(
            base,
            arm_0,
            axes="xy",
            inner_elem="shoulder_pin",
            outer_elem="arm_plate",
            margin=0.0,
            name="shoulder pin stays inside proximal link footprint",
        )
        ctx.expect_overlap(
            base,
            arm_0,
            axes="z",
            elem_a="shoulder_pin",
            elem_b="arm_plate",
            min_overlap=0.008,
            name="shoulder pin passes through link thickness",
        )
        ctx.expect_within(
            arm_0,
            arm_1,
            axes="xy",
            inner_elem="elbow_pin",
            outer_elem="arm_plate",
            margin=0.0,
            name="elbow pin stays inside upper link footprint",
        )
        ctx.expect_overlap(
            arm_0,
            arm_1,
            axes="z",
            elem_a="elbow_pin",
            elem_b="arm_plate",
            min_overlap=0.008,
            name="elbow pin passes through upper link thickness",
        )
        ctx.expect_within(
            arm_1,
            service_plate,
            axes="xy",
            inner_elem="wrist_pin",
            outer_elem="mount_plate",
            margin=0.0,
            name="wrist pin stays inside service plate footprint",
        )
        ctx.expect_overlap(
            arm_1,
            service_plate,
            axes="z",
            elem_a="wrist_pin",
            elem_b="mount_plate",
            min_overlap=0.008,
            name="wrist pin passes through service plate thickness",
        )

    folded_position = ctx.part_world_position(service_plate)
    with ctx.pose({elbow: 2.55}):
        extended_position = ctx.part_world_position(service_plate)
    ctx.check(
        "elbow fold opens outward",
        folded_position is not None
        and extended_position is not None
        and extended_position[0] > folded_position[0] + 0.30,
        details=f"folded={folded_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
