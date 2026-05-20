from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .faces(">Z")
        .workplane()
        .circle(inner_radius)
        .cutBlind(-length)
    )
    return mesh_from_cadquery(tube, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_training_pump")

    frame_black = model.material("frame_black", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dial = model.material("dial", rgba=(0.95, 0.95, 0.93, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.80, 0.86, 0.30))
    signal_red = model.material("signal_red", rgba=(0.82, 0.12, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.300, 0.105, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=frame_black,
        name="foot",
    )
    frame.visual(
        Box((0.030, 0.040, 0.022)),
        origin=Origin(xyz=(0.000, 0.002, 0.019)),
        material=frame_black,
        name="bridge",
    )
    frame.visual(
        Cylinder(radius=0.033, length=0.028),
        origin=Origin(xyz=(0.000, 0.020, 0.053)),
        material=frame_black,
        name="barrel_collar",
    )
    frame.visual(
        _tube_mesh(
            "pump_barrel_shell",
            outer_radius=0.028,
            inner_radius=0.024,
            length=0.230,
        ),
        origin=Origin(xyz=(0.000, 0.020, 0.053)),
        material=frame_black,
        name="barrel_shell",
    )
    for index, x_pos in enumerate((-0.020, 0.020)):
        frame.visual(
            Box((0.016, 0.030, 0.070)),
            origin=Origin(xyz=(x_pos, 0.020, 0.035)),
            material=frame_black,
            name=f"strut_{index}",
        )
    frame.visual(
        Box((0.040, 0.018, 0.090)),
        origin=Origin(xyz=(0.000, 0.052, 0.045)),
        material=frame_black,
        name="rear_brace",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.000, -0.018, 0.019)),
        material=frame_black,
        name="gauge_housing",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.000, -0.018, 0.032)),
        material=frame_black,
        name="gauge_bezel",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.0016),
        origin=Origin(xyz=(0.000, -0.018, 0.0297)),
        material=dial,
        name="gauge_face",
    )
    frame.visual(
        Cylinder(radius=0.029, length=0.0026),
        origin=Origin(xyz=(0.000, -0.018, 0.0335)),
        material=glass,
        name="gauge_glass",
    )
    frame.visual(
        _tube_mesh(
            "pump_button_boss",
            outer_radius=0.008,
            inner_radius=0.0056,
            length=0.011,
        ),
        origin=Origin(
            xyz=(0.0345, -0.018, 0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_black,
        name="button_boss",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.300, 0.105, 0.360)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.180)),
    )

    rod = model.part("rod")
    rod.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(xyz=(0.000, 0.000, -0.110)),
        material=steel,
        name="shaft",
    )
    rod.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.206)),
        material=rubber_black,
        name="piston",
    )
    rod.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        material=steel,
        name="stem",
    )
    rod.visual(
        Cylinder(radius=0.010, length=0.200),
        origin=Origin(
            xyz=(0.000, 0.000, 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )
    for index, x_pos in enumerate((-0.070, 0.070)):
        rod.visual(
            Cylinder(radius=0.013, length=0.060),
            origin=Origin(
                xyz=(x_pos, 0.000, 0.050),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber_black,
            name=f"grip_{index}",
        )
    rod.inertial = Inertial.from_geometry(
        Box((0.220, 0.030, 0.285)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, -0.078)),
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.0042, length=0.0010),
        origin=Origin(xyz=(0.000, 0.000, 0.0005)),
        material=signal_red,
        name="hub",
    )
    needle.visual(
        Box((0.022, 0.0025, 0.0008)),
        origin=Origin(xyz=(0.011, 0.000, 0.0009)),
        material=signal_red,
        name="arm",
    )
    needle.visual(
        Box((0.012, 0.0040, 0.0008)),
        origin=Origin(xyz=(0.024, 0.000, 0.0009)),
        material=signal_red,
        name="tip",
    )
    needle.visual(
        Box((0.008, 0.0018, 0.0008)),
        origin=Origin(xyz=(-0.007, 0.000, 0.0009)),
        material=signal_red,
        name="counterweight",
    )
    needle.inertial = Inertial.from_geometry(
        Box((0.036, 0.008, 0.004)),
        mass=0.01,
        origin=Origin(xyz=(0.006, 0.000, 0.001)),
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(
            xyz=(0.005, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=signal_red,
        name="stem",
    )
    button.visual(
        Cylinder(radius=0.0070, length=0.006),
        origin=Origin(
            xyz=(0.012, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=signal_red,
        name="cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.020, 0.016, 0.016)),
        mass=0.015,
        origin=Origin(xyz=(0.010, 0.000, 0.000)),
    )

    model.articulation(
        "frame_to_rod",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rod,
        origin=Origin(xyz=(0.000, 0.020, 0.283)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.6,
            lower=0.0,
            upper=0.170,
        ),
    )
    model.articulation(
        "frame_to_needle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=needle,
        origin=Origin(xyz=(0.000, -0.018, 0.0308)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=8.0,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "frame_to_button",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=button,
        origin=Origin(xyz=(0.036, -0.018, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rod = object_model.get_part("rod")
    needle = object_model.get_part("needle")
    button = object_model.get_part("button")

    rod_joint = object_model.get_articulation("frame_to_rod")
    needle_joint = object_model.get_articulation("frame_to_needle")
    button_joint = object_model.get_articulation("frame_to_button")

    rod_limits = rod_joint.motion_limits
    needle_limits = needle_joint.motion_limits
    button_limits = button_joint.motion_limits

    ctx.expect_within(
        rod,
        frame,
        axes="xy",
        inner_elem="shaft",
        outer_elem="barrel_shell",
        margin=0.001,
        name="rod stays centered in the barrel at rest",
    )
    ctx.expect_overlap(
        rod,
        frame,
        axes="z",
        elem_a="shaft",
        elem_b="barrel_shell",
        min_overlap=0.210,
        name="rod remains deeply inserted at rest",
    )
    ctx.expect_within(
        button,
        frame,
        axes="yz",
        inner_elem="stem",
        outer_elem="button_boss",
        margin=0.0008,
        name="button stem stays guided by the boss at rest",
    )
    ctx.expect_overlap(
        button,
        frame,
        axes="x",
        elem_a="stem",
        elem_b="button_boss",
        min_overlap=0.0035,
        name="button stem remains inserted at rest",
    )

    if rod_limits is not None and rod_limits.upper is not None:
        rest_rod_pos = ctx.part_world_position(rod)
        with ctx.pose({rod_joint: rod_limits.upper}):
            ctx.expect_within(
                rod,
                frame,
                axes="xy",
                inner_elem="shaft",
                outer_elem="barrel_shell",
                margin=0.001,
                name="rod stays centered in the barrel when extended",
            )
            ctx.expect_overlap(
                rod,
                frame,
                axes="z",
                elem_a="shaft",
                elem_b="barrel_shell",
                min_overlap=0.045,
                name="rod keeps retained insertion at full stroke",
            )
            extended_rod_pos = ctx.part_world_position(rod)
        ctx.check(
            "rod extends upward from the barrel",
            rest_rod_pos is not None
            and extended_rod_pos is not None
            and extended_rod_pos[2] > rest_rod_pos[2] + 0.12,
            details=f"rest={rest_rod_pos}, extended={extended_rod_pos}",
        )

    if needle_limits is not None and needle_limits.lower is not None and needle_limits.upper is not None:
        with ctx.pose({needle_joint: needle_limits.lower}):
            lower_tip = _aabb_center(ctx.part_element_world_aabb(needle, elem="tip"))
        with ctx.pose({needle_joint: needle_limits.upper}):
            upper_tip = _aabb_center(ctx.part_element_world_aabb(needle, elem="tip"))
        ctx.check(
            "needle sweeps across the gauge face",
            lower_tip is not None
            and upper_tip is not None
            and (
                abs(upper_tip[0] - lower_tip[0]) > 0.020
                or abs(upper_tip[1] - lower_tip[1]) > 0.020
            ),
            details=f"lower_tip={lower_tip}, upper_tip={upper_tip}",
        )

    if button_limits is not None and button_limits.upper is not None:
        rest_button_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper}):
            ctx.expect_within(
                button,
                frame,
                axes="yz",
                inner_elem="stem",
                outer_elem="button_boss",
                margin=0.0008,
                name="button stem stays guided when pressed",
            )
            ctx.expect_overlap(
                button,
                frame,
                axes="x",
                elem_a="stem",
                elem_b="button_boss",
                min_overlap=0.006,
                name="button stem remains retained when pressed",
            )
            pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            "button presses inward",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[0] < rest_button_pos[0] - 0.0025,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
