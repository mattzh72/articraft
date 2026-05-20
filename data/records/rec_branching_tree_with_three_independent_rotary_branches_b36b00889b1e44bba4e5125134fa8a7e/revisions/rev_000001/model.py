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


HINGE_RADIUS = 0.155
HINGE_Z = 0.039
ARM_THICKNESS = 0.018
ARM_COLLAR_THICKNESS = 0.006
ARM_REACH = 0.340


def _arm_link_shape() -> cq.Workplane:
    """Flat linkage arm with a pivot eye, raised boss, distal hole, and lightening slot."""

    eye_outer = 0.058
    pin_clearance = 0.031
    bar_width = 0.046
    bar_start = 0.040
    bar_end = ARM_REACH
    slot_start = 0.120
    slot_end = 0.270
    slot_width = 0.018

    eye = cq.Workplane("XY").cylinder(ARM_THICKNESS, eye_outer)
    bar = (
        cq.Workplane("XY")
        .box(bar_end - bar_start, bar_width, ARM_THICKNESS)
        .translate(((bar_start + bar_end) / 2.0, 0.0, 0.0))
    )
    distal_pad = (
        cq.Workplane("XY")
        .cylinder(ARM_THICKNESS, 0.036)
        .translate((ARM_REACH, 0.0, 0.0))
    )
    top_boss = (
        cq.Workplane("XY")
        .cylinder(ARM_COLLAR_THICKNESS, 0.046)
        .translate((0.0, 0.0, ARM_THICKNESS / 2.0 + ARM_COLLAR_THICKNESS / 2.0))
    )

    link = eye.union(bar).union(distal_pad).union(top_boss)

    pivot_bore = (
        cq.Workplane("XY")
        .cylinder(0.070, pin_clearance)
        .translate((0.0, 0.0, 0.004))
    )
    distal_bore = (
        cq.Workplane("XY")
        .cylinder(0.070, 0.012)
        .translate((ARM_REACH, 0.0, 0.0))
    )
    slot_middle = (
        cq.Workplane("XY")
        .box(slot_end - slot_start, slot_width, 0.070)
        .translate(((slot_start + slot_end) / 2.0, 0.0, 0.0))
    )
    slot_start_cap = (
        cq.Workplane("XY")
        .cylinder(0.070, slot_width / 2.0)
        .translate((slot_start, 0.0, 0.0))
    )
    slot_end_cap = (
        cq.Workplane("XY")
        .cylinder(0.070, slot_width / 2.0)
        .translate((slot_end, 0.0, 0.0))
    )
    lightening_slot = slot_middle.union(slot_start_cap).union(slot_end_cap)

    return link.cut(pivot_bore).cut(distal_bore).cut(lightening_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_rotary_tree")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_plate = model.material("graphite_plate", rgba=(0.16, 0.17, 0.18, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    arm_blue = model.material("anodized_blue", rgba=(0.08, 0.34, 0.78, 1.0))

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.095, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_plate,
        name="center_plate",
    )
    hub.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=aluminum,
        name="center_boss",
    )

    arm_mesh = mesh_from_cadquery(_arm_link_shape(), "tree_arm_link", tolerance=0.0008)

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        hinge_xy = (HINGE_RADIUS * c, HINGE_RADIUS * s)

        hub.visual(
            Box((HINGE_RADIUS, 0.050, 0.014)),
            origin=Origin(
                xyz=(0.5 * HINGE_RADIUS * c, 0.5 * HINGE_RADIUS * s, 0.007),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_plate,
            name=f"branch_spoke_{index}",
        )
        hub.visual(
            Cylinder(radius=0.066, length=0.014),
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], 0.007)),
            material=dark_plate,
            name=f"hinge_pad_{index}",
        )
        hub.visual(
            Cylinder(radius=0.052, length=0.006),
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], 0.027)),
            material=aluminum,
            name=f"thrust_washer_{index}",
        )
        hub.visual(
            Cylinder(radius=0.024, length=0.047),
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], 0.0375)),
            material=bearing_steel,
            name=f"hinge_pin_{index}",
        )
        hub.visual(
            Cylinder(radius=0.038, length=0.006),
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], 0.057)),
            material=bearing_steel,
            name=f"retaining_cap_{index}",
        )

        arm = model.part(f"arm_{index}")
        arm.visual(
            arm_mesh,
            origin=Origin(),
            material=arm_blue,
            name="arm_link",
        )
        arm.inertial = Inertial.from_geometry(
            Box((ARM_REACH + 0.070, 0.080, 0.030)),
            mass=0.18,
            origin=Origin(xyz=(0.170, 0.0, 0.002)),
        )

        model.articulation(
            f"hub_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=arm,
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], HINGE_Z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.75, upper=0.75),
        )

    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.240, length=0.064),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("hub")
    arms = [object_model.get_part(f"arm_{index}") for index in range(3)]
    hinges = [object_model.get_articulation(f"hub_to_arm_{index}") for index in range(3)]

    ctx.check("three independent arm parts", len(arms) == 3, details=f"arms={arms}")
    ctx.check(
        "three revolute hinges",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in hinges),
        details=f"hinges={[joint.articulation_type for joint in hinges]}",
    )

    for index, (arm, hinge) in enumerate(zip(arms, hinges)):
        limits = hinge.motion_limits
        ctx.check(
            f"arm_{index} has balanced rotary stops",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < -0.7
            and limits.upper > 0.7,
            details=f"limits={limits}",
        )
        ctx.expect_contact(
            arm,
            hub,
            elem_a="arm_link",
            elem_b=f"thrust_washer_{index}",
            contact_tol=0.0005,
            name=f"arm_{index} rests on thrust washer",
        )
        ctx.expect_contact(
            arm,
            hub,
            elem_a="arm_link",
            elem_b=f"retaining_cap_{index}",
            contact_tol=0.0005,
            name=f"arm_{index} captured under retaining cap",
        )

        rest_aabb = ctx.part_element_world_aabb(arm, elem="arm_link")
        with ctx.pose({hinge: 0.65}):
            moved_aabb = ctx.part_element_world_aabb(arm, elem="arm_link")
        if rest_aabb is None or moved_aabb is None:
            ctx.fail(f"arm_{index} pose aabb available", "could not measure arm link AABB")
        else:
            rest_center = (
                (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0,
                (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0,
            )
            moved_center = (
                (moved_aabb[0][0] + moved_aabb[1][0]) / 2.0,
                (moved_aabb[0][1] + moved_aabb[1][1]) / 2.0,
            )
            planar_shift = math.hypot(moved_center[0] - rest_center[0], moved_center[1] - rest_center[1])
            ctx.check(
                f"arm_{index} rotates about its own hinge",
                planar_shift > 0.035,
                details=f"rest_center={rest_center}, moved_center={moved_center}, shift={planar_shift}",
            )

    return ctx.report()


object_model = build_object_model()
