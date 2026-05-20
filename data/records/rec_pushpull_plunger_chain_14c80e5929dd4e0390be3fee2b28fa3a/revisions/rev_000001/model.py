from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _annular_cylinder_z(outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular tube along local +Z, with open ends."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _guide_tube_mesh():
    length = 0.320
    tube = _annular_cylinder_z(0.032, 0.023, length)

    # Raised end bushings keep the sleeve visually tubular while leaving the
    # bore clear for the sliding plunger.
    rear_bushing = _annular_cylinder_z(0.040, 0.0235, 0.022).translate((0.0, 0.0, -0.011))
    front_bushing = _annular_cylinder_z(0.040, 0.0235, 0.022).translate((0.0, 0.0, length - 0.011))
    tube = tube.union(rear_bushing).union(front_bushing)

    # A long relieved slot on the top of the guide reveals that this is a guide
    # tube rather than a solid bar.  The slot stops before the bushings.
    slot = cq.Workplane("XY").box(0.090, 0.028, 0.205).translate((-0.032, 0.0, length * 0.50))
    tube = tube.cut(slot)

    # Rotate the CadQuery Z-axis tube onto the object's +X slide axis.
    return tube.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)


def _nose_latch_mesh():
    width = 0.018

    # Side-profile of a small hooked nose latch, extruded across the hinge pin.
    latch_profile = [
        (0.000, -0.006),
        (0.090, -0.006),
        (0.098, -0.029),
        (0.113, -0.029),
        (0.107, 0.007),
        (0.010, 0.007),
    ]
    body = cq.Workplane("XZ").polyline(latch_profile).close().extrude(width).translate((0.0, width / 2.0, 0.0))

    eye = cq.Workplane("XZ").circle(0.0105).extrude(width).translate((0.0, width / 2.0, 0.0))
    bore = cq.Workplane("XZ").circle(0.0040).extrude(width * 1.8).translate((0.0, width * 0.9, 0.0))

    # Cut the pin bore after unioning the leaf and the eye so the hinge pin has
    # real visible clearance through the whole latch knuckle.
    return eye.union(body).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_plunger_nose_latch")

    dark_steel = Material("dark_burnished_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    blued_steel = Material("blued_guide_steel", rgba=(0.16, 0.18, 0.21, 1.0))
    polished_steel = Material("polished_plunger_steel", rgba=(0.62, 0.65, 0.67, 1.0))
    black_rubber = Material("black_rubber_grip", rgba=(0.015, 0.014, 0.013, 1.0))
    brass = Material("brass_pin", rgba=(0.85, 0.63, 0.23, 1.0))

    guide = model.part("guide_tube")
    guide.visual(
        mesh_from_cadquery(_guide_tube_mesh(), "guide_tube_slotted_shell", tolerance=0.0008),
        material=blued_steel,
        name="guide_shell",
    )
    guide.visual(
        Box((0.420, 0.120, 0.012)),
        origin=Origin(xyz=(0.165, 0.0, -0.052)),
        material=dark_steel,
        name="mounting_foot",
    )
    for x in (0.070, 0.255):
        guide.visual(
            Box((0.040, 0.096, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.032)),
            material=dark_steel,
            name=f"saddle_{x:.2f}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.017, length=0.450),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="slide_rod",
    )
    plunger.visual(
        Cylinder(radius=0.031, length=0.044),
        origin=Origin(xyz=(-0.520, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="pull_knob",
    )
    plunger.visual(
        Box((0.022, 0.052, 0.025)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=polished_steel,
        name="fork_bridge",
    )
    for y in (-0.020, 0.020):
        plunger.visual(
            Box((0.064, 0.007, 0.034)),
            origin=Origin(xyz=(-0.024, y, 0.0)),
            material=polished_steel,
            name=f"fork_cheek_{'neg' if y < 0 else 'pos'}",
        )
    plunger.visual(
        Cylinder(radius=0.0040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hinge_pin",
    )

    latch = model.part("nose_latch")
    latch.visual(
        mesh_from_cadquery(_nose_latch_mesh(), "nose_latch_hooked_leaf", tolerance=0.0005),
        material=brass,
        name="hooked_leaf",
    )

    model.articulation(
        "guide_to_plunger",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=plunger,
        # The child frame is the latch hinge line.  At rest it sits just ahead
        # of the guide mouth; positive travel pushes the whole plunger outward.
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.100),
    )
    model.articulation(
        "plunger_to_nose_latch",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=latch,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.35, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_tube")
    plunger = object_model.get_part("plunger")
    latch = object_model.get_part("nose_latch")
    slide = object_model.get_articulation("guide_to_plunger")
    hinge = object_model.get_articulation("plunger_to_nose_latch")

    ctx.allow_overlap(
        latch,
        plunger,
        elem_a="hooked_leaf",
        elem_b="hinge_pin",
        reason="The hinge pin is intentionally captured inside the nose latch eye as a local bearing fit.",
    )

    ctx.expect_overlap(
        plunger,
        guide,
        axes="x",
        elem_a="slide_rod",
        elem_b="guide_shell",
        min_overlap=0.240,
        name="plunger rod remains deeply inserted at rest",
    )
    ctx.expect_within(
        plunger,
        guide,
        axes="yz",
        inner_elem="slide_rod",
        outer_elem="guide_shell",
        margin=0.0,
        name="plunger rod is centered within the guide bore envelope",
    )
    ctx.expect_overlap(
        plunger,
        latch,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="hooked_leaf",
        min_overlap=0.006,
        name="nose latch eye is carried on the hinge pin",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.100}):
        ctx.expect_overlap(
            plunger,
            guide,
            axes="x",
            elem_a="slide_rod",
            elem_b="guide_shell",
            min_overlap=0.200,
            name="extended plunger is still captured by the tube",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "prismatic joint extends plunger outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.095,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({hinge: 0.90}):
        raised_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "nose latch rotates upward on carried hinge",
        closed_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
