from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_laser_clicker")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_black = model.material("satin_black", rgba=(0.020, 0.022, 0.025, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.25, 0.28, 0.30, 0.48))
    red_lens = model.material("red_lens", rgba=(1.0, 0.035, 0.015, 0.95))
    white_mark = model.material("white_mark", rgba=(0.85, 0.86, 0.82, 1.0))
    metal_pin = model.material("brushed_pin", rgba=(0.65, 0.65, 0.62, 1.0))

    body = model.part("body")
    # URDF cylinders are local-Z; rotate them so the clicker barrel runs along X.
    barrel_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    body.visual(
        Cylinder(radius=0.0120, length=0.112),
        origin=barrel_x,
        material=matte_black,
        name="main_barrel",
    )
    body.visual(
        Sphere(radius=0.0112),
        origin=Origin(xyz=(0.0555, 0.0, 0.0)),
        material=matte_black,
        name="rounded_nose",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0018),
        origin=Origin(xyz=(0.0672, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red_lens,
        name="laser_aperture",
    )
    body.visual(
        Cylinder(radius=0.01235, length=0.0012),
        origin=Origin(xyz=(-0.0552, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="battery_seam",
    )
    body.visual(
        Box((0.082, 0.015, 0.0040)),
        origin=Origin(xyz=(0.018, 0.0, 0.0140)),
        material=satin_black,
        name="top_saddle",
    )
    body.visual(
        Box((0.016, 0.010, 0.0012)),
        origin=Origin(xyz=(0.018, 0.0, 0.0166)),
        material=dark_rubber,
        name="button_recess",
    )

    hinge_y = -0.0130
    hinge_z = 0.0205
    hinge_x = 0.0180
    for index, x in enumerate((-0.018, 0.054)):
        body.visual(
            Box((0.012, 0.006, 0.006)),
            origin=Origin(xyz=(x, -0.0100, 0.0185)),
            material=matte_black,
            name=f"hinge_support_{index}",
        )
        body.visual(
            Cylinder(radius=0.00225, length=0.010),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"hinge_knuckle_{index}",
        )
    body.visual(
        Cylinder(radius=0.0009, length=0.084),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_pin,
        name="hinge_pin",
    )

    pointer_button = model.part("pointer_button")
    pointer_button.visual(
        Cylinder(radius=0.0052, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=red_lens,
        name="button_cap",
    )

    thumb_guard = model.part("thumb_guard")
    guard_plate_shape = (
        cq.Workplane("XY")
        .box(0.058, 0.020, 0.0022)
        .edges("|Z")
        .fillet(0.004)
    )
    thumb_guard.visual(
        mesh_from_cadquery(guard_plate_shape, "thumb_guard_plate", tolerance=0.00035),
        origin=Origin(xyz=(0.0, 0.0115, 0.0028)),
        material=smoky_clear,
        name="guard_plate",
    )
    thumb_guard.visual(
        Cylinder(radius=0.00225, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoky_clear,
        name="guard_barrel",
    )
    thumb_guard.visual(
        Box((0.026, 0.0022, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0020, 0.0018)),
        material=smoky_clear,
        name="guard_web",
    )

    end_cap = model.part("end_cap")
    end_cap.visual(
        Cylinder(radius=0.0120, length=0.024),
        origin=Origin(xyz=(-0.0120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="cap_shell",
    )
    for index, x in enumerate((-0.004, -0.008, -0.012, -0.016, -0.020)):
        end_cap.visual(
            Cylinder(radius=0.01275, length=0.0011),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name=f"grip_ring_{index}",
        )
    end_cap.visual(
        Box((0.010, 0.0020, 0.0010)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0122)),
        material=white_mark,
        name="cap_index_mark",
    )

    model.articulation(
        "body_to_pointer_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pointer_button,
        origin=Origin(xyz=(0.018, 0.0, 0.0160)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.04, lower=0.0, upper=0.0014),
    )
    model.articulation(
        "body_to_thumb_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=thumb_guard,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_end_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=end_cap,
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    pointer_button = object_model.get_part("pointer_button")
    thumb_guard = object_model.get_part("thumb_guard")
    end_cap = object_model.get_part("end_cap")
    guard_hinge = object_model.get_articulation("body_to_thumb_guard")
    cap_joint = object_model.get_articulation("body_to_end_cap")
    button_joint = object_model.get_articulation("body_to_pointer_button")

    ctx.allow_overlap(
        body,
        thumb_guard,
        elem_a="hinge_pin",
        elem_b="guard_barrel",
        reason="The metal hinge pin is intentionally captured inside the thumb guard hinge barrel.",
    )
    ctx.expect_within(
        body,
        thumb_guard,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="guard_barrel",
        margin=0.0002,
        name="hinge pin is centered inside guard barrel",
    )
    ctx.expect_overlap(
        body,
        thumb_guard,
        axes="x",
        elem_a="hinge_pin",
        elem_b="guard_barrel",
        min_overlap=0.020,
        name="hinge pin spans the guard barrel",
    )
    ctx.expect_gap(
        thumb_guard,
        pointer_button,
        axis="z",
        positive_elem="guard_plate",
        negative_elem="button_cap",
        min_gap=0.0015,
        max_gap=0.0060,
        name="guard clears the pointer button",
    )
    ctx.expect_contact(
        end_cap,
        body,
        elem_a="cap_shell",
        elem_b="main_barrel",
        contact_tol=0.0003,
        name="battery cap seats against barrel",
    )
    ctx.expect_overlap(
        end_cap,
        body,
        axes="yz",
        elem_a="cap_shell",
        elem_b="main_barrel",
        min_overlap=0.020,
        name="end cap shares the cylindrical body axis",
    )

    rest_guard_aabb = ctx.part_world_aabb(thumb_guard)
    with ctx.pose({guard_hinge: 1.25}):
        open_guard_aabb = ctx.part_world_aabb(thumb_guard)
    ctx.check(
        "thumb guard flips upward",
        rest_guard_aabb is not None
        and open_guard_aabb is not None
        and open_guard_aabb[1][2] > rest_guard_aabb[1][2] + 0.010,
        details=f"rest={rest_guard_aabb}, open={open_guard_aabb}",
    )

    rest_button_pos = ctx.part_world_position(pointer_button)
    with ctx.pose({button_joint: 0.0014}):
        pressed_button_pos = ctx.part_world_position(pointer_button)
    ctx.check(
        "pointer button depresses into body",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0010,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_cap_pos = ctx.part_world_position(end_cap)
    with ctx.pose({cap_joint: math.pi / 2.0}):
        turned_cap_pos = ctx.part_world_position(end_cap)
    ctx.check(
        "battery cap rotates about body axis",
        rest_cap_pos is not None
        and turned_cap_pos is not None
        and abs(rest_cap_pos[0] - turned_cap_pos[0]) < 1e-6
        and abs(rest_cap_pos[1] - turned_cap_pos[1]) < 1e-6
        and abs(rest_cap_pos[2] - turned_cap_pos[2]) < 1e-6,
        details=f"rest={rest_cap_pos}, turned={turned_cap_pos}",
    )

    return ctx.report()


object_model = build_object_model()
