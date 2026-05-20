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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_RADIUS = 0.145
BODY_HEIGHT = 0.218
HINGE_Y = 0.158
HINGE_Z = 0.220
HANDLE_Z = 0.212


def _dome_cap(radius: float, height: float) -> cq.Workplane:
    """Return a spherical cap with its flat base on z=0."""
    center_z = height - radius
    sphere = cq.Workplane("XY").sphere(radius).translate((0.0, 0.0, center_z))
    lower_halfspace = cq.Workplane("XY").box(1.0, 1.0, 2.0).translate((0.0, 0.0, -1.0))
    return sphere.cut(lower_halfspace)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_rice_cooker")

    warm_white = model.material("warm_white", rgba=(0.93, 0.91, 0.84, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.42, 0.43, 0.42, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.08, 0.075, 1.0))
    display_blue = model.material("display_blue", rgba=(0.12, 0.42, 0.70, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.08, 0.045, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.30, 0.31, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.166),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=warm_white,
        name="main_shell",
    )
    body.visual(
        Cylinder(radius=0.136, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=soft_gray,
        name="lower_band",
    )
    body.visual(
        Cylinder(radius=0.148, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=soft_gray,
        name="top_rim",
    )

    # Flat, readable front appliance controls on the cylindrical shell.
    body.visual(
        Box((0.090, 0.009, 0.074)),
        origin=Origin(xyz=(0.0, -0.1480, 0.139)),
        material=dark_gray,
        name="control_panel",
    )
    body.visual(
        Box((0.064, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.1540, 0.160)),
        material=display_blue,
        name="front_display",
    )
    body.visual(
        Box((0.076, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -0.1510, 0.204)),
        material=soft_gray,
        name="front_latch_zone",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.0, -0.1510, 0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="button_socket",
    )

    # Rear lid hinge knuckles and their shell brackets.
    for x in (-0.086, 0.086):
        body.visual(
            Box((0.042, 0.090, 0.018)),
            origin=Origin(xyz=(x, 0.133, 0.209)),
            material=hinge_gray,
            name=f"rear_hinge_bracket_{0 if x < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.009, length=0.046),
            origin=Origin(xyz=(x, 0.166, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_gray,
            name=f"rear_hinge_barrel_{0 if x < 0 else 1}",
        )

    # Side shoulder pivots for the folding carry handle.
    for x, suffix in ((-0.153, "0"), (0.153, "1")):
        body.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, 0.0, HANDLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_gray,
            name="side_pivot_0" if suffix == "0" else "side_pivot_1",
        )

    lid = model.part("lid")
    dome = _dome_cap(radius=0.235, height=0.052).translate((0.0, -0.149, 0.010))
    lid.visual(
        mesh_from_cadquery(dome, "domed_lid", tolerance=0.0015, angular_tolerance=0.08),
        material=warm_white,
        name="domed_lid",
    )
    lid.visual(
        Cylinder(radius=0.146, length=0.012),
        origin=Origin(xyz=(0.0, -0.149, 0.006)),
        material=soft_gray,
        name="lid_rim",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.070, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.278, 0.018)),
        material=soft_gray,
        name="lid_latch_tab",
    )

    handle = model.part("carry_handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.344),
        origin=Origin(xyz=(0.0, 0.0, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="top_grip",
    )
    for x, suffix in ((-0.169, "0"), (0.169, "1")):
        handle.visual(
            Cylinder(radius=0.006, length=0.106),
            origin=Origin(xyz=(x, 0.0, 0.053)),
            material=black_plastic,
            name="side_arm_0" if suffix == "0" else "side_arm_1",
        )
        handle.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name="pivot_eye_0" if suffix == "0" else "pivot_eye_1",
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_red,
        name="button_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.0, -0.154, 0.091)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.04, lower=0.0, upper=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("carry_handle")
    button = object_model.get_part("power_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")
    button_slide = object_model.get_articulation("button_slide")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="lid_rim",
        negative_elem="top_rim",
        name="closed lid sits just above top rim",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="pivot_eye_1",
        elem_b="side_pivot_1",
        contact_tol=0.0005,
        name="handle eye is carried by side pivot",
    )
    ctx.expect_contact(
        body,
        button,
        elem_a="button_socket",
        elem_b="button_cap",
        contact_tol=0.0005,
        name="power button is seated in socket",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    rest_handle_aabb = ctx.part_world_aabb(handle)
    rest_button_pos = ctx.part_world_position(button)

    with ctx.pose({lid_hinge: 0.9}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_pivot: 0.8}):
        folded_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "carry handle rotates about shoulder pivots",
        rest_handle_aabb is not None
        and folded_handle_aabb is not None
        and abs(folded_handle_aabb[0][1] - rest_handle_aabb[0][1]) > 0.035,
        details=f"rest={rest_handle_aabb}, rotated={folded_handle_aabb}",
    )

    with ctx.pose({button_slide: 0.003}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "power button presses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
