from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HINGE_X = 0.160
HINGE_Z = 0.108
PANEL_X = -0.221


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid centered at the local origin."""
    x, y, z = size
    radius = min(radius, x * 0.25, y * 0.25, z * 0.45)
    body = cq.Workplane("XY").box(x, y, z)
    if radius > 0.0001:
        body = body.edges("|Z").fillet(radius)
        top_radius = min(radius * 0.55, z * 0.22)
        if top_radius > 0.0001:
            body = body.edges(">Z").fillet(top_radius)
    return body


def _lower_shell() -> cq.Workplane:
    """Cast-metal lower body with a blended front control pod."""
    main = _rounded_box((0.360, 0.270, 0.074), 0.027).translate((0.0, 0.0, 0.037))
    front_pod = _rounded_box((0.070, 0.230, 0.058), 0.018).translate((-0.180, 0.0, 0.040))
    rear_boss_0 = _rounded_box((0.065, 0.070, 0.036), 0.014).translate((0.146, -0.140, 0.085))
    rear_boss_1 = _rounded_box((0.065, 0.070, 0.036), 0.014).translate((0.146, 0.140, 0.085))
    return main.union(front_pod).union(rear_boss_0).union(rear_boss_1)


def _lid_shell() -> cq.Workplane:
    """Rounded upper lid whose local origin is on the rear hinge axis."""
    base = _rounded_box((0.306, 0.266, 0.062), 0.030).translate((-0.154, 0.0, 0.026))
    crown = (
        cq.Workplane("XY")
        .ellipse(0.108, 0.087)
        .extrude(0.034)
        .edges(">Z")
        .fillet(0.020)
        .translate((-0.155, 0.0, 0.054))
    )
    rear_hinge_blend = _rounded_box((0.052, 0.180, 0.022), 0.010).translate((-0.026, 0.0, 0.003))
    return base.union(crown).union(rear_hinge_blend)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_waffle_maker")

    cast = model.material("cast_dark_grey", rgba=(0.23, 0.235, 0.24, 1.0))
    side = model.material("molded_charcoal", rgba=(0.12, 0.125, 0.13, 1.0))
    plate_mat = model.material("seasoned_cooking_plate", rgba=(0.035, 0.034, 0.032, 1.0))
    panel_mat = model.material("gloss_black_panel", rgba=(0.015, 0.016, 0.018, 1.0))
    handle_mat = model.material("cool_touch_handle", rgba=(0.055, 0.055, 0.058, 1.0))
    hinge_mat = model.material("dark_hinge_cover", rgba=(0.08, 0.08, 0.085, 1.0))
    dial_mat = model.material("satin_browning_dial", rgba=(0.78, 0.76, 0.70, 1.0))
    red = model.material("power_red", rgba=(0.85, 0.05, 0.035, 1.0))
    green = model.material("ready_green", rgba=(0.08, 0.75, 0.20, 1.0))
    amber = model.material("warm_indicator", rgba=(1.0, 0.56, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_lower_shell(), "lower_cast_base", tolerance=0.0012),
        material=cast,
        name="lower_cast_base",
    )
    base.visual(
        Box((0.285, 0.030, 0.055)),
        origin=Origin(xyz=(-0.010, -0.137, 0.064)),
        material=side,
        name="side_cheek_0",
    )
    base.visual(
        Box((0.285, 0.030, 0.055)),
        origin=Origin(xyz=(-0.010, 0.137, 0.064)),
        material=side,
        name="side_cheek_1",
    )
    base.visual(
        Box((0.008, 0.196, 0.052)),
        origin=Origin(xyz=(-0.217, 0.0, 0.046)),
        material=panel_mat,
        name="front_panel",
    )
    base.visual(
        Box((0.008, 0.030, 0.006)),
        origin=Origin(xyz=(-0.222, -0.032, 0.075)),
        material=amber,
        name="power_indicator",
    )
    base.visual(
        Box((0.008, 0.030, 0.006)),
        origin=Origin(xyz=(-0.222, 0.032, 0.075)),
        material=green,
        name="ready_indicator",
    )

    base.visual(
        Box((0.250, 0.190, 0.008)),
        origin=Origin(xyz=(-0.025, 0.0, 0.078)),
        material=plate_mat,
        name="lower_plate",
    )
    for idx, y in enumerate((-0.070, -0.035, 0.000, 0.035, 0.070)):
        base.visual(
            Box((0.230, 0.004, 0.006)),
            origin=Origin(xyz=(-0.025, y, 0.085)),
            material=plate_mat,
            name=f"waffle_rib_y_{idx}",
        )
    for idx, x in enumerate((-0.115, -0.070, -0.025, 0.020, 0.065)):
        base.visual(
            Box((0.004, 0.172, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.085)),
            material=plate_mat,
            name=f"waffle_rib_x_{idx}",
        )

    for idx, y in enumerate((-0.158, 0.158)):
        base.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"base_hinge_ear_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "domed_upper_lid", tolerance=0.0012),
        material=cast,
        name="domed_shell",
    )
    lid.visual(
        Box((0.240, 0.186, 0.008)),
        origin=Origin(xyz=(-0.153, 0.0, -0.008)),
        material=plate_mat,
        name="upper_plate",
    )
    lid.visual(
        Box((0.050, 0.152, 0.016)),
        origin=Origin(xyz=(-0.028, 0.0, 0.000)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.0125, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.176),
        origin=Origin(xyz=(-0.310, 0.0, 0.033), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="front_handle",
    )
    lid.visual(
        Box((0.040, 0.024, 0.035)),
        origin=Origin(xyz=(-0.285, -0.074, 0.018)),
        material=handle_mat,
        name="handle_mount_0",
    )
    lid.visual(
        Box((0.040, 0.024, 0.035)),
        origin=Origin(xyz=(-0.285, 0.074, 0.018)),
        material=handle_mat,
        name="handle_mount_1",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.5, velocity=2.2, lower=0.0, upper=1.35),
    )

    dial = model.part("browning_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.022,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.060, 0.0045, flare=0.05, chamfer=0.0008),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=22.0),
            ),
            "browning_dial_cap",
        ),
        origin=Origin(xyz=(-0.01175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_mat,
        name="dial_cap",
    )
    model.articulation(
        "browning_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(PANEL_X, 0.0, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=4.0),
    )

    def _button(name: str, y: float, material: Material) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.004, 0.018, 0.018)),
            origin=Origin(xyz=(-0.002, 0.0, 0.0)),
            material=material,
            name="button_shoulder",
        )
        model.articulation(
            f"{name}_slide",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(PANEL_X, y, 0.047)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0035),
        )

    _button("power_button", -0.068, red)
    _button("ready_button", 0.068, green)

    base.inertial = Inertial.from_geometry(
        Box((0.380, 0.300, 0.120)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.315, 0.270, 0.085)),
        mass=1.4,
        origin=Origin(xyz=(-0.150, 0.0, 0.030)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("browning_dial")
    power = object_model.get_part("power_button")
    ready = object_model.get_part("ready_button")
    hinge = object_model.get_articulation("lid_hinge")
    dial_joint = object_model.get_articulation("browning_dial_spin")
    power_slide = object_model.get_articulation("power_button_slide")
    ready_slide = object_model.get_articulation("ready_button_slide")

    ctx.check("lid uses rear revolute hinge", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("hinge axis is horizontal across width", tuple(hinge.axis) == (0.0, 1.0, 0.0))
    ctx.check(
        "lid opens to practical appliance angle",
        hinge.motion_limits is not None and hinge.motion_limits.upper is not None and hinge.motion_limits.upper > 1.2,
    )
    ctx.check("browning dial is continuous", dial_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("buttons are prismatic", power_slide.articulation_type == ArticulationType.PRISMATIC and ready_slide.articulation_type == ArticulationType.PRISMATIC)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.006,
            max_gap=0.018,
            name="closed cooking plates have realistic clearance",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.16,
            name="upper and lower waffle plates align",
        )
        closed_handle = ctx.part_element_world_aabb(lid, elem="front_handle")

    with ctx.pose({hinge: 1.10}):
        opened_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
    ctx.check(
        "lid hinge raises the front handle",
        closed_handle is not None
        and opened_handle is not None
        and float(opened_handle[0][2]) > float(closed_handle[1][2]) + 0.08,
        details=f"closed={closed_handle}, opened={opened_handle}",
    )

    for part_obj, elem, label in (
        (dial, "dial_cap", "browning dial"),
        (power, "button_cap", "power button"),
        (ready, "button_cap", "ready button"),
    ):
        ctx.expect_gap(
            base,
            part_obj,
            axis="x",
            positive_elem="front_panel",
            negative_elem=elem,
            max_gap=0.001,
            max_penetration=0.001,
            name=f"{label} seats on front panel",
        )
        ctx.expect_overlap(
            base,
            part_obj,
            axes="yz",
            elem_a="front_panel",
            elem_b=elem,
            min_overlap=0.020,
            name=f"{label} lies within panel area",
        )

    power_rest = ctx.part_world_position(power)
    ready_rest = ctx.part_world_position(ready)
    with ctx.pose({power_slide: 0.0035, ready_slide: 0.0035}):
        power_pressed = ctx.part_world_position(power)
        ready_pressed = ctx.part_world_position(ready)
    ctx.check(
        "front buttons push inward",
        power_rest is not None
        and ready_rest is not None
        and power_pressed is not None
        and ready_pressed is not None
        and power_pressed[0] > power_rest[0] + 0.003
        and ready_pressed[0] > ready_rest[0] + 0.003,
        details=f"power {power_rest}->{power_pressed}, ready {ready_rest}->{ready_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
