from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], corner_radius: float) -> cq.Workplane:
    """A gently radiused consumer-appliance box, centered at the local origin."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges("|Z")
        .fillet(corner_radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_size_panini_grill")

    dark_plastic = model.material("dark_insulated_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    black_plastic = model.material("black_handle_plastic", rgba=(0.005, 0.005, 0.006, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    grill_metal = model.material("dark_nonstick_grill_plate", rgba=(0.055, 0.058, 0.055, 1.0))
    warm_metal = model.material("warm_hinge_metal", rgba=(0.58, 0.56, 0.50, 1.0))
    white_print = model.material("white_temperature_print", rgba=(0.92, 0.90, 0.84, 1.0))

    # Overall scale: roughly 60 cm wide, 50 cm deep and 20 cm tall when closed.
    base_depth = 0.50
    base_width = 0.62
    base_height = 0.075
    lid_depth = 0.43
    lid_width = 0.57
    lid_height = 0.095
    hinge_x = -0.205
    hinge_z = 0.110

    base = model.part("base")
    base_shell = _rounded_box((base_depth, base_width, base_height), 0.030)
    base.visual(
        mesh_from_cadquery(base_shell, "base_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=dark_plastic,
        name="base_shell",
    )
    base.visual(
        Box((0.40, 0.50, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.081)),
        material=grill_metal,
        name="lower_plate",
    )
    for idx, y in enumerate([-0.195, -0.130, -0.065, 0.0, 0.065, 0.130, 0.195]):
        base.visual(
            Box((0.355, 0.014, 0.007)),
            origin=Origin(xyz=(0.025, y, 0.090)),
            material=grill_metal,
            name=f"lower_rib_{idx}",
        )
    # Four low rubber feet read as a countertop appliance rather than a bare box.
    for idx, (x, y) in enumerate(
        [(-0.175, -0.225), (-0.175, 0.225), (0.185, -0.225), (0.185, 0.225)]
    ):
        base.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(x, y, -0.004)),
            material=black_plastic,
            name=f"foot_{idx}",
        )

    # Rear hinge: two fixed outboard support barrels with brackets, leaving a
    # clear center span for the moving lid barrel.
    for idx, y in enumerate([-0.335, 0.335]):
        base.visual(
            Box((0.040, 0.050, 0.052)),
            origin=Origin(xyz=(hinge_x, y, 0.086)),
            material=dark_plastic,
            name=f"rear_hinge_bracket_{idx}",
        )
        base.visual(
            Cylinder(radius=0.017, length=0.090),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_metal,
            name=f"rear_hinge_barrel_{idx}",
        )
    base.visual(
        Cylinder(radius=0.006, length=0.690),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="rear_hinge_pin",
    )

    # Front catch that the small lid latch tab visually hooks over.
    base.visual(
        Box((0.030, 0.145, 0.028)),
        origin=Origin(xyz=(0.258, 0.0, 0.083)),
        material=warm_metal,
        name="front_latch_catch",
    )

    # Thermostat shaft and printed temperature marks on the side face.
    knob_x = 0.045
    knob_z = 0.056
    side_face_y = base_width / 2.0
    knob_joint_y = side_face_y + 0.030
    base.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(
            xyz=(knob_x, side_face_y + 0.015, knob_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="thermostat_shaft",
    )
    for idx, (dx, dz, h) in enumerate(
        [(-0.055, 0.009, 0.008), (-0.033, 0.014, 0.010), (0.0, 0.017, 0.012), (0.033, 0.014, 0.010), (0.055, 0.009, 0.008)]
    ):
        base.visual(
            Box((0.006, 0.006, h)),
            origin=Origin(xyz=(knob_x + dx, side_face_y - 0.001, knob_z + dz)),
            material=white_print,
            name=f"temperature_tick_{idx}",
        )

    lid = model.part("lid")
    lid_shell = _rounded_box((lid_depth, lid_width, lid_height), 0.026)
    lid.visual(
        mesh_from_cadquery(lid_shell, "lid_shell", tolerance=0.001),
        # The lid frame is exactly on the rear hinge axis; the closed shell
        # extends forward from the hinge line with clearance around the pin.
        origin=Origin(xyz=(lid_depth / 2.0 + 0.020, 0.0, 0.045)),
        material=dark_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((0.040, 0.360, 0.034)),
        origin=Origin(xyz=(0.020, 0.0, 0.023)),
        material=dark_plastic,
        name="rear_lid_web",
    )
    lid.visual(
        Box((0.335, 0.430, 0.006)),
        origin=Origin(xyz=(0.250, 0.0, 0.096)),
        material=stainless,
        name="top_stainless_panel",
    )
    lid.visual(
        Box((0.375, 0.500, 0.010)),
        origin=Origin(xyz=(0.225, 0.0, 0.003)),
        material=grill_metal,
        name="upper_plate",
    )
    for idx, y in enumerate([-0.195, -0.130, -0.065, 0.0, 0.065, 0.130, 0.195]):
        lid.visual(
            Box((0.340, 0.014, 0.007)),
            origin=Origin(xyz=(0.225, y, -0.004)),
            material=grill_metal,
            name=f"upper_rib_{idx}",
        )
    lid.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="rear_lid_barrel",
    )
    # Stout insulated carry handle across the lid.
    lid.visual(
        Cylinder(radius=0.014, length=0.480),
        origin=Origin(xyz=(0.285, 0.0, 0.132), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="top_handle_bar",
    )
    for idx, y in enumerate([-0.205, 0.205]):
        lid.visual(
            Cylinder(radius=0.012, length=0.045),
            origin=Origin(xyz=(0.285, y, 0.108)),
            material=black_plastic,
            name=f"top_handle_post_{idx}",
        )
    # Fixed outboard knuckles for the small front latch hinge.
    front_latch_x = lid_depth + 0.035
    for idx, y in enumerate([-0.083, 0.083]):
        lid.visual(
            Box((0.026, 0.034, 0.022)),
            origin=Origin(xyz=(front_latch_x - 0.010, y, 0.018)),
            material=dark_plastic,
            name=f"front_latch_ear_{idx}",
        )
        lid.visual(
            Cylinder(radius=0.0065, length=0.034),
            origin=Origin(xyz=(front_latch_x, y, 0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_metal,
            name=f"front_latch_knuckle_{idx}",
        )

    latch_tab = model.part("latch_tab")
    latch_tab.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="latch_hinge_barrel",
    )
    latch_tab.visual(
        Box((0.060, 0.135, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, -0.006)),
        material=black_plastic,
        name="latch_paddle",
    )
    latch_tab.visual(
        Box((0.012, 0.115, 0.030)),
        origin=Origin(xyz=(0.052, 0.0, -0.024)),
        material=black_plastic,
        name="latch_tooth",
    )

    thermostat_knob = model.part("thermostat_knob")
    knob_geometry = KnobGeometry(
        0.070,
        0.034,
        body_style="skirted",
        top_diameter=0.054,
        base_diameter=0.072,
        edge_radius=0.0012,
        grip=KnobGrip(style="fluted", count=22, depth=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0009, angle_deg=0.0),
        bore=KnobBore(style="round", diameter=0.010),
        center=False,
    )
    thermostat_knob.visual(
        mesh_from_geometry(knob_geometry, "thermostat_knob"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_body",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        # Closed lid geometry extends along local +X; -Y makes positive q open upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch_tab,
        origin=Origin(xyz=(front_latch_x, 0.0, 0.018)),
        # The tab projects forward in +X at rest; +Y rotates it downward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "base_to_thermostat_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=thermostat_knob,
        origin=Origin(xyz=(knob_x, knob_joint_y, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_tab")
    knob = object_model.get_part("thermostat_knob")
    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_hinge = object_model.get_articulation("lid_to_latch")
    knob_joint = object_model.get_articulation("base_to_thermostat_knob")

    ctx.allow_overlap(
        base,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="rear_lid_barrel",
        reason="The fixed rear hinge pin is intentionally captured inside the moving lid barrel.",
    )
    ctx.expect_within(
        base,
        lid,
        axes="xz",
        inner_elem="rear_hinge_pin",
        outer_elem="rear_lid_barrel",
        margin=0.001,
        name="rear hinge pin sits concentrically inside the lid barrel",
    )
    ctx.expect_overlap(
        base,
        lid,
        axes="y",
        elem_a="rear_hinge_pin",
        elem_b="rear_lid_barrel",
        min_overlap=0.30,
        name="rear hinge pin spans through the moving barrel",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.30,
        name="closed grill plates overlap like a clamshell press",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.004,
        max_gap=0.030,
        name="closed plates have a small food gap",
    )
    ctx.expect_gap(
        knob,
        base,
        axis="y",
        positive_elem="knob_body",
        negative_elem="thermostat_shaft",
        max_gap=0.003,
        max_penetration=0.001,
        name="thermostat knob seats on the side shaft",
    )
    ctx.expect_overlap(
        knob,
        base,
        axes="xz",
        elem_a="knob_body",
        elem_b="thermostat_shaft",
        min_overlap=0.010,
        name="knob is centered over the shaft",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens the thick lid upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.12,
        details=f"rest_lid_aabb={rest_lid_aabb}, open_lid_aabb={open_lid_aabb}",
    )

    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_hinge: 1.00}):
        released_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "front latch tab rotates downward to release",
        rest_latch_aabb is not None
        and released_latch_aabb is not None
        and released_latch_aabb[0][2] < rest_latch_aabb[0][2] - 0.025,
        details=f"rest_latch_aabb={rest_latch_aabb}, released_latch_aabb={released_latch_aabb}",
    )

    ctx.check(
        "thermostat knob is continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={knob_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
