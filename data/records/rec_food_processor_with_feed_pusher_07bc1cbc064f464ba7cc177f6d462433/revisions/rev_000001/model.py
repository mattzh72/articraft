from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


FRONT_Y = -1.0


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused appliance housing, authored about the origin."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _bowl_shell() -> LatheGeometry:
    """Transparent narrow processor bowl: a visibly thin shell with a drive hole."""
    outer_profile = (
        (0.064, 0.000),
        (0.076, 0.018),
        (0.084, 0.216),
        (0.088, 0.230),
    )
    inner_profile = (
        (0.025, 0.000),
        (0.052, 0.014),
        (0.075, 0.216),
        (0.079, 0.230),
    )
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _lid_shell() -> cq.Workplane:
    """Rear-hinged transparent lid with hollow feed tube and hollow hinge barrel."""
    # Child part frame is the hinge axis.  The bowl center is at local y=-0.120.
    top_disc = (
        cq.Workplane("XY")
        .circle(0.098)
        .extrude(0.016)
        .translate((0.0, -0.120, 0.006))
    )
    feed_hole = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.060)
        .translate((0.0, -0.155, -0.010))
    )
    skirt = _annular_cylinder(0.105, 0.092, 0.020).translate((0.0, -0.120, -0.013))
    feed_tube = _annular_cylinder(0.038, 0.028, 0.118).translate((0.0, -0.155, 0.021))
    hinge_leaf = cq.Workplane("XY").box(0.074, 0.032, 0.012).translate((0.0, -0.014, 0.005))
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.010)
        .circle(0.0055)
        .extrude(0.066)
        .translate((-0.033, 0.0, 0.0))
    )
    shell = top_disc.cut(feed_hole).union(skirt).union(feed_tube).union(hinge_leaf).union(hinge_barrel)
    return shell.edges("|Z").fillet(0.001)


def _blade_mesh() -> cq.Workplane:
    """A compact metal spindle with two swept-looking cutting paddles."""
    hub = cq.Workplane("XY").circle(0.016).extrude(0.110)
    lower_blade = (
        cq.Workplane("XY")
        .box(0.078, 0.016, 0.004)
        .translate((0.018, 0.000, 0.080))
        .rotate((0, 0, 0), (0, 0, 1), 13)
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.074, 0.015, 0.004)
        .translate((-0.017, 0.000, 0.098))
        .rotate((0, 0, 0), (0, 0, 1), 193)
    )
    cap = cq.Workplane("XY").circle(0.020).extrude(0.014).translate((0, 0, 0.108))
    return hub.union(lower_blade).union(upper_blade).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_food_processor")

    white_abs = model.material("warm_white_abs", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_panel = model.material("charcoal_panel", rgba=(0.035, 0.035, 0.040, 1.0))
    black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    button_mat = model.material("ivory_buttons", rgba=(0.93, 0.92, 0.86, 1.0))
    clear_plastic = model.material("smoky_clear_plastic", rgba=(0.64, 0.84, 0.96, 0.36))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box((0.310, 0.240, 0.120), 0.024), "rounded_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=white_abs,
        name="rounded_base",
    )
    base.visual(
        Box((0.205, 0.006, 0.076)),
        origin=Origin(xyz=(0.035, -0.1225, 0.064)),
        material=dark_panel,
        name="control_panel",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.086, 0.060, 0.017), "bowl_socket_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=dark_panel,
        name="bowl_socket_ring",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=black,
        name="drive_coupler",
    )
    base.visual(
        Box((0.076, 0.030, 0.235)),
        origin=Origin(xyz=(0.0, 0.122, 0.2375)),
        material=white_abs,
        name="rear_safety_tower",
    )
    base.visual(
        Box((0.170, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.120, 0.333)),
        material=white_abs,
        name="hinge_bridge",
    )
    for x in (-0.066, 0.066):
        base.visual(
            Box((0.032, 0.022, 0.054)),
            origin=Origin(xyz=(x, 0.120, 0.342)),
            material=white_abs,
            name=f"hinge_cheek_{0 if x < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.036),
            origin=Origin(xyz=(x, 0.120, 0.365), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white_abs,
            name=f"hinge_knuckle_{0 if x < 0 else 1}",
        )
    base.visual(
        Cylinder(radius=0.0045, length=0.185),
        origin=Origin(xyz=(0.0, 0.120, 0.365), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_bowl_shell(), "clear_bowl_shell"),
        material=clear_plastic,
        name="clear_bowl_shell",
    )
    bowl.visual(
        Box((0.020, 0.040, 0.150)),
        origin=Origin(xyz=(0.118, 0.000, 0.118)),
        material=clear_plastic,
        name="side_grip",
    )
    bowl.visual(
        Box((0.050, 0.034, 0.024)),
        origin=Origin(xyz=(0.098, 0.000, 0.188)),
        material=clear_plastic,
        name="upper_handle_mount",
    )
    bowl.visual(
        Box((0.048, 0.032, 0.024)),
        origin=Origin(xyz=(0.096, 0.000, 0.066)),
        material=clear_plastic,
        name="lower_handle_mount",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    spindle = model.part("blade_spindle")
    spindle.visual(
        mesh_from_cadquery(_blade_mesh(), "blade_spindle_mesh", tolerance=0.0006),
        material=steel,
        name="blade_spindle_mesh",
    )
    model.articulation(
        "base_to_blade_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=50.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "rear_hinged_lid", tolerance=0.0007),
        material=clear_plastic,
        name="rear_hinged_lid",
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.120, 0.365)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.024, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=black,
        name="cylindrical_plug",
    )
    pusher.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=black,
        name="top_cap",
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.155, 0.139)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=0.075),
    )

    speed_knob = model.part("speed_knob")
    knob_geom = KnobGeometry(
        0.052,
        0.030,
        body_style="skirted",
        top_diameter=0.041,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    speed_knob.visual(
        mesh_from_geometry(knob_geom, "speed_knob_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="speed_knob_cap",
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.070, -0.1255, 0.066)),
        axis=(0.0, FRONT_Y, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    for index, x in enumerate((0.045, 0.095)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.035, 0.016, 0.035)),
            origin=Origin(xyz=(0.0, -0.008, 0.0)),
            material=button_mat,
            name="square_button",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.1255, 0.066)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    spindle = object_model.get_part("blade_spindle")
    knob = object_model.get_part("speed_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_joint = object_model.get_articulation("base_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    spindle_joint = object_model.get_articulation("base_to_blade_spindle")
    button_joint_0 = object_model.get_articulation("base_to_button_0")
    button_joint_1 = object_model.get_articulation("base_to_button_1")

    ctx.allow_overlap(
        base,
        lid,
        elem_a="hinge_pin",
        elem_b="rear_hinged_lid",
        reason="The steel pin is intentionally captured through the lid hinge barrel.",
    )
    ctx.expect_overlap(
        base,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="rear_hinged_lid",
        min_overlap=0.060,
        name="hinge pin spans the lid barrel",
    )
    ctx.expect_within(
        base,
        lid,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="rear_hinged_lid",
        margin=0.003,
        name="hinge pin is captured inside the rear barrel envelope",
    )

    ctx.check(
        "primary mechanisms are articulated",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and spindle_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint_0.articulation_type == ArticulationType.PRISMATIC
        and button_joint_1.articulation_type == ArticulationType.PRISMATIC,
        details="Expected hinged lid, sliding pusher, spinning spindle/knob, and two independent push buttons.",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="cylindrical_plug",
        outer_elem="rear_hinged_lid",
        margin=0.0,
        name="pusher remains centered in the feed tube",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="cylindrical_plug",
        elem_b="rear_hinged_lid",
        min_overlap=0.080,
        name="pusher is retained in the feed tube at rest",
    )
    ctx.expect_within(
        spindle,
        bowl,
        axes="xy",
        inner_elem="blade_spindle_mesh",
        outer_elem="clear_bowl_shell",
        margin=0.010,
        name="blade spindle sits inside the narrow bowl footprint",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.15}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.055,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.070}):
        pusher_raised = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="cylindrical_plug",
            elem_b="rear_hinged_lid",
            min_overlap=0.035,
            name="raised pusher still retains insertion in the tube",
        )
    ctx.check(
        "pusher slides upward in the feed tube",
        pusher_rest is not None and pusher_raised is not None and pusher_raised[2] > pusher_rest[2] + 0.060,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.006}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_unpressed = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 presses inward independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_unpressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.004
        and abs(button_1_unpressed[1] - button_1_rest[1]) < 0.001,
        details=f"button0 rest/pressed={button_0_rest}/{button_0_pressed}, button1={button_1_rest}/{button_1_unpressed}",
    )

    with ctx.pose({button_joint_1: 0.006}):
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button_1 presses inward independently",
        button_1_rest is not None and button_1_pressed is not None and button_1_pressed[1] > button_1_rest[1] + 0.004,
        details=f"button1 rest/pressed={button_1_rest}/{button_1_pressed}",
    )

    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: math.pi}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "speed knob rotates about the front control axis",
        knob_rest is not None and knob_rotated is not None and abs(knob_rotated[1] - knob_rest[1]) < 0.001,
        details=f"knob positions should stay centered while rotating: {knob_rest}, {knob_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
