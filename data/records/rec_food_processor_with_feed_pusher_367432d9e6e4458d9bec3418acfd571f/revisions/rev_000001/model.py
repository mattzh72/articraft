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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_TOP_Z = 0.22
BOWL_HEIGHT = 0.235
LID_THICKNESS = 0.018
CHUTE_X = 0.055
CHUTE_Y = -0.040
CHUTE_HEIGHT = 0.160
PUSHER_TRAVEL = 0.075


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Vertical annular solid from z=0 to z=height."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate((0, 0, -0.002))
    return outer.cut(bore)


def _motor_base_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(0.42, 0.34, 0.20)
        .edges("|Z")
        .fillet(0.030)
        .translate((0.0, 0.0, 0.100))
    )
    raised_deck = cq.Workplane("XY").cylinder(0.030, 0.115).translate((0, 0, 0.205))
    return shell.union(raised_deck)


def _bowl_shape() -> cq.Workplane:
    outer_radius = 0.128
    inner_radius = 0.116
    bottom_thickness = 0.018
    shaft_clearance = 0.032

    body = cq.Workplane("XY").circle(outer_radius).extrude(BOWL_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(BOWL_HEIGHT - bottom_thickness + 0.008)
        .translate((0, 0, bottom_thickness))
    )
    center_hole = (
        cq.Workplane("XY")
        .circle(shaft_clearance)
        .extrude(bottom_thickness + 0.018)
        .translate((0, 0, -0.006))
    )
    body = body.cut(cavity).cut(center_hole)

    top_rim = _ring(0.136, inner_radius, 0.012).translate((0, 0, BOWL_HEIGHT - 0.012))
    bottom_lock_ring = _ring(0.090, shaft_clearance, 0.016)
    body = body.union(top_rim).union(bottom_lock_ring)

    # A large transparent side handle, fused into the bowl wall at two bosses.
    handle_grip = cq.Workplane("XY").box(0.024, 0.038, 0.140).translate((0.184, 0.0, 0.125))
    handle_top = cq.Workplane("XY").box(0.070, 0.038, 0.026).translate((0.153, 0.0, 0.188))
    handle_bottom = cq.Workplane("XY").box(0.070, 0.038, 0.026).translate((0.153, 0.0, 0.066))
    return body.union(handle_grip).union(handle_top).union(handle_bottom)


def _lid_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(0.142).extrude(LID_THICKNESS)
    feed_hole = (
        cq.Workplane("XY")
        .center(CHUTE_X, CHUTE_Y)
        .circle(0.038)
        .extrude(LID_THICKNESS + 0.010)
        .translate((0, 0, -0.004))
    )
    disk = disk.cut(feed_hole)

    lock_ear_0 = cq.Workplane("XY").box(0.034, 0.018, 0.012).translate((0.126, 0.0, 0.004))
    lock_ear_1 = cq.Workplane("XY").box(0.034, 0.018, 0.012).translate((-0.126, 0.0, 0.004))
    return disk.union(lock_ear_0).union(lock_ear_1)


def _feed_chute_shape() -> cq.Workplane:
    tube = _ring(0.045, 0.037, CHUTE_HEIGHT)
    # Four short buttresses make the tall chute read as supported by the lid,
    # not as a freestanding pipe.
    for x, y, sx, sy in (
        (0.056, 0.0, 0.028, 0.014),
        (-0.056, 0.0, 0.028, 0.014),
        (0.0, 0.056, 0.014, 0.028),
        (0.0, -0.056, 0.014, 0.028),
    ):
        rib = cq.Workplane("XY").box(sx, sy, 0.055).translate((x, y, 0.0275))
        tube = tube.union(rib)
    top_lip = _ring(0.050, 0.037, 0.010).translate((0, 0, CHUTE_HEIGHT - 0.010))
    return tube.union(top_lip)


def _pusher_shape() -> cq.Workplane:
    sleeve_height = CHUTE_HEIGHT
    outer = cq.Workplane("XY").circle(0.030).extrude(sleeve_height)
    hollow = cq.Workplane("XY").circle(0.023).extrude(sleeve_height - 0.010).translate((0, 0, -0.002))
    sleeve = outer.cut(hollow)
    top_flange = cq.Workplane("XY").circle(0.047).extrude(0.016).translate((0, 0, sleeve_height))
    finger_cap = cq.Workplane("XY").cylinder(0.028, 0.026).translate((0, 0, sleeve_height + 0.030))
    return sleeve.union(top_flange).union(finger_cap)


def _blade_rotor_shape() -> cq.Workplane:
    hub = _ring(0.028, 0.014, 0.050).translate((0, 0, -0.025))
    blade_a = (
        cq.Workplane("XY")
        .polyline([(0.024, -0.010), (0.098, -0.026), (0.104, -0.006), (0.034, 0.014)])
        .close()
        .extrude(0.006)
        .translate((0, 0, 0.002))
    )
    blade_b = blade_a.rotate((0, 0, 0), (0, 0, 1), 180).translate((0, 0, -0.010))
    return hub.union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_food_processor")

    brushed_metal = Material("brushed_metal", rgba=(0.62, 0.64, 0.65, 1.0))
    dark_plastic = Material("dark_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    clear_bowl = Material("clear_bowl", rgba=(0.72, 0.90, 1.0, 0.36))
    clear_lid = Material("clear_lid", rgba=(0.75, 0.88, 1.0, 0.42))
    frosted_pusher = Material("frosted_pusher", rgba=(0.90, 0.96, 1.0, 0.62))
    stainless = Material("stainless_blade", rgba=(0.86, 0.88, 0.90, 1.0))
    red = Material("red_pulse", rgba=(0.74, 0.07, 0.045, 1.0))

    base = model.part("motor_base")
    base.visual(
        mesh_from_cadquery(_motor_base_shape(), "motor_base_shell", tolerance=0.0015),
        material=brushed_metal,
        name="base_shell",
    )
    base.visual(
        Box((0.260, 0.010, 0.092)),
        origin=Origin(xyz=(0.0, -0.168, 0.098)),
        material=dark_plastic,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - 0.010)),
        material=dark_plastic,
        name="drive_socket",
    )
    for x in (-0.150, 0.150):
        for y in (-0.115, 0.115):
            base.visual(
                Cylinder(radius=0.030, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=black_rubber,
                name=f"rubber_foot_{x}_{y}",
            )

    bowl = model.part("work_bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shape(), "work_bowl_shell", tolerance=0.0012),
        material=clear_bowl,
        name="bowl_shell",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0),
    )

    center_shaft = model.part("center_shaft")
    center_shaft.visual(
        Cylinder(radius=0.014, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=dark_plastic,
        name="vertical_shaft",
    )
    center_shaft.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_plastic,
        name="shaft_coupler",
    )
    model.articulation(
        "base_to_shaft",
        ArticulationType.FIXED,
        parent=base,
        child=center_shaft,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
    )

    blade_hub = model.part("blade_hub")
    blade_hub.visual(
        mesh_from_cadquery(_blade_rotor_shape(), "blade_rotor", tolerance=0.0008),
        material=stainless,
        name="blade_rotor",
    )
    model.articulation(
        "shaft_to_blade",
        ArticulationType.CONTINUOUS,
        parent=center_shaft,
        child=blade_hub,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "locking_lid_shell", tolerance=0.0012),
        material=clear_lid,
        name="lid_shell",
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BOWL_HEIGHT)),
    )

    feed_chute = model.part("feed_chute")
    feed_chute.visual(
        mesh_from_cadquery(_feed_chute_shape(), "feed_chute_tube", tolerance=0.001),
        material=clear_lid,
        name="chute_tube",
    )
    model.articulation(
        "lid_to_chute",
        ArticulationType.FIXED,
        parent=lid,
        child=feed_chute,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, LID_THICKNESS)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_pusher_shape(), "pusher_sleeve", tolerance=0.001),
        material=frosted_pusher,
        name="pusher_sleeve",
    )
    model.articulation(
        "chute_to_pusher",
        ArticulationType.PRISMATIC,
        parent=feed_chute,
        child=pusher,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=PUSHER_TRAVEL),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.064,
                0.030,
                body_style="skirted",
                base_diameter=0.070,
                top_diameter=0.052,
                edge_radius=0.002,
                grip=KnobGrip(style="fluted", count=24, depth=0.002),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "speed_selector_knob",
        ),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_cap",
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.173, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    for index, x in enumerate((-0.078, 0.078)):
        button = model.part(f"pulse_button_{index}")
        button.visual(
            Box((0.044, 0.016, 0.030)),
            origin=Origin(xyz=(0.0, -0.0085, 0.0)),
            material=red,
            name="button_cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.171, 0.105)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=0.010),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("work_bowl")
    lid = object_model.get_part("lid")
    chute = object_model.get_part("feed_chute")
    pusher = object_model.get_part("pusher")
    blade = object_model.get_part("blade_hub")
    knob = object_model.get_part("speed_knob")
    base = object_model.get_part("motor_base")

    bowl_twist = object_model.get_articulation("base_to_bowl")
    blade_spin = object_model.get_articulation("shaft_to_blade")
    knob_spin = object_model.get_articulation("base_to_knob")
    pusher_slide = object_model.get_articulation("chute_to_pusher")

    ctx.allow_overlap(
        chute,
        pusher,
        elem_a="chute_tube",
        elem_b="pusher_sleeve",
        reason=(
            "The pusher is intentionally nested in the clear hollow feed chute; "
            "the mesh collision proxy reports their retained sleeve-in-tube fit as overlap."
        ),
    )
    ctx.allow_overlap(
        "center_shaft",
        blade,
        elem_a="vertical_shaft",
        elem_b="blade_rotor",
        reason=(
            "The blade hub is intentionally captured on the drive shaft; "
            "a tiny represented bore/shaft interference keeps the rotating hub supported."
        ),
    )

    ctx.check(
        "bowl uses continuous locking twist",
        bowl_twist.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={bowl_twist.articulation_type}",
    )
    ctx.check(
        "blade hub rotates continuously",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_spin.articulation_type}",
    )
    ctx.check(
        "front speed selector is continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    ctx.expect_contact(lid, bowl, elem_a="lid_shell", elem_b="bowl_shell", contact_tol=0.002, name="lid sits on bowl rim")
    ctx.expect_contact(chute, lid, elem_a="chute_tube", elem_b="lid_shell", contact_tol=0.002, name="feed chute is supported by lid")
    ctx.expect_contact(
        blade,
        "center_shaft",
        elem_a="blade_rotor",
        elem_b="vertical_shaft",
        contact_tol=0.001,
        name="blade hub is captured on shaft",
    )
    ctx.expect_within(blade, bowl, axes="xy", inner_elem="blade_rotor", outer_elem="bowl_shell", margin=0.010, name="blade fits within clear bowl")
    ctx.expect_gap(base, knob, axis="y", min_gap=-0.001, max_gap=0.010, name="speed knob is mounted on front face")

    ctx.expect_within(
        pusher,
        chute,
        axes="xy",
        inner_elem="pusher_sleeve",
        outer_elem="chute_tube",
        margin=0.003,
        name="pusher sleeve is centered inside chute",
    )
    ctx.expect_overlap(
        pusher,
        chute,
        axes="z",
        elem_a="pusher_sleeve",
        elem_b="chute_tube",
        min_overlap=0.120,
        name="seated pusher remains inserted in chute",
    )

    rest_position = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: PUSHER_TRAVEL}):
        ctx.expect_within(
            pusher,
            chute,
            axes="xy",
            inner_elem="pusher_sleeve",
            outer_elem="chute_tube",
            margin=0.003,
            name="raised pusher stays guided by chute",
        )
        ctx.expect_overlap(
            pusher,
            chute,
            axes="z",
            elem_a="pusher_sleeve",
            elem_b="chute_tube",
            min_overlap=0.075,
            name="raised pusher retains insertion",
        )
        raised_position = ctx.part_world_position(pusher)

    ctx.check(
        "pusher slides upward",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + PUSHER_TRAVEL * 0.9,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
