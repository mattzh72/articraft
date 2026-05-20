from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    radius: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges().fillet(radius)
    return shape.translate(center)


def _bowl_shell() -> LatheGeometry:
    # A true open metal shell: large radius, thick foot, and rolled lip.
    return LatheGeometry.from_shell_profiles(
        [
            (0.046, 0.000),
            (0.070, 0.018),
            (0.104, 0.100),
            (0.136, 0.202),
            (0.153, 0.235),
        ],
        [
            (0.018, 0.021),
            (0.052, 0.038),
            (0.094, 0.103),
            (0.120, 0.202),
            (0.133, 0.222),
        ],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )


def _carriage_stage() -> cq.Workplane:
    plate = _rounded_box((0.225, 0.245, 0.024), center=(0.0, 0.0, 0.012), radius=0.010)
    return plate


def _head_shell() -> cq.Workplane:
    body = _rounded_box((0.405, 0.235, 0.170), center=(0.258, 0.0, -0.020), radius=0.045)
    nose_weight = _rounded_box((0.115, 0.210, 0.145), center=(0.415, 0.0, -0.018), radius=0.038)
    rear_boss = _rounded_box((0.085, 0.205, 0.125), center=(0.078, 0.0, -0.005), radius=0.030)
    return body.union(nose_weight).union(rear_boss)


def _flat_beater() -> cq.Workplane:
    outer_loop = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.066, -0.044),
                (-0.063, -0.092),
                (-0.047, -0.132),
                (-0.020, -0.166),
                (0.000, -0.178),
                (0.020, -0.166),
                (0.047, -0.132),
                (0.063, -0.092),
                (0.066, -0.044),
                (0.033, -0.025),
                (0.000, -0.019),
                (-0.033, -0.025),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((0.0, -0.006, 0.0))
    )
    inner_opening = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.038, -0.058),
                (-0.036, -0.099),
                (-0.025, -0.129),
                (0.000, -0.150),
                (0.025, -0.129),
                (0.036, -0.099),
                (0.038, -0.058),
                (0.017, -0.048),
                (0.000, -0.045),
                (-0.017, -0.048),
            ]
        )
        .close()
        .extrude(0.030)
        .translate((0.0, -0.015, 0.0))
    )
    loop = outer_loop.cut(inner_opening)
    center_spine = _rounded_box((0.018, 0.014, 0.125), center=(0.0, 0.0, -0.102), radius=0.004)
    top_shaft = cq.Workplane("XY").circle(0.008).extrude(0.062).translate((0.0, 0.0, -0.062))
    collar = cq.Workplane("XY").circle(0.018).extrude(0.024).translate((0.0, 0.0, -0.079))
    return loop.union(center_spine).union(top_shaft).union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workhorse_tilt_head_stand_mixer")

    enamel = model.material("brushed_red_enamel", rgba=(0.58, 0.06, 0.035, 1.0))
    dark_metal = model.material("dark_machined_metal", rgba=(0.07, 0.075, 0.080, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.78, 0.74, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(
            _rounded_box((0.620, 0.360, 0.090), center=(0.165, 0.0, 0.045), radius=0.025),
            "base_plate",
            tolerance=0.0015,
        ),
        material=enamel,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(
            _rounded_box((0.190, 0.250, 0.235), center=(-0.105, 0.0, 0.2075), radius=0.035),
            "pedestal_shell",
            tolerance=0.0015,
        ),
        material=enamel,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.245, 0.026, 0.014)),
        origin=Origin(xyz=(0.175, -0.160, 0.095)),
        material=dark_metal,
        name="front_track",
    )
    base.visual(
        Box((0.245, 0.026, 0.014)),
        origin=Origin(xyz=(0.175, 0.160, 0.095)),
        material=dark_metal,
        name="rear_track",
    )
    base.visual(
        Box((0.060, 0.045, 0.180)),
        origin=Origin(xyz=(-0.137, -0.145, 0.365)),
        material=enamel,
        name="hinge_post_0",
    )
    base.visual(
        Box((0.060, 0.045, 0.180)),
        origin=Origin(xyz=(-0.137, 0.145, 0.365)),
        material=enamel,
        name="hinge_post_1",
    )
    base.visual(
        Box((0.072, 0.038, 0.090)),
        origin=Origin(xyz=(-0.137, -0.145, 0.450)),
        material=enamel,
        name="hinge_ear_0",
    )
    base.visual(
        Box((0.072, 0.038, 0.090)),
        origin=Origin(xyz=(-0.137, 0.145, 0.450)),
        material=enamel,
        name="hinge_ear_1",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(-0.137, -0.158, 0.462), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_pin_0",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(-0.137, 0.158, 0.462), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_pin_1",
    )
    base.visual(
        Box((0.075, 0.004, 0.022)),
        origin=Origin(xyz=(0.100, 0.181, 0.065)),
        material=black,
        name="lock_slot",
    )

    stage = model.part("bowl_stage")
    stage.visual(
        mesh_from_cadquery(_carriage_stage(), "bowl_stage", tolerance=0.001),
        material=dark_metal,
        name="stage_plate",
    )
    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_bowl_shell(), "bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "head_shell", tolerance=0.0015),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.245),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.014),
        origin=Origin(xyz=(0.320, 0.0, -0.106)),
        material=dark_metal,
        name="drive_plate",
    )

    beater = model.part("flat_beater")
    beater.visual(
        mesh_from_cadquery(_flat_beater(), "flat_beater", tolerance=0.0008),
        material=stainless,
        name="beater_frame",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.010, 0.008, 0.046)),
        origin=Origin(xyz=(0.0, -0.033, 0.000)),
        material=dark_metal,
        name="knob_grip",
    )

    lock_slider = model.part("lock_slider")
    lock_slider.visual(
        Box((0.052, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=black,
        name="slider_tab",
    )
    lock_slider.visual(
        Box((0.018, 0.014, 0.032)),
        origin=Origin(xyz=(-0.018, 0.026, 0.0)),
        material=dark_metal,
        name="slider_rib",
    )

    model.articulation(
        "base_to_bowl_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.180, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=-0.035, upper=0.055),
    )
    model.articulation(
        "stage_to_bowl",
        ArticulationType.FIXED,
        parent=stage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.137, 0.0, 0.462)),
        # The dense head extends forward along local +X, so -Y lifts the nose.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.9, lower=0.0, upper=math.radians(62.0)),
    )
    model.articulation(
        "head_to_flat_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.320, 0.0, -0.113)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=32.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.072, -0.125, 0.275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=math.radians(285.0)),
    )
    model.articulation(
        "base_to_lock_slider",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_slider,
        origin=Origin(xyz=(0.100, 0.183, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=-0.018, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage = object_model.get_part("bowl_stage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("flat_beater")
    speed_knob = object_model.get_part("speed_knob")
    lock_slider = object_model.get_part("lock_slider")

    stage_slide = object_model.get_articulation("base_to_bowl_stage")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_flat_beater")
    speed_turn = object_model.get_articulation("base_to_speed_knob")
    lock_slide = object_model.get_articulation("base_to_lock_slider")

    ctx.check("bowl rides on prismatic carriage", stage_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("head uses rear tilt hinge", head_tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("beater has continuous spin", beater_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("speed knob is revolute", speed_turn.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lock control is prismatic", lock_slide.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_gap(
        stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_plate",
        negative_elem="base_plate",
        name="carriage stage sits on base plate",
    )
    ctx.expect_contact(
        bowl,
        stage,
        elem_a="bowl_shell",
        elem_b="stage_plate",
        contact_tol=0.001,
        name="bowl foot is seated on the stage",
    )
    ctx.expect_overlap(
        beater,
        bowl,
        axes="xy",
        min_overlap=0.030,
        elem_a="beater_frame",
        elem_b="bowl_shell",
        name="flat beater is centered over bowl footprint",
    )

    rest_stage_pos = ctx.part_world_position(stage)
    rest_bowl_pos = ctx.part_world_position(bowl)
    rest_beater_pos = ctx.part_world_position(beater)
    rest_lock_pos = ctx.part_world_position(lock_slider)
    with ctx.pose({stage_slide: 0.055, head_tilt: math.radians(50.0), lock_slide: 0.018}):
        moved_stage_pos = ctx.part_world_position(stage)
        moved_bowl_pos = ctx.part_world_position(bowl)
        tilted_beater_pos = ctx.part_world_position(beater)
        moved_lock_pos = ctx.part_world_position(lock_slider)

    ctx.check(
        "carriage translates bowl forward",
        rest_stage_pos is not None
        and moved_stage_pos is not None
        and rest_bowl_pos is not None
        and moved_bowl_pos is not None
        and moved_stage_pos[0] > rest_stage_pos[0] + 0.045
        and moved_bowl_pos[0] > rest_bowl_pos[0] + 0.045,
        details=f"rest_stage={rest_stage_pos}, moved_stage={moved_stage_pos}, rest_bowl={rest_bowl_pos}, moved_bowl={moved_bowl_pos}",
    )
    ctx.check(
        "positive head tilt raises beater",
        rest_beater_pos is not None
        and tilted_beater_pos is not None
        and tilted_beater_pos[2] > rest_beater_pos[2] + 0.12,
        details=f"rest={rest_beater_pos}, tilted={tilted_beater_pos}",
    )
    ctx.check(
        "lock slider travels along base",
        rest_lock_pos is not None
        and moved_lock_pos is not None
        and moved_lock_pos[0] > rest_lock_pos[0] + 0.012,
        details=f"rest={rest_lock_pos}, moved={moved_lock_pos}",
    )

    return ctx.report()


object_model = build_object_model()
