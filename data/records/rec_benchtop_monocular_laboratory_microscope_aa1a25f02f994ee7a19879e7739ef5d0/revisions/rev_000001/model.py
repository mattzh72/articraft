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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_monocular_microscope")

    body_paint = model.material("warm_off_white", rgba=(0.82, 0.82, 0.76, 1.0))
    dark_metal = model.material("matte_black_metal", rgba=(0.015, 0.017, 0.018, 1.0))
    black = model.material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.70, 0.72, 0.72, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.82, 0.95, 0.55))

    optical_pitch = -math.radians(30.0)
    optical_rpy = (0.0, optical_pitch, 0.0)
    optical_axis = (-0.5, 0.0, math.cos(math.radians(30.0)))
    turret_center = (0.025, 0.0, 0.255)

    def along_axis(distance: float) -> tuple[float, float, float]:
        return (
            turret_center[0] + optical_axis[0] * distance,
            turret_center[1] + optical_axis[1] * distance,
            turret_center[2] + optical_axis[2] * distance,
        )

    stand = model.part("stand")

    base_shape = (
        cq.Workplane("XY")
        .box(0.22, 0.16, 0.055)
        .edges("|Z")
        .fillet(0.009)
    )
    stand.visual(
        mesh_from_cadquery(base_shape, "base_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=body_paint,
        name="base_housing",
    )

    lamp_shape = (
        cq.Workplane("XY")
        .box(0.112, 0.108, 0.040)
        .edges("|Z")
        .fillet(0.006)
    )
    stand.visual(
        mesh_from_cadquery(lamp_shape, "illuminator_compartment"),
        origin=Origin(xyz=(0.065, 0.0, 0.075)),
        material=body_paint,
        name="illuminator_compartment",
    )
    stand.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.068, 0.0, 0.097)),
        material=glass,
        name="lamp_window",
    )

    arm_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.095, 0.050),
                (-0.050, 0.050),
                (-0.034, 0.130),
                (-0.018, 0.215),
                (-0.020, 0.276),
                (0.000, 0.260),
                (-0.008, 0.230),
                (-0.028, 0.188),
                (-0.006, 0.110),
                (-0.020, 0.058),
            ]
        )
        .close()
        .extrude(0.046, both=True)
    )
    stand.visual(
        mesh_from_cadquery(arm_profile, "rising_arm"),
        material=body_paint,
        name="rising_arm",
    )

    stand.visual(
        Box((0.035, 0.070, 0.050)),
        origin=Origin(xyz=(-0.032, 0.0, 0.255), rpy=(0.0, -0.20, 0.0)),
        material=body_paint,
        name="head_housing",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=along_axis(0.008), rpy=optical_rpy),
        material=dark_metal,
        name="nosepiece_socket",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=along_axis(0.086), rpy=optical_rpy),
        material=dark_metal,
        name="inclined_tube",
    )
    stand.visual(
        Cylinder(radius=0.023, length=0.043),
        origin=Origin(xyz=along_axis(0.165), rpy=optical_rpy),
        material=black,
        name="eyepiece",
    )

    stand.visual(
        Box((0.095, 0.055, 0.032)),
        origin=Origin(xyz=(0.065, 0.0, 0.111)),
        material=body_paint,
        name="stage_guide",
    )
    stand.visual(
        Box((0.082, 0.011, 0.008)),
        origin=Origin(xyz=(0.065, -0.029, 0.131)),
        material=chrome,
        name="guide_rail_0",
    )
    stand.visual(
        Box((0.082, 0.011, 0.008)),
        origin=Origin(xyz=(0.065, 0.029, 0.131)),
        material=chrome,
        name="guide_rail_1",
    )

    stage_plate_shape = (
        cq.Workplane("XY")
        .box(0.120, 0.110, 0.006)
        .faces(">Z")
        .workplane()
        .circle(0.019)
        .cutThruAll()
        .edges("|Z")
        .fillet(0.002)
    )
    stage_support = model.part("stage_support")
    stage_support.visual(
        Box((0.130, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="stage_slider",
    )
    stage_support.visual(
        Box((0.022, 0.020, 0.038)),
        origin=Origin(xyz=(-0.035, 0.0, 0.029)),
        material=dark_metal,
        name="stage_post_0",
    )
    stage_support.visual(
        Box((0.022, 0.020, 0.038)),
        origin=Origin(xyz=(0.035, 0.0, 0.029)),
        material=dark_metal,
        name="stage_post_1",
    )
    stage_support.visual(
        mesh_from_cadquery(stage_plate_shape, "square_stage_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_metal,
        name="stage_plate",
    )
    stage_support.visual(
        Box((0.095, 0.010, 0.004)),
        origin=Origin(xyz=(0.002, 0.043, 0.0525)),
        material=chrome,
        name="slide_clip_0",
    )
    stage_support.visual(
        Box((0.095, 0.010, 0.004)),
        origin=Origin(xyz=(0.002, -0.043, 0.0525)),
        material=chrome,
        name="slide_clip_1",
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_support,
        origin=Origin(xyz=(0.065, 0.0, 0.127)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.08, lower=-0.025, upper=0.035),
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=dark_metal,
        name="turret_disk",
    )
    for i, (angle_deg, radius, length) in enumerate(
        ((0.0, 0.0065, 0.052), (120.0, 0.0085, 0.057), (240.0, 0.0075, 0.050))
    ):
        a = math.radians(angle_deg)
        x = 0.022 * math.cos(a)
        y = 0.022 * math.sin(a)
        nosepiece.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, -0.044)),
            material=chrome,
            name=f"objective_{i}",
        )
        nosepiece.visual(
            Cylinder(radius=radius * 1.06, length=0.004),
            origin=Origin(xyz=(x, y, -0.020)),
            material=black,
            name=f"objective_band_{i}",
        )
    model.articulation(
        "nosepiece_turret",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=nosepiece,
        origin=Origin(xyz=turret_center, rpy=optical_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    lamp_door = model.part("lamp_door")
    lamp_door.visual(
        Box((0.070, 0.004, 0.035)),
        origin=Origin(xyz=(0.035, -0.0005, 0.0)),
        material=body_paint,
        name="door_panel",
    )
    lamp_door.visual(
        Cylinder(radius=0.004, length=0.013),
        origin=Origin(xyz=(0.0, -0.0025, 0.011)),
        material=chrome,
        name="hinge_barrel_0",
    )
    lamp_door.visual(
        Cylinder(radius=0.004, length=0.013),
        origin=Origin(xyz=(0.0, -0.0025, -0.011)),
        material=chrome,
        name="hinge_barrel_1",
    )
    lamp_door.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.061, -0.0035, 0.0)),
        material=black,
        name="door_pull",
    )
    model.articulation(
        "lamp_door_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lamp_door,
        origin=Origin(xyz=(0.020, -0.0555, 0.075)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage_support = object_model.get_part("stage_support")
    nosepiece = object_model.get_part("nosepiece")
    lamp_door = object_model.get_part("lamp_door")
    stage_slide = object_model.get_articulation("stage_slide")
    turret = object_model.get_articulation("nosepiece_turret")
    door_hinge = object_model.get_articulation("lamp_door_hinge")

    ctx.check(
        "microscope has the three requested moving mechanisms",
        stage_slide is not None and turret is not None and door_hinge is not None,
        details="Expected stage prismatic slide, objective turret, and lamp-door hinge.",
    )
    ctx.allow_overlap(
        nosepiece,
        stand,
        elem_a="turret_disk",
        elem_b="nosepiece_socket",
        reason="The rotating objective turret is intentionally seated on a short hidden spindle/collar at the optical axis.",
    )
    ctx.expect_overlap(
        nosepiece,
        stand,
        axes="xy",
        elem_a="turret_disk",
        elem_b="nosepiece_socket",
        min_overlap=0.012,
        name="turret disk is captured by the optical-axis socket",
    )

    ctx.expect_contact(
        stage_support,
        stand,
        elem_a="stage_slider",
        elem_b="stage_guide",
        contact_tol=0.0015,
        name="stage slider rides on guide block",
    )
    ctx.expect_overlap(
        stage_support,
        stand,
        axes="xy",
        elem_a="stage_slider",
        elem_b="stage_guide",
        min_overlap=0.025,
        name="stage slider is retained on guide block",
    )
    with ctx.pose({stage_slide: 0.035}):
        ctx.expect_overlap(
            stage_support,
            stand,
            axes="xy",
            elem_a="stage_slider",
            elem_b="stage_guide",
            min_overlap=0.025,
            name="advanced stage remains on guide block",
        )
        advanced_stage = ctx.part_world_position(stage_support)
    with ctx.pose({stage_slide: -0.025}):
        retracted_stage = ctx.part_world_position(stage_support)
    ctx.check(
        "stage support translates fore and aft",
        advanced_stage is not None
        and retracted_stage is not None
        and advanced_stage[0] > retracted_stage[0] + 0.050,
        details=f"retracted={retracted_stage}, advanced={advanced_stage}",
    )

    with ctx.pose({turret: 0.0}):
        rest_nose = ctx.part_world_position(nosepiece)
    with ctx.pose({turret: math.pi / 2.0}):
        turned_nose = ctx.part_world_position(nosepiece)
    ctx.check(
        "objective turret rotates about fixed optical-axis origin",
        rest_nose is not None
        and turned_nose is not None
        and abs(rest_nose[0] - turned_nose[0]) < 0.001
        and abs(rest_nose[1] - turned_nose[1]) < 0.001
        and abs(rest_nose[2] - turned_nose[2]) < 0.001,
        details=f"rest={rest_nose}, turned={turned_nose}",
    )

    ctx.expect_gap(
        stand,
        lamp_door,
        axis="y",
        positive_elem="illuminator_compartment",
        negative_elem="door_panel",
        min_gap=0.0,
        max_gap=0.006,
        name="closed lamp door sits just outside side compartment",
    )
    closed_aabb = ctx.part_world_aabb(lamp_door)
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(lamp_door)
    ctx.check(
        "lamp compartment door swings outward on side hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.020,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
