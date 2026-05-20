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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _stage_plate_mesh():
    """Rectangular microscope stage with a real central aperture."""
    return (
        cq.Workplane("XY")
        .box(0.150, 0.115, 0.014)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.018)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_monocular_microscope")

    black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark = model.material("dark_plastic", rgba=(0.05, 0.055, 0.060, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.88, 0.86, 1.0))
    glass = model.material("pale_glass", rgba=(0.74, 0.90, 1.0, 0.38))
    brass = model.material("objective_brass", rgba=(0.80, 0.58, 0.25, 1.0))
    lens_black = model.material("lens_black", rgba=(0.005, 0.005, 0.006, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.285, 0.185, 0.035)),
        origin=Origin(xyz=(0.000, 0.000, 0.0175)),
        material=black,
        name="base",
    )
    frame.visual(
        Box((0.180, 0.120, 0.010)),
        origin=Origin(xyz=(-0.010, 0.000, 0.040)),
        material=dark,
        name="base_plinth",
    )

    arm_path = [
        (-0.096, 0.000, 0.035),
        (-0.112, 0.000, 0.150),
        (-0.098, 0.000, 0.270),
        (-0.064, 0.000, 0.390),
        (-0.036, 0.000, 0.425),
    ]
    arm_geom = sweep_profile_along_spline(
        arm_path,
        profile=rounded_rect_profile(0.034, 0.026, radius=0.006),
        samples_per_segment=12,
        up_hint=(0.0, 1.0, 0.0),
        cap_profile=True,
    )
    frame.visual(
        mesh_from_geometry(arm_geom, "curved_arm"),
        material=black,
        name="curved_arm",
    )

    frame.visual(
        Box((0.028, 0.085, 0.240)),
        origin=Origin(xyz=(-0.055, 0.000, 0.305)),
        material=black,
        name="guide_column",
    )
    for y in (-0.050, 0.050):
        frame.visual(
            Cylinder(radius=0.0040, length=0.238),
            origin=Origin(xyz=(-0.038, y, 0.306)),
            material=chrome,
            name=f"guide_rail_{'n' if y < 0 else 'p'}",
        )

    frame.visual(
        Box((0.120, 0.045, 0.028)),
        origin=Origin(xyz=(-0.015, 0.000, 0.192)),
        material=black,
        name="stage_support",
    )
    frame.visual(
        mesh_from_cadquery(_stage_plate_mesh(), "stage_plate", tolerance=0.0008),
        origin=Origin(xyz=(0.030, 0.000, 0.207)),
        material=steel,
        name="stage_plate",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.030, 0.000, 0.181)),
        material=black,
        name="condenser_housing",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.030, 0.000, 0.205)),
        material=lens_black,
        name="stage_aperture_shadow",
    )
    frame.visual(
        Box((0.018, 0.040, 0.012)),
        origin=Origin(xyz=(0.030, -0.042, 0.181)),
        material=black,
        name="lever_boss_bridge",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.030, -0.052, 0.181)),
        material=chrome,
        name="lever_boss",
    )

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        Box((0.145, 0.030, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=chrome,
        name="cross_slide",
    )
    stage_carriage.visual(
        Box((0.070, 0.058, 0.010)),
        origin=Origin(xyz=(0.040, 0.032, 0.005)),
        material=chrome,
        name="projecting_carriage",
    )
    stage_carriage.visual(
        Box((0.090, 0.026, 0.003)),
        origin=Origin(xyz=(0.006, -0.015, 0.0095)),
        material=glass,
        name="glass_slide",
    )
    for x in (-0.038, 0.050):
        stage_carriage.visual(
            Box((0.020, 0.006, 0.011)),
            origin=Origin(xyz=(x, -0.018, 0.0105)),
            material=steel,
            name=f"slide_clip_{0 if x < 0 else 1}",
        )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.018, 0.082, 0.092)),
        origin=Origin(xyz=(0.009, 0.000, 0.046)),
        material=black,
        name="guide_saddle",
    )
    head_carriage.visual(
        Box((0.052, 0.070, 0.048)),
        origin=Origin(xyz=(0.041, 0.000, 0.063)),
        material=black,
        name="head_block",
    )
    head_carriage.visual(
        Box((0.070, 0.066, 0.042)),
        origin=Origin(xyz=(0.086, 0.000, 0.078)),
        material=black,
        name="optical_head",
    )
    head_carriage.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(0.075, 0.000, 0.138), rpy=(0.0, -0.40, 0.0)),
        material=black,
        name="monocular_tube",
    )
    head_carriage.visual(
        Cylinder(radius=0.021, length=0.035),
        origin=Origin(xyz=(0.053, 0.000, 0.191), rpy=(0.0, -0.40, 0.0)),
        material=dark,
        name="eyepiece",
    )
    head_carriage.visual(
        Cylinder(radius=0.024, length=0.025),
        origin=Origin(xyz=(0.081, 0.000, 0.0485)),
        material=chrome,
        name="nosepiece_mount",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(),
        material=chrome,
        name="turret_disk",
    )
    for i, angle in enumerate((0.0, 2.094, 4.188)):
        x = 0.022 * math.cos(angle)
        y = 0.022 * math.sin(angle)
        turret.visual(
            Cylinder(radius=0.0075, length=0.044),
            origin=Origin(xyz=(x, y, -0.028)),
            material=brass if i == 0 else black,
            name=f"objective_{i}",
        )
        turret.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, -0.009)),
            material=chrome,
            name=f"objective_collar_{i}",
        )

    diaphragm_lever = model.part("diaphragm_lever")
    diaphragm_lever.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material=chrome,
        name="pivot_cap",
    )
    diaphragm_lever.visual(
        Box((0.010, 0.072, 0.004)),
        origin=Origin(xyz=(0.000, -0.038, -0.004)),
        material=steel,
        name="lever_blade",
    )
    diaphragm_lever.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.000, -0.076, -0.004)),
        material=dark,
        name="lever_handle",
    )

    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head_carriage,
        origin=Origin(xyz=(-0.041, 0.000, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.10, lower=0.0, upper=0.060),
    )
    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=head_carriage,
        child=turret,
        origin=Origin(xyz=(0.081, 0.000, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage_carriage,
        origin=Origin(xyz=(0.030, 0.000, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "diaphragm_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.030, -0.052, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    head = object_model.get_part("head_carriage")
    turret = object_model.get_part("turret")
    stage_carriage = object_model.get_part("stage_carriage")
    lever = object_model.get_part("diaphragm_lever")

    head_slide = object_model.get_articulation("head_slide")
    turret_spin = object_model.get_articulation("turret_spin")
    stage_slide = object_model.get_articulation("stage_slide")
    diaphragm_pivot = object_model.get_articulation("diaphragm_pivot")

    ctx.check(
        "prompt mechanisms are articulated",
        head_slide.articulation_type == ArticulationType.PRISMATIC
        and turret_spin.articulation_type == ArticulationType.CONTINUOUS
        and stage_slide.articulation_type == ArticulationType.PRISMATIC
        and diaphragm_pivot.articulation_type == ArticulationType.REVOLUTE,
        details="Expected head slide, continuous turret, stage slide, and diaphragm pivot.",
    )

    ctx.expect_gap(
        head,
        frame,
        axis="x",
        positive_elem="guide_saddle",
        negative_elem="guide_column",
        max_gap=0.001,
        max_penetration=0.0,
        name="head carriage rides on visible guide",
    )
    ctx.expect_gap(
        stage_carriage,
        frame,
        axis="z",
        positive_elem="cross_slide",
        negative_elem="stage_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage carriage sits on the stage",
    )
    ctx.expect_contact(
        turret,
        head,
        elem_a="turret_disk",
        elem_b="nosepiece_mount",
        contact_tol=0.001,
        name="turret is carried by the nosepiece mount",
    )
    ctx.expect_contact(
        lever,
        frame,
        elem_a="pivot_cap",
        elem_b="lever_boss",
        contact_tol=0.001,
        name="diaphragm lever is supported by condenser boss",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_head = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.060}):
        raised_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            frame,
            axes="z",
            elem_a="guide_saddle",
            elem_b="guide_column",
            min_overlap=0.055,
            name="raised head remains engaged on guide",
        )
    ctx.check(
        "head carriage slides upward",
        rest_head is not None
        and raised_head is not None
        and raised_head[2] > rest_head[2] + 0.055,
        details=f"rest={rest_head}, raised={raised_head}",
    )

    rest_stage = ctx.part_world_position(stage_carriage)
    with ctx.pose({stage_slide: 0.035}):
        shifted_stage = ctx.part_world_position(stage_carriage)
        ctx.expect_overlap(
            stage_carriage,
            frame,
            axes="xy",
            elem_a="cross_slide",
            elem_b="stage_plate",
            min_overlap=0.015,
            name="stage carriage remains on stage at travel limit",
        )
    ctx.check(
        "stage carriage slides across stage",
        rest_stage is not None
        and shifted_stage is not None
        and shifted_stage[1] > rest_stage[1] + 0.030,
        details=f"rest={rest_stage}, shifted={shifted_stage}",
    )

    objective_rest = aabb_center(ctx.part_element_world_aabb(turret, elem="objective_0"))
    with ctx.pose({turret_spin: 1.2}):
        objective_rotated = aabb_center(ctx.part_element_world_aabb(turret, elem="objective_0"))
    ctx.check(
        "objective turret rotates around optical axis",
        objective_rest is not None
        and objective_rotated is not None
        and abs(objective_rotated[1] - objective_rest[1]) > 0.015,
        details=f"rest={objective_rest}, rotated={objective_rotated}",
    )

    handle_rest = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_handle"))
    with ctx.pose({diaphragm_pivot: 0.60}):
        handle_rotated = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_handle"))
    ctx.check(
        "diaphragm lever swings beneath stage",
        handle_rest is not None
        and handle_rotated is not None
        and abs(handle_rotated[0] - handle_rest[0]) > 0.025,
        details=f"rest={handle_rest}, rotated={handle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
