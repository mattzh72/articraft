from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_cleaner")

    # Main Body (Motor unit)
    main_body_cq = (
        cq.Workplane("XY")
        .box(0.12, 0.12, 0.3, centered=(True, True, False))
        .translate((0, 0, 0.8))
    )
    bracket = (
        cq.Workplane("XY")
        .box(0.04, 0.04, 0.04, centered=(True, True, True))
        .translate((0, 0.08, 0.82))
    )
    shoulder_barrel = (
        cq.Workplane("XZ", origin=(0, 0.12, 0.82))
        .circle(0.02)
        .extrude(0.02, both=True)
    )
    main_body_cq = main_body_cq.union(bracket).union(shoulder_barrel)

    main_body = model.part("main_body")
    main_body.visual(
        mesh_from_cadquery(main_body_cq, "main_body_mesh"),
        origin=Origin(),
        name="main_body_vis",
    )

    # Wand Upper
    wand_upper_cq = cq.Workplane("XY").circle(0.02).extrude(-0.4)
    shoulder_clevis_1 = cq.Workplane("XZ", origin=(0, 0.02, 0)).circle(0.025).extrude(0.02)
    shoulder_clevis_2 = cq.Workplane("XZ", origin=(0, -0.02, 0)).circle(0.025).extrude(-0.02)
    elbow_barrel = cq.Workplane("XZ", origin=(0, 0, -0.4)).circle(0.02).extrude(0.02, both=True)
    
    wand_upper_cq = wand_upper_cq.union(shoulder_clevis_1).union(shoulder_clevis_2).union(elbow_barrel)

    wand_upper = model.part("wand_upper")
    wand_upper.visual(
        mesh_from_cadquery(wand_upper_cq, "wand_upper_mesh"),
        origin=Origin(),
        name="wand_upper_vis",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=wand_upper,
        origin=Origin(xyz=(0.0, 0.12, 0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    # Wand Lower
    wand_lower_cq = cq.Workplane("XY").circle(0.02).extrude(-0.34)
    elbow_clevis_1 = cq.Workplane("XZ", origin=(0, 0.02, 0)).circle(0.025).extrude(0.02)
    elbow_clevis_2 = cq.Workplane("XZ", origin=(0, -0.02, 0)).circle(0.025).extrude(-0.02)
    wrist_barrel = cq.Workplane("XZ", origin=(0, 0, -0.34)).circle(0.02).extrude(0.02, both=True)
    
    wand_lower_cq = wand_lower_cq.union(elbow_clevis_1).union(elbow_clevis_2).union(wrist_barrel)

    wand_lower = model.part("wand_lower")
    wand_lower.visual(
        mesh_from_cadquery(wand_lower_cq, "wand_lower_mesh"),
        origin=Origin(),
        name="wand_lower_vis",
    )

    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=wand_upper,
        child=wand_lower,
        origin=Origin(xyz=(0.0, 0.0, -0.4)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    # Floor Nozzle
    nozzle_cq = (
        cq.Workplane("XY")
        .box(0.25, 0.15, 0.05, centered=(True, True, False))
        .translate((0.05, 0, -0.08))
    )
    neck = cq.Workplane("XY").circle(0.02).extrude(-0.08)
    wrist_clevis_1 = cq.Workplane("XZ", origin=(0, 0.02, 0)).circle(0.025).extrude(0.02)
    wrist_clevis_2 = cq.Workplane("XZ", origin=(0, -0.02, 0)).circle(0.025).extrude(-0.02)
    
    nozzle_cq = nozzle_cq.union(neck).union(wrist_clevis_1).union(wrist_clevis_2)

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        mesh_from_cadquery(nozzle_cq, "floor_nozzle_mesh"),
        origin=Origin(),
        name="floor_nozzle_vis",
    )

    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=wand_lower,
        child=floor_nozzle,
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_body = object_model.get_part("main_body")
    wand_upper = object_model.get_part("wand_upper")
    wand_lower = object_model.get_part("wand_lower")
    floor_nozzle = object_model.get_part("floor_nozzle")

    ctx.allow_overlap(
        wand_upper, main_body,
        reason="The outer clevis of the shoulder joint intentionally overlaps the main body barrel.",
    )
    ctx.allow_overlap(
        wand_lower, wand_upper,
        reason="The outer clevis of the elbow joint intentionally overlaps the wand upper barrel.",
    )
    ctx.allow_overlap(
        floor_nozzle, wand_lower,
        reason="The outer clevis of the wrist joint intentionally overlaps the wand lower barrel.",
    )

    ctx.expect_contact(main_body, wand_upper, name="shoulder_contact")
    ctx.expect_contact(wand_upper, wand_lower, name="elbow_contact")
    ctx.expect_contact(wand_lower, floor_nozzle, name="wrist_contact")

    return ctx.report()


object_model = build_object_model()