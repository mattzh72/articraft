from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_document_wall_safe")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_interior = model.material("dark_interior", rgba=(0.035, 0.040, 0.045, 1.0))
    worn_edges = model.material("worn_edges", rgba=(0.34, 0.36, 0.38, 1.0))
    black = model.material("black", rgba=(0.01, 0.01, 0.012, 1.0))
    satin_chrome = model.material("satin_chrome", rgba=(0.68, 0.70, 0.72, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.24, 0.27, 0.30, 1.0))

    safe_body = model.part("safe_body")
    # Document-safe proportions: a tall shallow wall box with a narrow,
    # rectangular front opening and a dark back panel visible through it.
    safe_body.visual(
        Box((0.55, 0.025, 1.25)),
        origin=Origin(xyz=(0.0, 0.0975, 0.625)),
        material=dark_interior,
        name="back_panel",
    )
    safe_body.visual(
        Box((0.085, 0.22, 1.25)),
        origin=Origin(xyz=(-0.2325, 0.0, 0.625)),
        material=painted_steel,
        name="side_wall_0",
    )
    safe_body.visual(
        Box((0.085, 0.22, 1.25)),
        origin=Origin(xyz=(0.2325, 0.0, 0.625)),
        material=painted_steel,
        name="side_wall_1",
    )
    safe_body.visual(
        Box((0.55, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=painted_steel,
        name="top_wall",
    )
    safe_body.visual(
        Box((0.55, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_steel,
        name="bottom_wall",
    )
    # A proud front flange makes the rectangular aperture read as a thick safe
    # opening rather than a flat cutout.
    safe_body.visual(
        Box((0.09, 0.030, 1.07)),
        origin=Origin(xyz=(-0.235, -0.122, 0.635)),
        material=worn_edges,
        name="front_stile_0",
    )
    safe_body.visual(
        Box((0.09, 0.030, 1.07)),
        origin=Origin(xyz=(0.235, -0.122, 0.635)),
        material=worn_edges,
        name="front_stile_1",
    )
    safe_body.visual(
        Box((0.55, 0.030, 0.09)),
        origin=Origin(xyz=(0.0, -0.122, 1.175)),
        material=worn_edges,
        name="front_rail_top",
    )
    safe_body.visual(
        Box((0.55, 0.030, 0.09)),
        origin=Origin(xyz=(0.0, -0.122, 0.095)),
        material=worn_edges,
        name="front_rail_bottom",
    )
    # Short side runners/guides for the lower tray.  They touch the side walls
    # and remain visible below the tray at rest and at full extension.
    safe_body.visual(
        Box((0.046, 0.170, 0.014)),
        origin=Origin(xyz=(-0.167, -0.015, 0.299)),
        material=satin_chrome,
        name="side_guide_0",
    )
    safe_body.visual(
        Box((0.046, 0.170, 0.014)),
        origin=Origin(xyz=(0.167, -0.015, 0.299)),
        material=satin_chrome,
        name="side_guide_1",
    )
    # Hinge knuckles fixed to the body side of the left-hand jamb.
    safe_body.visual(
        Cylinder(radius=0.005, length=0.950),
        origin=Origin(xyz=(-0.218, -0.153, 0.615)),
        material=satin_chrome,
        name="hinge_pin",
    )
    for idx, zc in enumerate((0.245, 0.925)):
        safe_body.visual(
            Cylinder(radius=0.012, length=0.220),
            origin=Origin(xyz=(-0.218, -0.153, zc)),
            material=satin_chrome,
            name=f"body_hinge_barrel_{idx}",
        )
        safe_body.visual(
            Box((0.050, 0.014, 0.220)),
            origin=Origin(xyz=(-0.205, -0.136, zc)),
            material=satin_chrome,
            name=f"body_hinge_leaf_{idx}",
        )

    door = model.part("door")
    door.visual(
        Box((0.390, 0.035, 0.950)),
        # The door part frame is the hinge line at the lower hinge height; the
        # plate extends rightward from that line.
        origin=Origin(xyz=(0.225, 0.0, 0.475)),
        material=painted_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.320, 0.006, 0.830)),
        origin=Origin(xyz=(0.240, -0.0205, 0.475)),
        material=dark_interior,
        name="recessed_face",
    )
    door.visual(
        Box((0.040, 0.016, 0.340)),
        origin=Origin(xyz=(0.020, 0.010, 0.475)),
        material=satin_chrome,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=satin_chrome,
        name="door_hinge_barrel",
    )
    for idx, (x, z) in enumerate(((0.080, 0.080), (0.365, 0.080), (0.080, 0.870), (0.365, 0.870))):
        door.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.021, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_chrome,
            name=f"door_screw_{idx}",
        )

    # The current pose leaves the heavy door swung outward so the narrow cavity
    # and tray supports are visible.  Posing the joint to about +1.05 rad closes it.
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=safe_body,
        child=door,
        origin=Origin(xyz=(-0.218, -0.153, 0.140), rpy=(0.0, 0.0, -1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.05, upper=1.05),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="dial_spindle",
    )
    dial.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_disc",
    )
    dial.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="dial_face",
    )
    dial.visual(
        Box((0.006, 0.004, 0.052)),
        origin=Origin(xyz=(0.0, -0.060, 0.010)),
        material=black,
        name="dial_index",
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.285, -0.0175, 0.690)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )

    lever_handle = model.part("lever_handle")
    lever_handle.visual(
        Cylinder(radius=0.014, length=0.027),
        origin=Origin(xyz=(0.0, -0.0135, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="handle_spindle",
    )
    lever_handle.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="handle_hub",
    )
    lever_handle.visual(
        Box((0.125, 0.018, 0.032)),
        origin=Origin(xyz=(0.070, -0.058, 0.0)),
        material=satin_chrome,
        name="handle_bar",
    )
    lever_handle.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.138, -0.058, 0.0)),
        material=satin_chrome,
        name="handle_tip",
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lever_handle,
        origin=Origin(xyz=(0.285, -0.0175, 0.535)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.90, upper=0.90),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.300, 0.140, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tray_finish,
        name="tray_floor",
    )
    tray.visual(
        Box((0.012, 0.140, 0.040)),
        origin=Origin(xyz=(-0.156, 0.0, 0.026)),
        material=tray_finish,
        name="tray_side_0",
    )
    tray.visual(
        Box((0.012, 0.140, 0.040)),
        origin=Origin(xyz=(0.156, 0.0, 0.026)),
        material=tray_finish,
        name="tray_side_1",
    )
    tray.visual(
        Box((0.300, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.076, 0.026)),
        material=tray_finish,
        name="tray_lip_front",
    )
    tray.visual(
        Box((0.300, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.076, 0.026)),
        material=tray_finish,
        name="tray_lip_rear",
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=safe_body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.015, 0.312)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("lever_handle")
    tray = object_model.get_part("tray")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")
    tray_joint = object_model.get_articulation("body_to_tray")

    # Captured shafts are intentionally nested in the simplified solid door and
    # hinge barrels.  Scope each allowance to the named shaft/barrel feature.
    ctx.allow_overlap(
        dial,
        door,
        elem_a="dial_spindle",
        elem_b="door_slab",
        reason="The combination dial spindle is intentionally captured through the safe door plate.",
    )
    ctx.allow_overlap(
        dial,
        door,
        elem_a="dial_spindle",
        elem_b="recessed_face",
        reason="The dial spindle also passes through the thin recessed front face plate.",
    )
    ctx.allow_overlap(
        door,
        handle,
        elem_a="door_slab",
        elem_b="handle_spindle",
        reason="The lever spindle intentionally passes through the door plate.",
    )
    ctx.allow_overlap(
        door,
        handle,
        elem_a="recessed_face",
        elem_b="handle_spindle",
        reason="The lever spindle also passes through the thin recessed front face plate.",
    )
    ctx.allow_overlap(
        door,
        body,
        elem_a="door_hinge_barrel",
        elem_b="hinge_pin",
        reason="The body-mounted hinge pin intentionally runs through the door hinge barrel.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and handle_joint.articulation_type == ArticulationType.REVOLUTE
        and tray_joint.articulation_type == ArticulationType.PRISMATIC,
    )

    ctx.expect_overlap(
        dial,
        door,
        axes="xz",
        min_overlap=0.020,
        elem_a="dial_spindle",
        elem_b="door_slab",
        name="dial spindle passes through door face",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="xz",
        min_overlap=0.020,
        elem_a="dial_spindle",
        elem_b="recessed_face",
        name="dial spindle passes through front plate",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="xz",
        min_overlap=0.020,
        elem_a="handle_spindle",
        elem_b="door_slab",
        name="handle spindle passes through door face",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="xz",
        min_overlap=0.020,
        elem_a="handle_spindle",
        elem_b="recessed_face",
        name="handle spindle passes through front plate",
    )
    ctx.expect_within(
        body,
        door,
        axes="xy",
        margin=0.003,
        inner_elem="hinge_pin",
        outer_elem="door_hinge_barrel",
        name="hinge pin is captured inside door barrel",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="z",
        min_overlap=0.250,
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel",
        name="hinge pin runs through the door knuckle",
    )

    # The tray sits on both visible side guides and remains engaged when pulled.
    for guide_name in ("side_guide_0", "side_guide_1"):
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem="tray_floor",
            negative_elem=guide_name,
            name=f"tray floor rides on {guide_name}",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="xy",
            min_overlap=0.005,
            elem_a="tray_floor",
            elem_b=guide_name,
            name=f"tray overlaps {guide_name} in plan",
        )

    open_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.05}):
        closed_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings from open display pose toward the jamb",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.20,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    rest_tray_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tray_joint: 0.100}):
        extended_tray_aabb = ctx.part_world_aabb(tray)
        for guide_name in ("side_guide_0", "side_guide_1"):
            ctx.expect_overlap(
                tray,
                body,
                axes="xy",
                min_overlap=0.005,
                elem_a="tray_floor",
                elem_b=guide_name,
                name=f"extended tray remains on {guide_name}",
            )
    ctx.check(
        "tray slides outward on runners",
        rest_tray_aabb is not None
        and extended_tray_aabb is not None
        and extended_tray_aabb[0][1] < rest_tray_aabb[0][1] - 0.08,
        details=f"rest={rest_tray_aabb}, extended={extended_tray_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
