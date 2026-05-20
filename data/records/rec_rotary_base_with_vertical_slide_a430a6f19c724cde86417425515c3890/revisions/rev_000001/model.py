from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_lift_unit")

    dark_shell = model.material("dark_shell", color=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("rubber_black", color=(0.015, 0.015, 0.018, 1.0))
    safety_yellow = model.material("safety_yellow", color=(1.0, 0.68, 0.08, 1.0))
    satin_metal = model.material("satin_metal", color=(0.55, 0.58, 0.58, 1.0))
    blue_cover = model.material("blue_cover", color=(0.10, 0.26, 0.42, 1.0))
    guide_metal = model.material("guide_metal", color=(0.78, 0.80, 0.78, 1.0))

    base_body = model.part("base_body")

    # Broad, low, grounded plinth with rounded plan corners.
    base_shell = (
        cq.Workplane("XY")
        .box(0.70, 0.46, 0.070)
        .edges("|Z")
        .fillet(0.045)
    )
    base_body.visual(
        mesh_from_cadquery(base_shell, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_shell,
        name="base_shell",
    )
    for idx, (x, y) in enumerate(
        ((-0.245, -0.155), (0.245, -0.155), (-0.245, 0.155), (0.245, 0.155))
    ):
        base_body.visual(
            Box((0.130, 0.075, 0.020)),
            origin=Origin(xyz=(x, y, 0.010)),
            material=rubber,
            name=f"foot_{idx}",
        )
    base_body.visual(
        Cylinder(radius=0.195, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=satin_metal,
        name="bearing_pedestal",
    )
    base_body.visual(
        Cylinder(radius=0.145, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=guide_metal,
        name="bearing_race",
    )
    base_body.visual(
        Box((0.230, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.216, 0.097)),
        material=safety_yellow,
        name="front_status_strip",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.170, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_metal,
        name="turntable_disk",
    )
    rotary_stage.visual(
        Cylinder(radius=0.128, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_shell,
        name="center_cap",
    )
    rotary_stage.visual(
        Box((0.340, 0.240, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=blue_cover,
        name="rotating_deck",
    )
    # Stationary guide structure carried by the rotating stage.
    for idx, x in enumerate((-0.115, 0.115)):
        rotary_stage.visual(
            Cylinder(radius=0.018, length=0.400),
            origin=Origin(xyz=(x, 0.0, 0.285)),
            material=guide_metal,
            name=f"guide_rail_{idx}",
        )
    rotary_stage.visual(
        Box((0.280, 0.170, 0.035)),
        origin=Origin(xyz=(0.0, 0.060, 0.5025)),
        material=dark_shell,
        name="top_crosshead",
    )
    rotary_stage.visual(
        Box((0.050, 0.035, 0.420)),
        origin=Origin(xyz=(0.0, 0.125, 0.295)),
        material=dark_shell,
        name="rear_spine",
    )
    rotary_stage.visual(
        Box((0.150, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.105, 0.1025)),
        material=dark_shell,
        name="spine_foot",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.150, 0.105, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=safety_yellow,
        name="lift_block",
    )
    carriage.visual(
        Box((0.180, 0.165, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=blue_cover,
        name="service_plate",
    )
    carriage.visual(
        Box((0.150, 0.022, 0.055)),
        origin=Origin(xyz=(0.0, -0.0935, 0.1775)),
        material=dark_shell,
        name="front_lip",
    )
    for level, z in enumerate((0.045, 0.125)):
        for side, x in enumerate((-0.1125, 0.1125)):
            sign = -1.0 if x < 0.0 else 1.0
            carriage.visual(
                Box((0.030, 0.060, 0.055)),
                origin=Origin(xyz=(sign * 0.082, 0.0, z)),
                material=dark_shell,
                name=f"guide_shoe_{level}_{side}",
            )
            carriage.visual(
                Box((0.044, 0.045, 0.032)),
                origin=Origin(xyz=(sign * 0.055, 0.0, z)),
                material=dark_shell,
                name=f"shoe_bridge_{level}_{side}",
            )

    model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base_body,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-3.1416, upper=3.1416),
    )
    model.articulation(
        "rotary_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.22, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_body")
    rotary = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    pan = object_model.get_articulation("base_to_rotary")
    lift = object_model.get_articulation("rotary_to_carriage")

    ctx.expect_gap(
        rotary,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_disk",
        negative_elem="bearing_pedestal",
        name="turntable sits on bearing pedestal",
    )
    ctx.expect_overlap(
        rotary,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a="turntable_disk",
        elem_b="bearing_pedestal",
        name="rotary disk is centered over the bearing",
    )
    ctx.expect_within(
        carriage,
        rotary,
        axes="xy",
        margin=0.010,
        inner_elem="lift_block",
        outer_elem="rotating_deck",
        name="lift block stays over rotating deck footprint",
    )
    ctx.expect_gap(
        carriage,
        rotary,
        axis="z",
        min_gap=0.024,
        max_gap=0.026,
        positive_elem="lift_block",
        negative_elem="rotating_deck",
        name="lowered carriage clears the deck",
    )

    rest_z = ctx.part_world_position(carriage)[2]
    with ctx.pose({pan: 1.0, lift: 0.180}):
        raised_z = ctx.part_world_position(carriage)[2]
        ctx.expect_within(
            carriage,
            rotary,
            axes="xy",
            margin=0.012,
            inner_elem="lift_block",
            outer_elem="rotating_deck",
            name="raised carriage remains centered on pan stage",
        )
        ctx.expect_gap(
            carriage,
            rotary,
            axis="z",
            min_gap=0.204,
            max_gap=0.206,
            positive_elem="lift_block",
            negative_elem="rotating_deck",
            name="upper carriage travels vertically",
        )
    ctx.check(
        "prismatic lift raises carriage",
        raised_z is not None and rest_z is not None and raised_z > rest_z + 0.170,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
