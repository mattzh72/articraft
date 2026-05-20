from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_trunnion_fixture")

    cast = model.material("dark_cast_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    ground = model.material("ground_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    black = model.material("blackened_recess", rgba=(0.01, 0.012, 0.014, 1.0))
    bolt = model.material("socket_head_bolts", rgba=(0.015, 0.016, 0.018, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=cast,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=cast,
        name="bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.185, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=ground,
        name="thrust_ring",
    )
    for i, (x, y) in enumerate(((0.255, 0.0), (-0.255, 0.0), (0.0, 0.255), (0.0, -0.255))):
        pedestal.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, y, 0.089)),
            material=bolt,
            name=f"base_bolt_{i}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.32, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=machined,
        name="platter_disk",
    )
    platter.visual(
        Box((0.58, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=ground,
        name="mounting_saddle",
    )
    platter.visual(
        mesh_from_geometry(TorusGeometry(radius=0.245, tube=0.004, radial_segments=12, tubular_segments=72), "platter_oil_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=black,
        name="oil_groove",
    )

    frame = model.part("trunnion_frame")
    yoke_geometry = TrunnionYokeGeometry(
        (0.56, 0.30, 0.42),
        span_width=0.38,
        trunnion_diameter=0.078,
        trunnion_center_z=0.25,
        base_thickness=0.05,
        corner_radius=0.012,
        center=False,
    )
    frame.visual(
        mesh_from_geometry(yoke_geometry, "trunnion_yoke"),
        origin=Origin(),
        material=cast,
        name="yoke_body",
    )
    for i, x in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.050, 0.040, 0.172)),
            origin=Origin(xyz=(x, 0.0, 0.136)),
            material=ground,
            name=f"bearing_shoe_{i}",
        )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.160, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="faceplate_disk",
    )
    faceplate.visual(
        Cylinder(radius=0.028, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ground,
        name="trunnion_shaft",
    )
    for i, x in enumerate((-0.155, 0.155)):
        faceplate.visual(
            Cylinder(radius=0.050, length=0.026),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ground,
            name=f"inner_collar_{i}",
        )
    faceplate.visual(
        Cylinder(radius=0.072, length=0.028),
        origin=Origin(xyz=(0.0, 0.039, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=ground,
        name="front_hub",
    )
    faceplate.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="center_bore",
    )
    slot_specs = (
        ("horizontal_slot", (0.220, 0.004, 0.018), (0.0, 0.028, 0.075), 0.0),
        ("lower_slot", (0.220, 0.004, 0.018), (0.0, 0.028, -0.075), 0.0),
        ("vertical_slot", (0.018, 0.004, 0.220), (0.075, 0.028, 0.0), 0.0),
        ("side_slot", (0.018, 0.004, 0.220), (-0.075, 0.028, 0.0), 0.0),
    )
    for name, size, xyz, yaw in slot_specs:
        faceplate.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=black,
            name=name,
        )

    model.articulation(
        "pedestal_to_platter",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "platter_to_frame",
        ArticulationType.FIXED,
        parent=platter,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )
    model.articulation(
        "frame_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    platter = object_model.get_part("platter")
    frame = object_model.get_part("trunnion_frame")
    faceplate = object_model.get_part("faceplate")
    rotary = object_model.get_articulation("pedestal_to_platter")
    tilt = object_model.get_articulation("frame_to_faceplate")

    ctx.check(
        "platter rotates about vertical axis",
        tuple(round(v, 6) for v in rotary.axis) == (0.0, 0.0, 1.0),
        details=f"axis={rotary.axis}",
    )
    ctx.check(
        "faceplate tilts about horizontal trunnion axis",
        tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.expect_gap(
        platter,
        pedestal,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="thrust_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter seats on pedestal thrust ring",
    )
    ctx.expect_overlap(
        platter,
        pedestal,
        axes="xy",
        elem_a="platter_disk",
        elem_b="thrust_ring",
        min_overlap=0.14,
        name="platter is centered over thrust ring",
    )
    ctx.expect_gap(
        frame,
        platter,
        axis="z",
        positive_elem="yoke_body",
        negative_elem="mounting_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="trunnion yoke is bolted to saddle",
    )
    ctx.expect_overlap(
        frame,
        platter,
        axes="xy",
        elem_a="yoke_body",
        elem_b="mounting_saddle",
        min_overlap=0.22,
        name="trunnion yoke footprint sits on saddle",
    )

    rest_saddle_aabb = ctx.part_element_world_aabb(platter, elem="mounting_saddle")
    with ctx.pose({rotary: math.pi / 2.0}):
        turned_saddle_aabb = ctx.part_element_world_aabb(platter, elem="mounting_saddle")
    ctx.check(
        "rotary pose turns the rectangular saddle on the platter",
        rest_saddle_aabb is not None
        and turned_saddle_aabb is not None
        and (turned_saddle_aabb[1][1] - turned_saddle_aabb[0][1])
        > (rest_saddle_aabb[1][1] - rest_saddle_aabb[0][1]) + 0.20,
        details=f"rest={rest_saddle_aabb}, turned={turned_saddle_aabb}",
    )

    rest_faceplate_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_disk")
    with ctx.pose({tilt: 0.70}):
        tilted_faceplate_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_disk")
    ctx.check(
        "tilt pose tips the faceplate out of vertical",
        rest_faceplate_aabb is not None
        and tilted_faceplate_aabb is not None
        and (tilted_faceplate_aabb[1][1] - tilted_faceplate_aabb[0][1])
        > (rest_faceplate_aabb[1][1] - rest_faceplate_aabb[0][1]) + 0.08,
        details=f"rest={rest_faceplate_aabb}, tilted={tilted_faceplate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
