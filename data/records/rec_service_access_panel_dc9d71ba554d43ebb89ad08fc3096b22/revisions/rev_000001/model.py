from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    powder_coat = model.material("powder_coat_gray", rgba=(0.36, 0.39, 0.39, 1.0))
    trim = model.material("dark_trim_frame", rgba=(0.12, 0.13, 0.13, 1.0))
    door_paint = model.material("slightly_lighter_door", rgba=(0.47, 0.50, 0.49, 1.0))
    gasket = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.014, 1.0))
    hardware = model.material("brushed_stainless_hardware", rgba=(0.66, 0.67, 0.64, 1.0))
    shadow = model.material("dark_internal_shadow", rgba=(0.025, 0.027, 0.03, 1.0))

    body = model.part("equipment_face")

    # The equipment face is built from continuous plate strips around an
    # actual rectangular service opening rather than a single solid slab.
    body.visual(
        Box((0.130, 0.035, 0.500)),
        origin=Origin(xyz=(-0.295, -0.0175, 0.0)),
        material=powder_coat,
        name="face_stile_0",
    )
    body.visual(
        Box((0.130, 0.035, 0.500)),
        origin=Origin(xyz=(0.295, -0.0175, 0.0)),
        material=powder_coat,
        name="face_stile_1",
    )
    body.visual(
        Box((0.720, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, -0.0175, 0.205)),
        material=powder_coat,
        name="face_rail_0",
    )
    body.visual(
        Box((0.720, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, -0.0175, -0.205)),
        material=powder_coat,
        name="face_rail_1",
    )
    body.visual(
        Box((0.012, 0.035, 0.320)),
        origin=Origin(xyz=(-0.226, -0.0175, 0.0)),
        material=shadow,
        name="opening_wall_0",
    )
    body.visual(
        Box((0.012, 0.035, 0.320)),
        origin=Origin(xyz=(0.226, -0.0175, 0.0)),
        material=shadow,
        name="opening_wall_1",
    )
    body.visual(
        Box((0.452, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.0175, 0.156)),
        material=shadow,
        name="opening_wall_2",
    )
    body.visual(
        Box((0.452, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.0175, -0.156)),
        material=shadow,
        name="opening_wall_3",
    )
    body.visual(
        Box((0.452, 0.004, 0.320)),
        origin=Origin(xyz=(0.0, -0.0365, 0.0)),
        material=shadow,
        name="dark_opening_void",
    )

    # Raised rectangular trim frame around the opening.  The four members
    # intentionally overlap at the corners so the body reads as one mounted
    # frame assembly.
    body.visual(
        Box((0.055, 0.013, 0.405)),
        origin=Origin(xyz=(-0.2575, 0.0055, 0.0)),
        material=trim,
        name="frame_stile_0",
    )
    body.visual(
        Box((0.055, 0.013, 0.405)),
        origin=Origin(xyz=(0.2575, 0.0055, 0.0)),
        material=trim,
        name="frame_stile_1",
    )
    body.visual(
        Box((0.57, 0.013, 0.055)),
        origin=Origin(xyz=(0.0, 0.0055, 0.1775)),
        material=trim,
        name="frame_rail_0",
    )
    body.visual(
        Box((0.57, 0.013, 0.055)),
        origin=Origin(xyz=(0.0, 0.0055, -0.1775)),
        material=trim,
        name="frame_rail_1",
    )

    # A narrow compression gasket just inside the frame lip.
    body.visual(
        Box((0.020, 0.006, 0.330)),
        origin=Origin(xyz=(-0.218, 0.013, 0.0)),
        material=gasket,
        name="gasket_stile_0",
    )
    body.visual(
        Box((0.020, 0.006, 0.330)),
        origin=Origin(xyz=(0.218, 0.013, 0.0)),
        material=gasket,
        name="gasket_stile_1",
    )
    body.visual(
        Box((0.456, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.013, 0.155)),
        material=gasket,
        name="gasket_rail_0",
    )
    body.visual(
        Box((0.456, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.013, -0.155)),
        material=gasket,
        name="gasket_rail_1",
    )

    # Fixed latch keeper on the body at the edge opposite the hinge.
    body.visual(
        Box((0.030, 0.012, 0.090)),
        origin=Origin(xyz=(0.292, 0.018, 0.0)),
        material=hardware,
        name="latch_keeper",
    )
    body.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(0.276, 0.025, 0.0)),
        material=hardware,
        name="keeper_lip",
    )

    hinge_x = -0.295
    hinge_y = 0.020
    hinge_radius = 0.010

    # Short, tucked hinge hardware: two compact stations with interleaved
    # fixed knuckles, rather than a long piano hinge.
    for i, zc in enumerate((0.145, -0.145)):
        body.visual(
            Box((0.035, 0.006, 0.115)),
            origin=Origin(xyz=(hinge_x + 0.013, 0.007, zc)),
            material=hardware,
            name=f"fixed_hinge_leaf_{i}",
        )
        for j, dz in enumerate((-0.0375, 0.0375)):
            body.visual(
                Cylinder(radius=hinge_radius, length=0.035),
                origin=Origin(xyz=(hinge_x, hinge_y, zc + dz)),
                material=hardware,
                name=f"fixed_hinge_knuckle_{i}_{j}",
            )

    door = model.part("door")
    door.visual(
        Box((0.525, 0.018, 0.360)),
        # Child frame is on the hinge pin.  The closed door extends along +X
        # from that vertical axis and sits just proud of the gasket.
        origin=Origin(xyz=(0.2875, 0.005, 0.0)),
        material=door_paint,
        name="door_skin",
    )
    door.visual(
        Box((0.455, 0.006, 0.275)),
        origin=Origin(xyz=(0.292, 0.0160, 0.0)),
        material=powder_coat,
        name="door_recess_panel",
    )
    door.visual(
        Box((0.016, 0.010, 0.330)),
        origin=Origin(xyz=(0.512, 0.017, 0.0)),
        material=gasket,
        name="latch_edge_strip",
    )
    door.visual(
        Box((0.070, 0.018, 0.045)),
        origin=Origin(xyz=(0.455, 0.022, 0.0)),
        material=hardware,
        name="latch_handle",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.418, 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="latch_boss",
    )

    for i, zc in enumerate((0.145, -0.145)):
        door.visual(
            Box((0.052, 0.006, 0.040)),
            origin=Origin(xyz=(0.026, 0.009, zc)),
            material=hardware,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hardware,
            name=f"door_hinge_knuckle_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_skin",
            negative_elem="gasket_stile_0",
            min_gap=0.0,
            max_gap=0.004,
            name="closed door sits just proud of gasket",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_skin",
            elem_b="dark_opening_void",
            min_overlap=0.28,
            name="door covers the service opening",
        )
        rest_latch_aabb = ctx.part_element_world_aabb(door, elem="latch_edge_strip")

    with ctx.pose({hinge: 1.35}):
        open_latch_aabb = ctx.part_element_world_aabb(door, elem="latch_edge_strip")
        ctx.expect_origin_gap(
            door,
            body,
            axis="z",
            min_gap=-0.001,
            max_gap=0.001,
            name="vertical hinge keeps door from lifting",
        )

    ctx.check(
        "positive hinge swing moves latch edge outward",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][1] > rest_latch_aabb[1][1] + 0.18,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
