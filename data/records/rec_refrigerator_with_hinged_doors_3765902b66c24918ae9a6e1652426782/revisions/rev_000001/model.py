from __future__ import annotations

import math

import cadquery as cq

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


def _rounded_xz_slab(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """A cabinet/door slab with a rounded front-view rectangle extruded in depth."""
    profile = cq.Sketch().rect(width, height).vertices().fillet(radius)
    return (
        cq.Workplane("XZ")
        .placeSketch(profile)
        .extrude(depth)
        .translate((0.0, depth / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_refrigerator")

    enamel = model.material("warm_ivory_enamel", rgba=(0.92, 0.86, 0.70, 1.0))
    shadow = model.material("dark_rubber_shadow", rgba=(0.035, 0.032, 0.030, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    black = model.material("black_foot", rgba=(0.04, 0.035, 0.03, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(
            _rounded_xz_slab(0.82, 0.72, 1.72, 0.105),
            "rounded_cabinet_body",
            tolerance=0.002,
            angular_tolerance=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=enamel,
        name="cabinet_body",
    )
    cabinet.visual(
        Box((0.70, 0.62, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black,
        name="recessed_plinth",
    )
    # Fixed hinge barrels and a small chrome leaf make the side hinge line visible.
    hinge_axis_x = -0.425
    hinge_axis_y = -0.382
    for i, z in enumerate((0.42, 0.92, 1.42)):
        cabinet.visual(
            Cylinder(radius=0.020, length=0.18),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z), rpy=(0.0, 0.0, 0.0)),
            material=chrome,
            name=f"cabinet_hinge_knuckle_{i}",
        )
        cabinet.visual(
            Box((0.040, 0.024, 0.22)),
            origin=Origin(xyz=(-0.425, -0.372, z)),
            material=chrome,
            name=f"cabinet_hinge_leaf_{i}",
        )
    cabinet.visual(
        Cylinder(radius=0.006, length=1.26),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.92)),
        material=chrome,
        name="hinge_pin",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(
            _rounded_xz_slab(0.765, 0.075, 1.56, 0.075),
            "rounded_door_panel",
            tolerance=0.0015,
            angular_tolerance=0.10,
        ),
        # Door coordinates are measured from the vertical hinge axis; the slab
        # extends toward +X and proud of the cabinet front along -Y.
        origin=Origin(xyz=(0.4175, -0.0375, 0.0)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((0.635, 0.006, 0.024)),
        origin=Origin(xyz=(0.385, -0.078, 0.48)),
        material=chrome,
        name="top_chrome_band",
    )
    door.visual(
        Box((0.620, 0.006, 0.018)),
        origin=Origin(xyz=(0.395, -0.078, -0.47)),
        material=chrome,
        name="lower_chrome_band",
    )
    door.visual(
        Box((0.675, 0.007, 0.026)),
        origin=Origin(xyz=(0.383, -0.078, 0.0)),
        material=shadow,
        name="center_shadow_reveal",
    )
    for i, z in enumerate((-0.25, 0.25)):
        door.visual(
            Box((0.035, 0.020, 0.26)),
            origin=Origin(xyz=(0.0275, -0.010, z)),
            material=chrome,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.017, length=0.22),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"door_hinge_knuckle_{i}",
        )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.92)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.1, lower=0.0, upper=1.75),
    )

    handle_latch = model.part("handle_latch")
    # The latch pivots on a short door-normal spindle; a vertical pull bar and
    # connecting lugs make the handle read as one chrome casting.
    handle_latch.visual(
        Cylinder(radius=0.048, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    handle_latch.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.055, -0.030, 0.0)),
        material=chrome,
        name="vertical_grip",
    )
    handle_latch.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(0.028, -0.030, 0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="upper_lug",
    )
    handle_latch.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(0.028, -0.030, -0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lower_lug",
    )
    handle_latch.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.025, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="center_lug",
    )
    handle_latch.visual(
        Box((0.030, 0.010, 0.070)),
        origin=Origin(xyz=(-0.004, -0.012, -0.055)),
        material=chrome,
        name="latch_tooth",
    )

    model.articulation(
        "door_to_handle_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle_latch,
        origin=Origin(xyz=(0.735, -0.0895, 0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    handle_latch = object_model.get_part("handle_latch")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    latch_pivot = object_model.get_articulation("door_to_handle_latch")

    for i in range(2):
        knuckle = f"door_hinge_knuckle_{i}"
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The visible hinge pin is intentionally captured inside the door hinge barrel proxy.",
        )
        ctx.expect_within(
            cabinet,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle,
            margin=0.0,
            name=f"hinge pin is centered in door knuckle {i}",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.18,
            name=f"hinge pin passes through door knuckle {i}",
        )

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="cabinet_body",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.030,
        name="closed door sits just proud of cabinet",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="cabinet_body",
        min_overlap=0.68,
        name="broad door covers the tall cabinet face",
    )
    ctx.expect_gap(
        door,
        handle_latch,
        axis="y",
        positive_elem="door_panel",
        negative_elem="pivot_hub",
        min_gap=0.0,
        max_gap=0.004,
        name="latch pivot is seated on the door face",
    )
    ctx.expect_overlap(
        handle_latch,
        door,
        axes="xz",
        elem_a="pivot_hub",
        elem_b="door_panel",
        min_overlap=0.050,
        name="latch pivot lies near the opposite door edge",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge swings the free edge outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.20,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    latched_aabb = ctx.part_world_aabb(handle_latch)
    with ctx.pose({latch_pivot: 0.42}):
        turned_aabb = ctx.part_world_aabb(handle_latch)
    ctx.check(
        "handle latch rotates on its own short pivot",
        latched_aabb is not None
        and turned_aabb is not None
        and abs(turned_aabb[1][0] - latched_aabb[1][0]) > 0.020,
        details=f"latched_aabb={latched_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
