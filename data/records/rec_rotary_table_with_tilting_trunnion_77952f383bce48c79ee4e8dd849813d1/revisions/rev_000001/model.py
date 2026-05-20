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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cradle_indexer")

    cast_iron = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.18, 0.24, 0.30, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.015, 0.016, 0.018, 1.0))
    brass = model.material("brass_index_marks", rgba=(0.78, 0.60, 0.26, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.24, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=machined_steel,
        name="bearing_race",
    )
    for i in range(4):
        a = math.radians(45.0 + 90.0 * i)
        base.visual(
            Box((0.090, 0.032, 0.014)),
            origin=Origin(
                xyz=(0.175 * math.cos(a), 0.175 * math.sin(a), -0.007),
                rpy=(0.0, 0.0, a),
            ),
            material=black_oxide,
            name=f"rubber_foot_{i}",
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=blued_steel,
        name="lower_stage",
    )
    cradle.visual(
        Cylinder(radius=0.168, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=machined_steel,
        name="index_table",
    )
    cradle.visual(
        Cylinder(radius=0.070, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=cast_iron,
        name="rotary_hub",
    )
    for i in range(12):
        a = math.radians(30.0 * i)
        cradle.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(0.145 * math.cos(a), 0.145 * math.sin(a), 0.067)),
            material=brass if i % 3 == 0 else black_oxide,
            name=f"index_pin_{i}",
        )

    # The yoke is deliberately offset on the circular stage, like a compact
    # cradle indexer head rather than a centered turntable.
    yoke = TrunnionYokeGeometry(
        (0.310, 0.082, 0.285),
        span_width=0.205,
        trunnion_diameter=0.030,
        trunnion_center_z=0.180,
        base_thickness=0.032,
        corner_radius=0.006,
        center=False,
    )
    cradle.visual(
        mesh_from_geometry(yoke, "offset_trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.050, 0.066)),
        material=cast_iron,
        name="yoke_frame",
    )

    faceplate = model.part("faceplate")
    # The plate's local X axis is the trunnion axis.  Its round face is normal
    # to local Y, so the revolute joint tilts the whole fixture head.
    faceplate.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="plate_disk",
    )
    faceplate.visual(
        Cylinder(radius=0.098, length=0.007),
        origin=Origin(xyz=(0.0, -0.0185, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blued_steel,
        name="front_rim",
    )
    faceplate.visual(
        Cylinder(radius=0.034, length=0.058),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="center_boss",
    )
    faceplate.visual(
        Cylinder(radius=0.0151, length=0.285),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="trunnion_shaft",
    )
    for side, x in (("neg", -0.074), ("pos", 0.074)):
        faceplate.visual(
            Cylinder(radius=0.022, length=0.016),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name=f"trunnion_collar_{side}",
        )
    faceplate.visual(
        Box((0.122, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -0.0225, 0.0)),
        material=black_oxide,
        name="horizontal_slot",
    )
    faceplate.visual(
        Box((0.012, 0.003, 0.122)),
        origin=Origin(xyz=(0.0, -0.0227, 0.0)),
        material=black_oxide,
        name="vertical_slot",
    )
    for i in range(6):
        a = math.radians(60.0 * i + 30.0)
        faceplate.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(
                xyz=(0.060 * math.cos(a), -0.0245, 0.060 * math.sin(a)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=f"socket_bolt_{i}",
        )

    lower_axis = model.articulation(
        "base_to_cradle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    lower_axis.meta["description"] = "Circular lower stage indexes about the vertical bearing axis."

    tilt_axis = model.articulation(
        "cradle_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.050, 0.246)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.9, lower=-1.35, upper=1.35),
    )
    tilt_axis.meta["description"] = "Horizontal trunnion axis through the cradle cheeks tilts the faceplate."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cradle = object_model.get_part("cradle")
    faceplate = object_model.get_part("faceplate")
    lower_axis = object_model.get_articulation("base_to_cradle")
    tilt_axis = object_model.get_articulation("cradle_to_faceplate")

    ctx.allow_overlap(
        cradle,
        faceplate,
        elem_a="yoke_frame",
        elem_b="trunnion_shaft",
        reason=(
            "The faceplate trunnion shaft is intentionally captured in the yoke "
            "bearing bore; the helper yoke is a mesh proxy for the bushed cheeks."
        ),
    )
    ctx.expect_contact(
        "base",
        cradle,
        elem_a="bearing_race",
        elem_b="lower_stage",
        contact_tol=0.001,
        name="lower stage sits on bearing race",
    )
    ctx.expect_overlap(
        faceplate,
        cradle,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        min_overlap=0.24,
        name="trunnion shaft spans both yoke cheeks",
    )
    ctx.expect_within(
        faceplate,
        cradle,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="yoke_frame",
        margin=0.0,
        name="trunnion shaft is centered in the yoke bore",
    )
    ctx.expect_within(
        faceplate,
        cradle,
        axes="xz",
        inner_elem="plate_disk",
        outer_elem="yoke_frame",
        margin=0.025,
        name="faceplate disk is carried inside the cradle opening",
    )

    rest_pos = ctx.part_world_position(faceplate)
    with ctx.pose({lower_axis: math.pi / 2.0}):
        indexed_pos = ctx.part_world_position(faceplate)
    ctx.check(
        "lower stage rotates offset cradle",
        rest_pos is not None
        and indexed_pos is not None
        and abs(indexed_pos[0] + 0.050) < 0.004
        and abs(indexed_pos[1]) < 0.004,
        details=f"rest={rest_pos}, indexed={indexed_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(faceplate, elem="plate_disk")
    with ctx.pose({tilt_axis: 1.0}):
        tilted_aabb = ctx.part_element_world_aabb(faceplate, elem="plate_disk")
    ctx.check(
        "faceplate tilts about horizontal trunnion",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[1][1] - tilted_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.015,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
