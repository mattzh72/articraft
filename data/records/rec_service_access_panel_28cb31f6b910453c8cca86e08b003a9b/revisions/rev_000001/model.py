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
)

import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_steel = Material("painted_steel", color=(0.30, 0.34, 0.36, 1.0))
    dark_gasket = Material("dark_gasket", color=(0.025, 0.027, 0.026, 1.0))
    bare_metal = Material("bare_metal", color=(0.63, 0.64, 0.61, 1.0))
    door_blue = Material("door_blue", color=(0.08, 0.18, 0.28, 1.0))
    latch_black = Material("latch_black", color=(0.01, 0.01, 0.012, 1.0))

    face = model.part("equipment_face")

    # A large vertical equipment skin with a real long, shallow rectangular opening.
    face.visual(
        Box((1.35, 0.040, 0.225)),
        origin=Origin(xyz=(0.0, 0.0, 0.2375)),
        material=painted_steel,
        name="upper_skin",
    )
    face.visual(
        Box((1.35, 0.040, 0.225)),
        origin=Origin(xyz=(0.0, 0.0, -0.2375)),
        material=painted_steel,
        name="lower_skin",
    )
    face.visual(
        Box((0.200, 0.040, 0.250)),
        origin=Origin(xyz=(-0.575, 0.0, 0.0)),
        material=painted_steel,
        name="hinge_side_skin",
    )
    face.visual(
        Box((0.200, 0.040, 0.250)),
        origin=Origin(xyz=(0.575, 0.0, 0.0)),
        material=painted_steel,
        name="latch_side_skin",
    )

    # Dark return surfaces make the aperture read as an actual cut-through service opening.
    face.visual(
        Box((0.018, 0.070, 0.250)),
        origin=Origin(xyz=(-0.484, -0.015, 0.0)),
        material=dark_gasket,
        name="hinge_return",
    )
    face.visual(
        Box((0.018, 0.070, 0.250)),
        origin=Origin(xyz=(0.484, -0.015, 0.0)),
        material=dark_gasket,
        name="latch_return",
    )
    face.visual(
        Box((0.968, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, -0.015, 0.134)),
        material=dark_gasket,
        name="upper_return",
    )
    face.visual(
        Box((0.968, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, -0.015, -0.134)),
        material=dark_gasket,
        name="lower_return",
    )

    # Raised rectangular frame around the opening.
    face.visual(
        Box((1.120, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, 0.029, 0.1475)),
        material=bare_metal,
        name="frame_top",
    )
    face.visual(
        Box((1.120, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, 0.029, -0.1475)),
        material=bare_metal,
        name="frame_bottom",
    )
    face.visual(
        Box((0.045, 0.018, 0.340)),
        origin=Origin(xyz=(-0.515, 0.029, 0.0)),
        material=bare_metal,
        name="hinge_frame",
    )
    face.visual(
        Box((0.045, 0.018, 0.340)),
        origin=Origin(xyz=(0.515, 0.029, 0.0)),
        material=bare_metal,
        name="latch_frame",
    )

    screw_positions = [
        (-0.42, 0.1475),
        (0.42, 0.1475),
        (-0.42, -0.1475),
        (0.42, -0.1475),
    ]
    for i, (x, z) in enumerate(screw_positions):
        face.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, 0.040, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_gasket,
            name=f"frame_screw_{i}",
        )

    # Static hinge brackets on the equipment face, separated from the moving knuckle.
    face.visual(
        Box((0.038, 0.020, 0.080)),
        origin=Origin(xyz=(-0.511, 0.043, 0.092)),
        material=bare_metal,
        name="upper_hinge_leaf",
    )
    face.visual(
        Box((0.038, 0.020, 0.080)),
        origin=Origin(xyz=(-0.511, 0.043, -0.092)),
        material=bare_metal,
        name="lower_hinge_leaf",
    )
    face.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(-0.490, 0.052, 0.092)),
        material=bare_metal,
        name="upper_hinge_knuckle",
    )
    face.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(-0.490, 0.052, -0.092)),
        material=bare_metal,
        name="lower_hinge_knuckle",
    )
    face.visual(
        Cylinder(radius=0.004, length=0.270),
        origin=Origin(xyz=(-0.490, 0.052, 0.0)),
        material=bare_metal,
        name="hinge_pin",
    )
    face.visual(
        Box((0.035, 0.016, 0.100)),
        origin=Origin(xyz=(0.515, 0.046, 0.0)),
        material=bare_metal,
        name="latch_keeper",
    )

    door = model.part("door")
    door.visual(
        Box((0.920, 0.022, 0.230)),
        origin=Origin(xyz=(0.495, 0.010, 0.0)),
        material=door_blue,
        name="door_skin",
    )
    door.visual(
        Box((0.036, 0.014, 0.080)),
        origin=Origin(xyz=(0.024, -0.002, 0.0)),
        material=bare_metal,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bare_metal,
        name="door_hinge_knuckle",
    )
    door.visual(
        Box((0.050, 0.014, 0.120)),
        origin=Origin(xyz=(0.910, 0.024, 0.0)),
        material=latch_black,
        name="latch_edge",
    )
    door.visual(
        Box((0.018, 0.010, 0.220)),
        origin=Origin(xyz=(0.946, 0.010, 0.0)),
        material=bare_metal,
        name="thin_free_edge",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(-0.490, 0.052, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        face,
        door,
        elem_a="hinge_pin",
        elem_b="door_hinge_knuckle",
        reason="The fixed hinge pin is intentionally captured inside the moving door knuckle.",
    )
    ctx.expect_within(
        face,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="door_hinge_knuckle",
        margin=0.001,
        name="hinge pin sits inside door knuckle",
    )
    ctx.expect_overlap(
        face,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="door_hinge_knuckle",
        min_overlap=0.065,
        name="hinge pin captures moving knuckle",
    )

    ctx.expect_gap(
        door,
        face,
        axis="y",
        positive_elem="door_skin",
        negative_elem="frame_top",
        min_gap=0.010,
        max_gap=0.014,
        name="closed door skin stands proud of frame",
    )
    ctx.expect_gap(
        face,
        door,
        axis="x",
        positive_elem="latch_keeper",
        negative_elem="thin_free_edge",
        min_gap=0.025,
        max_gap=0.060,
        name="free edge stops at latch keeper",
    )

    door_box = ctx.part_element_world_aabb(door, elem="door_skin")
    if door_box is not None:
        dx = door_box[1][0] - door_box[0][0]
        dz = door_box[1][2] - door_box[0][2]
        ctx.check(
            "door is long and thin",
            dx > 3.5 * dz,
            details=f"door_skin width={dx:.3f}, height={dz:.3f}",
        )

    rest_latch = ctx.part_element_world_aabb(door, elem="latch_edge")
    with ctx.pose({hinge: 1.20}):
        open_latch = ctx.part_element_world_aabb(door, elem="latch_edge")
    ctx.check(
        "vertical side hinge swings door outward",
        rest_latch is not None
        and open_latch is not None
        and open_latch[1][1] > rest_latch[1][1] + 0.55,
        details=f"closed={rest_latch}, open={open_latch}",
    )

    return ctx.report()


object_model = build_object_model()
