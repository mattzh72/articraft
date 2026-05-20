from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


OUTER_WIDTH = 0.48
OUTER_HEIGHT = 0.32
FRAME_DEPTH = 0.055
SIDE_RAIL = 0.055
TOP_RAIL = 0.040
OPENING_WIDTH = OUTER_WIDTH - 2.0 * SIDE_RAIL
BLADE_COUNT = 5
BLADE_PITCH = 0.045
FIRST_BLADE_Z = TOP_RAIL + 0.035
BLADE_LENGTH = 0.300
BLADE_DEPTH = 0.042
BLADE_THICKNESS = 0.020
BLADE_REST_ANGLE = math.radians(25.0)
PIN_RADIUS = 0.010
PIN_LENGTH = 0.022
BOSS_RADIUS = 0.017
BOSS_LENGTH = 0.020


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_vent")

    frame_mat = Material("dark_powder_coated_frame", rgba=(0.08, 0.09, 0.095, 1.0))
    lip_mat = Material("slightly_worn_front_lip", rgba=(0.12, 0.13, 0.14, 1.0))
    blade_mat = Material("satin_rigid_blades", rgba=(0.55, 0.58, 0.58, 1.0))
    pin_mat = Material("dark_pivot_pins", rgba=(0.025, 0.025, 0.025, 1.0))

    frame = model.part("frame")

    # Four overlapping rails form one grounded, continuous rectangular perimeter.
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(-OUTER_WIDTH / 2.0 + SIDE_RAIL / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(OUTER_WIDTH / 2.0 - SIDE_RAIL / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_1",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, TOP_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, TOP_RAIL / 2.0)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, TOP_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT - TOP_RAIL / 2.0)),
        material=frame_mat,
        name="top_rail",
    )

    # A shallow proud front lip and rear sleeve make the part read as a service
    # register rather than a flat drawing.
    lip_depth = 0.010
    lip_y = -FRAME_DEPTH / 2.0 - lip_depth / 2.0
    frame.visual(
        Box((OUTER_WIDTH, lip_depth, 0.014)),
        origin=Origin(xyz=(0.0, lip_y, 0.014 / 2.0)),
        material=lip_mat,
        name="bottom_front_lip",
    )
    frame.visual(
        Box((OUTER_WIDTH, lip_depth, 0.014)),
        origin=Origin(xyz=(0.0, lip_y, OUTER_HEIGHT - 0.014 / 2.0)),
        material=lip_mat,
        name="top_front_lip",
    )
    frame.visual(
        Box((0.014, lip_depth, OUTER_HEIGHT)),
        origin=Origin(xyz=(-OUTER_WIDTH / 2.0 + 0.014 / 2.0, lip_y, OUTER_HEIGHT / 2.0)),
        material=lip_mat,
        name="side_front_lip_0",
    )
    frame.visual(
        Box((0.014, lip_depth, OUTER_HEIGHT)),
        origin=Origin(xyz=(OUTER_WIDTH / 2.0 - 0.014 / 2.0, lip_y, OUTER_HEIGHT / 2.0)),
        material=lip_mat,
        name="side_front_lip_1",
    )

    sleeve_y = FRAME_DEPTH / 2.0 + 0.012
    frame.visual(
        Box((SIDE_RAIL * 0.62, 0.024, OUTER_HEIGHT - 2.0 * TOP_RAIL)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - SIDE_RAIL * 0.31, sleeve_y, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="rear_sleeve_0",
    )
    frame.visual(
        Box((SIDE_RAIL * 0.62, 0.024, OUTER_HEIGHT - 2.0 * TOP_RAIL)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + SIDE_RAIL * 0.31, sleeve_y, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="rear_sleeve_1",
    )

    # Fixed bearing pads, slightly embedded into the side rails.  The moving
    # blade pins touch the inner boss faces without interpenetrating them.
    left_boss_x = -OPENING_WIDTH / 2.0 + BOSS_LENGTH / 4.0
    right_boss_x = OPENING_WIDTH / 2.0 - BOSS_LENGTH / 4.0
    for i in range(BLADE_COUNT):
        z = FIRST_BLADE_Z + i * BLADE_PITCH
        frame.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(xyz=(left_boss_x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lip_mat,
            name=f"pivot_boss_{i}_0",
        )
        frame.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(xyz=(right_boss_x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lip_mat,
            name=f"pivot_boss_{i}_1",
        )

    blade_profile = rounded_rect_profile(
        BLADE_THICKNESS,
        BLADE_DEPTH,
        radius=0.006,
        corner_segments=6,
    )

    for i in range(BLADE_COUNT):
        blade = model.part(f"blade_{i}")
        blade_geom = ExtrudeGeometry.centered(blade_profile, BLADE_LENGTH)
        blade_geom.rotate_y(math.pi / 2.0).rotate_x(BLADE_REST_ANGLE)
        blade.visual(
            mesh_from_geometry(blade_geom, f"blade_{i}_rounded_body"),
            origin=Origin(),
            material=blade_mat,
            name="body",
        )
        blade.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(-BLADE_LENGTH / 2.0 - PIN_LENGTH / 2.0 + 0.002, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pin_mat,
            name="pin_0",
        )
        blade.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(BLADE_LENGTH / 2.0 + PIN_LENGTH / 2.0 - 0.002, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pin_mat,
            name="pin_1",
        )
        model.articulation(
            f"frame_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, FIRST_BLADE_Z + i * BLADE_PITCH)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.55,
                upper=0.55,
                effort=2.0,
                velocity=2.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{i}") for i in range(BLADE_COUNT)]
    joints = [object_model.get_articulation(f"frame_to_blade_{i}") for i in range(BLADE_COUNT)]

    ctx.check(
        "five independent blade hinges",
        len(joints) == BLADE_COUNT and len({joint.name for joint in joints}) == BLADE_COUNT,
        details=f"joints={[joint.name for joint in joints]}",
    )
    for i, joint in enumerate(joints):
        limits = joint.motion_limits
        ctx.check(
            f"blade_{i} has bounded revolute motion",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == -0.55
            and limits.upper == 0.55,
            details=f"type={joint.articulation_type}, limits={limits}",
        )
        ctx.expect_contact(
            blades[i],
            frame,
            elem_a="pin_0",
            elem_b=f"pivot_boss_{i}_0",
            contact_tol=0.001,
            name=f"blade_{i} left pin seats on boss",
        )
        ctx.expect_contact(
            blades[i],
            frame,
            elem_a="pin_1",
            elem_b=f"pivot_boss_{i}_1",
            contact_tol=0.001,
            name=f"blade_{i} right pin seats on boss",
        )

    for i in range(BLADE_COUNT - 1):
        ctx.expect_gap(
            blades[i + 1],
            blades[i],
            axis="z",
            positive_elem="body",
            negative_elem="body",
            min_gap=0.006,
            name=f"clear air gap between blade_{i} and blade_{i + 1}",
        )

    moving_blade = blades[2]
    neighbor_blade = blades[1]
    rest_aabb = ctx.part_element_world_aabb(moving_blade, elem="body")
    neighbor_rest = ctx.part_element_world_aabb(neighbor_blade, elem="body")
    with ctx.pose({joints[2]: 0.45}):
        moved_aabb = ctx.part_element_world_aabb(moving_blade, elem="body")
        neighbor_moved = ctx.part_element_world_aabb(neighbor_blade, elem="body")
    ctx.check(
        "one blade can rotate independently",
        rest_aabb is not None
        and moved_aabb is not None
        and neighbor_rest is not None
        and neighbor_moved is not None
        and abs(moved_aabb[1][2] - rest_aabb[1][2]) > 0.006
        and abs(neighbor_moved[1][1] - neighbor_rest[1][1]) < 0.001,
        details=f"rest={rest_aabb}, moved={moved_aabb}, neighbor_rest={neighbor_rest}, neighbor_moved={neighbor_moved}",
    )

    return ctx.report()


object_model = build_object_model()
