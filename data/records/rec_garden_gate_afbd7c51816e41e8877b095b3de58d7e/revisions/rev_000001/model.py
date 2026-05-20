from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
    wire_from_points,
)


POST_SIZE = 0.09
POST_DEPTH = 0.09
POST_ABOVE_GROUND = 0.92
POST_BURIED = 0.20
POST_TOTAL_HEIGHT = POST_ABOVE_GROUND + POST_BURIED

OPENING_WIDTH = 1.02

GATE_WIDTH = 0.94
GATE_HEIGHT = 0.76
GATE_THICKNESS = 0.036
GATE_BOTTOM = 0.10
GATE_MID_Z = GATE_BOTTOM + GATE_HEIGHT / 2.0

STILE_WIDTH = 0.03
RAIL_HEIGHT = 0.08


def _front_face_y() -> float:
    return -GATE_THICKNESS / 2.0


def _make_bolt_staple(*, z: float, name: str):
    face_y = _front_face_y()
    staple = wire_from_points(
        [
            (0.0, face_y + 0.0015, z - 0.017),
            (0.0, face_y - 0.0130, z - 0.017),
            (0.0, face_y - 0.0130, z + 0.017),
            (0.0, face_y + 0.0015, z + 0.017),
        ],
        radius=0.0022,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.006,
        corner_segments=10,
    )
    return mesh_from_geometry(staple, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_picket_gate")

    wood = model.material("painted_wood", rgba=(0.84, 0.84, 0.79, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.78, 0.80, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.68, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((POST_SIZE, POST_DEPTH, POST_TOTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                -POST_SIZE / 2.0,
                0.0,
                (POST_ABOVE_GROUND - POST_BURIED) / 2.0,
            )
        ),
        material=weathered_wood,
        name="left_post",
    )
    frame.visual(
        Box((POST_SIZE, POST_DEPTH, POST_TOTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH + POST_SIZE / 2.0,
                0.0,
                (POST_ABOVE_GROUND - POST_BURIED) / 2.0,
            )
        ),
        material=weathered_wood,
        name="right_post",
    )
    frame.visual(
        Box((OPENING_WIDTH + POST_SIZE, 0.06, 0.08)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0, 0.0, -0.16)),
        material=weathered_wood,
        name="buried_tie",
    )
    frame.visual(
        Box((0.06, 0.008, 0.09)),
        origin=Origin(xyz=(OPENING_WIDTH - 0.006, _front_face_y(), GATE_MID_Z + 0.18)),
        material=dark_metal,
        name="latch_keeper",
    )

    leaf = model.part("leaf")
    leaf.visual(
        Box((STILE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        origin=Origin(xyz=(STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=wood,
        name="left_stile",
    )
    leaf.visual(
        Box((STILE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        origin=Origin(xyz=(GATE_WIDTH - STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=wood,
        name="right_stile",
    )
    leaf.visual(
        Box((GATE_WIDTH - 2.0 * STILE_WIDTH + 0.01, GATE_THICKNESS * 0.92, RAIL_HEIGHT)),
        origin=Origin(xyz=(GATE_WIDTH / 2.0, 0.0, GATE_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)),
        material=wood,
        name="upper_rail",
    )
    leaf.visual(
        Box((GATE_WIDTH - 2.0 * STILE_WIDTH + 0.01, GATE_THICKNESS * 0.92, RAIL_HEIGHT)),
        origin=Origin(xyz=(GATE_WIDTH / 2.0, 0.0, -GATE_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        material=wood,
        name="lower_rail",
    )

    picket_count = 8
    inner_left = STILE_WIDTH + 0.065
    inner_right = GATE_WIDTH - STILE_WIDTH - 0.065
    picket_span = inner_right - inner_left
    picket_step = picket_span / (picket_count - 1)
    for index in range(picket_count):
        x_pos = inner_left + index * picket_step
        leaf.visual(
            Box((0.055, GATE_THICKNESS * 0.50, GATE_HEIGHT - 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=wood,
            name=f"picket_{index + 1}",
        )

    brace_dx = GATE_WIDTH - 0.22
    brace_dz = GATE_HEIGHT - 0.20
    brace_length = math.sqrt(brace_dx * brace_dx + brace_dz * brace_dz)
    brace_angle = math.atan2(brace_dz, brace_dx)
    leaf.visual(
        Box((brace_length, 0.018, 0.05)),
        origin=Origin(
            xyz=(GATE_WIDTH / 2.0, 0.010, -0.01),
            rpy=(0.0, -brace_angle, 0.0),
        ),
        material=weathered_wood,
        name="diagonal_brace",
    )

    for z_pos, name in ((0.22, "upper_hinge_strap"), (-0.22, "lower_hinge_strap")):
        leaf.visual(
            Box((0.26, 0.006, 0.06)),
            origin=Origin(xyz=(0.13, _front_face_y() - 0.001, z_pos)),
            material=dark_metal,
            name=name,
        )

    latch_plate_x = GATE_WIDTH - 0.022
    latch_plate_z = 0.18
    leaf.visual(
        Box((0.042, 0.006, 0.08)),
        origin=Origin(xyz=(latch_plate_x, _front_face_y() - 0.001, latch_plate_z)),
        material=dark_metal,
        name="latch_backplate",
    )

    bolt_x = GATE_WIDTH - 0.022
    leaf.visual(
        _make_bolt_staple(z=-0.17, name="upper_bolt_staple_mesh"),
        origin=Origin(xyz=(bolt_x, 0.0, 0.0)),
        material=galvanized,
        name="upper_bolt_staple",
    )
    leaf.visual(
        _make_bolt_staple(z=-0.29, name="lower_bolt_staple_mesh"),
        origin=Origin(xyz=(bolt_x, 0.0, 0.0)),
        material=galvanized,
        name="lower_bolt_staple",
    )

    ring_latch = model.part("ring_latch")
    ring_mesh = TorusGeometry(radius=0.026, tube=0.0045)
    ring_mesh.rotate_x(math.pi / 2.0)
    ring_latch.visual(
        mesh_from_geometry(ring_mesh, "latch_ring_mesh"),
        origin=Origin(xyz=(0.0, -0.006, -0.042)),
        material=dark_metal,
        name="latch_ring",
    )
    ring_latch.visual(
        Box((0.016, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, -0.008)),
        material=dark_metal,
        name="latch_lug",
    )
    ring_latch.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_pivot_pin",
    )

    drop_bolt = model.part("drop_bolt")
    drop_bolt.visual(
        Cylinder(radius=0.005, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=galvanized,
        name="bolt_rod",
    )
    drop_bolt.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=galvanized,
        name="bolt_stop_collar",
    )
    drop_bolt.visual(
        Cylinder(radius=0.0032, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="bolt_handle",
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, GATE_MID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "leaf_to_ring_latch",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=ring_latch,
        origin=Origin(xyz=(latch_plate_x, _front_face_y() - 0.004, latch_plate_z + 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.9,
            upper=0.9,
        ),
    )
    model.articulation(
        "leaf_to_drop_bolt",
        ArticulationType.PRISMATIC,
        parent=leaf,
        child=drop_bolt,
        origin=Origin(xyz=(bolt_x, _front_face_y() - 0.013, -0.17)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    leaf = object_model.get_part("leaf")
    ring_latch = object_model.get_part("ring_latch")
    drop_bolt = object_model.get_part("drop_bolt")

    gate_hinge = object_model.get_articulation("frame_to_leaf")
    latch_joint = object_model.get_articulation("leaf_to_ring_latch")
    bolt_joint = object_model.get_articulation("leaf_to_drop_bolt")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (frame, leaf, ring_latch, drop_bolt)),
        details="Expected frame, leaf, ring latch, and drop bolt parts.",
    )
    ctx.check(
        "gate hinge is vertical",
        tuple(gate_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={gate_hinge.axis}",
    )
    ctx.check(
        "ring latch pivots through gate thickness",
        tuple(latch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={latch_joint.axis}",
    )
    ctx.check(
        "drop bolt slides downward",
        tuple(bolt_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={bolt_joint.axis}",
    )

    with ctx.pose({gate_hinge: 0.0, latch_joint: 0.0, bolt_joint: 0.0}):
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="right_post",
            min_gap=0.03,
            max_gap=0.10,
            name="closed leaf clears the latch post",
        )

        closed_leaf_aabb = ctx.part_world_aabb(leaf)
        closed_ring_aabb = ctx.part_element_world_aabb(ring_latch, elem="latch_ring")
        bolt_rest = ctx.part_world_position(drop_bolt)

    with ctx.pose({gate_hinge: 1.10}):
        open_leaf_aabb = ctx.part_world_aabb(leaf)

    with ctx.pose({latch_joint: 0.70}):
        rotated_ring_aabb = ctx.part_element_world_aabb(ring_latch, elem="latch_ring")

    with ctx.pose({bolt_joint: bolt_joint.motion_limits.upper}):
        bolt_extended = ctx.part_world_position(drop_bolt)

    leaf_swings_open = (
        closed_leaf_aabb is not None
        and open_leaf_aabb is not None
        and open_leaf_aabb[1][1] > closed_leaf_aabb[1][1] + 0.25
    )
    ctx.check(
        "leaf swings outward on the hinge",
        leaf_swings_open,
        details=f"closed={closed_leaf_aabb}, open={open_leaf_aabb}",
    )

    if closed_ring_aabb is not None and rotated_ring_aabb is not None:
        closed_ring_center_x = (closed_ring_aabb[0][0] + closed_ring_aabb[1][0]) / 2.0
        rotated_ring_center_x = (rotated_ring_aabb[0][0] + rotated_ring_aabb[1][0]) / 2.0
        ring_moved = abs(rotated_ring_center_x - closed_ring_center_x) > 0.015
    else:
        ring_moved = False
    ctx.check(
        "ring latch rotates about its pivot",
        ring_moved,
        details=f"closed={closed_ring_aabb}, rotated={rotated_ring_aabb}",
    )

    bolt_drops = (
        bolt_rest is not None
        and bolt_extended is not None
        and bolt_extended[2] < bolt_rest[2] - 0.08
    )
    ctx.check(
        "drop bolt extends downward",
        bolt_drops,
        details=f"rest={bolt_rest}, extended={bolt_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
