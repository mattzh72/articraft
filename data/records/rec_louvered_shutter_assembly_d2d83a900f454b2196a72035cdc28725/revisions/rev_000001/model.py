from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_WIDTH = 0.68
OUTER_HEIGHT = 1.40
FRAME_DEPTH = 0.052
STILE_WIDTH = 0.065
RAIL_HEIGHT = 0.075

INNER_WIDTH = OUTER_WIDTH - 2.0 * STILE_WIDTH
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * RAIL_HEIGHT
INNER_HALF_X = INNER_WIDTH / 2.0
OUTER_HALF_X = OUTER_WIDTH / 2.0

LOUVER_COUNT = 13
LOUVER_PITCH = 0.092
LOUVER_ZS = tuple((i - (LOUVER_COUNT - 1) / 2.0) * LOUVER_PITCH for i in range(LOUVER_COUNT))
LOUVER_BLADE_LENGTH = INNER_WIDTH - 0.040
LOUVER_CHORD = 0.080
LOUVER_THICKNESS = 0.014
PIVOT_PIN_RADIUS = 0.006
PIVOT_PIN_LENGTH = (INNER_WIDTH - LOUVER_BLADE_LENGTH) / 2.0
PIVOT_HOLE_RADIUS = 0.008

TILT_X = -INNER_HALF_X + 0.072
TILT_Y = 0.082
TILT_TRAVEL = 0.028
LOUVER_MIMIC_GAIN = -23.0


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _frame_shape() -> cq.Workplane:
    left_x = -OUTER_HALF_X + STILE_WIDTH / 2.0
    right_x = OUTER_HALF_X - STILE_WIDTH / 2.0
    top_z = OUTER_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0
    bottom_z = -OUTER_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0

    frame = _box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT), (left_x, 0.0, 0.0))
    frame = frame.union(_box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT), (right_x, 0.0, 0.0)))
    frame = frame.union(_box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT), (0.0, 0.0, top_z)))
    frame = frame.union(_box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT), (0.0, 0.0, bottom_z)))

    # Small molded pivot bosses on the inside faces of both stiles.
    for z in LOUVER_ZS:
        frame = frame.union(_box((0.014, FRAME_DEPTH + 0.006, 0.034), (-INNER_HALF_X + 0.007, 0.0, z)))
        frame = frame.union(_box((0.014, FRAME_DEPTH + 0.006, 0.034), (INNER_HALF_X - 0.007, 0.0, z)))

    # Two short guide channels hold the full-height tilt rod while letting it slide.
    guide_height = 0.205
    guide_rail_w = 0.007
    guide_rail_d = 0.020
    guide_offset = 0.0115
    for zc in (-0.47, 0.47):
        for sx in (-1.0, 1.0):
            frame = frame.union(
                _box(
                    (guide_rail_w, guide_rail_d, guide_height),
                    (TILT_X + sx * guide_offset, TILT_Y, zc),
                )
            )
        post_y = (FRAME_DEPTH / 2.0 + TILT_Y) / 2.0
        post_depth = TILT_Y - FRAME_DEPTH / 2.0 + 0.010
        frame = frame.union(
            _box(
                (0.018, post_depth, 0.024),
                (-INNER_HALF_X - 0.006, post_y, zc),
            )
        )
        arm_start = -INNER_HALF_X - 0.012
        arm_end = TILT_X - guide_offset + 0.006
        arm_len = arm_end - arm_start
        frame = frame.union(
            _box(
                (arm_len, 0.018, 0.018),
                (arm_start + arm_len / 2.0, TILT_Y, zc),
            )
        )

    # Clearance holes through the side-stile bosses keep the pivot pins captured
    # without solid interpenetration.
    hole_len = STILE_WIDTH + 0.030
    for z in LOUVER_ZS:
        frame = frame.cut(_x_cylinder(PIVOT_HOLE_RADIUS, hole_len, (-INNER_HALF_X - STILE_WIDTH / 2.0, 0.0, z)))
        frame = frame.cut(_x_cylinder(PIVOT_HOLE_RADIUS, hole_len, (INNER_HALF_X + STILE_WIDTH / 2.0, 0.0, z)))

    return frame


def _louver_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("YZ")
        .ellipse(LOUVER_CHORD / 2.0, LOUVER_THICKNESS / 2.0)
        .extrude(LOUVER_BLADE_LENGTH / 2.0, both=True)
    )
    axle = _x_cylinder(
        PIVOT_PIN_RADIUS,
        LOUVER_BLADE_LENGTH + 2.0 * PIVOT_PIN_LENGTH,
        (0.0, 0.0, 0.0),
    )
    tab = _box((0.030, 0.030, 0.018), (TILT_X, 0.040 + 0.030 / 2.0, 0.0))
    return blade.union(axle).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter_panel")

    painted_wood = Material("warm_white_painted_wood", rgba=(0.93, 0.91, 0.84, 1.0))
    brass = Material("aged_brass_pin", rgba=(0.72, 0.56, 0.30, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(-OUTER_HALF_X + STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="stile_0",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(OUTER_HALF_X - STILE_WIDTH / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="stile_1",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)),
        material=painted_wood,
        name="top_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -OUTER_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        material=painted_wood,
        name="bottom_rail",
    )
    for i, z in enumerate(LOUVER_ZS):
        for side in (-1.0, 1.0):
            x = side * (INNER_HALF_X + 0.002)
            for loc, center, size in (
                ("upper", (x, 0.0, z + PIVOT_PIN_RADIUS + 0.003), (0.016, 0.042, 0.006)),
                ("lower", (x, 0.0, z - PIVOT_PIN_RADIUS - 0.003), (0.016, 0.042, 0.006)),
                ("front", (x, PIVOT_PIN_RADIUS + 0.004, z), (0.016, 0.006, 0.026)),
                ("rear", (x, -PIVOT_PIN_RADIUS - 0.004, z), (0.016, 0.006, 0.026)),
            ):
                frame.visual(
                    Box(size),
                    origin=Origin(xyz=center),
                    material=painted_wood,
                    name=f"pivot_{i}_{'neg' if side < 0 else 'pos'}_{loc}",
                )

    guide_offset = 0.0115
    for gi, zc in enumerate((-0.506, 0.506)):
        for side in (-1.0, 1.0):
            frame.visual(
                Box((0.007, 0.020, 0.205)),
                origin=Origin(xyz=(TILT_X + side * guide_offset, TILT_Y, zc)),
                material=painted_wood,
                name=f"guide_{gi}_{'neg' if side < 0 else 'pos'}_rail",
            )
        post_y = (FRAME_DEPTH / 2.0 + TILT_Y) / 2.0
        post_depth = TILT_Y - FRAME_DEPTH / 2.0 + 0.010
        frame.visual(
            Box((0.018, post_depth, 0.024)),
            origin=Origin(xyz=(-INNER_HALF_X - 0.006, post_y, zc)),
            material=painted_wood,
            name=f"guide_{gi}_standoff",
        )
        arm_start = -INNER_HALF_X - 0.012
        arm_end = TILT_X + guide_offset + 0.006
        arm_len = arm_end - arm_start
        frame.visual(
            Box((arm_len, 0.014, 0.018)),
            origin=Origin(xyz=(arm_start + arm_len / 2.0, TILT_Y - 0.017, zc)),
            material=painted_wood,
            name=f"guide_{gi}_arm",
        )
    louver_mesh = mesh_from_cadquery(_louver_shape(), "rounded_louver_with_end_pivots", tolerance=0.0008)
    louver_limit = abs(LOUVER_MIMIC_GAIN * TILT_TRAVEL)
    louver_driver_joint = "frame_to_louver_0"

    rod = model.part("tilt_rod")
    rod_height = INNER_HEIGHT - 0.040
    rod.visual(
        Box((0.016, 0.016, rod_height)),
        origin=Origin(),
        material=painted_wood,
        name="vertical_rod",
    )
    for i, z in enumerate(LOUVER_ZS):
        rod.visual(
            Box((0.018, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.008, z)),
            material=brass,
            name=f"link_pin_{i}",
        )

    rod_joint = model.articulation(
        "frame_to_tilt_rod",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rod,
        origin=Origin(xyz=(TILT_X, TILT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=-TILT_TRAVEL, upper=TILT_TRAVEL),
    )

    for i, z in enumerate(LOUVER_ZS):
        louver = model.part(f"louver_{i}")
        louver.visual(louver_mesh, material=painted_wood, name="louver_body")
        joint_name = f"frame_to_louver_{i}"
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-louver_limit, upper=louver_limit),
            mimic=None if i == 0 else Mimic(joint=louver_driver_joint, multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rod = object_model.get_part("tilt_rod")
    rod_joint = object_model.get_articulation("frame_to_tilt_rod")
    louver_driver = object_model.get_articulation("frame_to_louver_0")

    ctx.check(
        "stack has full set of louvers",
        sum(1 for p in object_model.parts if p.name.startswith("louver_")) == LOUVER_COUNT,
        details="Expected a full-height stack of separately articulated louvers.",
    )
    ctx.expect_within(
        rod,
        frame,
        axes="x",
        margin=0.0,
        inner_elem="vertical_rod",
        name="tilt rod stays inside frame width",
    )
    ctx.expect_overlap(
        rod,
        frame,
        axes="z",
        min_overlap=1.0,
        elem_a="vertical_rod",
        name="tilt rod is full-height within the panel",
    )

    for idx in (0, LOUVER_COUNT // 2, LOUVER_COUNT - 1):
        louver = object_model.get_part(f"louver_{idx}")
        ctx.expect_within(
            louver,
            frame,
            axes="x",
            margin=0.002,
            inner_elem="louver_body",
            name=f"louver_{idx} end pivots remain within the side stiles",
        )
        ctx.expect_overlap(
            louver,
            frame,
            axes="x",
            min_overlap=0.004,
            elem_a="louver_body",
            elem_b=f"pivot_{idx}_neg_front",
            name=f"louver_{idx} pivot pins project into stile sockets",
        )

    rest_rod_pos = ctx.part_world_position(rod)
    mid_louver = object_model.get_part(f"louver_{LOUVER_COUNT // 2}")
    rest_box = ctx.part_element_world_aabb(mid_louver, elem="louver_body")
    with ctx.pose({rod_joint: TILT_TRAVEL, louver_driver: LOUVER_MIMIC_GAIN * TILT_TRAVEL}):
        raised_rod_pos = ctx.part_world_position(rod)
        tilted_box = ctx.part_element_world_aabb(mid_louver, elem="louver_body")

    ctx.check(
        "tilt rod slides upward on its guide",
        rest_rod_pos is not None
        and raised_rod_pos is not None
        and raised_rod_pos[2] > rest_rod_pos[2] + TILT_TRAVEL * 0.9,
        details=f"rest={rest_rod_pos}, raised={raised_rod_pos}",
    )
    if rest_box is not None and tilted_box is not None:
        rest_z = rest_box[1][2] - rest_box[0][2]
        tilted_z = tilted_box[1][2] - tilted_box[0][2]
        ctx.check(
            "linked louver rotates as tilt rod moves",
            tilted_z > rest_z + 0.025,
            details=f"rest_z_extent={rest_z:.4f}, tilted_z_extent={tilted_z:.4f}",
        )
    else:
        ctx.fail("linked louver rotates as tilt rod moves", "Could not measure louver element AABB.")

    return ctx.report()


object_model = build_object_model()
