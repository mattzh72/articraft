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


ORANGE = Material("powder_coated_safety_orange", (0.95, 0.38, 0.05, 1.0))
DARK_ORANGE = Material("reinforced_orange_shadow", (0.70, 0.22, 0.03, 1.0))
GALVANIZED = Material("galvanized_steel", (0.62, 0.65, 0.64, 1.0))
BLACK_RUBBER = Material("black_molded_rubber", (0.015, 0.014, 0.012, 1.0))
DARK_TREAD = Material("dark_grit_tread", (0.06, 0.065, 0.06, 1.0))


H = 1.45
FRONT_FOOT_X = -0.38
REAR_FOOT_X = 0.58


def _leg_angle(dx: float, dz: float) -> float:
    """Rotation about Y that aligns a box local +Z with an XZ vector."""
    return math.atan2(dx, dz)


def _cyl_y(part, *, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_simple_aframe_step_ladder")

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # --- Front climbing frame -------------------------------------------------
    # The part frame sits on the top hinge axis.  All floor contacts are at
    # local z ~= -H, so the open ladder stands at a believable full-size height.
    front_len = math.sqrt(FRONT_FOOT_X * FRONT_FOOT_X + H * H)
    front_rail_angle = _leg_angle(-FRONT_FOOT_X, H)
    for side in (-1.0, 1.0):
        suffix = "0" if side < 0 else "1"
        front.visual(
            Box((0.070, 0.055, front_len)),
            origin=Origin(
                xyz=(FRONT_FOOT_X / 2.0, side * 0.31, -H / 2.0),
                rpy=(0.0, front_rail_angle, 0.0),
            ),
            material=ORANGE,
            name=f"front_rail_{suffix}",
        )
        front.visual(
            Box((0.185, 0.140, 0.055)),
            origin=Origin(xyz=(FRONT_FOOT_X - 0.015, side * 0.31, -H + 0.028)),
            material=BLACK_RUBBER,
            name=f"front_foot_{suffix}",
        )
        front.visual(
            Box((0.095, 0.016, 0.170)),
            origin=Origin(
                xyz=(-0.065, side * 0.354, -0.105),
                rpy=(0.0, 0.18 * side, 0.0),
            ),
            material=DARK_ORANGE,
            name=f"top_side_plate_{suffix}",
        )

    # Continuous hinge barrel carried by the front frame.
    _cyl_y(front, name="top_hinge_tube", radius=0.035, length=0.68, xyz=(0.0, 0.0, 0.0), material=GALVANIZED)

    # Molded utility top with raised lips and dark tool-well inserts.
    front.visual(
        Box((0.380, 0.720, 0.058)),
        origin=Origin(xyz=(-0.085, 0.0, -0.045)),
        material=ORANGE,
        name="top_platform",
    )
    for name, xyz, size in (
        ("top_front_lip", (-0.285, 0.0, -0.010), (0.040, 0.720, 0.070)),
        ("top_rear_lip", (0.105, 0.0, -0.010), (0.040, 0.720, 0.070)),
        ("top_side_lip_0", (-0.085, -0.338, -0.010), (0.380, 0.035, 0.070)),
        ("top_side_lip_1", (-0.085, 0.338, -0.010), (0.380, 0.035, 0.070)),
    ):
        front.visual(Box(size), origin=Origin(xyz=xyz), material=DARK_ORANGE, name=name)
    for i, (x, y, r) in enumerate(((-0.16, -0.18, 0.038), (-0.16, 0.18, 0.038), (0.005, 0.0, 0.050))):
        front.visual(
            Cylinder(radius=r, length=0.007),
            origin=Origin(xyz=(x, y, -0.019)),
            material=DARK_TREAD,
            name=f"tool_well_{i}",
        )

    # Practical wide treads with upturned front lips, gritty ribs, and visible
    # steel rivets.  The deck overlaps the side rails like a bolted utility
    # ladder tread, keeping the front frame a single supported assembly.
    tread_levels = (0.30, 0.60, 0.90, 1.18)
    for ti, level in enumerate(tread_levels):
        z = level - H
        rail_x = FRONT_FOOT_X * (1.0 - level / H)
        x = rail_x + 0.060
        front.visual(
            Box((0.285, 0.700, 0.046)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=ORANGE,
            name=f"tread_{ti}",
        )
        front.visual(
            Box((0.032, 0.700, 0.070)),
            origin=Origin(xyz=(x - 0.154, 0.0, z + 0.014)),
            material=DARK_ORANGE,
            name=f"tread_{ti}_front_lip",
        )
        front.visual(
            Box((0.022, 0.665, 0.050)),
            origin=Origin(xyz=(x + 0.148, 0.0, z + 0.006)),
            material=DARK_ORANGE,
            name=f"tread_{ti}_rear_web",
        )
        for ri, rib_x in enumerate((-0.090, -0.045, 0.000, 0.045, 0.090)):
            front.visual(
                Box((0.012, 0.610, 0.008)),
                origin=Origin(xyz=(x + rib_x, 0.0, z + 0.027)),
                material=DARK_TREAD,
                name=f"tread_{ti}_rib_{ri}",
            )
        for bi, (bx, by) in enumerate(((x - 0.105, -0.287), (x + 0.105, -0.287), (x - 0.105, 0.287), (x + 0.105, 0.287))):
            front.visual(
                Cylinder(radius=0.012, length=0.009),
                origin=Origin(xyz=(bx, by, z + 0.031)),
                material=GALVANIZED,
                name=f"tread_{ti}_rivet_{bi}",
            )

    # Long side pins for the folding spreader bars.  They visibly bridge from
    # the ladder rails out to the external brace straps.
    for side in (-1.0, 1.0):
        suffix = "0" if side < 0 else "1"
        y_mid = side * 0.408
        _cyl_y(
            front,
            name=f"brace_pin_{suffix}",
            radius=0.016,
            length=0.165,
            xyz=(-0.204, y_mid, -0.780),
            material=GALVANIZED,
        )

    # --- Rear support frame ---------------------------------------------------
    rear_len = math.sqrt(REAR_FOOT_X * REAR_FOOT_X + H * H)
    rear_rail_angle = _leg_angle(-REAR_FOOT_X, H)
    for side in (-1.0, 1.0):
        suffix = "0" if side < 0 else "1"
        rear.visual(
            Box((0.066, 0.055, rear_len)),
            origin=Origin(
                xyz=(REAR_FOOT_X / 2.0, side * 0.405, -H / 2.0),
                rpy=(0.0, rear_rail_angle, 0.0),
            ),
            material=ORANGE,
            name=f"rear_rail_{suffix}",
        )
        rear.visual(
            Box((0.190, 0.140, 0.055)),
            origin=Origin(xyz=(REAR_FOOT_X + 0.010, side * 0.405, -H + 0.028)),
            material=BLACK_RUBBER,
            name=f"rear_foot_{suffix}",
        )
        _cyl_y(
            rear,
            name=f"hinge_lug_{suffix}",
            radius=0.037,
            length=0.094,
            xyz=(0.0, side * 0.410, 0.0),
            material=GALVANIZED,
        )
        _cyl_y(
            rear,
            name=f"hinge_washer_{suffix}",
            radius=0.052,
            length=0.018,
            xyz=(0.0, side * 0.462, 0.0),
            material=GALVANIZED,
        )

    for ci, local_z in enumerate((-0.46, -0.82, -1.18)):
        x = -REAR_FOOT_X * local_z / H
        rear.visual(
            Box((0.055, 0.840, 0.052)),
            origin=Origin(xyz=(x, 0.0, local_z)),
            material=ORANGE,
            name=f"rear_crossbar_{ci}",
        )
        rear.visual(
            Box((0.030, 0.780, 0.016)),
            origin=Origin(xyz=(x + 0.018, 0.0, local_z + 0.036)),
            material=DARK_ORANGE,
            name=f"rear_crossbar_rib_{ci}",
        )

    for side in (-1.0, 1.0):
        suffix = "0" if side < 0 else "1"
        _cyl_y(
            rear,
            name=f"brace_pin_{suffix}",
            radius=0.016,
            length=0.100,
            xyz=(0.350, side * 0.445, -0.900),
            material=GALVANIZED,
        )

    # --- Articulated spread-limit braces -------------------------------------
    # These are outside the side rails.  Each strap is hinged at the front and
    # overlaps only its captured steel pins at the front and rear eyes.
    front_brace_pivot = (-0.204, 0.0, -0.780)
    rear_brace_pivot = (0.350, 0.0, -0.900)
    brace_dx = rear_brace_pivot[0] - front_brace_pivot[0]
    brace_dz = rear_brace_pivot[2] - front_brace_pivot[2]
    brace_len = math.sqrt(brace_dx * brace_dx + brace_dz * brace_dz)
    brace_angle = math.atan2(-brace_dz, brace_dx)

    braces = []
    for side in (-1.0, 1.0):
        suffix = "0" if side < 0 else "1"
        brace = model.part(f"brace_{suffix}")
        braces.append((brace, suffix, side))
        brace.visual(
            Box((brace_len - 0.070, 0.026, 0.018)),
            origin=Origin(
                xyz=(brace_dx / 2.0, 0.0, brace_dz / 2.0),
                rpy=(0.0, brace_angle, 0.0),
            ),
            material=GALVANIZED,
            name="strap",
        )
        brace.visual(
            Cylinder(radius=0.040, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=GALVANIZED,
            name="front_eye",
        )
        brace.visual(
            Cylinder(radius=0.040, length=0.028),
            origin=Origin(xyz=(brace_dx, 0.0, brace_dz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=GALVANIZED,
            name="rear_eye",
        )
        brace.visual(
            Box((0.070, 0.030, 0.026)),
            origin=Origin(xyz=(brace_dx * 0.50, 0.0, brace_dz * 0.50 + 0.012)),
            material=DARK_ORANGE,
            name="lock_tab",
        )

    model.articulation(
        "frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=0.20),
    )
    for brace, suffix, side in braces:
        model.articulation(
            f"brace_hinge_{suffix}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(front_brace_pivot[0], side * 0.490, front_brace_pivot[2])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    frame_hinge = object_model.get_articulation("frame_hinge")

    # The brace eyes are intentionally modeled as captured over steel hinge pins.
    # These local overlaps are the visible pin/eye fit, not broad frame collisions.
    for suffix in ("0", "1"):
        brace = object_model.get_part(f"brace_{suffix}")
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_pin_{suffix}",
            elem_b="front_eye",
            reason="The spreader brace front eye is captured around the exposed steel hinge pin.",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"brace_pin_{suffix}",
            elem_b="rear_eye",
            reason="The spreader brace rear eye is shown latched around the rear limit pin.",
        )
        ctx.expect_overlap(
            front,
            brace,
            axes="xz",
            min_overlap=0.010,
            elem_a=f"brace_pin_{suffix}",
            elem_b="front_eye",
            name=f"brace {suffix} front eye surrounds pin",
        )
        ctx.expect_overlap(
            rear,
            brace,
            axes="xz",
            min_overlap=0.010,
            elem_a=f"brace_pin_{suffix}",
            elem_b="rear_eye",
            name=f"brace {suffix} rear eye surrounds pin",
        )

    front_aabb = ctx.part_world_aabb(front)
    rear_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "rear feet stand behind front climbing frame",
        front_aabb is not None
        and rear_aabb is not None
        and rear_aabb[1][0] > front_aabb[1][0] + 0.38
        and rear_aabb[0][0] > front_aabb[0][0] + 0.20,
        details=f"front_aabb={front_aabb}, rear_aabb={rear_aabb}",
    )
    ctx.expect_overlap(front, rear, axes="y", min_overlap=0.45, name="ladder frames share a stable working width")

    with ctx.pose({frame_hinge: 0.20}):
        rear_folded_aabb = ctx.part_world_aabb(rear)
        ctx.expect_overlap(front, rear, axes="y", min_overlap=0.45, name="folded hinge stays coaxial across width")

    ctx.check(
        "rear frame folds toward front frame",
        rear_aabb is not None and rear_folded_aabb is not None and rear_folded_aabb[1][0] < rear_aabb[1][0] - 0.05,
        details=f"rest={rear_aabb}, folded={rear_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
