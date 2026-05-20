from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_RADIUS = 0.007
FRAME_RADIUS = 0.009
HINGE_RADIUS = 0.010


def _rod_origin(p1: tuple[float, float, float], p2: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("rod endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5, (p1[2] + p2[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_rod(part, name: str, p1, p2, radius: float, material) -> None:
    origin, length = _rod_origin(p1, p2)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_central_frame(central, rail_mat, hinge_mat) -> None:
    z = 0.85
    half_x = 0.44
    half_y = 0.26

    # Slim, powder-coated outer rectangle.
    _add_rod(central, "front_tube", (-half_x, -half_y, z), (half_x, -half_y, z), FRAME_RADIUS, rail_mat)
    _add_rod(central, "rear_tube", (-half_x, half_y, z), (half_x, half_y, z), FRAME_RADIUS, rail_mat)
    _add_rod(central, "side_tube_0", (-half_x, -half_y, z), (-half_x, half_y, z), FRAME_RADIUS, rail_mat)
    _add_rod(central, "side_tube_1", (half_x, -half_y, z), (half_x, half_y, z), FRAME_RADIUS, rail_mat)

    # Thin hanging rails across the central span.
    for i, x in enumerate([-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30]):
        _add_rod(central, f"center_rail_{i}", (x, -half_y, z), (x, half_y, z), RAIL_RADIUS, rail_mat)

    # Alternating hinge knuckles along the side hinge lines.  They are tied back
    # to the side tubes with short tabs, leaving the middle gap for the wing
    # knuckle so the joint reads as a real hinge rather than two overlapping rods.
    for side, x in enumerate([-0.465, 0.465]):
        side_tube_x = -half_x if x < 0.0 else half_x
        _add_rod(central, f"wing_pin_{side}", (x, -half_y, z), (x, half_y, z), 0.004, hinge_mat)
        for j, (y0, y1) in enumerate([(-0.245, -0.130), (0.130, 0.245)]):
            y_mid = (y0 + y1) * 0.5
            _add_rod(central, f"wing_hinge_{side}_{j}", (x, y0, z), (x, y1, z), HINGE_RADIUS, hinge_mat)
            _add_rod(
                central,
                f"wing_tab_{side}_{j}",
                (side_tube_x, y_mid, z),
                (x, y_mid, z),
                0.0055,
                hinge_mat,
            )

    # Underside hinge for the folding lower support frame, below the rear rail.
    hinge_y = -0.300
    hinge_z = 0.810
    _add_rod(central, "support_pin", (-0.420, hinge_y, hinge_z), (0.420, hinge_y, hinge_z), 0.004, hinge_mat)
    for j, (x0, x1) in enumerate([(-0.405, -0.185), (0.185, 0.405)]):
        x_mid = (x0 + x1) * 0.5
        _add_rod(central, f"support_hinge_{j}", (x0, hinge_y, hinge_z), (x1, hinge_y, hinge_z), HINGE_RADIUS, hinge_mat)
        _add_rod(
            central,
            f"support_tab_{j}",
            (x_mid, -half_y, z),
            (x_mid, hinge_y, hinge_z),
            0.0055,
            hinge_mat,
        )


def _add_wing_frame(wing, rail_mat, hinge_mat) -> None:
    half_y = 0.26
    inner_x = 0.040
    outer_x = 0.420

    # Wing hinge knuckle occupies the center gap between the fixed knuckles.
    _add_rod(wing, "hinge_barrel", (0.0, -0.080, 0.0), (0.0, 0.080, 0.0), HINGE_RADIUS, hinge_mat)
    _add_rod(wing, "hinge_web", (0.008, 0.0, -0.006), (inner_x, 0.0, 0.0), 0.0055, hinge_mat)

    _add_rod(wing, "inner_tube", (inner_x, -half_y, 0.0), (inner_x, half_y, 0.0), FRAME_RADIUS, rail_mat)
    _add_rod(wing, "outer_tube", (outer_x, -half_y, 0.0), (outer_x, half_y, 0.0), FRAME_RADIUS, rail_mat)
    _add_rod(wing, "front_tube", (inner_x, -half_y, 0.0), (outer_x, -half_y, 0.0), FRAME_RADIUS, rail_mat)
    _add_rod(wing, "rear_tube", (inner_x, half_y, 0.0), (outer_x, half_y, 0.0), FRAME_RADIUS, rail_mat)

    for i, x in enumerate([0.115, 0.190, 0.265, 0.340]):
        _add_rod(wing, f"wing_rail_{i}", (x, -half_y, 0.0), (x, half_y, 0.0), RAIL_RADIUS, rail_mat)


def _add_lower_support(support, rail_mat, hinge_mat, foot_mat) -> None:
    # Local frame is the underside hinge line.  The rack is drawn deployed; a
    # negative joint angle folds it upward toward the central frame.
    half_x = 0.380
    top_y, top_z = -0.050, -0.030
    low_y, low_z = -0.420, -0.580

    _add_rod(support, "hinge_barrel", (-0.120, 0.0, 0.0), (0.120, 0.0, 0.0), HINGE_RADIUS, hinge_mat)
    for x in [-0.075, 0.075]:
        _add_rod(
            support,
            f"hinge_web_{0 if x < 0 else 1}",
            (x, -0.009, -0.004),
            (x, top_y, top_z),
            0.0055,
            hinge_mat,
        )

    _add_rod(support, "top_tube", (-half_x, top_y, top_z), (half_x, top_y, top_z), FRAME_RADIUS, rail_mat)
    _add_rod(support, "lower_tube", (-half_x, low_y, low_z), (half_x, low_y, low_z), FRAME_RADIUS, rail_mat)
    _add_rod(support, "side_tube_0", (-half_x, top_y, top_z), (-half_x, low_y, low_z), FRAME_RADIUS, rail_mat)
    _add_rod(support, "side_tube_1", (half_x, top_y, top_z), (half_x, low_y, low_z), FRAME_RADIUS, rail_mat)

    for i, t in enumerate([0.25, 0.45, 0.65, 0.83]):
        y = top_y + (low_y - top_y) * t
        z = top_z + (low_z - top_z) * t
        _add_rod(support, f"lower_rail_{i}", (-half_x, y, z), (half_x, y, z), RAIL_RADIUS, rail_mat)

    # Small dark rubber sleeves at the lower contact bar ends, visibly seated on
    # the tube instead of floating.
    _add_rod(support, "foot_0", (-half_x - 0.035, low_y, low_z), (-half_x + 0.025, low_y, low_z), 0.011, foot_mat)
    _add_rod(support, "foot_1", (half_x - 0.025, low_y, low_z), (half_x + 0.035, low_y, low_z), 0.011, foot_mat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    rail_mat = model.material("white_powdercoat", rgba=(0.90, 0.92, 0.90, 1.0))
    hinge_mat = model.material("soft_grey_hinges", rgba=(0.56, 0.58, 0.56, 1.0))
    foot_mat = model.material("dark_rubber", rgba=(0.05, 0.05, 0.045, 1.0))

    central = model.part("central_frame")
    _add_central_frame(central, rail_mat, hinge_mat)

    wing_0 = model.part("wing_0")
    _add_wing_frame(wing_0, rail_mat, hinge_mat)

    wing_1 = model.part("wing_1")
    _add_wing_frame(wing_1, rail_mat, hinge_mat)

    support = model.part("lower_support")
    _add_lower_support(support, rail_mat, hinge_mat, foot_mat)

    wing_limits = MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.25)
    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(-0.465, 0.0, 0.85), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=wing_limits,
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(0.465, 0.0, 0.85)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=wing_limits,
    )
    model.articulation(
        "central_to_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=support,
        origin=Origin(xyz=(0.0, -0.300, 0.810)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.95, upper=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    support = object_model.get_part("lower_support")
    joint_0 = object_model.get_articulation("central_to_wing_0")
    joint_1 = object_model.get_articulation("central_to_wing_1")
    support_joint = object_model.get_articulation("central_to_support")

    ctx.allow_overlap(
        central,
        wing_0,
        elem_a="wing_pin_0",
        elem_b="hinge_barrel",
        reason="A slim fixed hinge pin intentionally passes through the wing hinge barrel.",
    )
    ctx.allow_overlap(
        central,
        wing_1,
        elem_a="wing_pin_1",
        elem_b="hinge_barrel",
        reason="A slim fixed hinge pin intentionally passes through the wing hinge barrel.",
    )
    ctx.allow_overlap(
        central,
        support,
        elem_a="support_pin",
        elem_b="hinge_barrel",
        reason="A slim fixed hinge pin intentionally passes through the folding support hinge barrel.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        len(object_model.articulations) == 3
        and joint_0.motion_limits.upper >= 1.2
        and joint_1.motion_limits.upper >= 1.2
        and support_joint.motion_limits.lower <= -0.9,
        details="Expected two wing hinges and one lower support hinge with useful folding ranges.",
    )
    ctx.expect_overlap(
        central,
        wing_0,
        axes="y",
        min_overlap=0.12,
        elem_a="wing_pin_0",
        elem_b="hinge_barrel",
        name="wing_0 barrel captures pin",
    )
    ctx.expect_overlap(
        central,
        wing_1,
        axes="y",
        min_overlap=0.12,
        elem_a="wing_pin_1",
        elem_b="hinge_barrel",
        name="wing_1 barrel captures pin",
    )
    ctx.expect_overlap(
        central,
        support,
        axes="x",
        min_overlap=0.20,
        elem_a="support_pin",
        elem_b="hinge_barrel",
        name="support barrel captures pin",
    )

    # The open pose should raise both wing free edges above the central rack.
    central_aabb = ctx.part_world_aabb(central)
    with ctx.pose({joint_0: 0.95, joint_1: 0.95}):
        wing_0_aabb = ctx.part_world_aabb(wing_0)
        wing_1_aabb = ctx.part_world_aabb(wing_1)
    ctx.check(
        "wings lift on hinge lines",
        central_aabb is not None
        and wing_0_aabb is not None
        and wing_1_aabb is not None
        and wing_0_aabb[1][2] > central_aabb[1][2] + 0.25
        and wing_1_aabb[1][2] > central_aabb[1][2] + 0.25,
        details=f"central={central_aabb}, wing_0={wing_0_aabb}, wing_1={wing_1_aabb}",
    )

    # The lower support starts as a deployed stand below the rack and folds
    # upward around its own hinge instead of translating or detaching.
    support_deployed = ctx.part_world_aabb(support)
    with ctx.pose({support_joint: -0.85}):
        support_folded = ctx.part_world_aabb(support)
    ctx.check(
        "lower support folds upward",
        central_aabb is not None
        and support_deployed is not None
        and support_folded is not None
        and support_deployed[0][2] < central_aabb[0][2] - 0.45
        and support_folded[0][2] > support_deployed[0][2] + 0.20,
        details=f"central={central_aabb}, deployed={support_deployed}, folded={support_folded}",
    )

    return ctx.report()


object_model = build_object_model()
