from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan, cos, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _planar_offset(radius: float, yaw: float) -> tuple[float, float]:
    return (radius * cos(yaw), radius * sin(yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_closing_garden_gate")

    closer_arm_closed_yaw = radians(17.807270835766918)
    closer_shoe_closed_yaw = radians(148.5042118685453)

    cedar = model.material("cedar", rgba=(0.58, 0.40, 0.22, 1.0))
    cedar_dark = model.material("cedar_dark", rgba=(0.44, 0.30, 0.16, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.72, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.19, 0.22, 0.21, 1.0))
    black_iron = model.material("black_iron", rgba=(0.12, 0.12, 0.13, 1.0))

    post_frame = model.part("post_frame")
    post_frame.inertial = Inertial.from_geometry(
        Box((1.00, 0.10, 1.32)),
        mass=38.0,
        origin=Origin(xyz=(0.45, 0.0, 0.60)),
    )
    post_frame.visual(
        Box((0.10, 0.10, 1.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=cedar_dark,
        name="hinge_post",
    )
    post_frame.visual(
        Box((0.10, 0.10, 1.24)),
        origin=Origin(xyz=(0.90, 0.0, 0.62)),
        material=cedar_dark,
        name="latch_post",
    )
    post_frame.visual(
        Box((1.00, 0.08, 0.12)),
        origin=Origin(xyz=(0.45, 0.0, -0.06)),
        material=cedar_dark,
        name="buried_tie",
    )
    post_frame.visual(
        Box((0.130, 0.024, 0.058)),
        origin=Origin(xyz=(0.068, 0.024, 1.118)),
        material=painted_steel,
        name="closer_body",
    )
    post_frame.visual(
        Box((0.026, 0.016, 0.034)),
        origin=Origin(xyz=(0.128, 0.022, 1.136)),
        material=painted_steel,
        name="closer_post_bracket",
    )
    post_frame.visual(
        Box((0.030, 0.050, 0.085)),
        origin=Origin(xyz=(0.867, 0.0, 0.945)),
        material=galvanized,
        name="keeper_block",
    )
    post_frame.visual(
        Box((0.016, 0.050, 0.018)),
        origin=Origin(xyz=(0.845, 0.0, 0.975)),
        material=black_iron,
        name="keeper_lip",
    )
    for z_pos, stem_name, plate_name in (
        (0.205, "lower_hinge_pin", "lower_hinge_plate"),
        (0.905, "upper_hinge_pin", "upper_hinge_plate"),
    ):
        post_frame.visual(
            Cylinder(radius=0.012, length=0.100),
            origin=Origin(xyz=(0.050, 0.0, z_pos)),
            material=galvanized,
            name=stem_name,
        )
        post_frame.visual(
            Box((0.028, 0.055, 0.090)),
            origin=Origin(xyz=(0.035, 0.0, z_pos)),
            material=black_iron,
            name=plate_name,
        )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.inertial = Inertial.from_geometry(
        Box((0.79, 0.04, 1.02)),
        mass=18.0,
        origin=Origin(xyz=(0.395, 0.0, 0.57)),
    )
    gate_leaf.visual(
        Box((0.085, 0.038, 1.02)),
        origin=Origin(xyz=(0.050, 0.0, 0.57)),
        material=cedar,
        name="leaf_hinge_stile",
    )
    gate_leaf.visual(
        Box((0.085, 0.038, 1.02)),
        origin=Origin(xyz=(0.7475, 0.0, 0.57)),
        material=cedar,
        name="leaf_latch_stile",
    )
    gate_leaf.visual(
        Box((0.6125, 0.038, 0.085)),
        origin=Origin(xyz=(0.39875, 0.0, 1.0375)),
        material=cedar,
        name="leaf_top_rail",
    )
    gate_leaf.visual(
        Box((0.6125, 0.038, 0.085)),
        origin=Origin(xyz=(0.39875, 0.0, 0.1025)),
        material=cedar,
        name="leaf_bottom_rail",
    )
    gate_leaf.visual(
        Box((0.6125, 0.038, 0.070)),
        origin=Origin(xyz=(0.39875, 0.0, 0.57)),
        material=cedar_dark,
        name="leaf_mid_rail",
    )
    for idx, x_pos in enumerate((0.20, 0.34, 0.48, 0.62), start=1):
        gate_leaf.visual(
            Box((0.095, 0.018, 0.830)),
            origin=Origin(xyz=(x_pos, -0.010, 0.580)),
            material=cedar,
            name=f"leaf_board_{idx}",
        )
    gate_leaf.visual(
        Box((0.99, 0.026, 0.068)),
        origin=Origin(
            xyz=(0.402, 0.008, 0.57),
            rpy=(0.0, -atan((0.93 - 0.17) / (0.705 - 0.10)), 0.0),
        ),
        material=cedar_dark,
        name="leaf_diagonal_brace",
    )
    gate_leaf.visual(
        Box((0.16, 0.005, 0.060)),
        origin=Origin(xyz=(0.105, 0.0215, 0.905)),
        material=black_iron,
        name="upper_hinge_strap",
    )
    gate_leaf.visual(
        Box((0.16, 0.005, 0.060)),
        origin=Origin(xyz=(0.105, 0.0215, 0.205)),
        material=black_iron,
        name="lower_hinge_strap",
    )
    gate_leaf.visual(
        Box((0.074, 0.020, 0.086)),
        origin=Origin(xyz=(0.690, 0.022, 1.113)),
        material=black_iron,
        name="closer_gate_bracket",
    )
    gate_leaf.visual(
        Box((0.024, 0.004, 0.120)),
        origin=Origin(xyz=(0.750, 0.021, 0.945)),
        material=galvanized,
        name="latch_mount_plate",
    )

    latch = model.part("latch")
    latch.inertial = Inertial.from_geometry(
        Box((0.085, 0.045, 0.090)),
        mass=0.35,
        origin=Origin(xyz=(0.030, 0.0, 0.015)),
    )
    latch.visual(
        Box((0.028, 0.010, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=galvanized,
        name="latch_tongue",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=black_iron,
        name="latch_pivot_barrel",
    )
    latch.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.015, 0.0, 0.008)),
        material=black_iron,
        name="latch_handle_web",
    )
    latch.visual(
        Box((0.016, 0.010, 0.044)),
        origin=Origin(xyz=(0.030, 0.0, 0.016)),
        material=black_iron,
        name="latch_grip",
    )

    closer_arm = model.part("closer_arm")
    closer_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.020, 0.018)),
        mass=0.6,
        origin=Origin(xyz=(0.20, 0.0, 0.004)),
    )
    closer_arm.visual(
        Box((0.060, 0.016, 0.014)),
        origin=Origin(
            xyz=(*_planar_offset(0.030, closer_arm_closed_yaw), 0.004),
            rpy=(0.0, 0.0, closer_arm_closed_yaw),
        ),
        material=painted_steel,
        name="closer_arm_tab",
    )
    closer_arm.visual(
        Box((0.360, 0.014, 0.012)),
        origin=Origin(
            xyz=(*_planar_offset(0.210, closer_arm_closed_yaw), 0.004),
            rpy=(0.0, 0.0, closer_arm_closed_yaw),
        ),
        material=painted_steel,
        name="closer_arm_link",
    )
    closer_arm.visual(
        Box((0.040, 0.018, 0.014)),
        origin=Origin(
            xyz=(*_planar_offset(0.410, closer_arm_closed_yaw), 0.004),
            rpy=(0.0, 0.0, closer_arm_closed_yaw),
        ),
        material=black_iron,
        name="closer_arm_tip",
    )

    closer_shoe = model.part("closer_shoe")
    closer_shoe.inertial = Inertial.from_geometry(
        Box((0.24, 0.020, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(-0.12, 0.0, -0.004)),
    )
    closer_shoe.visual(
        Box((0.040, 0.016, 0.014)),
        origin=Origin(
            xyz=(*_planar_offset(0.020, closer_shoe_closed_yaw), -0.004),
            rpy=(0.0, 0.0, closer_shoe_closed_yaw),
        ),
        material=painted_steel,
        name="closer_shoe_tab",
    )
    closer_shoe.visual(
        Box((0.180, 0.014, 0.012)),
        origin=Origin(
            xyz=(*_planar_offset(0.110, closer_shoe_closed_yaw), -0.004),
            rpy=(0.0, 0.0, closer_shoe_closed_yaw),
        ),
        material=painted_steel,
        name="closer_shoe_link",
    )
    closer_shoe.visual(
        Box((0.040, 0.018, 0.014)),
        origin=Origin(
            xyz=(*_planar_offset(0.220, closer_shoe_closed_yaw), -0.004),
            rpy=(0.0, 0.0, closer_shoe_closed_yaw),
        ),
        material=black_iron,
        name="closer_shoe_pad",
    )

    model.articulation(
        "post_to_leaf",
        ArticulationType.REVOLUTE,
        parent=post_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=0.0,
            upper=radians(92.0),
        ),
    )
    model.articulation(
        "leaf_to_latch",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=latch,
        origin=Origin(xyz=(0.742, 0.026, 0.945)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=radians(35.0),
        ),
    )
    model.articulation(
        "post_to_closer_arm",
        ArticulationType.REVOLUTE,
        parent=post_frame,
        child=closer_arm,
        origin=Origin(xyz=(0.145, 0.022, 1.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(80.0),
        ),
    )
    model.articulation(
        "leaf_to_closer_shoe",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=closer_shoe,
        origin=Origin(xyz=(0.690, 0.022, 1.167)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post_frame = object_model.get_part("post_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    latch = object_model.get_part("latch")
    closer_arm = object_model.get_part("closer_arm")
    closer_shoe = object_model.get_part("closer_shoe")

    gate_hinge = object_model.get_articulation("post_to_leaf")
    latch_joint = object_model.get_articulation("leaf_to_latch")
    closer_post_joint = object_model.get_articulation("post_to_closer_arm")
    closer_gate_joint = object_model.get_articulation("leaf_to_closer_shoe")

    with ctx.pose(
        {
            gate_hinge: 0.0,
            latch_joint: 0.0,
            closer_post_joint: 0.0,
            closer_gate_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            gate_leaf,
            post_frame,
            axis="z",
            negative_elem="buried_tie",
            min_gap=0.055,
            max_gap=0.085,
            name="leaf clears the walkway surface",
        )
        ctx.expect_gap(
            post_frame,
            gate_leaf,
            axis="x",
            positive_elem="keeper_block",
            negative_elem="leaf_latch_stile",
            min_gap=0.010,
            max_gap=0.080,
            name="free edge sits just shy of the keeper post",
        )
        ctx.expect_contact(
            closer_arm,
            closer_shoe,
            elem_a="closer_arm_tip",
            elem_b="closer_shoe_pad",
            contact_tol=0.006,
            name="closed closer arm meets the gate shoe",
        )

        closed_leaf_center = _aabb_center(ctx.part_world_aabb(gate_leaf))
        closed_latch_grip = _aabb_center(ctx.part_element_world_aabb(latch, elem="latch_grip"))
        closed_arm_tip = _aabb_center(ctx.part_element_world_aabb(closer_arm, elem="closer_arm_tip"))
        closed_shoe_pad = _aabb_center(ctx.part_element_world_aabb(closer_shoe, elem="closer_shoe_pad"))

    with ctx.pose(
        {
            gate_hinge: radians(70.0),
            closer_post_joint: radians(66.7511),
            closer_gate_joint: radians(31.6597),
        }
    ):
        open_leaf_center = _aabb_center(ctx.part_world_aabb(gate_leaf))
        open_arm_tip = _aabb_center(ctx.part_element_world_aabb(closer_arm, elem="closer_arm_tip"))
        open_shoe_pad = _aabb_center(ctx.part_element_world_aabb(closer_shoe, elem="closer_shoe_pad"))

        ctx.check(
            "gate swings toward +y from the hinge post",
            closed_leaf_center is not None
            and open_leaf_center is not None
            and open_leaf_center[1] > closed_leaf_center[1] + 0.18,
            details=f"closed={closed_leaf_center}, open={open_leaf_center}",
        )
        ctx.expect_contact(
            closer_arm,
            closer_shoe,
            elem_a="closer_arm_tip",
            elem_b="closer_shoe_pad",
            contact_tol=0.015,
            name="closer linkage can stay engaged with the gate open",
        )
        ctx.check(
            "post pivot of closer arm rotates as the gate opens",
            closed_arm_tip is not None
            and open_arm_tip is not None
            and abs(open_arm_tip[1] - closed_arm_tip[1]) > 0.10,
            details=f"closed={closed_arm_tip}, open={open_arm_tip}",
        )
        ctx.check(
            "gate shoe pivot rotates with the gate leaf",
            closed_shoe_pad is not None
            and open_shoe_pad is not None
            and abs(open_shoe_pad[1] - closed_shoe_pad[1]) > 0.08,
            details=f"closed={closed_shoe_pad}, open={open_shoe_pad}",
        )

    with ctx.pose({latch_joint: radians(28.0)}):
        open_latch_grip = _aabb_center(ctx.part_element_world_aabb(latch, elem="latch_grip"))
        ctx.check(
            "latch handle lifts on its own pivot",
            closed_latch_grip is not None
            and open_latch_grip is not None
            and open_latch_grip[2] > closed_latch_grip[2] + 0.010,
            details=f"closed={closed_latch_grip}, lifted={open_latch_grip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
