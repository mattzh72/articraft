from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2

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


LADDER_HEIGHT = 1.72
RAIL_CENTER_X = 0.21
RAIL_WIDTH = 0.042
RAIL_DEPTH = 0.024
FOOT_WIDTH = 0.078
FOOT_DEPTH = 0.052
FOOT_HEIGHT = 0.028

FRONT_TOP_Y = 0.045
FRONT_BOTTOM_Y = 0.43
FRONT_TOP_Z = 1.69
FRONT_BOTTOM_Z = 0.0305
REAR_TOP_Y_LOCAL = -0.045
REAR_BOTTOM_Y_LOCAL = -0.43
REAR_TOP_Z_LOCAL = -0.03
REAR_BOTTOM_Z_LOCAL = -1.67


def _rail_angle(top_y: float, top_z: float, bottom_y: float, bottom_z: float) -> float:
    return atan2(bottom_y - top_y, top_z - bottom_z)


def _line_y_at_z(top_y: float, top_z: float, bottom_y: float, bottom_z: float, z: float) -> float:
    if abs(top_z - bottom_z) < 1e-9:
        return top_y
    t = (top_z - z) / (top_z - bottom_z)
    return top_y + (bottom_y - top_y) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    tread_gray = model.material("tread_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    cap_black = model.material("cap_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.64, LADDER_HEIGHT)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.10, LADDER_HEIGHT * 0.5)),
    )

    front_rail_len = ((FRONT_BOTTOM_Y - FRONT_TOP_Y) ** 2 + (FRONT_TOP_Z - FRONT_BOTTOM_Z) ** 2) ** 0.5
    front_rail_angle = _rail_angle(FRONT_TOP_Y, FRONT_TOP_Z, FRONT_BOTTOM_Y, FRONT_BOTTOM_Z)
    front_rail_center_y = 0.5 * (FRONT_TOP_Y + FRONT_BOTTOM_Y)
    front_rail_center_z = 0.5 * (FRONT_TOP_Z + FRONT_BOTTOM_Z)

    for side, x in (("left", -RAIL_CENTER_X), ("right", RAIL_CENTER_X)):
        front_frame.visual(
            Box((RAIL_WIDTH, RAIL_DEPTH, front_rail_len)),
            origin=Origin(xyz=(x, front_rail_center_y, front_rail_center_z), rpy=(front_rail_angle, 0.0, 0.0)),
            material=aluminum,
            name=f"front_{side}_rail",
        )
        front_frame.visual(
            Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
            origin=Origin(xyz=(x, FRONT_BOTTOM_Y, FOOT_HEIGHT * 0.5)),
            material=rubber_black,
            name=f"front_{side}_foot",
        )

    step_span = 2.0 * (RAIL_CENTER_X - 0.5 * RAIL_WIDTH) + 0.008
    step_specs = (
        (0.28, 0.12),
        (0.60, 0.13),
        (0.92, 0.15),
        (1.24, 0.17),
    )
    for index, (z, depth) in enumerate(step_specs, start=1):
        front_frame.visual(
            Box((step_span, depth, 0.030)),
            origin=Origin(xyz=(0.0, _line_y_at_z(FRONT_TOP_Y, FRONT_TOP_Z, FRONT_BOTTOM_Y, FRONT_BOTTOM_Z, z), z)),
            material=tread_gray,
            name=f"tread_{index}",
        )

    front_frame.visual(
        Box((0.45, 0.20, 0.060)),
        origin=Origin(xyz=(0.0, 0.118, LADDER_HEIGHT)),
        material=cap_black,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.18, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.020, 1.695)),
        material=steel,
        name="hinge_support_block",
    )
    front_frame.visual(
        Cylinder(radius=0.015, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, LADDER_HEIGHT), rpy=(0.0, 1.57079632679, 0.0)),
        material=steel,
        name="center_hinge_barrel",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.56, LADDER_HEIGHT)),
        mass=5.5,
        origin=Origin(xyz=(0.0, -0.24, -0.84)),
    )

    rear_rail_len = (
        (REAR_BOTTOM_Y_LOCAL - REAR_TOP_Y_LOCAL) ** 2 + (REAR_TOP_Z_LOCAL - REAR_BOTTOM_Z_LOCAL) ** 2
    ) ** 0.5
    rear_rail_angle = -front_rail_angle
    rear_rail_center_y = 0.5 * (REAR_TOP_Y_LOCAL + REAR_BOTTOM_Y_LOCAL)
    rear_rail_center_z = 0.5 * (REAR_TOP_Z_LOCAL + REAR_BOTTOM_Z_LOCAL)

    for side, x in (("left", -RAIL_CENTER_X), ("right", RAIL_CENTER_X)):
        rear_frame.visual(
            Box((RAIL_WIDTH, RAIL_DEPTH, rear_rail_len)),
            origin=Origin(xyz=(x, rear_rail_center_y, rear_rail_center_z), rpy=(rear_rail_angle, 0.0, 0.0)),
            material=aluminum,
            name=f"rear_{side}_rail",
        )
        rear_frame.visual(
            Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
            origin=Origin(xyz=(x, REAR_BOTTOM_Y_LOCAL, REAR_BOTTOM_Z_LOCAL + FOOT_HEIGHT * 0.5)),
            material=rubber_black,
            name=f"rear_{side}_foot",
        )

    rear_frame.visual(
        Box((0.44, 0.024, 0.080)),
        origin=Origin(xyz=(0.0, -0.055, -0.065)),
        material=aluminum,
        name="rear_top_bridge",
    )
    for index, z in enumerate((-0.54, -0.96, -1.34), start=1):
        rear_frame.visual(
            Box((step_span, 0.022, 0.028)),
            origin=Origin(
                xyz=(0.0, _line_y_at_z(REAR_TOP_Y_LOCAL, REAR_TOP_Z_LOCAL, REAR_BOTTOM_Y_LOCAL, REAR_BOTTOM_Z_LOCAL, z), z)
            ),
            material=aluminum,
            name=f"rear_brace_{index}",
        )

    for side, x in (("left", -0.18), ("right", 0.18)):
        rear_frame.visual(
            Box((0.060, 0.060, 0.070)),
            origin=Origin(xyz=(x, -0.020, -0.035)),
            material=steel,
            name=f"{side}_hinge_ear",
        )
        rear_frame.visual(
            Cylinder(radius=0.015, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=steel,
            name=f"{side}_hinge_barrel",
        )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, LADDER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.08, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_frame_hinge")

    def _center_from_aabb(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        index = {"x": 0, "y": 1, "z": 2}[axis]
        return 0.5 * (aabb[0][index] + aabb[1][index])

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="front_left_foot",
        negative_elem="rear_left_foot",
        min_gap=0.74,
        name="open stance keeps front and rear feet well separated",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        elem_a="front_left_foot",
        elem_b="rear_left_foot",
        min_overlap=0.06,
        name="left feet stay laterally aligned for an A-frame stance",
    )

    front_left_foot = ctx.part_element_world_aabb(front_frame, elem="front_left_foot")
    rear_left_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    front_y = _center_from_aabb(front_left_foot, "y")
    rear_y = _center_from_aabb(rear_left_foot, "y")
    ctx.check(
        "open ladder is nearly symmetric about the center plane",
        front_y is not None and rear_y is not None and abs(front_y + rear_y) <= 0.05,
        details=f"front_y={front_y}, rear_y={rear_y}",
    )

    rest_rear_y = rear_y
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        folded_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
        folded_rear_y = _center_from_aabb(folded_rear_foot, "y")
        ctx.check(
            "rear frame folds forward toward the front frame",
            rest_rear_y is not None and folded_rear_y is not None and folded_rear_y > rest_rear_y + 0.60,
            details=f"rest_rear_y={rest_rear_y}, folded_rear_y={folded_rear_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
