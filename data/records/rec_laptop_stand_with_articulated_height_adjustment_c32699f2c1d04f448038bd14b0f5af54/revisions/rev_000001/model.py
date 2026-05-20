from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_DEPTH = 0.29
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.010

LOWER_PIVOT_X = 0.015
LOWER_PIVOT_Z = 0.034

ARM_SPAN_X = 0.105
ARM_SPAN_Z = 0.160
ARM_LENGTH = sqrt(ARM_SPAN_X**2 + ARM_SPAN_Z**2)
ARM_ANGLE = atan2(ARM_SPAN_Z, ARM_SPAN_X)
ARM_Y_OFFSET = 0.079
ARM_THICKNESS = 0.012
ARM_BAR_DEPTH = 0.020
PIVOT_BARREL_RADIUS = 0.008
ARM_END_RADIUS = 0.014
PIVOT_SHAFT_LENGTH = 0.192

TRAY_DEPTH = 0.29
TRAY_WIDTH = 0.235
TRAY_THICKNESS = 0.006
TRAY_PITCH = -0.26
TRAY_DECK_CENTER = (-0.040, 0.0, 0.017)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x_pos, y_pos, z_pos = point
    c_val = cos(angle)
    s_val = sin(angle)
    return (
        c_val * x_pos + s_val * z_pos,
        y_pos,
        -s_val * x_pos + c_val * z_pos,
    )


def _tray_origin(point: tuple[float, float, float]) -> Origin:
    return Origin(xyz=_rotate_y(point, TRAY_PITCH), rpy=(0.0, TRAY_PITCH, 0.0))


def _arm_bar_origin(y_pos: float) -> Origin:
    return Origin(
        xyz=(ARM_SPAN_X * 0.5, y_pos, ARM_SPAN_Z * 0.5),
        rpy=(0.0, -ARM_ANGLE, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    model.material("powder_coat", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("anodized_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("dark_polymer", rgba=(0.14, 0.15, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material="powder_coat",
        name="plate",
    )
    for y_pos, name in ((0.101, "pedestal_0"), (-0.101, "pedestal_1")):
        base.visual(
            Box((0.036, 0.014, 0.016)),
            origin=Origin(xyz=(LOWER_PIVOT_X, y_pos, 0.018)),
            material="powder_coat",
            name=name,
        )
        base.visual(
            Box((0.020, 0.014, 0.012)),
            origin=Origin(xyz=(LOWER_PIVOT_X - 0.018, y_pos, 0.016)),
            material="powder_coat",
            name=f"gusset_{0 if y_pos > 0 else 1}",
        )

    arm_pair = model.part("arm_pair")
    arm_pair.visual(
        Cylinder(radius=PIVOT_BARREL_RADIUS, length=PIVOT_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_polymer",
        name="lower_shaft",
    )
    arm_pair.visual(
        Cylinder(radius=PIVOT_BARREL_RADIUS, length=PIVOT_SHAFT_LENGTH),
        origin=Origin(
            xyz=(ARM_SPAN_X, 0.0, ARM_SPAN_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="dark_polymer",
        name="upper_shaft",
    )
    for index, y_pos in enumerate((ARM_Y_OFFSET, -ARM_Y_OFFSET)):
        arm_pair.visual(
            Box((ARM_LENGTH, ARM_THICKNESS, ARM_BAR_DEPTH)),
            origin=_arm_bar_origin(y_pos),
            material="anodized_aluminum",
            name=f"arm_{index}",
        )
        arm_pair.visual(
            Cylinder(radius=ARM_END_RADIUS, length=ARM_THICKNESS),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="anodized_aluminum",
            name=f"lower_knuckle_{index}",
        )
        arm_pair.visual(
            Cylinder(radius=ARM_END_RADIUS, length=ARM_THICKNESS),
            origin=Origin(
                xyz=(ARM_SPAN_X, y_pos, ARM_SPAN_Z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="anodized_aluminum",
            name=f"upper_knuckle_{index}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_THICKNESS)),
        origin=_tray_origin(TRAY_DECK_CENTER),
        material="anodized_aluminum",
        name="deck",
    )
    tray.visual(
        Box((0.012, 0.180, 0.008)),
        origin=_tray_origin((-0.125, 0.0, 0.011)),
        material="dark_polymer",
        name="front_rib",
    )
    tray.visual(
        Box((0.012, 0.180, 0.008)),
        origin=_tray_origin((0.060, 0.0, 0.011)),
        material="dark_polymer",
        name="rear_rib",
    )
    for index, y_pos in enumerate((0.070, -0.070)):
        tray.visual(
            Box((0.014, 0.032, 0.016)),
            origin=_tray_origin((-0.178, y_pos, 0.027)),
            material="dark_polymer",
            name=f"lip_{index}",
        )
    for index, y_pos in enumerate((0.102, -0.102)):
        tray.visual(
            Box((0.020, 0.014, 0.016)),
            origin=_tray_origin((0.0, y_pos, 0.016)),
            material="anodized_aluminum",
            name=f"lug_{index}",
        )

    model.articulation(
        "base_to_arms",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_pair,
        origin=Origin(xyz=(LOWER_PIVOT_X, 0.0, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.40,
            upper=0.35,
        ),
    )
    model.articulation(
        "arms_to_tray",
        ArticulationType.REVOLUTE,
        parent=arm_pair,
        child=tray,
        origin=Origin(xyz=(ARM_SPAN_X, 0.0, ARM_SPAN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    base_to_arms = object_model.get_articulation("base_to_arms")
    arms_to_tray = object_model.get_articulation("arms_to_tray")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.10,
        positive_elem="deck",
        negative_elem="plate",
        name="tray deck sits clearly above the base plate",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.12,
        elem_a="deck",
        elem_b="plate",
        name="tray stays over the base footprint",
    )
    lip_aabb = ctx.part_element_world_aabb(tray, elem="lip_0")
    front_rib_aabb = ctx.part_element_world_aabb(tray, elem="front_rib")
    lip_top = None if lip_aabb is None else lip_aabb[1][2]
    front_rib_top = None if front_rib_aabb is None else front_rib_aabb[1][2]
    lip_front = None if lip_aabb is None else lip_aabb[0][0]
    deck_aabb = ctx.part_element_world_aabb(tray, elem="deck")
    deck_front = None if deck_aabb is None else deck_aabb[0][0]
    ctx.check(
        "retaining lip stands proud at the tray front edge",
        lip_top is not None
        and front_rib_top is not None
        and lip_front is not None
        and deck_front is not None
        and lip_top > front_rib_top + 0.004
        and lip_front <= deck_front + 0.005,
        details=(
            f"lip_top={lip_top}, front_rib_top={front_rib_top}, "
            f"lip_front={lip_front}, deck_front={deck_front}"
        ),
    )

    def rib_slope() -> float | None:
        front_aabb = ctx.part_element_world_aabb(tray, elem="front_rib")
        rear_aabb = ctx.part_element_world_aabb(tray, elem="rear_rib")
        if front_aabb is None or rear_aabb is None:
            return None
        front_center_z = 0.5 * (front_aabb[0][2] + front_aabb[1][2])
        rear_center_z = 0.5 * (rear_aabb[0][2] + rear_aabb[1][2])
        return rear_center_z - front_center_z

    rest_pos = ctx.part_world_position(tray)
    rest_slope = rib_slope()
    with ctx.pose({base_to_arms: 0.30, arms_to_tray: 0.30}):
        raised_pos = ctx.part_world_position(tray)
        raised_slope = rib_slope()
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.13,
            positive_elem="deck",
            negative_elem="plate",
            name="raised tray keeps clearance above the base",
        )
    with ctx.pose({base_to_arms: -0.30, arms_to_tray: -0.30}):
        lowered_pos = ctx.part_world_position(tray)
        lowered_slope = rib_slope()
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.07,
            positive_elem="deck",
            negative_elem="plate",
            name="lowered tray still clears the base",
        )

    ctx.check(
        "coordinated arm motion raises the tray",
        rest_pos is not None
        and raised_pos is not None
        and lowered_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.02
        and lowered_pos[2] < rest_pos[2] - 0.02,
        details=f"lowered={lowered_pos}, rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "counter-rotating tray joint preserves tray pitch",
        rest_slope is not None
        and raised_slope is not None
        and lowered_slope is not None
        and abs(rest_slope - raised_slope) < 0.008
        and abs(rest_slope - lowered_slope) < 0.008,
        details=f"lowered={lowered_slope}, rest={rest_slope}, raised={raised_slope}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
