from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRONT_FOOT_X = 0.24
REAR_FOOT_X = -0.34
HINGE_Z = 1.08
FOOT_PAD_HEIGHT = 0.03
RAIL_BOTTOM_Z = FOOT_PAD_HEIGHT * 0.5
FRAME_HALF_WIDTH = 0.17
REAR_FRAME_HALF_WIDTH = 0.215
RAIL_DEPTH = 0.03
RAIL_WIDTH = 0.045
STEP_DEPTH = 0.16
STEP_WIDTH = 0.325
STEP_THICKNESS = 0.03


def _line_x(foot_x: float, z: float) -> float:
    return foot_x * (1.0 - z / HINGE_Z)


def _slanted_rail_origin(
    bottom_xyz: tuple[float, float, float],
    top_xyz: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = top_xyz[0] - bottom_xyz[0]
    dy = top_xyz[1] - bottom_xyz[1]
    dz = top_xyz[2] - bottom_xyz[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    pitch = math.atan2(dx, dz)
    origin = Origin(
        xyz=(
            0.5 * (bottom_xyz[0] + top_xyz[0]),
            0.5 * (bottom_xyz[1] + top_xyz[1]),
            0.5 * (bottom_xyz[2] + top_xyz[2]),
        ),
        rpy=(0.0, pitch, 0.0),
    )
    return origin, length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    tread_gray = model.material("tread_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    cap_orange = model.material("cap_orange", rgba=(0.92, 0.45, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    front = model.part("front_frame")

    for rail_name, pad_name, rail_y in (
        ("front_left_rail", "front_left_pad", FRAME_HALF_WIDTH),
        ("front_right_rail", "front_right_pad", -FRAME_HALF_WIDTH),
    ):
        rail_origin, rail_length = _slanted_rail_origin(
            (FRONT_FOOT_X, rail_y, RAIL_BOTTOM_Z),
            (0.0, rail_y, HINGE_Z),
        )
        front.visual(
            Box((RAIL_DEPTH, RAIL_WIDTH, rail_length)),
            origin=rail_origin,
            material=aluminum,
            name=rail_name,
        )
        front.visual(
            Box((0.07, 0.06, FOOT_PAD_HEIGHT)),
            origin=Origin(xyz=(FRONT_FOOT_X, rail_y, FOOT_PAD_HEIGHT * 0.5)),
            material=rubber_black,
            name=pad_name,
        )

    step_heights = (0.23, 0.45, 0.67, 0.87)
    for index, step_z in enumerate(step_heights, start=1):
        front.visual(
            Box((STEP_DEPTH, STEP_WIDTH, STEP_THICKNESS)),
            origin=Origin(
                xyz=(
                    _line_x(FRONT_FOOT_X, step_z) + 0.035,
                    0.0,
                    step_z,
                )
            ),
            material=tread_gray,
            name=f"step_{index}",
        )

    front.visual(
        Box((0.10, 0.37, 0.05)),
        origin=Origin(xyz=(0.066, 0.0, HINGE_Z - 0.025)),
        material=cap_orange,
        name="top_cap",
    )
    front.visual(
        Box((0.11, 0.22, 0.025)),
        origin=Origin(xyz=(0.085, 0.0, HINGE_Z + 0.0125)),
        material=cap_orange,
        name="tray_pedestal",
    )
    front.visual(
        Box((0.22, 0.30, 0.010)),
        origin=Origin(xyz=(0.11, 0.0, HINGE_Z + 0.028)),
        material=cap_orange,
        name="tray_deck",
    )
    front.visual(
        Box((0.22, 0.015, 0.022)),
        origin=Origin(xyz=(0.11, FRAME_HALF_WIDTH - 0.0275, HINGE_Z + 0.039)),
        material=cap_orange,
        name="tray_left_lip",
    )
    front.visual(
        Box((0.22, 0.015, 0.022)),
        origin=Origin(xyz=(0.11, -FRAME_HALF_WIDTH + 0.0275, HINGE_Z + 0.039)),
        material=cap_orange,
        name="tray_right_lip",
    )
    front.visual(
        Box((0.015, 0.27, 0.022)),
        origin=Origin(xyz=(0.2125, 0.0, HINGE_Z + 0.039)),
        material=cap_orange,
        name="tray_front_lip",
    )
    front.inertial = Inertial.from_geometry(
        Box((0.42, 0.40, HINGE_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.12, 0.0, HINGE_Z * 0.5)),
    )

    rear = model.part("rear_frame")

    rear_bottom_z_local = RAIL_BOTTOM_Z - HINGE_Z
    for rail_name, pad_name, rail_y in (
        ("rear_left_rail", "rear_left_pad", REAR_FRAME_HALF_WIDTH),
        ("rear_right_rail", "rear_right_pad", -REAR_FRAME_HALF_WIDTH),
    ):
        rail_origin, rail_length = _slanted_rail_origin(
            (REAR_FOOT_X, rail_y, rear_bottom_z_local),
            (0.0, rail_y, 0.0),
        )
        rear.visual(
            Box((RAIL_DEPTH, RAIL_WIDTH, rail_length)),
            origin=rail_origin,
            material=aluminum,
            name=rail_name,
        )
        rear.visual(
            Box((0.07, 0.06, FOOT_PAD_HEIGHT)),
            origin=Origin(xyz=(REAR_FOOT_X, rail_y, rear_bottom_z_local)),
            material=rubber_black,
            name=pad_name,
        )

    brace_world_z = 0.37
    rear.visual(
        Box((0.04, 0.405, 0.03)),
        origin=Origin(
            xyz=(
                _line_x(REAR_FOOT_X, brace_world_z),
                0.0,
                brace_world_z - HINGE_Z,
            )
        ),
        material=tread_gray,
        name="rear_lower_brace",
    )
    rear.inertial = Inertial.from_geometry(
        Box((0.40, 0.38, HINGE_Z)),
        mass=3.8,
        origin=Origin(xyz=(REAR_FOOT_X * 0.5, 0.0, -HINGE_Z * 0.5)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_hinge")
    hinge_upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None

    def elem_aabb(part_name, elem_name):
        return ctx.part_element_world_aabb(part_name, elem=elem_name)

    def aabb_center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def z_gap(upper_aabb, lower_aabb):
        if upper_aabb is None or lower_aabb is None:
            return None
        return upper_aabb[0][2] - lower_aabb[1][2]

    ctx.expect_gap(
        front,
        rear,
        axis="x",
        positive_elem="front_left_pad",
        negative_elem="rear_left_pad",
        min_gap=0.45,
        name="deployed ladder keeps the rear feet behind the front feet",
    )

    tray_gap = z_gap(elem_aabb(front, "tray_deck"), elem_aabb(front, "step_4"))
    ctx.check(
        "tool tray sits above the top tread",
        tray_gap is not None and tray_gap >= 0.18,
        details=f"tray_to_top_tread_gap={tray_gap}",
    )

    open_rear_left_x = aabb_center_x(elem_aabb(rear, "rear_left_pad"))
    open_front_left_x = aabb_center_x(elem_aabb(front, "front_left_pad"))

    with ctx.pose({hinge: hinge_upper}):
        folded_gap_ok = ctx.expect_gap(
            front,
            rear,
            axis="x",
            positive_elem="front_left_pad",
            negative_elem="rear_left_pad",
            min_gap=0.0,
            max_gap=0.08,
            name="folded pose brings the rear feet close to the front feet",
        )
        folded_rear_left_x = aabb_center_x(elem_aabb(rear, "rear_left_pad"))
        ctx.check(
            "rear frame folds forward toward the climbing frame",
            folded_gap_ok
            and open_rear_left_x is not None
            and folded_rear_left_x is not None
            and open_front_left_x is not None
            and folded_rear_left_x > open_rear_left_x + 0.30
            and folded_rear_left_x < open_front_left_x + 0.02,
            details=(
                f"open_rear_left_x={open_rear_left_x}, "
                f"folded_rear_left_x={folded_rear_left_x}, "
                f"front_left_x={open_front_left_x}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
