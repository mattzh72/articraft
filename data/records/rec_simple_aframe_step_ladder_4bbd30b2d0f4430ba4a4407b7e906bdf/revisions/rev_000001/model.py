from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot

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


def _slanted_member_origin(
    *,
    x: float,
    start_y: float,
    start_z: float,
    end_y: float,
    end_z: float,
) -> tuple[Origin, float]:
    dy = end_y - start_y
    dz = end_z - start_z
    length = hypot(dy, dz)
    roll = atan2(-dy, dz)
    return (
        Origin(
            xyz=(x, (start_y + end_y) * 0.5, (start_z + end_z) * 0.5),
            rpy=(roll, 0.0, 0.0),
        ),
        length,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.81, 0.78, 1.0))
    tread_gray = model.material("tread_gray", rgba=(0.36, 0.37, 0.39, 1.0))
    top_cap_black = model.material("top_cap_black", rgba=(0.14, 0.15, 0.17, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))

    rail_center_x = 0.215
    rail_width = 0.034
    rail_depth = 0.020
    foot_size = (0.046, 0.040, 0.018)

    front_foot_y = 0.115
    front_foot_z = foot_size[2] * 0.5
    front_rail_top_y = 0.010
    front_rail_top_z = 0.915

    hinge_z = 0.932
    rear_foot_y = -0.338
    rear_foot_z_local = -0.923
    rear_rail_top_y_local = -0.055
    rear_rail_top_z_local = -0.026
    rear_rail_bottom_y_local = rear_foot_y
    rear_rail_bottom_z_local = -0.914

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.50, 0.28, 0.96)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.04, 0.48)),
    )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        rail_origin, rail_length = _slanted_member_origin(
            x=side_sign * rail_center_x,
            start_y=front_foot_y,
            start_z=front_foot_z,
            end_y=front_rail_top_y,
            end_z=front_rail_top_z,
        )
        front_frame.visual(
            Box((rail_width, rail_depth, rail_length)),
            origin=rail_origin,
            material=aluminum,
            name=f"front_{side_name}_rail",
        )
    front_frame.visual(
        Box(foot_size),
        origin=Origin(xyz=(-rail_center_x, front_foot_y, front_foot_z)),
        material=rubber,
        name="front_left_foot",
    )
    front_frame.visual(
        Box(foot_size),
        origin=Origin(xyz=(rail_center_x, front_foot_y, front_foot_z)),
        material=rubber,
        name="front_right_foot",
    )

    step_heights = (0.225, 0.425, 0.625, 0.805)
    for index, step_z in enumerate(step_heights, start=1):
        t = (step_z - front_foot_z) / (front_rail_top_z - front_foot_z)
        step_y = front_foot_y + (front_rail_top_y - front_foot_y) * t
        step_width = 0.425 if index == len(step_heights) else 0.410
        step_depth = 0.108 if index == len(step_heights) else 0.094
        front_frame.visual(
            Box((step_width, step_depth, 0.025)),
            origin=Origin(xyz=(0.0, step_y, step_z)),
            material=aluminum,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Box((step_width * 0.92, step_depth * 0.64, 0.006)),
            origin=Origin(xyz=(0.0, step_y + 0.002, step_z + 0.0155)),
            material=tread_gray,
            name=f"tread_pad_{index}",
        )

    front_frame.visual(
        Box((0.46, 0.078, 0.022)),
        origin=Origin(xyz=(0.0, 0.004, 0.958)),
        material=top_cap_black,
        name="top_cap_cover",
    )
    front_frame.visual(
        Box((0.36, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.043, 0.936)),
        material=top_cap_black,
        name="top_cap_front_lip",
    )
    front_frame.visual(
        Box((0.090, 0.106, 0.050)),
        origin=Origin(xyz=(-0.170, -0.004, 0.936)),
        material=top_cap_black,
        name="top_cap_left_cheek",
    )
    front_frame.visual(
        Box((0.090, 0.106, 0.050)),
        origin=Origin(xyz=(0.170, -0.004, 0.936)),
        material=top_cap_black,
        name="top_cap_right_cheek",
    )
    front_frame.visual(
        Box((0.440, 0.036, 0.038)),
        origin=Origin(xyz=(0.0, -0.052, 0.929)),
        material=steel,
        name="top_cap_rear_housing",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.38, 0.94)),
        mass=4.9,
        origin=Origin(xyz=(0.0, -0.18, -0.46)),
    )

    rear_frame.visual(
        Box((0.420, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.068, -0.026)),
        material=top_cap_black,
        name="rear_top_yoke",
    )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        rear_rail_origin, rear_rail_length = _slanted_member_origin(
            x=side_sign * rail_center_x,
            start_y=rear_rail_bottom_y_local,
            start_z=rear_rail_bottom_z_local,
            end_y=rear_rail_top_y_local,
            end_z=rear_rail_top_z_local,
        )
        rear_frame.visual(
            Box((rail_width, rail_depth, rear_rail_length)),
            origin=rear_rail_origin,
            material=aluminum,
            name=f"rear_{side_name}_rail",
        )
    rear_frame.visual(
        Box(foot_size),
        origin=Origin(xyz=(-rail_center_x, rear_foot_y, rear_foot_z_local)),
        material=rubber,
        name="rear_left_foot",
    )
    rear_frame.visual(
        Box(foot_size),
        origin=Origin(xyz=(rail_center_x, rear_foot_y, rear_foot_z_local)),
        material=rubber,
        name="rear_right_foot",
    )

    rear_frame.visual(
        Box((0.450, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.285, -0.785)),
        material=aluminum,
        name="rear_lower_brace",
    )
    rear_frame.visual(
        Box((0.434, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, -0.165, -0.455)),
        material=aluminum,
        name="rear_mid_brace",
    )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.010, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=0.0, upper=0.44),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_frame_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="front_left_foot",
        negative_elem="rear_left_foot",
        min_gap=0.34,
        name="rear feet sit well behind the front feet when open",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="top_cap_rear_housing",
        elem_b="rear_top_yoke",
        name="rear support yoke nests into the top cap hinge housing",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        elem_a="top_cap_cover",
        elem_b="rear_top_yoke",
        min_overlap=0.18,
        name="rear yoke stays centered under the top cap",
    )

    def _elem_center_y(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    open_rear_foot_y = _elem_center_y("rear_frame", "rear_left_foot")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="y",
            positive_elem="front_left_foot",
            negative_elem="rear_left_foot",
            max_penetration=0.012,
            max_gap=0.09,
            name="rear feet tuck near the front feet when folded",
        )
        folded_rear_foot_y = _elem_center_y("rear_frame", "rear_left_foot")

    ctx.check(
        "rear frame folds forward around the top cap hinge",
        open_rear_foot_y is not None
        and folded_rear_foot_y is not None
        and folded_rear_foot_y > open_rear_foot_y + 0.30,
        details=f"open_y={open_rear_foot_y}, folded_y={folded_rear_foot_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
