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
)


def _add_diagonal_brace(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    depth: float,
    thickness: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    part.visual(
        Box((length, depth, thickness)),
        origin=Origin(
            xyz=((start[0] + end[0]) * 0.5, start[1], (start[2] + end[2]) * 0.5),
            rpy=(0.0, angle, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_garden_gate")

    concrete = model.material("concrete", rgba=(0.57, 0.58, 0.60, 1.0))
    steel = model.material("powder_coated_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    aluminum = model.material("anodized_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("roller_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    opening_width = 1.32
    tail_length = 0.82
    gate_length = opening_width + tail_length
    gate_height = 1.48
    frame_depth = 0.05
    stile_width = 0.06
    bottom_beam_height = 0.11
    top_rail_height = 0.06
    slat_width = 0.032
    slat_depth = 0.024
    leaf_base_z = 0.319

    support = model.part("support_frame")

    beam_min_x = -0.55
    beam_max_x = 1.42
    support.visual(
        Box((beam_max_x - beam_min_x, 0.32, 0.14)),
        origin=Origin(xyz=((beam_min_x + beam_max_x) * 0.5, -0.05, 0.07)),
        material=concrete,
        name="grade_beam",
    )

    support.visual(
        Box((0.10, 0.10, 1.86)),
        origin=Origin(xyz=(0.0, -0.13, 1.05)),
        material=steel,
        name="guide_post",
    )
    support.visual(
        Box((0.09, 0.09, 1.76)),
        origin=Origin(xyz=(1.35, -0.13, 1.00)),
        material=steel,
        name="latch_post",
    )

    truck_specs = (
        (-0.22, "rear_truck"),
        (0.18, "front_truck"),
    )
    for truck_x, prefix in truck_specs:
        support.visual(
            Box((0.12, 0.16, 0.10)),
            origin=Origin(xyz=(truck_x, 0.0, 0.19)),
            material=steel,
            name=f"{prefix}_pedestal",
        )
        support.visual(
            Box((0.16, 0.064, 0.03)),
            origin=Origin(xyz=(truck_x, 0.0, 0.255)),
            material=steel,
            name=f"{prefix}_saddle",
        )
        for side_y, side_name in ((-0.038, "left"), (0.038, "right")):
            support.visual(
                Box((0.10, 0.012, 0.08)),
                origin=Origin(xyz=(truck_x, side_y, 0.279)),
                material=steel,
                name=f"{prefix}_{side_name}_cheek",
            )
        for side_y, side_name in ((-0.018, "left"), (0.018, "right")):
            support.visual(
                Cylinder(radius=0.040, length=0.028),
                origin=Origin(xyz=(truck_x, side_y, 0.279), rpy=(math.pi * 0.5, 0.0, 0.0)),
                material=rubber,
                name=f"{prefix}_{side_name}_roller",
            )

    for bracket_z, prefix in ((0.70, "lower"), (1.34, "upper")):
        support.visual(
            Box((0.10, 0.04, 0.05)),
            origin=Origin(xyz=(0.05, -0.06, bracket_z)),
            material=steel,
            name=f"{prefix}_guide_bracket",
        )
        support.visual(
            Box((0.02, 0.136, 0.05)),
            origin=Origin(xyz=(0.075, 0.0, bracket_z)),
            material=steel,
            name=f"{prefix}_guide_crossbar",
        )
        for side_y, side_name in ((-0.048, "rear"), (0.048, "front")):
            support.visual(
                Cylinder(radius=0.018, length=0.032),
                origin=Origin(xyz=(0.075, side_y, bracket_z), rpy=(0.0, math.pi * 0.5, 0.0)),
                material=rubber,
                name=f"{prefix}_guide_{side_name}_roller",
            )

    support.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(1.33, -0.105, 1.02)),
        material=steel,
        name="receiver_body",
    )
    support.visual(
        Box((0.024, 0.02, 0.14)),
        origin=Origin(xyz=(1.306, -0.075, 1.02)),
        material=steel,
        name="receiver_plate",
    )

    leaf = model.part("gate_leaf")
    leaf.visual(
        Box((gate_length, 0.06, bottom_beam_height)),
        origin=Origin(xyz=(-gate_length * 0.5, 0.0, bottom_beam_height * 0.5)),
        material=aluminum,
        name="leaf_bottom_beam",
    )
    leaf.visual(
        Box((gate_length, frame_depth, top_rail_height)),
        origin=Origin(xyz=(-gate_length * 0.5, 0.0, gate_height - top_rail_height * 0.5)),
        material=aluminum,
        name="leaf_top_rail",
    )
    leaf.visual(
        Box((stile_width, frame_depth, gate_height)),
        origin=Origin(xyz=(-stile_width * 0.5, 0.0, gate_height * 0.5)),
        material=aluminum,
        name="leading_stile",
    )
    leaf.visual(
        Box((stile_width, frame_depth, gate_height)),
        origin=Origin(xyz=(-opening_width + stile_width * 0.5, 0.0, gate_height * 0.5)),
        material=aluminum,
        name="guide_stile",
    )
    leaf.visual(
        Box((stile_width, frame_depth, gate_height)),
        origin=Origin(xyz=(-gate_length + stile_width * 0.5, 0.0, gate_height * 0.5)),
        material=aluminum,
        name="rear_stile",
    )
    leaf.visual(
        Box((opening_width - 0.10, 0.035, 0.05)),
        origin=Origin(xyz=(-opening_width * 0.5, 0.0, 0.82)),
        material=aluminum,
        name="mid_rail",
    )

    slat_height = gate_height - top_rail_height - bottom_beam_height + 0.008
    slat_center_z = (bottom_beam_height + gate_height - top_rail_height) * 0.5
    slat_left_x = -opening_width + stile_width + 0.07
    slat_right_x = -stile_width - 0.07
    slat_count = 11
    for index in range(slat_count):
        t = index / (slat_count - 1)
        x_pos = slat_left_x + (slat_right_x - slat_left_x) * t
        leaf.visual(
            Box((slat_width, slat_depth, slat_height)),
            origin=Origin(xyz=(x_pos, 0.0, slat_center_z)),
            material=aluminum,
            name=f"slat_{index + 1}",
        )

    _add_diagonal_brace(
        leaf,
        (-gate_length + 0.04, 0.0, bottom_beam_height + 0.05),
        (-opening_width + 0.08, 0.0, gate_height - top_rail_height - 0.05),
        depth=0.032,
        thickness=0.032,
        material=aluminum,
        name="tail_brace_rising",
    )
    _add_diagonal_brace(
        leaf,
        (-gate_length + 0.04, 0.0, gate_height - top_rail_height - 0.04),
        (-opening_width + 0.10, 0.0, bottom_beam_height + 0.21),
        depth=0.032,
        thickness=0.032,
        material=aluminum,
        name="tail_brace_falling",
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=handle_finish,
        name="handle_hub",
    )
    handle.visual(
        Box((0.105, 0.018, 0.016)),
        origin=Origin(xyz=(0.0615, 0.0, 0.0)),
        material=handle_finish,
        name="handle_grip",
    )
    handle.visual(
        Box((0.018, 0.024, 0.024)),
        origin=Origin(xyz=(0.114, 0.0, 0.0)),
        material=handle_finish,
        name="handle_end_stop",
    )

    model.articulation(
        "support_to_leaf",
        ArticulationType.PRISMATIC,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(opening_width, 0.0, leaf_base_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "leaf_to_handle",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=handle,
        origin=Origin(xyz=(0.009, 0.0, 0.98)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=-0.45,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    leaf = object_model.get_part("gate_leaf")
    handle = object_model.get_part("latch_handle")
    slide = object_model.get_articulation("support_to_leaf")
    handle_joint = object_model.get_articulation("leaf_to_handle")

    ctx.expect_contact(
        leaf,
        support,
        elem_a="leaf_bottom_beam",
        elem_b="front_truck_left_roller",
        name="front truck supports the cantilever beam",
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a="leaf_bottom_beam",
        elem_b="rear_truck_left_roller",
        name="rear truck supports the cantilever beam",
    )
    ctx.expect_overlap(
        support,
        leaf,
        axes="xz",
        elem_a="receiver_plate",
        elem_b="leading_stile",
        min_overlap=0.020,
        name="receiver plate aligns with the closed leading stile",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="y",
        positive_elem="leading_stile",
        negative_elem="receiver_plate",
        min_gap=0.030,
        max_gap=0.060,
        name="receiver plate sits just behind the closed leaf",
    )
    ctx.expect_origin_gap(
        handle,
        leaf,
        axis="x",
        min_gap=0.005,
        max_gap=0.020,
        name="handle pivot sits at the leaf front edge",
    )
    ctx.expect_origin_gap(
        handle,
        leaf,
        axis="z",
        min_gap=0.92,
        max_gap=1.05,
        name="handle pivot is mounted near mid height",
    )
    ctx.expect_contact(
        handle,
        leaf,
        elem_a="handle_hub",
        elem_b="leading_stile",
        name="handle hub mounts against the leading stile",
    )

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    rest_leaf_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: slide_upper}):
        open_leaf_pos = ctx.part_world_position(leaf)
        ctx.expect_overlap(
            leaf,
            support,
            axes="xy",
            elem_a="leaf_bottom_beam",
            elem_b="front_truck_left_roller",
            min_overlap=0.020,
            name="open leaf still spans the front truck",
        )
        ctx.expect_overlap(
            leaf,
            support,
            axes="xy",
            elem_a="leaf_bottom_beam",
            elem_b="rear_truck_left_roller",
            min_overlap=0.020,
            name="open leaf still spans the rear truck",
        )
    ctx.check(
        "leaf slides rearward to open",
        rest_leaf_pos is not None
        and open_leaf_pos is not None
        and open_leaf_pos[0] < rest_leaf_pos[0] - 0.90,
        details=f"rest={rest_leaf_pos}, open={open_leaf_pos}",
    )

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: 0.60}):
        raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    rest_handle_z = _aabb_center_z(rest_handle_aabb)
    raised_handle_z = _aabb_center_z(raised_handle_aabb)
    ctx.check(
        "handle rotates upward on its pivot",
        rest_handle_z is not None
        and raised_handle_z is not None
        and raised_handle_z > rest_handle_z + 0.020,
        details=f"rest_z={rest_handle_z}, raised_z={raised_handle_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
