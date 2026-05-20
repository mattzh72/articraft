from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="writing_table")

    table_width = 1.20
    table_depth = 0.65
    table_height = 0.76
    top_thickness = 0.03
    apron_height = 0.09
    apron_thickness = 0.02

    leg_top = 0.05
    leg_foot = 0.035
    leg_inset = 0.025
    leg_height = table_height - top_thickness

    leg_center_x = table_width / 2 - leg_inset - leg_top / 2
    leg_center_y = table_depth / 2 - leg_inset - leg_top / 2
    apron_top_z = leg_height
    apron_bottom_z = apron_top_z - apron_height

    drawer_opening_width = 0.50
    drawer_opening_height = 0.065
    top_rail_height = apron_height - drawer_opening_height

    drawer_front_width = 0.49
    drawer_front_height = 0.062
    drawer_front_thickness = 0.018
    drawer_side_thickness = 0.012
    drawer_side_height = 0.05
    drawer_side_depth = 0.40
    drawer_back_width = 0.454
    drawer_bottom_width = 0.454
    drawer_bottom_depth = 0.394
    drawer_bottom_thickness = 0.006
    drawer_slide_travel = 0.28

    runner_width = 0.03
    runner_height = 0.012
    runner_length = 0.53

    walnut = model.material("walnut", rgba=(0.45, 0.30, 0.18, 1.0))
    runner_wood = model.material("runner_wood", rgba=(0.62, 0.46, 0.28, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.62, 0.32, 1.0))

    def square_loop(half_width: float, z: float) -> tuple[tuple[float, float, float], ...]:
        return (
            (-half_width, -half_width, z),
            (half_width, -half_width, z),
            (half_width, half_width, z),
            (-half_width, half_width, z),
        )

    tapered_leg_mesh = mesh_from_geometry(
        section_loft(
            [
                square_loop(leg_top / 2, 0.0),
                square_loop((leg_top * 0.92) / 2, -0.18),
                square_loop(leg_foot / 2, -leg_height),
            ]
        ),
        "tapered_square_leg",
    )

    frame = model.part("frame")
    frame.visual(
        Box((table_width, table_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, table_height - top_thickness / 2)),
        material=walnut,
        name="top",
    )

    full_apron_span_x = 2 * leg_center_x - leg_top
    full_apron_span_y = 2 * leg_center_y - leg_top
    front_y = -leg_center_y
    rear_y = leg_center_y
    left_x = -leg_center_x + leg_top / 2 - apron_thickness / 2
    right_x = leg_center_x - leg_top / 2 + apron_thickness / 2

    stile_width = (full_apron_span_x - drawer_opening_width) / 2

    frame.visual(
        Box((stile_width, apron_thickness, apron_height)),
        origin=Origin(
            xyz=(
                -(drawer_opening_width / 2 + stile_width / 2),
                front_y,
                apron_bottom_z + apron_height / 2,
            )
        ),
        material=walnut,
        name="front_left_stile",
    )
    frame.visual(
        Box((stile_width, apron_thickness, apron_height)),
        origin=Origin(
            xyz=(
                (drawer_opening_width / 2 + stile_width / 2),
                front_y,
                apron_bottom_z + apron_height / 2,
            )
        ),
        material=walnut,
        name="front_right_stile",
    )
    frame.visual(
        Box((drawer_opening_width, apron_thickness, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                apron_top_z - top_rail_height / 2,
            )
        ),
        material=walnut,
        name="top_rail",
    )
    frame.visual(
        Box((full_apron_span_x, apron_thickness, apron_height)),
        origin=Origin(
            xyz=(0.0, rear_y, apron_bottom_z + apron_height / 2),
        ),
        material=walnut,
        name="rear_apron",
    )
    frame.visual(
        Box((apron_thickness, full_apron_span_y, apron_height)),
        origin=Origin(
            xyz=(left_x, 0.0, apron_bottom_z + apron_height / 2),
        ),
        material=walnut,
        name="left_apron",
    )
    frame.visual(
        Box((apron_thickness, full_apron_span_y, apron_height)),
        origin=Origin(
            xyz=(right_x, 0.0, apron_bottom_z + apron_height / 2),
        ),
        material=walnut,
        name="right_apron",
    )

    runner_z = 0.636
    frame.visual(
        Box((runner_width, runner_length, runner_height)),
        origin=Origin(xyz=(-0.235, 0.0, runner_z)),
        material=runner_wood,
        name="left_runner",
    )
    frame.visual(
        Box((runner_width, runner_length, runner_height)),
        origin=Origin(xyz=(0.235, 0.0, runner_z)),
        material=runner_wood,
        name="right_runner",
    )

    for name, x_sign, y_sign in (
        ("front_left_leg", -1.0, -1.0),
        ("front_right_leg", 1.0, -1.0),
        ("rear_left_leg", -1.0, 1.0),
        ("rear_right_leg", 1.0, 1.0),
    ):
        leg = model.part(name)
        leg.visual(tapered_leg_mesh, material=walnut, name="leg_body")
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.FIXED,
            parent=frame,
            child=leg,
            origin=Origin(
                xyz=(
                    x_sign * leg_center_x,
                    y_sign * leg_center_y,
                    leg_height,
                )
            ),
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
        origin=Origin(xyz=(0.0, drawer_front_thickness / 2, 0.0)),
        material=walnut,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_side_depth, drawer_side_height)),
        origin=Origin(xyz=(-0.233, 0.218, -0.004)),
        material=walnut,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_side_depth, drawer_side_height)),
        origin=Origin(xyz=(0.233, 0.218, -0.004)),
        material=walnut,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_back_width, drawer_side_thickness, drawer_side_height)),
        origin=Origin(xyz=(0.0, 0.412, -0.004)),
        material=walnut,
        name="drawer_back",
    )
    drawer.visual(
        Box((drawer_bottom_width, drawer_bottom_depth, drawer_bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.215, -0.027)),
        material=runner_wood,
        name="drawer_bottom",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=brass,
        name="drawer_knob",
    )

    drawer_joint_z = apron_bottom_z + drawer_front_height / 2 + 0.001
    model.articulation(
        "frame_to_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(0.0, front_y - apron_thickness / 2, drawer_joint_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=drawer_slide_travel,
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

    frame = object_model.get_part("frame")
    drawer = object_model.get_part("drawer")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")
    drawer_slide = object_model.get_articulation("frame_to_drawer")

    ctx.check(
        "drawer uses front-to-back prismatic slide",
        drawer_slide.axis == (0.0, -1.0, 0.0)
        and drawer_slide.motion_limits is not None
        and drawer_slide.motion_limits.lower == 0.0
        and drawer_slide.motion_limits.upper == 0.28,
        details=(
            f"axis={drawer_slide.axis}, limits="
            f"{drawer_slide.motion_limits.lower if drawer_slide.motion_limits else None}, "
            f"{drawer_slide.motion_limits.upper if drawer_slide.motion_limits else None}"
        ),
    )

    for leg in (front_left_leg, front_right_leg, rear_left_leg, rear_right_leg):
        ctx.expect_contact(
            frame,
            leg,
            name=f"{leg.name} is mounted to the table frame",
        )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_gap(
            frame,
            drawer,
            axis="x",
            positive_elem="front_right_stile",
            negative_elem="drawer_front",
            min_gap=0.004,
            max_gap=0.006,
            name="drawer front has a tight right-side reveal",
        )
        ctx.expect_gap(
            drawer,
            frame,
            axis="x",
            positive_elem="drawer_front",
            negative_elem="front_left_stile",
            min_gap=0.004,
            max_gap=0.006,
            name="drawer front has a tight left-side reveal",
        )
        ctx.expect_gap(
            frame,
            drawer,
            axis="z",
            positive_elem="top_rail",
            negative_elem="drawer_front",
            min_gap=0.001,
            max_gap=0.003,
            name="drawer front clears the top rail with a narrow shadow gap",
        )
        ctx.expect_contact(
            frame,
            drawer,
            elem_a="left_runner",
            elem_b="drawer_bottom",
            name="drawer bottom bears on the left wooden runner",
        )
        ctx.expect_contact(
            frame,
            drawer,
            elem_a="right_runner",
            elem_b="drawer_bottom",
            name="drawer bottom bears on the right wooden runner",
        )
        closed_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_slide: 0.28}):
        ctx.expect_overlap(
            frame,
            drawer,
            axes="y",
            elem_a="left_runner",
            elem_b="drawer_bottom",
            min_overlap=0.10,
            name="left runner retains drawer engagement at full extension",
        )
        ctx.expect_overlap(
            frame,
            drawer,
            axes="y",
            elem_a="right_runner",
            elem_b="drawer_bottom",
            min_overlap=0.10,
            name="right runner retains drawer engagement at full extension",
        )
        open_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward from the front apron",
        closed_pos is not None and open_pos is not None and open_pos[1] < closed_pos[1] - 0.20,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
