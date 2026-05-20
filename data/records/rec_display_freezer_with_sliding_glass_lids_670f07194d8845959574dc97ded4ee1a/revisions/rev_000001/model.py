from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frozen_food_merchandiser")

    cabinet_white = model.material("cabinet_white", rgba=(0.90, 0.92, 0.94, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.99, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.86, 0.94, 0.35))

    cabinet = model.part("cabinet")

    length = 2.40
    width = 1.00
    height = 0.95
    plinth_h = 0.08
    wall_t = 0.045
    body_h = 0.82
    top_frame_top = height
    top_frame_z = plinth_h + body_h
    top_frame_h = top_frame_top - top_frame_z
    floor_t = 0.03

    inner_length = length - 2.0 * wall_t
    inner_width = width - 2.0 * wall_t

    front_frame_w = 0.07
    end_frame_w = 0.06
    opening_half_y = 0.395
    opening_half_x = 1.115
    opening_length = 2.0 * opening_half_x
    opening_width = 2.0 * opening_half_y

    cabinet.visual(
        Box((length, width, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h / 2.0)),
        material=trim_gray,
        name="plinth",
    )
    cabinet.visual(
        Box((length, wall_t, body_h)),
        origin=Origin(
            xyz=(0.0, width / 2.0 - wall_t / 2.0, plinth_h + body_h / 2.0)
        ),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((length, wall_t, body_h)),
        origin=Origin(
            xyz=(0.0, -(width / 2.0 - wall_t / 2.0), plinth_h + body_h / 2.0)
        ),
        material=cabinet_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((wall_t, inner_width, body_h)),
        origin=Origin(
            xyz=(-(length / 2.0 - wall_t / 2.0), 0.0, plinth_h + body_h / 2.0)
        ),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_t, inner_width, body_h)),
        origin=Origin(
            xyz=(length / 2.0 - wall_t / 2.0, 0.0, plinth_h + body_h / 2.0)
        ),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((inner_length, inner_width, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + floor_t / 2.0)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((inner_length, 0.012, body_h - 0.08)),
        origin=Origin(xyz=(0.0, inner_width / 2.0 - 0.006, plinth_h + 0.36)),
        material=liner_white,
        name="front_liner",
    )
    cabinet.visual(
        Box((inner_length, 0.012, body_h - 0.08)),
        origin=Origin(xyz=(0.0, -(inner_width / 2.0 - 0.006), plinth_h + 0.36)),
        material=liner_white,
        name="rear_liner",
    )
    cabinet.visual(
        Box((0.012, inner_width, body_h - 0.08)),
        origin=Origin(xyz=(-(inner_length / 2.0 - 0.006), 0.0, plinth_h + 0.36)),
        material=liner_white,
        name="left_liner",
    )
    cabinet.visual(
        Box((0.012, inner_width, body_h - 0.08)),
        origin=Origin(xyz=(inner_length / 2.0 - 0.006, 0.0, plinth_h + 0.36)),
        material=liner_white,
        name="right_liner",
    )
    cabinet.visual(
        Box((inner_length, front_frame_w, top_frame_h)),
        origin=Origin(xyz=(0.0, 0.430, top_frame_z + top_frame_h / 2.0)),
        material=aluminum,
        name="front_frame",
    )
    cabinet.visual(
        Box((inner_length, front_frame_w, top_frame_h)),
        origin=Origin(xyz=(0.0, -0.430, top_frame_z + top_frame_h / 2.0)),
        material=aluminum,
        name="rear_frame",
    )
    cabinet.visual(
        Box((end_frame_w, opening_width, top_frame_h)),
        origin=Origin(xyz=(-1.145, 0.0, top_frame_z + top_frame_h / 2.0)),
        material=aluminum,
        name="left_end_frame",
    )
    cabinet.visual(
        Box((end_frame_w, opening_width, top_frame_h)),
        origin=Origin(xyz=(1.145, 0.0, top_frame_z + top_frame_h / 2.0)),
        material=aluminum,
        name="right_end_frame",
    )

    basket_rail_h = 0.012
    basket_rail_y = 0.377
    basket_rail_z = 0.890 - basket_rail_h / 2.0
    basket_rail_len = opening_length + 0.04
    cabinet.visual(
        Box((basket_rail_len, 0.012, basket_rail_h)),
        origin=Origin(xyz=(0.0, basket_rail_y, basket_rail_z)),
        material=aluminum,
        name="front_basket_rail",
    )
    cabinet.visual(
        Box((basket_rail_len, 0.012, basket_rail_h)),
        origin=Origin(xyz=(0.0, -basket_rail_y, basket_rail_z)),
        material=aluminum,
        name="rear_basket_rail",
    )
    for x_sign, side in ((-1.0, "left"), (1.0, "right")):
        x_pos = x_sign * 1.10
        cabinet.visual(
            Box((0.05, 0.020, 0.022)),
            origin=Origin(xyz=(x_pos, 0.389, 0.895)),
            material=aluminum,
            name=f"front_basket_bracket_{side}",
        )
        cabinet.visual(
            Box((0.05, 0.020, 0.022)),
            origin=Origin(xyz=(x_pos, -0.389, 0.895)),
            material=aluminum,
            name=f"rear_basket_bracket_{side}",
        )

    track_len = opening_length - 0.02
    track_w = 0.010
    track_specs = (
        ("low", 0.399, 0.959),
        ("mid", 0.417, 0.983),
        ("high", 0.435, 1.007),
    )
    for prefix, y_abs, top_z in track_specs:
        track_h = top_z - top_frame_top
        center_z = top_frame_top + track_h / 2.0
        cabinet.visual(
            Box((track_len, track_w, track_h)),
            origin=Origin(xyz=(0.0, y_abs, center_z)),
            material=aluminum,
            name=f"{prefix}_front_track",
        )
        cabinet.visual(
            Box((track_len, track_w, track_h)),
            origin=Origin(xyz=(0.0, -y_abs, center_z)),
            material=aluminum,
            name=f"{prefix}_rear_track",
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((length, width, height)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    lid_length = 0.84
    side_rail_w = 0.014
    end_rail_w = 0.03
    frame_t = 0.018
    glass_t = 0.006
    runner_len = lid_length - 0.04
    runner_h = 0.008
    runner_center_z = -0.011
    runner_bottom_offset = abs(runner_center_z) + runner_h / 2.0
    handle_z = 0.007

    def add_lid(
        name: str,
        joint_name: str,
        closed_x: float,
        runner_y: float,
        runner_top_z: float,
        axis_x: float,
        lower: float,
        upper: float,
    ) -> None:
        lid = model.part(name)
        lid_width = 2.0 * (runner_y + 0.010)
        side_rail_center_y = runner_y + 0.003
        lid.visual(
            Box((lid_length, side_rail_w, frame_t)),
            origin=Origin(xyz=(0.0, side_rail_center_y, 0.0)),
            material=aluminum,
            name="front_rail",
        )
        lid.visual(
            Box((lid_length, side_rail_w, frame_t)),
            origin=Origin(xyz=(0.0, -side_rail_center_y, 0.0)),
            material=aluminum,
            name="rear_rail",
        )
        lid.visual(
            Box((end_rail_w, lid_width - 2.0 * side_rail_w, frame_t)),
            origin=Origin(xyz=(-(lid_length / 2.0 - end_rail_w / 2.0), 0.0, 0.0)),
            material=aluminum,
            name="left_stile",
        )
        lid.visual(
            Box((end_rail_w, lid_width - 2.0 * side_rail_w, frame_t)),
            origin=Origin(xyz=(lid_length / 2.0 - end_rail_w / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="right_stile",
        )
        lid.visual(
            Box((lid_length - 0.04, lid_width - 0.04, glass_t)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=glass,
            name="glass_panel",
        )
        lid.visual(
            Box((runner_len, track_w, runner_h)),
            origin=Origin(xyz=(0.0, runner_y, runner_center_z)),
            material=trim_gray,
            name="runner_front",
        )
        lid.visual(
            Box((runner_len, track_w, runner_h)),
            origin=Origin(xyz=(0.0, -runner_y, runner_center_z)),
            material=trim_gray,
            name="runner_rear",
        )
        lid.visual(
            Box((0.13, 0.05, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, handle_z)),
            material=trim_gray,
            name="pull_handle",
        )
        lid.inertial = Inertial.from_geometry(
            Box((lid_length, lid_width, 0.05)),
            mass=9.0,
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=lid,
            origin=Origin(xyz=(closed_x, 0.0, runner_top_z + runner_bottom_offset)),
            axis=(axis_x, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=0.35,
                lower=lower,
                upper=upper,
            ),
        )

    add_lid(
        name="left_lid",
        joint_name="cabinet_to_left_lid",
        closed_x=-0.695,
        runner_y=0.401,
        runner_top_z=0.959,
        axis_x=1.0,
        lower=0.0,
        upper=0.55,
    )
    add_lid(
        name="center_lid",
        joint_name="cabinet_to_center_lid",
        closed_x=0.0,
        runner_y=0.415,
        runner_top_z=0.983,
        axis_x=1.0,
        lower=-0.30,
        upper=0.30,
    )
    add_lid(
        name="right_lid",
        joint_name="cabinet_to_right_lid",
        closed_x=0.695,
        runner_y=0.429,
        runner_top_z=1.007,
        axis_x=-1.0,
        lower=0.0,
        upper=0.55,
    )

    basket_length = 0.70
    basket_width = 0.74
    basket_depth = 0.30
    basket_wall_t = 0.006
    hanger_h = 0.012
    support_z = 0.890

    def add_basket(name: str, joint_name: str, center_x: float) -> None:
        basket = model.part(name)
        basket.visual(
            Box((basket_length, 0.012, hanger_h)),
            origin=Origin(xyz=(0.0, basket_width / 2.0, hanger_h / 2.0)),
            material=basket_gray,
            name="front_hanger",
        )
        basket.visual(
            Box((basket_length, 0.012, hanger_h)),
            origin=Origin(xyz=(0.0, -(basket_width / 2.0), hanger_h / 2.0)),
            material=basket_gray,
            name="rear_hanger",
        )
        basket.visual(
            Box((basket_length, basket_wall_t, basket_depth)),
            origin=Origin(
                xyz=(0.0, basket_width / 2.0 - basket_wall_t / 2.0, -basket_depth / 2.0)
            ),
            material=basket_gray,
            name="front_wall",
        )
        basket.visual(
            Box((basket_length, basket_wall_t, basket_depth)),
            origin=Origin(
                xyz=(0.0, -(basket_width / 2.0 - basket_wall_t / 2.0), -basket_depth / 2.0)
            ),
            material=basket_gray,
            name="rear_wall",
        )
        basket.visual(
            Box((basket_wall_t, basket_width - 2.0 * basket_wall_t, basket_depth)),
            origin=Origin(
                xyz=(
                    -(basket_length / 2.0 - basket_wall_t / 2.0),
                    0.0,
                    -basket_depth / 2.0,
                )
            ),
            material=basket_gray,
            name="left_wall",
        )
        basket.visual(
            Box((basket_wall_t, basket_width - 2.0 * basket_wall_t, basket_depth)),
            origin=Origin(
                xyz=(
                    basket_length / 2.0 - basket_wall_t / 2.0,
                    0.0,
                    -basket_depth / 2.0,
                )
            ),
            material=basket_gray,
            name="right_wall",
        )
        basket.visual(
            Box(
                (
                    basket_length - 2.0 * basket_wall_t,
                    basket_width - 2.0 * basket_wall_t,
                    0.016,
                )
            ),
            origin=Origin(xyz=(0.0, 0.0, -basket_depth + 0.008)),
            material=basket_gray,
            name="basket_bottom",
        )
        basket.inertial = Inertial.from_geometry(
            Box((basket_length, basket_width, basket_depth + hanger_h)),
            mass=3.0,
            origin=Origin(xyz=(0.0, 0.0, -0.14)),
        )

        model.articulation(
            joint_name,
            ArticulationType.FIXED,
            parent=cabinet,
            child=basket,
            origin=Origin(xyz=(center_x, 0.0, support_z)),
        )

    add_basket("left_basket", "cabinet_to_left_basket", -0.75)
    add_basket("center_basket", "cabinet_to_center_basket", 0.0)
    add_basket("right_basket", "cabinet_to_right_basket", 0.75)

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
    cabinet = object_model.get_part("cabinet")
    left_lid = object_model.get_part("left_lid")
    center_lid = object_model.get_part("center_lid")
    right_lid = object_model.get_part("right_lid")
    left_basket = object_model.get_part("left_basket")
    center_basket = object_model.get_part("center_basket")
    right_basket = object_model.get_part("right_basket")

    left_slide = object_model.get_articulation("cabinet_to_left_lid")
    center_slide = object_model.get_articulation("cabinet_to_center_lid")
    right_slide = object_model.get_articulation("cabinet_to_right_lid")

    low_front_track = cabinet.get_visual("low_front_track")
    low_rear_track = cabinet.get_visual("low_rear_track")
    mid_front_track = cabinet.get_visual("mid_front_track")
    mid_rear_track = cabinet.get_visual("mid_rear_track")
    high_front_track = cabinet.get_visual("high_front_track")
    high_rear_track = cabinet.get_visual("high_rear_track")
    front_basket_rail = cabinet.get_visual("front_basket_rail")
    rear_basket_rail = cabinet.get_visual("rear_basket_rail")

    left_runner_front = left_lid.get_visual("runner_front")
    left_runner_rear = left_lid.get_visual("runner_rear")
    center_runner_front = center_lid.get_visual("runner_front")
    center_runner_rear = center_lid.get_visual("runner_rear")
    right_runner_front = right_lid.get_visual("runner_front")
    right_runner_rear = right_lid.get_visual("runner_rear")

    for lid, runner_front, runner_rear, track_front, track_rear, prefix in (
        (left_lid, left_runner_front, left_runner_rear, low_front_track, low_rear_track, "left"),
        (
            center_lid,
            center_runner_front,
            center_runner_rear,
            mid_front_track,
            mid_rear_track,
            "center",
        ),
        (
            right_lid,
            right_runner_front,
            right_runner_rear,
            high_front_track,
            high_rear_track,
            "right",
        ),
    ):
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a=runner_front,
            elem_b=track_front,
            name=f"{prefix} lid front runner sits on its track",
        )
        ctx.expect_contact(
            lid,
            cabinet,
            elem_a=runner_rear,
            elem_b=track_rear,
            name=f"{prefix} lid rear runner sits on its track",
        )

    for basket, prefix in (
        (left_basket, "left"),
        (center_basket, "center"),
        (right_basket, "right"),
    ):
        ctx.expect_contact(
            basket,
            cabinet,
            elem_a=basket.get_visual("front_hanger"),
            elem_b=front_basket_rail,
            name=f"{prefix} basket front hanger rests on support rail",
        )
        ctx.expect_contact(
            basket,
            cabinet,
            elem_a=basket.get_visual("rear_hanger"),
            elem_b=rear_basket_rail,
            name=f"{prefix} basket rear hanger rests on support rail",
        )

    left_rest = ctx.part_world_position(left_lid)
    center_rest = ctx.part_world_position(center_lid)
    right_rest = ctx.part_world_position(right_lid)

    with ctx.pose({left_slide: left_slide.motion_limits.upper}):
        ctx.expect_contact(
            left_lid,
            cabinet,
            elem_a=left_runner_front,
            elem_b=low_front_track,
            name="left lid stays supported when opened",
        )
        left_open = ctx.part_world_position(left_lid)

    with ctx.pose({center_slide: center_slide.motion_limits.upper}):
        ctx.expect_contact(
            center_lid,
            cabinet,
            elem_a=center_runner_front,
            elem_b=mid_front_track,
            name="center lid stays supported at positive travel",
        )
        center_open = ctx.part_world_position(center_lid)

    with ctx.pose({right_slide: right_slide.motion_limits.upper}):
        ctx.expect_contact(
            right_lid,
            cabinet,
            elem_a=right_runner_front,
            elem_b=high_front_track,
            name="right lid stays supported when opened",
        )
        right_open = ctx.part_world_position(right_lid)

    ctx.check(
        "left lid slides toward the middle of the cabinet",
        left_rest is not None
        and left_open is not None
        and left_open[0] > left_rest[0] + 0.45,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "center lid positive travel moves rightward",
        center_rest is not None
        and center_open is not None
        and center_open[0] > center_rest[0] + 0.25,
        details=f"rest={center_rest}, open={center_open}",
    )
    ctx.check(
        "right lid positive travel moves leftward toward the middle",
        right_rest is not None
        and right_open is not None
        and right_open[0] < right_rest[0] - 0.45,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
