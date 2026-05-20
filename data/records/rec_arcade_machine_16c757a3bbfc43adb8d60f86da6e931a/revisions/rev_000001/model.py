from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_side_arcade_cabinet")

    cabinet_width = 0.82
    cabinet_depth = 0.92
    cabinet_height = 1.88
    wall_thickness = 0.03

    model.material("cabinet_red", rgba=(0.60, 0.08, 0.08, 1.0))
    model.material("cabinet_black", rgba=(0.10, 0.10, 0.12, 1.0))
    model.material("cabinet_trim", rgba=(0.82, 0.14, 0.12, 1.0))
    model.material("screen_black", rgba=(0.03, 0.03, 0.04, 1.0))
    model.material("coin_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("drawer_gray", rgba=(0.36, 0.38, 0.42, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((cabinet_width, cabinet_depth, 0.07)),
        origin=Origin(xyz=(0.0, cabinet_depth / 2.0, 0.035)),
        material="cabinet_black",
        name="plinth",
    )
    body.visual(
        Box((wall_thickness, cabinet_depth, 0.84)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, cabinet_depth / 2.0, 0.42)),
        material="cabinet_red",
        name="left_lower_side",
    )
    body.visual(
        Box((wall_thickness, 0.32, 1.06)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, 0.76, 1.29)),
        material="cabinet_red",
        name="left_rear_side",
    )
    body.visual(
        Box((wall_thickness, 0.40, 0.12)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, 0.22, 0.91), rpy=(-0.30, 0.0, 0.0)),
        material="cabinet_red",
        name="left_control_cheek",
    )
    body.visual(
        Box((wall_thickness, 0.40, 0.62)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, 0.41, 1.31), rpy=(-0.38, 0.0, 0.0)),
        material="cabinet_red",
        name="left_monitor_cheek",
    )
    body.visual(
        Box((wall_thickness, 0.22, 0.18)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, 0.49, 1.75)),
        material="cabinet_red",
        name="left_marquee_cheek",
    )
    body.visual(
        Box((wall_thickness, cabinet_depth, 0.84)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, cabinet_depth / 2.0, 0.42)),
        material="cabinet_red",
        name="right_lower_side",
    )
    body.visual(
        Box((wall_thickness, 0.32, 1.06)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, 0.76, 1.29)),
        material="cabinet_red",
        name="right_rear_side",
    )
    body.visual(
        Box((wall_thickness, 0.40, 0.12)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, 0.22, 0.91), rpy=(-0.30, 0.0, 0.0)),
        material="cabinet_red",
        name="right_control_cheek",
    )
    body.visual(
        Box((wall_thickness, 0.40, 0.62)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, 0.41, 1.31), rpy=(-0.38, 0.0, 0.0)),
        material="cabinet_red",
        name="right_monitor_cheek",
    )
    body.visual(
        Box((wall_thickness, 0.22, 0.18)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, 0.49, 1.75)),
        material="cabinet_red",
        name="right_marquee_cheek",
    )
    body.visual(
        Box((cabinet_width, 0.02, 0.46)),
        origin=Origin(xyz=(0.0, 0.01, 0.34)),
        material="cabinet_red",
        name="lower_front_panel",
    )
    body.visual(
        Box((cabinet_width, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, 0.02, 0.81)),
        material="cabinet_trim",
        name="front_top_rail",
    )
    body.visual(
        Box((cabinet_width - 2.0 * wall_thickness, wall_thickness, 0.74)),
        origin=Origin(xyz=(0.0, cabinet_depth - wall_thickness / 2.0, 0.37)),
        material="cabinet_red",
        name="rear_lower_panel",
    )
    body.visual(
        Box((0.115, 0.46, 0.03)),
        origin=Origin(xyz=(-0.3225, 0.31, 0.685)),
        material="drawer_gray",
        name="runner_left",
    )
    body.visual(
        Box((0.115, 0.46, 0.03)),
        origin=Origin(xyz=(0.3225, 0.31, 0.685)),
        material="drawer_gray",
        name="runner_right",
    )
    body.visual(
        Box((0.56, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.49, 0.68)),
        material="cabinet_black",
        name="drawer_backstop",
    )
    body.visual(
        Box((cabinet_width, 0.28, 0.78)),
        origin=Origin(xyz=(0.0, 0.69, 1.35)),
        material="cabinet_red",
        name="rear_upper_shell",
    )
    body.visual(
        Box((cabinet_width, 0.36, 0.10)),
        origin=Origin(xyz=(0.0, 0.26, 0.93), rpy=(-0.30, 0.0, 0.0)),
        material="cabinet_black",
        name="control_panel_housing",
    )
    body.visual(
        Box((0.70, 0.28, 0.018)),
        origin=Origin(xyz=(0.0, 0.225, 0.975), rpy=(-0.30, 0.0, 0.0)),
        material="cabinet_trim",
        name="control_deck",
    )
    body.visual(
        Box((cabinet_width, 0.34, 0.70)),
        origin=Origin(xyz=(0.0, 0.42, 1.33), rpy=(-0.38, 0.0, 0.0)),
        material="cabinet_red",
        name="monitor_head",
    )
    body.visual(
        Box((0.66, 0.03, 0.46)),
        origin=Origin(xyz=(0.0, 0.225, 1.26), rpy=(-0.38, 0.0, 0.0)),
        material="cabinet_black",
        name="monitor_bezel",
    )
    body.visual(
        Box((0.56, 0.01, 0.36)),
        origin=Origin(xyz=(0.0, 0.214, 1.255), rpy=(-0.38, 0.0, 0.0)),
        material="screen_black",
        name="screen",
    )
    body.visual(
        Box((cabinet_width, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, 0.49, 1.76)),
        material="cabinet_trim",
        name="marquee_box",
    )
    body.visual(
        Box((0.16, 0.012, 0.02)),
        origin=Origin(xyz=(0.0, 0.021, 0.54)),
        material="coin_metal",
        name="coin_slot_trim",
    )
    body.visual(
        Box((0.13, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.024, 0.47)),
        material="coin_metal",
        name="coin_return_frame",
    )

    drawer = model.part("ticket_drawer")
    drawer.visual(
        Box((0.56, 0.026, 0.20)),
        origin=Origin(xyz=(0.0, 0.019, 0.10)),
        material="cabinet_trim",
        name="drawer_face",
    )
    drawer.visual(
        Box((0.50, 0.40, 0.018)),
        origin=Origin(xyz=(0.0, 0.236, 0.009)),
        material="drawer_gray",
        name="drawer_box_bottom",
    )
    drawer.visual(
        Box((0.015, 0.39, 0.17)),
        origin=Origin(xyz=(-0.2575, 0.225, 0.094)),
        material="drawer_gray",
        name="drawer_side_left",
    )
    drawer.visual(
        Box((0.015, 0.39, 0.17)),
        origin=Origin(xyz=(0.2575, 0.225, 0.094)),
        material="drawer_gray",
        name="drawer_side_right",
    )
    drawer.visual(
        Box((0.50, 0.015, 0.17)),
        origin=Origin(xyz=(0.0, 0.4325, 0.094)),
        material="drawer_gray",
        name="drawer_rear",
    )
    drawer.visual(
        Box((0.16, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.003, 0.10)),
        material="coin_metal",
        name="drawer_pull",
    )

    coin_flap = model.part("coin_return_flap")
    coin_flap.visual(
        Box((0.12, 0.014, 0.05)),
        origin=Origin(xyz=(0.0, 0.007, -0.025)),
        material="coin_metal",
        name="flap_panel",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.018, 0.58, 0.86)),
        origin=Origin(xyz=(0.009, 0.29, 0.43)),
        material="cabinet_black",
        name="service_panel",
    )
    service_door.visual(
        Box((0.02, 0.10, 0.03)),
        origin=Origin(xyz=(0.019, 0.45, 0.43)),
        material="coin_metal",
        name="service_handle",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.4,
            lower=0.0,
            upper=0.28,
        ),
    )
    model.articulation(
        "coin_return_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=coin_flap,
        origin=Origin(xyz=(0.0, 0.028, 0.47)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "service_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(cabinet_width / 2.0, 0.19, 0.18)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.6,
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

    body = object_model.get_part("cabinet_body")
    drawer = object_model.get_part("ticket_drawer")
    coin_flap = object_model.get_part("coin_return_flap")
    service_door = object_model.get_part("service_door")

    drawer_slide = object_model.get_articulation("drawer_slide")
    coin_return_hinge = object_model.get_articulation("coin_return_hinge")
    service_door_hinge = object_model.get_articulation("service_door_hinge")

    ctx.expect_gap(
        drawer,
        body,
        axis="z",
        positive_elem="drawer_face",
        negative_elem="lower_front_panel",
        min_gap=0.0,
        max_gap=0.03,
        name="ticket drawer sits directly above the lower front panel",
    )
    ctx.expect_gap(
        coin_flap,
        body,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="coin_return_frame",
        min_gap=0.0,
        max_gap=0.001,
        name="coin return flap closes against the coin door frame",
    )
    ctx.expect_gap(
        service_door,
        body,
        axis="x",
        positive_elem="service_panel",
        min_gap=0.0,
        max_gap=0.03,
        name="service door closes flush to the cabinet side",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        drawer_open = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="xz",
            min_overlap=0.18,
            name="drawer remains captured within the cabinet width and height when extended",
        )
    ctx.check(
        "ticket drawer slides straight forward on runners",
        drawer_rest is not None
        and drawer_open is not None
        and abs(drawer_open[0] - drawer_rest[0]) < 1e-6
        and abs(drawer_open[2] - drawer_rest[2]) < 1e-6
        and drawer_open[1] < drawer_rest[1] - 0.20,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    flap_closed = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")
    with ctx.pose({coin_return_hinge: coin_return_hinge.motion_limits.upper}):
        flap_open = ctx.part_element_world_aabb(coin_flap, elem="flap_panel")
    ctx.check(
        "coin return flap opens outward from the coin door",
        flap_closed is not None
        and flap_open is not None
        and flap_open[0][1] < flap_closed[0][1] - 0.02,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    service_closed = ctx.part_element_world_aabb(service_door, elem="service_panel")
    with ctx.pose({service_door_hinge: service_door_hinge.motion_limits.upper}):
        service_open = ctx.part_element_world_aabb(service_door, elem="service_panel")
    ctx.check(
        "service door swings outward on a vertical side hinge",
        service_closed is not None
        and service_open is not None
        and service_open[1][0] > service_closed[1][0] + 0.18,
        details=f"closed={service_closed}, open={service_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
