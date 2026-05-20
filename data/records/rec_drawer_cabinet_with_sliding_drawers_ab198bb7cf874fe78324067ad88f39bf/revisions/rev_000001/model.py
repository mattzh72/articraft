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
    model = ArticulatedObject(name="stacking_single_drawer_storage_unit")

    body_color = model.material("body_color", rgba=(0.78, 0.80, 0.78, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.93, 0.94, 0.95, 1.0))
    rail_color = model.material("rail_color", rgba=(0.66, 0.68, 0.69, 1.0))
    handle_color = model.material("handle_color", rgba=(0.24, 0.25, 0.27, 1.0))

    body_depth = 0.34
    body_width = 0.42
    body_height = 0.23
    wall_t = 0.008
    top_t = 0.010
    bottom_t = 0.010
    back_t = 0.008

    rail_length = 0.25
    rail_width = 0.013
    rail_height = 0.008
    rail_center_x = -0.025
    rail_center_y = 0.195
    rail_center_z = 0.036

    drawer_depth = 0.30
    drawer_width = 0.376
    drawer_height = 0.158
    drawer_wall_t = 0.006
    drawer_bottom_t = 0.006
    front_t = 0.014
    front_width = 0.404
    front_height = 0.185
    runner_length = 0.22
    runner_width = 0.011
    runner_height = 0.006
    runner_center_x = -0.035
    runner_center_y = 0.1935
    runner_center_z = -0.059

    drawer_joint_x = 0.013
    drawer_joint_z = 0.102
    drawer_travel = 0.185

    peg_positions = {
        "front_left": (0.108, 0.145),
        "front_right": (0.108, -0.145),
        "rear_left": (-0.108, 0.145),
        "rear_right": (-0.108, -0.145),
    }

    body = model.part("body")
    body.visual(
        Box((body_depth, body_width, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_t / 2.0)),
        material=body_color,
        name="top_panel",
    )
    body.visual(
        Box((body_depth, body_width, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_color,
        name="bottom_panel",
    )
    body.visual(
        Box((body_depth, wall_t, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall_t / 2.0, body_height / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((body_depth, wall_t, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall_t / 2.0, body_height / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((back_t, body_width - 2.0 * wall_t + 0.004, body_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + back_t / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="back_panel",
    )
    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(rail_center_x, rail_center_y, rail_center_z)),
        material=rail_color,
        name="left_rail",
    )
    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(rail_center_x, -rail_center_y, rail_center_z)),
        material=rail_color,
        name="right_rail",
    )
    body.visual(
        Box((0.022, rail_width, 0.022)),
        origin=Origin(xyz=(-0.145, rail_center_y, 0.021)),
        material=body_color,
        name="left_rail_rear_bracket",
    )
    body.visual(
        Box((0.022, rail_width, 0.022)),
        origin=Origin(xyz=(-0.145, -rail_center_y, 0.021)),
        material=body_color,
        name="right_rail_rear_bracket",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((drawer_depth, drawer_width, drawer_bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, -drawer_height / 2.0 + drawer_bottom_t / 2.0)),
        material=drawer_color,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_depth, drawer_wall_t, drawer_height)),
        origin=Origin(xyz=(0.0, drawer_width / 2.0 - drawer_wall_t / 2.0, 0.0)),
        material=drawer_color,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_depth, drawer_wall_t, drawer_height)),
        origin=Origin(xyz=(0.0, -drawer_width / 2.0 + drawer_wall_t / 2.0, 0.0)),
        material=drawer_color,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_wall_t, drawer_width, drawer_height)),
        origin=Origin(xyz=(-drawer_depth / 2.0 + drawer_wall_t / 2.0, 0.0, 0.0)),
        material=drawer_color,
        name="drawer_back",
    )
    drawer.visual(
        Box((front_t, front_width, front_height)),
        origin=Origin(
            xyz=(drawer_depth / 2.0 + front_t / 2.0, 0.0, (front_height - drawer_height) / 2.0)
        ),
        material=drawer_color,
        name="drawer_front",
    )
    drawer.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_center_x, runner_center_y, runner_center_z)),
        material=rail_color,
        name="left_runner",
    )
    drawer.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_center_x, -runner_center_y, runner_center_z)),
        material=rail_color,
        name="right_runner",
    )
    drawer.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(drawer_depth / 2.0 + front_t + 0.006, 0.075, 0.010)),
        material=handle_color,
        name="left_handle_post",
    )
    drawer.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(drawer_depth / 2.0 + front_t + 0.006, -0.075, 0.010)),
        material=handle_color,
        name="right_handle_post",
    )
    drawer.visual(
        Box((0.022, 0.190, 0.018)),
        origin=Origin(xyz=(drawer_depth / 2.0 + front_t + 0.017, 0.0, 0.010)),
        material=handle_color,
        name="pull_bar",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.44, 0.42, 0.20)),
        mass=1.1,
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(drawer_joint_x, 0.0, drawer_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    peg_size = 0.026
    peg_height = 0.010
    peg_cap_size = 0.018
    peg_cap_height = 0.004
    socket_outer = 0.036
    socket_inner = 0.028
    socket_cap_t = 0.003
    socket_depth = 0.014
    socket_wall_t = (socket_outer - socket_inner) / 2.0
    socket_wall_h = socket_depth - socket_cap_t

    for label, (pos_x, pos_y) in peg_positions.items():
        peg = model.part(f"peg_{label}")
        peg.visual(
            Box((peg_size, peg_size, peg_height)),
            origin=Origin(xyz=(0.0, 0.0, peg_height / 2.0)),
            material=body_color,
            name="peg_stem",
        )
        peg.visual(
            Box((peg_cap_size, peg_cap_size, peg_cap_height)),
            origin=Origin(xyz=(0.0, 0.0, peg_height + peg_cap_height / 2.0)),
            material=body_color,
            name="peg_cap",
        )
        model.articulation(
            f"body_to_peg_{label}",
            ArticulationType.FIXED,
            parent=body,
            child=peg,
            origin=Origin(xyz=(pos_x, pos_y, body_height)),
        )

        socket = model.part(f"socket_{label}")
        socket.visual(
            Box((socket_outer, socket_outer, socket_cap_t)),
            origin=Origin(xyz=(0.0, 0.0, -socket_cap_t / 2.0)),
            material=body_color,
            name="socket_cap",
        )
        socket.visual(
            Box((socket_wall_t, socket_outer, socket_wall_h)),
            origin=Origin(
                xyz=(socket_inner / 2.0 + socket_wall_t / 2.0, 0.0, -socket_cap_t - socket_wall_h / 2.0)
            ),
            material=body_color,
            name="socket_left_wall",
        )
        socket.visual(
            Box((socket_wall_t, socket_outer, socket_wall_h)),
            origin=Origin(
                xyz=(-socket_inner / 2.0 - socket_wall_t / 2.0, 0.0, -socket_cap_t - socket_wall_h / 2.0)
            ),
            material=body_color,
            name="socket_right_wall",
        )
        socket.visual(
            Box((socket_inner, socket_wall_t, socket_wall_h)),
            origin=Origin(
                xyz=(0.0, socket_inner / 2.0 + socket_wall_t / 2.0, -socket_cap_t - socket_wall_h / 2.0)
            ),
            material=body_color,
            name="socket_front_wall",
        )
        socket.visual(
            Box((socket_inner, socket_wall_t, socket_wall_h)),
            origin=Origin(
                xyz=(0.0, -socket_inner / 2.0 - socket_wall_t / 2.0, -socket_cap_t - socket_wall_h / 2.0)
            ),
            material=body_color,
            name="socket_rear_wall",
        )
        model.articulation(
            f"body_to_socket_{label}",
            ArticulationType.FIXED,
            parent=body,
            child=socket,
            origin=Origin(xyz=(pos_x, pos_y, 0.0)),
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

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    peg_front_left = object_model.get_part("peg_front_left")
    socket_front_left = object_model.get_part("socket_front_left")

    rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.014,
            name="drawer stays centered in the cabinet opening at rest",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="left_runner",
            negative_elem="left_rail",
            max_gap=0.0005,
            max_penetration=0.0,
            name="left runner sits on the left guide rail at rest",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="left_runner",
            elem_b="left_rail",
            min_overlap=0.20,
            name="left runner is deeply engaged with the rail at rest",
        )
        ctx.expect_contact(
            peg_front_left,
            body,
            name="front left stacking peg is mounted to the top panel",
        )
        ctx.expect_contact(
            socket_front_left,
            body,
            name="front left stacking socket is mounted to the base",
        )
        ctx.expect_origin_distance(
            peg_front_left,
            socket_front_left,
            axes="xy",
            max_dist=1e-6,
            name="front left peg aligns with its matching bottom socket",
        )

    upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({drawer_slide: upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                margin=0.014,
                name="drawer remains laterally captured when fully extended",
            )
            ctx.expect_gap(
                drawer,
                body,
                axis="z",
                positive_elem="left_runner",
                negative_elem="left_rail",
                max_gap=0.0005,
                max_penetration=0.0,
                name="left runner stays supported by the guide rail when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="left_runner",
                elem_b="left_rail",
                min_overlap=0.045,
                name="left runner retains insertion on the rail at full extension",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends outward along +x",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
