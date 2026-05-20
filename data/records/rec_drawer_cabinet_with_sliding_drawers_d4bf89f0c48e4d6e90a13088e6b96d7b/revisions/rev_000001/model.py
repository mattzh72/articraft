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


ROWS = 6
COLS = 4


def _drawer_name(row: int, col: int) -> str:
    return f"drawer_r{row + 1}_c{col + 1}"


def _slide_name(row: int, col: int) -> str:
    return f"body_to_drawer_r{row + 1}_c{col + 1}"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_parts_sorter_cabinet")

    body_steel = model.material("body_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    drawer_blue = model.material("drawer_blue", rgba=(0.28, 0.45, 0.64, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    label_grey = model.material("label_grey", rgba=(0.86, 0.87, 0.88, 1.0))

    body_w = 0.72
    body_d = 0.34
    cabinet_h = 1.54
    foot_h = 0.102
    wall_t = 0.018
    divider_t = 0.012
    back_t = 0.018

    cavity_w = (body_w - 2.0 * wall_t - (COLS - 1) * divider_t) / COLS
    cavity_h = (cabinet_h - 2.0 * wall_t - (ROWS - 1) * divider_t) / ROWS
    inner_front_y = body_d * 0.5
    inner_back_y = -body_d * 0.5 + back_t
    cavity_depth = inner_front_y - inner_back_y

    drawer_shell_w = 0.144
    drawer_shell_h = 0.226
    drawer_shell_d = 0.286
    drawer_front_w = 0.158
    drawer_front_h = 0.236
    drawer_front_t = 0.016
    drawer_wall_t = 0.004
    rail_t = 0.0035
    fixed_rail_t = 0.0063
    rail_gap = 0.002
    drawer_travel = 0.18
    drawer_closed_y = inner_front_y - drawer_shell_d * 0.5 + 0.0008

    body = model.part("cabinet_body")
    body.visual(
        Box((wall_t, body_d, cabinet_h)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, foot_h + cabinet_h * 0.5)),
        material=body_steel,
        name="left_side",
    )
    body.visual(
        Box((wall_t, body_d, cabinet_h)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, foot_h + cabinet_h * 0.5)),
        material=body_steel,
        name="right_side",
    )
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + wall_t * 0.5)),
        material=body_steel,
        name="bottom_panel",
    )
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + cabinet_h - wall_t * 0.5)),
        material=body_steel,
        name="top_panel",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, back_t, cabinet_h - 2.0 * wall_t)),
        origin=Origin(
            xyz=(0.0, -body_d * 0.5 + back_t * 0.5, foot_h + cabinet_h * 0.5)
        ),
        material=body_steel,
        name="back_panel",
    )

    for col in range(COLS - 1):
        x_center = (
            -body_w * 0.5
            + wall_t
            + (col + 1) * cavity_w
            + col * divider_t
            + divider_t * 0.5
        )
        body.visual(
            Box((divider_t, cavity_depth, cabinet_h - 2.0 * wall_t)),
            origin=Origin(xyz=(x_center, (inner_front_y + inner_back_y) * 0.5, foot_h + cabinet_h * 0.5)),
            material=body_steel,
            name=f"vertical_divider_{col + 1}",
        )

    for row in range(ROWS - 1):
        z_center = foot_h + wall_t + (row + 1) * cavity_h + row * divider_t + divider_t * 0.5
        body.visual(
            Box((body_w - 2.0 * wall_t, cavity_depth, divider_t)),
            origin=Origin(xyz=(0.0, (inner_front_y + inner_back_y) * 0.5, z_center)),
            material=body_steel,
            name=f"horizontal_divider_{row + 1}",
        )

    foot_w = 0.18
    foot_d = 0.28
    for index, x_center in enumerate((-0.19, 0.19), start=1):
        body.visual(
            Box((foot_w, foot_d, foot_h)),
            origin=Origin(xyz=(x_center, 0.0, foot_h * 0.5)),
            material=body_steel,
            name=f"foot_{index}",
        )

    fixed_rail_h = 0.018
    fixed_rail_d = 0.255
    fixed_rail_y = -0.010
    fixed_rail_z_offset = -0.046
    for row in range(ROWS):
        z_center = foot_h + wall_t + row * (cavity_h + divider_t) + cavity_h * 0.5
        for col in range(COLS):
            x_center = (
                -body_w * 0.5
                + wall_t
                + col * (cavity_w + divider_t)
                + cavity_w * 0.5
            )
            for side, sign in (("left", -1.0), ("right", 1.0)):
                body.visual(
                    Box((fixed_rail_t, fixed_rail_d, fixed_rail_h)),
                    origin=Origin(
                        xyz=(
                            x_center + sign * (cavity_w * 0.5 - fixed_rail_t * 0.5),
                            fixed_rail_y,
                            z_center + fixed_rail_z_offset,
                        )
                    ),
                    material=slide_steel,
                    name=f"fixed_slide_{row + 1}_{col + 1}_{side}",
                )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, foot_h + cabinet_h)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, (foot_h + cabinet_h) * 0.5)),
    )

    moving_rail_d = 0.252
    moving_rail_h = 0.016
    moving_rail_y = -0.008
    moving_rail_z = -0.040
    handle_w = 0.084
    handle_d = 0.016
    handle_h = 0.018
    label_w = 0.058
    label_t = 0.002
    label_h = 0.024
    label_y = drawer_shell_d * 0.5 + drawer_front_t + label_t * 0.5

    for row in range(ROWS):
        z_center = (
            foot_h
            + cabinet_h
            - wall_t
            - row * (cavity_h + divider_t)
            - cavity_h * 0.5
        )
        for col in range(COLS):
            x_center = (
                -body_w * 0.5
                + wall_t
                + col * (cavity_w + divider_t)
                + cavity_w * 0.5
            )
            drawer = model.part(_drawer_name(row, col))
            drawer.visual(
                Box((drawer_shell_w, drawer_shell_d, drawer_wall_t)),
                origin=Origin(xyz=(0.0, 0.0, -drawer_shell_h * 0.5 + drawer_wall_t * 0.5)),
                material=drawer_blue,
                name="tray_bottom",
            )
            drawer.visual(
                Box((drawer_wall_t, drawer_shell_d, drawer_shell_h)),
                origin=Origin(
                    xyz=(-drawer_shell_w * 0.5 + drawer_wall_t * 0.5, 0.0, 0.0)
                ),
                material=drawer_blue,
                name="tray_side_left",
            )
            drawer.visual(
                Box((drawer_wall_t, drawer_shell_d, drawer_shell_h)),
                origin=Origin(
                    xyz=(drawer_shell_w * 0.5 - drawer_wall_t * 0.5, 0.0, 0.0)
                ),
                material=drawer_blue,
                name="tray_side_right",
            )
            drawer.visual(
                Box((drawer_shell_w, drawer_wall_t, drawer_shell_h)),
                origin=Origin(
                    xyz=(0.0, -drawer_shell_d * 0.5 + drawer_wall_t * 0.5, 0.0)
                ),
                material=drawer_blue,
                name="tray_back",
            )
            drawer.visual(
                Box((drawer_front_w, drawer_front_t, drawer_front_h)),
                origin=Origin(
                    xyz=(0.0, drawer_shell_d * 0.5 + drawer_front_t * 0.5, 0.0)
                ),
                material=drawer_blue,
                name="front_panel",
            )
            drawer.visual(
                Box((handle_w, handle_d, handle_h)),
                origin=Origin(
                    xyz=(
                        0.0,
                        drawer_shell_d * 0.5 + drawer_front_t + handle_d * 0.5 - 0.004,
                        0.045,
                    )
                ),
                material=handle_dark,
                name="pull_handle",
            )
            drawer.visual(
                Box((label_w, label_t, label_h)),
                origin=Origin(xyz=(0.0, label_y, -0.040)),
                material=label_grey,
                name="label_plate",
            )
            for side, sign in (("left", -1.0), ("right", 1.0)):
                drawer.visual(
                    Box((rail_t, moving_rail_d, moving_rail_h)),
                    origin=Origin(
                        xyz=(
                            sign * (drawer_shell_w * 0.5 + rail_t * 0.5 - 0.0008),
                            moving_rail_y,
                            moving_rail_z,
                        )
                    ),
                    material=slide_steel,
                    name=f"moving_slide_{side}",
                )

            drawer.inertial = Inertial.from_geometry(
                Box((drawer_front_w, drawer_shell_d + drawer_front_t, drawer_front_h)),
                mass=1.2,
                origin=Origin(xyz=(0.0, 0.008, 0.0)),
            )

            model.articulation(
                _slide_name(row, col),
                ArticulationType.PRISMATIC,
                parent=body,
                child=drawer,
                origin=Origin(xyz=(x_center, drawer_closed_y, z_center)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=90.0,
                    velocity=0.30,
                    lower=0.0,
                    upper=drawer_travel,
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

    drawers = []
    joints = []
    articulation_ok = True
    articulation_details = []
    for row in range(ROWS):
        for col in range(COLS):
            drawer = object_model.get_part(_drawer_name(row, col))
            joint = object_model.get_articulation(_slide_name(row, col))
            drawers.append(drawer)
            joints.append(joint)
            limits = joint.motion_limits
            joint_ok = (
                joint.articulation_type == ArticulationType.PRISMATIC
                and joint.axis == (0.0, 1.0, 0.0)
                and limits is not None
                and limits.lower == 0.0
                and limits.upper == 0.18
            )
            articulation_ok = articulation_ok and joint_ok
            if not joint_ok:
                articulation_details.append(
                    f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}, limits={limits}"
                )

    ctx.check(
        "cabinet has twenty-four drawer bins",
        len(drawers) == 24 and len(joints) == 24,
        details=f"parts={len(drawers)}, joints={len(joints)}",
    )
    ctx.check(
        "all drawers use forward prismatic slides",
        articulation_ok,
        details="; ".join(articulation_details),
    )

    body_aabb = ctx.part_world_aabb(body)
    body_dims_ok = False
    if body_aabb is not None:
        min_corner, max_corner = body_aabb
        width = max_corner[0] - min_corner[0]
        depth = max_corner[1] - min_corner[1]
        height = max_corner[2] - min_corner[2]
        body_dims_ok = (
            abs(min_corner[2]) <= 1e-6
            and height > 1.55
            and height > 2.1 * depth
            and width > depth
        )
        dims_details = (
            f"min={min_corner}, max={max_corner}, "
            f"width={width:.3f}, depth={depth:.3f}, height={height:.3f}"
        )
    else:
        dims_details = "missing body AABB"
    ctx.check("cabinet stands on two floor feet with narrow-tall proportions", body_dims_ok, dims_details)

    left_foot_aabb = ctx.part_element_world_aabb(body, elem="foot_1")
    right_foot_aabb = ctx.part_element_world_aabb(body, elem="foot_2")
    feet_ok = (
        left_foot_aabb is not None
        and right_foot_aabb is not None
        and abs(left_foot_aabb[0][2]) <= 1e-6
        and abs(right_foot_aabb[0][2]) <= 1e-6
        and left_foot_aabb[1][0] < 0.0
        and right_foot_aabb[0][0] > 0.0
    )
    ctx.check(
        "cabinet body is carried by two floor feet",
        feet_ok,
        details=f"left={left_foot_aabb}, right={right_foot_aabb}",
    )

    representative_pairs = (
        (_drawer_name(0, 0), _slide_name(0, 0)),
        (_drawer_name(2, 1), _slide_name(2, 1)),
        (_drawer_name(5, 3), _slide_name(5, 3)),
    )
    for drawer_name, joint_name in representative_pairs:
        drawer = object_model.get_part(drawer_name)
        joint = object_model.get_articulation(joint_name)

        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem="front_panel",
            min_gap=0.0002,
            max_gap=0.0020,
            name=f"{drawer_name} front panel sits just proud of cabinet face",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            inner_elem="front_panel",
            margin=0.0025,
            name=f"{drawer_name} front panel stays centered in its column",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: joint.motion_limits.upper}):
            ctx.expect_gap(
                drawer,
                body,
                axis="y",
                positive_elem="front_panel",
                min_gap=0.175,
                max_gap=0.185,
                name=f"{drawer_name} opens outward on its slides",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="tray_bottom",
                min_overlap=0.10,
                name=f"{drawer_name} retains slide insertion when extended",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer_name} translates forward",
            rest_pos is not None and open_pos is not None and open_pos[1] > rest_pos[1] + 0.17,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
