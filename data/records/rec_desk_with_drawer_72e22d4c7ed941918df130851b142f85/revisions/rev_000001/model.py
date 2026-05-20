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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_double_pedestal_desk")

    walnut = model.material("walnut", rgba=(0.35, 0.22, 0.14, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.24, 0.14, 0.08, 1.0))
    drawer_veneer = model.material("drawer_veneer", rgba=(0.42, 0.27, 0.17, 1.0))
    rail_graphite = model.material("rail_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.71, 0.73, 0.75, 1.0))

    desk_height = 0.76
    top_thickness = 0.04
    carcass_height = desk_height - top_thickness
    top_depth = 0.85
    top_width = 1.80
    pedestal_depth = 0.76
    pedestal_width = 0.46
    pedestal_center_x = -0.005
    drawer_front_x = pedestal_center_x + pedestal_depth / 2.0
    wall_thickness = 0.022
    back_thickness = 0.018
    floor_thickness = 0.022
    divider_thickness = 0.018
    interior_width = pedestal_width - 2.0 * wall_thickness
    interior_depth = drawer_front_x - (pedestal_center_x - pedestal_depth / 2.0 + back_thickness)
    drawer_box_width = 0.372
    drawer_front_width = 0.402
    drawer_side_thickness = 0.012
    drawer_bottom_thickness = 0.010
    drawer_back_thickness = 0.012

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((top_depth, top_width, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=walnut,
        name="desktop",
    )
    desk_frame.visual(
        Box((0.060, 0.780, 0.060)),
        origin=Origin(xyz=(0.345, 0.0, 0.690)),
        material=walnut_dark,
        name="center_apron",
    )
    desk_frame.visual(
        Box((0.018, 0.780, 0.580)),
        origin=Origin(xyz=(-0.367, 0.0, 0.290)),
        material=walnut_dark,
        name="modesty_panel",
    )

    drawer_bay_centers = {
        "bottom": 0.167,
        "middle": 0.420,
        "top": 0.593,
    }
    rail_x_center = (0.345 + (-0.135)) / 2.0
    rail_length = 0.480
    rail_width = 0.014
    rail_height = 0.016
    rail_offset = interior_width / 2.0 - rail_width / 2.0

    def add_pedestal_shell(prefix: str, center_y: float) -> None:
        side_sign = -1.0 if center_y < 0.0 else 1.0
        outer_wall_y = center_y + side_sign * (pedestal_width / 2.0 - wall_thickness / 2.0)
        inner_wall_y = center_y - side_sign * (pedestal_width / 2.0 - wall_thickness / 2.0)
        pedestal_back_x = pedestal_center_x - pedestal_depth / 2.0 + back_thickness / 2.0
        shelf_x_center = (drawer_front_x + (pedestal_center_x - pedestal_depth / 2.0 + back_thickness)) / 2.0

        desk_frame.visual(
            Box((pedestal_depth, wall_thickness, carcass_height)),
            origin=Origin(xyz=(pedestal_center_x, outer_wall_y, carcass_height / 2.0)),
            material=walnut_dark,
            name=f"{prefix}_outer_wall",
        )
        desk_frame.visual(
            Box((pedestal_depth, wall_thickness, carcass_height)),
            origin=Origin(xyz=(pedestal_center_x, inner_wall_y, carcass_height / 2.0)),
            material=walnut_dark,
            name=f"{prefix}_inner_wall",
        )
        desk_frame.visual(
            Box((back_thickness, interior_width, carcass_height)),
            origin=Origin(xyz=(pedestal_back_x, center_y, carcass_height / 2.0)),
            material=walnut_dark,
            name=f"{prefix}_back_panel",
        )
        desk_frame.visual(
            Box((interior_depth, interior_width, floor_thickness)),
            origin=Origin(xyz=(shelf_x_center, center_y, floor_thickness / 2.0)),
            material=walnut_dark,
            name=f"{prefix}_bottom_panel",
        )
        desk_frame.visual(
            Box((interior_depth, interior_width, divider_thickness)),
            origin=Origin(xyz=(shelf_x_center, center_y, 0.321)),
            material=walnut_dark,
            name=f"{prefix}_lower_divider",
        )
        desk_frame.visual(
            Box((interior_depth, interior_width, divider_thickness)),
            origin=Origin(xyz=(shelf_x_center, center_y, 0.519)),
            material=walnut_dark,
            name=f"{prefix}_upper_divider",
        )

        for bay_name, bay_z in drawer_bay_centers.items():
            for side_name, side_sign_local in (("left", -1.0), ("right", 1.0)):
                desk_frame.visual(
                    Box((rail_length, rail_width, rail_height)),
                    origin=Origin(
                        xyz=(
                            rail_x_center,
                            center_y + side_sign_local * rail_offset,
                            bay_z,
                        )
                    ),
                    material=rail_graphite,
                    name=f"{prefix}_{bay_name}_rail_{side_name}",
                )

    add_pedestal_shell("left_pedestal", -0.620)
    add_pedestal_shell("right_pedestal", 0.620)

    desk_frame.inertial = Inertial.from_geometry(
        Box((top_depth, top_width, desk_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, desk_height / 2.0)),
    )

    def add_handle(drawer_part, *, grip_length: float, grip_height: float = 0.0) -> None:
        post_offset = grip_length * 0.36
        drawer_part.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(xyz=(0.009, -post_offset, grip_height), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_nickel,
            name="handle_post_left",
        )
        drawer_part.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(xyz=(0.009, post_offset, grip_height), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_nickel,
            name="handle_post_right",
        )
        drawer_part.visual(
            Cylinder(radius=0.0035, length=grip_length),
            origin=Origin(xyz=(0.018, 0.0, grip_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_nickel,
            name="handle_grip",
        )

    def build_standard_drawer(
        part_name: str,
        *,
        front_height: float,
        box_height: float,
        depth: float,
        grip_length: float,
        mass: float,
    ):
        drawer = model.part(part_name)
        side_length = depth - 0.022 - drawer_back_thickness
        side_x = -0.022 - side_length / 2.0
        back_x = -depth + drawer_back_thickness / 2.0
        runner_length = min(0.460, side_length)
        runner_x = -0.022 - runner_length / 2.0
        runner_thickness = 0.008

        drawer.visual(
            Box((0.022, drawer_front_width, front_height)),
            origin=Origin(xyz=(-0.011, 0.0, 0.0)),
            material=drawer_veneer,
            name="front_panel",
        )
        drawer.visual(
            Box((side_length, drawer_side_thickness, box_height)),
            origin=Origin(
                xyz=(
                    side_x,
                    -(drawer_box_width / 2.0 - drawer_side_thickness / 2.0),
                    0.0,
                )
            ),
            material=walnut,
            name="box_side_left",
        )
        drawer.visual(
            Box((side_length, drawer_side_thickness, box_height)),
            origin=Origin(
                xyz=(
                    side_x,
                    drawer_box_width / 2.0 - drawer_side_thickness / 2.0,
                    0.0,
                )
            ),
            material=walnut,
            name="box_side_right",
        )
        drawer.visual(
            Box((side_length, drawer_box_width - 2.0 * drawer_side_thickness, drawer_bottom_thickness)),
            origin=Origin(
                xyz=(
                    side_x,
                    0.0,
                    -box_height / 2.0 + drawer_bottom_thickness / 2.0,
                )
            ),
            material=walnut,
            name="box_bottom",
        )
        drawer.visual(
            Box((drawer_back_thickness, drawer_box_width, box_height)),
            origin=Origin(xyz=(back_x, 0.0, 0.0)),
            material=walnut,
            name="box_back",
        )
        drawer.visual(
            Box((runner_length, runner_thickness, 0.014)),
            origin=Origin(
                xyz=(
                    runner_x,
                    -(drawer_box_width / 2.0 + runner_thickness / 2.0),
                    0.0,
                )
            ),
            material=rail_graphite,
            name="runner_left",
        )
        drawer.visual(
            Box((runner_length, runner_thickness, 0.014)),
            origin=Origin(
                xyz=(
                    runner_x,
                    drawer_box_width / 2.0 + runner_thickness / 2.0,
                    0.0,
                )
            ),
            material=rail_graphite,
            name="runner_right",
        )
        add_handle(drawer, grip_length=grip_length)
        drawer.inertial = Inertial.from_geometry(
            Box((depth, drawer_front_width, front_height)),
            mass=mass,
            origin=Origin(xyz=(-depth / 2.0, 0.0, 0.0)),
        )
        return drawer

    def build_filing_drawer():
        drawer = model.part("right_filing_drawer")
        depth = 0.560
        front_height = 0.286
        box_height = 0.270
        side_length = depth - 0.022 - drawer_back_thickness
        side_x = -0.022 - side_length / 2.0
        back_x = -depth + drawer_back_thickness / 2.0
        file_rail_width = 0.010
        file_rail_offset = drawer_box_width / 2.0 - drawer_side_thickness - file_rail_width / 2.0
        runner_length = min(0.470, side_length)
        runner_x = -0.022 - runner_length / 2.0
        runner_thickness = 0.008

        drawer.visual(
            Box((0.022, drawer_front_width, front_height)),
            origin=Origin(xyz=(-0.011, 0.0, 0.0)),
            material=drawer_veneer,
            name="front_panel",
        )
        drawer.visual(
            Box((side_length, drawer_side_thickness, box_height)),
            origin=Origin(
                xyz=(
                    side_x,
                    -(drawer_box_width / 2.0 - drawer_side_thickness / 2.0),
                    0.0,
                )
            ),
            material=walnut,
            name="box_side_left",
        )
        drawer.visual(
            Box((side_length, drawer_side_thickness, box_height)),
            origin=Origin(
                xyz=(
                    side_x,
                    drawer_box_width / 2.0 - drawer_side_thickness / 2.0,
                    0.0,
                )
            ),
            material=walnut,
            name="box_side_right",
        )
        drawer.visual(
            Box((side_length, drawer_box_width - 2.0 * drawer_side_thickness, drawer_bottom_thickness)),
            origin=Origin(
                xyz=(
                    side_x,
                    0.0,
                    -box_height / 2.0 + drawer_bottom_thickness / 2.0,
                )
            ),
            material=walnut,
            name="box_bottom",
        )
        drawer.visual(
            Box((drawer_back_thickness, drawer_box_width, box_height)),
            origin=Origin(xyz=(back_x, 0.0, 0.0)),
            material=walnut,
            name="box_back",
        )
        drawer.visual(
            Box((runner_length, runner_thickness, 0.016)),
            origin=Origin(
                xyz=(
                    runner_x,
                    -(drawer_box_width / 2.0 + runner_thickness / 2.0),
                    0.0,
                )
            ),
            material=rail_graphite,
            name="runner_left",
        )
        drawer.visual(
            Box((runner_length, runner_thickness, 0.016)),
            origin=Origin(
                xyz=(
                    runner_x,
                    drawer_box_width / 2.0 + runner_thickness / 2.0,
                    0.0,
                )
            ),
            material=rail_graphite,
            name="runner_right",
        )
        drawer.visual(
            Box((side_length - 0.050, file_rail_width, 0.014)),
            origin=Origin(
                xyz=(
                    side_x + 0.025,
                    -file_rail_offset,
                    box_height / 2.0 - 0.027,
                )
            ),
            material=rail_graphite,
            name="hanging_rail_left",
        )
        drawer.visual(
            Box((side_length - 0.050, file_rail_width, 0.014)),
            origin=Origin(
                xyz=(
                    side_x + 0.025,
                    file_rail_offset,
                    box_height / 2.0 - 0.027,
                )
            ),
            material=rail_graphite,
            name="hanging_rail_right",
        )
        add_handle(drawer, grip_length=0.185)
        drawer.inertial = Inertial.from_geometry(
            Box((depth, drawer_front_width, front_height)),
            mass=12.0,
            origin=Origin(xyz=(-depth / 2.0, 0.0, 0.0)),
        )
        return drawer

    drawer_specs = [
        ("left_top_drawer", -0.620, 0.593, 0.122, 0.096, 0.540, 0.122, 4.2),
        ("left_middle_drawer", -0.620, 0.420, 0.172, 0.146, 0.540, 0.132, 5.1),
        ("left_bottom_drawer", -0.620, 0.167, 0.286, 0.250, 0.560, 0.165, 8.0),
        ("right_top_drawer", 0.620, 0.593, 0.122, 0.096, 0.540, 0.122, 4.2),
        ("right_middle_drawer", 0.620, 0.420, 0.172, 0.146, 0.540, 0.132, 5.1),
    ]

    for part_name, center_y, center_z, front_h, box_h, depth, grip_length, mass in drawer_specs:
        drawer = build_standard_drawer(
            part_name,
            front_height=front_h,
            box_height=box_h,
            depth=depth,
            grip_length=grip_length,
            mass=mass,
        )
        model.articulation(
            f"desk_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=desk_frame,
            child=drawer,
            origin=Origin(xyz=(drawer_front_x, center_y, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=110.0,
                velocity=0.40,
                lower=0.0,
                upper=0.320 if "bottom" not in part_name else 0.340,
            ),
        )

    filing_drawer = build_filing_drawer()
    model.articulation(
        "desk_to_right_filing_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=filing_drawer,
        origin=Origin(xyz=(drawer_front_x, 0.620, 0.167)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.35,
            lower=0.0,
            upper=0.340,
        ),
    )

    file_lock_bar = model.part("file_lock_bar")
    file_lock_bar.visual(
        Cylinder(radius=0.005, length=0.334),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_nickel,
        name="hinge_rod",
    )
    file_lock_bar.visual(
        Box((0.010, 0.334, 0.050)),
        origin=Origin(xyz=(-0.008, 0.0, -0.025)),
        material=satin_nickel,
        name="gate_plate",
    )
    file_lock_bar.inertial = Inertial.from_geometry(
        Box((0.012, 0.334, 0.055)),
        mass=0.8,
        origin=Origin(xyz=(-0.004, 0.0, -0.022)),
    )
    model.articulation(
        "filing_drawer_to_lock_bar",
        ArticulationType.REVOLUTE,
        parent=filing_drawer,
        child=file_lock_bar,
        origin=Origin(xyz=(-0.045, 0.0, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=0.0,
            upper=1.15,
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

    desk_frame = object_model.get_part("desk_frame")
    drawers = {
        "left_top_drawer": object_model.get_part("left_top_drawer"),
        "left_middle_drawer": object_model.get_part("left_middle_drawer"),
        "left_bottom_drawer": object_model.get_part("left_bottom_drawer"),
        "right_top_drawer": object_model.get_part("right_top_drawer"),
        "right_middle_drawer": object_model.get_part("right_middle_drawer"),
        "right_filing_drawer": object_model.get_part("right_filing_drawer"),
    }
    drawer_joints = {
        name: object_model.get_articulation(
            "desk_to_right_filing_drawer" if name == "right_filing_drawer" else f"desk_to_{name}"
        )
        for name in drawers
    }
    file_lock_bar = object_model.get_part("file_lock_bar")
    lock_joint = object_model.get_articulation("filing_drawer_to_lock_bar")

    with ctx.pose():
        for drawer_name, drawer in drawers.items():
            ctx.expect_overlap(
                drawer,
                desk_frame,
                axes="yz",
                min_overlap=0.10,
                name=f"{drawer_name} aligns with its pedestal opening",
            )

    for drawer_name, drawer in drawers.items():
        drawer_joint = drawer_joints[drawer_name]
        upper = drawer_joint.motion_limits.upper
        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: upper}):
            extended_position = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                desk_frame,
                axes="x",
                min_overlap=0.18,
                name=f"{drawer_name} stays retained on its guide rails",
            )
        ctx.check(
            f"{drawer_name} opens forward",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.18,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    with ctx.pose(
        {drawer_joints["right_filing_drawer"]: drawer_joints["right_filing_drawer"].motion_limits.upper}
    ):
        closed_gate = ctx.part_element_world_aabb(file_lock_bar, elem="gate_plate")
    with ctx.pose(
        {
            drawer_joints["right_filing_drawer"]: drawer_joints["right_filing_drawer"].motion_limits.upper,
            lock_joint: lock_joint.motion_limits.upper,
        }
    ):
        ctx.expect_within(
            file_lock_bar,
            drawers["right_filing_drawer"],
            axes="yz",
            margin=0.005,
            name="lock bar stays within the filing drawer width and height",
        )
        open_gate = ctx.part_element_world_aabb(file_lock_bar, elem="gate_plate")

    ctx.check(
        "lock bar swings up into the filing drawer",
        closed_gate is not None
        and open_gate is not None
        and open_gate[1][2] > closed_gate[1][2] + 0.010
        and open_gate[0][0] < closed_gate[0][0] - 0.020,
        details=f"closed={closed_gate}, open={open_gate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
