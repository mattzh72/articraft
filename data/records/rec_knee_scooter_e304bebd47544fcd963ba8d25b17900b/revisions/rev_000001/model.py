from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _rotated_x_rod(part, name, start, end, thickness, material):
    """Add a rectangular tube whose local +X runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    if abs(dy) > 1e-9:
        raise ValueError("_rotated_x_rod only handles rods in an XZ plane")
    length = math.sqrt(dx * dx + dz * dz)
    pitch = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    blue = Material("powder_coated_blue", rgba=(0.02, 0.14, 0.42, 1.0))
    black = Material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = Material("soft_black_rubber", rgba=(0.01, 0.009, 0.008, 1.0))
    metal = Material("brushed_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_metal = Material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    basket_wire = Material("coated_basket_wire", rgba=(0.90, 0.92, 0.90, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.055,
            0.036,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(radius=0.020, width=0.028, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "small_scooter_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.044,
            inner_radius=0.055,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.04),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "small_scooter_tire",
    )

    frame = model.part("frame")
    frame.visual(Box((0.83, 0.035, 0.035)), origin=Origin(xyz=(-0.13, 0.15, 0.18)), material=blue, name="side_rail_0")
    frame.visual(Box((0.83, 0.035, 0.035)), origin=Origin(xyz=(-0.13, -0.15, 0.18)), material=blue, name="side_rail_1")
    frame.visual(Box((0.050, 0.34, 0.035)), origin=Origin(xyz=(-0.50, 0.0, 0.18)), material=blue, name="rear_crossbar")
    frame.visual(Box((0.060, 0.34, 0.035)), origin=Origin(xyz=(0.25, 0.0, 0.18)), material=blue, name="front_crossbar")
    frame.visual(Box((0.46, 0.130, 0.026)), origin=Origin(xyz=(-0.19, 0.095, 0.206)), material=black, name="platform_0")
    frame.visual(Box((0.46, 0.130, 0.026)), origin=Origin(xyz=(-0.19, -0.095, 0.206)), material=black, name="platform_1")
    frame.visual(Box((0.13, 0.042, 0.030)), origin=Origin(xyz=(-0.12, 0.046, 0.222)), material=dark_metal, name="post_socket_0")
    frame.visual(Box((0.13, 0.042, 0.030)), origin=Origin(xyz=(-0.12, -0.046, 0.222)), material=dark_metal, name="post_socket_1")

    # Fixed outer sleeve for the telescoping knee-pad post.
    frame.visual(Cylinder(radius=0.027, length=0.230), origin=Origin(xyz=(-0.12, 0.0, 0.285)), material=metal, name="outer_sleeve")

    # Rear axle carrier and dropouts.
    frame.visual(Box((0.060, 0.410, 0.025)), origin=Origin(xyz=(-0.52, 0.0, 0.080)), material=blue, name="rear_axle_beam")
    frame.visual(Box((0.055, 0.028, 0.120)), origin=Origin(xyz=(-0.50, 0.18, 0.125)), material=blue, name="rear_stay_0")
    frame.visual(Box((0.055, 0.028, 0.120)), origin=Origin(xyz=(-0.50, -0.18, 0.125)), material=blue, name="rear_stay_1")
    frame.visual(Box((0.060, 0.025, 0.085)), origin=Origin(xyz=(-0.52, 0.210, 0.075)), material=dark_metal, name="rear_dropout_0")
    frame.visual(Box((0.060, 0.025, 0.085)), origin=Origin(xyz=(-0.52, -0.210, 0.075)), material=dark_metal, name="rear_dropout_1")
    frame.visual(Box((0.026, 0.008, 0.026)), origin=Origin(xyz=(-0.52, 0.2265, 0.075)), material=dark_metal, name="rear_axle_stub_0")
    frame.visual(Box((0.026, 0.008, 0.026)), origin=Origin(xyz=(-0.52, -0.2265, 0.075)), material=dark_metal, name="rear_axle_stub_1")

    # Steering head yoke leaves an open bore for the rotating column.
    _rotated_x_rod(frame, "front_neck", (0.25, 0.0, 0.18), (0.385, 0.0, 0.25), 0.035, blue)
    frame.visual(Box((0.043, 0.096, 0.026)), origin=Origin(xyz=(0.3865, 0.0, 0.250)), material=blue, name="steering_bridge")
    frame.visual(Box((0.050, 0.014, 0.180)), origin=Origin(xyz=(0.430, 0.029, 0.250)), material=blue, name="steering_yoke_0")
    frame.visual(Box((0.050, 0.014, 0.180)), origin=Origin(xyz=(0.430, -0.029, 0.250)), material=blue, name="steering_yoke_1")

    front_fork = model.part("front_fork")
    front_fork.visual(Cylinder(radius=0.022, length=0.950), origin=Origin(xyz=(0.0, 0.0, 0.275)), material=metal, name="steering_column")
    front_fork.visual(Box((0.052, 0.410, 0.030)), origin=Origin(xyz=(0.0, 0.0, -0.175)), material=blue, name="front_axle_beam")
    front_fork.visual(Box((0.070, 0.025, 0.085)), origin=Origin(xyz=(0.0, 0.205, -0.175)), material=dark_metal, name="front_dropout_0")
    front_fork.visual(Box((0.070, 0.025, 0.085)), origin=Origin(xyz=(0.0, -0.205, -0.175)), material=dark_metal, name="front_dropout_1")
    front_fork.visual(Box((0.065, 0.560, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.720)), material=blue, name="handlebar")
    front_fork.visual(Box((0.080, 0.110, 0.045)), origin=Origin(xyz=(0.0, 0.315, 0.720)), material=rubber, name="grip_0")
    front_fork.visual(Box((0.080, 0.110, 0.045)), origin=Origin(xyz=(0.0, -0.315, 0.720)), material=rubber, name="grip_1")
    front_fork.visual(Box((0.012, 0.160, 0.044)), origin=Origin(xyz=(0.027, 0.0, 0.420)), material=dark_metal, name="basket_hinge_clamp")

    model.articulation(
        "frame_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.430, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=-0.90, upper=0.90),
    )

    knee_post = model.part("knee_post")
    knee_post.visual(Cylinder(radius=0.017, length=0.420), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=metal, name="inner_post")
    knee_post.visual(Box((0.200, 0.125, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.215)), material=dark_metal, name="pad_mount_plate")
    knee_post.visual(Box((0.380, 0.195, 0.060)), origin=Origin(xyz=(-0.015, 0.0, 0.255)), material=black, name="knee_pad")
    knee_post.visual(Box((0.300, 0.012, 0.004)), origin=Origin(xyz=(-0.020, 0.050, 0.287)), material=dark_metal, name="pad_groove_0")
    knee_post.visual(Box((0.300, 0.012, 0.004)), origin=Origin(xyz=(-0.020, -0.050, 0.287)), material=dark_metal, name="pad_groove_1")

    model.articulation(
        "frame_to_knee_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=knee_post,
        origin=Origin(xyz=(-0.120, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.14),
    )

    basket_support = model.part("basket_support")
    basket_support.visual(Cylinder(radius=0.014, length=0.180), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="hinge_barrel")
    basket_support.visual(Box((0.205, 0.016, 0.016)), origin=Origin(xyz=(0.1025, 0.070, 0.0)), material=dark_metal, name="support_arm_0")
    basket_support.visual(Box((0.205, 0.016, 0.016)), origin=Origin(xyz=(0.1025, -0.070, 0.0)), material=dark_metal, name="support_arm_1")
    basket_support.visual(Box((0.018, 0.270, 0.016)), origin=Origin(xyz=(0.198, 0.0, 0.0)), material=dark_metal, name="support_crossbar")

    # Open wire basket: connected bottom rim, top rim, and vertical basket rods.
    rear_x, front_x = 0.180, 0.420
    half_w = 0.170
    top_z, mid_z, bot_z = 0.180, 0.090, 0.0
    rod = 0.012
    for z, label in ((bot_z, "bottom"), (top_z, "top")):
        basket_support.visual(Box((rod, 0.352, rod)), origin=Origin(xyz=(rear_x, 0.0, z)), material=basket_wire, name=f"{label}_rear_rail")
        basket_support.visual(Box((rod, 0.352, rod)), origin=Origin(xyz=(front_x, 0.0, z)), material=basket_wire, name=f"{label}_front_rail")
        basket_support.visual(Box((front_x - rear_x, rod, rod)), origin=Origin(xyz=((front_x + rear_x) * 0.5, half_w, z)), material=basket_wire, name=f"{label}_side_rail_0")
        basket_support.visual(Box((front_x - rear_x, rod, rod)), origin=Origin(xyz=((front_x + rear_x) * 0.5, -half_w, z)), material=basket_wire, name=f"{label}_side_rail_1")

    for x in (rear_x, front_x):
        for y in (-half_w, half_w):
            basket_support.visual(Box((rod, rod, top_z + 0.010)), origin=Origin(xyz=(x, y, mid_z)), material=basket_wire, name=f"corner_rod_{x:.2f}_{y:.2f}")
    for x in (0.260, 0.340):
        basket_support.visual(Box((rod, rod, top_z + 0.010)), origin=Origin(xyz=(x, half_w, mid_z)), material=basket_wire, name=f"side_bar_0_{x:.2f}")
        basket_support.visual(Box((rod, rod, top_z + 0.010)), origin=Origin(xyz=(x, -half_w, mid_z)), material=basket_wire, name=f"side_bar_1_{x:.2f}")
    for y in (-0.085, 0.0, 0.085):
        basket_support.visual(Box((rod, rod, top_z + 0.010)), origin=Origin(xyz=(front_x, y, mid_z)), material=basket_wire, name=f"front_bar_{y:.2f}")
    for y in (-0.085, 0.0, 0.085):
        basket_support.visual(Box((front_x - rear_x, 0.008, 0.008)), origin=Origin(xyz=((front_x + rear_x) * 0.5, y, bot_z)), material=basket_wire, name=f"bottom_grid_x_{y:.2f}")
    for x in (0.260, 0.340):
        basket_support.visual(Box((0.008, 0.330, 0.008)), origin=Origin(xyz=(x, 0.0, bot_z)), material=basket_wire, name=f"bottom_grid_y_{x:.2f}")

    model.articulation(
        "front_fork_to_basket_support",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=basket_support,
        origin=Origin(xyz=(0.047, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    def add_wheel(name: str, parent, joint_name: str, xyz):
        wheel = model.part(name)
        visual_yaw = math.pi / 2.0 if xyz[1] >= 0.0 else -math.pi / 2.0
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, visual_yaw)), material=rubber, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(rpy=(0.0, 0.0, visual_yaw)), material=metal, name="rim")
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=30.0),
        )
        return wheel

    add_wheel("front_wheel_0", front_fork, "front_fork_to_front_wheel_0", (0.0, 0.2395, -0.175))
    add_wheel("front_wheel_1", front_fork, "front_fork_to_front_wheel_1", (0.0, -0.2395, -0.175))
    add_wheel("rear_wheel_0", frame, "frame_to_rear_wheel_0", (-0.520, 0.2445, 0.075))
    add_wheel("rear_wheel_1", frame, "frame_to_rear_wheel_1", (-0.520, -0.2445, 0.075))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_fork = object_model.get_part("front_fork")
    knee_post = object_model.get_part("knee_post")
    basket_support = object_model.get_part("basket_support")
    steering = object_model.get_articulation("frame_to_front_fork")
    post_slide = object_model.get_articulation("frame_to_knee_post")
    basket_hinge = object_model.get_articulation("front_fork_to_basket_support")

    ctx.allow_overlap(
        frame,
        knee_post,
        elem_a="outer_sleeve",
        elem_b="inner_post",
        reason="The telescoping knee-pad post is intentionally represented as a sliding member nested inside the fixed sleeve.",
    )
    ctx.expect_within(
        knee_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="telescoping post stays centered in sleeve",
    )
    ctx.expect_overlap(
        knee_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="lower knee post has retained insertion",
    )

    with ctx.pose({post_slide: 0.14}):
        ctx.expect_overlap(
            knee_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.055,
            name="raised knee post remains inserted",
        )

    ctx.expect_gap(
        object_model.get_part("front_wheel_0"),
        front_fork,
        axis="y",
        positive_elem="tire",
        negative_elem="front_dropout_0",
        min_gap=0.0,
        max_gap=0.010,
        name="front wheel has fork clearance",
    )
    ctx.expect_gap(
        front_fork,
        object_model.get_part("front_wheel_1"),
        axis="y",
        positive_elem="front_dropout_1",
        negative_elem="tire",
        min_gap=0.0,
        max_gap=0.010,
        name="opposite front wheel has fork clearance",
    )

    front_wheel = object_model.get_part("front_wheel_0")
    rest_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.55}):
        steered_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork steers about vertical axis",
        rest_pos is not None
        and steered_pos is not None
        and abs(steered_pos[0] - rest_pos[0]) > 0.025
        and abs(steered_pos[1] - rest_pos[1]) > 0.015,
        details=f"rest={rest_pos}, steered={steered_pos}",
    )

    def _aabb_center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    rest_basket = ctx.part_world_aabb(basket_support)
    with ctx.pose({basket_hinge: 1.05}):
        raised_basket = ctx.part_world_aabb(basket_support)
    rest_z = _aabb_center_z(rest_basket)
    raised_z = _aabb_center_z(raised_basket)
    ctx.check(
        "basket support folds upward",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.070,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    wheel_joint_names = {
        "front_fork_to_front_wheel_0",
        "front_fork_to_front_wheel_1",
        "frame_to_rear_wheel_0",
        "frame_to_rear_wheel_1",
    }
    ctx.check(
        "four continuous wheel spin joints",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS for name in wheel_joint_names),
        details=f"wheel_joints={sorted(wheel_joint_names)}",
    )

    return ctx.report()


object_model = build_object_model()
