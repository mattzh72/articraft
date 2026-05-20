from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _curved_slab_mesh(
    *,
    width: float,
    height: float,
    front_center_y: float,
    depth: float,
    sagitta: float,
    curve_half_width: float,
    segments: int = 36,
) -> MeshGeometry:
    """Closed wide monitor slab curved across X, with thickness along local +Y."""

    geom = MeshGeometry()
    half = width * 0.5
    z_min = -height * 0.5
    z_max = height * 0.5

    front_bottom: list[int] = []
    front_top: list[int] = []
    rear_bottom: list[int] = []
    rear_top: list[int] = []

    for i in range(segments + 1):
        x = -half + width * i / segments
        y_front = front_center_y - sagitta * (x / curve_half_width) ** 2
        y_rear = y_front + depth
        front_bottom.append(geom.add_vertex(x, y_front, z_min))
        front_top.append(geom.add_vertex(x, y_front, z_max))
        rear_bottom.append(geom.add_vertex(x, y_rear, z_min))
        rear_top.append(geom.add_vertex(x, y_rear, z_max))

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        # Curved front and rear skins.
        quad(front_bottom[i], front_bottom[i + 1], front_top[i + 1], front_top[i])
        quad(rear_bottom[i + 1], rear_bottom[i], rear_top[i], rear_top[i + 1])
        # Top and bottom edges close the shell.
        quad(front_top[i], front_top[i + 1], rear_top[i + 1], rear_top[i])
        quad(front_bottom[i + 1], front_bottom[i], rear_bottom[i], rear_bottom[i + 1])

    # Side caps.
    quad(front_bottom[0], front_top[0], rear_top[0], rear_bottom[0])
    quad(front_bottom[-1], rear_bottom[-1], rear_top[-1], front_top[-1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    plastic = model.material("matte_black_plastic", rgba=(0.025, 0.026, 0.030, 1.0))
    shell_mat = model.material("dark_graphite_shell", rgba=(0.075, 0.078, 0.085, 1.0))
    screen_mat = model.material("glossy_near_black_screen", rgba=(0.005, 0.009, 0.014, 1.0))
    metal = model.material("satin_black_metal", rgba=(0.015, 0.016, 0.018, 1.0))
    red = model.material("red_power_button", rgba=(0.75, 0.025, 0.020, 1.0))

    # Root: large desktop tripod foot, sized for an ultrawide gaming monitor.
    base = model.part("tripod_base")
    base.visual(
        Cylinder(radius=0.082, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=metal,
        name="swivel_hub",
    )
    leg_specs = (
        ("front_leg", -math.pi / 2.0, (0.0, -0.260, 0.020), (0.560, 0.072, 0.038)),
        ("rear_leg_0", math.radians(28.0), (0.215, 0.150, 0.020), (0.440, 0.066, 0.034)),
        ("rear_leg_1", math.radians(152.0), (-0.215, 0.150, 0.020), (0.440, 0.066, 0.034)),
    )
    for name, yaw, xyz, size in leg_specs:
        base.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=metal,
            name=name,
        )

    # Swiveling stand and split yoke.  It is one connected metal assembly,
    # leaving an open center behind the display.
    stand = model.part("stand_yoke")
    stand.visual(
        Cylinder(radius=0.038, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=metal,
        name="vertical_post",
    )
    stand.visual(
        Box((0.160, 0.070, 0.110)),
        origin=Origin(xyz=(0.0, 0.000, 0.760)),
        material=metal,
        name="neck_block",
    )
    stand.visual(
        Box((0.690, 0.060, 0.046)),
        origin=Origin(xyz=(0.0, 0.000, 0.805)),
        material=metal,
        name="lower_bridge",
    )
    for x, name in ((-0.360, "yoke_arm_0"), (0.360, "yoke_arm_1")):
        stand.visual(
            Box((0.058, 0.060, 0.225)),
            origin=Origin(xyz=(x, 0.0, 0.910)),
            material=metal,
            name=name,
        )
        stand.visual(
            Cylinder(radius=0.038, length=0.072),
            origin=Origin(xyz=(x, 0.0, 0.960), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"pivot_bushing_{0 if x < 0 else 1}",
        )

    # Tilted child assembly: curved rear shell and hinge boss.  The hinge line is
    # just behind the rear shell so the yoke arms carry the screen at its sides.
    display = model.part("display_shell")
    shell_mesh = _curved_slab_mesh(
        width=1.240,
        height=0.420,
        front_center_y=-0.105,
        depth=0.060,
        sagitta=0.055,
        curve_half_width=0.620,
        segments=40,
    )
    display.visual(
        mesh_from_geometry(shell_mesh, "curved_rear_shell"),
        material=shell_mat,
        name="curved_rear_shell",
    )
    display.visual(
        Box((0.170, 0.052, 0.115)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=shell_mat,
        name="central_hinge_boss",
    )
    display.visual(
        Cylinder(radius=0.014, length=0.648),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_axle",
    )
    display.visual(
        Box((0.285, 0.055, 0.016)),
        origin=Origin(xyz=(0.0, -0.105, -0.218)),
        material=plastic,
        name="control_pocket",
    )

    screen_mesh = _curved_slab_mesh(
        width=1.150,
        height=0.340,
        front_center_y=-0.108,
        depth=0.003,
        sagitta=0.055,
        curve_half_width=0.620,
        segments=40,
    )
    display.visual(
        mesh_from_geometry(screen_mesh, "curved_screen_glass"),
        material=screen_mat,
        name="curved_screen_glass",
    )

    # Underside cluster: each visible cap is an independently sliding button.
    button_positions = (-0.096, -0.048, 0.0, 0.048, 0.096)
    buttons = []
    for idx, x in enumerate(button_positions):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.034, 0.022, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=red if idx == 2 else plastic,
            name="button_cap",
        )
        buttons.append(button)

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.28, upper=0.32),
    )
    for idx, (x, button) in enumerate(zip(button_positions, buttons)):
        model.articulation(
            f"button_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, -0.105, -0.226)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=0.006),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    display = object_model.get_part("display_shell")
    stand = object_model.get_part("stand_yoke")
    base = object_model.get_part("tripod_base")
    tilt = object_model.get_articulation("screen_tilt")
    swivel = object_model.get_articulation("base_swivel")

    ctx.check("swivel is continuous", swivel.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("tilt is revolute", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check(
        "tilt axis is horizontal",
        tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    display_aabb = ctx.part_world_aabb(display)
    if display_aabb is not None:
        lower, upper = display_aabb
        width = upper[0] - lower[0]
        height = upper[2] - lower[2]
        ctx.check(
            "ultrawide large desktop scale",
            width > 1.15 and height > 0.38,
            details=f"width={width:.3f}, height={height:.3f}",
        )

    ctx.expect_within(
        display,
        display,
        axes="xz",
        margin=0.0,
        inner_elem="curved_screen_glass",
        outer_elem="curved_rear_shell",
        name="screen glass is inset inside curved shell face",
    )

    ctx.expect_gap(
        stand,
        display,
        axis="y",
        min_gap=0.002,
        max_gap=0.080,
        positive_elem="pivot_bushing_0",
        negative_elem="curved_rear_shell",
        name="left yoke bushing clears rear shell",
    )

    button_joints = [object_model.get_articulation(f"button_{idx}_slide") for idx in range(5)]
    ctx.check(
        "all underside controls are independent prismatic buttons",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
    )
    for idx in range(5):
        ctx.expect_contact(
            object_model.get_part(f"button_{idx}"),
            display,
            elem_a="button_cap",
            elem_b="control_pocket",
            contact_tol=0.001,
            name=f"button_{idx} seated against underside pocket",
        )

    button_2 = object_model.get_part("button_2")
    button_1 = object_model.get_part("button_1")
    rest_2 = ctx.part_world_position(button_2)
    rest_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_joints[2]: 0.006}):
        pressed_2 = ctx.part_world_position(button_2)
        pressed_1 = ctx.part_world_position(button_1)
    ctx.check(
        "center button presses independently upward",
        rest_2 is not None
        and pressed_2 is not None
        and rest_1 is not None
        and pressed_1 is not None
        and pressed_2[2] > rest_2[2] + 0.005
        and abs(pressed_1[2] - rest_1[2]) < 0.0005,
        details=f"button_2 rest={rest_2}, pressed={pressed_2}; button_1 rest={rest_1}, pressed={pressed_1}",
    )

    with ctx.pose({tilt: 0.25}):
        tilted_aabb = ctx.part_world_aabb(display)
    ctx.check("tilt pose remains evaluable", tilted_aabb is not None)

    return ctx.report()


object_model = build_object_model()
