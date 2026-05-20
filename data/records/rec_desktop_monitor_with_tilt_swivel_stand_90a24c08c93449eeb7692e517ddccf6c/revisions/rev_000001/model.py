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
    tube_from_spline_points,
)


def _curved_panel_slab(
    width: float,
    height: float,
    thickness: float,
    *,
    front_center_y: float,
    edge_wrap: float,
    x_segments: int = 28,
) -> MeshGeometry:
    """Closed curved monitor slab: wide in X, vertical in Z, curved in Y."""
    geom = MeshGeometry()
    rows: list[tuple[int, int, int, int]] = []
    half_w = width / 2.0
    half_h = height / 2.0

    for i in range(x_segments + 1):
        u = -1.0 + 2.0 * i / x_segments
        x = half_w * u
        # Edges are pulled toward the viewer (-Y), leaving the center farther back.
        y_front = front_center_y - edge_wrap * (u * u)
        y_back = y_front + thickness
        rows.append(
            (
                geom.add_vertex(x, y_front, -half_h),
                geom.add_vertex(x, y_front, half_h),
                geom.add_vertex(x, y_back, -half_h),
                geom.add_vertex(x, y_back, half_h),
            )
        )

    for i in range(x_segments):
        f0b, f0t, b0b, b0t = rows[i]
        f1b, f1t, b1b, b1t = rows[i + 1]
        # Front display plane.
        geom.add_face(f0b, f1b, f1t)
        geom.add_face(f0b, f1t, f0t)
        # Rear shell plane.
        geom.add_face(b0b, b0t, b1t)
        geom.add_face(b0b, b1t, b1b)
        # Bottom and top rims.
        geom.add_face(f0b, b0b, b1b)
        geom.add_face(f0b, b1b, f1b)
        geom.add_face(f0t, f1t, b1t)
        geom.add_face(f0t, b1t, b0t)

    # End caps.
    for idx in (0, -1):
        fb, ft, bb, bt = rows[idx]
        if idx == 0:
            geom.add_face(fb, ft, bt)
            geom.add_face(fb, bt, bb)
        else:
            geom.add_face(fb, bb, bt)
            geom.add_face(fb, bt, ft)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    glass = model.material("smoked_glass", rgba=(0.015, 0.030, 0.055, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    metal = model.material("dark_gunmetal", rgba=(0.10, 0.105, 0.115, 1.0))
    blue = model.material("cool_blue_accent", rgba=(0.05, 0.22, 0.85, 1.0))
    button_mat = model.material("satin_button_plastic", rgba=(0.055, 0.057, 0.062, 1.0))
    icon_white = model.material("dim_white_icons", rgba=(0.80, 0.84, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=charcoal,
        name="central_hub",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=metal,
        name="swivel_socket",
    )
    base.visual(
        Box((0.135, 0.460, 0.032)),
        origin=Origin(xyz=(0.0, -0.210, 0.016)),
        material=matte_black,
        name="front_foot",
    )
    base.visual(
        Box((0.110, 0.420, 0.030)),
        origin=Origin(xyz=(-0.130, 0.125, 0.015), rpy=(0.0, 0.0, 0.92)),
        material=matte_black,
        name="foot_0",
    )
    base.visual(
        Box((0.110, 0.420, 0.030)),
        origin=Origin(xyz=(0.130, 0.125, 0.015), rpy=(0.0, 0.0, -0.92)),
        material=matte_black,
        name="foot_1",
    )
    for name, x, y, yaw, sx, sy in (
        ("front_pad", 0.0, -0.390, 0.0, 0.105, 0.060),
        ("pad_0", -0.270, 0.250, 0.92, 0.085, 0.052),
        ("pad_1", 0.270, 0.250, -0.92, 0.085, 0.052),
    ):
        base.visual(
            Box((sx, sy, 0.006)),
            origin=Origin(xyz=(x, y, 0.003), rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=name,
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=metal,
        name="swivel_collar",
    )
    yoke.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.010), (0.0, 0.0, 0.110), (0.0, 0.0, 0.205)],
                radius=0.020,
                samples_per_segment=10,
                radial_segments=18,
            ),
            "yoke_upright_neck",
        ),
        material=metal,
        name="upright_neck",
    )
    yoke.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.190), (-0.090, 0.050, 0.255), (-0.275, 0.050, 0.360)],
                radius=0.016,
                samples_per_segment=14,
                radial_segments=18,
            ),
            "yoke_arm_0",
        ),
        material=metal,
        name="arm_0",
    )
    yoke.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.190), (0.090, 0.050, 0.255), (0.275, 0.050, 0.360)],
                radius=0.016,
                samples_per_segment=14,
                radial_segments=18,
            ),
            "yoke_arm_1",
        ),
        material=metal,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(-0.250, 0.0, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="bearing_0",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.250, 0.0, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="bearing_1",
    )
    yoke.visual(
        Box((0.125, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        material=metal,
        name="split_bridge",
    )

    display = model.part("display")
    display.visual(
        mesh_from_geometry(
            _curved_panel_slab(
                0.920,
                0.405,
                0.034,
                front_center_y=-0.086,
                edge_wrap=0.047,
            ),
            "curved_outer_shell",
        ),
        material=matte_black,
        name="shell",
    )
    display.visual(
        mesh_from_geometry(
            _curved_panel_slab(
                0.820,
                0.310,
                0.006,
                front_center_y=-0.090,
                edge_wrap=0.041,
            ),
            "curved_display_glass",
        ),
        origin=Origin(xyz=(0.0, -0.002, 0.020)),
        material=glass,
        name="glass_panel",
    )
    display.visual(
        mesh_from_geometry(
            _curved_panel_slab(
                0.340,
                0.190,
                0.035,
                front_center_y=-0.060,
                edge_wrap=0.011,
                x_segments=16,
            ),
            "raised_rear_housing",
        ),
        material=charcoal,
        name="rear_housing",
    )
    display.visual(
        Box((0.200, 0.066, 0.155)),
        origin=Origin(xyz=(0.0, -0.025, 0.000)),
        material=charcoal,
        name="rear_mount_plate",
    )
    display.visual(
        Cylinder(radius=0.023, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_barrel",
    )
    display.visual(
        Box((0.140, 0.004, 0.007)),
        origin=Origin(xyz=(0.0, -0.088, -0.176)),
        material=blue,
        name="lower_light_strip",
    )
    for name, x, z in (
        ("screw_0", -0.058, -0.048),
        ("screw_1", 0.058, -0.048),
        ("screw_2", -0.058, 0.048),
        ("screw_3", 0.058, 0.048),
    ):
        display.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, 0.010, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=name,
        )

    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0),
    )
    model.articulation(
        "yoke_to_display",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.28, upper=0.34),
    )

    # Five independent underside menu buttons, placed under the lower bezel.
    for i, x in enumerate((0.225, 0.260, 0.295, 0.330, 0.365)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.024, 0.018, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, -0.0045)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.0025, 0.0015)),
            origin=Origin(xyz=(0.0, -0.0095, -0.006)),
            material=icon_white,
            name="icon_tick",
        )
        model.articulation(
            f"display_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, -0.095, -0.2019)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    display = object_model.get_part("display")
    swivel = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_display")

    ctx.allow_overlap(
        display,
        yoke,
        elem_a="tilt_barrel",
        elem_b="bearing_0",
        reason="The display tilt barrel is intentionally captured inside the left yoke bearing.",
    )
    ctx.allow_overlap(
        display,
        yoke,
        elem_a="tilt_barrel",
        elem_b="bearing_1",
        reason="The display tilt barrel is intentionally captured inside the right yoke bearing.",
    )

    ctx.check(
        "stand swivels continuously about vertical axis",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "screen tilts on horizontal yoke joint",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        positive_elem="swivel_collar",
        negative_elem="swivel_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel collar seats on base socket",
    )
    for bearing in ("bearing_0", "bearing_1"):
        ctx.expect_overlap(
            display,
            yoke,
            axes="x",
            elem_a="tilt_barrel",
            elem_b=bearing,
            min_overlap=0.020,
            name=f"{bearing} captures barrel along hinge",
        )
        ctx.expect_overlap(
            display,
            yoke,
            axes="yz",
            elem_a="tilt_barrel",
            elem_b=bearing,
            min_overlap=0.020,
            name=f"{bearing} surrounds barrel cross section",
        )

    shell_aabb = ctx.part_element_world_aabb(display, elem="shell")
    if shell_aabb is not None:
        shell_min, shell_max = shell_aabb
        ctx.check(
            "display is gaming monitor scale and wide",
            shell_max[0] - shell_min[0] > 0.85 and shell_max[2] - shell_min[2] > 0.38,
            details=f"shell_min={shell_min}, shell_max={shell_max}",
        )
    else:
        ctx.fail("display shell aabb available", "shell visual was not measurable")

    rest_shell = ctx.part_element_world_aabb(display, elem="shell")
    with ctx.pose({tilt: 0.25}):
        tilted_shell = ctx.part_element_world_aabb(display, elem="shell")
    ctx.check(
        "tilt joint visibly changes screen pitch",
        rest_shell is not None
        and tilted_shell is not None
        and abs(tilted_shell[0][1] - rest_shell[0][1]) > 0.030,
        details=f"rest={rest_shell}, tilted={tilted_shell}",
    )

    rest_swivel_shell = ctx.part_element_world_aabb(display, elem="shell")
    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_shell = ctx.part_element_world_aabb(display, elem="shell")
    ctx.check(
        "continuous swivel rotates the screen footprint",
        rest_swivel_shell is not None
        and swiveled_shell is not None
        and abs((swiveled_shell[0][0] + swiveled_shell[1][0]) - (rest_swivel_shell[0][0] + rest_swivel_shell[1][0])) > 0.060,
        details=f"rest={rest_swivel_shell}, swiveled={swiveled_shell}",
    )

    for i in range(5):
        button = object_model.get_part(f"button_{i}")
        button_joint = object_model.get_articulation(f"display_to_button_{i}")
        ctx.allow_overlap(
            display,
            button,
            elem_a="shell",
            elem_b="button_cap",
            reason="The push button cap has a tiny seated compression into the lower bezel at rest.",
        )
        ctx.expect_gap(
            display,
            button,
            axis="z",
            positive_elem="shell",
            negative_elem="button_cap",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"button_{i} is seated under lower bezel",
        )
        ctx.expect_overlap(
            button,
            display,
            axes="xy",
            elem_a="button_cap",
            elem_b="shell",
            min_overlap=0.010,
            name=f"button_{i} sits below the bezel footprint",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.006}):
            pushed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} pushes upward independently",
            rest_pos is not None and pushed_pos is not None and pushed_pos[2] > rest_pos[2] + 0.005,
            details=f"rest={rest_pos}, pushed={pushed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
