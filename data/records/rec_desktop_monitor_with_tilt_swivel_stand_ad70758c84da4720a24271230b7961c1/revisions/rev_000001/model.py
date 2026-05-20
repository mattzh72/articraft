from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="height_adjustable_desktop_monitor")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    glass = model.material("dark_display_glass", rgba=(0.03, 0.045, 0.065, 1.0))
    soft_button = model.material("soft_button", rgba=(0.08, 0.085, 0.09, 1.0))
    label_gray = model.material("label_gray", rgba=(0.55, 0.57, 0.58, 1.0))

    # Root: broad, low desktop pedestal at real office-monitor scale.
    pedestal = model.part("pedestal")
    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.30, 0.22, 0.055, corner_segments=10), 0.030),
        "rounded_pedestal",
    )
    pedestal.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_black,
        name="rounded_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_graphite,
        name="bearing_ring",
    )

    swivel_plate = model.part("swivel_plate")
    swivel_plate.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_black,
        name="turntable_disc",
    )
    # Four wall boxes form a visible hollow height-adjustment sleeve, leaving
    # the sliding column in a real clearanced opening rather than a solid proxy.
    sleeve_height = 0.220
    sleeve_z = 0.018 + sleeve_height / 2.0
    swivel_plate.visual(
        Box((0.070, 0.006, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.034, sleeve_z)),
        material=matte_black,
        name="front_sleeve_wall",
    )
    swivel_plate.visual(
        Box((0.070, 0.006, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.034, sleeve_z)),
        material=matte_black,
        name="rear_sleeve_wall",
    )
    swivel_plate.visual(
        Box((0.006, 0.068, sleeve_height)),
        origin=Origin(xyz=(-0.035, 0.0, sleeve_z)),
        material=matte_black,
        name="side_sleeve_wall_0",
    )
    swivel_plate.visual(
        Box((0.006, 0.068, sleeve_height)),
        origin=Origin(xyz=(0.035, 0.0, sleeve_z)),
        material=matte_black,
        name="side_sleeve_wall_1",
    )
    swivel_plate.visual(
        Box((0.082, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_graphite,
        name="lower_sleeve_collar",
    )

    slider_column = model.part("slider_column")
    slider_column.visual(
        Box((0.044, 0.034, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=dark_graphite,
        name="inner_mast",
    )
    # Bearing pads fill the sleeve clearance locally so the telescoping mast
    # reads as supported by its guide sleeve while still leaving most of the
    # column visibly clearanced.
    slider_column.visual(
        Box((0.030, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.024, -0.165)),
        material=dark_graphite,
        name="front_guide_pad",
    )
    slider_column.visual(
        Box((0.030, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.024, -0.165)),
        material=dark_graphite,
        name="rear_guide_pad",
    )
    slider_column.visual(
        Box((0.036, 0.026, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_graphite,
        name="slim_neck",
    )
    # Yoke cheeks support the monitor's trunnion at the top of the neck.
    slider_column.visual(
        Box((0.018, 0.040, 0.050)),
        origin=Origin(xyz=(-0.060, 0.0, 0.173)),
        material=matte_black,
        name="hinge_cheek_0",
    )
    slider_column.visual(
        Box((0.018, 0.040, 0.050)),
        origin=Origin(xyz=(0.060, 0.0, 0.173)),
        material=matte_black,
        name="hinge_cheek_1",
    )
    slider_column.visual(
        Box((0.105, 0.025, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=matte_black,
        name="hinge_bridge",
    )

    display = model.part("display")
    # Child frame is the horizontal tilt axis.  The screen housing is offset
    # forward from that trunnion, with a full-size 24-inch-class shell.
    display.visual(
        Cylinder(radius=0.014, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="tilt_trunnion",
    )
    display.visual(
        Box((0.110, 0.034, 0.070)),
        origin=Origin(xyz=(0.0, -0.026, -0.004)),
        material=matte_black,
        name="rear_mount_plate",
    )
    display.visual(
        Box((0.560, 0.045, 0.340)),
        origin=Origin(xyz=(0.0, -0.065, -0.020)),
        material=satin_black,
        name="display_shell",
    )
    display.visual(
        Box((0.506, 0.004, 0.242)),
        origin=Origin(xyz=(0.0, -0.0895, 0.006)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.245, 0.0045, 0.030)),
        origin=Origin(xyz=(-0.040, -0.08975, -0.155)),
        material=matte_black,
        name="control_recess",
    )
    display.visual(
        Box((0.070, 0.003, 0.008)),
        origin=Origin(xyz=(0.185, -0.0890, -0.154)),
        material=label_gray,
        name="brand_mark",
    )
    display.visual(
        Box((0.040, 0.026, 0.012)),
        origin=Origin(xyz=(0.130, -0.078, -0.196)),
        material=matte_black,
        name="joystick_socket",
    )

    button_xs = (-0.135, -0.095, -0.055, -0.015)
    for idx, x in enumerate(button_xs):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.024, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=soft_button,
            name="button_cap",
        )
        model.articulation(
            f"display_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, -0.0875, -0.155)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=0.04, lower=0.0, upper=0.003),
        )

    joystick_yoke = model.part("joystick_yoke")
    joystick_yoke.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_button,
        name="gimbal_bar",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=soft_button,
        name="short_stem",
    )
    joystick.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=soft_button,
        name="control_nub",
    )

    model.articulation(
        "pedestal_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=swivel_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "swivel_to_slider",
        ArticulationType.PRISMATIC,
        parent=swivel_plate,
        child=slider_column,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.16, lower=0.0, upper=0.100),
    )
    model.articulation(
        "slider_to_display",
        ArticulationType.REVOLUTE,
        parent=slider_column,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.173)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=math.radians(-12.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "display_to_joystick_yoke",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick_yoke,
        origin=Origin(xyz=(0.130, -0.078, -0.202)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=2.0, lower=-0.25, upper=0.25),
    )
    model.articulation(
        "joystick_yoke_to_joystick",
        ArticulationType.REVOLUTE,
        parent=joystick_yoke,
        child=joystick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=2.0, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    swivel = object_model.get_part("swivel_plate")
    slider = object_model.get_part("slider_column")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")

    height_slide = object_model.get_articulation("swivel_to_slider")
    tilt = object_model.get_articulation("slider_to_display")
    yoke_pitch = object_model.get_articulation("display_to_joystick_yoke")
    joystick_roll = object_model.get_articulation("joystick_yoke_to_joystick")

    ctx.expect_contact(
        pedestal,
        swivel,
        elem_a="bearing_ring",
        elem_b="turntable_disc",
        contact_tol=0.001,
        name="swivel turntable is seated on the base bearing",
    )
    ctx.expect_within(
        slider,
        swivel,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.0,
        name="sliding mast fits inside the sleeve width",
    )
    ctx.expect_overlap(
        slider,
        swivel,
        axes="z",
        elem_a="inner_mast",
        elem_b="front_sleeve_wall",
        min_overlap=0.15,
        name="collapsed height column remains deeply inserted",
    )
    with ctx.pose({height_slide: 0.100}):
        ctx.expect_overlap(
            slider,
            swivel,
            axes="z",
            elem_a="inner_mast",
            elem_b="front_sleeve_wall",
            min_overlap=0.08,
            name="raised height column remains retained in the sleeve",
        )

    display_rest = ctx.part_world_position(display)
    with ctx.pose({tilt: math.radians(18.0)}):
        display_tilted = ctx.part_world_position(display)
    ctx.check(
        "tilt hinge uses the horizontal screen axis",
        display_rest is not None
        and display_tilted is not None
        and abs(display_tilted[0] - display_rest[0]) < 0.002,
        details=f"rest={display_rest}, tilted={display_tilted}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    joy_rest = aabb_center(ctx.part_element_world_aabb(joystick, elem="control_nub"))
    with ctx.pose({yoke_pitch: 0.22, joystick_roll: -0.22}):
        joy_deflected = aabb_center(ctx.part_element_world_aabb(joystick, elem="control_nub"))
    ctx.check(
        "joystick has a short two-axis local pivot",
        joy_rest is not None
        and joy_deflected is not None
        and abs(joy_deflected[0] - joy_rest[0]) > 0.002
        and abs(joy_deflected[1] - joy_rest[1]) > 0.002,
        details=f"rest={joy_rest}, deflected={joy_deflected}",
    )

    return ctx.report()


object_model = build_object_model()
