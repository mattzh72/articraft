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
    Sphere,
    TestContext,
    TestReport,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_monitor")

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.85, 0.86, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="base_hub",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_gray,
        name="swivel_plinth",
    )

    def add_leg(name: str, points: list[tuple[float, float, float]], radius: float, foot_radius: float) -> None:
        leg_geom = tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_geom, name),
            material=graphite,
            name=name,
        )
        tip = points[-1]
        base.visual(
            Sphere(radius=foot_radius),
            origin=Origin(xyz=tip),
            material=matte_black,
            name=f"{name}_foot",
        )

    add_leg(
        "left_leg",
        [(0.0, 0.010, 0.018), (-0.150, 0.140, 0.013), (-0.245, 0.215, 0.010)],
        radius=0.013,
        foot_radius=0.013,
    )
    add_leg(
        "right_leg",
        [(0.0, 0.010, 0.018), (0.150, 0.140, 0.013), (0.245, 0.215, 0.010)],
        radius=0.013,
        foot_radius=0.013,
    )
    add_leg(
        "rear_spur",
        [(0.0, -0.006, 0.018), (0.0, -0.090, 0.015), (0.0, -0.155, 0.010)],
        radius=0.012,
        foot_radius=0.012,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.52, 0.40, 0.065)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.035, 0.024)),
    )

    swivel_stage = model.part("swivel_stage")
    swivel_stage.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_gray,
        name="rotating_collar",
    )
    swivel_stage.visual(
        Box((0.082, 0.006, 0.170)),
        origin=Origin(xyz=(0.0, -0.027, 0.109)),
        material=dark_gray,
        name="sleeve_front_wall",
    )
    swivel_stage.visual(
        Box((0.082, 0.006, 0.170)),
        origin=Origin(xyz=(0.0, 0.027, 0.109)),
        material=dark_gray,
        name="sleeve_rear_wall",
    )
    swivel_stage.visual(
        Box((0.006, 0.048, 0.170)),
        origin=Origin(xyz=(-0.038, 0.0, 0.109)),
        material=dark_gray,
        name="sleeve_left_wall",
    )
    swivel_stage.visual(
        Box((0.006, 0.048, 0.170)),
        origin=Origin(xyz=(0.038, 0.0, 0.109)),
        material=dark_gray,
        name="outer_sleeve",
    )
    swivel_stage.visual(
        Box((0.086, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.029, 0.201)),
        material=graphite,
        name="cap_front_lip",
    )
    swivel_stage.visual(
        Box((0.086, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.029, 0.201)),
        material=graphite,
        name="cap_rear_lip",
    )
    swivel_stage.visual(
        Box((0.006, 0.052, 0.014)),
        origin=Origin(xyz=(-0.040, 0.0, 0.201)),
        material=graphite,
        name="cap_left_lip",
    )
    swivel_stage.visual(
        Box((0.006, 0.052, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.201)),
        material=graphite,
        name="sleeve_cap",
    )
    swivel_stage.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.215)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swivel_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.044, 0.022, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=matte_black,
        name="mast_shaft",
    )
    mast.visual(
        Box((0.048, 0.006, 0.160)),
        origin=Origin(xyz=(0.0, -0.014, 0.060)),
        material=graphite,
        name="mast_front_trim",
    )
    mast.visual(
        Box((0.010, 0.014, 0.150)),
        origin=Origin(xyz=(-0.030, 0.0, -0.020)),
        material=graphite,
        name="left_slider_rail",
    )
    mast.visual(
        Box((0.010, 0.014, 0.150)),
        origin=Origin(xyz=(0.030, 0.0, -0.020)),
        material=graphite,
        name="right_slider_rail",
    )
    mast.visual(
        Box((0.008, 0.012, 0.150)),
        origin=Origin(xyz=(-0.026, 0.0, -0.020)),
        material=matte_black,
        name="left_slider_bridge",
    )
    mast.visual(
        Box((0.008, 0.012, 0.150)),
        origin=Origin(xyz=(0.026, 0.0, -0.020)),
        material=matte_black,
        name="right_slider_bridge",
    )
    mast.visual(
        Box((0.066, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.015, 0.179)),
        material=dark_gray,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.012, 0.022, 0.040)),
        origin=Origin(xyz=(-0.034, 0.015, 0.200)),
        material=dark_gray,
        name="left_yoke_ear",
    )
    mast.visual(
        Box((0.012, 0.022, 0.040)),
        origin=Origin(xyz=(0.034, 0.015, 0.200)),
        material=dark_gray,
        name="right_yoke_ear",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.080, 0.040, 0.340)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.005, 0.080)),
    )

    model.articulation(
        "swivel_to_mast",
        ArticulationType.PRISMATIC,
        parent=swivel_stage,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.008, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="tilt_barrel",
    )
    tilt_head.visual(
        Box((0.062, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.012, 0.012)),
        material=dark_gray,
        name="tilt_bridge",
    )
    tilt_head.visual(
        Box((0.052, 0.046, 0.026)),
        origin=Origin(xyz=(0.0, -0.037, 0.0)),
        material=graphite,
        name="tilt_neck",
    )
    tilt_head.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.0, -0.064, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="portrait_housing",
    )
    tilt_head.visual(
        Box((0.102, 0.004, 0.102)),
        origin=Origin(xyz=(0.0, -0.073, 0.0)),
        material=graphite,
        name="vesa_plate",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.11, 0.09, 0.11)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
    )

    model.articulation(
        "mast_to_tilt_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.015, 0.200)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.25,
            upper=0.40,
        ),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.620, 0.022, 0.390)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=matte_black,
        name="rear_shell",
    )
    screen.visual(
        Box((0.620, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, -0.025, 0.181)),
        material=graphite,
        name="top_bezel",
    )
    screen.visual(
        Box((0.014, 0.006, 0.334)),
        origin=Origin(xyz=(-0.303, -0.025, 0.0)),
        material=graphite,
        name="left_bezel",
    )
    screen.visual(
        Box((0.014, 0.006, 0.334)),
        origin=Origin(xyz=(0.303, -0.025, 0.0)),
        material=graphite,
        name="right_bezel",
    )
    screen.visual(
        Box((0.296, 0.006, 0.028)),
        origin=Origin(xyz=(-0.162, -0.025, -0.181)),
        material=graphite,
        name="lower_left_bezel",
    )
    screen.visual(
        Box((0.296, 0.006, 0.028)),
        origin=Origin(xyz=(0.162, -0.025, -0.181)),
        material=graphite,
        name="lower_right_bezel",
    )
    screen.visual(
        Box((0.592, 0.002, 0.334)),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material=screen_black,
        name="display_panel",
    )
    screen.visual(
        Box((0.235, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=matte_black,
        name="rear_bulge",
    )
    screen.visual(
        Box((0.220, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.145)),
        material=indicator_white,
        name="rear_vent_strip",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.620, 0.040, 0.390)),
        mass=3.2,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
    )

    model.articulation(
        "tilt_head_to_screen",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=screen,
        origin=Origin(xyz=(0.0, -0.071, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.016, 0.004, 0.009)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=indicator_white,
        name="button_cap",
    )
    power_button.visual(
        Box((0.010, 0.002, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=indicator_white,
        name="button_plunger",
    )
    power_button.inertial = Inertial.from_geometry(
        Box((0.016, 0.006, 0.009)),
        mass=0.01,
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
    )

    model.articulation(
        "screen_to_power_button",
        ArticulationType.PRISMATIC,
        parent=screen,
        child=power_button,
        origin=Origin(xyz=(0.0, -0.023, -0.181)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=0.001,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    screen = object_model.get_part("screen")
    button = object_model.get_part("power_button")

    swivel_joint = object_model.get_articulation("base_to_swivel")
    height_joint = object_model.get_articulation("swivel_to_mast")
    tilt_joint = object_model.get_articulation("mast_to_tilt_head")
    portrait_joint = object_model.get_articulation("tilt_head_to_screen")
    button_joint = object_model.get_articulation("screen_to_power_button")

    def aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            upper[0] - lower[0],
            upper[1] - lower[1],
            upper[2] - lower[2],
        )

    ctx.expect_gap(
        screen,
        base,
        axis="z",
        min_gap=0.18,
        name="screen clears the base",
    )
    ctx.expect_contact(
        button,
        screen,
        name="power button remains mounted to the display housing",
    )

    rest_screen_pos = ctx.part_world_position(screen)
    rest_button_pos = ctx.part_world_position(button)
    rest_screen_dims = aabb_dims(ctx.part_world_aabb(screen))

    height_upper = height_joint.motion_limits.upper if height_joint.motion_limits is not None else None
    with ctx.pose({height_joint: height_upper if height_upper is not None else 0.08}):
        raised_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "height adjustment raises the display",
        rest_screen_pos is not None
        and raised_screen_pos is not None
        and raised_screen_pos[2] > rest_screen_pos[2] + 0.07,
        details=f"rest={rest_screen_pos}, raised={raised_screen_pos}",
    )

    with ctx.pose({swivel_joint: 0.5}):
        swiveled_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "base swivel rotates the display around the stand axis",
        rest_screen_pos is not None
        and swiveled_screen_pos is not None
        and abs(swiveled_screen_pos[0] - rest_screen_pos[0]) > 0.02
        and abs(swiveled_screen_pos[2] - rest_screen_pos[2]) < 0.01,
        details=f"rest={rest_screen_pos}, swiveled={swiveled_screen_pos}",
    )

    with ctx.pose({tilt_joint: 0.35}):
        tilted_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "tilt hinge changes the screen elevation",
        rest_screen_pos is not None
        and tilted_screen_pos is not None
        and tilted_screen_pos[2] > rest_screen_pos[2] + 0.015,
        details=f"rest={rest_screen_pos}, tilted={tilted_screen_pos}",
    )

    portrait_upper = portrait_joint.motion_limits.upper if portrait_joint.motion_limits is not None else None
    with ctx.pose({portrait_joint: portrait_upper if portrait_upper is not None else math.pi / 2.0}):
        portrait_screen_dims = aabb_dims(ctx.part_world_aabb(screen))
    ctx.check(
        "portrait pivot swaps the screen aspect orientation",
        rest_screen_dims is not None
        and portrait_screen_dims is not None
        and rest_screen_dims[0] > rest_screen_dims[2]
        and portrait_screen_dims[2] > portrait_screen_dims[0],
        details=f"landscape_dims={rest_screen_dims}, portrait_dims={portrait_screen_dims}",
    )

    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    with ctx.pose({button_joint: button_upper if button_upper is not None else 0.001}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "power button has short prismatic travel",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.0008
        and abs(pressed_button_pos[2] - rest_button_pos[2]) < 1e-6,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
