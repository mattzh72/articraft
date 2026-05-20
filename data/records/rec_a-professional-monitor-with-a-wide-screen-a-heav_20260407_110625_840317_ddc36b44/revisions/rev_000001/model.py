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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_monitor")

    def xy_section(
        *, z: float, center_y: float, width: float, depth: float, radius: float
    ) -> list[tuple[float, float, float]]:
        return [
            (px, py + center_y, z)
            for px, py in rounded_rect_profile(width, depth, radius)
        ]

    def xz_section(
        *, y: float, width: float, height: float, radius: float
    ) -> list[tuple[float, float, float]]:
        return [
            (px, y, pz)
            for px, pz in rounded_rect_profile(width, height, radius)
        ]

    stand_dark = model.material("stand_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    housing_black = model.material("housing_black", rgba=(0.10, 0.11, 0.12, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    joystick_dark = model.material("joystick_dark", rgba=(0.12, 0.12, 0.12, 1.0))

    pedestal = model.part("pedestal")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.29, 0.23, 0.028), 0.018),
        "monitor_base_plate",
    )
    pedestal.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_dark,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=charcoal,
        name="pedestal_stem",
    )
    pedestal.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=charcoal,
        name="pedestal_collar",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.29, 0.23, 0.07)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.042, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=charcoal,
        name="rotating_collar",
    )
    column_mesh = mesh_from_geometry(
        section_loft(
            [
                xy_section(
                    z=0.012,
                    center_y=0.000,
                    width=0.072,
                    depth=0.036,
                    radius=0.011,
                ),
                xy_section(
                    z=0.180,
                    center_y=-0.008,
                    width=0.064,
                    depth=0.032,
                    radius=0.010,
                ),
                xy_section(
                    z=0.332,
                    center_y=-0.014,
                    width=0.054,
                    depth=0.028,
                    radius=0.008,
                ),
            ]
        ),
        "monitor_column",
    )
    stand.visual(
        column_mesh,
        material=charcoal,
        name="upright_column",
    )
    stand.visual(
        Box((0.074, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, 0.338)),
        material=charcoal,
        name="head_bridge",
    )
    stand.visual(
        Cylinder(radius=0.008, length=0.160),
        origin=Origin(xyz=(0.0, -0.026, 0.338), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_dark,
        name="hinge_barrel",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.35)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.008, 0.175)),
    )

    display = model.part("display")
    display_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                xz_section(y=-0.032, width=0.620, height=0.372, radius=0.018),
                xz_section(y=-0.020, width=0.616, height=0.368, radius=0.018),
                xz_section(y=-0.008, width=0.568, height=0.320, radius=0.024),
            ]
        ),
        "monitor_display_shell",
    )
    display.visual(
        display_shell_mesh,
        material=housing_black,
        name="display_shell",
    )
    display.visual(
        Box((0.592, 0.004, 0.334)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=screen_black,
        name="screen_glass",
    )
    display.visual(
        Box((0.226, 0.018, 0.154)),
        origin=Origin(xyz=(0.0, -0.017, 0.0)),
        material=charcoal,
        name="rear_bulge",
    )
    display.visual(
        Box((0.124, 0.016, 0.072)),
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
        material=stand_dark,
        name="rear_mount_plate",
    )
    display.visual(
        Box((0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.128, -0.006, -0.155)),
        material=stand_dark,
        name="joystick_support",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.62, 0.05, 0.38)),
        mass=4.9,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joystick_dark,
        name="pivot_barrel",
    )
    joystick.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(xyz=(0.0, 0.010, -0.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joystick_dark,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(0.0, 0.018, -0.008)),
        material=joystick_dark,
        name="joystick_cap",
    )
    joystick.inertial = Inertial.from_geometry(
        Box((0.014, 0.026, 0.016)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.012, -0.004)),
    )

    stand_swivel = model.articulation(
        "pedestal_to_stand_swivel",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=-math.radians(55.0),
            upper=math.radians(55.0),
        ),
    )

    display_tilt = model.articulation(
        "stand_to_display_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=display,
        origin=Origin(xyz=(0.0, -0.026, 0.338)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-5.0),
            upper=math.radians(22.0),
        ),
    )

    joystick_pivot = model.articulation(
        "display_to_joystick",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick,
        origin=Origin(xyz=(0.128, 0.0, -0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    model.meta["prompt_articulations"] = {
        "swivel": stand_swivel.name,
        "tilt": display_tilt.name,
        "joystick": joystick_pivot.name,
    }

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    stand = object_model.get_part("stand")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")

    stand_swivel = object_model.get_articulation("pedestal_to_stand_swivel")
    display_tilt = object_model.get_articulation("stand_to_display_tilt")
    joystick_pivot = object_model.get_articulation("display_to_joystick")

    ctx.expect_gap(
        stand,
        pedestal,
        axis="z",
        positive_elem="rotating_collar",
        negative_elem="pedestal_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating collar seats cleanly on the pedestal collar",
    )
    ctx.expect_gap(
        stand,
        display,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="rear_mount_plate",
        max_gap=0.001,
        max_penetration=1e-5,
        name="display mount plate sits against the tilt barrel without burying into it",
    )

    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({stand_swivel: math.radians(35.0)}):
        swivel_display_pos = ctx.part_world_position(display)
    ctx.check(
        "swivel joint moves the display around the pedestal axis",
        rest_display_pos is not None
        and swivel_display_pos is not None
        and abs(swivel_display_pos[0] - rest_display_pos[0]) > 0.012,
        details=f"rest={rest_display_pos}, swiveled={swivel_display_pos}",
    )

    rest_screen_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
    with ctx.pose({display_tilt: math.radians(18.0)}):
        tilted_screen_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
    ctx.check(
        "tilt hinge tips the top of the display rearward",
        rest_screen_aabb is not None
        and tilted_screen_aabb is not None
        and tilted_screen_aabb[1][1] > rest_screen_aabb[1][1] + 0.015,
        details=f"rest={rest_screen_aabb}, tilted={tilted_screen_aabb}",
    )

    rest_joystick_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
    with ctx.pose({joystick_pivot: 0.25}):
        pivoted_joystick_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
    ctx.check(
        "power joystick pivots upward from its rear support",
        rest_joystick_aabb is not None
        and pivoted_joystick_aabb is not None
        and pivoted_joystick_aabb[1][2] > rest_joystick_aabb[1][2] + 0.003,
        details=f"rest={rest_joystick_aabb}, pivoted={pivoted_joystick_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
