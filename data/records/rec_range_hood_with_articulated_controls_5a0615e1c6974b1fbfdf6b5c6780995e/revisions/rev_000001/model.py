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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _extrude_yz_profile_along_x(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness * 0.5
    left_ids = [geom.add_vertex(-half, y, z) for y, z in profile]
    right_ids = [geom.add_vertex(half, y, z) for y, z in profile]
    count = len(profile)

    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(
            geom,
            left_ids[index],
            left_ids[nxt],
            right_ids[nxt],
            right_ids[index],
        )

    for index in range(1, count - 1):
        geom.add_face(left_ids[0], left_ids[index + 1], left_ids[index])
        geom.add_face(right_ids[0], right_ids[index], right_ids[index + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood")

    width = 0.76
    depth = 0.52
    back_height = 0.088
    front_height = 0.062
    shell_thickness = 0.006
    half_width = width * 0.5

    body_color = model.material("body_brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    trim_color = model.material("trim_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    filter_color = model.material("filter_grey", rgba=(0.42, 0.45, 0.48, 1.0))
    button_color = model.material("button_black", rgba=(0.08, 0.09, 0.10, 1.0))
    flap_color = model.material("smoke_glass", rgba=(0.28, 0.34, 0.37, 0.45))

    hood_body = model.part("hood_body")

    side_profile = [
        (0.0, 0.0),
        (0.0, back_height),
        (depth, front_height),
        (depth, 0.0),
    ]
    side_panel_mesh = mesh_from_geometry(
        _extrude_yz_profile_along_x(side_profile, thickness=shell_thickness),
        "range_hood_side_panel",
    )

    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(-half_width + shell_thickness * 0.5, 0.0, 0.0)),
        material=body_color,
        name="left_side_panel",
    )
    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(half_width - shell_thickness * 0.5, 0.0, 0.0)),
        material=body_color,
        name="right_side_panel",
    )

    top_slope_length = math.hypot(depth, back_height - front_height)
    top_slope_angle = math.atan2(front_height - back_height, depth)
    hood_body.visual(
        Box((width, top_slope_length, shell_thickness)),
        origin=Origin(
            xyz=(0.0, depth * 0.5, (back_height + front_height) * 0.5),
            rpy=(top_slope_angle, 0.0, 0.0),
        ),
        material=body_color,
        name="top_shell",
    )
    hood_body.visual(
        Box((0.72, 0.024, 0.076)),
        origin=Origin(xyz=(0.0, 0.012, 0.050)),
        material=trim_color,
        name="wall_mount_plate",
    )
    hood_body.visual(
        Box((width, shell_thickness, 0.026)),
        origin=Origin(xyz=(0.0, depth - shell_thickness * 0.5, 0.059)),
        material=body_color,
        name="front_fascia_upper",
    )
    hood_body.visual(
        Box((width, shell_thickness, 0.010)),
        origin=Origin(xyz=(0.0, depth - shell_thickness * 0.5, 0.037)),
        material=body_color,
        name="front_fascia_lower",
    )
    hood_body.visual(
        Box((0.28, shell_thickness, 0.014)),
        origin=Origin(xyz=(-0.235, depth - shell_thickness * 0.5, 0.049)),
        material=body_color,
        name="left_control_bezel",
    )
    hood_body.visual(
        Box((0.28, shell_thickness, 0.014)),
        origin=Origin(xyz=(0.235, depth - shell_thickness * 0.5, 0.049)),
        material=body_color,
        name="right_control_bezel",
    )
    hood_body.visual(
        Box((0.700, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.088, 0.016)),
        material=trim_color,
        name="rear_filter_rail",
    )
    hood_body.visual(
        Box((0.700, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.438, 0.016)),
        material=trim_color,
        name="front_filter_rail",
    )
    hood_body.visual(
        Box((0.048, 0.370, 0.012)),
        origin=Origin(xyz=(-0.350, 0.263, 0.016)),
        material=trim_color,
        name="left_filter_rail",
    )
    hood_body.visual(
        Box((0.048, 0.370, 0.012)),
        origin=Origin(xyz=(0.350, 0.263, 0.016)),
        material=trim_color,
        name="right_filter_rail",
    )
    hood_body.visual(
        Box((0.652, 0.330, 0.004)),
        origin=Origin(xyz=(0.0, 0.263, 0.018)),
        material=filter_color,
        name="intake_filter",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((width, depth, back_height)),
        mass=11.5,
        origin=Origin(xyz=(0.0, depth * 0.5, back_height * 0.5)),
    )

    deflector = model.part("deflector_flap")
    deflector.visual(
        Cylinder(radius=0.004, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_color,
        name="deflector_hinge_rod",
    )
    deflector.visual(
        Box((0.700, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, 0.004, -0.020)),
        material=flap_color,
        name="deflector_panel",
    )
    deflector.inertial = Inertial.from_geometry(
        Box((0.700, 0.008, 0.040)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.003, -0.020)),
    )

    model.articulation(
        "body_to_deflector",
        ArticulationType.REVOLUTE,
        parent=hood_body,
        child=deflector,
        origin=Origin(xyz=(0.0, depth + 0.004, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    button_positions = (-0.060, 0.0, 0.060)
    button_names = ("power_button", "light_button", "fan_button")
    for x_pos, part_name in zip(button_positions, button_names):
        button = model.part(part_name)
        button.visual(
            Box((0.026, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, 0.0025, 0.0)),
            material=button_color,
            name="button_cap",
        )
        button.visual(
            Box((0.016, 0.010, 0.008)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=button_color,
            name="button_stem",
        )
        button.visual(
            Box((0.010, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
            material=trim_color,
            name="button_plunger",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.026, 0.018, 0.010)),
            mass=0.03,
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(x_pos, depth - shell_thickness - 0.001, 0.049)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood_body = object_model.get_part("hood_body")
    deflector = object_model.get_part("deflector_flap")
    power_button = object_model.get_part("power_button")
    light_button = object_model.get_part("light_button")
    fan_button = object_model.get_part("fan_button")

    deflector_joint = object_model.get_articulation("body_to_deflector")
    power_joint = object_model.get_articulation("body_to_power_button")
    light_joint = object_model.get_articulation("body_to_light_button")
    fan_joint = object_model.get_articulation("body_to_fan_button")

    ctx.check(
        "deflector uses a horizontal hinge",
        deflector_joint.axis == (1.0, 0.0, 0.0)
        and deflector_joint.motion_limits is not None
        and deflector_joint.motion_limits.lower == 0.0
        and deflector_joint.motion_limits.upper is not None
        and deflector_joint.motion_limits.upper >= 0.9,
        details=f"axis={deflector_joint.axis}, limits={deflector_joint.motion_limits}",
    )

    for joint, name in (
        (power_joint, "power"),
        (light_joint, "light"),
        (fan_joint, "fan"),
    ):
        ctx.check(
            f"{name} button plunges inward",
            joint.axis == (0.0, -1.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper == 0.004,
            details=f"axis={joint.axis}, limits={joint.motion_limits}",
        )

    ctx.expect_gap(
        deflector,
        hood_body,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0003,
        name="deflector sits tight to the hood front edge",
    )
    ctx.expect_overlap(
        deflector,
        hood_body,
        axes="x",
        min_overlap=0.65,
        name="deflector spans the intake width",
    )

    rest_deflector_aabb = ctx.part_world_aabb(deflector)
    with ctx.pose({deflector_joint: 0.85}):
        open_deflector_aabb = ctx.part_world_aabb(deflector)
    ctx.check(
        "deflector swings outward when opened",
        rest_deflector_aabb is not None
        and open_deflector_aabb is not None
        and open_deflector_aabb[1][1] > rest_deflector_aabb[1][1] + 0.020,
        details=f"rest={rest_deflector_aabb}, open={open_deflector_aabb}",
    )

    for button_part, joint, label in (
        (power_button, power_joint, "power"),
        (light_button, light_joint, "light"),
        (fan_button, fan_joint, "fan"),
    ):
        rest_aabb = ctx.part_world_aabb(button_part)
        with ctx.pose({joint: 0.004}):
            pressed_aabb = ctx.part_world_aabb(button_part)
        ctx.check(
            f"{label} button retracts into the control strip",
            rest_aabb is not None
            and pressed_aabb is not None
            and pressed_aabb[1][1] < rest_aabb[1][1] - 0.003,
            details=f"rest={rest_aabb}, pressed={pressed_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
