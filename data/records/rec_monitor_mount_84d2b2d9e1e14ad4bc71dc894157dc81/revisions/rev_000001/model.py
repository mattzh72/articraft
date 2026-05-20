from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _vesa_plate_mesh():
    outer = rounded_rect_profile(0.140, 0.140, 0.012, corner_segments=8)
    hole = _circle_profile(0.0042, segments=28)
    pitch = 0.075
    holes = [
        _translated_profile(hole, sx * pitch * 0.5, sy * pitch * 0.5)
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, 0.012, center=True),
        "vesa_plate",
    )


def _vertical_cylinder(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_monitor_arm")

    black = model.material("matte_black", rgba=(0.035, 0.038, 0.042, 1.0))
    dark = model.material("dark_anodized", rgba=(0.12, 0.13, 0.14, 1.0))
    grey = model.material("cast_grey", rgba=(0.40, 0.42, 0.44, 1.0))
    metal = model.material("brushed_steel", rgba=(0.72, 0.72, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    clamp = model.part("clamp_base")
    clamp.visual(
        Box((0.190, 0.135, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -0.420)),
        material=black,
        name="top_clamp_plate",
    )
    clamp.visual(
        Box((0.145, 0.105, 0.006)),
        origin=Origin(xyz=(0.018, 0.000, -0.407)),
        material=rubber,
        name="top_rubber_pad",
    )
    clamp.visual(
        Box((0.035, 0.125, 0.165)),
        origin=Origin(xyz=(-0.078, 0.000, -0.495)),
        material=black,
        name="rear_clamp_spine",
    )
    clamp.visual(
        Box((0.138, 0.105, 0.024)),
        origin=Origin(xyz=(-0.023, 0.000, -0.568)),
        material=black,
        name="lower_clamp_jaw",
    )
    _vertical_cylinder(
        clamp,
        radius=0.011,
        length=0.125,
        center=(0.020, 0.000, -0.508),
        material=metal,
        name="clamp_screw",
    )
    _vertical_cylinder(
        clamp,
        radius=0.035,
        length=0.010,
        center=(0.020, 0.000, -0.443),
        material=rubber,
        name="pressure_pad",
    )
    _x_cylinder(
        clamp,
        radius=0.013,
        length=0.092,
        center=(0.020, 0.000, -0.574),
        material=dark,
        name="screw_t_handle",
    )
    _vertical_cylinder(
        clamp,
        radius=0.053,
        length=0.058,
        center=(0.000, 0.000, -0.386),
        material=dark,
        name="post_receiver",
    )
    _vertical_cylinder(
        clamp,
        radius=0.032,
        length=0.344,
        center=(0.000, 0.000, -0.185),
        material=metal,
        name="support_post",
    )
    _vertical_cylinder(
        clamp,
        radius=0.056,
        length=0.030,
        center=(0.000, 0.000, -0.015),
        material=dark,
        name="shoulder_socket",
    )

    lower = model.part("lower_arm")
    _vertical_cylinder(
        lower,
        radius=0.052,
        length=0.035,
        center=(0.000, 0.000, 0.0175),
        material=dark,
        name="shoulder_hub",
    )
    _vertical_cylinder(
        lower,
        radius=0.050,
        length=0.035,
        center=(0.420, 0.000, 0.0175),
        material=dark,
        name="elbow_lower_hub",
    )
    for y, name in ((-0.032, "lower_side_bar_0"), (0.032, "lower_side_bar_1")):
        lower.visual(
            Box((0.330, 0.024, 0.024)),
            origin=Origin(xyz=(0.210, y, 0.022)),
            material=grey,
            name=name,
        )
    _x_cylinder(
        lower,
        radius=0.009,
        length=0.285,
        center=(0.210, 0.000, 0.044),
        material=metal,
        name="lower_gas_strut",
    )
    lower.visual(
        Box((0.032, 0.070, 0.020)),
        origin=Origin(xyz=(0.070, 0.000, 0.039)),
        material=dark,
        name="lower_strut_mount",
    )
    lower.visual(
        Box((0.032, 0.070, 0.020)),
        origin=Origin(xyz=(0.350, 0.000, 0.039)),
        material=dark,
        name="elbow_strut_mount",
    )
    lower.visual(
        Box((0.300, 0.084, 0.010)),
        origin=Origin(xyz=(0.210, 0.000, 0.006)),
        material=black,
        name="lower_cable_tray",
    )

    upper = model.part("upper_arm")
    _vertical_cylinder(
        upper,
        radius=0.047,
        length=0.034,
        center=(0.000, 0.000, 0.017),
        material=dark,
        name="elbow_upper_hub",
    )
    _vertical_cylinder(
        upper,
        radius=0.044,
        length=0.034,
        center=(0.380, 0.000, 0.017),
        material=dark,
        name="head_hub",
    )
    for y, name in ((-0.029, "upper_side_bar_0"), (0.029, "upper_side_bar_1")):
        upper.visual(
            Box((0.312, 0.022, 0.023)),
            origin=Origin(xyz=(0.190, y, 0.021)),
            material=grey,
            name=name,
        )
    _x_cylinder(
        upper,
        radius=0.008,
        length=0.245,
        center=(0.190, 0.000, 0.042),
        material=metal,
        name="upper_gas_strut",
    )
    upper.visual(
        Box((0.028, 0.064, 0.018)),
        origin=Origin(xyz=(0.065, 0.000, 0.037)),
        material=dark,
        name="upper_strut_mount",
    )
    upper.visual(
        Box((0.028, 0.064, 0.018)),
        origin=Origin(xyz=(0.315, 0.000, 0.037)),
        material=dark,
        name="head_strut_mount",
    )
    upper.visual(
        Box((0.270, 0.078, 0.009)),
        origin=Origin(xyz=(0.190, 0.000, 0.006)),
        material=black,
        name="upper_cable_tray",
    )

    head = model.part("head_swivel")
    _vertical_cylinder(
        head,
        radius=0.041,
        length=0.030,
        center=(0.000, 0.000, 0.015),
        material=dark,
        name="swivel_bearing",
    )
    head.visual(
        Box((0.080, 0.054, 0.030)),
        origin=Origin(xyz=(0.045, 0.000, 0.028)),
        material=dark,
        name="head_neck",
    )
    head.visual(
        Box((0.022, 0.098, 0.058)),
        origin=Origin(xyz=(0.068, 0.000, 0.050)),
        material=dark,
        name="yoke_bridge",
    )
    for y, name in ((-0.044, "tilt_cheek_0"), (0.044, "tilt_cheek_1")):
        head.visual(
            Box((0.070, 0.016, 0.070)),
            origin=Origin(xyz=(0.105, y, 0.050)),
            material=dark,
            name=name,
        )
    _y_cylinder(
        head,
        radius=0.012,
        length=0.026,
        center=(0.120, -0.054, 0.050),
        material=metal,
        name="tilt_pin_cap_0",
    )
    _y_cylinder(
        head,
        radius=0.012,
        length=0.026,
        center=(0.120, 0.054, 0.050),
        material=metal,
        name="tilt_pin_cap_1",
    )

    plate = model.part("mount_plate")
    _y_cylinder(
        plate,
        radius=0.018,
        length=0.072,
        center=(0.000, 0.000, 0.000),
        material=metal,
        name="tilt_trunnion",
    )
    plate.visual(
        Box((0.038, 0.052, 0.040)),
        origin=Origin(xyz=(0.019, 0.000, 0.000)),
        material=dark,
        name="plate_lug",
    )
    plate.visual(
        _vesa_plate_mesh(),
        origin=Origin(xyz=(0.044, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="vesa_plate",
    )
    for index, (y, z) in enumerate(
        (
            (-0.0375, -0.0375),
            (-0.0375, 0.0375),
            (0.0375, -0.0375),
            (0.0375, 0.0375),
        )
    ):
        _x_cylinder(
            plate,
            radius=0.0065,
            length=0.002,
            center=(0.0505, y, z),
            material=black,
            name=f"vesa_bore_{index}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=lower,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.420, 0.000, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-2.5, upper=2.5),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=head,
        origin=Origin(xyz=(0.380, 0.000, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=plate,
        origin=Origin(xyz=(0.120, 0.000, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp = object_model.get_part("clamp_base")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    head = object_model.get_part("head_swivel")
    plate = object_model.get_part("mount_plate")

    for joint_name, expected_axis in (
        ("shoulder", (0.0, 0.0, 1.0)),
        ("elbow", (0.0, 0.0, 1.0)),
        ("head_swivel", (0.0, 0.0, 1.0)),
        ("head_tilt", (0.0, 1.0, 0.0)),
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_is_revolute",
            joint is not None and joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint_name}={joint!r}",
        )
        if joint is not None:
            ctx.check(
                f"{joint_name}_axis",
                tuple(round(v, 6) for v in joint.axis) == expected_axis,
                details=f"axis={joint.axis!r}",
            )

    ctx.expect_contact(
        lower,
        clamp,
        elem_a="shoulder_hub",
        elem_b="shoulder_socket",
        name="lower arm sits on shoulder bearing",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="elbow_upper_hub",
        elem_b="elbow_lower_hub",
        name="upper arm sits on elbow bearing",
    )
    ctx.expect_contact(
        head,
        upper,
        elem_a="swivel_bearing",
        elem_b="head_hub",
        name="head swivel sits on upper bearing",
    )
    ctx.expect_contact(
        plate,
        head,
        elem_a="tilt_trunnion",
        elem_b="tilt_cheek_0",
        contact_tol=0.002,
        name="tilt trunnion is captured by yoke",
    )

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    tilt = object_model.get_articulation("head_tilt")

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: 0.6}):
        turned_head_pos = ctx.part_world_position(head)
    ctx.check(
        "shoulder yaw moves downstream arm",
        rest_head_pos is not None
        and turned_head_pos is not None
        and abs(turned_head_pos[1] - rest_head_pos[1]) > 0.15,
        details=f"rest={rest_head_pos}, turned={turned_head_pos}",
    )

    rest_plate_pos = ctx.part_world_position(plate)
    with ctx.pose({elbow: -0.7}):
        bent_plate_pos = ctx.part_world_position(plate)
    ctx.check(
        "elbow yaw bends upper arm",
        rest_plate_pos is not None
        and bent_plate_pos is not None
        and abs(bent_plate_pos[1] - rest_plate_pos[1]) > 0.15,
        details=f"rest={rest_plate_pos}, bent={bent_plate_pos}",
    )

    with ctx.pose({tilt: 0.55}):
        ctx.expect_contact(
            plate,
            head,
            elem_a="tilt_trunnion",
            elem_b="tilt_cheek_0",
            contact_tol=0.003,
            name="tilted head remains in yoke span",
        )

    return ctx.report()


object_model = build_object_model()
