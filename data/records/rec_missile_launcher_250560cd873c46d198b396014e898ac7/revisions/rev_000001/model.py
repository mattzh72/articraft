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
)


def _cylinder_between(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    """Return a cylinder and transform whose local +Z runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    pitch = math.atan2(horizontal, dz)
    origin = Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, yaw))
    return Cylinder(radius=0.018, length=length), origin


def _aabb_center_z(aabb: object) -> float | None:
    if aabb is None:
        return None
    aabb_min, aabb_max = aabb
    return (aabb_min[2] + aabb_max[2]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_missile_launcher")

    olive = model.material("olive_drab", rgba=(0.24, 0.30, 0.18, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.12, 0.15, 0.10, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.01, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    steel = model.material("brushed_steel", rgba=(0.45, 0.46, 0.43, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=gunmetal,
        name="center_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=olive,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=gunmetal,
        name="top_bearing_seat",
    )
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        cx = 0.45 * math.cos(yaw)
        cy = 0.45 * math.sin(yaw)
        fx = 0.87 * math.cos(yaw)
        fy = 0.87 * math.sin(yaw)
        pedestal.visual(
            Box((0.82, 0.075, 0.055)),
            origin=Origin(xyz=(cx, cy, 0.075), rpy=(0.0, 0.0, yaw)),
            material=olive,
            name=f"tripod_leg_{i}",
        )
        pedestal.visual(
            Box((0.24, 0.14, 0.035)),
            origin=Origin(xyz=(fx, fy, 0.035), rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=f"foot_pad_{i}",
        )
        brace_geom, brace_origin = _cylinder_between(
            (0.22 * math.cos(yaw), 0.22 * math.sin(yaw), 0.12),
            (0.66 * math.cos(yaw), 0.66 * math.sin(yaw), 0.05),
        )
        pedestal.visual(brace_geom, origin=brace_origin, material=steel, name=f"leg_stay_{i}")

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.20, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=gunmetal,
        name="yaw_disk",
    )
    turntable.visual(
        Box((0.48, 0.72, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=olive,
        name="top_plate",
    )
    for support_name, cheek_name, y in (
        ("side_support_0", "bearing_cheek_0", -0.33),
        ("side_support_1", "bearing_cheek_1", 0.33),
    ):
        turntable.visual(
            Box((0.18, 0.08, 0.48)),
            origin=Origin(xyz=(0.02, y, 0.38)),
            material=olive,
            name=support_name,
        )
        turntable.visual(
            Box((0.14, 0.095, 0.14)),
            origin=Origin(xyz=(0.02, y, 0.42)),
            material=dark_olive,
            name=cheek_name,
        )
    turntable.visual(
        Box((0.12, 0.72, 0.08)),
        origin=Origin(xyz=(-0.16, 0.0, 0.18)),
        material=dark_olive,
        name="rear_crossbeam",
    )

    cradle = model.part("cradle")
    pod_length = 1.10
    pod_width = 0.45
    pod_height = 0.34
    wall = 0.035
    pod_center_x = 0.33
    pod_center_z = 0.03
    front_x = pod_center_x + pod_length / 2.0
    rear_x = pod_center_x - pod_length / 2.0
    cradle.visual(
        Cylinder(radius=0.048, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pitch_axle",
    )
    cradle.visual(
        Box((0.70, 0.54, 0.06)),
        origin=Origin(xyz=(0.23, 0.0, pod_center_z - pod_height / 2.0 - 0.03)),
        material=gunmetal,
        name="cradle_saddle",
    )
    cradle.visual(
        Box((0.16, 0.52, 0.12)),
        origin=Origin(xyz=(-0.02, 0.0, pod_center_z - 0.13)),
        material=gunmetal,
        name="trunnion_block",
    )
    cradle.visual(
        Box((pod_length, pod_width, wall)),
        origin=Origin(xyz=(pod_center_x, 0.0, pod_center_z + pod_height / 2.0 - wall / 2.0)),
        material=olive,
        name="pod_top_wall",
    )
    cradle.visual(
        Box((pod_length, pod_width, wall)),
        origin=Origin(xyz=(pod_center_x, 0.0, pod_center_z - pod_height / 2.0 + wall / 2.0)),
        material=olive,
        name="pod_bottom_wall",
    )
    for name, y in (("pod_side_wall_0", -pod_width / 2.0 + wall / 2.0), ("pod_side_wall_1", pod_width / 2.0 - wall / 2.0)):
        cradle.visual(
            Box((pod_length, wall, pod_height)),
            origin=Origin(xyz=(pod_center_x, y, pod_center_z)),
            material=olive,
            name=name,
        )
    cradle.visual(
        Box((wall, pod_width, pod_height)),
        origin=Origin(xyz=(rear_x + wall / 2.0, 0.0, pod_center_z)),
        material=dark_olive,
        name="rear_cap",
    )
    cradle.visual(
        Box((pod_length, wall * 0.8, pod_height)),
        origin=Origin(xyz=(pod_center_x, 0.0, pod_center_z)),
        material=dark_olive,
        name="vertical_divider",
    )
    cradle.visual(
        Box((pod_length, pod_width, wall * 0.8)),
        origin=Origin(xyz=(pod_center_x, 0.0, pod_center_z)),
        material=dark_olive,
        name="horizontal_divider",
    )
    # A proud, dark front frame makes the four square launch cells legible while leaving them visibly open.
    cradle.visual(
        Box((0.045, pod_width + 0.02, 0.055)),
        origin=Origin(xyz=(front_x + 0.012, 0.0, pod_center_z + pod_height / 2.0 - 0.027)),
        material=black,
        name="front_top_lip",
    )
    cradle.visual(
        Box((0.045, pod_width + 0.02, 0.055)),
        origin=Origin(xyz=(front_x + 0.012, 0.0, pod_center_z - pod_height / 2.0 + 0.027)),
        material=black,
        name="front_bottom_lip",
    )
    for name, y in (("front_side_lip_0", -pod_width / 2.0 + 0.027), ("front_side_lip_1", pod_width / 2.0 - 0.027)):
        cradle.visual(
            Box((0.045, 0.055, pod_height + 0.02)),
            origin=Origin(xyz=(front_x + 0.012, y, pod_center_z)),
            material=black,
            name=name,
        )
    cradle.visual(
        Box((0.045, 0.050, pod_height + 0.02)),
        origin=Origin(xyz=(front_x + 0.014, 0.0, pod_center_z)),
        material=black,
        name="front_vertical_bar",
    )
    cradle.visual(
        Box((0.045, pod_width + 0.02, 0.050)),
        origin=Origin(xyz=(front_x + 0.014, 0.0, pod_center_z)),
        material=black,
        name="front_horizontal_bar",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.02, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.9, lower=-0.25, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    for support_name in ("side_support_0", "side_support_1"):
        ctx.allow_overlap(
            turntable,
            cradle,
            elem_a=support_name,
            elem_b="pitch_axle",
            reason="The pitch axle is intentionally captured through the solid side-support bearing proxy.",
        )
        ctx.expect_overlap(
            turntable,
            cradle,
            axes="xz",
            elem_a=support_name,
            elem_b="pitch_axle",
            min_overlap=0.04,
            name=f"{support_name} captures the pitch axle",
        )
    for cheek_name in ("bearing_cheek_0", "bearing_cheek_1"):
        ctx.allow_overlap(
            turntable,
            cradle,
            elem_a=cheek_name,
            elem_b="pitch_axle",
            reason="The visible cheek block represents a bearing block around the captured pitch axle.",
        )
        ctx.expect_overlap(
            turntable,
            cradle,
            axes="xz",
            elem_a=cheek_name,
            elem_b="pitch_axle",
            min_overlap=0.04,
            name=f"{cheek_name} surrounds the pitch axle",
        )

    ctx.expect_contact(
        pedestal,
        turntable,
        elem_a="top_bearing_seat",
        elem_b="yaw_disk",
        contact_tol=0.002,
        name="yaw turntable sits on pedestal bearing",
    )
    ctx.expect_overlap(
        turntable,
        cradle,
        axes="y",
        elem_a="top_plate",
        elem_b="cradle_saddle",
        min_overlap=0.40,
        name="cradle is centered between yoke supports",
    )

    rest_front_z = _aabb_center_z(ctx.part_element_world_aabb(cradle, elem="front_top_lip"))
    with ctx.pose({pitch: 0.7}):
        raised_front_z = _aabb_center_z(ctx.part_element_world_aabb(cradle, elem="front_top_lip"))
        ctx.expect_overlap(
            turntable,
            cradle,
            axes="y",
            elem_a="side_support_0",
            elem_b="pitch_axle",
            min_overlap=0.02,
            name="raised cradle remains on pitch axle",
        )
    ctx.check(
        "pitch joint raises the launch pod nose",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.15,
        details=f"rest_front_z={rest_front_z}, raised_front_z={raised_front_z}",
    )

    yaw_rest = ctx.part_world_position(turntable)
    with ctx.pose({yaw: 0.8}):
        yawed_position = ctx.part_world_position(turntable)
    ctx.check(
        "yaw joint keeps the turntable on its vertical pedestal axis",
        yaw_rest is not None
        and yawed_position is not None
        and abs(yaw_rest[0] - yawed_position[0]) < 1e-6
        and abs(yaw_rest[1] - yawed_position[1]) < 1e-6,
        details=f"rest={yaw_rest}, yawed={yawed_position}",
    )

    return ctx.report()


object_model = build_object_model()
