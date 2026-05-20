from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ARM_WIDTH = 0.084
SIDE_Y = ARM_WIDTH / 2.0
LOWER_LEN = 0.36
FOREARM_LEN = 0.33


def _shade_shell_mesh(
    *,
    start_x: float = 0.052,
    length: float = 0.180,
    radius: float = 0.055,
    wall: float = 0.006,
    center_z: float = -0.055,
    segments: int = 56,
) -> MeshGeometry:
    """Thin open cylindrical lampshade, with its cylinder axis along local +X."""

    inner_radius = radius - wall
    end_x = start_x + length
    geom = MeshGeometry()

    def add_ring(x: float, r: float) -> list[int]:
        ids: list[int] = []
        for i in range(segments):
            a = 2.0 * pi * i / segments
            ids.append(geom.add_vertex(x, r * cos(a), center_z + r * sin(a)))
        return ids

    outer_back = add_ring(start_x, radius)
    outer_front = add_ring(end_x, radius)
    inner_back = add_ring(start_x, inner_radius)
    inner_front = add_ring(end_x, inner_radius)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        j = (i + 1) % segments
        # outside, inside, front lip, and rear lip; front and rear remain open.
        quad(outer_back[i], outer_back[j], outer_front[j], outer_front[i])
        quad(inner_front[i], inner_front[j], inner_back[j], inner_back[i])
        quad(outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        quad(inner_back[i], inner_back[j], outer_back[j], outer_back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_task_lamp")

    model.material("matte_black", rgba=(0.025, 0.026, 0.028, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("dark_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    model.material("warm_light", rgba=(1.00, 0.78, 0.35, 0.90))
    model.material("soft_white", rgba=(0.96, 0.93, 0.84, 1.0))

    # The root frame is the shoulder hinge line.  The wall plate sits just
    # behind it on the -X side, so the arm clearly grows out from a wall mount.
    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.030, 0.180, 0.320)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material="matte_black",
        name="wall_plate",
    )
    wall_mount.visual(
        Box((0.045, 0.040, 0.050)),
        origin=Origin(xyz=(-0.0375, 0.0, 0.0)),
        material="matte_black",
        name="shoulder_standoff",
    )
    wall_mount.visual(
        Cylinder(radius=0.029, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="shoulder_barrel",
    )
    # Four proud screw pads are slightly sunk into the plate face so they read
    # as mounted hardware rather than floating dots.
    for yi in (-0.055, 0.055):
        for zi in (-0.110, 0.110):
            wall_mount.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(-0.043, yi, zi), rpy=(0.0, pi / 2.0, 0.0)),
                material="brushed_steel",
                name=f"screw_{'p' if yi > 0 else 'n'}_{'p' if zi > 0 else 'n'}",
            )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_rubber",
        name="shoulder_pin",
    )
    for ear_name, bar_name, tab_name, y in (
        ("shoulder_ear_0", "side_bar_0", "shoulder_tab_0", -SIDE_Y),
        ("shoulder_ear_1", "side_bar_1", "shoulder_tab_1", SIDE_Y),
    ):
        lower_arm.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="brushed_steel",
            name=ear_name,
        )
        lower_arm.visual(
            Box((0.320, 0.012, 0.018)),
            origin=Origin(xyz=(0.180, y, 0.0)),
            material="brushed_steel",
            name=bar_name,
        )
        lower_arm.visual(
            Box((0.050, 0.014, 0.016)),
            origin=Origin(xyz=(0.032, y, 0.0)),
            material="brushed_steel",
            name=tab_name,
        )
    lower_arm.visual(
        Box((0.018, 0.096, 0.016)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material="brushed_steel",
        name="shoulder_bridge",
    )
    lower_arm.visual(
        Box((0.030, 0.096, 0.016)),
        origin=Origin(xyz=(LOWER_LEN - 0.040, 0.0, 0.0)),
        material="brushed_steel",
        name="elbow_bridge",
    )
    lower_arm.visual(
        Cylinder(radius=0.025, length=0.038),
        origin=Origin(xyz=(LOWER_LEN, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="elbow_barrel",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.010, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_rubber",
        name="elbow_pin",
    )
    for ear_name, bar_name, tab_name, yoke_name, shade_tab_name, y in (
        ("elbow_ear_0", "side_bar_0", "elbow_tab_0", "shade_yoke_0", "shade_tab_0", -SIDE_Y),
        ("elbow_ear_1", "side_bar_1", "elbow_tab_1", "shade_yoke_1", "shade_tab_1", SIDE_Y),
    ):
        forearm.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="brushed_steel",
            name=ear_name,
        )
        forearm.visual(
            Box((0.292, 0.012, 0.016)),
            origin=Origin(xyz=(0.165, y, 0.0)),
            material="brushed_steel",
            name=bar_name,
        )
        forearm.visual(
            Box((0.048, 0.014, 0.016)),
            origin=Origin(xyz=(0.030, y, 0.0)),
            material="brushed_steel",
            name=tab_name,
        )
        forearm.visual(
            Cylinder(radius=0.019, length=0.018),
            origin=Origin(xyz=(FOREARM_LEN, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="brushed_steel",
            name=yoke_name,
        )
        forearm.visual(
            Box((0.070, 0.014, 0.0133)),
            origin=Origin(xyz=(FOREARM_LEN - 0.030, y, -0.02535)),
            material="brushed_steel",
            name=shade_tab_name,
        )
    forearm.visual(
        Box((0.018, 0.096, 0.014)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material="brushed_steel",
        name="elbow_bridge",
    )
    forearm.visual(
        Box((0.018, 0.096, 0.014)),
        origin=Origin(xyz=(FOREARM_LEN - 0.080, 0.0, 0.0)),
        material="brushed_steel",
        name="tip_bridge",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_geometry(_shade_shell_mesh(), "shade_shell"),
        material="matte_black",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="shade_barrel",
    )
    shade.visual(
        Box((0.060, 0.026, 0.036)),
        origin=Origin(xyz=(0.044, 0.0, -0.020)),
        material="matte_black",
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.092, 0.0, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material="soft_white",
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.148, 0.0, -0.055)),
        material="warm_light",
        name="bulb",
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.05, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=forearm,
        origin=Origin(xyz=(LOWER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.45, upper=1.45, effort=12.0, velocity=1.6),
    )
    model.articulation(
        "shade_swivel",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=shade,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    lower_arm = object_model.get_part("lower_arm")
    forearm = object_model.get_part("forearm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")
    swivel = object_model.get_articulation("shade_swivel")

    ctx.allow_overlap(
        wall_mount,
        lower_arm,
        elem_a="shoulder_barrel",
        elem_b="shoulder_pin",
        reason="The lower-arm hinge pin is intentionally captured inside the wall-mounted shoulder bushing.",
    )
    ctx.allow_overlap(
        lower_arm,
        forearm,
        elem_a="elbow_barrel",
        elem_b="elbow_pin",
        reason="The forearm hinge pin is intentionally captured inside the elbow bushing.",
    )
    ctx.allow_overlap(
        forearm,
        shade,
        elem_a="shade_yoke_0",
        elem_b="shade_barrel",
        reason="The shade swivel shaft intentionally passes through the tip yoke ear.",
    )
    ctx.allow_overlap(
        forearm,
        shade,
        elem_a="shade_yoke_1",
        elem_b="shade_barrel",
        reason="The shade swivel shaft intentionally passes through the tip yoke ear.",
    )

    ctx.expect_within(
        lower_arm,
        wall_mount,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_barrel",
        name="shoulder pin stays centered in the wall bushing",
    )
    ctx.expect_overlap(
        lower_arm,
        wall_mount,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        min_overlap=0.035,
        name="shoulder pin remains captured through the bushing",
    )
    ctx.expect_within(
        forearm,
        lower_arm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_barrel",
        name="elbow pin stays centered in the elbow bushing",
    )
    ctx.expect_overlap(
        forearm,
        lower_arm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        min_overlap=0.030,
        name="elbow pin remains captured through the bushing",
    )
    ctx.expect_overlap(
        lower_arm,
        wall_mount,
        axes="xz",
        elem_a="shoulder_ear_0",
        elem_b="shoulder_barrel",
        min_overlap=0.020,
        name="shoulder hinge barrels share a coaxial envelope",
    )
    ctx.expect_overlap(
        forearm,
        lower_arm,
        axes="xz",
        elem_a="elbow_ear_0",
        elem_b="elbow_barrel",
        min_overlap=0.018,
        name="elbow hinge barrels share a coaxial envelope",
    )
    ctx.expect_overlap(
        shade,
        forearm,
        axes="xz",
        elem_a="shade_barrel",
        elem_b="shade_yoke_0",
        min_overlap=0.015,
        name="shade swivel is held between tip yoke ears",
    )
    ctx.expect_overlap(
        shade,
        forearm,
        axes="y",
        elem_a="shade_barrel",
        elem_b="shade_yoke_0",
        min_overlap=0.008,
        name="shade shaft passes through one yoke ear",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.70, elbow: 0.55, swivel: -0.60}):
        raised_tip = ctx.part_world_position(shade)
        ctx.expect_origin_gap(
            shade,
            wall_mount,
            axis="x",
            min_gap=0.35,
            name="folded arm still projects out from the wall",
        )
    ctx.check(
        "two arm hinges raise the shade",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[2] > rest_tip[2] + 0.25,
        details=f"rest={rest_tip}, raised={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
