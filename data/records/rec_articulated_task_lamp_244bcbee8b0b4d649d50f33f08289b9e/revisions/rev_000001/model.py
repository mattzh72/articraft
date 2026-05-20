from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    MeshGeometry,
)


def _rpy_cylinder_z_to_x() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def _rpy_cylinder_z_to_y() -> tuple[float, float, float]:
    return (-math.pi / 2.0, 0.0, 0.0)


def _spring_geometry(
    *,
    x0: float,
    x1: float,
    y: float,
    z: float,
    coil_radius: float = 0.012,
    turns: int = 12,
    samples_per_turn: int = 10,
):
    points = [(x0, y, z)]
    count = turns * samples_per_turn
    for i in range(count + 1):
        t = i / count
        angle = 2.0 * math.pi * turns * t
        x = x0 + (x1 - x0) * t
        points.append((x, y + coil_radius * math.cos(angle), z + coil_radius * math.sin(angle)))
    points.append((x1, y, z))
    return tube_from_spline_points(
        points,
        radius=0.0024,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
    )


def _shade_shell_geometry() -> MeshGeometry:
    """Thin hollow cylindrical task-lamp shade, open at the mouth."""

    segments = 56
    center_z = -0.055
    # x, outer radius, inner radius. The rear annulus is mostly closed, while
    # the front remains open and visibly hollow.
    stations = (
        (0.045, 0.055, 0.030),
        (0.080, 0.071, 0.060),
        (0.225, 0.076, 0.067),
    )

    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, outer_r, inner_r in stations:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ca = math.cos(a)
            sa = math.sin(a)
            outer_ring.append(geom.add_vertex(x, outer_r * ca, center_z + outer_r * sa))
            inner_ring.append(geom.add_vertex(x, inner_r * ca, center_z + inner_r * sa))
        outer.append(outer_ring)
        inner.append(inner_ring)

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    # Outer skin.
    for j in range(len(stations) - 1):
        for i in range(segments):
            ni = (i + 1) % segments
            add_quad(outer[j][i], outer[j][ni], outer[j + 1][ni], outer[j + 1][i])

    # Inner skin with reversed winding.
    for j in range(len(stations) - 1):
        for i in range(segments):
            ni = (i + 1) % segments
            add_quad(inner[j + 1][i], inner[j + 1][ni], inner[j][ni], inner[j][i])

    # Rear annulus and rounded front lip.
    for i in range(segments):
        ni = (i + 1) % segments
        add_quad(outer[0][i], inner[0][i], inner[0][ni], outer[0][ni])
        add_quad(outer[-1][ni], inner[-1][ni], inner[-1][i], outer[-1][i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_mount_task_lamp")

    black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("dark_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.64, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.78, 0.79, 0.76, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    warm = model.material("warm_glass", rgba=(1.0, 0.86, 0.52, 0.82))

    clamp = model.part("clamp")
    clamp.visual(Box((0.190, 0.120, 0.025)), origin=Origin(xyz=(-0.040, 0.0, -0.130)), material=black, name="top_jaw")
    clamp.visual(Box((0.170, 0.110, 0.024)), origin=Origin(xyz=(-0.045, 0.0, -0.300)), material=black, name="lower_jaw")
    clamp.visual(Box((0.036, 0.120, 0.185)), origin=Origin(xyz=(-0.126, 0.0, -0.215)), material=black, name="clamp_spine")
    clamp.visual(Cylinder(radius=0.023, length=0.102), origin=Origin(xyz=(-0.055, 0.0, -0.086)), material=dark, name="upright_post")
    clamp.visual(Box((0.050, 0.096, 0.010)), origin=Origin(xyz=(0.020, 0.0, -0.147)), material=rubber, name="jaw_pad")
    clamp.visual(Cylinder(radius=0.008, length=0.135), origin=Origin(xyz=(0.020, 0.0, -0.245)), material=steel, name="clamp_screw")
    clamp.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(0.020, 0.0, -0.181)), material=rubber, name="pressure_foot")
    clamp.visual(
        Cylinder(radius=0.008, length=0.115),
        origin=Origin(xyz=(0.020, 0.0, -0.313), rpy=_rpy_cylinder_z_to_y()),
        material=dark,
        name="screw_handle",
    )
    clamp.visual(
        Box((0.082, 0.014, 0.074)),
        origin=Origin(xyz=(0.000, 0.055, 0.000)),
        material=black,
        name="shoulder_yoke_0",
    )
    clamp.visual(
        Box((0.082, 0.014, 0.074)),
        origin=Origin(xyz=(0.000, -0.055, 0.000)),
        material=black,
        name="shoulder_yoke_1",
    )
    clamp.visual(Box((0.085, 0.126, 0.014)), origin=Origin(xyz=(-0.018, 0.0, -0.044)), material=black, name="shoulder_bridge")

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.026, length=0.096),
        origin=Origin(rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="shoulder_barrel",
    )
    arm.visual(Box((0.070, 0.074, 0.024)), origin=Origin(xyz=(0.040, 0.0, 0.018)), material=dark, name="shoulder_lug")
    for idx, y in enumerate((-0.030, 0.030)):
        arm.visual(
            Cylinder(radius=0.0075, length=0.545),
            origin=Origin(xyz=(0.305, y, 0.018), rpy=_rpy_cylinder_z_to_x()),
            material=steel,
            name=f"side_tube_{idx}",
        )
    arm.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.070, 0.0, 0.018), rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="shoulder_crossbar",
    )
    arm.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.555, 0.0, 0.018), rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="tip_crossbar",
    )
    arm.visual(Box((0.064, 0.078, 0.026)), origin=Origin(xyz=(0.570, 0.0, 0.012)), material=dark, name="tip_lug")
    arm.visual(Box((0.040, 0.126, 0.018)), origin=Origin(xyz=(0.585, 0.0, 0.034)), material=black, name="tip_yoke_bridge")
    arm.visual(
        Box((0.060, 0.012, 0.070)),
        origin=Origin(xyz=(0.620, 0.058, 0.000)),
        material=black,
        name="tip_yoke_0",
    )
    arm.visual(
        Box((0.060, 0.012, 0.070)),
        origin=Origin(xyz=(0.620, -0.058, 0.000)),
        material=black,
        name="tip_yoke_1",
    )
    arm.visual(
        Cylinder(radius=0.006, length=0.126),
        origin=Origin(xyz=(0.105, 0.0, -0.025), rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="spring_anchor_0",
    )
    arm.visual(Box((0.014, 0.082, 0.050)), origin=Origin(xyz=(0.105, 0.0, -0.003)), material=dark, name="spring_hanger_0")
    arm.visual(
        Cylinder(radius=0.006, length=0.126),
        origin=Origin(xyz=(0.495, 0.0, -0.025), rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="spring_anchor_1",
    )
    arm.visual(Box((0.014, 0.082, 0.050)), origin=Origin(xyz=(0.495, 0.0, -0.003)), material=dark, name="spring_hanger_1")
    for idx, y in enumerate((-0.046, 0.046)):
        arm.visual(
            mesh_from_geometry(_spring_geometry(x0=0.105, x1=0.495, y=y, z=-0.025), f"balance_spring_{idx}"),
            material=spring_steel,
            name=f"balance_spring_{idx}",
        )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.104),
        origin=Origin(rpy=_rpy_cylinder_z_to_y()),
        material=steel,
        name="shade_trunnion",
    )
    shade.visual(Box((0.092, 0.034, 0.030)), origin=Origin(xyz=(0.046, 0.0, -0.012)), material=dark, name="shade_neck")
    shade.visual(mesh_from_geometry(_shade_shell_geometry(), "shade_shell"), material=black, name="shade_shell")
    shade.visual(
        Cylinder(radius=0.031, length=0.025),
        origin=Origin(xyz=(0.050, 0.0, -0.055), rpy=_rpy_cylinder_z_to_x()),
        material=dark,
        name="socket_collar",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.074),
        origin=Origin(xyz=(0.090, 0.0, -0.055), rpy=_rpy_cylinder_z_to_x()),
        material=dark,
        name="lamp_socket",
    )
    shade.visual(
        Cylinder(radius=0.007, length=0.045),
        origin=Origin(xyz=(0.128, 0.0, -0.055), rpy=_rpy_cylinder_z_to_x()),
        material=steel,
        name="bulb_stem",
    )
    shade.visual(Sphere(radius=0.026), origin=Origin(xyz=(0.156, 0.0, -0.055)), material=warm, name="bulb")

    model.articulation(
        "clamp_to_arm",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=arm,
        origin=Origin(rpy=(0.0, -0.55, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.35, upper=0.65),
        motion_properties=MotionProperties(damping=0.18, friction=0.04),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.620, 0.0, 0.000), rpy=(0.0, 1.05, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("clamp_to_arm")
    shade_pivot = object_model.get_articulation("arm_to_shade")

    ctx.expect_within(
        arm,
        clamp,
        axes="y",
        inner_elem="shoulder_barrel",
        outer_elem="shoulder_bridge",
        margin=0.0,
        name="shoulder barrel is captured between clamp yoke cheeks",
    )
    ctx.expect_overlap(
        arm,
        clamp,
        axes="xz",
        elem_a="shoulder_barrel",
        elem_b="shoulder_yoke_0",
        min_overlap=0.020,
        name="shoulder pivot barrel aligns with yoke plates",
    )
    ctx.expect_within(
        shade,
        arm,
        axes="y",
        inner_elem="shade_trunnion",
        outer_elem="tip_yoke_bridge",
        margin=0.0,
        name="shade trunnion is captured between tip yoke cheeks",
    )
    ctx.expect_overlap(
        shade,
        arm,
        axes="xz",
        elem_a="shade_trunnion",
        elem_b="tip_yoke_0",
        min_overlap=0.020,
        name="shade pivot is centered in the arm-tip yoke",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.55}):
        raised_tip = ctx.part_world_position(shade)
    ctx.check(
        "positive shoulder motion raises the lamp tip",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.20,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    with ctx.pose({shade_pivot: -0.45}):
        lower_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_pivot: 0.45}):
        upper_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    lower_center_z = None if lower_aabb is None else (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
    upper_center_z = None if upper_aabb is None else (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
    ctx.check(
        "shade pivot visibly changes shade aim",
        lower_center_z is not None and upper_center_z is not None and abs(upper_center_z - lower_center_z) > 0.045,
        details=f"lower_center_z={lower_center_z}, upper_center_z={upper_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
