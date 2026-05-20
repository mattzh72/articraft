from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube_x(
    sections: list[tuple[float, float]],
    inner_radius: float,
    *,
    segments: int = 80,
) -> MeshGeometry:
    """Stepped hollow tube whose axis is local X."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, outer_radius in sections:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ca, sa = math.cos(a), math.sin(a)
            outer_row.append(geom.add_vertex(x, outer_radius * ca, outer_radius * sa))
            inner_row.append(geom.add_vertex(x, inner_radius * ca, inner_radius * sa))
        outer.append(outer_row)
        inner.append(inner_row)

    for row in range(len(sections) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            # Outer skin.
            geom.add_face(outer[row][i], outer[row + 1][i], outer[row + 1][j])
            geom.add_face(outer[row][i], outer[row + 1][j], outer[row][j])
            # Inner bore, wound inward.
            geom.add_face(inner[row][i], inner[row + 1][j], inner[row + 1][i])
            geom.add_face(inner[row][i], inner[row][j], inner[row + 1][j])

    # Annular end caps.
    for rings in ((outer[0], inner[0]), (outer[-1], inner[-1])):
        outer_ring, inner_ring = rings
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(outer_ring[i], outer_ring[j], inner_ring[j])
            geom.add_face(outer_ring[i], inner_ring[j], inner_ring[i])
    return geom


def _loop_with_hole_x(
    outer_loop: list[tuple[float, float]],
    inner_loop: list[tuple[float, float]],
    thickness: float,
    *,
    x_center: float = 0.0,
) -> MeshGeometry:
    """Extrude matching 2-D YZ loops into a thin plate with one through hole."""
    if len(outer_loop) != len(inner_loop):
        raise ValueError("outer_loop and inner_loop need the same sample count")
    n = len(outer_loop)
    geom = MeshGeometry()
    x0 = x_center - thickness / 2.0
    x1 = x_center + thickness / 2.0

    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for y, z in outer_loop:
        outer0.append(geom.add_vertex(x0, y, z))
        outer1.append(geom.add_vertex(x1, y, z))
    for y, z in inner_loop:
        inner0.append(geom.add_vertex(x0, y, z))
        inner1.append(geom.add_vertex(x1, y, z))

    for i in range(n):
        j = (i + 1) % n
        # Front/back annular faces.
        geom.add_face(outer0[i], outer0[j], inner0[j])
        geom.add_face(outer0[i], inner0[j], inner0[i])
        geom.add_face(outer1[i], inner1[j], outer1[j])
        geom.add_face(outer1[i], inner1[i], inner1[j])
        # Outer and inner side walls.
        geom.add_face(outer0[i], outer1[i], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer0[j])
        geom.add_face(inner0[i], inner1[j], inner1[i])
        geom.add_face(inner0[i], inner0[j], inner1[j])
    return geom


def _solid_loop_x(
    loop: list[tuple[float, float]],
    thickness: float,
    *,
    x_center: float = 0.0,
) -> MeshGeometry:
    """Extrude a convex YZ loop into a thin solid plate."""
    n = len(loop)
    geom = MeshGeometry()
    x0 = x_center - thickness / 2.0
    x1 = x_center + thickness / 2.0
    back: list[int] = []
    front: list[int] = []
    for y, z in loop:
        back.append(geom.add_vertex(x0, y, z))
        front.append(geom.add_vertex(x1, y, z))
    c0 = geom.add_vertex(x0, sum(p[0] for p in loop) / n, sum(p[1] for p in loop) / n)
    c1 = geom.add_vertex(x1, sum(p[0] for p in loop) / n, sum(p[1] for p in loop) / n)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back[i], front[i], front[j])
        geom.add_face(back[i], front[j], back[j])
        geom.add_face(c0, back[j], back[i])
        geom.add_face(c1, front[i], front[j])
    return geom


def _vertical_capsule_loop(
    *,
    center_z: float,
    length: float,
    width: float,
    samples_per_end: int = 32,
) -> list[tuple[float, float]]:
    """Rounded long slot/profile in the YZ plane."""
    r = width / 2.0
    straight = max(0.0, length - width)
    top_z = center_z + straight / 2.0
    bottom_z = center_z - straight / 2.0
    pts: list[tuple[float, float]] = []
    # Right side bottom-to-top through the top cap, then left side downward.
    for k in range(samples_per_end + 1):
        a = -math.pi / 2.0 + math.pi * k / samples_per_end
        pts.append((r * math.cos(a), top_z + r * math.sin(a)))
    for k in range(samples_per_end + 1):
        a = math.pi / 2.0 + math.pi * k / samples_per_end
        pts.append((r * math.cos(a), bottom_z + r * math.sin(a)))
    return pts


def _oval_tooth_loops(
    *,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    teeth: int = 40,
    samples: int = 160,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(samples):
        a = 2.0 * math.pi * i / samples
        tooth_phase = (a * teeth / (2.0 * math.pi)) % 1.0
        # Pointed but small narrow-wide tooth scallop.
        tooth = 1.0 + 0.045 * (1.0 - abs(2.0 * tooth_phase - 1.0))
        outer.append((outer_y * tooth * math.cos(a), outer_z * tooth * math.sin(a)))
        inner.append((inner_y * math.cos(a), inner_z * math.sin(a)))
    return outer, inner


def _spoke_origin(angle: float, radius: float, x: float) -> Origin:
    # Local box Z length is rotated into the YZ radial direction.
    return Origin(xyz=(x, radius * math.sin(angle), radius * math.cos(angle)), rpy=(-angle, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gravel_one_by_crankset")

    forged_black = model.material("forged_black", rgba=(0.015, 0.018, 0.020, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.23, 0.24, 0.24, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.005, 0.006, 0.007, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.04, 0.045, 0.048, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    composite_black = model.material("composite_black", rgba=(0.025, 0.027, 0.030, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.58, 0.58, 1.0))

    shell = model.part("bottom_bracket_shell")
    shell_body = _annular_tube_x(
        [
            (-0.058, 0.046),
            (-0.049, 0.046),
            (-0.049, 0.038),
            (0.049, 0.038),
            (0.049, 0.046),
            (0.058, 0.046),
        ],
        inner_radius=0.021,
        segments=96,
    )
    shell.visual(
        mesh_from_geometry(shell_body, "wide_hollow_bottom_bracket_shell"),
        material=satin_graphite,
        name="shell_body",
    )
    shell.visual(
        mesh_from_geometry(_annular_tube_x([(-0.057, 0.0216), (-0.050, 0.0216)], 0.0135, segments=64), "left_bearing_race"),
        material=bearing_steel,
        name="bearing_race_0",
    )
    shell.visual(
        mesh_from_geometry(_annular_tube_x([(0.050, 0.0216), (0.057, 0.0216)], 0.0135, segments=64), "right_bearing_race"),
        material=bearing_steel,
        name="bearing_race_1",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="common_spindle",
    )

    # Matching hollow-forged arms: true through-slots in thin forged plates.
    arm_outer_down = _vertical_capsule_loop(center_z=-0.085, length=0.174, width=0.032, samples_per_end=36)
    arm_inner_down = _vertical_capsule_loop(center_z=-0.087, length=0.105, width=0.012, samples_per_end=36)
    arm_outer_up = [(y, -z) for y, z in arm_outer_down]
    arm_inner_up = [(y, -z) for y, z in arm_inner_down]
    crankset.visual(
        mesh_from_geometry(_loop_with_hole_x(arm_outer_down, arm_inner_down, 0.015, x_center=0.067), "right_hollow_crank_arm"),
        material=forged_black,
        name="right_arm",
    )
    crankset.visual(
        mesh_from_geometry(_loop_with_hole_x(arm_outer_up, arm_inner_up, 0.015, x_center=-0.067), "left_hollow_crank_arm"),
        material=forged_black,
        name="left_arm",
    )
    crankset.visual(
        mesh_from_geometry(_solid_loop_x(_vertical_capsule_loop(center_z=-0.090, length=0.100, width=0.014, samples_per_end=18), 0.016, x_center=0.075), "right_recess_shadow"),
        material=dark_recess,
        name="right_arm_recess",
    )
    crankset.visual(
        mesh_from_geometry(_solid_loop_x(_vertical_capsule_loop(center_z=0.090, length=0.100, width=0.014, samples_per_end=18), 0.016, x_center=-0.075), "left_recess_shadow"),
        material=dark_recess,
        name="left_arm_recess",
    )

    # Axle and pedal bosses overlap their own arm plates, as a forged assembly would.
    for x, z, name in (
        (0.067, 0.0, "right_hub_boss"),
        (-0.067, 0.0, "left_hub_boss"),
        (0.067, -0.170, "right_pedal_boss"),
        (-0.067, 0.170, "left_pedal_boss"),
    ):
        crankset.visual(
            Cylinder(radius=0.028 if z == 0.0 else 0.020, length=0.018),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=forged_black,
            name=name,
        )

    # Stub axles for the pedal bodies.  They run inside the hollow pedal bushings.
    crankset.visual(
        Cylinder(radius=0.0055, length=0.060),
        origin=Origin(xyz=(0.101, 0.0, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="right_pedal_axle",
    )
    crankset.visual(
        Cylinder(radius=0.0055, length=0.060),
        origin=Origin(xyz=(-0.101, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="left_pedal_axle",
    )
    crankset.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.077, 0.0, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="right_axle_collar",
    )
    crankset.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(-0.077, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="left_axle_collar",
    )

    # Right-side direct-mount spider and the wide 40T oval 1x ring.
    for idx, angle in enumerate([0.0, 2.0 * math.pi / 5.0, 4.0 * math.pi / 5.0, 6.0 * math.pi / 5.0, 8.0 * math.pi / 5.0]):
        crankset.visual(
            Box((0.007, 0.013, 0.078)),
            origin=_spoke_origin(angle, 0.050, 0.080),
            material=hard_anodized,
            name=f"spider_spoke_{idx}",
        )
    outer_ring, inner_ring = _oval_tooth_loops(outer_y=0.081, outer_z=0.089, inner_y=0.056, inner_z=0.063)
    crankset.visual(
        mesh_from_geometry(_loop_with_hole_x(outer_ring, inner_ring, 0.0055, x_center=0.086), "forty_tooth_oval_chainring"),
        material=hard_anodized,
        name="oval_chainring",
    )
    crankset.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hard_anodized,
        name="direct_mount_lockring",
    )

    right_pedal = model.part("right_pedal")
    left_pedal = model.part("left_pedal")
    for part, side in ((right_pedal, 1.0), (left_pedal, -1.0)):
        side_name = "right" if side > 0 else "left"
        part.visual(
            mesh_from_geometry(_annular_tube_x([(-0.030, 0.013), (0.030, 0.013)], 0.0065, segments=48), f"{side_name}_pedal_hollow_bushing"),
            material=spring_steel,
            name="axle_bushing",
        )
        part.visual(
            Box((0.066, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, 0.038, 0.0)),
            material=composite_black,
            name="front_rail",
        )
        part.visual(
            Box((0.066, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, -0.038, 0.0)),
            material=composite_black,
            name="rear_rail",
        )
        part.visual(
            Box((0.052, 0.050, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=spring_steel,
            name="upper_clip",
        )
        part.visual(
            Box((0.052, 0.050, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=spring_steel,
            name="lower_clip",
        )
        part.visual(
            Box((0.012, 0.010, 0.032)),
            origin=Origin(xyz=(0.0, 0.028, 0.0)),
            material=spring_steel,
            name="front_clip_post",
        )
        part.visual(
            Box((0.012, 0.010, 0.032)),
            origin=Origin(xyz=(0.0, -0.028, 0.0)),
            material=spring_steel,
            name="rear_clip_post",
        )
        part.visual(
            Box((0.012, 0.070, 0.010)),
            origin=Origin(xyz=(side * 0.033, 0.0, 0.0)),
            material=composite_black,
            name="outer_bridge",
        )

    model.articulation(
        "bottom_bracket_axle",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "right_pedal_axle",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=(0.108, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.004),
    )
    model.articulation(
        "left_pedal_axle",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=(-0.108, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.004),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("bottom_bracket_shell")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    bb = object_model.get_articulation("bottom_bracket_axle")
    right_axle = object_model.get_articulation("right_pedal_axle")
    left_axle = object_model.get_articulation("left_pedal_axle")

    ctx.check("40 tooth oval chainring visual is present", crankset.get_visual("oval_chainring") is not None)
    ctx.check("two clip-in pedals are articulated", right_axle.axis == (1.0, 0.0, 0.0) and left_axle.axis == (1.0, 0.0, 0.0))
    ctx.expect_overlap(
        crankset,
        shell,
        axes="x",
        min_overlap=0.090,
        elem_a="common_spindle",
        elem_b="shell_body",
        name="common spindle passes through the wide shell",
    )
    ctx.expect_within(
        crankset,
        shell,
        axes="yz",
        margin=0.0,
        elem_a="common_spindle",
        elem_b="shell_body",
        name="spindle is centered within shell diameter",
    )
    ctx.expect_gap(
        crankset,
        shell,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="right_arm",
        negative_elem="shell_body",
        name="right crank arm clears the shell face",
    )
    ctx.expect_gap(
        shell,
        crankset,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="shell_body",
        negative_elem="left_arm",
        name="left crank arm clears the shell face",
    )

    rest_right = ctx.part_world_position(right_pedal)
    with ctx.pose({bb: math.pi / 2.0}):
        turned_right = ctx.part_world_position(right_pedal)
    ctx.check(
        "bottom bracket axle rotates the pedal orbit",
        rest_right is not None
        and turned_right is not None
        and turned_right[1] > rest_right[1] + 0.14
        and turned_right[2] > rest_right[2] + 0.12,
        details=f"rest={rest_right}, turned={turned_right}",
    )

    with ctx.pose({right_axle: 0.85, left_axle: -0.85}):
        ctx.expect_within(
            crankset,
            right_pedal,
            axes="yz",
            margin=0.0015,
            elem_a="right_pedal_axle",
            elem_b="axle_bushing",
            name="right pedal body revolves around its stub axle",
        )
        ctx.expect_within(
            crankset,
            left_pedal,
            axes="yz",
            margin=0.0015,
            elem_a="left_pedal_axle",
            elem_b="axle_bushing",
            name="left pedal body revolves around its stub axle",
        )

    return ctx.report()


object_model = build_object_model()
