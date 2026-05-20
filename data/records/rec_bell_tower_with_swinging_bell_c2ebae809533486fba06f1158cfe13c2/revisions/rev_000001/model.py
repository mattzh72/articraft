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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _bell_shell_geometry() -> MeshGeometry:
    """Thin flared bronze bell shell in the bell part frame.

    The hinge pin is at local z=0.  The bell shell lives below it with a
    visibly open mouth, an inner wall, and a rolled rim.
    """

    segments = 64
    outer_profile = [
        (0.050, -0.105),
        (0.066, -0.135),
        (0.081, -0.185),
        (0.106, -0.265),
        (0.137, -0.345),
        (0.152, -0.385),
    ]
    inner_profile = [
        (0.038, -0.125),
        (0.052, -0.145),
        (0.068, -0.195),
        (0.092, -0.270),
        (0.124, -0.345),
        (0.140, -0.377),
    ]

    geom = MeshGeometry()

    def add_ring(radius: float, z: float) -> list[int]:
        ids: list[int] = []
        for s in range(segments):
            theta = 2.0 * math.pi * s / segments
            ids.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        return ids

    outer = [add_ring(radius, z) for radius, z in outer_profile]
    inner = [add_ring(radius, z) for radius, z in inner_profile]

    def stitch(a: list[int], b: list[int], *, reverse: bool = False) -> None:
        for s in range(segments):
            sn = (s + 1) % segments
            if reverse:
                geom.add_face(a[s], b[sn], b[s])
                geom.add_face(a[s], a[sn], b[sn])
            else:
                geom.add_face(a[s], b[s], b[sn])
                geom.add_face(a[s], b[sn], a[sn])

    for i in range(len(outer) - 1):
        stitch(outer[i], outer[i + 1])
        stitch(inner[i], inner[i + 1], reverse=True)

    # Close the crown annulus and the thick mouth lip while leaving the bell
    # visibly hollow and open at the bottom.
    stitch(inner[0], outer[0])
    stitch(outer[-1], inner[-1])

    center_top = geom.add_vertex(0.0, 0.0, inner_profile[0][1])
    for s in range(segments):
        sn = (s + 1) % segments
        geom.add_face(center_top, inner[0][s], inner[0][sn])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_bell_post")

    blackened_steel = Material("blackened_steel", rgba=(0.02, 0.025, 0.025, 1.0))
    worn_edges = Material("worn_steel_edges", rgba=(0.10, 0.11, 0.10, 1.0))
    bronze = Material("aged_bronze", rgba=(0.58, 0.35, 0.13, 1.0))
    dark_bronze = Material("dark_bronze_shadow", rgba=(0.29, 0.17, 0.06, 1.0))
    cord_fiber = Material("tan_pull_cord", rgba=(0.78, 0.66, 0.42, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.34, 0.34, 0.025)),
        origin=Origin(xyz=(-0.45, 0.0, 0.0125)),
        material=blackened_steel,
        name="base_plate",
    )
    frame.visual(
        Box((0.055, 0.055, 1.65)),
        origin=Origin(xyz=(-0.45, 0.0, 0.85)),
        material=blackened_steel,
        name="square_post",
    )
    frame.visual(
        Box((0.92, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        material=blackened_steel,
        name="cross_arm",
    )
    frame.visual(
        Box((0.030, 0.055, 0.055)),
        origin=Origin(xyz=(-0.475, 0.0, 1.62)),
        material=worn_edges,
        name="arm_cap_0",
    )
    frame.visual(
        Box((0.030, 0.055, 0.055)),
        origin=Origin(xyz=(0.475, 0.0, 1.62)),
        material=worn_edges,
        name="arm_cap_1",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(-0.45, 0.0, 1.700)),
        material=worn_edges,
        name="post_finial_base",
    )
    frame.visual(
        Sphere(0.032),
        origin=Origin(xyz=(-0.45, 0.0, 1.748)),
        material=worn_edges,
        name="post_finial",
    )

    hinge_z = 1.525
    frame.visual(
        Box((0.120, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 1.591)),
        material=blackened_steel,
        name="clevis_base",
    )
    frame.visual(
        Box((0.014, 0.038, 0.112)),
        origin=Origin(xyz=(-0.044, 0.0, 1.533)),
        material=blackened_steel,
        name="clevis_cheek_0",
    )
    frame.visual(
        Box((0.014, 0.038, 0.112)),
        origin=Origin(xyz=(0.044, 0.0, 1.533)),
        material=blackened_steel,
        name="clevis_cheek_1",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="hinge_pin",
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_geometry(TorusGeometry(0.0155, 0.008, radial_segments=18, tubular_segments=48), "hanger_eye"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="hanger_eye",
    )
    bell.visual(
        Cylinder(radius=0.012, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=bronze,
        name="hanger_stem",
    )
    bell.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=bronze,
        name="crown_cap",
    )
    bell.visual(
        mesh_from_geometry(_bell_shell_geometry(), "bell_shell"),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        mesh_from_geometry(TorusGeometry(0.104, 0.0045, radial_segments=14, tubular_segments=64), "waist_bead"),
        origin=Origin(xyz=(0.0, 0.0, -0.260)),
        material=dark_bronze,
        name="waist_bead",
    )
    bell.visual(
        mesh_from_geometry(TorusGeometry(0.147, 0.008, radial_segments=16, tubular_segments=64), "rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, -0.382)),
        material=dark_bronze,
        name="rolled_rim",
    )
    bell.visual(
        Box((0.007, 0.026, 0.070)),
        origin=Origin(xyz=(-0.045, 0.0, -0.128)),
        material=bronze,
        name="clapper_lug_0",
    )
    bell.visual(
        Box((0.007, 0.026, 0.070)),
        origin=Origin(xyz=(0.045, 0.0, -0.128)),
        material=bronze,
        name="clapper_lug_1",
    )
    bell.visual(
        Cylinder(radius=0.004, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.155), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="clapper_pin",
    )

    clapper = model.part("clapper")
    clapper.visual(
        mesh_from_geometry(TorusGeometry(0.0075, 0.004, radial_segments=14, tubular_segments=36), "clapper_eye"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="clapper_eye",
    )
    clapper.visual(
        Cylinder(radius=0.005, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=worn_edges,
        name="lever_rod",
    )
    clapper.visual(
        Sphere(0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=dark_bronze,
        name="striker_ball",
    )
    clapper.visual(
        Cylinder(radius=0.0025, length=0.510),
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        material=cord_fiber,
        name="pull_cord",
    )

    model.articulation(
        "bell_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "clapper_hinge",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-0.52, upper=0.52),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    bell_hinge = object_model.get_articulation("bell_hinge")
    clapper_hinge = object_model.get_articulation("clapper_hinge")

    ctx.allow_overlap(
        bell,
        frame,
        elem_a="hanger_eye",
        elem_b="hinge_pin",
        reason="The bronze hanger eye is intentionally captured around the fixed steel hinge pin with a tight bearing fit.",
    )
    ctx.allow_overlap(
        clapper,
        bell,
        elem_a="clapper_eye",
        elem_b="clapper_pin",
        reason="The clapper eye is intentionally retained on the small revolute pin inside the bell.",
    )

    ctx.expect_within(
        frame,
        bell,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hanger_eye",
        margin=0.010,
        name="bell hanger eye surrounds main hinge pin",
    )
    ctx.expect_overlap(
        frame,
        bell,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hanger_eye",
        min_overlap=0.006,
        name="main hinge pin passes through hanger eye",
    )
    ctx.expect_within(
        bell,
        clapper,
        axes="yz",
        inner_elem="clapper_pin",
        outer_elem="clapper_eye",
        margin=0.006,
        name="clapper eye surrounds its revolute pin",
    )
    ctx.expect_overlap(
        bell,
        clapper,
        axes="x",
        elem_a="clapper_pin",
        elem_b="clapper_eye",
        min_overlap=0.004,
        name="clapper pin passes through clapper eye",
    )
    ctx.expect_within(
        clapper,
        bell,
        axes="xy",
        inner_elem="striker_ball",
        outer_elem="bell_shell",
        margin=0.002,
        name="striker ball hangs inside the bell mouth",
    )

    bell_shell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    cord_aabb = ctx.part_element_world_aabb(clapper, elem="pull_cord")
    ctx.check(
        "pull cord hangs below the open bell",
        bell_shell_aabb is not None
        and cord_aabb is not None
        and cord_aabb[0][2] < bell_shell_aabb[0][2] - 0.25,
        details=f"bell_shell_aabb={bell_shell_aabb}, cord_aabb={cord_aabb}",
    )

    def center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_bell_y = center_y(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    with ctx.pose({bell_hinge: 0.30}):
        swung_bell_y = center_y(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    ctx.check(
        "bell swings about the horizontal cross-arm hinge",
        rest_bell_y is not None and swung_bell_y is not None and swung_bell_y > rest_bell_y + 0.045,
        details=f"rest_y={rest_bell_y}, swung_y={swung_bell_y}",
    )

    rest_ball_y = center_y(ctx.part_element_world_aabb(clapper, elem="striker_ball"))
    with ctx.pose({clapper_hinge: 0.45}):
        swung_ball_y = center_y(ctx.part_element_world_aabb(clapper, elem="striker_ball"))
    ctx.check(
        "pull-cord clapper lever rotates on its secondary pin",
        rest_ball_y is not None and swung_ball_y is not None and swung_ball_y > rest_ball_y + 0.075,
        details=f"rest_y={rest_ball_y}, swung_y={swung_ball_y}",
    )

    return ctx.report()


object_model = build_object_model()
