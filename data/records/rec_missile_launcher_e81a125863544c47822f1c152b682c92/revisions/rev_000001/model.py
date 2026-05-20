from __future__ import annotations

from math import pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _armored_prism_mesh(length: float, bottom_width: float, top_width: float, height: float) -> MeshGeometry:
    """A compact trapezoidal armored hull, centered on its local origin."""
    lx = length / 2.0
    by = bottom_width / 2.0
    ty = top_width / 2.0
    hz = height / 2.0
    vertices = [
        (-lx, -by, -hz),
        (lx, -by, -hz),
        (lx, by, -hz),
        (-lx, by, -hz),
        (-lx, -ty, hz),
        (lx, -ty, hz),
        (lx, ty, hz),
        (-lx, ty, hz),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_range_launcher_turret")

    olive = model.material("matte_olive", rgba=(0.23, 0.29, 0.20, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.12, 0.16, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.18, 1.0))
    black = model.material("flat_black", rgba=(0.01, 0.011, 0.01, 1.0))
    glass = model.material("blue_glass", rgba=(0.05, 0.18, 0.32, 0.86))
    warning = model.material("safety_yellow", rgba=(0.95, 0.72, 0.12, 1.0))

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.39, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_olive,
        name="base_plinth",
    )
    yaw_base.visual(
        Cylinder(radius=0.26, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=gunmetal,
        name="bearing_top",
    )
    yaw_base.visual(
        Cylinder(radius=0.31, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=gunmetal,
        name="bolt_ring",
    )
    for i, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0, pi, 4.0 * pi / 3.0, 5.0 * pi / 3.0)):
        yaw_base.visual(
            Cylinder(radius=0.018, length=0.011),
            origin=Origin(xyz=(0.285 * sin(angle + pi / 2.0), 0.285 * sin(angle), 0.1545)),
            material=gunmetal,
            name=f"base_bolt_{i}",
        )

    turret_body = model.part("turret_body")
    turret_body.visual(
        Cylinder(radius=0.245, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=gunmetal,
        name="turntable",
    )
    turret_body.visual(
        mesh_from_geometry(_armored_prism_mesh(0.56, 0.42, 0.31, 0.20), "armored_hull"),
        origin=Origin(xyz=(0.03, 0.0, 0.14)),
        material=olive,
        name="armored_hull",
    )
    turret_body.visual(
        Box((0.18, 0.19, 0.045)),
        origin=Origin(xyz=(0.30, -0.13, 0.075)),
        material=dark_olive,
        name="cradle_foot",
    )
    turret_body.visual(
        Box((0.15, 0.035, 0.22)),
        origin=Origin(xyz=(0.355, -0.235, 0.17)),
        material=olive,
        name="cradle_plate_0",
    )
    turret_body.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.36, -0.235, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="cradle_boss_0",
    )
    turret_body.visual(
        Box((0.15, 0.035, 0.22)),
        origin=Origin(xyz=(0.355, -0.025, 0.17)),
        material=olive,
        name="cradle_plate_1",
    )
    turret_body.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.36, -0.025, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="cradle_boss_1",
    )
    turret_body.visual(
        Box((0.12, 0.11, 0.085)),
        origin=Origin(xyz=(0.25, 0.145, 0.265)),
        material=olive,
        name="sensor_pylon",
    )
    turret_body.visual(
        Cylinder(radius=0.070, length=0.17),
        origin=Origin(xyz=(0.30, 0.145, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_olive,
        name="sensor_pod_shell",
    )
    turret_body.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.391, 0.145, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="sensor_lens",
    )
    turret_body.visual(
        Box((0.08, 0.115, 0.010)),
        origin=Origin(xyz=(0.19, 0.145, 0.365)),
        material=black,
        name="sensor_sunshade",
    )

    launcher_box = model.part("launcher_box")
    launcher_box.visual(
        Box((0.43, 0.17, 0.18)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=olive,
        name="launcher_shell",
    )
    launcher_box.visual(
        Cylinder(radius=0.035, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="tilt_pin",
    )
    launcher_box.visual(
        Box((0.40, 0.008, 0.13)),
        origin=Origin(xyz=(0.205, -0.082, 0.0)),
        material=dark_olive,
        name="side_rail_0",
    )
    launcher_box.visual(
        Box((0.40, 0.008, 0.13)),
        origin=Origin(xyz=(0.205, 0.082, 0.0)),
        material=dark_olive,
        name="side_rail_1",
    )
    launcher_box.visual(
        Box((0.03, 0.18, 0.18)),
        origin=Origin(xyz=(0.422, 0.0, 0.0)),
        material=gunmetal,
        name="front_frame",
    )
    for row, z in enumerate((-0.043, 0.043)):
        for col, y in enumerate((-0.044, 0.044)):
            launcher_box.visual(
                Cylinder(radius=0.026, length=0.014),
                origin=Origin(xyz=(0.442, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=black,
                name=f"launch_tube_{row}_{col}",
            )
    launcher_box.visual(
        Box((0.16, 0.016, 0.018)),
        origin=Origin(xyz=(0.25, 0.094, 0.066)),
        material=warning,
        name="warning_stripe",
    )
    for suffix, y in enumerate((-0.073, 0.073)):
        launcher_box.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(xyz=(0.02, y, 0.098), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"cover_hinge_knuckle_{suffix}",
        )

    reload_cover = model.part("reload_cover")
    reload_cover.visual(
        Box((0.30, 0.12, 0.016)),
        origin=Origin(xyz=(0.15, 0.0, 0.008)),
        material=dark_olive,
        name="cover_panel",
    )
    reload_cover.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="cover_hinge_barrel",
    )
    reload_cover.visual(
        Box((0.035, 0.070, 0.008)),
        origin=Origin(xyz=(0.265, 0.0, 0.020)),
        material=black,
        name="cover_latch",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.CONTINUOUS,
        parent=yaw_base,
        child=turret_body,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6),
    )
    model.articulation(
        "body_to_launcher",
        ArticulationType.REVOLUTE,
        parent=turret_body,
        child=launcher_box,
        origin=Origin(xyz=(0.36, -0.13, 0.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-0.25, upper=0.75),
    )
    model.articulation(
        "launcher_to_cover",
        ArticulationType.REVOLUTE,
        parent=launcher_box,
        child=reload_cover,
        origin=Origin(xyz=(0.02, 0.0, 0.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("turret_body")
    base = object_model.get_part("yaw_base")
    launcher = object_model.get_part("launcher_box")
    cover = object_model.get_part("reload_cover")
    pitch = object_model.get_articulation("body_to_launcher")
    cover_hinge = object_model.get_articulation("launcher_to_cover")

    ctx.allow_overlap(
        body,
        launcher,
        elem_a="cradle_plate_0",
        elem_b="tilt_pin",
        reason="The launcher trunnion pin is intentionally captured inside the cradle cheek.",
    )
    ctx.allow_overlap(
        body,
        launcher,
        elem_a="cradle_plate_1",
        elem_b="tilt_pin",
        reason="The launcher trunnion pin is intentionally captured inside the cradle cheek.",
    )
    ctx.allow_overlap(
        body,
        launcher,
        elem_a="cradle_boss_0",
        elem_b="tilt_pin",
        reason="The visible cradle bushing surrounds the launcher trunnion pin.",
    )
    ctx.allow_overlap(
        body,
        launcher,
        elem_a="cradle_boss_1",
        elem_b="tilt_pin",
        reason="The visible cradle bushing surrounds the launcher trunnion pin.",
    )

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="turntable",
        negative_elem="bearing_top",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on yaw bearing",
    )
    ctx.expect_overlap(
        body,
        base,
        axes="xy",
        elem_a="turntable",
        elem_b="bearing_top",
        min_overlap=0.20,
        name="yaw bearing footprint supports turret",
    )
    ctx.expect_overlap(
        launcher,
        body,
        axes="yz",
        elem_a="tilt_pin",
        elem_b="cradle_plate_0",
        min_overlap=0.020,
        name="outer cradle cheek captures tilt pin",
    )
    ctx.expect_overlap(
        launcher,
        body,
        axes="yz",
        elem_a="tilt_pin",
        elem_b="cradle_plate_1",
        min_overlap=0.020,
        name="inner cradle cheek captures tilt pin",
    )
    ctx.expect_overlap(
        launcher,
        body,
        axes="yz",
        elem_a="tilt_pin",
        elem_b="cradle_boss_0",
        min_overlap=0.006,
        name="outer bushing surrounds tilt pin",
    )
    ctx.expect_overlap(
        launcher,
        body,
        axes="yz",
        elem_a="tilt_pin",
        elem_b="cradle_boss_1",
        min_overlap=0.006,
        name="inner bushing surrounds tilt pin",
    )
    ctx.expect_gap(
        cover,
        launcher,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="launcher_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="reload cover rests on launcher top",
    )

    rest_launcher = ctx.part_element_world_aabb(launcher, elem="launcher_shell")
    with ctx.pose({pitch: 0.60}):
        raised_launcher = ctx.part_element_world_aabb(launcher, elem="launcher_shell")
    ctx.check(
        "launcher pitches upward on cradle hinge",
        rest_launcher is not None
        and raised_launcher is not None
        and raised_launcher[1][2] > rest_launcher[1][2] + 0.12,
        details=f"rest={rest_launcher}, raised={raised_launcher}",
    )

    rest_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_hinge: 1.0}):
        raised_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "reload cover hinges upward from box top",
        rest_cover is not None
        and raised_cover is not None
        and raised_cover[1][2] > rest_cover[1][2] + 0.08,
        details=f"rest={rest_cover}, raised={raised_cover}",
    )

    return ctx.report()


object_model = build_object_model()
