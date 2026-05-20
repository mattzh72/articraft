from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48, *, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(phase + math.tau * i / segments), radius * math.sin(phase + math.tau * i / segments))
        for i in range(segments)
    ]


def _toothed_profile(
    *,
    teeth: int = 38,
    root_radius: float = 0.164,
    tip_radius: float = 0.176,
) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    # Four samples per tooth: root flank, tip plateau, tip plateau, root flank.
    for tooth in range(teeth):
        base = math.tau * tooth / teeth
        for frac, radius in ((0.08, root_radius), (0.32, tip_radius), (0.68, tip_radius), (0.92, root_radius)):
            angle = base + math.tau * frac / teeth
            pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def _build_chainring_mesh() -> MeshGeometry:
    holes = [_circle_profile(0.118, 80)]
    # Four bolt-circle windows plus four ovalized lightening windows make the part
    # read as a real machined chainring rather than a plain disk.
    for i in range(4):
        angle = math.tau * i / 4.0 + math.pi / 4.0
        cx = 0.060 * math.cos(angle)
        cy = 0.060 * math.sin(angle)
        holes.append([(cx + x, cy + y) for x, y in _circle_profile(0.008, 24)])
    for i in range(4):
        angle = math.tau * i / 4.0
        cx = 0.092 * math.cos(angle)
        cy = 0.092 * math.sin(angle)
        # Rounded rectangular-ish reliefs arranged on the web.
        relief: list[tuple[float, float]] = []
        for j in range(20):
            t = math.tau * j / 20
            relief.append((cx + 0.018 * math.cos(t) * math.cos(angle) - 0.007 * math.sin(t) * math.sin(angle),
                           cy + 0.018 * math.cos(t) * math.sin(angle) + 0.007 * math.sin(t) * math.cos(angle)))
        holes.append(relief)

    ring = ExtrudeWithHolesGeometry(
        _toothed_profile(),
        holes,
        0.006,
        center=True,
    )
    # Extrusion axis becomes the bottom-bracket X axis; the annulus lies in the
    # crank rotation plane.
    ring.rotate_y(math.pi / 2.0)
    ring.translate(0.096, 0.0, 0.0)
    return ring


def _build_crank_arm_mesh(*, upward: bool, x_offset: float) -> MeshGeometry:
    # 2-D planform in local (length, width); after rotation local +length maps
    # to world -Z.  A pi flip around X makes the non-drive crank 180 degrees
    # opposed.
    profile = [
        (-0.018, -0.010),
        (-0.024, 0.000),
        (-0.018, 0.010),
        (0.018, 0.021),
        (0.080, 0.015),
        (0.145, 0.011),
        (0.171, 0.017),
        (0.184, 0.000),
        (0.171, -0.017),
        (0.145, -0.011),
        (0.080, -0.015),
        (0.018, -0.021),
    ]
    arm = ExtrudeGeometry(profile, 0.016, center=True)
    arm.rotate_y(math.pi / 2.0)
    if upward:
        arm.rotate_x(math.pi)
    arm.translate(x_offset, 0.0, 0.0)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torque_sensing_ebike_crankset")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.035, 0.038, 0.043, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.62, 0.64, 0.67, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    amber_sensor = model.material("amber_sensor", rgba=(0.92, 0.48, 0.09, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))

    shell = model.part("bottom_bracket_shell")
    shell.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.041, -0.084),
                    (0.041, -0.070),
                    (0.036, -0.070),
                    (0.036, 0.070),
                    (0.041, 0.070),
                    (0.041, 0.084),
                ],
                [
                    (0.020, -0.084),
                    (0.020, 0.084),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "wide_bottom_bracket_shell",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="hollow_shell",
    )
    shell.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.0395, -0.011), (0.0395, 0.011)],
                [(0.0352, -0.011), (0.0352, 0.011)],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
            "torque_sensor_band",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber_sensor,
        name="sensor_band",
    )
    shell.visual(
        Box((0.044, 0.026, 0.014)),
        origin=Origin(xyz=(0.018, 0.000, 0.040)),
        material=matte_black,
        name="sensor_pod",
    )
    shell.visual(
        Cylinder(radius=0.0032, length=0.058),
        origin=Origin(xyz=(0.048, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="sensor_cable",
    )
    for x_pos, bearing_name in ((-0.055, "bearing_0"), (0.055, "bearing_1")):
        shell.visual(
            mesh_from_geometry(
                LatheGeometry.from_shell_profiles(
                    [(0.0204, -0.006), (0.0204, 0.006)],
                    [(0.0186, -0.006), (0.0186, 0.006)],
                    segments=48,
                    start_cap="flat",
                    end_cap="flat",
                ),
                bearing_name,
            ),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=bearing_name,
        )
        for i in range(6):
            angle = math.tau * i / 6.0
            shell.visual(
                Sphere(radius=0.0041),
                origin=Origin(xyz=(x_pos, 0.0146 * math.cos(angle), 0.0146 * math.sin(angle))),
                material=brushed_steel,
                name=f"{bearing_name}_ball_{i}",
            )

    crank = model.part("crank_assembly")
    crank.visual(
        Cylinder(radius=0.0105, length=0.230),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="drive_spline_collar",
    )
    crank.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(-0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="spindle_collar",
    )
    crank.visual(
        mesh_from_geometry(_build_crank_arm_mesh(upward=False, x_offset=0.123), "drive_crank_arm"),
        material=dark_anodized,
        name="drive_arm",
    )
    crank.visual(
        mesh_from_geometry(_build_crank_arm_mesh(upward=True, x_offset=-0.123), "crank_arm"),
        material=dark_anodized,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.123, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="drive_hub_boss",
    )
    crank.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(-0.123, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hub_boss",
    )
    crank.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.123, 0.0, -0.172), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="drive_pedal_eye",
    )
    crank.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.123, 0.0, 0.172), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pedal_eye",
    )
    crank.visual(
        mesh_from_geometry(_build_chainring_mesh(), "toothed_chainring"),
        material=matte_black,
        name="chainring",
    )
    for i in range(5):
        theta = math.tau * i / 5.0 + math.radians(18.0)
        radius = 0.082
        crank.visual(
            Box((0.008, 0.017, 0.106)),
            origin=Origin(
                xyz=(0.096, radius * math.cos(theta), radius * math.sin(theta)),
                rpy=(theta - math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_anodized,
            name=f"spider_arm_{i}",
        )
        crank.visual(
            Cylinder(radius=0.0062, length=0.010),
            origin=Origin(
                xyz=(0.101, 0.132 * math.cos(theta), 0.132 * math.sin(theta)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_steel,
            name=f"chainring_bolt_{i}",
        )

    drive_pedal = model.part("pedal_spindle_0")
    drive_pedal.visual(
        Cylinder(radius=0.0052, length=0.080),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pedal_threaded_shaft",
    )
    drive_pedal.visual(
        Box((0.012, 0.014, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_hex",
    )
    drive_pedal.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.087, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pedal_outer_bushing",
    )

    pedal = model.part("pedal_spindle_1")
    pedal.visual(
        Cylinder(radius=0.0052, length=0.080),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pedal_threaded_shaft",
    )
    pedal.visual(
        Box((0.012, 0.014, 0.014)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_hex",
    )
    pedal.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(-0.087, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pedal_outer_bushing",
    )

    model.articulation(
        "bottom_bracket_to_crank",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=16.0),
    )
    model.articulation(
        "drive_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=drive_pedal,
        origin=Origin(xyz=(0.132, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal,
        origin=Origin(xyz=(-0.132, 0.0, 0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bottom_bracket_shell")
    crank = object_model.get_part("crank_assembly")
    drive_pedal = object_model.get_part("pedal_spindle_0")
    pedal = object_model.get_part("pedal_spindle_1")
    main_spin = object_model.get_articulation("bottom_bracket_to_crank")
    drive_pedal_spin = object_model.get_articulation("drive_pedal_spin")
    pedal_spin = object_model.get_articulation("pedal_spin")

    ctx.check(
        "bottom bracket joint is continuous about the shell axis",
        main_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(main_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={main_spin.articulation_type}, axis={main_spin.axis}",
    )
    ctx.check(
        "both pedal spindles spin continuously about their own axes",
        drive_pedal_spin.articulation_type == ArticulationType.CONTINUOUS
        and pedal_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(drive_pedal_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(pedal_spin.axis) == (1.0, 0.0, 0.0),
        details=f"drive={drive_pedal_spin.articulation_type}/{drive_pedal_spin.axis}, "
        f"other={pedal_spin.articulation_type}/{pedal_spin.axis}",
    )
    ctx.expect_within(
        crank,
        shell,
        axes="yz",
        inner_elem="spindle",
        outer_elem="hollow_shell",
        margin=0.010,
        name="spindle runs through the hollow bottom bracket bore",
    )
    ctx.expect_gap(
        drive_pedal,
        crank,
        axis="x",
        positive_elem="pedal_threaded_shaft",
        negative_elem="drive_pedal_eye",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="drive pedal spindle seats at the crank eye",
    )
    ctx.expect_gap(
        crank,
        pedal,
        axis="x",
        positive_elem="pedal_eye",
        negative_elem="pedal_threaded_shaft",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="opposed pedal spindle seats at the crank eye",
    )

    rest_aabb = ctx.part_element_world_aabb(crank, elem="drive_arm")
    rest_center_y = None if rest_aabb is None else 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
    with ctx.pose({main_spin: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(crank, elem="drive_arm")
        rotated_center_y = None if rotated_aabb is None else 0.5 * (rotated_aabb[0][1] + rotated_aabb[1][1])
    ctx.check(
        "crank arm follows continuous bottom bracket rotation",
        rest_center_y is not None and rotated_center_y is not None and rotated_center_y > rest_center_y + 0.06,
        details=f"rest_y={rest_center_y}, rotated_y={rotated_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
