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
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
    wire_from_points,
)


def _ridged_prism(
    *,
    x0: float,
    x1: float,
    half_width0: float,
    half_width1: float,
    bottom0: float,
    bottom1: float,
    edge_top0: float,
    edge_top1: float,
    ridge_top0: float,
    ridge_top1: float,
) -> MeshGeometry:
    """Five-sided faceted prism with a central ridge, used for toy body panels."""
    geom = MeshGeometry()
    sections = (
        (x0, half_width0, bottom0, edge_top0, ridge_top0),
        (x1, half_width1, bottom1, edge_top1, ridge_top1),
    )
    loops: list[list[int]] = []
    for x, hw, bottom, edge, ridge in sections:
        loop = [
            geom.add_vertex(x, -hw, bottom),
            geom.add_vertex(x, hw, bottom),
            geom.add_vertex(x, hw, edge),
            geom.add_vertex(x, 0.0, ridge),
            geom.add_vertex(x, -hw, edge),
        ]
        loops.append(loop)

    a, b = loops
    # Caps.
    geom.add_face(a[0], a[1], a[2])
    geom.add_face(a[0], a[2], a[3])
    geom.add_face(a[0], a[3], a[4])
    geom.add_face(b[0], b[2], b[1])
    geom.add_face(b[0], b[3], b[2])
    geom.add_face(b[0], b[4], b[3])
    # Longitudinal faces.
    for i in range(5):
        j = (i + 1) % 5
        geom.add_face(a[i], b[i], b[j])
        geom.add_face(a[i], b[j], a[j])
    return geom


def _door_panel(side: float) -> MeshGeometry:
    """Thin trapezoid half-door prism in a child frame whose origin is the hinge pin."""
    geom = MeshGeometry()
    y_inner = 0.0
    y_outer = side * 0.028
    # Local x extends rearward from the hinge.  Local z is centered on hinge height.
    profile = [
        (-0.390, -0.105),
        (-0.006, -0.105),
        (-0.006, 0.115),
        (-0.255, 0.075),
        (-0.390, 0.020),
    ]
    loops: list[list[int]] = []
    for y in (y_inner, y_outer):
        loops.append([geom.add_vertex(x, y, z) for x, z in profile])

    a, b = loops
    geom.add_face(a[0], a[1], a[2])
    geom.add_face(a[0], a[2], a[3])
    geom.add_face(a[0], a[3], a[4])
    geom.add_face(b[0], b[2], b[1])
    geom.add_face(b[0], b[3], b[2])
    geom.add_face(b[0], b[4], b[3])
    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        geom.add_face(a[i], b[i], b[j])
        geom.add_face(a[i], b[j], a[j])
    return geom


def _fender_arch(cx: float, side: float) -> MeshGeometry:
    y_body = side * 0.298
    y_outer = side * 0.392
    points = [
        (cx - 0.225, y_body, 0.295),
        (cx - 0.205, y_outer, 0.315),
        (cx - 0.115, y_outer, 0.425),
        (cx, y_outer, 0.462),
        (cx + 0.115, y_outer, 0.425),
        (cx + 0.205, y_outer, 0.315),
        (cx + 0.225, y_body, 0.295),
    ]
    return wire_from_points(
        points,
        radius=0.018,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.035,
        corner_segments=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_off_road_buggy")

    body_orange = model.material("faceted_orange", rgba=(0.95, 0.38, 0.08, 1.0))
    dark_gray = model.material("dark_chassis", rgba=(0.06, 0.07, 0.08, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    rim_silver = model.material("brushed_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.015, 0.016, 0.018, 1.0))
    seat_black = model.material("matte_seat", rgba=(0.025, 0.025, 0.030, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((1.16, 0.46, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=dark_gray,
        name="belly_pan",
    )
    # Faceted toy body: sloped hood and rear deck plus separated side sills leave the
    # center cabin visibly open for the half-doors.
    chassis.visual(
        mesh_from_geometry(
            _ridged_prism(
                x0=0.145,
                x1=0.650,
                half_width0=0.285,
                half_width1=0.225,
                bottom0=0.268,
                bottom1=0.248,
                edge_top0=0.430,
                edge_top1=0.305,
                ridge_top0=0.485,
                ridge_top1=0.350,
            ),
            "hood_faceted",
        ),
        material=body_orange,
        name="hood_faceted",
    )
    chassis.visual(
        mesh_from_geometry(
            _ridged_prism(
                x0=-0.655,
                x1=-0.250,
                half_width0=0.235,
                half_width1=0.290,
                bottom0=0.248,
                bottom1=0.268,
                edge_top0=0.325,
                edge_top1=0.405,
                ridge_top0=0.370,
                ridge_top1=0.465,
            ),
            "rear_deck_faceted",
        ),
        material=body_orange,
        name="rear_deck_faceted",
    )
    for side in (-1.0, 1.0):
        suffix = "pos" if side > 0 else "neg"
        chassis.visual(
            Box((0.555, 0.055, 0.070)),
            origin=Origin(xyz=(-0.015, side * 0.292, 0.275)),
            material=body_orange,
            name=f"side_sill_{suffix}",
        )
        chassis.visual(
            Box((0.285, 0.055, 0.165)),
            origin=Origin(xyz=(0.445, side * 0.292, 0.350)),
            material=body_orange,
            name=f"front_quarter_{suffix}",
        )
        chassis.visual(
            Box((0.265, 0.055, 0.155)),
            origin=Origin(xyz=(-0.455, side * 0.292, 0.345)),
            material=body_orange,
            name=f"rear_quarter_{suffix}",
        )
        # Door hinge and rear latch posts frame the opening without filling it.
        chassis.visual(
            Cylinder(radius=0.012, length=0.280),
            origin=Origin(xyz=(0.180, side * 0.284, 0.420)),
            material=hinge_black,
            name=f"hinge_post_{suffix}",
        )
        chassis.visual(
            Cylinder(radius=0.010, length=0.220),
            origin=Origin(xyz=(-0.250, side * 0.276, 0.405)),
            material=hinge_black,
            name=f"latch_post_{suffix}",
        )

    # Four separate open fender hoops make the wheel arches read as open, not capped.
    for cx, label in ((0.445, "front"), (-0.445, "rear")):
        for side in (-1.0, 1.0):
            suffix = "pos" if side > 0 else "neg"
            chassis.visual(
                mesh_from_geometry(_fender_arch(cx, side), f"{label}_fender_{suffix}"),
                material=body_orange,
                name=f"{label}_fender_{suffix}",
            )

    # A single continuous tubular roof/roll bar with hard toy-like bends.
    roof_bar = wire_from_points(
        [
            (-0.365, -0.250, 0.420),
            (-0.365, -0.250, 0.710),
            (0.175, -0.250, 0.795),
            (0.175, 0.250, 0.795),
            (-0.365, 0.250, 0.710),
            (-0.365, 0.250, 0.420),
        ],
        radius=0.018,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.050,
        corner_segments=8,
    )
    chassis.visual(
        mesh_from_geometry(roof_bar, "roof_bar"),
        material=hinge_black,
        name="roof_bar",
    )
    # Toy seats are fused to the floor so the open cabin has visible depth.
    for y, name in ((-0.095, "seat_0"), (0.095, "seat_1")):
        chassis.visual(
            Box((0.155, 0.100, 0.055)),
            origin=Origin(xyz=(-0.130, y, 0.318)),
            material=seat_black,
            name=f"{name}_cushion",
        )
        chassis.visual(
            Box((0.045, 0.100, 0.160)),
            origin=Origin(xyz=(-0.215, y, 0.405), rpy=(0.0, math.radians(-14.0), 0.0)),
            material=seat_black,
            name=f"{name}_back",
        )

    # Axles are fixed on the chassis; the wheels are continuous child joints.
    for x, label in ((0.445, "front"), (-0.445, "rear")):
        chassis.visual(
            Cylinder(radius=0.018, length=0.700),
            origin=Origin(xyz=(x, 0.0, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name=f"{label}_axle",
        )
        for side in (-1.0, 1.0):
            suffix = "pos" if side > 0 else "neg"
            chassis.visual(
                Cylinder(radius=0.052, length=0.016),
                origin=Origin(
                    xyz=(x, side * 0.3510, 0.190),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_gray,
                name=f"{label}_hub_stop_{suffix}",
            )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.180,
            0.135,
            inner_radius=0.112,
            carcass=TireCarcass(belt_width_ratio=0.76, sidewall_bulge=0.045),
            tread=TireTread(style="block", depth=0.014, count=18, land_ratio=0.54),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.012, radius=0.004),
        ),
        "chunky_tire",
    )
    wheel_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (0.085, 0.000, 0.000),
                (0.000, 0.000, 0.085),
                (-0.085, 0.000, 0.000),
                (0.000, 0.000, -0.085),
            ],
            radius=0.006,
            radial_segments=10,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.018,
        ),
        "rim_outer_ring",
    )

    wheel_specs = (
        ("wheel_0", 0.445, -0.432),
        ("wheel_1", 0.445, 0.432),
        ("wheel_2", -0.445, -0.432),
        ("wheel_3", -0.445, 0.432),
    )
    for wheel_name, x, y in wheel_specs:
        wheel = model.part(wheel_name)
        # Wheel family helpers spin around local X, so rotate their visuals so the
        # authored wheel axle lies along the buggy's lateral Y axis.
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.114, length=0.122),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rim_silver,
            name="rim_disc",
        )
        wheel.visual(
            Cylinder(radius=0.040, length=0.146),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rim_silver,
            name="hub",
        )
        wheel.visual(
            wheel_mesh,
            material=rim_silver,
            name="rim_ring",
        )
        for spoke_i in range(5):
            theta = spoke_i * 2.0 * math.pi / 5.0
            wheel.visual(
                Box((0.078, 0.018, 0.014)),
                origin=Origin(
                    xyz=(0.061 * math.cos(theta), 0.0, 0.061 * math.sin(theta)),
                    rpy=(0.0, -theta, 0.0),
                ),
                material=rim_silver,
                name=f"spoke_{spoke_i}",
            )
        model.articulation(
            f"chassis_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.190)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    for door_name, side, axis in (
        ("side_door_0", -1.0, (0.0, 0.0, 1.0)),
        ("side_door_1", 1.0, (0.0, 0.0, -1.0)),
    ):
        door = model.part(door_name)
        door.visual(
            mesh_from_geometry(_door_panel(side), f"{door_name}_panel"),
            material=body_orange,
            name="panel",
        )
        door.visual(
            Cylinder(radius=0.014, length=0.200),
            origin=Origin(),
            material=hinge_black,
            name="hinge_sleeve",
        )
        door.visual(
            Box((0.070, 0.010, 0.018)),
            origin=Origin(xyz=(-0.245, side * 0.031, 0.018)),
            material=hinge_black,
            name="pull_handle",
        )
        model.articulation(
            f"chassis_to_{door_name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=door,
            origin=Origin(xyz=(0.180, side * 0.310, 0.420)),
            axis=axis,
            motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel_joints = [object_model.get_articulation(f"chassis_to_wheel_{i}") for i in range(4)]
    ctx.check(
        "four wheels use continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=str([j.articulation_type for j in wheel_joints]),
    )

    door_joints = [
        object_model.get_articulation("chassis_to_side_door_0"),
        object_model.get_articulation("chassis_to_side_door_1"),
    ]
    ctx.check(
        "side half-doors have vertical limited hinges",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            and abs(j.axis[2]) == 1.0
            and j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and j.motion_limits.upper is not None
            and j.motion_limits.upper > 1.0
            for j in door_joints
        ),
        details=str([(j.articulation_type, j.axis, j.motion_limits) for j in door_joints]),
    )

    # At rest, each side door sits inside the side opening and overlaps the opening
    # footprint in XZ without merging into the front quarter panel.
    chassis = object_model.get_part("chassis")
    for door_name in ("side_door_0", "side_door_1"):
        door = object_model.get_part(door_name)
        ctx.expect_overlap(
            door,
            chassis,
            axes="xz",
            min_overlap=0.10,
            elem_a="panel",
            name=f"{door_name} spans side opening",
        )

    # Positive travel opens each door outward away from the cabin.
    neg_door = object_model.get_part("side_door_0")
    pos_door = object_model.get_part("side_door_1")
    neg_closed = ctx.part_world_aabb(neg_door)
    pos_closed = ctx.part_world_aabb(pos_door)
    with ctx.pose({"chassis_to_side_door_0": 1.0, "chassis_to_side_door_1": 1.0}):
        neg_open = ctx.part_world_aabb(neg_door)
        pos_open = ctx.part_world_aabb(pos_door)
    ctx.check(
        "door hinges open outward",
        neg_closed is not None
        and neg_open is not None
        and pos_closed is not None
        and pos_open is not None
        and neg_open[0][1] < neg_closed[0][1] - 0.08
        and pos_open[1][1] > pos_closed[1][1] + 0.08,
        details=f"neg_closed={neg_closed}, neg_open={neg_open}, pos_closed={pos_closed}, pos_open={pos_open}",
    )

    return ctx.report()


object_model = build_object_model()
