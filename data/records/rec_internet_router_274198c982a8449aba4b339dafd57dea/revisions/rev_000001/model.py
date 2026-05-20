from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _octagon_loop(length: float, width: float, corner: float, z: float) -> list[tuple[float, float, float]]:
    hx = length * 0.5
    hy = width * 0.5
    c = min(corner, hx * 0.75, hy * 0.75)
    return [
        (-hx + c, -hy, z),
        (hx - c, -hy, z),
        (hx, -hy + c, z),
        (hx, hy - c, z),
        (hx - c, hy, z),
        (-hx + c, hy, z),
        (-hx, hy - c, z),
        (-hx, -hy + c, z),
    ]


def _faceted_body_geometry() -> MeshGeometry:
    """Low wedge-like octagonal router shell with real sloped facets."""
    geom = MeshGeometry()
    bottom = _octagon_loop(0.280, 0.180, 0.030, 0.000)
    rim = _octagon_loop(0.280, 0.180, 0.030, 0.034)
    top = _octagon_loop(0.212, 0.112, 0.021, 0.052)
    loops = [bottom, rim, top]
    loop_indices: list[list[int]] = []
    for loop in loops:
        loop_indices.append([geom.add_vertex(x, y, z) for x, y, z in loop])

    count = len(bottom)
    for lower, upper in zip(loop_indices[:-1], loop_indices[1:]):
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])

    bottom_center = geom.add_vertex(0.0, 0.0, 0.0)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(bottom_center, loop_indices[0][i], loop_indices[0][j])

    top_center = geom.add_vertex(0.0, 0.0, 0.052)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(top_center, loop_indices[-1][j], loop_indices[-1][i])

    return geom


def _add_socket(body, *, name: str, xyz: tuple[float, float, float], size: tuple[float, float, float], material) -> None:
    body.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _build_antenna(part, *, rubber, graphite) -> None:
    # Child frame is the hinge axis.  The rectangular blade rises along local +Z.
    part.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="root_neck",
    )
    part.visual(
        Box((0.021, 0.0065, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=rubber,
        name="antenna_blade",
    )
    part.visual(
        Box((0.007, 0.0075, 0.128)),
        origin=Origin(xyz=(0.0, -0.0006, 0.104)),
        material=graphite,
        name="raised_spine",
    )
    part.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        material=rubber,
        name="rounded_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_home_router")

    shell_black = model.material("faceted_black", rgba=(0.018, 0.020, 0.024, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.100, 0.112, 0.125, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.030, 0.031, 0.034, 1.0))
    port_black = model.material("port_black", rgba=(0.003, 0.004, 0.006, 1.0))
    muted_metal = model.material("muted_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    status_cyan = model.material("status_cyan", rgba=(0.10, 0.82, 1.00, 0.82))

    body = model.part("body")
    body.visual(
        _save_mesh(_faceted_body_geometry(), "faceted_router_body"),
        material=shell_black,
        name="body_shell",
    )
    body.visual(
        Box((0.156, 0.070, 0.003)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0530)),
        material=satin_graphite,
        name="top_inlay",
    )
    body.visual(
        _save_mesh(
            SlotPatternPanelGeometry(
                (0.100, 0.044),
                0.003,
                slot_size=(0.014, 0.004),
                pitch=(0.021, 0.011),
                frame=0.006,
                corner_radius=0.004,
            ),
            "rear_top_vent",
        ),
        origin=Origin(xyz=(-0.052, 0.0, 0.0540)),
        material=port_black,
        name="top_vent",
    )
    body.visual(
        _save_mesh(TorusGeometry(radius=0.0185, tube=0.0014, radial_segments=36, tubular_segments=10), "status_light_ring"),
        origin=Origin(xyz=(0.086, 0.0, 0.0525)),
        material=status_cyan,
        name="status_ring",
    )
    body.visual(
        Box((0.004, 0.070, 0.004)),
        origin=Origin(xyz=(0.1415, 0.0, 0.025)),
        material=status_cyan,
        name="front_light_bar",
    )
    for index, y in enumerate((-0.045, -0.015, 0.015, 0.045)):
        body.visual(
            Box((0.004, 0.018, 0.010)),
            origin=Origin(xyz=(-0.1415, y, 0.023)),
            material=port_black,
            name=f"ethernet_port_{index}",
        )
        body.visual(
            Box((0.0045, 0.012, 0.003)),
            origin=Origin(xyz=(-0.144, y, 0.020)),
            material=muted_metal,
            name=f"port_contact_{index}",
        )

    _add_socket(
        body,
        name="rear_socket_0",
        xyz=(-0.136, -0.046, 0.0435),
        size=(0.026, 0.048, 0.023),
        material=satin_graphite,
    )
    _add_socket(
        body,
        name="rear_socket_1",
        xyz=(-0.136, 0.046, 0.0435),
        size=(0.026, 0.048, 0.023),
        material=satin_graphite,
    )
    _add_socket(
        body,
        name="side_socket_0",
        xyz=(0.010, 0.0875, 0.0435),
        size=(0.052, 0.020, 0.023),
        material=satin_graphite,
    )
    _add_socket(
        body,
        name="side_socket_1",
        xyz=(0.010, -0.0875, 0.0435),
        size=(0.052, 0.020, 0.023),
        material=satin_graphite,
    )

    rear_antenna_0 = model.part("rear_antenna_0")
    rear_antenna_1 = model.part("rear_antenna_1")
    side_antenna_0 = model.part("side_antenna_0")
    side_antenna_1 = model.part("side_antenna_1")
    for antenna in (rear_antenna_0, rear_antenna_1, side_antenna_0, side_antenna_1):
        _build_antenna(antenna, rubber=soft_rubber, graphite=satin_graphite)

    knob = model.part("brightness_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=muted_metal,
        name="short_shaft",
    )
    knob.visual(
        _save_mesh(
            KnobGeometry(
                0.026,
                0.014,
                body_style="faceted",
                base_diameter=0.028,
                top_diameter=0.020,
                edge_radius=0.0006,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0007, width=0.0013),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                top_feature=KnobTopFeature(style="top_insert", diameter=0.010, height=0.001),
                center=False,
            ),
            "brightness_knob_cap",
        ),
        origin=Origin(),
        material=satin_graphite,
        name="knob_cap",
    )

    antenna_limits = MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.25)
    model.articulation(
        "body_to_rear_antenna_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_antenna_0,
        origin=Origin(xyz=(-0.145, -0.046, 0.066), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=antenna_limits,
    )
    model.articulation(
        "body_to_rear_antenna_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_antenna_1,
        origin=Origin(xyz=(-0.145, 0.046, 0.066), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=antenna_limits,
    )
    model.articulation(
        "body_to_side_antenna_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_antenna_0,
        origin=Origin(xyz=(0.010, 0.095, 0.066)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=antenna_limits,
    )
    model.articulation(
        "body_to_side_antenna_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_antenna_1,
        origin=Origin(xyz=(0.010, -0.095, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=antenna_limits,
    )
    model.articulation(
        "body_to_brightness_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.086, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    knob = object_model.get_part("brightness_knob")
    knob_joint = object_model.get_articulation("body_to_brightness_knob")

    antenna_specs = [
        ("rear_antenna_0", "body_to_rear_antenna_0", "rear_socket_0", "x", -1.0),
        ("rear_antenna_1", "body_to_rear_antenna_1", "rear_socket_1", "x", -1.0),
        ("side_antenna_0", "body_to_side_antenna_0", "side_socket_0", "y", 1.0),
        ("side_antenna_1", "body_to_side_antenna_1", "side_socket_1", "y", -1.0),
    ]

    ctx.check(
        "four independent antenna hinges",
        all(object_model.get_articulation(spec[1]) is not None for spec in antenna_specs),
        details="Expected two rear and two side antenna revolute joints.",
    )

    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.020,
        elem_a="knob_cap",
        elem_b="body_shell",
        name="brightness knob footprint sits on the router top",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0005,
        name="brightness knob is seated on its short shaft",
    )

    body_aabb = ctx.part_world_aabb(body)
    knob_aabb = ctx.part_world_aabb(knob)
    if body_aabb is not None and knob_aabb is not None:
        body_front = body_aabb[1][0]
        knob_center_x = 0.5 * (knob_aabb[0][0] + knob_aabb[1][0])
        ctx.check(
            "brightness knob is near the front edge",
            knob_center_x > body_front - 0.070,
            details=f"front={body_front:.3f}, knob_center_x={knob_center_x:.3f}",
        )

    for part_name, joint_name, socket_name, outward_axis, outward_sign in antenna_specs:
        antenna = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        ctx.expect_gap(
            antenna,
            body,
            axis="z",
            positive_elem="hinge_barrel",
            negative_elem=socket_name,
            max_gap=0.0015,
            max_penetration=0.0005,
            name=f"{part_name} barrel sits on its base socket",
        )

        rest_aabb = ctx.part_world_aabb(antenna)
        with ctx.pose({joint: 1.10}):
            folded_aabb = ctx.part_world_aabb(antenna)
        if rest_aabb is not None and folded_aabb is not None:
            axis_index = {"x": 0, "y": 1, "z": 2}[outward_axis]
            if outward_sign > 0.0:
                moved_outward = folded_aabb[1][axis_index] > rest_aabb[1][axis_index] + 0.050
            else:
                moved_outward = folded_aabb[0][axis_index] < rest_aabb[0][axis_index] - 0.050
            lowered_tip = folded_aabb[1][2] < rest_aabb[1][2] - 0.030
            ctx.check(
                f"{part_name} rotates outward on its hinge",
                moved_outward and lowered_tip,
                details=f"rest={rest_aabb}, folded={folded_aabb}",
            )

    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: pi}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "brightness knob spins without translating",
        knob_rest is not None
        and knob_rotated is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(knob_rest, knob_rotated)),
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
