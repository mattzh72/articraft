from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 1.22
BODY_DEPTH = 0.80
BODY_HEIGHT = 0.21
FRONT_FACE_Y = -BODY_DEPTH / 2.0
REAR_FACE_Y = BODY_DEPTH / 2.0
DECK_Z = BODY_HEIGHT
KNOB_CENTER_Z = 0.095

BURNER_XS = (-0.39, 0.00, 0.39)
BURNER_YS = (-0.09, 0.22)
KNOB_XS = (-0.47, -0.28, -0.09, 0.09, 0.28, 0.47)


def _build_body_shape() -> cq.Workplane:
    side_profile = [
        (FRONT_FACE_Y, 0.00),
        (FRONT_FACE_Y, 0.12),
        (-0.33, 0.18),
        (-0.33, BODY_HEIGHT),
        (0.36, BODY_HEIGHT),
        (REAR_FACE_Y, BODY_HEIGHT),
        (REAR_FACE_Y, 0.03),
        (0.37, 0.00),
    ]
    chassis = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(BODY_WIDTH / 2.0, both=True)
    )
    rear_guard = cq.Workplane("XY").box(BODY_WIDTH, 0.018, 0.045).translate(
        (0.0, REAR_FACE_Y - 0.009, BODY_HEIGHT + 0.0225)
    )
    return chassis.union(rear_guard)


def _build_burner_shape() -> cq.Workplane:
    tray_outer_radius = 0.108
    tray_inner_radius = 0.093
    tray_height = 0.024
    cavity_depth = 0.018
    floor_z = tray_height - cavity_depth

    tray = cq.Workplane("XY").circle(tray_outer_radius).extrude(tray_height)
    tray = tray.faces(">Z").workplane().circle(tray_inner_radius).cutBlind(-cavity_depth)

    flame_ring = (
        cq.Workplane("XY")
        .circle(0.064)
        .circle(0.044)
        .extrude(0.004)
        .translate((0.0, 0.0, floor_z))
    )
    pedestal = cq.Workplane("XY").circle(0.026).extrude(0.010).translate((0.0, 0.0, floor_z))
    burner_cap = cq.Workplane("XY").circle(0.045).extrude(0.010).translate((0.0, 0.0, floor_z + 0.010))

    return tray.union(flame_ring).union(pedestal).union(burner_cap)


def _build_grate_shape() -> cq.Workplane:
    outer_size = 0.27
    inner_size = 0.21
    frame_height = 0.018
    foot_height = 0.034
    bridge_width = 0.022
    ring_outer = 0.074
    ring_inner = 0.052
    frame_bottom_z = 0.032

    frame = cq.Workplane("XY").rect(outer_size, outer_size).extrude(frame_height)
    frame = frame.cut(cq.Workplane("XY").rect(inner_size, inner_size).extrude(frame_height))
    frame = frame.translate((0.0, 0.0, frame_bottom_z))

    center_ring = (
        cq.Workplane("XY")
        .circle(ring_outer)
        .circle(ring_inner)
        .extrude(0.016)
        .translate((0.0, 0.0, frame_bottom_z))
    )

    bridge_length = (inner_size / 2.0) - ring_outer
    bridge_center = ring_outer + (bridge_length / 2.0)
    bridge_z = frame_bottom_z

    x_bridge = cq.Workplane("XY").box(bridge_length, bridge_width, 0.016).translate((bridge_center, 0.0, bridge_z + 0.008))
    x_bridge_mirror = cq.Workplane("XY").box(bridge_length, bridge_width, 0.016).translate((-bridge_center, 0.0, bridge_z + 0.008))
    y_bridge = cq.Workplane("XY").box(bridge_width, bridge_length, 0.016).translate((0.0, bridge_center, bridge_z + 0.008))
    y_bridge_mirror = cq.Workplane("XY").box(bridge_width, bridge_length, 0.016).translate((0.0, -bridge_center, bridge_z + 0.008))

    foot_offset = 0.115
    feet = None
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            foot = cq.Workplane("XY").box(0.020, 0.020, foot_height).translate(
                (sx * foot_offset, sy * foot_offset, foot_height / 2.0)
            )
            feet = foot if feet is None else feet.union(foot)

    grate = frame.union(center_ring)
    grate = grate.union(x_bridge).union(x_bridge_mirror).union(y_bridge).union(y_bridge_mirror)
    return grate.union(feet)


def _cluster_count(values: list[float], tol: float) -> int:
    if not values:
        return 0
    clusters = [sorted(values)[0]]
    for value in sorted(values)[1:]:
        if abs(value - clusters[-1]) > tol:
            clusters.append(value)
    return len(clusters)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_six_burner_stovetop")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_finish = model.material("burner_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.07, 0.07, 0.08, 1.0))

    body_mesh = mesh_from_cadquery(_build_body_shape(), "commercial_stove_body")
    burner_mesh = mesh_from_cadquery(_build_burner_shape(), "open_burner")
    grate_mesh = mesh_from_cadquery(_build_grate_shape(), "burner_grate")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.038,
            body_style="skirted",
            top_diameter=0.058,
            crown_radius=0.004,
            edge_radius=0.002,
            skirt=KnobSkirt(0.088, 0.010, flare=0.10),
            grip=KnobGrip(style="fluted", count=20, depth=0.0018),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0012, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.008, flat_depth=0.0015),
            center=False,
        ),
        "range_knob",
    )

    body = model.part("body")
    body.visual(body_mesh, material=stainless, name="housing")

    for row, y in enumerate(BURNER_YS):
        for col, x in enumerate(BURNER_XS):
            body.visual(
                burner_mesh,
                origin=Origin(xyz=(x, y, DECK_Z - 0.001)),
                material=burner_finish,
                name=f"burner_{row}_{col}",
            )
            body.visual(
                grate_mesh,
                origin=Origin(xyz=(x, y, DECK_Z - 0.001)),
                material=cast_iron,
                name=f"grate_{row}_{col}",
            )

    for index, x in enumerate(KNOB_XS):
        knob = model.part(f"knob_{index}")
        knob.visual(knob_mesh, material=knob_finish, name="knob_shell")
        knob.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=stainless,
            name="shaft_collar",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, FRONT_FACE_Y, KNOB_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    burner_names = [f"burner_{row}_{col}" for row in range(2) for col in range(3)]
    grate_names = [f"grate_{row}_{col}" for row in range(2) for col in range(3)]
    knob_names = [f"knob_{index}" for index in range(6)]
    knob_joint_names = [f"body_to_knob_{index}" for index in range(6)]
    body_visual_names = {visual.name for visual in body.visuals}

    ctx.check(
        "six burners six grates and six front knobs are present",
        all(name in body_visual_names for name in burner_names)
        and all(name in body_visual_names for name in grate_names)
        and len(knob_names) == 6,
        details=f"body_visuals={sorted(body_visual_names)!r}, knobs={knob_names}",
    )

    burner_positions = []
    for name in burner_names:
        aabb = ctx.part_element_world_aabb(body, elem=name)
        if aabb is None:
            burner_positions.append(None)
            continue
        mins, maxs = aabb
        burner_positions.append(
            (
                (mins[0] + maxs[0]) / 2.0,
                (mins[1] + maxs[1]) / 2.0,
                (mins[2] + maxs[2]) / 2.0,
            )
        )

    knob_positions = [
        ctx.part_world_position(object_model.get_part(name))
        for name in knob_names
    ]

    if all(position is not None for position in burner_positions):
        burner_xs = [position[0] for position in burner_positions if position is not None]
        burner_ys = [position[1] for position in burner_positions if position is not None]
        ctx.check(
            "burners form a three by two field",
            _cluster_count(burner_xs, 0.18) == 3 and _cluster_count(burner_ys, 0.16) == 2,
            details=f"burner_positions={burner_positions!r}",
        )

    if all(position is not None for position in burner_positions) and all(position is not None for position in knob_positions):
        burner_front_y = min(position[1] for position in burner_positions if position is not None)
        burner_z = min(position[2] for position in burner_positions if position is not None)
        knob_xs = [position[0] for position in knob_positions if position is not None]
        knob_ys = [position[1] for position in knob_positions if position is not None]
        knob_zs = [position[2] for position in knob_positions if position is not None]
        ctx.check(
            "knobs run across the front rail beneath the burner field",
            _cluster_count(knob_xs, 0.08) == 6
            and max(knob_ys) < burner_front_y - 0.20
            and max(knob_zs) < burner_z - 0.06,
            details=f"knob_positions={knob_positions!r}, burner_positions={burner_positions!r}",
        )

    knobs_are_continuous = True
    joint_details: list[str] = []
    for joint_name in knob_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        joint_ok = (
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
        )
        knobs_are_continuous = knobs_are_continuous and joint_ok
        joint_details.append(
            f"{joint_name}: type={joint.articulation_type}, lower={None if limits is None else limits.lower}, upper={None if limits is None else limits.upper}"
        )
    ctx.check(
        "all six knobs rotate continuously",
        knobs_are_continuous,
        details="; ".join(joint_details),
    )

    for name in knob_names:
        knob = object_model.get_part(name)
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            min_gap=0.0,
            max_gap=0.002,
            name=f"{name} stays seated on the front panel",
        )

    with ctx.pose({object_model.get_articulation("body_to_knob_2"): pi / 2.0}):
        ctx.expect_gap(
            body,
            object_model.get_part("knob_2"),
            axis="y",
            min_gap=0.0,
            max_gap=0.002,
            name="rotated center knob remains front mounted",
        )

    return ctx.report()


object_model = build_object_model()
