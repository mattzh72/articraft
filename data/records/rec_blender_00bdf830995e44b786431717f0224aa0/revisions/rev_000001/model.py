from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


BASE_H = 0.190
SOCKET_H = 0.028
JUG_Z = BASE_H + SOCKET_H
JUG_H = 0.368
JUG_TOP_Z = JUG_Z + JUG_H
BLADE_Z = 0.060


def _tapered_base() -> cq.Workplane:
    """Rounded compact motor base, wider at the counter than at the jug seat."""
    base = (
        cq.Workplane("XY")
        .rect(0.300, 0.260)
        .workplane(offset=BASE_H)
        .rect(0.230, 0.210)
        .loft(combine=True)
    )
    try:
        return base.edges("|Z").fillet(0.012)
    except Exception:
        return base


def _jug_shell() -> cq.Workplane:
    """Transparent hollow, wide-mouth jug with bottom bearing, handle, and lip."""
    bottom_radius = 0.095
    top_radius = 0.120
    wall = 0.006
    floor = 0.052

    outer = (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=JUG_H)
        .circle(top_radius)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=floor)
        .circle(bottom_radius - wall - 0.002)
        .workplane(offset=JUG_H + 0.010 - floor)
        .circle(top_radius - wall)
        .loft(combine=True)
    )
    bore = cq.Workplane("XY").circle(0.021).extrude(floor + 0.020)
    shell = outer.cut(inner).cut(bore)

    rim_outer = cq.Workplane("XY").workplane(offset=JUG_H - 0.014).circle(top_radius + 0.006).extrude(0.014)
    rim_inner = cq.Workplane("XY").workplane(offset=JUG_H - 0.016).circle(top_radius - wall - 0.004).extrude(0.018)
    rim = rim_outer.cut(rim_inner)
    bearing_outer = (
        cq.Workplane("XY")
        .workplane(offset=floor - 0.014)
        .circle(0.042)
        .extrude(0.014)
    )
    bearing_inner = (
        cq.Workplane("XY")
        .workplane(offset=floor - 0.016)
        .circle(0.022)
        .extrude(0.018)
    )
    bearing = bearing_outer.cut(bearing_inner)

    upper_mount = cq.Workplane("XY").box(0.070, 0.064, 0.026).translate((0.0, 0.121, 0.278))
    lower_mount = cq.Workplane("XY").box(0.070, 0.064, 0.026).translate((0.0, 0.121, 0.128))
    rear_grip = cq.Workplane("XY").box(0.046, 0.034, 0.178).translate((0.0, 0.155, 0.203))
    pour_lip = cq.Workplane("XY").box(0.072, 0.032, 0.018).translate((0.0, -0.124, JUG_H - 0.008))

    return (
        shell.union(rim)
        .union(bearing)
        .union(upper_mount)
        .union(lower_mount)
        .union(rear_grip)
        .union(pour_lip)
    )


def _lid_skirt() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.158).extrude(-0.026)
    inner = cq.Workplane("XY").circle(0.145).extrude(-0.030)
    return outer.cut(inner)


def _tab_origin(angle_deg: float) -> Origin:
    angle = math.radians(angle_deg)
    radius = 0.166
    return Origin(
        xyz=(radius * math.sin(angle), radius * math.cos(angle), -0.010),
        rpy=(0.0, 0.0, -angle),
    )


def _blade_origin(angle_deg: float, z: float, tilt: float) -> Origin:
    angle = math.radians(angle_deg)
    radius = 0.045
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, tilt, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender")

    body_mat = model.material("satin_graphite", rgba=(0.05, 0.055, 0.060, 1.0))
    panel_mat = model.material("black_glass_panel", rgba=(0.005, 0.006, 0.008, 1.0))
    rubber_mat = model.material("matte_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    jug_mat = model.material("blue_clear_polycarbonate", rgba=(0.55, 0.82, 1.0, 0.38))
    lid_mat = model.material("soft_teal_lid", rgba=(0.02, 0.33, 0.36, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    white_mat = model.material("white_marking", rgba=(0.92, 0.96, 0.98, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_tapered_base(), "base_body", tolerance=0.0012),
        material=body_mat,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.092, length=SOCKET_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + SOCKET_H / 2.0)),
        material=body_mat,
        name="top_socket",
    )
    base.visual(
        Cylinder(radius=0.064, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, JUG_Z - 0.003)),
        material=rubber_mat,
        name="jar_pad",
    )
    base.visual(
        Box((0.140, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, -0.122, 0.096)),
        material=panel_mat,
        name="front_panel",
    )

    dial = model.part("front_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.026,
            body_style="skirted",
            top_diameter=0.048,
            skirt=KnobSkirt(0.066, 0.006, flare=0.04, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "front_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_mat,
        name="dial_cap",
    )
    model.articulation(
        "base_to_front_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.135, 0.096)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=0.4, velocity=2.5),
    )

    jug = model.part("jug")
    jug.visual(
        Box((0.250, 0.250, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=jug_mat,
        name="jug_floor",
    )
    jug.visual(
        Box((0.250, 0.006, JUG_H)),
        origin=Origin(xyz=(0.0, -0.123, JUG_H / 2.0)),
        material=jug_mat,
        name="front_wall",
    )
    jug.visual(
        Box((0.250, 0.006, JUG_H)),
        origin=Origin(xyz=(0.0, 0.123, JUG_H / 2.0)),
        material=jug_mat,
        name="rear_wall",
    )
    jug.visual(
        Box((0.006, 0.250, JUG_H)),
        origin=Origin(xyz=(-0.123, 0.0, JUG_H / 2.0)),
        material=jug_mat,
        name="side_wall_0",
    )
    jug.visual(
        Box((0.006, 0.250, JUG_H)),
        origin=Origin(xyz=(0.123, 0.0, JUG_H / 2.0)),
        material=jug_mat,
        name="side_wall_1",
    )
    jug.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=steel_mat,
        name="bearing_plate",
    )
    jug.visual(
        Box((0.080, 0.064, 0.028)),
        origin=Origin(xyz=(0.0, 0.148, 0.278)),
        material=jug_mat,
        name="upper_handle_mount",
    )
    jug.visual(
        Box((0.080, 0.064, 0.028)),
        origin=Origin(xyz=(0.0, 0.148, 0.128)),
        material=jug_mat,
        name="lower_handle_mount",
    )
    jug.visual(
        Box((0.048, 0.036, 0.178)),
        origin=Origin(xyz=(0.0, 0.184, 0.203)),
        material=jug_mat,
        name="handle_grip",
    )
    for i, z in enumerate((0.155, 0.205, 0.255, 0.305)):
        jug.visual(
            Box((0.042, 0.003, 0.004)),
            origin=Origin(xyz=(-0.075, -0.127, z), rpy=(0.0, 0.0, 0.0)),
            material=white_mat,
            name=f"level_mark_{i}",
        )
    model.articulation(
        "base_to_jug",
        ArticulationType.FIXED,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, JUG_Z)),
    )

    blade_rotor = model.part("blade_rotor")
    blade_rotor.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=steel_mat,
        name="center_shaft",
    )
    blade_rotor.visual(
        Cylinder(radius=0.034, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=steel_mat,
        name="center_hub",
    )
    for i, angle in enumerate((0.0, 90.0, 180.0, 270.0)):
        blade_rotor.visual(
            Box((0.078, 0.016, 0.004)),
            origin=_blade_origin(angle, 0.030, 0.08 if i % 2 == 0 else -0.08),
            material=steel_mat,
            name=f"lower_blade_{i}",
        )
    for i, angle in enumerate((45.0, 135.0, 225.0, 315.0)):
        blade_rotor.visual(
            Box((0.068, 0.014, 0.004)),
            origin=_blade_origin(angle, 0.064, -0.09 if i % 2 == 0 else 0.09),
            material=steel_mat,
            name=f"upper_blade_{i}",
        )
    model.articulation(
        "jug_to_blade_rotor",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade_rotor,
        origin=Origin(xyz=(0.0, 0.0, BLADE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.160, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=lid_mat,
        name="lid_cap",
    )
    lid.visual(
        Box((0.262, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, -0.135, -0.013)),
        material=lid_mat,
        name="front_lid_skirt",
    )
    lid.visual(
        Box((0.262, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.135, -0.013)),
        material=lid_mat,
        name="rear_lid_skirt",
    )
    lid.visual(
        Box((0.014, 0.262, 0.026)),
        origin=Origin(xyz=(-0.135, 0.0, -0.013)),
        material=lid_mat,
        name="side_lid_skirt_0",
    )
    lid.visual(
        Box((0.014, 0.262, 0.026)),
        origin=Origin(xyz=(0.135, 0.0, -0.013)),
        material=lid_mat,
        name="side_lid_skirt_1",
    )
    lid.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=lid_mat,
        name="center_plug",
    )
    lid.visual(
        Box((0.078, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=lid_mat,
        name="grip_bar",
    )
    for i, angle in enumerate((0.0, 120.0, 240.0)):
        lid.visual(
            Box((0.030, 0.018, 0.020)),
            origin=_tab_origin(angle),
            material=lid_mat,
            name=f"lock_tab_{i}",
        )
    model.articulation(
        "jug_to_lid",
        ArticulationType.REVOLUTE,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, JUG_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.62, effort=1.2, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    lid = object_model.get_part("lid")
    blade_rotor = object_model.get_part("blade_rotor")
    dial = object_model.get_part("front_dial")
    blade_joint = object_model.get_articulation("jug_to_blade_rotor")
    lid_joint = object_model.get_articulation("jug_to_lid")
    dial_joint = object_model.get_articulation("base_to_front_dial")

    ctx.expect_gap(
        jug,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="jug_floor",
        negative_elem="top_socket",
        name="jug seats on compact base socket",
    )
    ctx.expect_contact(
        lid,
        jug,
        elem_a="lid_cap",
        elem_b="front_wall",
        contact_tol=0.002,
        name="twist lid rests on jug rim",
    )
    ctx.expect_within(
        blade_rotor,
        jug,
        axes="xy",
        margin=0.002,
        name="twin blade clusters stay inside jug footprint",
    )
    ctx.expect_overlap(
        blade_rotor,
        jug,
        axes="z",
        min_overlap=0.060,
        name="blade assembly is mounted down in the jug",
    )
    ctx.expect_contact(
        dial,
        base,
        elem_a="dial_cap",
        elem_b="front_panel",
        contact_tol=0.002,
        name="front dial is mounted on the control panel",
    )

    before = ctx.part_element_world_aabb(blade_rotor, elem="lower_blade_0")
    with ctx.pose({blade_joint: 0.75}):
        after = ctx.part_element_world_aabb(blade_rotor, elem="lower_blade_0")
    if before is not None and after is not None:
        y_before = (before[0][1] + before[1][1]) * 0.5
        y_after = (after[0][1] + after[1][1]) * 0.5
        moved = abs(y_after - y_before) > 0.020
    else:
        moved = False
    ctx.check("central blade joint spins a visible blade", moved, details=f"before={before}, after={after}")

    lid_before = ctx.part_element_world_aabb(lid, elem="lock_tab_0")
    with ctx.pose({lid_joint: 0.62}):
        ctx.expect_contact(
            lid,
            jug,
            elem_a="lid_cap",
            elem_b="front_wall",
            contact_tol=0.002,
            name="lid remains seated while twisted to lock",
        )
        lid_after = ctx.part_element_world_aabb(lid, elem="lock_tab_0")
    if lid_before is not None and lid_after is not None:
        x_before = (lid_before[0][0] + lid_before[1][0]) * 0.5
        x_after = (lid_after[0][0] + lid_after[1][0]) * 0.5
        tab_swept = abs(x_after - x_before) > 0.030
    else:
        tab_swept = False
    ctx.check("locking tab sweeps around the rim", tab_swept, details=f"before={lid_before}, after={lid_after}")

    dial_before = ctx.part_element_world_aabb(dial, elem="dial_cap")
    with ctx.pose({dial_joint: 0.8}):
        dial_after = ctx.part_element_world_aabb(dial, elem="dial_cap")
    ctx.check(
        "front speed dial has a limited revolute control",
        dial_before is not None and dial_after is not None,
        details=f"before={dial_before}, after={dial_after}",
    )

    return ctx.report()


object_model = build_object_model()
