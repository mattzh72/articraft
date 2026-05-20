from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _armored_hull_mesh():
    """A compact tapered upper housing with flat armored side skins."""
    hull = (
        cq.Workplane("XY")
        .rect(0.74, 0.56)
        .workplane(offset=0.36)
        .rect(0.62, 0.44)
        .loft()
    )
    return mesh_from_cadquery(hull, "tapered_upper_hull", tolerance=0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_tilt_weapon_station")

    dark_olive = model.material("dark_olive_armor", rgba=(0.18, 0.21, 0.16, 1.0))
    black = model.material("matte_black_hardware", rgba=(0.015, 0.016, 0.014, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.06, 0.065, 0.06, 1.0))
    tan = model.material("worn_service_panel", rgba=(0.30, 0.34, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    glass = model.material("smoked_glass", rgba=(0.04, 0.08, 0.08, 0.72))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=graphite,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.23, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=dark_olive,
        name="tapered_column",
    )
    pedestal.visual(
        Cylinder(radius=0.29, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.4175)),
        material=black,
        name="slew_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=graphite,
        name="bearing_spigot",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        pedestal.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(
                xyz=(0.295 * math.cos(angle), 0.295 * math.sin(angle), 0.086),
            ),
            material=black,
            name=f"base_bolt_{i}",
        )

    upper_body = model.part("upper_body")
    upper_body.visual(
        Cylinder(radius=0.255, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black,
        name="rotating_bearing_plate",
    )
    upper_body.visual(
        Cylinder(radius=0.18, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=graphite,
        name="lower_neck",
    )
    upper_body.visual(
        _armored_hull_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_olive,
        name="armored_hull",
    )
    upper_body.visual(
        Box((0.26, 0.055, 0.13)),
        origin=Origin(xyz=(0.0, 0.277, 0.255)),
        material=graphite,
        name="front_equipment_bay",
    )
    upper_body.visual(
        Box((0.20, 0.028, 0.075)),
        origin=Origin(xyz=(0.0, 0.318, 0.268)),
        material=glass,
        name="front_sensor_window",
    )
    upper_body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.20, 0.12),
                0.006,
                slot_size=(0.060, 0.010),
                pitch=(0.075, 0.028),
                frame=0.018,
                corner_radius=0.006,
            ),
            "rear_cooling_slots",
        ),
        origin=Origin(xyz=(0.0, -0.255, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rear_cooling_slots",
    )
    upper_body.visual(
        Box((0.043, 0.030, 0.25)),
        origin=Origin(xyz=(0.3605, -0.160, 0.280)),
        material=black,
        name="fixed_hinge_leaf",
    )
    upper_body.visual(
        Cylinder(radius=0.010, length=0.245),
        origin=Origin(xyz=(0.392, -0.166, 0.280)),
        material=black,
        name="service_hinge_pin",
    )
    upper_body.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.44, 0.22, 0.255),
                span_width=0.275,
                trunnion_diameter=0.068,
                trunnion_center_z=0.168,
                base_thickness=0.052,
                corner_radius=0.014,
                center=False,
            ),
            "short_elevation_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.235, 0.440)),
        material=dark_olive,
        name="elevation_yoke",
    )

    weapon_mount = model.part("weapon_mount")
    weapon_mount.visual(
        Cylinder(radius=0.034, length=0.275),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="trunnion_axle",
    )
    weapon_mount.visual(
        Box((0.225, 0.31, 0.115)),
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        material=graphite,
        name="elevation_cradle",
    )
    weapon_mount.visual(
        Box((0.18, 0.16, 0.13)),
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        material=dark_olive,
        name="rear_counterweight",
    )
    weapon_mount.visual(
        Cylinder(radius=0.050, length=0.60),
        origin=Origin(xyz=(0.0, 0.405, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="weapon_tube",
    )
    weapon_mount.visual(
        Cylinder(radius=0.062, length=0.13),
        origin=Origin(xyz=(0.0, 0.185, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_olive,
        name="recoil_sleeve",
    )
    weapon_mount.visual(
        Cylinder(radius=0.068, length=0.055),
        origin=Origin(xyz=(0.0, 0.720, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="muzzle_collar",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.018, 0.310, 0.215)),
        origin=Origin(xyz=(0.0, 0.164, 0.0)),
        material=tan,
        name="panel_plate",
    )
    service_panel.visual(
        Box((0.024, 0.270, 0.170)),
        origin=Origin(xyz=(0.012, 0.164, 0.0)),
        material=dark_olive,
        name="raised_inner_panel",
    )
    service_panel.visual(
        Box((0.012, 0.030, 0.010)),
        origin=Origin(xyz=(0.026, 0.290, 0.0)),
        material=black,
        name="flush_latch",
    )

    model.articulation(
        "pedestal_to_body",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "body_to_weapon",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=weapon_mount,
        origin=Origin(xyz=(0.0, 0.235, 0.608)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.0,
            lower=math.radians(-18.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "body_to_panel",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=service_panel,
        origin=Origin(xyz=(0.391, -0.160, 0.280)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_body = object_model.get_part("upper_body")
    weapon_mount = object_model.get_part("weapon_mount")
    service_panel = object_model.get_part("service_panel")
    pan = object_model.get_articulation("pedestal_to_body")
    pitch = object_model.get_articulation("body_to_weapon")
    panel_hinge = object_model.get_articulation("body_to_panel")

    ctx.allow_overlap(
        upper_body,
        weapon_mount,
        elem_a="elevation_yoke",
        elem_b="trunnion_axle",
        reason=(
            "The trunnion axle is intentionally modeled as a captured shaft "
            "inside the short elevation yoke bore proxy."
        ),
    )

    ctx.expect_gap(
        upper_body,
        pedestal,
        axis="z",
        positive_elem="rotating_bearing_plate",
        negative_elem="bearing_spigot",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating body sits on the pedestal bearing",
    )
    ctx.expect_overlap(
        upper_body,
        pedestal,
        axes="xy",
        elem_a="rotating_bearing_plate",
        elem_b="slew_bearing",
        min_overlap=0.20,
        name="slew bearing supports the rotating upper body",
    )
    ctx.expect_within(
        weapon_mount,
        upper_body,
        axes="x",
        inner_elem="elevation_cradle",
        outer_elem="elevation_yoke",
        margin=0.002,
        name="weapon cradle fits between yoke cheeks",
    )
    ctx.expect_overlap(
        weapon_mount,
        upper_body,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="elevation_yoke",
        min_overlap=0.25,
        name="trunnion axle spans the elevation yoke",
    )
    ctx.expect_within(
        weapon_mount,
        upper_body,
        axes="yz",
        inner_elem="trunnion_axle",
        outer_elem="elevation_yoke",
        margin=0.002,
        name="trunnion axle is retained in the yoke bore envelope",
    )

    rest_muzzle = ctx.part_element_world_aabb(weapon_mount, elem="muzzle_collar")
    with ctx.pose({pitch: math.radians(45.0)}):
        raised_muzzle = ctx.part_element_world_aabb(weapon_mount, elem="muzzle_collar")
    ctx.check(
        "weapon mount pitches upward",
        rest_muzzle is not None
        and raised_muzzle is not None
        and raised_muzzle[0][2] > rest_muzzle[0][2] + 0.20,
        details=f"rest={rest_muzzle}, raised={raised_muzzle}",
    )

    rest_sensor = ctx.part_element_world_aabb(upper_body, elem="front_sensor_window")
    with ctx.pose({pan: math.pi / 2.0}):
        panned_sensor = ctx.part_element_world_aabb(upper_body, elem="front_sensor_window")
    ctx.check(
        "upper body pans about the vertical pedestal axis",
        rest_sensor is not None
        and panned_sensor is not None
        and panned_sensor[0][0] < rest_sensor[0][0] - 0.20,
        details=f"rest={rest_sensor}, panned={panned_sensor}",
    )

    closed_panel = ctx.part_element_world_aabb(service_panel, elem="flush_latch")
    with ctx.pose({panel_hinge: 1.2}):
        open_panel = ctx.part_element_world_aabb(service_panel, elem="flush_latch")
    ctx.check(
        "side service panel swings outward on its hinge",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][0] > closed_panel[1][0] + 0.10,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


object_model = build_object_model()
