from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(cx: float, cz: float, radius: float, segments: int = 32):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cz + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _hole(loop):
    # The mesh extruder expects inner loops wound opposite the outer profile.
    return list(reversed(loop))


def _stadium_profile(length: float, height: float, segments: int = 14):
    radius = height * 0.5
    pts = []
    for i in range(segments + 1):
        a = -math.pi * 0.5 + math.pi * i / segments
        pts.append((length + radius * math.cos(a), radius * math.sin(a)))
    for i in range(segments + 1):
        a = math.pi * 0.5 + math.pi * i / segments
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _translated(profile, dx: float, dz: float):
    return [(x + dx, z + dz) for x, z in profile]


def _side_plate_mesh(
    length: float,
    height: float,
    thickness: float,
    hole_radius: float,
    name: str,
    *,
    slot_count: int,
):
    holes = [_hole(_circle_profile(0.0, 0.0, hole_radius)), _hole(_circle_profile(length, 0.0, hole_radius))]
    if slot_count:
        usable = max(0.04, length - 0.17)
        slot_len = min(0.115, usable / slot_count * 0.66)
        slot_h = height * 0.30
        start = 0.085
        step = usable / slot_count
        for i in range(slot_count):
            cx = start + step * (i + 0.5)
            holes.append(_hole(_translated(_stadium_profile(slot_len, slot_h, segments=8), cx - slot_len * 0.5, 0.0)))

    geom = ExtrudeWithHolesGeometry(
        _stadium_profile(length, height, segments=18),
        holes,
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _root_cheek_mesh(thickness: float, name: str):
    outer = _translated(rounded_rect_profile(0.120, 0.165, 0.020, corner_segments=8), 0.0, -0.040)
    holes = [_hole(_circle_profile(0.0, 0.0, 0.014))]
    geom = ExtrudeWithHolesGeometry(outer, holes, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _cyl_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _add_cylinder_y(part, radius: float, length: float, xyz, material, name: str):
    part.visual(
        _cyl_y(radius, length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_link(
    part,
    *,
    length: float,
    height: float,
    thickness: float,
    y_offset: float,
    hole_radius: float,
    plate_material: Material,
    edge_material: Material,
    hardware_material: Material,
    prefix: str,
    slot_count: int,
):
    plate_mesh = _side_plate_mesh(length, height, thickness, hole_radius, f"{prefix}_plate_mesh", slot_count=slot_count)
    for side, y, plate_name in (("near", -y_offset, "near_plate"), ("far", y_offset, "far_plate")):
        sign = 1.0 if y > 0.0 else -1.0
        part.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=plate_material,
            name=plate_name,
        )
        # Folded edges and a shallow pressed rib make each link read as formed
        # steel plate rather than a featureless flat bar.
        part.visual(
            Box((length * 0.82, 0.014, 0.016)),
            origin=Origin(xyz=(length * 0.50, y, height * 0.50)),
            material=edge_material,
            name=f"{side}_upper_lip",
        )
        part.visual(
            Box((length * 0.82, 0.014, 0.016)),
            origin=Origin(xyz=(length * 0.50, y, -height * 0.50)),
            material=edge_material,
            name=f"{side}_lower_lip",
        )
        part.visual(
            Box((length * 0.52, 0.004, height * 0.16)),
            origin=Origin(xyz=(length * 0.50, y + sign * (thickness * 0.5 + 0.0015), 0.0)),
            material=edge_material,
            name=f"{side}_pressed_rib",
        )

    spacer_len = 2.0 * y_offset + thickness + 0.004
    for x, label in ((length * 0.34, "inner_spacer"), (length * 0.66, "outer_spacer")):
        _add_cylinder_y(part, height * 0.18, spacer_len, (x, 0.0, 0.0), hardware_material, label)

    # Shoulder bolt and paired washers at the proximal pivot.  The shaft is
    # slimmer than every bore in the neighboring clevis/link, while the washers
    # overlap this link's plates just enough to read as captured hardware.
    shaft_len = 2.0 * y_offset + thickness + 0.055
    _add_cylinder_y(part, hole_radius * 0.64, shaft_len, (0.0, 0.0, 0.0), hardware_material, "shoulder_bolt")
    washer_y = y_offset + thickness * 0.5 + 0.002
    washer_len = 0.006
    _add_cylinder_y(part, hole_radius * 1.35, washer_len, (0.0, washer_y, 0.0), hardware_material, "far_washer")
    _add_cylinder_y(part, hole_radius * 1.35, washer_len, (0.0, -washer_y, 0.0), hardware_material, "near_washer")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_bracket_arm")

    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.055, 0.065, 0.070, 1.0))
    plate_steel = model.material("charcoal_formed_plate", rgba=(0.095, 0.105, 0.105, 1.0))
    worn_edges = model.material("worn_formed_edges", rgba=(0.145, 0.155, 0.150, 1.0))
    zinc = model.material("zinc_plated_hardware", rgba=(0.62, 0.60, 0.54, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    latch_orange = model.material("root_latch_orange", rgba=(0.95, 0.36, 0.08, 1.0))

    pivot_z = 0.180
    theta_0 = math.radians(26.0)
    theta_1 = math.radians(-18.0)
    theta_2 = math.radians(18.0)
    theta_tray = math.radians(-5.0)

    foot = model.part("foot")
    foot.visual(
        Box((0.360, 0.245, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, 0.014)),
        material=dark_steel,
        name="ground_plate",
    )
    foot.visual(
        Box((0.150, 0.170, 0.076)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=dark_steel,
        name="root_plinth",
    )
    foot.visual(
        Box((0.135, 0.175, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=plate_steel,
        name="clevis_bridge",
    )
    cheek_mesh = _root_cheek_mesh(0.012, "root_cheek")
    for side, y, cheek_name in (("near", -0.075, "near_clevis_cheek"), ("far", 0.075, "far_clevis_cheek")):
        foot.visual(
            cheek_mesh,
            origin=Origin(xyz=(0.0, y, pivot_z)),
            material=plate_steel,
            name=cheek_name,
        )
        foot.visual(
            Box((0.020, 0.006, 0.095)),
            origin=Origin(xyz=(-0.058, y, 0.116)),
            material=worn_edges,
            name=f"{side}_rear_formed_edge",
        )
        foot.visual(
            Box((0.020, 0.006, 0.095)),
            origin=Origin(xyz=(0.058, y, 0.116)),
            material=worn_edges,
            name=f"{side}_front_formed_edge",
        )

    # Rubber feet slightly embed into the base plate so they read as mounted
    # pads rather than loose blocks.
    for ix, x in enumerate((-0.160, 0.075)):
        for iy, y in enumerate((-0.090, 0.090)):
            foot.visual(
                Box((0.052, 0.040, 0.012)),
                origin=Origin(xyz=(x, y, 0.001)),
                material=black_rubber,
                name=f"rubber_pad_{ix}_{iy}",
            )

    # Compact root latch/stop details, attached to the near cheek and plinth.
    _add_cylinder_y(foot, 0.010, 0.018, (0.030, -0.090, 0.128), zinc, "latch_pivot")
    foot.visual(
        Box((0.074, 0.009, 0.014)),
        origin=Origin(xyz=(0.068, -0.098, 0.129), rpy=(0.0, math.radians(-8.0), 0.0)),
        material=latch_orange,
        name="latch_pawl",
    )
    foot.visual(
        Box((0.030, 0.014, 0.026)),
        origin=Origin(xyz=(0.066, -0.078, 0.102)),
        material=zinc,
        name="latch_stop",
    )
    _add_cylinder_y(foot, 0.007, 0.190, (-0.108, 0.0, 0.026), zinc, "base_cross_bolt")

    long_link = model.part("long_link_0")
    _add_link(
        long_link,
        length=0.420,
        height=0.062,
        thickness=0.008,
        y_offset=0.052,
        hole_radius=0.014,
        plate_material=plate_steel,
        edge_material=worn_edges,
        hardware_material=zinc,
        prefix="long0",
        slot_count=2,
    )

    short_link = model.part("short_link_0")
    _add_link(
        short_link,
        length=0.240,
        height=0.052,
        thickness=0.008,
        y_offset=0.030,
        hole_radius=0.0125,
        plate_material=plate_steel,
        edge_material=worn_edges,
        hardware_material=zinc,
        prefix="short0",
        slot_count=1,
    )

    light_link = model.part("long_link_1")
    _add_link(
        light_link,
        length=0.340,
        height=0.047,
        thickness=0.007,
        y_offset=0.046,
        hole_radius=0.012,
        plate_material=plate_steel,
        edge_material=worn_edges,
        hardware_material=zinc,
        prefix="long1",
        slot_count=2,
    )

    tray = model.part("tray_bracket")
    # A light tray-like end bracket: shallow pan, side lips, pivot ears, and a
    # small clamp tab.  The bottom sits below the wrist axis, as on a real end
    # fixture carried by a side-plate linkage.
    tray.visual(
        Box((0.155, 0.110, 0.008)),
        origin=Origin(xyz=(0.086, 0.0, -0.037)),
        material=plate_steel,
        name="tray_floor",
    )
    tray.visual(
        Box((0.142, 0.008, 0.032)),
        origin=Origin(xyz=(0.092, 0.057, -0.019)),
        material=worn_edges,
        name="far_tray_lip",
    )
    tray.visual(
        Box((0.142, 0.008, 0.032)),
        origin=Origin(xyz=(0.092, -0.057, -0.019)),
        material=worn_edges,
        name="near_tray_lip",
    )
    tray.visual(
        Box((0.012, 0.112, 0.032)),
        origin=Origin(xyz=(0.159, 0.0, -0.019)),
        material=worn_edges,
        name="tray_front_lip",
    )
    tray.visual(
        Box((0.052, 0.078, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, -0.033)),
        material=plate_steel,
        name="wrist_bridge",
    )
    for side, y in (("near", -0.027), ("far", 0.027)):
        tray.visual(
            Box((0.060, 0.010, 0.056)),
            origin=Origin(xyz=(0.000, y, -0.002)),
            material=plate_steel,
            name=f"{side}_wrist_ear",
        )
    _add_cylinder_y(tray, 0.008, 0.128, (0.0, 0.0, 0.0), zinc, "wrist_shoulder_bolt")
    _add_cylinder_y(tray, 0.017, 0.006, (0.0, 0.035, 0.0), zinc, "far_wrist_washer")
    _add_cylinder_y(tray, 0.017, 0.006, (0.0, -0.035, 0.0), zinc, "near_wrist_washer")
    tray.visual(
        Box((0.044, 0.006, 0.018)),
        origin=Origin(xyz=(0.104, -0.061, -0.002)),
        material=latch_orange,
        name="tray_spring_clip",
    )

    model.articulation(
        "foot_to_long",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=long_link,
        origin=Origin(xyz=(0.0, 0.0, pivot_z), rpy=(0.0, -theta_0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-0.32, upper=0.40),
    )
    model.articulation(
        "long_to_short",
        ArticulationType.REVOLUTE,
        parent=long_link,
        child=short_link,
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(0.0, theta_0 - theta_1, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=-0.38, upper=0.42),
    )
    model.articulation(
        "short_to_long",
        ArticulationType.REVOLUTE,
        parent=short_link,
        child=light_link,
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, theta_1 - theta_2, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.4, lower=-0.36, upper=0.44),
    )
    model.articulation(
        "long_to_tray",
        ArticulationType.REVOLUTE,
        parent=light_link,
        child=tray,
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(0.0, theta_2 - theta_tray, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.55, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    long0 = object_model.get_part("long_link_0")
    short0 = object_model.get_part("short_link_0")
    long1 = object_model.get_part("long_link_1")
    tray = object_model.get_part("tray_bracket")

    foot_joint = object_model.get_articulation("foot_to_long")
    elbow_joint = object_model.get_articulation("long_to_short")
    knuckle_joint = object_model.get_articulation("short_to_long")
    wrist_joint = object_model.get_articulation("long_to_tray")

    joints = [foot_joint, elbow_joint, knuckle_joint, wrist_joint]
    ctx.check(
        "four parallel revolute joints",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in (j.axis or ())) == (0.0, 1.0, 0.0) for j in joints),
        details=f"joints={[j.name for j in joints]} axes={[j.axis for j in joints]}",
    )
    ctx.check(
        "long short long hierarchy",
        long0.get_visual("near_plate") is not None
        and short0.get_visual("near_plate") is not None
        and long1.get_visual("near_plate") is not None,
        details="alternating side-plate link visuals are present",
    )

    # The reported part-pair intersections are intentional captured shoulder
    # bolts through bored clevis/link plates.  They are scoped to the bolt/plate
    # element pairs and paired with projection checks proving the pin is centered
    # and retained through the local side-plate thickness.
    pin_allowances = [
        (foot, long0, "near_clevis_cheek", "shoulder_bolt", "root shoulder bolt runs through the grounded clevis bore"),
        (foot, long0, "far_clevis_cheek", "shoulder_bolt", "root shoulder bolt runs through the grounded clevis bore"),
        (long0, short0, "near_plate", "shoulder_bolt", "elbow shoulder bolt is captured by the long link bore"),
        (long0, short0, "far_plate", "shoulder_bolt", "elbow shoulder bolt is captured by the long link bore"),
        (long1, short0, "shoulder_bolt", "near_plate", "knuckle shoulder bolt is captured by the short link bore"),
        (long1, short0, "shoulder_bolt", "far_plate", "knuckle shoulder bolt is captured by the short link bore"),
        (long1, tray, "near_plate", "wrist_shoulder_bolt", "wrist shoulder bolt is captured by the final long link bore"),
        (long1, tray, "far_plate", "wrist_shoulder_bolt", "wrist shoulder bolt is captured by the final long link bore"),
    ]
    for part_a, part_b, elem_a, elem_b, reason in pin_allowances:
        ctx.allow_overlap(part_a, part_b, elem_a=elem_a, elem_b=elem_b, reason=reason)

    ctx.expect_within(long0, foot, axes="xz", inner_elem="shoulder_bolt", outer_elem="near_clevis_cheek", margin=0.006, name="root pin stays inside clevis cheek profile")
    ctx.expect_overlap(long0, foot, axes="y", elem_a="shoulder_bolt", elem_b="near_clevis_cheek", min_overlap=0.006, name="root pin spans clevis cheek thickness")
    ctx.expect_within(short0, long0, axes="xz", inner_elem="shoulder_bolt", outer_elem="near_plate", margin=0.006, name="elbow pin stays inside long link bore region")
    ctx.expect_overlap(short0, long0, axes="y", elem_a="shoulder_bolt", elem_b="near_plate", min_overlap=0.006, name="elbow pin is retained through side plate")
    ctx.expect_within(long1, short0, axes="xz", inner_elem="shoulder_bolt", outer_elem="near_plate", margin=0.006, name="knuckle pin stays inside short link bore region")
    ctx.expect_overlap(long1, short0, axes="y", elem_a="shoulder_bolt", elem_b="near_plate", min_overlap=0.006, name="knuckle pin is retained through side plate")
    ctx.expect_within(tray, long1, axes="xz", inner_elem="wrist_shoulder_bolt", outer_elem="near_plate", margin=0.006, name="wrist pin stays inside final link bore region")
    ctx.expect_overlap(tray, long1, axes="y", elem_a="wrist_shoulder_bolt", elem_b="near_plate", min_overlap=0.006, name="wrist pin is retained through side plate")

    ctx.expect_gap(short0, foot, axis="z", min_gap=0.035, name="short link clears grounded foot")
    ctx.expect_gap(tray, foot, axis="z", min_gap=0.100, name="tray stays above root hardware in deployed pose")

    rest_tray = ctx.part_world_position(tray)
    with ctx.pose({foot_joint: -0.18, elbow_joint: 0.14, knuckle_joint: -0.10, wrist_joint: 0.18}):
        raised_tray = ctx.part_world_position(tray)
        ctx.expect_gap(tray, foot, axis="z", min_gap=0.100, name="tray clears root through raised articulation sample")
    ctx.check(
        "articulated tip moves with planar joints",
        rest_tray is not None and raised_tray is not None and raised_tray[2] > rest_tray[2] + 0.020,
        details=f"rest_tray={rest_tray}, raised_tray={raised_tray}",
    )

    return ctx.report()


object_model = build_object_model()
