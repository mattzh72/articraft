from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

FOOT_HEIGHT = 0.014
BASE_THICKNESS = 0.020
PEDESTAL_HEIGHT = 0.064
DECK_THICKNESS = 0.012
YAW_HOUSING_HEIGHT = 0.016

BASE_JOINT_Z = FOOT_HEIGHT + BASE_THICKNESS + PEDESTAL_HEIGHT + DECK_THICKNESS + YAW_HOUSING_HEIGHT

PLATE_THICKNESS = 0.010
HUB_LENGTH = 0.052
HUB_RADIUS = 0.024
SMALL_HUB_RADIUS = 0.022
JOINT_CLEARANCE = 0.0015
BOSS_CAP_THICKNESS = 0.0025
INNER_JOINT_SPAN = HUB_LENGTH + 2.0 * JOINT_CLEARANCE
OUTER_JOINT_SPAN = INNER_JOINT_SPAN + 2.0 * PLATE_THICKNESS
PLATE_OFFSET_Y = INNER_JOINT_SPAN / 2.0 + PLATE_THICKNESS / 2.0

SHOULDER_ORIGIN = (0.058, 0.0, 0.168)
ELBOW_X = 0.182
WRIST_X = 0.145


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _box_tube(length: float, width: float, height: float, wall: float, cap: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, width, height, centered=(False, True, True))
    inner = (
        cq.Workplane("XY")
        .box(length - 2.0 * cap, width - 2.0 * wall, height - 2.0 * wall, centered=(False, True, True))
        .translate((cap, 0.0, 0.0))
    )
    return outer.cut(inner)


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((0.0, length / 2.0, 0.0))
    )


def _y_annulus(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, length / 2.0, 0.0))
    )


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((-length / 2.0, 0.0, 0.0))


def _bolt_heads_xy(points: list[tuple[float, float]], radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").pushPoints(points).circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _bolt_heads_xz(points: list[tuple[float, float]], radius: float, height: float, y0: float) -> cq.Workplane:
    return cq.Workplane("XZ").pushPoints(points).circle(radius).extrude(height).translate((0.0, y0, 0.0))


def _bolt_heads_yz(points: list[tuple[float, float]], radius: float, height: float, x0: float) -> cq.Workplane:
    return cq.Workplane("YZ").pushPoints(points).circle(radius).extrude(height).translate((x0, 0.0, 0.0))


def _joint_bosses(x_pos: float, z_pos: float, radius: float, length: float) -> cq.Workplane:
    outer_face = PLATE_OFFSET_Y + PLATE_THICKNESS / 2.0
    cap_radius = radius + 0.003
    positive = (
        cq.Workplane("XZ")
        .pushPoints([(x_pos, z_pos)])
        .circle(radius)
        .extrude(length)
        .translate((0.0, outer_face + length, 0.0))
    )
    negative = (
        cq.Workplane("XZ")
        .pushPoints([(x_pos, z_pos)])
        .circle(radius)
        .extrude(length)
        .translate((0.0, -outer_face, 0.0))
    )
    positive_cap = (
        cq.Workplane("XZ")
        .pushPoints([(x_pos, z_pos)])
        .circle(cap_radius)
        .extrude(BOSS_CAP_THICKNESS)
        .translate((0.0, outer_face + length + BOSS_CAP_THICKNESS, 0.0))
    )
    negative_cap = (
        cq.Workplane("XZ")
        .pushPoints([(x_pos, z_pos)])
        .circle(cap_radius)
        .extrude(BOSS_CAP_THICKNESS)
        .translate((0.0, -outer_face - length, 0.0))
    )
    return _union_all(positive, negative, positive_cap, negative_cap)


def _joint_clearance_cut(x_pos: float, z_pos: float, radius: float) -> cq.Workplane:
    return _y_cylinder(radius, OUTER_JOINT_SPAN + 2.0 * BOSS_CAP_THICKNESS).translate((x_pos, 0.0, z_pos))


def _retained_hub(
    hub_radius: float,
    hub_length: float,
    shaft_radius: float,
    collar_radius: float,
    collar_thickness: float = 0.0015,
) -> cq.Workplane:
    outboard_face = PLATE_OFFSET_Y + (PLATE_THICKNESS / 2.0) + 0.006 + BOSS_CAP_THICKNESS
    shaft = _y_cylinder(shaft_radius, 2.0 * (outboard_face + collar_thickness))
    drum = _y_cylinder(hub_radius, hub_length)
    positive_collar = _y_annulus(collar_radius, shaft_radius, collar_thickness).translate(
        (0.0, outboard_face + collar_thickness / 2.0, 0.0)
    )
    negative_collar = _y_annulus(collar_radius, shaft_radius, collar_thickness).translate(
        (0.0, -outboard_face - collar_thickness / 2.0, 0.0)
    )
    return _union_all(shaft, drum, positive_collar, negative_collar)


def _build_base_shapes() -> dict[str, cq.Workplane]:
    foot_points = [(-0.125, -0.075), (-0.125, 0.075), (0.125, -0.075), (0.125, 0.075)]

    feet = cq.Workplane("XY").pushPoints(foot_points).rect(0.050, 0.042).extrude(FOOT_HEIGHT)
    base_plate = cq.Workplane("XY").box(0.340, 0.220, BASE_THICKNESS).translate((0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS / 2.0))
    pedestal = cq.Workplane("XY").box(0.180, 0.120, PEDESTAL_HEIGHT).translate((0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0))
    deck = cq.Workplane("XY").box(0.120, 0.080, DECK_THICKNESS).translate((0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS + PEDESTAL_HEIGHT + DECK_THICKNESS / 2.0))
    datum_pad_pos = cq.Workplane("XY").box(0.006, 0.040, 0.028).translate((0.093, 0.0, 0.076))
    datum_pad_neg = cq.Workplane("XY").box(0.006, 0.040, 0.028).translate((-0.093, 0.0, 0.076))

    front_pocket = cq.Workplane("XY").box(0.110, 0.048, 0.030).translate((0.0, -0.036, 0.066))
    rear_pocket = cq.Workplane("XY").box(0.110, 0.048, 0.030).translate((0.0, 0.036, 0.066))
    mounting_holes = cq.Workplane("XY").pushPoints(foot_points).circle(0.0065).extrude(0.060)

    body = _union_all(feet, base_plate, pedestal, deck, datum_pad_pos, datum_pad_neg)
    body = body.cut(front_pocket).cut(rear_pocket).cut(mounting_holes)

    yaw_housing = _union_all(
        cq.Workplane("XY").circle(0.056).extrude(0.010),
        cq.Workplane("XY").circle(0.048).extrude(YAW_HOUSING_HEIGHT),
    ).translate((0.0, 0.0, BASE_JOINT_Z - YAW_HOUSING_HEIGHT))

    front_cover = cq.Workplane("XY").box(0.090, 0.004, 0.045).translate((0.0, -0.062, 0.068))
    front_cover_bolts = _bolt_heads_xz(
        [(-0.030, 0.084), (0.030, 0.084), (-0.030, 0.052), (0.030, 0.052)],
        radius=0.003,
        height=0.003,
        y0=-0.067,
    )
    hardware = front_cover.union(front_cover_bolts)
    return {"base_body": body, "yaw_housing": yaw_housing, "base_hardware": hardware}


def _build_turret_shapes() -> dict[str, cq.Workplane]:
    base_flange = _union_all(
        cq.Workplane("XY").box(0.100, 0.080, 0.010).translate((0.0, 0.0, 0.005)),
        cq.Workplane("XY").circle(0.052).extrude(0.012),
    )

    column = cq.Workplane("XY").box(0.078, 0.055, 0.130).translate((0.0, 0.0, 0.077))
    column_window = cq.Workplane("XY").box(0.042, 0.030, 0.052).translate((0.0, 0.0, 0.088))
    lower_brace = cq.Workplane("XY").box(0.050, 0.055, 0.032).translate((0.018, 0.0, 0.132))
    shoulder_pedestal = cq.Workplane("XY").box(0.026, 0.055, 0.024).translate((0.042, 0.0, 0.150))
    spine_pad = cq.Workplane("XY").box(0.036, 0.006, 0.024).translate((-0.010, -0.030, 0.086))
    shoulder_relief = _y_cylinder(HUB_RADIUS + 0.008, 0.070).translate(SHOULDER_ORIGIN)
    shoulder_throat = cq.Workplane("XY").box(0.052, 0.060, 0.070).translate((SHOULDER_ORIGIN[0] - 0.002, 0.0, SHOULDER_ORIGIN[2] + 0.012))
    body = _union_all(column, lower_brace, shoulder_pedestal, spine_pad).cut(column_window).cut(shoulder_relief).cut(shoulder_throat)

    clevis_web = cq.Workplane("XY").box(0.040, INNER_JOINT_SPAN, 0.036).translate((0.035, 0.0, 0.150))
    plate_pos = cq.Workplane("XY").box(0.036, PLATE_THICKNESS, 0.082).translate((SHOULDER_ORIGIN[0], PLATE_OFFSET_Y, SHOULDER_ORIGIN[2]))
    plate_neg = cq.Workplane("XY").box(0.036, PLATE_THICKNESS, 0.082).translate((SHOULDER_ORIGIN[0], -PLATE_OFFSET_Y, SHOULDER_ORIGIN[2]))
    shoulder_clevis = _union_all(clevis_web, plate_pos, plate_neg).cut(
        _joint_clearance_cut(SHOULDER_ORIGIN[0], SHOULDER_ORIGIN[2], HUB_RADIUS + JOINT_CLEARANCE)
    )

    rear_cover = cq.Workplane("XY").box(0.050, 0.004, 0.060).translate((-0.005, 0.0295, 0.086))
    rear_cover_bolts = _bolt_heads_xz(
        [(-0.016, 0.106), (0.016, 0.106), (-0.016, 0.066), (0.016, 0.066)],
        radius=0.003,
        height=0.003,
        y0=0.0315,
    )
    shoulder_bosses = _joint_bosses(SHOULDER_ORIGIN[0], SHOULDER_ORIGIN[2], radius=0.018, length=0.006)
    hardware = _union_all(rear_cover, rear_cover_bolts, shoulder_bosses)
    return {
        "base_flange": base_flange,
        "turret_body": body,
        "shoulder_clevis": shoulder_clevis,
        "turret_hardware": hardware,
    }


def _build_upper_arm_shapes() -> dict[str, cq.Workplane]:
    journal_core = _y_cylinder(HUB_RADIUS, HUB_LENGTH)
    root_hub = _retained_hub(hub_radius=HUB_RADIUS, hub_length=HUB_LENGTH, shaft_radius=0.010, collar_radius=0.026)

    root_block = cq.Workplane("XY").box(0.034, 0.042, 0.024, centered=(False, True, True)).translate((0.004, 0.0, -0.016))
    transition_block = cq.Workplane("XY").box(0.028, 0.042, 0.020, centered=(False, True, True)).translate((0.024, 0.0, -0.017))
    beam = _box_tube(0.102, 0.050, 0.044, wall=0.0055, cap=0.010).translate((0.050, 0.0, -0.024))
    datum_pad = cq.Workplane("XY").box(0.046, 0.006, 0.016).translate((0.098, 0.028, -0.012))
    lower_datum = cq.Workplane("XY").box(0.040, 0.008, 0.010).translate((0.084, -0.026, -0.039))
    pocket = cq.Workplane("XY").box(0.066, 0.034, 0.003).translate((0.098, 0.0, -0.003))
    arm_body = _union_all(root_block, transition_block, beam, datum_pad, lower_datum).cut(pocket)

    clevis_web = cq.Workplane("XY").box(0.030, INNER_JOINT_SPAN, 0.046).translate((0.158, 0.0, 0.0))
    plate_pos = cq.Workplane("XY").box(0.034, PLATE_THICKNESS, 0.072).translate((ELBOW_X, PLATE_OFFSET_Y, 0.0))
    plate_neg = cq.Workplane("XY").box(0.034, PLATE_THICKNESS, 0.072).translate((ELBOW_X, -PLATE_OFFSET_Y, 0.0))
    elbow_clevis = _union_all(clevis_web, plate_pos, plate_neg).cut(
        _joint_clearance_cut(ELBOW_X, 0.0, HUB_RADIUS + JOINT_CLEARANCE)
    )

    cover = cq.Workplane("XY").box(0.070, 0.040, 0.004).translate((0.098, 0.0, -0.0005))
    cover_bolts = _bolt_heads_xy(
        [(0.070, -0.014), (0.070, 0.014), (0.126, -0.014), (0.126, 0.014)],
        radius=0.0028,
        height=0.003,
        z0=0.0015,
    )
    elbow_bosses = _joint_bosses(ELBOW_X, 0.0, radius=0.016, length=0.006)
    hardware = _union_all(cover, cover_bolts, elbow_bosses)
    return {
        "root_hub": root_hub,
        "journal_core": journal_core,
        "arm_body": arm_body,
        "elbow_clevis": elbow_clevis,
        "arm_hardware": hardware,
    }


def _build_forearm_shapes() -> dict[str, cq.Workplane]:
    journal_core = _y_cylinder(HUB_RADIUS, HUB_LENGTH)
    root_hub = _retained_hub(hub_radius=HUB_RADIUS, hub_length=HUB_LENGTH, shaft_radius=0.010, collar_radius=0.026)

    root_block = cq.Workplane("XY").box(0.030, 0.040, 0.022, centered=(False, True, True)).translate((0.002, 0.0, -0.015))
    transition_block = cq.Workplane("XY").box(0.028, 0.040, 0.018, centered=(False, True, True)).translate((0.022, 0.0, -0.016))
    beam = _box_tube(0.080, 0.046, 0.038, wall=0.0055, cap=0.009).translate((0.048, 0.0, -0.020))
    datum_rail = cq.Workplane("XY").box(0.054, 0.006, 0.014).translate((0.090, 0.026, -0.011))
    lower_wear_pad = cq.Workplane("XY").box(0.044, 0.008, 0.010).translate((0.080, -0.023, -0.034))
    side_pocket = cq.Workplane("XY").box(0.054, 0.003, 0.026).translate((0.088, -0.0215, -0.014))
    wrist_relief = _y_cylinder(SMALL_HUB_RADIUS + 0.008, 0.074).translate((WRIST_X - 0.014, 0.0, 0.0))
    arm_body = _union_all(root_block, transition_block, beam, datum_rail, lower_wear_pad).cut(side_pocket).cut(wrist_relief)

    clevis_web = cq.Workplane("XY").box(0.028, INNER_JOINT_SPAN, 0.040).translate((0.124, 0.0, 0.0))
    plate_pos = cq.Workplane("XY").box(0.030, PLATE_THICKNESS, 0.066).translate((WRIST_X, PLATE_OFFSET_Y, 0.0))
    plate_neg = cq.Workplane("XY").box(0.030, PLATE_THICKNESS, 0.066).translate((WRIST_X, -PLATE_OFFSET_Y, 0.0))
    wrist_clevis = _union_all(clevis_web, plate_pos, plate_neg).cut(
        _joint_clearance_cut(WRIST_X, 0.0, SMALL_HUB_RADIUS + JOINT_CLEARANCE)
    )

    side_cover = cq.Workplane("XY").box(0.056, 0.004, 0.032).translate((0.088, -0.028, -0.014))
    side_cover_bolts = _bolt_heads_xz(
        [(0.062, -0.002), (0.062, -0.026), (0.112, -0.002), (0.112, -0.026)],
        radius=0.0028,
        height=0.003,
        y0=-0.0315,
    )
    wrist_bosses = _joint_bosses(WRIST_X, 0.0, radius=0.015, length=0.006)
    hardware = _union_all(side_cover, side_cover_bolts, wrist_bosses)
    return {
        "root_hub": root_hub,
        "journal_core": journal_core,
        "arm_body": arm_body,
        "wrist_clevis": wrist_clevis,
        "arm_hardware": hardware,
    }


def _build_wrist_shapes() -> dict[str, cq.Workplane]:
    journal_core = _y_cylinder(SMALL_HUB_RADIUS, HUB_LENGTH)
    root_hub = _retained_hub(hub_radius=SMALL_HUB_RADIUS, hub_length=HUB_LENGTH, shaft_radius=0.009, collar_radius=0.023)

    root_block = cq.Workplane("XY").box(0.026, 0.038, 0.020, centered=(False, True, True)).translate((0.002, 0.0, -0.014))
    transition_block = cq.Workplane("XY").box(0.022, 0.036, 0.016, centered=(False, True, True)).translate((0.018, 0.0, -0.015))
    beam = _box_tube(0.050, 0.044, 0.034, wall=0.005, cap=0.009).translate((0.036, 0.0, -0.018))
    top_pocket = cq.Workplane("XY").box(0.040, 0.030, 0.003).translate((0.060, 0.0, -0.0005))
    wrist_body = _union_all(root_block, transition_block, beam).cut(top_pocket)

    end_mount = _union_all(
        cq.Workplane("XY").box(0.028, 0.070, 0.070).translate((0.092, 0.0, 0.0)),
        cq.Workplane("XY").box(0.012, 0.054, 0.054).translate((0.112, 0.0, 0.0)),
    )
    end_mount = end_mount.cut(_x_cylinder(0.013, 0.010).translate((0.110, 0.0, 0.0)))

    top_cover = cq.Workplane("XY").box(0.042, 0.032, 0.004).translate((0.060, 0.0, 0.0015))
    top_cover_bolts = _bolt_heads_xy(
        [(0.042, -0.011), (0.042, 0.011), (0.078, -0.011), (0.078, 0.011)],
        radius=0.0026,
        height=0.003,
        z0=0.0035,
    )
    face_bolts = _bolt_heads_yz(
        [(-0.020, -0.020), (-0.020, 0.020), (0.020, -0.020), (0.020, 0.020)],
        radius=0.0028,
        height=0.003,
        x0=0.118,
    )
    hardware = _union_all(top_cover, top_cover_bolts, face_bolts)
    return {
        "root_hub": root_hub,
        "journal_core": journal_core,
        "wrist_body": wrist_body,
        "end_mount": end_mount,
        "wrist_hardware": hardware,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_joint_armature_study", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    medium_steel = model.material("medium_steel", rgba=(0.47, 0.49, 0.53, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    cover_black = model.material("cover_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base_frame")
    base_shapes = _build_base_shapes()
    for name, shape, material in (
        ("base_body", base_shapes["base_body"], dark_steel),
        ("yaw_housing", base_shapes["yaw_housing"], medium_steel),
        ("base_hardware", base_shapes["base_hardware"], bright_steel),
    ):
        base.visual(mesh_from_cadquery(shape, f"{name}.obj", assets=ASSETS), material=material, name=name)
    base.inertial = Inertial.from_geometry(Box((0.34, 0.22, BASE_JOINT_Z)), mass=18.0, origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z / 2.0)))

    turret = model.part("turret")
    turret_shapes = _build_turret_shapes()
    turret.visual(mesh_from_cadquery(turret_shapes["base_flange"], "turret_base_flange.obj", assets=ASSETS), material=medium_steel, name="base_flange")
    turret.visual(mesh_from_cadquery(turret_shapes["turret_body"], "turret_body.obj", assets=ASSETS), material=dark_steel, name="turret_body")
    turret.visual(mesh_from_cadquery(turret_shapes["shoulder_clevis"], "turret_shoulder_clevis.obj", assets=ASSETS), material=medium_steel, name="shoulder_clevis")
    turret.visual(mesh_from_cadquery(turret_shapes["turret_hardware"], "turret_hardware.obj", assets=ASSETS), material=cover_black, name="turret_hardware")
    turret.inertial = Inertial.from_geometry(Box((0.12, 0.09, 0.19)), mass=6.0, origin=Origin(xyz=(0.02, 0.0, 0.10)))

    upper_arm = model.part("upper_arm")
    upper_shapes = _build_upper_arm_shapes()
    upper_arm.visual(mesh_from_cadquery(upper_shapes["root_hub"], "upper_root_hub.obj", assets=ASSETS), material=bright_steel, name="root_hub")
    upper_arm.visual(mesh_from_cadquery(upper_shapes["journal_core"], "upper_journal_core.obj", assets=ASSETS), material=bright_steel, name="journal_core")
    upper_arm.visual(mesh_from_cadquery(upper_shapes["arm_body"], "upper_arm_body.obj", assets=ASSETS), material=dark_steel, name="arm_body")
    upper_arm.visual(mesh_from_cadquery(upper_shapes["elbow_clevis"], "upper_elbow_clevis.obj", assets=ASSETS), material=medium_steel, name="elbow_clevis")
    upper_arm.visual(mesh_from_cadquery(upper_shapes["arm_hardware"], "upper_arm_hardware.obj", assets=ASSETS), material=cover_black, name="arm_hardware")
    upper_arm.inertial = Inertial.from_geometry(Box((0.21, 0.08, 0.08)), mass=4.2, origin=Origin(xyz=(0.10, 0.0, 0.0)))

    forearm = model.part("forearm")
    forearm_shapes = _build_forearm_shapes()
    forearm.visual(mesh_from_cadquery(forearm_shapes["root_hub"], "forearm_root_hub.obj", assets=ASSETS), material=bright_steel, name="root_hub")
    forearm.visual(mesh_from_cadquery(forearm_shapes["journal_core"], "forearm_journal_core.obj", assets=ASSETS), material=bright_steel, name="journal_core")
    forearm.visual(mesh_from_cadquery(forearm_shapes["arm_body"], "forearm_body.obj", assets=ASSETS), material=dark_steel, name="arm_body")
    forearm.visual(mesh_from_cadquery(forearm_shapes["wrist_clevis"], "forearm_wrist_clevis.obj", assets=ASSETS), material=medium_steel, name="wrist_clevis")
    forearm.visual(mesh_from_cadquery(forearm_shapes["arm_hardware"], "forearm_hardware.obj", assets=ASSETS), material=cover_black, name="arm_hardware")
    forearm.inertial = Inertial.from_geometry(Box((0.17, 0.08, 0.08)), mass=3.0, origin=Origin(xyz=(0.08, 0.0, 0.0)))

    wrist = model.part("wrist_head")
    wrist_shapes = _build_wrist_shapes()
    wrist.visual(mesh_from_cadquery(wrist_shapes["root_hub"], "wrist_root_hub.obj", assets=ASSETS), material=bright_steel, name="root_hub")
    wrist.visual(mesh_from_cadquery(wrist_shapes["journal_core"], "wrist_journal_core.obj", assets=ASSETS), material=bright_steel, name="journal_core")
    wrist.visual(mesh_from_cadquery(wrist_shapes["wrist_body"], "wrist_body.obj", assets=ASSETS), material=dark_steel, name="wrist_body")
    wrist.visual(mesh_from_cadquery(wrist_shapes["end_mount"], "wrist_end_mount.obj", assets=ASSETS), material=medium_steel, name="end_mount")
    wrist.visual(mesh_from_cadquery(wrist_shapes["wrist_hardware"], "wrist_hardware.obj", assets=ASSETS), material=cover_black, name="wrist_hardware")
    wrist.inertial = Inertial.from_geometry(Box((0.13, 0.08, 0.08)), mass=1.9, origin=Origin(xyz=(0.07, 0.0, 0.0)))

    model.articulation(
        "base_to_turret",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "turret_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=SHOULDER_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.5, lower=-1.0, upper=0.2),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=48.0, velocity=1.8, lower=-1.15, upper=0.3),
    )
    model.articulation(
        "forearm_to_wrist_head",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(WRIST_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=2.2, lower=-1.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_head")

    base_to_turret = object_model.get_articulation("base_to_turret")
    turret_to_upper_arm = object_model.get_articulation("turret_to_upper_arm")
    upper_arm_to_forearm = object_model.get_articulation("upper_arm_to_forearm")
    forearm_to_wrist_head = object_model.get_articulation("forearm_to_wrist_head")

    yaw_housing = base.get_visual("yaw_housing")
    base_flange = turret.get_visual("base_flange")
    shoulder_clevis = turret.get_visual("shoulder_clevis")
    turret_hardware = turret.get_visual("turret_hardware")
    upper_root_hub = upper_arm.get_visual("root_hub")
    upper_journal_core = upper_arm.get_visual("journal_core")
    elbow_clevis = upper_arm.get_visual("elbow_clevis")
    upper_hardware = upper_arm.get_visual("arm_hardware")
    forearm_root_hub = forearm.get_visual("root_hub")
    forearm_journal_core = forearm.get_visual("journal_core")
    wrist_clevis = forearm.get_visual("wrist_clevis")
    forearm_hardware = forearm.get_visual("arm_hardware")
    wrist_root_hub = wrist.get_visual("root_hub")
    wrist_journal_core = wrist.get_visual("journal_core")

    def _center_from_aabb(bounds: object) -> tuple[float, float, float] | None:
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        turret,
        upper_arm,
        elem_a=turret_hardware,
        elem_b=upper_root_hub,
        reason="shoulder joint uses captive retainer hardware that intentionally nests around the upper-arm hub",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a=upper_hardware,
        elem_b=forearm_root_hub,
        reason="elbow joint uses captive retainer hardware that intentionally nests around the forearm hub",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a=forearm_hardware,
        elem_b=wrist_root_hub,
        reason="wrist joint uses captive retainer hardware that intentionally nests around the wrist hub",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=20, ignore_adjacent=True)

    ctx.check("expected part count", len(object_model.parts) == 5, f"expected 5 parts, got {len(object_model.parts)}")
    ctx.check("expected joint count", len(object_model.articulations) == 4, f"expected 4 joints, got {len(object_model.articulations)}")

    ctx.expect_overlap(turret, base, axes="xy", min_overlap=0.060, elem_a=base_flange, elem_b=yaw_housing)
    ctx.expect_gap(
        turret,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=base_flange,
        negative_elem=yaw_housing,
    )

    ctx.expect_overlap(upper_arm, turret, axes="xz", min_overlap=0.040, elem_a=upper_journal_core, elem_b=shoulder_clevis)
    ctx.expect_within(upper_arm, turret, axes="y", margin=0.012, inner_elem=upper_journal_core, outer_elem=shoulder_clevis)
    ctx.expect_contact(upper_arm, turret, elem_a=upper_root_hub, elem_b=turret_hardware)

    ctx.expect_overlap(forearm, upper_arm, axes="xz", min_overlap=0.038, elem_a=forearm_journal_core, elem_b=elbow_clevis)
    ctx.expect_within(forearm, upper_arm, axes="y", margin=0.012, inner_elem=forearm_journal_core, outer_elem=elbow_clevis)
    ctx.expect_contact(forearm, upper_arm, elem_a=forearm_root_hub, elem_b=upper_hardware)

    ctx.expect_overlap(wrist, forearm, axes="xz", min_overlap=0.034, elem_a=wrist_journal_core, elem_b=wrist_clevis)
    ctx.expect_within(wrist, forearm, axes="y", margin=0.012, inner_elem=wrist_journal_core, outer_elem=wrist_clevis)
    ctx.expect_contact(wrist, forearm, elem_a=wrist_root_hub, elem_b=forearm_hardware)

    with ctx.pose({base_to_turret: 0.85}):
        ctx.expect_overlap(turret, base, axes="xy", min_overlap=0.060, elem_a=base_flange, elem_b=yaw_housing)
        yawed_wrist = ctx.part_world_position(wrist)
        ctx.check(
            "base yaw swings chain laterally",
            yawed_wrist is not None and abs(yawed_wrist[1]) > 0.12,
            f"wrist position at yaw pose was {yawed_wrist}",
        )

    with ctx.pose({turret_to_upper_arm: -0.70}):
        ctx.expect_overlap(upper_arm, turret, axes="xz", min_overlap=0.040, elem_a=upper_root_hub, elem_b=shoulder_clevis)
        raised_end = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem="end_mount"))
        ctx.check(
            "shoulder pitch lifts terminal mount",
            raised_end is not None and raised_end[2] > 0.43,
            f"terminal mount center at shoulder pose was {raised_end}",
        )

    with ctx.pose({turret_to_upper_arm: -0.25, upper_arm_to_forearm: -0.10}):
        straight_wrist = ctx.part_world_position(wrist)
    with ctx.pose({turret_to_upper_arm: -0.25, upper_arm_to_forearm: -0.95}):
        folded_wrist = ctx.part_world_position(wrist)
        ctx.expect_overlap(forearm, upper_arm, axes="xz", min_overlap=0.038, elem_a=forearm_root_hub, elem_b=elbow_clevis)
        ctx.check(
            "elbow fold retracts wrist origin",
            straight_wrist is not None and folded_wrist is not None and folded_wrist[0] < straight_wrist[0] - 0.08,
            f"straight wrist={straight_wrist}, folded wrist={folded_wrist}",
        )
        ctx.check(
            "elbow folded pose clears base deck",
            folded_wrist is not None and folded_wrist[2] > BASE_JOINT_Z + 0.16,
            f"folded wrist position was {folded_wrist}",
        )

    with ctx.pose({turret_to_upper_arm: -0.20, upper_arm_to_forearm: -0.20}):
        neutral_end = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem="end_mount"))
    with ctx.pose({turret_to_upper_arm: -0.20, upper_arm_to_forearm: -0.20, forearm_to_wrist_head: -0.75}):
        pitched_end = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem="end_mount"))
        ctx.expect_overlap(wrist, forearm, axes="xz", min_overlap=0.034, elem_a=wrist_root_hub, elem_b=wrist_clevis)
        ctx.check(
            "wrist pitch reorients terminal mount",
            neutral_end is not None and pitched_end is not None and pitched_end[2] > neutral_end[2] + 0.012,
            f"neutral end={neutral_end}, pitched end={pitched_end}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
