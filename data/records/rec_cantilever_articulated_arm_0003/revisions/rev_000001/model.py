from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_PLATE_X = 0.34
BASE_PLATE_Y = 0.26
BASE_PLATE_Z = 0.02

COLUMN_CENTER_X = -0.105
COLUMN_OUTER_X = 0.11
COLUMN_OUTER_Y = 0.10
COLUMN_HEIGHT = 0.34
COLUMN_WALL = 0.014
COLUMN_FRONT_X = COLUMN_CENTER_X + COLUMN_OUTER_X / 2.0

SHOULDER_X = 0.04
SHOULDER_Z = 0.338
SHOULDER_PLATE_GAP = 0.10
SHOULDER_PLATE_T = 0.012
SHOULDER_CARTRIDGE_T = 0.01
SHOULDER_CARTRIDGE_OUTER_R = 0.031
SHOULDER_PIN_R = 0.0135
SHOULDER_HOLE_R = 0.0146

ELBOW_PLATE_GAP = 0.086
ELBOW_PLATE_T = 0.012
ELBOW_CARTRIDGE_T = 0.01
ELBOW_CARTRIDGE_OUTER_R = 0.0265
ELBOW_PIN_R = 0.0136
ELBOW_HOLE_R = 0.0146

UPPER_ARM_ELBOW_X = 0.44
TOOL_MOUNT_X = 0.355


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").transformed(offset=center).box(*size)


def _cyl_y(
    center: tuple[float, float, float],
    radius: float,
    length: float,
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, cy, 0.0))
    )


def _ring_y(
    center: tuple[float, float, float],
    outer_radius: float,
    inner_radius: float,
    length: float,
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, cy, 0.0))
    )


def _cyl_x(
    center: tuple[float, float, float],
    radius: float,
    length: float,
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .center(cy, cz)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((cx, 0.0, 0.0))
    )


def _cyl_z(
    center: tuple[float, float, float],
    radius: float,
    length: float,
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, cz))
    )


def _slot_cut_xz(
    center: tuple[float, float, float],
    length_x: float,
    width_z: float,
    depth_y: float,
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .slot2D(length_x, width_z)
        .extrude(depth_y / 2.0, both=True)
        .translate((0.0, cy, 0.0))
    )


def _bolt_heads_y(
    center_x: float,
    center_z: float,
    center_y: float,
    pitch_x: float,
    pitch_z: float,
    radius: float,
    thickness: float,
) -> cq.Workplane:
    bolts = cq.Workplane("XY")
    for x_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            bolts = bolts.union(
                _cyl_y(
                    (
                        center_x + x_sign * pitch_x / 2.0,
                        center_y,
                        center_z + z_sign * pitch_z / 2.0,
                    ),
                    radius,
                    thickness,
                )
            )
    return bolts


def _make_base_structure_shape() -> cq.Workplane:
    base_plate = _box((0.0, 0.0, BASE_PLATE_Z / 2.0), (BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z))
    pads = cq.Workplane("XY")
    for x in (-0.115, 0.115):
        for y in (-0.080, 0.080):
            pads = pads.union(_box((x, y, 0.030), (0.060, 0.050, 0.020)))
    base = base_plate.union(pads)
    for x in (-0.115, 0.115):
        for y in (-0.080, 0.080):
            base = base.cut(_cyl_z((x, y, 0.024), 0.010, 0.060))

    column_outer = _box(
        (COLUMN_CENTER_X, 0.0, BASE_PLATE_Z + COLUMN_HEIGHT / 2.0),
        (COLUMN_OUTER_X, COLUMN_OUTER_Y, COLUMN_HEIGHT),
    )
    column_inner = _box(
        (COLUMN_CENTER_X, 0.0, BASE_PLATE_Z + COLUMN_WALL + (COLUMN_HEIGHT - COLUMN_WALL) / 2.0),
        (
            COLUMN_OUTER_X - 2.0 * COLUMN_WALL,
            COLUMN_OUTER_Y - 2.0 * COLUMN_WALL,
            COLUMN_HEIGHT - COLUMN_WALL,
        ),
    )
    access_opening = _box((COLUMN_FRONT_X - 0.005, 0.0, 0.202), (0.020, 0.064, 0.182))
    column = column_outer.cut(column_inner).cut(access_opening)

    rear_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (COLUMN_CENTER_X - 0.053, BASE_PLATE_Z),
                (COLUMN_CENTER_X + 0.008, BASE_PLATE_Z),
                (COLUMN_CENTER_X + 0.008, 0.190),
            ]
        )
        .close()
        .extrude(0.018, both=True)
    )
    gussets = rear_gusset.translate((0.0, 0.038, 0.0)).union(rear_gusset.translate((0.0, -0.038, 0.0)))

    shoulder_plate_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.034, 0.230),
                (-0.012, 0.286),
                (0.012, 0.344),
                (0.038, 0.372),
                (0.070, 0.372),
                (0.070, 0.320),
                (0.050, 0.252),
                (0.016, 0.228),
            ]
        )
        .close()
        .extrude(SHOULDER_PLATE_T, both=True)
    )
    shoulder_plate_profile = shoulder_plate_profile.cut(
        _cyl_y((SHOULDER_X, 0.0, SHOULDER_Z), SHOULDER_HOLE_R, SHOULDER_PLATE_T + 0.010)
    )
    shoulder_plate_profile = shoulder_plate_profile.cut(_slot_cut_xz((0.006, 0.0, 0.308), 0.034, 0.026, 0.040))
    shoulder_plate_profile = shoulder_plate_profile.cut(_slot_cut_xz((0.030, 0.0, 0.350), 0.030, 0.022, 0.040))
    shoulder_plate_center_y = SHOULDER_PLATE_GAP / 2.0 + SHOULDER_PLATE_T / 2.0
    shoulder_plates = shoulder_plate_profile.translate((0.0, shoulder_plate_center_y, 0.0)).union(
        shoulder_plate_profile.translate((0.0, -shoulder_plate_center_y, 0.0))
    )

    top_bridge = _box(
        (0.010, 0.0, 0.394),
        (0.088, SHOULDER_PLATE_GAP + 2.0 * SHOULDER_PLATE_T, 0.016),
    )
    lower_bridge = _box(
        (-0.010, 0.0, 0.246),
        (0.060, SHOULDER_PLATE_GAP + 2.0 * SHOULDER_PLATE_T, 0.018),
    )
    forward_web = (
        cq.Workplane("XZ")
        .polyline([(-0.020, 0.230), (0.018, 0.230), (0.010, 0.260)])
        .close()
        .extrude(0.018, both=True)
    )
    forward_webs = forward_web.translate((0.0, 0.030, 0.0)).union(forward_web.translate((0.0, -0.030, 0.0)))

    return base.union(column).union(gussets).union(shoulder_plates).union(top_bridge).union(lower_bridge).union(forward_webs)


def _make_base_hardware_shape() -> cq.Workplane:
    cartridge_center_y = SHOULDER_PLATE_GAP / 2.0 + SHOULDER_PLATE_T + SHOULDER_CARTRIDGE_T / 2.0
    hardware = _ring_y(
        (SHOULDER_X, cartridge_center_y, SHOULDER_Z),
        SHOULDER_CARTRIDGE_OUTER_R,
        SHOULDER_HOLE_R,
        SHOULDER_CARTRIDGE_T,
    ).union(
        _ring_y(
            (SHOULDER_X, -cartridge_center_y, SHOULDER_Z),
            SHOULDER_CARTRIDGE_OUTER_R,
            SHOULDER_HOLE_R,
            SHOULDER_CARTRIDGE_T,
        )
    )
    hardware = hardware.union(
        _bolt_heads_y(SHOULDER_X, SHOULDER_Z, cartridge_center_y, 0.036, 0.036, 0.0042, 0.004)
    ).union(
        _bolt_heads_y(SHOULDER_X, SHOULDER_Z, -cartridge_center_y, 0.036, 0.036, 0.0042, 0.004)
    )
    return hardware


def _make_base_access_cover_shape() -> cq.Workplane:
    cover = _box((COLUMN_FRONT_X + 0.0015, 0.0, 0.202), (0.004, 0.076, 0.190))
    for z in (0.130, 0.274):
        for y in (-0.030, 0.030):
            cover = cover.union(_cyl_x((COLUMN_FRONT_X + 0.004, y, z), 0.004, 0.004))
    return cover


def _make_upper_arm_structure_shape() -> cq.Workplane:
    shoulder_journal = _make_upper_arm_shoulder_journal_shape()
    root_block = _box((0.160, 0.0, -0.018), (0.072, 0.040, 0.046))
    root_block = root_block.cut(_box((0.160, 0.0, -0.018), (0.026, 0.018, 0.022)))

    beam_outer = _box((0.235, 0.0, -0.026), (0.250, 0.050, 0.048))
    beam_inner = _box((0.235, 0.0, -0.026), (0.206, 0.028, 0.026))
    beam = beam_outer.cut(beam_inner)
    beam = beam.cut(_box((0.238, 0.0, -0.004), (0.182, 0.018, 0.010)))
    beam = beam.cut(_box((0.170, 0.0, -0.044), (0.054, 0.024, 0.012)))

    root_web = (
        cq.Workplane("XZ")
        .polyline([(0.132, -0.034), (0.182, -0.034), (0.174, -0.010), (0.138, -0.010)])
        .close()
        .extrude(0.006, both=True)
        .translate((0.0, 0.020, 0.0))
    )
    root_webs = root_web.union(root_web.mirror("XZ"))

    side_rib = _box((0.240, 0.027, -0.024), (0.192, 0.004, 0.056))
    side_rib = side_rib.cut(_slot_cut_xz((0.184, 0.027, -0.024), 0.066, 0.012, 0.008))
    side_rib = side_rib.cut(_slot_cut_xz((0.286, 0.027, -0.024), 0.054, 0.010, 0.008))
    side_ribs = side_rib.union(side_rib.mirror("XZ"))

    clevis_ear = _box((0.430, 0.047, -0.004), (0.062, 0.010, 0.088))
    clevis_ear = clevis_ear.cut(_cyl_y((UPPER_ARM_ELBOW_X, 0.047, 0.0), ELBOW_HOLE_R, 0.024))
    clevis_bridge = _box((0.404, 0.0, -0.024), (0.026, 0.040, 0.034))
    clevis = clevis_ear.union(clevis_ear.mirror("XZ")).union(clevis_bridge)

    return shoulder_journal.union(root_block).union(root_webs).union(beam).union(side_ribs).union(clevis)


def _make_upper_arm_shoulder_journal_shape() -> cq.Workplane:
    journal = _cyl_y((0.0, 0.0, 0.0), 0.0140, 0.146)
    lock_nuts = _cyl_y((0.0, 0.076, 0.0), 0.018, 0.006).union(_cyl_y((0.0, -0.076, 0.0), 0.018, 0.006))
    transition = _box((0.082, 0.0, -0.014), (0.164, 0.022, 0.026))
    nose = _box((0.152, 0.0, -0.016), (0.024, 0.026, 0.026))
    return journal.union(lock_nuts).union(transition).union(nose)


def _make_upper_arm_hardware_shape() -> cq.Workplane:
    elbow_cartridge_center_y = 0.053
    elbow_hardware = _ring_y(
        (UPPER_ARM_ELBOW_X, elbow_cartridge_center_y, 0.0),
        0.0225,
        ELBOW_HOLE_R,
        0.008,
    ).union(
        _ring_y(
            (UPPER_ARM_ELBOW_X, -elbow_cartridge_center_y, 0.0),
            0.0225,
            ELBOW_HOLE_R,
            0.008,
        )
    )
    elbow_hardware = elbow_hardware.union(
        _bolt_heads_y(UPPER_ARM_ELBOW_X, 0.0, elbow_cartridge_center_y, 0.028, 0.028, 0.0035, 0.004)
    ).union(
        _bolt_heads_y(UPPER_ARM_ELBOW_X, 0.0, -elbow_cartridge_center_y, 0.028, 0.028, 0.0035, 0.004)
    )

    return elbow_hardware


def _make_upper_arm_cover_shape() -> cq.Workplane:
    cover = _box((0.210, 0.0, -0.058), (0.122, 0.042, 0.004))
    for x in (0.152, 0.258):
        for y in (-0.018, 0.018):
            cover = cover.union(_cyl_z((x, y, -0.056), 0.0032, 0.004))
    return cover


def _make_forearm_structure_shape() -> cq.Workplane:
    elbow_journal = _cyl_y((0.0, 0.0, 0.0), 0.0140, 0.114)
    elbow_collars = _cyl_y((0.0, 0.061, 0.0), 0.017, 0.006).union(
        _cyl_y((0.0, -0.061, 0.0), 0.017, 0.006)
    )
    elbow_transition = _box((0.070, 0.0, -0.014), (0.140, 0.020, 0.024))
    elbow_nose = _box((0.118, 0.0, -0.016), (0.020, 0.024, 0.024))

    root_block = _box((0.118, 0.0, -0.018), (0.070, 0.032, 0.040))
    root_block = root_block.cut(_box((0.118, 0.0, -0.018), (0.024, 0.014, 0.018)))

    beam_outer = _box((0.190, 0.0, -0.024), (0.188, 0.040, 0.042))
    beam_inner = _box((0.190, 0.0, -0.024), (0.148, 0.022, 0.022))
    beam = beam_outer.cut(beam_inner)
    beam = beam.cut(_box((0.194, 0.0, -0.004), (0.122, 0.016, 0.010)))

    side_rib = _box((0.196, 0.022, -0.022), (0.140, 0.004, 0.050))
    side_rib = side_rib.cut(_slot_cut_xz((0.162, 0.022, -0.022), 0.050, 0.010, 0.008))
    side_rib = side_rib.cut(_slot_cut_xz((0.224, 0.022, -0.022), 0.040, 0.010, 0.008))
    side_ribs = side_rib.union(side_rib.mirror("XZ"))

    root_web = (
        cq.Workplane("XZ")
        .polyline([(0.100, -0.028), (0.140, -0.028), (0.134, -0.008), (0.106, -0.008)])
        .close()
        .extrude(0.006, both=True)
        .translate((0.0, 0.014, 0.0))
    )
    root_webs = root_web.union(root_web.mirror("XZ"))

    mount_block = _box((0.338, 0.0, -0.004), (0.034, 0.044, 0.060))
    mount_block = mount_block.cut(_box((0.334, 0.0, -0.004), (0.012, 0.018, 0.028)))

    tip_register = _cyl_x((TOOL_MOUNT_X - 0.012, 0.0, -0.004), 0.018, 0.010)

    return (
        elbow_journal.union(elbow_collars)
        .union(elbow_transition)
        .union(elbow_nose)
        .union(root_block)
        .union(root_webs)
        .union(beam)
        .union(side_ribs)
        .union(mount_block)
        .union(tip_register)
    )


def _make_forearm_hardware_shape() -> cq.Workplane:
    return _cyl_x((0.338, 0.016, -0.004), 0.0030, 0.008).union(
        _cyl_x((0.338, -0.016, -0.004), 0.0030, 0.008)
    )


def _make_forearm_cover_shape() -> cq.Workplane:
    cover = _box((0.186, 0.0, -0.050), (0.090, 0.034, 0.004))
    for x in (0.128, 0.200):
        for y in (-0.015, 0.015):
            cover = cover.union(_cyl_z((x, y, -0.048), 0.0030, 0.004))
    return cover


def _make_tool_flange_shape() -> cq.Workplane:
    tool = _box((0.007, 0.0, -0.004), (0.014, 0.064, 0.072))
    tool = tool.union(_cyl_x((0.020, 0.0, -0.004), 0.022, 0.018))
    tool = tool.union(_cyl_x((0.040, 0.0, -0.004), 0.043, 0.014))
    tool = tool.union(_cyl_x((0.051, 0.0, -0.004), 0.018, 0.010))
    tool = tool.cut(_cyl_x((0.040, 0.0, -0.004), 0.008, 0.042))

    bolt_points = [
        (0.028 * math.cos(angle), 0.028 * math.sin(angle))
        for angle in [index * math.tau / 6.0 for index in range(6)]
    ]
    flange_bolt_holes = (
        cq.Workplane("YZ")
        .transformed(offset=(0.040, 0.0, -0.004))
        .pushPoints(bolt_points)
        .circle(0.0038)
        .extrude(0.028, both=True)
    )
    dowel_holes = (
        cq.Workplane("YZ")
        .transformed(offset=(0.040, 0.0, -0.004))
        .pushPoints([(0.0, 0.018), (0.0, -0.018)])
        .circle(0.0026)
        .extrude(0.028, both=True)
    )
    return tool.cut(flange_bolt_holes).cut(dowel_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_articulated_arm", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.41, 0.44, 0.47, 1.0))
    oxide = model.material("oxide_black", rgba=(0.14, 0.15, 0.16, 1.0))
    machined = model.material("machined_steel", rgba=(0.63, 0.66, 0.69, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_make_base_structure_shape(), "base_structure.obj", assets=ASSETS),
        material=steel_dark,
        name="base_structure",
    )
    base.visual(
        mesh_from_cadquery(_make_base_hardware_shape(), "base_hardware.obj", assets=ASSETS),
        material=machined,
        name="shoulder_cartridges",
    )
    base.visual(
        mesh_from_cadquery(_make_base_access_cover_shape(), "base_cover.obj", assets=ASSETS),
        material=oxide,
        name="access_cover",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.40)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_structure_shape(), "upper_arm_structure.obj", assets=ASSETS),
        material=steel_mid,
        name="arm_structure",
    )
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_hardware_shape(), "upper_arm_hardware.obj", assets=ASSETS),
        material=machined,
        name="elbow_hardware",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.48, 0.09, 0.13)),
        mass=15.5,
        origin=Origin(xyz=(0.19, 0.0, -0.03)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_make_forearm_structure_shape(), "forearm_structure.obj", assets=ASSETS),
        material=oxide,
        name="arm_structure",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.36, 0.08, 0.11)),
        mass=10.5,
        origin=Origin(xyz=(0.17, 0.0, -0.025)),
    )

    tool = model.part("tool_flange")
    tool.visual(
        mesh_from_cadquery(_make_tool_flange_shape(), "tool_flange.obj", assets=ASSETS),
        material=machined,
        name="flange_body",
    )
    tool.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.06),
        mass=2.4,
        origin=Origin(xyz=(0.03, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.3, lower=-0.30, upper=1.10),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_ELBOW_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.7, lower=-0.10, upper=1.90),
    )
    model.articulation(
        "forearm_to_tool",
        ArticulationType.FIXED,
        parent=forearm,
        child=tool,
        origin=Origin(xyz=(TOOL_MOUNT_X, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    tool = object_model.get_part("tool_flange")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    base_structure = base.get_visual("base_structure")
    base_hardware = base.get_visual("shoulder_cartridges")
    base_cover = base.get_visual("access_cover")
    upper_structure = upper_arm.get_visual("arm_structure")
    upper_elbow_hardware = upper_arm.get_visual("elbow_hardware")
    forearm_structure = forearm.get_visual("arm_structure")
    tool_body = tool.get_visual("flange_body")

    def _span(aabb: object) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            float(maxs[0] - mins[0]),
            float(maxs[1] - mins[1]),
            float(maxs[2] - mins[2]),
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        base,
        upper_arm,
        elem_a=base_structure,
        elem_b=upper_structure,
        reason="shoulder trunnion journal intentionally nests through the cantilever bracket and bearing seats",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "detailed_visual_groups_exist",
        all(
            visual is not None
            for visual in (
                base_structure,
                base_hardware,
                base_cover,
                upper_structure,
                upper_elbow_hardware,
                forearm_structure,
                tool_body,
            )
        ),
        details="expected structural, hardware, and access-cover visuals are present",
    )

    ctx.expect_contact(
        upper_arm,
        base,
        elem_a=upper_structure,
        elem_b=base_hardware,
        contact_tol=0.0012,
        name="shoulder_trunnion_hub_runs_in_base_cartridges",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=forearm_structure,
        elem_b=upper_elbow_hardware,
        contact_tol=0.0012,
        name="elbow_trunnion_hub_runs_in_upper_cartridges",
    )
    ctx.expect_gap(
        tool,
        forearm,
        axis="x",
        max_gap=0.0006,
        max_penetration=0.0,
        positive_elem=tool_body,
        negative_elem=forearm_structure,
        name="tool_flange_mounts_flush_to_forearm_face",
    )

    ctx.expect_origin_gap(upper_arm, base, axis="z", min_gap=0.33, max_gap=0.35, name="shoulder_axis_sits_high_on_cantilever_column")
    ctx.expect_origin_gap(forearm, upper_arm, axis="x", min_gap=0.43, max_gap=0.45, name="upper_arm_sets_long_elbow_reach")
    ctx.expect_origin_gap(tool, forearm, axis="x", min_gap=0.35, max_gap=0.36, name="tool_flange_mounts_at_forearm_tip")

    ctx.expect_overlap(upper_arm, base, axes="yz", min_overlap=0.07, name="shoulder_link_overlaps_bracket_envelope")
    ctx.expect_overlap(forearm, upper_arm, axes="yz", min_overlap=0.05, name="forearm_overlaps_elbow_joint_envelope")
    ctx.expect_overlap(tool, forearm, axes="yz", min_overlap=0.06, name="tool_flange_backplate_matches_forearm_mount_face")
    ctx.expect_overlap(upper_arm, base, axes="x", min_overlap=0.02, name="offset_shoulder_bracket_reaches_forward_from_column")

    base_span = _span(ctx.part_world_aabb(base))
    upper_span = _span(ctx.part_world_aabb(upper_arm))
    forearm_span = _span(ctx.part_world_aabb(forearm))
    tool_span = _span(ctx.part_world_aabb(tool))
    ctx.check(
        "cantilever_base_is_tall_and_compact",
        base_span is not None and base_span[2] > 0.39 and base_span[0] < 0.36 and base_span[1] < 0.28,
        details=f"base spans={base_span}",
    )
    ctx.check(
        "upper_arm_reads_as_long_rigid_link",
        upper_span is not None and upper_span[0] > 0.34 and upper_span[2] < 0.16 and upper_span[1] < 0.18,
        details=f"upper arm spans={upper_span}",
    )
    ctx.check(
        "forearm_reads_as_second_long_link",
        forearm_span is not None and forearm_span[0] > 0.35 and forearm_span[2] < 0.14 and forearm_span[1] < 0.16,
        details=f"forearm spans={forearm_span}",
    )
    ctx.check(
        "tool_flange_remains_compact",
        tool_span is not None and tool_span[0] < 0.07 and tool_span[1] < 0.10 and tool_span[2] < 0.10,
        details=f"tool spans={tool_span}",
    )

    rest_tool = ctx.part_world_position(tool)
    with ctx.pose({shoulder: 0.75, elbow: 0.10}):
        ctx.expect_origin_gap(tool, base, axis="z", min_gap=0.76, name="raised_shoulder_lifts_tool_high_above_base")
        raised_tool = ctx.part_world_position(tool)
    with ctx.pose({shoulder: 0.35, elbow: 0.00}):
        extended_tool = ctx.part_world_position(tool)
    with ctx.pose({shoulder: 0.35, elbow: 1.15}):
        ctx.expect_origin_gap(tool, base, axis="x", min_gap=0.22, name="folded_elbow_keeps_tool_forward_of_column")
        folded_tool = ctx.part_world_position(tool)

    ctx.check(
        "shoulder_pose_changes_tool_height",
        rest_tool is not None and raised_tool is not None and raised_tool[2] > rest_tool[2] + 0.32,
        details=f"rest={rest_tool}, raised={raised_tool}",
    )
    ctx.check(
        "elbow_pose_retracts_and_raises_tool",
        extended_tool is not None
        and folded_tool is not None
        and folded_tool[0] < extended_tool[0] - 0.18
        and folded_tool[2] > extended_tool[2] + 0.12,
        details=f"extended={extended_tool}, folded={folded_tool}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
