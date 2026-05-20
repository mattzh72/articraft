from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.34
BASE_Y = 0.22
BASE_Z = 0.04

COLUMN_X = 0.12
COLUMN_Y = 0.16
COLUMN_Z = 0.18

SHOULDER_Z = BASE_Z + COLUMN_Z + 0.07

PEDESTAL_YOKE_X = 0.12
PEDESTAL_YOKE_Y = 0.16
PEDESTAL_YOKE_Z = 0.17
PEDESTAL_YOKE_Z0 = 0.19
PEDESTAL_YOKE_SLOT_X = 0.10
PEDESTAL_YOKE_SLOT_Y = 0.11
PEDESTAL_YOKE_SLOT_Z = 0.13
PEDESTAL_YOKE_SLOT_Z0 = 0.22

SHOULDER_PIN_R = 0.022
SHOULDER_DRUM_R = 0.070
SHOULDER_AXLE_LEN = 0.170
SHOULDER_DRUM_LEN = 0.034
SHOULDER_DRUM_Y = 0.106
SHOULDER_CAP_R = 0.032
SHOULDER_CAP_LEN = 0.020
SHOULDER_CAP_Y = -0.086

UPPER_ROOT_X0 = 0.0
UPPER_ROOT_LEN = 0.085
UPPER_ROOT_Y = 0.072
UPPER_ROOT_Z = 0.092
UPPER_BODY_X0 = 0.07
UPPER_BODY_LEN = 0.20
UPPER_BODY_Y = 0.078
UPPER_BODY_Z = 0.078
UPPER_RIB_X0 = 0.10
UPPER_RIB_LEN = 0.14
UPPER_RIB_Y = 0.050
UPPER_RIB_Z = 0.022
UPPER_RIB_Z_OFFSET = -0.032
ELBOW_X = 0.34

FORK_BLOCK_X0 = 0.24
FORK_BLOCK_LEN = 0.12
FORK_BLOCK_Y = 0.135
FORK_BLOCK_Z = 0.102
FORK_SLOT_X0 = 0.285
FORK_SLOT_LEN = 0.075
FORK_SLOT_Y = 0.090
FORK_SLOT_Z = 0.106

ELBOW_PIN_R = 0.018
ELBOW_DRUM_R = 0.050
ELBOW_AXLE_LEN = 0.150
ELBOW_DRUM_LEN = 0.030
ELBOW_DRUM_Y = -0.093
ELBOW_CAP_R = 0.026
ELBOW_CAP_LEN = 0.018
ELBOW_CAP_Y = 0.074

FOREARM_LEN = 0.30
FOREARM_ROOT_X0 = 0.0
FOREARM_ROOT_LEN = 0.090
FOREARM_ROOT_Y = 0.070
FOREARM_ROOT_Z = 0.088
FOREARM_BODY_X0 = 0.07
FOREARM_BODY_LEN = 0.18
FOREARM_BODY_Y = 0.064
FOREARM_BODY_Z = 0.064
FOREARM_RIB_X0 = 0.10
FOREARM_RIB_LEN = 0.12
FOREARM_RIB_Y = 0.046
FOREARM_RIB_Z = 0.020
FOREARM_RIB_Z_OFFSET = -0.027
FOREARM_TIP_X0 = 0.235
FOREARM_TIP_LEN = FOREARM_LEN - FOREARM_TIP_X0
FOREARM_TIP_Y = 0.072
FOREARM_TIP_Z = 0.080

PLATE_BOSS_LEN = 0.024
PLATE_BOSS_Y = 0.050
PLATE_BOSS_Z = 0.050
END_PLATE_THICK = 0.018
END_PLATE_Y = 0.120
END_PLATE_Z = 0.120

SHOULDER_BASE_R = 0.085
SHOULDER_BASE_H = 0.060
UPPER_HUB_R = 0.095
UPPER_HUB_H = 0.050
UPPER_BEAM_X0 = 0.025
UPPER_BEAM_LEN = 0.260
UPPER_BEAM_Y = 0.070
UPPER_BEAM_Z = 0.045
UPPER_RIB2_X0 = 0.09
UPPER_RIB2_LEN = 0.15
UPPER_RIB2_Y = 0.040
UPPER_RIB2_Z = 0.018
UPPER_RIB2_Z_OFFSET = -0.004
ELBOW_SUPPORT_R = 0.040
ELBOW_SUPPORT_H = 0.038
ELBOW_X = 0.34

ELBOW_HUB_R = 0.055
ELBOW_HUB_H = 0.040
FOREARM_BEAM_X0 = 0.020
FOREARM_BEAM_LEN = 0.280
FOREARM_BEAM_Y = 0.060
FOREARM_BEAM_Z = 0.040
FOREARM_RIB2_X0 = 0.085
FOREARM_RIB2_LEN = 0.14
FOREARM_RIB2_Y = 0.034
FOREARM_RIB2_Z = 0.016
FOREARM_RIB2_Z_OFFSET = -0.004


def y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length, both=True).translate((0.0, y, 0.0))


def z_cylinder(radius: float, height: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def x_box(
    length: float,
    width: float,
    height: float,
    *,
    x0: float,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x0, y, z))
    )


def make_pedestal() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_Z, centered=(True, True, False))
    column = (
        cq.Workplane("XY")
        .box(COLUMN_X, COLUMN_Y, COLUMN_Z, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_Z))
    )
    shoulder_base = z_cylinder(
        SHOULDER_BASE_R,
        SHOULDER_BASE_H,
        z0=SHOULDER_Z - SHOULDER_BASE_H - 0.015,
    )
    return base.union(column).union(shoulder_base)


def make_upper_link() -> cq.Workplane:
    shoulder_hub = z_cylinder(UPPER_HUB_R, UPPER_HUB_H)
    beam = x_box(
        UPPER_BEAM_LEN,
        UPPER_BEAM_Y,
        UPPER_BEAM_Z,
        x0=UPPER_BEAM_X0,
        z=UPPER_BEAM_Z / 2.0,
    )
    rib = x_box(
        UPPER_RIB2_LEN,
        UPPER_RIB2_Y,
        UPPER_RIB2_Z,
        x0=UPPER_RIB2_X0,
        z=UPPER_RIB2_Z_OFFSET,
    )
    elbow_support = z_cylinder(ELBOW_SUPPORT_R, ELBOW_SUPPORT_H, x=ELBOW_X, z0=-ELBOW_SUPPORT_H)
    return (
        shoulder_hub.union(beam)
        .union(beam)
        .union(rib)
        .union(elbow_support)
    )


def make_forearm() -> cq.Workplane:
    elbow_hub = z_cylinder(ELBOW_HUB_R, ELBOW_HUB_H)
    beam = x_box(
        FOREARM_BEAM_LEN,
        FOREARM_BEAM_Y,
        FOREARM_BEAM_Z,
        x0=FOREARM_BEAM_X0,
        z=FOREARM_BEAM_Z / 2.0,
    )
    rib = x_box(
        FOREARM_RIB2_LEN,
        FOREARM_RIB2_Y,
        FOREARM_RIB2_Z,
        x0=FOREARM_RIB2_X0,
        z=FOREARM_RIB2_Z_OFFSET,
    )
    tip = x_box(0.040, 0.075, 0.050, x0=FOREARM_LEN - 0.040, z=0.025)
    return elbow_hub.union(beam).union(rib).union(tip)


def make_end_plate() -> cq.Workplane:
    return x_box(END_PLATE_THICK, END_PLATE_Y, END_PLATE_Z, x0=0.0, z=END_PLATE_Z / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_root_elbow_arm")
    shoulder_hub_radius = 0.055
    elbow_hub_radius = 0.035
    shoulder_axis_z = 0.305
    elbow_axis_x = 0.290
    forearm_len = 0.300

    model.meta["shoulder_drum_radius"] = shoulder_hub_radius
    model.meta["elbow_drum_radius"] = elbow_hub_radius

    dark_base = model.material("dark_base", rgba=(0.16, 0.18, 0.20, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    forearm_gray = model.material("forearm_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.34, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_base,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.12, 0.12, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_base,
        name="column",
    )
    pedestal.visual(
        Box((0.10, 0.13, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2325)),
        material=dark_base,
        name="yoke_bridge",
    )
    pedestal.visual(
        Box((0.10, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, 0.055, shoulder_axis_z)),
        material=dark_base,
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.10, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, -0.055, shoulder_axis_z)),
        material=dark_base,
        name="right_cheek",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=shoulder_hub_radius, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=arm_gray,
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((0.21, 0.08, 0.06)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=arm_gray,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.12, 0.04, 0.016)),
        origin=Origin(xyz=(0.150, 0.0, -0.022)),
        material=arm_gray,
        name="upper_rib",
    )
    upper_link.visual(
        Box((0.12, 0.022, 0.09)),
        origin=Origin(xyz=(0.290, 0.051, 0.0)),
        material=arm_gray,
        name="fork_left",
    )
    upper_link.visual(
        Box((0.12, 0.022, 0.09)),
        origin=Origin(xyz=(0.290, -0.051, 0.0)),
        material=arm_gray,
        name="fork_right",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=elbow_hub_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=forearm_gray,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.21, 0.06, 0.05)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=forearm_gray,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.13, 0.034, 0.016)),
        origin=Origin(xyz=(0.145, 0.0, -0.017)),
        material=forearm_gray,
        name="forearm_rib",
    )
    forearm.visual(
        Box((0.055, 0.072, 0.07)),
        origin=Origin(xyz=(0.2725, 0.0, 0.0)),
        material=forearm_gray,
        name="wrist_block",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box((0.018, 0.12, 0.12)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=plate_gray,
        name="end_plate_shell",
    )

    model.articulation(
        "pedestal_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, shoulder_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.20,
        ),
    )

    model.articulation(
        "upper_link_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(elbow_axis_x, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.8,
            lower=-0.15,
            upper=1.45,
        ),
    )

    model.articulation(
        "forearm_to_end_plate",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(forearm_len, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("pedestal_to_upper_link")
    elbow = object_model.get_articulation("upper_link_to_forearm")
    plate_mount = object_model.get_articulation("forearm_to_end_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (pedestal, upper_link, forearm, end_plate)),
        "Expected pedestal, upper_link, forearm, and end_plate parts.",
    )
    ctx.check(
        "serial_parallel_revolutes",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and shoulder.axis == elbow.axis,
        f"Shoulder and elbow must both be revolute and share one axis; got {shoulder.axis} and {elbow.axis}.",
    )
    ctx.check(
        "end_plate_is_fixed",
        plate_mount.articulation_type == ArticulationType.FIXED,
        f"End plate should be plain and fixed; got {plate_mount.articulation_type}.",
    )
    ctx.check(
        "shoulder_hub_visibly_larger_than_elbow_hub",
        object_model.meta["shoulder_drum_radius"] > object_model.meta["elbow_drum_radius"],
        "Shoulder hub radius must exceed elbow hub radius.",
    )

    ctx.expect_contact(
        upper_link,
        pedestal,
        contact_tol=1e-5,
        name="shoulder_trunnion_is_supported_by_pedestal",
    )
    ctx.expect_contact(
        forearm,
        upper_link,
        contact_tol=1e-5,
        name="elbow_trunnion_is_supported_by_upper_link",
    )
    ctx.expect_contact(
        end_plate,
        forearm,
        contact_tol=1e-5,
        name="end_plate_mount_contacts_forearm",
    )
    ctx.expect_gap(
        end_plate,
        forearm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="end_plate_seats_flush_on_forearm_tip",
    )
    ctx.expect_overlap(
        end_plate,
        forearm,
        axes="yz",
        min_overlap=0.045,
        name="end_plate_mount_has_forearm_footprint_overlap",
    )

    elbow_rest = ctx.part_world_position(forearm)
    plate_rest = ctx.part_world_position(end_plate)
    with ctx.pose({shoulder: 0.85}):
        elbow_raised = ctx.part_world_position(forearm)
    ctx.check(
        "positive_shoulder_rotation_raises_elbow",
        elbow_rest is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_rest[2] + 0.20
        and elbow_raised[0] < elbow_rest[0],
        f"Elbow origin should rise and sweep back under positive shoulder motion; rest={elbow_rest}, raised={elbow_raised}.",
    )

    with ctx.pose({elbow: 1.05}):
        plate_flexed = ctx.part_world_position(end_plate)
    ctx.check(
        "positive_elbow_rotation_raises_end_plate",
        plate_rest is not None
        and plate_flexed is not None
        and plate_flexed[2] > plate_rest[2] + 0.20
        and plate_flexed[0] < plate_rest[0] - 0.08,
        f"End plate should lift and retract under positive elbow motion; rest={plate_rest}, flexed={plate_flexed}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
