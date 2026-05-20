from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.012
PLATE_W = 0.090
PLATE_H = 0.240
PLATE_X = -0.028

EAR_T = 0.009
CLEVIS_GAP = 0.016
CLEVIS_OUTER_W = CLEVIS_GAP + 2.0 * EAR_T

JOINT_BLOCK_LEN = 0.044
JOINT_H = 0.060
LUG_LEN = 0.028
LUG_T = 0.012
COLLAR_T = 0.005

BEAM_W = 0.040
BEAM_H = 0.048
BEAM_WALL = 0.004

ROOT_LEN = 0.180
DISTAL_LEN = 0.340

CLAMP_BLOCK_LEN = 0.030
CLAMP_PLATE_T = 0.008
CLAMP_W = 0.100
CLAMP_H = 0.140

WALL_TO_ROOT_X = 0.030
ROOT_TO_DISTAL_X = 0.208

MESH_TOL = 0.0005
MESH_ANGULAR_TOL = 0.05


def box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cyl_x(length: float, radius: float, start_x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def make_wall_plate() -> cq.Workplane:
    plate = box_solid((PLATE_T, PLATE_W, PLATE_H), (PLATE_X, 0.0, 0.0))
    shoulder_block = box_solid((0.048, 0.060, 0.100), (0.006, 0.0, 0.0))
    backbone = box_solid((0.022, 0.054, 0.150), (-0.017, 0.0, 0.0))
    rib_pos = 0.024
    side_rib = box_solid((0.034, 0.014, 0.160), (-0.010, rib_pos, 0.0))
    center_web = box_solid((0.028, 0.032, 0.092), (-0.002, 0.0, 0.0))
    wall = (
        plate.union(shoulder_block)
        .union(backbone)
        .union(center_web)
        .union(side_rib)
        .union(side_rib.mirror("YZ"))
    )

    for hole_z in (-0.078, -0.026, 0.026, 0.078):
        wall = wall.cut(cyl_x(0.020, 0.0045, -0.040, 0.0, hole_z))

    return wall


def make_root_link() -> cq.Workplane:
    shoulder_block = box_solid((0.028, 0.060, 0.070), (0.014, 0.0, 0.0))
    shoulder_transition = box_solid((0.020, 0.054, 0.064), (0.038, 0.0, 0.0))

    beam_outer_start = 0.048
    beam_outer_end = 0.172
    beam_outer_len = beam_outer_end - beam_outer_start
    beam_outer_center = ((beam_outer_start + beam_outer_end) / 2.0, 0.0, 0.0)
    beam_outer = box_solid((beam_outer_len, 0.050, 0.060), beam_outer_center)

    beam_inner_start = 0.060
    beam_inner_end = 0.160
    beam_inner_len = beam_inner_end - beam_inner_start
    beam_inner_center = ((beam_inner_start + beam_inner_end) / 2.0, 0.0, 0.0)
    beam_inner = box_solid((beam_inner_len, 0.042, 0.052), beam_inner_center)
    beam = beam_outer.cut(beam_inner)

    elbow_block = box_solid((0.036, 0.056, 0.068), (0.190, 0.0, 0.0))
    return shoulder_block.union(shoulder_transition).union(beam).union(elbow_block)


def make_distal_link() -> cq.Workplane:
    elbow_block = box_solid((0.028, 0.056, 0.068), (0.014, 0.0, 0.0))
    elbow_transition = box_solid((0.020, 0.054, 0.064), (0.038, 0.0, 0.0))

    beam_outer_start = 0.048
    beam_outer_end = 0.310
    beam_outer_len = beam_outer_end - beam_outer_start
    beam_outer_center = ((beam_outer_start + beam_outer_end) / 2.0, 0.0, 0.0)
    beam_outer = box_solid((beam_outer_len, 0.048, 0.058), beam_outer_center)

    beam_inner_start = 0.060
    beam_inner_end = 0.298
    beam_inner_len = beam_inner_end - beam_inner_start
    beam_inner_center = ((beam_inner_start + beam_inner_end) / 2.0, 0.0, 0.0)
    beam_inner = box_solid((beam_inner_len, 0.040, 0.050), beam_inner_center)
    beam = beam_outer.cut(beam_inner)

    end_block = box_solid((0.030, 0.052, 0.066), (DISTAL_LEN - 0.015, 0.0, 0.0))
    return elbow_block.union(elbow_transition).union(beam).union(end_block)


def make_clamp_plate() -> cq.Workplane:
    mount_block = box_solid((CLAMP_BLOCK_LEN, 0.050, 0.060), (CLAMP_BLOCK_LEN / 2.0, 0.0, 0.0))
    plate = box_solid((CLAMP_PLATE_T, CLAMP_W, CLAMP_H), (CLAMP_BLOCK_LEN + CLAMP_PLATE_T / 2.0, 0.0, 0.0))
    top_flange = box_solid((0.024, CLAMP_W, 0.012), (0.018, 0.0, CLAMP_H / 2.0 - 0.006))
    bottom_flange = box_solid((0.024, CLAMP_W, 0.012), (0.018, 0.0, -CLAMP_H / 2.0 + 0.006))
    side_stiffener = box_solid((0.018, 0.012, CLAMP_H - 0.032), (0.016, CLAMP_W / 2.0 - 0.006, 0.0))
    clamp = (
        mount_block.union(plate)
        .union(top_flange)
        .union(bottom_flange)
        .union(side_stiffener)
        .union(side_stiffener.mirror("XZ"))
    )
    slot_x = CLAMP_BLOCK_LEN + CLAMP_PLATE_T / 2.0
    clamp = clamp.cut(box_solid((CLAMP_PLATE_T + 0.004, 0.016, 0.034), (slot_x, 0.0, 0.040)))
    clamp = clamp.cut(box_solid((CLAMP_PLATE_T + 0.004, 0.016, 0.034), (slot_x, 0.0, -0.040)))
    return clamp


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_utility_arm")

    wall_plate_mat = model.material("wall_plate_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    link_mat = model.material("painted_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    clamp_mat = model.material("clamp_plate_finish", rgba=(0.28, 0.29, 0.31, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(
            make_wall_plate(),
            "wall_plate",
            tolerance=MESH_TOL,
            angular_tolerance=MESH_ANGULAR_TOL,
        ),
        material=wall_plate_mat,
        name="wall_plate_body",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.060, 0.090, 0.240)),
        mass=3.0,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )

    root_link = model.part("root_link")
    root_link.visual(
        mesh_from_cadquery(
            make_root_link(),
            "root_link",
            tolerance=MESH_TOL,
            angular_tolerance=MESH_ANGULAR_TOL,
        ),
        material=link_mat,
        name="root_link_body",
    )
    root_link.inertial = Inertial.from_geometry(
        Box((ROOT_TO_DISTAL_X, 0.060, 0.070)),
        mass=1.6,
        origin=Origin(xyz=(ROOT_TO_DISTAL_X / 2.0, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(
            make_distal_link(),
            "distal_link",
            tolerance=MESH_TOL,
            angular_tolerance=MESH_ANGULAR_TOL,
        ),
        material=link_mat,
        name="distal_link_body",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((DISTAL_LEN, 0.056, 0.068)),
        mass=2.1,
        origin=Origin(xyz=(DISTAL_LEN / 2.0, 0.0, 0.0)),
    )

    clamp_plate = model.part("clamp_plate")
    clamp_plate.visual(
        mesh_from_cadquery(
            make_clamp_plate(),
            "clamp_plate",
            tolerance=MESH_TOL,
            angular_tolerance=MESH_ANGULAR_TOL,
        ),
        material=clamp_mat,
        name="clamp_plate_body",
    )
    clamp_plate.inertial = Inertial.from_geometry(
        Box((0.040, CLAMP_W, CLAMP_H)),
        mass=0.9,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_root",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=root_link,
        origin=Origin(xyz=(WALL_TO_ROOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.35, upper=0.70),
    )
    model.articulation(
        "root_to_distal",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=distal_link,
        origin=Origin(xyz=(ROOT_TO_DISTAL_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-2.20, upper=0.15),
    )
    model.articulation(
        "distal_to_clamp",
        ArticulationType.FIXED,
        parent=distal_link,
        child=clamp_plate,
        origin=Origin(xyz=(DISTAL_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    root_link = object_model.get_part("root_link")
    distal_link = object_model.get_part("distal_link")
    clamp_plate = object_model.get_part("clamp_plate")
    shoulder = object_model.get_articulation("wall_to_root")
    elbow = object_model.get_articulation("root_to_distal")

    wall_visual = wall_plate.get_visual("wall_plate_body")
    root_visual = root_link.get_visual("root_link_body")
    distal_visual = distal_link.get_visual("distal_link_body")
    clamp_visual = clamp_plate.get_visual("clamp_plate_body")

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
        "named_visuals_present",
        all(v is not None for v in (wall_visual, root_visual, distal_visual, clamp_visual)),
        "Expected one named visual on each major part.",
    )
    ctx.check(
        "serial_revolute_axes_are_planar",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and shoulder.axis == (0.0, 1.0, 0.0)
        and elbow.axis == (0.0, 1.0, 0.0),
        f"Expected both moving joints to be revolute about +Y, got {shoulder.axis} and {elbow.axis}.",
    )
    ctx.check(
        "joint_limits_span_working_range",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and shoulder.motion_limits.lower is not None
        and shoulder.motion_limits.upper is not None
        and elbow.motion_limits.lower is not None
        and elbow.motion_limits.upper is not None
        and shoulder.motion_limits.lower < 0.0 < shoulder.motion_limits.upper
        and elbow.motion_limits.lower < 0.0 <= elbow.motion_limits.upper,
        "Joint limits should allow a neutral forward pose and downward folding.",
    )

    ctx.expect_contact(root_link, wall_plate, name="shoulder_joint_has_bearing_contact")
    ctx.expect_contact(distal_link, root_link, name="elbow_joint_has_bearing_contact")
    ctx.expect_contact(clamp_plate, distal_link, name="terminal_plate_is_mounted_to_distal_link")

    ctx.expect_origin_gap(
        distal_link,
        root_link,
        axis="x",
        min_gap=ROOT_TO_DISTAL_X - 0.005,
        max_gap=ROOT_TO_DISTAL_X + 0.005,
        name="distal_joint_is_at_root_link_tip",
    )
    ctx.expect_origin_gap(
        clamp_plate,
        distal_link,
        axis="x",
        min_gap=DISTAL_LEN - 0.005,
        max_gap=DISTAL_LEN + 0.005,
        name="clamp_plate_is_at_distal_tip",
    )

    with ctx.pose({shoulder: -0.60, elbow: -0.90}):
        ctx.expect_contact(root_link, wall_plate, name="shoulder_joint_stays_connected_when_lowered")
        ctx.expect_contact(distal_link, root_link, name="elbow_joint_stays_connected_when_lowered")
        ctx.expect_gap(
            clamp_plate,
            wall_plate,
            axis="x",
            min_gap=0.040,
            name="clamp_plate_keeps_forward_clearance_from_wall_when_lowered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
