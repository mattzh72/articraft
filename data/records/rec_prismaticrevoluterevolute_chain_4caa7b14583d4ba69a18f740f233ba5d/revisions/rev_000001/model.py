from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.30
BASE_WIDTH = 0.20
BASE_THICK = 0.03

UPRIGHT_X = 0.028
UPRIGHT_Y = 0.026
UPRIGHT_HEIGHT = 0.58
UPRIGHT_CENTER_Y = 0.068
UPRIGHT_CENTER_X = 0.0

TOP_BAR_Z = 0.028
TOP_BAR_WIDTH = 0.162
BACK_SPINE_X = 0.022
BACK_SPINE_Y = 0.080
BACK_SPINE_Z = 0.38
BACK_SPINE_CENTER_X = -0.040

RAIL_X = 0.020
RAIL_Y = 0.012
RAIL_Z = 0.44
RAIL_CENTER_X = 0.024
RAIL_CENTER_Y = 0.049
RAIL_CENTER_Z = 0.315

GUIDE_SHOE_X = 0.022
GUIDE_SHOE_Y = 0.012
GUIDE_SHOE_Z = 0.18
GUIDE_SHOE_CENTER_X = 0.012
GUIDE_SHOE_CENTER_Y = 0.037

BODY_BLOCK_X = 0.052
BODY_BLOCK_Y = 0.060
BODY_BLOCK_Z = 0.110
BODY_BLOCK_CENTER_X = 0.028

FRONT_PLATE_X = 0.034
FRONT_PLATE_Y = 0.094
FRONT_PLATE_Z = 0.145
FRONT_PLATE_CENTER_X = 0.055

CLEVIS_EAR_X = 0.024
CLEVIS_EAR_Y = 0.012
CLEVIS_EAR_Z = 0.058
CLEVIS_CENTER_X = 0.096
CLEVIS_CENTER_Y = 0.025

SLIDE_HOME_Z = 0.210
SLIDE_TRAVEL = 0.220

SHOULDER_BARREL_RADIUS = 0.018
SHOULDER_BARREL_LENGTH = 0.038

ARM_ROOT_X = 0.034
ARM_ROOT_Y = 0.050
ARM_ROOT_Z = 0.050
ARM_ROOT_CENTER_X = 0.035

ARM_BEAM_X = 0.160
ARM_BEAM_Y = 0.034
ARM_BEAM_Z = 0.030
ARM_BEAM_CENTER_X = 0.116

ARM_RIB_X = 0.100
ARM_RIB_Y = 0.022
ARM_RIB_Z = 0.018
ARM_RIB_CENTER_X = 0.102
ARM_RIB_CENTER_Z = -0.020

DISTAL_EAR_X = 0.028
DISTAL_EAR_Y = 0.012
DISTAL_EAR_Z = 0.052
DISTAL_EAR_CENTER_X = 0.206
DISTAL_EAR_CENTER_Y = 0.020

ELBOW_X = 0.206

TIP_BARREL_RADIUS = 0.014
TIP_BARREL_LENGTH = 0.028

TIP_ROOT_X = 0.036
TIP_ROOT_Y = 0.040
TIP_ROOT_Z = 0.044
TIP_ROOT_CENTER_X = 0.034

TIP_BODY_X = 0.108
TIP_BODY_Y = 0.032
TIP_BODY_Z = 0.032
TIP_BODY_CENTER_X = 0.074

TIP_SLOT_X = 0.062
TIP_SLOT_Y = 0.024
TIP_SLOT_Z = 0.012
TIP_SLOT_CENTER_X = 0.090


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def _support_frame_body() -> cq.Workplane:
    base = _box((BASE_DEPTH, BASE_WIDTH, BASE_THICK), (0.020, 0.0, BASE_THICK / 2.0))
    left_upright = _box(
        (UPRIGHT_X, UPRIGHT_Y, UPRIGHT_HEIGHT),
        (UPRIGHT_CENTER_X, UPRIGHT_CENTER_Y, BASE_THICK + (UPRIGHT_HEIGHT / 2.0)),
    )
    right_upright = _box(
        (UPRIGHT_X, UPRIGHT_Y, UPRIGHT_HEIGHT),
        (UPRIGHT_CENTER_X, -UPRIGHT_CENTER_Y, BASE_THICK + (UPRIGHT_HEIGHT / 2.0)),
    )
    top_bar = _box(
        (UPRIGHT_X, TOP_BAR_WIDTH, TOP_BAR_Z),
        (UPRIGHT_CENTER_X, 0.0, BASE_THICK + UPRIGHT_HEIGHT - (TOP_BAR_Z / 2.0)),
    )
    back_spine = _box(
        (BACK_SPINE_X, BACK_SPINE_Y, BACK_SPINE_Z),
        (BACK_SPINE_CENTER_X, 0.0, BASE_THICK + (BACK_SPINE_Z / 2.0)),
    )
    front_feet = _fuse_all(
        [
            _box((0.070, 0.050, 0.018), (0.100, 0.060, 0.009)),
            _box((0.070, 0.050, 0.018), (0.100, -0.060, 0.009)),
        ]
    )
    return _fuse_all([base, left_upright, right_upright, top_bar, back_spine, front_feet])


def _guide_rails() -> cq.Workplane:
    left_rail = _box(
        (RAIL_X, RAIL_Y, RAIL_Z),
        (RAIL_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z),
    )
    right_rail = _box(
        (RAIL_X, RAIL_Y, RAIL_Z),
        (RAIL_CENTER_X, -RAIL_CENTER_Y, RAIL_CENTER_Z),
    )
    return _fuse_all([left_rail, right_rail])


def _carriage_body() -> cq.Workplane:
    saddle = _box(
        (BODY_BLOCK_X, BODY_BLOCK_Y, BODY_BLOCK_Z),
        (BODY_BLOCK_CENTER_X, 0.0, 0.0),
    )
    front_plate = _box(
        (FRONT_PLATE_X, FRONT_PLATE_Y, FRONT_PLATE_Z),
        (FRONT_PLATE_CENTER_X, 0.0, 0.0),
    )
    rear_bridge = _box((0.020, 0.090, 0.050), (0.002, 0.0, 0.0))
    return _fuse_all([saddle, front_plate, rear_bridge])


def _guide_shoes() -> cq.Workplane:
    left_shoe = _box(
        (GUIDE_SHOE_X, GUIDE_SHOE_Y, GUIDE_SHOE_Z),
        (GUIDE_SHOE_CENTER_X, GUIDE_SHOE_CENTER_Y, 0.0),
    )
    right_shoe = _box(
        (GUIDE_SHOE_X, GUIDE_SHOE_Y, GUIDE_SHOE_Z),
        (GUIDE_SHOE_CENTER_X, -GUIDE_SHOE_CENTER_Y, 0.0),
    )
    return _fuse_all([left_shoe, right_shoe])


def _shoulder_clevis() -> cq.Workplane:
    left_ear = _box(
        (CLEVIS_EAR_X, CLEVIS_EAR_Y, CLEVIS_EAR_Z),
        (CLEVIS_CENTER_X, CLEVIS_CENTER_Y, 0.0),
    )
    right_ear = _box(
        (CLEVIS_EAR_X, CLEVIS_EAR_Y, CLEVIS_EAR_Z),
        (CLEVIS_CENTER_X, -CLEVIS_CENTER_Y, 0.0),
    )
    left_strut = _box((0.018, CLEVIS_EAR_Y, 0.046), (0.081, CLEVIS_CENTER_Y, 0.0))
    right_strut = _box((0.018, CLEVIS_EAR_Y, 0.046), (0.081, -CLEVIS_CENTER_Y, 0.0))
    return _fuse_all([left_ear, right_ear, left_strut, right_strut])


def _arm_body() -> cq.Workplane:
    root = _box((ARM_ROOT_X, ARM_ROOT_Y, ARM_ROOT_Z), (ARM_ROOT_CENTER_X, 0.0, 0.0))
    beam = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, -0.022),
                (0.050, -0.023),
                (0.135, -0.019),
                (0.196, -0.013),
                (0.196, 0.013),
                (0.138, 0.019),
                (0.054, 0.021),
                (0.018, 0.024),
            ]
        )
        .close()
        .extrude(ARM_BEAM_Y / 2.0, both=True)
    )
    lower_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.040, -0.010),
                (0.110, -0.024),
                (0.164, -0.016),
                (0.164, -0.006),
                (0.040, 0.000),
            ]
        )
        .close()
        .extrude(ARM_RIB_Y / 2.0, both=True)
    )
    return _fuse_all([root, beam, lower_rib])


def _distal_fork() -> cq.Workplane:
    left_ear = _box(
        (DISTAL_EAR_X, DISTAL_EAR_Y, DISTAL_EAR_Z),
        (DISTAL_EAR_CENTER_X, DISTAL_EAR_CENTER_Y, 0.0),
    )
    right_ear = _box(
        (DISTAL_EAR_X, DISTAL_EAR_Y, DISTAL_EAR_Z),
        (DISTAL_EAR_CENTER_X, -DISTAL_EAR_CENTER_Y, 0.0),
    )
    return _fuse_all([left_ear, right_ear])


def _tip_body() -> cq.Workplane:
    root = _box((TIP_ROOT_X, TIP_ROOT_Y, TIP_ROOT_Z), (TIP_ROOT_CENTER_X, 0.0, 0.0))
    bridge = _box((0.056, TIP_BODY_Y, TIP_BODY_Z), (0.070, 0.0, 0.0))
    upper_tine = _box((0.054, 0.018, 0.010), (0.105, 0.0, 0.011))
    lower_tine = _box((0.054, 0.018, 0.010), (0.105, 0.0, -0.011))
    return _fuse_all([root, bridge, upper_tine, lower_tine])


def _tip_root_tongue() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.030, 0.028, 0.036)
        .translate((0.015, 0.0, 0.0))
        .faces(">X")
        .edges()
        .chamfer(0.005)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_slide")

    model.material("frame_graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("rail_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.48, 0.13, 1.0))
    model.material("arm_gray", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("tip_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_support_frame_body(), "support_frame_body"),
        material="frame_graphite",
        name="frame_body",
    )
    support_frame.visual(
        mesh_from_cadquery(_guide_rails(), "support_frame_guide_rails"),
        material="rail_aluminum",
        name="guide_rails",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, UPRIGHT_HEIGHT + BASE_THICK)),
        mass=14.0,
        origin=Origin(xyz=(0.02, 0.0, (UPRIGHT_HEIGHT + BASE_THICK) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body(), "carriage_body"),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_guide_shoes(), "carriage_guide_shoes"),
        material="rail_aluminum",
        name="guide_shoes",
    )
    carriage.visual(
        mesh_from_cadquery(_shoulder_clevis(), "carriage_shoulder_clevis"),
        material="carriage_orange",
        name="shoulder_clevis",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.11, 0.10, GUIDE_SHOE_Z)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=SHOULDER_BARREL_RADIUS, length=SHOULDER_BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="tip_steel",
        name="shoulder_barrel",
    )
    shoulder_link.visual(
        mesh_from_cadquery(_arm_body(), "shoulder_link_body"),
        material="arm_gray",
        name="arm_body",
    )
    shoulder_link.visual(
        mesh_from_cadquery(_distal_fork(), "shoulder_link_distal_fork"),
        material="tip_steel",
        name="distal_fork",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((0.25, 0.05, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    tip_link = model.part("tip_link")
    tip_link.visual(
        mesh_from_cadquery(_tip_root_tongue(), "articulated_tip_root_tongue"),
        material="tip_steel",
        name="tip_root_tongue",
    )
    tip_link.visual(
        mesh_from_cadquery(_tip_body(), "articulated_tip_body"),
        material="tip_steel",
        name="tip_body",
    )
    tip_link.inertial = Inertial.from_geometry(
        Box((0.12, 0.04, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=500.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder_link,
        origin=Origin(xyz=(CLEVIS_CENTER_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.15,
            effort=45.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "shoulder_to_tip",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=tip_link,
        origin=Origin(xyz=(ELBOW_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=1.25,
            effort=20.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    def element_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    carriage = object_model.get_part("carriage")
    shoulder_link = object_model.get_part("shoulder_link")
    tip_link = object_model.get_part("tip_link")

    slide = object_model.get_articulation("frame_to_carriage")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    tip_joint = object_model.get_articulation("shoulder_to_tip")

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

    ctx.expect_contact(
        carriage,
        support_frame,
        elem_a="guide_shoes",
        elem_b="guide_rails",
        name="carriage guide shoes stay mounted to the rear frame rails",
    )
    ctx.expect_contact(
        shoulder_link,
        carriage,
        elem_a="shoulder_barrel",
        elem_b="shoulder_clevis",
        name="shoulder barrel seats inside the carriage clevis",
    )
    ctx.expect_contact(
        tip_link,
        shoulder_link,
        elem_a="tip_root_tongue",
        elem_b="distal_fork",
        name="articulated tip tongue seats inside the distal fork",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            support_frame,
            elem_a="guide_shoes",
            elem_b="guide_rails",
            name="guide shoes remain on the rails at full lift",
        )
        ctx.expect_overlap(
            carriage,
            support_frame,
            axes="z",
            elem_a="guide_shoes",
            elem_b="guide_rails",
            min_overlap=0.17,
            name="guide shoes retain tall rail engagement at full lift",
        )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        carriage_raised = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic carriage moves upward",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.18,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    with ctx.pose({slide: 0.08}):
        tip_level = element_center(tip_link, "tip_body")
    with ctx.pose({slide: 0.08, shoulder_joint: 0.85}):
        tip_shoulder_lifted = element_center(tip_link, "tip_body")
    ctx.check(
        "shoulder joint pitches the arm upward",
        tip_level is not None
        and tip_shoulder_lifted is not None
        and tip_shoulder_lifted[2] > tip_level[2] + 0.10,
        details=f"level={tip_level}, lifted={tip_shoulder_lifted}",
    )

    with ctx.pose({slide: 0.08, shoulder_joint: 0.40}):
        tip_straight = element_center(tip_link, "tip_body")
    with ctx.pose({slide: 0.08, shoulder_joint: 0.40, tip_joint: 0.95}):
        tip_curled = element_center(tip_link, "tip_body")
    ctx.check(
        "distal revolute joint curls the tip upward",
        tip_straight is not None
        and tip_curled is not None
        and tip_curled[2] > tip_straight[2] + 0.04,
        details=f"straight={tip_straight}, curled={tip_curled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
