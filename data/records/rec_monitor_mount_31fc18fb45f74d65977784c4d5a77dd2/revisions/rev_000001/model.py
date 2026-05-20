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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_T = 0.008
PLATE_W = 0.090
PLATE_H = 0.160
PLATE_CORNER = 0.010
PLATE_CENTER_X = -0.028
PLATE_HOLE_D = 0.010
PLATE_HOLE_PATTERN_Y = 0.028
PLATE_HOLE_PATTERN_Z = 0.050

JOINT_GAP = 0.012
LUG_T = 0.006
LUG_R = 0.024
HUB_R = 0.021

PRIMARY_LENGTH = 0.220
PRIMARY_ARM_W = 0.038
SECONDARY_LENGTH = 0.185
SECONDARY_ARM_W = 0.034

WRIST_TO_TILT = 0.060
YOKE_ARM_T = 0.008
YOKE_ARM_H = 0.060

HEAD_W = 0.110
HEAD_H = 0.095
HEAD_D = 0.024
HEAD_CENTER_X = 0.028
HEAD_TRUNNION_R = 0.009
HEAD_SIDE_CLEARANCE = 0.006
HEAD_TRUNNION_LEN = HEAD_W + (2.0 * HEAD_SIDE_CLEARANCE)
HEAD_TRUNNION_STUB_LEN = 0.008

PRIMARY_LIMITS = (-1.80, 1.80)
ELBOW_LIMITS = (-2.35, 2.35)
SWIVEL_LIMITS = (-1.60, 1.60)
TILT_LIMITS = (-0.95, 1.05)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY", origin=(cx, cy, cz - (length / 2.0)))
        .circle(radius)
        .extrude(length)
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    shape = (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    return shape.translate(center)


def _build_wall_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_T, PLATE_W, PLATE_H)
        .edges("|X")
        .fillet(PLATE_CORNER)
        .translate((PLATE_CENTER_X, 0.0, 0.0))
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-PLATE_HOLE_PATTERN_Y, -PLATE_HOLE_PATTERN_Z),
                (-PLATE_HOLE_PATTERN_Y, PLATE_HOLE_PATTERN_Z),
                (PLATE_HOLE_PATTERN_Y, -PLATE_HOLE_PATTERN_Z),
                (PLATE_HOLE_PATTERN_Y, PLATE_HOLE_PATTERN_Z),
            ]
        )
        .hole(PLATE_HOLE_D)
    )

    backbone = (
        cq.Workplane("XY")
        .box(0.018, 0.050, 0.070)
        .edges("|Z")
        .fillet(0.005)
        .translate((-0.019, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.030, 0.028, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((-0.009, 0.0, (JOINT_GAP + LUG_T) / 2.0))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.030, 0.028, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((-0.009, 0.0, -((JOINT_GAP + LUG_T) / 2.0)))
    )

    top_lug = _cylinder_z(LUG_R, LUG_T, (0.0, 0.0, (JOINT_GAP + LUG_T) / 2.0))
    bottom_lug = _cylinder_z(LUG_R, LUG_T, (0.0, 0.0, -(JOINT_GAP + LUG_T) / 2.0))

    return plate.union(backbone).union(top_rib).union(bottom_rib).union(top_lug).union(bottom_lug)


def _build_primary_link_shape() -> cq.Workplane:
    proximal_tongue = (
        cq.Workplane("XY")
        .box(0.028, 0.028, JOINT_GAP)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.014, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(PRIMARY_LENGTH - 0.072, PRIMARY_ARM_W, JOINT_GAP)
        .edges("|Z")
        .fillet(0.004)
        .translate((((PRIMARY_LENGTH - 0.072) / 2.0) + 0.024, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.036, 0.024, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((PRIMARY_LENGTH - 0.018, 0.0, (JOINT_GAP + LUG_T) / 2.0))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.036, 0.024, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((PRIMARY_LENGTH - 0.018, 0.0, -((JOINT_GAP + LUG_T) / 2.0)))
    )
    top_lug = _cylinder_z(LUG_R, LUG_T, (PRIMARY_LENGTH, 0.0, (JOINT_GAP + LUG_T) / 2.0))
    bottom_lug = _cylinder_z(
        LUG_R,
        LUG_T,
        (PRIMARY_LENGTH, 0.0, -(JOINT_GAP + LUG_T) / 2.0),
    )
    link = proximal_tongue.union(beam).union(top_rib).union(bottom_rib).union(top_lug).union(bottom_lug)

    lightening_slot = (
        cq.Workplane("XY")
        .box(0.108, 0.018, JOINT_GAP + 0.004)
        .translate((0.100, 0.0, 0.0))
    )
    return link.cut(lightening_slot)


def _build_secondary_link_shape() -> cq.Workplane:
    proximal_tongue = (
        cq.Workplane("XY")
        .box(0.028, 0.026, JOINT_GAP)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.014, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(SECONDARY_LENGTH - 0.072, SECONDARY_ARM_W, JOINT_GAP)
        .edges("|Z")
        .fillet(0.004)
        .translate((((SECONDARY_LENGTH - 0.072) / 2.0) + 0.024, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.034, 0.022, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((SECONDARY_LENGTH - 0.017, 0.0, (JOINT_GAP + LUG_T) / 2.0))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.034, 0.022, LUG_T + 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((SECONDARY_LENGTH - 0.017, 0.0, -((JOINT_GAP + LUG_T) / 2.0)))
    )
    top_lug = _cylinder_z(
        LUG_R,
        LUG_T,
        (SECONDARY_LENGTH, 0.0, (JOINT_GAP + LUG_T) / 2.0),
    )
    bottom_lug = _cylinder_z(
        LUG_R,
        LUG_T,
        (SECONDARY_LENGTH, 0.0, -(JOINT_GAP + LUG_T) / 2.0),
    )
    link = proximal_tongue.union(beam).union(top_rib).union(bottom_rib).union(top_lug).union(bottom_lug)

    lightening_slot = (
        cq.Workplane("XY")
        .box(0.082, 0.016, JOINT_GAP + 0.004)
        .translate((0.085, 0.0, 0.0))
    )
    return link.cut(lightening_slot)


def _build_head_yoke_shape() -> cq.Workplane:
    proximal_tongue = (
        cq.Workplane("XY")
        .box(0.028, 0.022, JOINT_GAP)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.014, 0.0, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.022, 0.020, JOINT_GAP)
        .edges("|Z")
        .fillet(0.0035)
        .translate((0.035, 0.0, 0.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.012, HEAD_TRUNNION_LEN + (2.0 * YOKE_ARM_T), 0.016)
        .translate((WRIST_TO_TILT - 0.026, 0.0, 0.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.030, YOKE_ARM_T, YOKE_ARM_H)
        .translate(
            (
                WRIST_TO_TILT - 0.015,
                (HEAD_TRUNNION_LEN / 2.0) + (YOKE_ARM_T / 2.0),
                0.0,
            )
        )
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.030, YOKE_ARM_T, YOKE_ARM_H)
        .translate(
            (
                WRIST_TO_TILT - 0.015,
                -((HEAD_TRUNNION_LEN / 2.0) + (YOKE_ARM_T / 2.0)),
                0.0,
            )
        )
    )
    return (
        proximal_tongue.union(stem)
        .union(rear_bridge)
        .union(right_arm)
        .union(left_arm)
    )


def _build_monitor_head_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HEAD_D, HEAD_W, HEAD_H)
        .edges("|X")
        .fillet(0.008)
        .translate((HEAD_CENTER_X, 0.0, 0.0))
    )
    body = (
        body.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.0375, -0.0375),
                (-0.0375, 0.0375),
                (0.0375, -0.0375),
                (0.0375, 0.0375),
            ]
        )
        .hole(0.0045, depth=HEAD_D + 0.006)
    )

    rear_spine = (
        cq.Workplane("XY")
        .box(0.010, 0.048, 0.056)
        .edges("|X")
        .fillet(0.003)
        .translate((0.006, 0.0, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.032)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.010, (HEAD_W / 2.0) + 0.004, 0.0))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.032)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.010, -((HEAD_W / 2.0) + 0.004), 0.0))
    )
    trunnion_right = _cylinder_y(
        HEAD_TRUNNION_R,
        HEAD_TRUNNION_STUB_LEN,
        (0.008, (HEAD_W / 2.0) + 0.008, 0.0),
    )
    trunnion_left = _cylinder_y(
        HEAD_TRUNNION_R,
        HEAD_TRUNNION_STUB_LEN,
        (0.008, -((HEAD_W / 2.0) + 0.008), 0.0),
    )

    return (
        body.union(rear_spine)
        .union(right_cheek)
        .union(left_cheek)
        .union(trunnion_right)
        .union(trunnion_left)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_monitor_arm")

    model.material("powder_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("aluminum", rgba=(0.71, 0.73, 0.75, 1.0))
    model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        origin=Origin(xyz=(PLATE_CENTER_X, 0.0, 0.0)),
        material="powder_steel",
        name="plate_panel",
    )
    wall_plate.visual(
        Box((0.018, 0.050, 0.070)),
        origin=Origin(xyz=(-0.019, 0.0, 0.0)),
        material="powder_steel",
        name="plate_backbone",
    )
    wall_plate.visual(
        Box((0.016, 0.028, 0.012)),
        origin=Origin(xyz=(-0.008, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="powder_steel",
        name="shoulder_top_web",
    )
    wall_plate.visual(
        Box((0.016, 0.028, 0.012)),
        origin=Origin(xyz=(-0.008, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="powder_steel",
        name="shoulder_bottom_web",
    )
    wall_plate.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(0.0, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="powder_steel",
        name="shoulder_top_lug",
    )
    wall_plate.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(0.0, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="powder_steel",
        name="shoulder_bottom_lug",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.050, PLATE_W, PLATE_H)),
        mass=1.15,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=0.016, length=JOINT_GAP + (2.0 * LUG_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="aluminum",
        name="primary_base_barrel",
    )
    primary_link.visual(
        Box((0.018, 0.024, JOINT_GAP)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material="aluminum",
        name="primary_base_neck",
    )
    primary_link.visual(
        Box((PRIMARY_LENGTH - 0.064, PRIMARY_ARM_W, JOINT_GAP)),
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        material="aluminum",
        name="primary_beam",
    )
    primary_link.visual(
        Box((0.028, 0.024, 0.012)),
        origin=Origin(xyz=(0.200, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="aluminum",
        name="primary_elbow_top_web",
    )
    primary_link.visual(
        Box((0.028, 0.024, 0.012)),
        origin=Origin(xyz=(0.200, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="aluminum",
        name="primary_elbow_bottom_web",
    )
    primary_link.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="aluminum",
        name="primary_elbow_top_lug",
    )
    primary_link.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="aluminum",
        name="primary_elbow_bottom_lug",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((PRIMARY_LENGTH, PRIMARY_ARM_W, JOINT_GAP + (2.0 * LUG_T))),
        mass=0.88,
        origin=Origin(xyz=(PRIMARY_LENGTH / 2.0, 0.0, 0.0)),
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=0.016, length=JOINT_GAP + (2.0 * LUG_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="aluminum",
        name="secondary_base_barrel",
    )
    secondary_link.visual(
        Box((0.018, 0.022, JOINT_GAP)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material="aluminum",
        name="secondary_base_neck",
    )
    secondary_link.visual(
        Box((SECONDARY_LENGTH - 0.064, SECONDARY_ARM_W, JOINT_GAP)),
        origin=Origin(xyz=(0.0905, 0.0, 0.0)),
        material="aluminum",
        name="secondary_beam",
    )
    secondary_link.visual(
        Box((0.026, 0.022, 0.012)),
        origin=Origin(xyz=(0.164, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="aluminum",
        name="secondary_wrist_top_web",
    )
    secondary_link.visual(
        Box((0.026, 0.022, 0.012)),
        origin=Origin(xyz=(0.164, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="aluminum",
        name="secondary_wrist_bottom_web",
    )
    secondary_link.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, (JOINT_GAP + LUG_T) / 2.0)),
        material="aluminum",
        name="secondary_wrist_top_lug",
    )
    secondary_link.visual(
        Cylinder(radius=LUG_R, length=LUG_T),
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, -((JOINT_GAP + LUG_T) / 2.0))),
        material="aluminum",
        name="secondary_wrist_bottom_lug",
    )
    secondary_link.inertial = Inertial.from_geometry(
        Box((SECONDARY_LENGTH, SECONDARY_ARM_W, JOINT_GAP + (2.0 * LUG_T))),
        mass=0.70,
        origin=Origin(xyz=(SECONDARY_LENGTH / 2.0, 0.0, 0.0)),
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        Cylinder(radius=0.016, length=JOINT_GAP + (2.0 * LUG_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="powder_steel",
        name="yoke_base_barrel",
    )
    head_yoke.visual(
        Box((0.018, 0.020, JOINT_GAP)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="powder_steel",
        name="yoke_stem",
    )
    head_yoke.visual(
        Box((0.018, HEAD_TRUNNION_LEN + (2.0 * YOKE_ARM_T), 0.016)),
        origin=Origin(xyz=(WRIST_TO_TILT - 0.022, 0.0, 0.0)),
        material="powder_steel",
        name="yoke_crossbar",
    )
    head_yoke.visual(
        Box((0.020, YOKE_ARM_T, YOKE_ARM_H)),
        origin=Origin(
            xyz=(
                WRIST_TO_TILT - 0.006,
                (HEAD_W / 2.0) + HEAD_SIDE_CLEARANCE + (YOKE_ARM_T / 2.0),
                0.0,
            )
        ),
        material="powder_steel",
        name="yoke_right_arm",
    )
    head_yoke.visual(
        Box((0.020, YOKE_ARM_T, YOKE_ARM_H)),
        origin=Origin(
            xyz=(
                WRIST_TO_TILT - 0.006,
                -((HEAD_W / 2.0) + HEAD_SIDE_CLEARANCE + (YOKE_ARM_T / 2.0)),
                0.0,
            )
        ),
        material="powder_steel",
        name="yoke_left_arm",
    )
    head_yoke.inertial = Inertial.from_geometry(
        Box((WRIST_TO_TILT + 0.020, HEAD_TRUNNION_LEN + (2.0 * YOKE_ARM_T), YOKE_ARM_H)),
        mass=0.34,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    monitor_head = model.part("monitor_head")
    monitor_head.visual(
        Box((HEAD_D, HEAD_W, HEAD_H)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material="graphite",
        name="head_body",
    )
    monitor_head.visual(
        Box((0.012, 0.052, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material="graphite",
        name="head_rear_spine",
    )
    monitor_head.visual(
        Box((0.012, 0.014, 0.032)),
        origin=Origin(xyz=(0.010, (HEAD_W / 2.0) + 0.005, 0.0)),
        material="graphite",
        name="head_right_cheek",
    )
    monitor_head.visual(
        Box((0.012, 0.014, 0.032)),
        origin=Origin(xyz=(0.010, -((HEAD_W / 2.0) + 0.005), 0.0)),
        material="graphite",
        name="head_left_cheek",
    )
    monitor_head.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(0.006, (HEAD_W / 2.0) + 0.008, 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="graphite",
        name="head_right_trunnion",
    )
    monitor_head.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(0.006, -((HEAD_W / 2.0) + 0.008), 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="graphite",
        name="head_left_trunnion",
    )
    monitor_head.inertial = Inertial.from_geometry(
        Box((HEAD_D + 0.012, HEAD_TRUNNION_LEN, HEAD_H)),
        mass=0.56,
        origin=Origin(xyz=(HEAD_CENTER_X, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.4,
            lower=PRIMARY_LIMITS[0],
            upper=PRIMARY_LIMITS[1],
        ),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )
    model.articulation(
        "secondary_to_yoke_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_yoke,
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=SWIVEL_LIMITS[0],
            upper=SWIVEL_LIMITS[1],
        ),
    )
    model.articulation(
        "yoke_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=monitor_head,
        origin=Origin(xyz=(WRIST_TO_TILT, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=TILT_LIMITS[0],
            upper=TILT_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    head_yoke = object_model.get_part("head_yoke")
    monitor_head = object_model.get_part("monitor_head")

    wall_to_primary = object_model.get_articulation("wall_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_yoke_swivel = object_model.get_articulation("secondary_to_yoke_swivel")
    yoke_to_head_tilt = object_model.get_articulation("yoke_to_head_tilt")

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
    ctx.allow_overlap(
        wall_plate,
        primary_link,
        reason="shoulder hinge simplifies nested clevis cheeks and central barrel around one hidden pivot pin",
    )
    ctx.allow_overlap(
        primary_link,
        secondary_link,
        reason="elbow hinge models interleaved clevis lugs and center barrel without explicit pin holes",
    )
    ctx.allow_overlap(
        secondary_link,
        head_yoke,
        reason="swivel wrist uses the same simplified nested hinge-barrel representation",
    )
    ctx.allow_overlap(
        head_yoke,
        monitor_head,
        reason="tilt trunnions sit inside the yoke arms in a simplified bearing-seat representation",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        wall_plate,
        primary_link,
        contact_tol=2e-4,
        name="wall plate supports primary link at shoulder",
    )
    ctx.expect_contact(
        primary_link,
        secondary_link,
        contact_tol=2e-4,
        name="primary and secondary links meet at elbow hub",
    )
    ctx.expect_contact(
        secondary_link,
        head_yoke,
        contact_tol=2e-4,
        name="secondary link carries swivel yoke",
    )
    ctx.expect_contact(
        head_yoke,
        monitor_head,
        contact_tol=2e-4,
        name="monitor head is cradled by tilt yoke",
    )

    with ctx.pose({wall_to_primary: 0.95}):
        secondary_pos = ctx.part_world_position(secondary_link)
        ctx.check(
            "primary joint positive rotates arm toward +Y",
            secondary_pos is not None and secondary_pos[1] > 0.12,
            f"secondary link origin should swing toward +Y, got {secondary_pos}",
        )

    with ctx.pose({primary_to_secondary: 1.00}):
        yoke_pos = ctx.part_world_position(head_yoke)
        ctx.check(
            "secondary joint positive sweeps wrist forward-left",
            yoke_pos is not None and yoke_pos[1] > 0.12 and yoke_pos[0] > 0.28,
            f"wrist origin should move into +Y while staying forward, got {yoke_pos}",
        )

    with ctx.pose({secondary_to_yoke_swivel: 1.00}):
        head_pos = ctx.part_world_position(monitor_head)
        ctx.check(
            "head swivel turns monitor head around wrist",
            head_pos is not None and head_pos[1] > 0.02,
            f"monitor head tilt axis should move toward +Y under swivel, got {head_pos}",
        )

    neutral_head_aabb = ctx.part_world_aabb(monitor_head)
    neutral_head_center_z = None
    if neutral_head_aabb is not None:
        neutral_head_center_z = (neutral_head_aabb[0][2] + neutral_head_aabb[1][2]) / 2.0

    with ctx.pose({yoke_to_head_tilt: 0.65}):
        tilted_head_aabb = ctx.part_world_aabb(monitor_head)
        tilted_center_z = None
        if tilted_head_aabb is not None:
            tilted_center_z = (tilted_head_aabb[0][2] + tilted_head_aabb[1][2]) / 2.0
        ctx.check(
            "tilt joint positive lifts the head body upward",
            neutral_head_center_z is not None
            and tilted_center_z is not None
            and tilted_center_z > neutral_head_center_z + 0.004,
            (
                "monitor head center should rise when tilted up; "
                f"neutral_z={neutral_head_center_z}, tilted_z={tilted_center_z}"
            ),
        )

    with ctx.pose(
        {
            wall_to_primary: 0.75,
            primary_to_secondary: -0.85,
            secondary_to_yoke_swivel: 1.10,
            yoke_to_head_tilt: 0.55,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in articulated display pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
