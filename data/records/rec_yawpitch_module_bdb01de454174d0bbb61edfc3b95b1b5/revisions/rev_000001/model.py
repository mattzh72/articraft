from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_X = 0.18
BASE_PLATE_Y = 0.14
BASE_PLATE_Z = 0.014

TOWER_X = 0.102
TOWER_Y = 0.082
TOWER_Z = 0.100

PEDESTAL_COLLAR_RADIUS = 0.055
PEDESTAL_COLLAR_Z = 0.018
YAW_AXIS_Z = BASE_PLATE_Z + TOWER_Z + PEDESTAL_COLLAR_Z

YAW_DISK_RADIUS = 0.062
YAW_DISK_Z = 0.024

CHEEK_THICKNESS = 0.012
CHEEK_GAP = 0.088
CHEEK_OFFSET = CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0
CHEEK_BOSS_RADIUS = 0.017

PITCH_AXIS_X = 0.018
PITCH_AXIS_Z = 0.092

TRUNNION_RADIUS = 0.009
SHAFT_RADIUS = 0.0075
TRUNNION_COLLAR_RADIUS = 0.014
TRUNNION_COLLAR_THICKNESS = 0.004
SHAFT_LENGTH = CHEEK_GAP - 2.0 * TRUNNION_COLLAR_THICKNESS

YOKE_ARM_THICKNESS = 0.012
YOKE_OUTER_WIDTH = 0.076
YOKE_ARM_CENTER_Y = YOKE_OUTER_WIDTH / 2.0 - YOKE_ARM_THICKNESS / 2.0


def _pedestal_shape():
    base = (
        cq.Workplane("XY")
        .box(BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.060, -0.042),
                (-0.060, 0.042),
                (0.060, -0.042),
                (0.060, 0.042),
            ]
        )
        .hole(0.012)
    )

    tower = (
        cq.Workplane("XY")
        .box(TOWER_X, TOWER_Y, TOWER_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.004)
        .translate((0.0, 0.0, BASE_PLATE_Z))
    )

    collar = (
        cq.Workplane("XY")
        .circle(PEDESTAL_COLLAR_RADIUS)
        .extrude(PEDESTAL_COLLAR_Z)
        .edges(">Z")
        .fillet(0.003)
        .translate((0.0, 0.0, BASE_PLATE_Z + TOWER_Z))
    )

    return base.union(tower).union(collar)


def _yaw_cheek():
    cheek_profile = [
        (0.000, 0.020),
        (0.028, 0.020),
        (0.030, 0.032),
        (0.030, 0.088),
        (0.024, 0.104),
        (0.010, 0.112),
        (-0.002, 0.104),
        (-0.006, 0.088),
        (-0.006, 0.030),
    ]
    cheek = (
        cq.Workplane("XZ")
        .moveTo(*cheek_profile[0])
        .polyline(cheek_profile[1:])
        .close()
        .extrude(CHEEK_THICKNESS / 2.0, both=True)
    )
    return cheek


def _yaw_base_shape():
    disk = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(YAW_DISK_Z)
        .edges(">Z")
        .fillet(0.003)
    )

    cheek = _yaw_cheek()
    left_cheek = cheek.translate((0.0, -CHEEK_OFFSET, 0.0))
    right_cheek = cheek.translate((0.0, CHEEK_OFFSET, 0.0))

    spindle_hub = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(0.010)
        .edges(">Z")
        .fillet(0.002)
        .translate((0.0, 0.0, YAW_DISK_Z))
    )

    return disk.union(spindle_hub).union(left_cheek).union(right_cheek)


def _pitch_yoke_shape():
    shaft = (
        cq.Workplane("XZ")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH / 2.0, both=True)
    )

    left_collar = (
        cq.Workplane("XZ")
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_THICKNESS / 2.0, both=True)
        .translate((0.0, -(CHEEK_GAP / 2.0 - TRUNNION_COLLAR_THICKNESS / 2.0), 0.0))
    )
    right_collar = (
        cq.Workplane("XZ")
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_THICKNESS / 2.0, both=True)
        .translate((0.0, CHEEK_GAP / 2.0 - TRUNNION_COLLAR_THICKNESS / 2.0, 0.0))
    )

    yoke = shaft.union(left_collar).union(right_collar)
    for arm_y in (-YOKE_ARM_CENTER_Y, YOKE_ARM_CENTER_Y):
        top_rail = (
            cq.Workplane("XY")
            .box(0.060, YOKE_ARM_THICKNESS, 0.014, centered=(False, True, True))
            .translate((-0.002, arm_y, -0.012))
        )
        front_drop = (
            cq.Workplane("XY")
            .box(0.012, YOKE_ARM_THICKNESS, 0.034, centered=(False, True, True))
            .translate((0.046, arm_y, -0.034))
        )
        yoke = yoke.union(top_rail).union(front_drop)

    front_bar = (
        cq.Workplane("XZ")
        .center(0.052, -0.034)
        .circle(0.007)
        .extrude(0.033, both=True)
    )
    lower_crossmember = (
        cq.Workplane("XY")
        .box(0.018, YOKE_OUTER_WIDTH - 0.004, 0.010, centered=(False, True, True))
        .translate((0.030, 0.0, -0.050))
    )

    return yoke.union(front_bar).union(lower_crossmember)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_yaw_pitch_module")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.23, 0.24, 0.26, 1.0))
    yaw_paint = model.material("yaw_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.68, 0.71, 0.74, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_shell"),
        material=pedestal_paint,
        name="pedestal_shell",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        mesh_from_cadquery(_yaw_base_shape(), "yaw_base_shell"),
        material=yaw_paint,
        name="yaw_base_shell",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_pitch_yoke_shape(), "pitch_yoke_shell"),
        material=yoke_finish,
        name="pitch_yoke_shell",
    )

    model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.8,
            lower=-2.6,
            upper=2.6,
        ),
    )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.85,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    pedestal = object_model.get_part("pedestal")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check(
        "yaw joint uses a vertical axis",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint uses a horizontal lateral axis",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.allow_overlap(
        pitch_yoke,
        yaw_base,
        reason=(
            "The pitch yoke uses a zero-clearance bearing-seat representation "
            "against the yaw cheek faces; the intended relationship is rolling "
            "contact at the pivot rather than free gap."
        ),
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_gap(
            yaw_base,
            pedestal,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="yaw base seats on pedestal collar",
        )
        ctx.expect_overlap(
            yaw_base,
            pedestal,
            axes="xy",
            min_overlap=0.080,
            name="yaw base stays centered over the tower",
        )
        ctx.expect_contact(
            pitch_yoke,
            yaw_base,
            contact_tol=1e-4,
            name="pitch yoke remains carried by the yaw cheeks",
        )

    rest_pos = ctx.part_world_position(pitch_yoke)
    with ctx.pose({yaw_joint: 0.8}):
        yawed_pos = ctx.part_world_position(pitch_yoke)
    ctx.check(
        "positive yaw swings the pitch stage around the tower",
        rest_pos is not None
        and yawed_pos is not None
        and yawed_pos[1] > rest_pos[1] + 0.010
        and abs(yawed_pos[2] - rest_pos[2]) < 0.002,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    rest_aabb = ctx.part_world_aabb(pitch_yoke)
    with ctx.pose({pitch_joint: 0.8}):
        raised_aabb = ctx.part_world_aabb(pitch_yoke)
        ctx.expect_contact(
            pitch_yoke,
            yaw_base,
            contact_tol=1e-4,
            name="pitch trunnion stays seated at raised pose",
        )
    ctx.check(
        "positive pitch lifts the yoke nose",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.030
        and raised_aabb[0][0] < rest_aabb[0][0] - 0.002,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
