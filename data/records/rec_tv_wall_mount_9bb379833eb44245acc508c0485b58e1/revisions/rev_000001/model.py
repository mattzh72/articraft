from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isfinite

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


WALL_PLATE_W = 0.085
WALL_PLATE_H = 0.220
WALL_PLATE_T = 0.006
WALL_BLOCK_T = 0.022
WALL_NOSE_T = 0.010
SHOULDER_X = WALL_PLATE_T + WALL_BLOCK_T + WALL_NOSE_T

ARM_1_LEN = 0.160
ARM_2_LEN = 0.132
ARM_WIDTH = 0.048
ARM_THICK = 0.014

PAN_TO_TILT_X = 0.068


def _wall_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        WALL_PLATE_T,
        WALL_PLATE_W,
        WALL_PLATE_H,
        centered=(False, True, True),
    )
    hole_cutters = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.024, 0.072),
                (0.024, 0.072),
                (-0.024, -0.072),
                (0.024, -0.072),
            ]
        )
        .circle(0.0055)
        .extrude(WALL_PLATE_T + 0.006)
        .translate((-0.003, 0.0, 0.0))
    )
    plate = plate.cut(hole_cutters)

    block = cq.Workplane("XY").box(
        WALL_BLOCK_T,
        0.056,
        0.126,
        centered=(False, True, True),
    ).translate((WALL_PLATE_T, 0.0, 0.0))
    nose = cq.Workplane("YZ").circle(0.030).extrude(WALL_NOSE_T).translate(
        (WALL_PLATE_T + WALL_BLOCK_T, 0.0, 0.0)
    )
    return plate.union(block).union(nose)


def _arm_link_shape(length: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, ARM_WIDTH, ARM_THICK, centered=(False, True, True))
    window = cq.Workplane("XY").box(
        length - 0.050,
        ARM_WIDTH - 0.022,
        ARM_THICK + 0.002,
        centered=(False, True, True),
    ).translate((0.025, 0.0, 0.0))
    return body.cut(window)


def _pan_head_shape() -> cq.Workplane:
    rear_disk = cq.Workplane("YZ").circle(0.024).extrude(0.012)
    neck = cq.Workplane("XY").box(0.020, 0.040, 0.026, centered=(False, True, True)).translate(
        (0.012, 0.0, 0.0)
    )
    left_ear = cq.Workplane("XY").box(
        0.040,
        0.009,
        0.032,
        centered=(False, True, True),
    ).translate((0.024, 0.0215, 0.0))
    right_ear = cq.Workplane("XY").box(
        0.040,
        0.009,
        0.032,
        centered=(False, True, True),
    ).translate((0.024, -0.0215, 0.0))
    left_boss = cq.Workplane("YZ").circle(0.010).extrude(0.008).translate((0.056, 0.0215, 0.0))
    right_boss = cq.Workplane("YZ").circle(0.010).extrude(0.008).translate((0.056, -0.0215, 0.0))
    return (
        rear_disk.union(neck)
        .union(left_ear)
        .union(right_ear)
        .union(left_boss)
        .union(right_boss)
    )


def _tilt_head_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XY").box(0.016, 0.032, 0.016, centered=(True, True, True))
    neck = cq.Workplane("XY").box(0.020, 0.012, 0.014, centered=(False, True, True)).translate(
        (0.002, 0.0, 0.0)
    )
    spine = cq.Workplane("XY").box(0.030, 0.016, 0.050, centered=(False, True, True)).translate(
        (0.014, 0.0, 0.0)
    )
    plate = cq.Workplane("XY").box(0.006, 0.110, 0.088, centered=(False, True, True)).translate(
        (0.036, 0.0, 0.0)
    )
    hole_cutters = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.030, 0.022),
                (0.030, 0.022),
                (-0.030, -0.022),
                (0.030, -0.022),
            ]
        )
        .circle(0.004)
        .extrude(0.012)
        .translate((0.034, 0.0, 0.0))
    )
    return trunnion.union(neck).union(spine).union(plate).cut(hole_cutters)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_arm_tv_wall_bracket")

    model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("satin_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("zinc_hardware", rgba=(0.68, 0.70, 0.74, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="powder_black",
        name="wall_plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((WALL_PLATE_T + WALL_BLOCK_T + WALL_NOSE_T, WALL_PLATE_W, WALL_PLATE_H)),
        mass=1.6,
        origin=Origin(
            xyz=(
                (WALL_PLATE_T + WALL_BLOCK_T + WALL_NOSE_T) / 2.0,
                0.0,
                0.0,
            )
        ),
    )

    arm_link_1 = model.part("arm_link_1")
    arm_link_1.visual(
        mesh_from_cadquery(_arm_link_shape(ARM_1_LEN), "arm_link_1"),
        material="satin_graphite",
        name="arm_link_1_shell",
    )
    arm_link_1.inertial = Inertial.from_geometry(
        Box((ARM_1_LEN, ARM_WIDTH, ARM_THICK)),
        mass=0.75,
        origin=Origin(xyz=(ARM_1_LEN / 2.0, 0.0, 0.0)),
    )

    arm_link_2 = model.part("arm_link_2")
    arm_link_2.visual(
        mesh_from_cadquery(_arm_link_shape(ARM_2_LEN), "arm_link_2"),
        material="satin_graphite",
        name="arm_link_2_shell",
    )
    arm_link_2.inertial = Inertial.from_geometry(
        Box((ARM_2_LEN, ARM_WIDTH, ARM_THICK)),
        mass=0.62,
        origin=Origin(xyz=(ARM_2_LEN / 2.0, 0.0, 0.0)),
    )

    head_pan = model.part("head_pan")
    head_pan.visual(
        mesh_from_cadquery(_pan_head_shape(), "head_pan"),
        material="powder_black",
        name="head_pan_shell",
    )
    head_pan.inertial = Inertial.from_geometry(
        Box((0.076, 0.050, 0.048)),
        mass=0.34,
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
    )

    head_tilt = model.part("head_tilt")
    head_tilt.visual(
        mesh_from_cadquery(_tilt_head_shape(), "head_tilt"),
        material="zinc_hardware",
        name="tilt_plate",
    )
    head_tilt.inertial = Inertial.from_geometry(
        Box((0.040, 0.110, 0.088)),
        mass=0.28,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_arm1",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=arm_link_1,
        origin=Origin(xyz=(SHOULDER_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.4,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "arm1_to_arm2",
        ArticulationType.REVOLUTE,
        parent=arm_link_1,
        child=arm_link_2,
        origin=Origin(xyz=(ARM_1_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.6,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "arm2_to_head_pan",
        ArticulationType.REVOLUTE,
        parent=arm_link_2,
        child=head_pan,
        origin=Origin(xyz=(ARM_2_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-1.60,
            upper=1.60,
        ),
    )
    model.articulation(
        "head_pan_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_pan,
        child=head_tilt,
        origin=Origin(xyz=(PAN_TO_TILT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.20,
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

    wall_plate = object_model.get_part("wall_plate")
    arm_link_1 = object_model.get_part("arm_link_1")
    arm_link_2 = object_model.get_part("arm_link_2")
    head_pan = object_model.get_part("head_pan")
    head_tilt = object_model.get_part("head_tilt")

    wall_to_arm1 = object_model.get_articulation("wall_to_arm1")
    arm1_to_arm2 = object_model.get_articulation("arm1_to_arm2")
    arm2_to_head_pan = object_model.get_articulation("arm2_to_head_pan")
    head_pan_to_head_tilt = object_model.get_articulation("head_pan_to_head_tilt")

    joints = (
        wall_to_arm1,
        arm1_to_arm2,
        arm2_to_head_pan,
        head_pan_to_head_tilt,
    )
    ctx.check(
        "all bracket joints are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.articulation_type for j in joints]),
    )
    ctx.check(
        "yaw joints use vertical axes and tilt uses lateral axis",
        wall_to_arm1.axis == (0.0, 0.0, 1.0)
        and arm1_to_arm2.axis == (0.0, 0.0, 1.0)
        and arm2_to_head_pan.axis == (0.0, 0.0, 1.0)
        and head_pan_to_head_tilt.axis == (0.0, -1.0, 0.0),
        details=str([j.axis for j in joints]),
    )

    with ctx.pose(
        {
            wall_to_arm1: 0.0,
            arm1_to_arm2: 0.0,
            arm2_to_head_pan: 0.0,
            head_pan_to_head_tilt: 0.0,
        }
    ):
        ctx.expect_gap(
            head_tilt,
            wall_plate,
            axis="x",
            min_gap=0.200,
            name="mounting head projects forward from the wall plate",
        )
        ctx.expect_contact(
            arm_link_1,
            wall_plate,
            name="first arm is physically mounted to the wall plate",
        )

        rest_arm2_pos = ctx.part_world_position(arm_link_2)
        rest_head_pos = ctx.part_world_position(head_pan)
        rest_tilt_aabb = ctx.part_element_world_aabb(head_tilt, elem="tilt_plate")

    with ctx.pose({wall_to_arm1: 0.75}):
        swung_arm2_pos = ctx.part_world_position(arm_link_2)

    with ctx.pose({arm1_to_arm2: -0.95}):
        elbowed_head_pos = ctx.part_world_position(head_pan)

    with ctx.pose({arm2_to_head_pan: 0.95}):
        swiveled_tilt_pos = ctx.part_world_position(head_tilt)

    with ctx.pose({head_pan_to_head_tilt: 0.18}):
        tilted_aabb = ctx.part_element_world_aabb(head_tilt, elem="tilt_plate")

    ctx.check(
        "shoulder yaw swings the second link sideways",
        rest_arm2_pos is not None
        and swung_arm2_pos is not None
        and swung_arm2_pos[1] > rest_arm2_pos[1] + 0.10,
        details=f"rest={rest_arm2_pos}, swung={swung_arm2_pos}",
    )
    ctx.check(
        "elbow yaw repositions the head laterally",
        rest_head_pos is not None
        and elbowed_head_pos is not None
        and elbowed_head_pos[1] < rest_head_pos[1] - 0.05,
        details=f"rest={rest_head_pos}, elbowed={elbowed_head_pos}",
    )
    ctx.check(
        "pan swivel moves the tilt head around the swivel axis",
        rest_head_pos is not None
        and swiveled_tilt_pos is not None
        and abs(swiveled_tilt_pos[1]) > 0.04,
        details=f"rest_head={rest_head_pos}, swiveled_tilt={swiveled_tilt_pos}",
    )

    def _aabb_center_xz(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        cx = (min_corner[0] + max_corner[0]) / 2.0
        cz = (min_corner[2] + max_corner[2]) / 2.0
        if not (isfinite(cx) and isfinite(cz)):
            return None
        return cx, cz

    rest_center = _aabb_center_xz(rest_tilt_aabb)
    tilted_center = _aabb_center_xz(tilted_aabb)
    ctx.check(
        "tilt joint lifts the mounting plate center",
        rest_center is not None
        and tilted_center is not None
        and tilted_center[1] > rest_center[1] + 0.002,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
