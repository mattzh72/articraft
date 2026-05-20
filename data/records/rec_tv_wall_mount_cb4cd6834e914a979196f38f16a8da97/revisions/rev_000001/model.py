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


PLATE_T = 0.012
PLATE_W = 0.120
PLATE_H = 0.220
SPINE_T = 0.018
INTERFACE_T = 0.008

ARM1_LEN = 0.235
ARM2_LEN = 0.190
ARM1_W = 0.042
ARM1_H = 0.024
ARM2_W = 0.038
ARM2_H = 0.022
ARM_WALL = 0.0045

SHOULDER_R = 0.029
ELBOW_R = 0.026
SWIVEL_R = 0.024

TILT_X = 0.050
EAR_T = 0.008
EAR_H = 0.052
BARREL_R = 0.0055
BARREL_LEN = 0.018

FRAME_OFFSET = 0.028
FRAME_T = 0.006
FRAME_W = 0.135
FRAME_H = 0.095
FRAME_BORDER = 0.014


def _span_box(
    x0: float,
    x1: float,
    size_y: float,
    size_z: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, size_y, size_z).translate(
        ((x0 + x1) / 2.0, y, z)
    )


def _disc_x(
    radius: float,
    thickness: float,
    *,
    x0: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(thickness).translate((x0, y, z))


def _wall_plate_shape() -> cq.Workplane:
    shape = _span_box(0.0, PLATE_T, PLATE_W, PLATE_H)
    shape = shape.union(_span_box(PLATE_T, PLATE_T + SPINE_T, 0.058, 0.150))
    shape = shape.union(_disc_x(SHOULDER_R, INTERFACE_T, x0=PLATE_T + SPINE_T))
    shape = shape.union(_span_box(PLATE_T, PLATE_T + 0.010, 0.052, 0.070))

    for y_pos in (-0.035, 0.035):
        for z_pos in (-0.070, 0.070):
            shape = shape.union(_disc_x(0.008, 0.003, x0=PLATE_T, y=y_pos, z=z_pos))

    return shape


def _arm_link_shape(
    *,
    length: float,
    prox_radius: float,
    dist_radius: float,
    width: float,
    height: float,
) -> cq.Workplane:
    shape = _disc_x(prox_radius, INTERFACE_T, x0=0.0)
    rail_y_size = width * 0.26
    rail_y = width * 0.21
    rail_z = height * 0.82
    rail_start = INTERFACE_T * 0.45
    rail_end = length - INTERFACE_T * 0.45
    shape = shape.union(
        _span_box(rail_start, rail_end, rail_y_size, rail_z, y=rail_y)
    )
    shape = shape.union(
        _span_box(rail_start, rail_end, rail_y_size, rail_z, y=-rail_y)
    )
    shape = shape.union(_span_box(0.008, 0.056, width * 0.76, height * 0.86))
    shape = shape.union(
        _span_box(length - 0.056, length - 0.008, width * 0.76, height * 0.86)
    )
    shape = shape.union(
        _span_box(length * 0.43, length * 0.57, width * 0.36, height * 0.52)
    )
    shape = shape.union(_disc_x(dist_radius, INTERFACE_T, x0=length - INTERFACE_T))
    return shape


def _swivel_carrier_shape() -> cq.Workplane:
    shape = _disc_x(SWIVEL_R, INTERFACE_T, x0=0.0)
    shape = shape.union(_span_box(INTERFACE_T * 0.75, TILT_X - 0.020, 0.022, 0.016))
    shape = shape.union(_span_box(TILT_X - 0.020, TILT_X - 0.010, 0.014, 0.034))
    ear_center_z = 0.013
    shape = shape.union(
        _span_box(TILT_X - 0.012, TILT_X + 0.004, BARREL_LEN + 0.004, 0.010, z=ear_center_z)
    )
    shape = shape.union(
        _span_box(TILT_X - 0.012, TILT_X + 0.004, BARREL_LEN + 0.004, 0.010, z=-ear_center_z)
    )
    return shape


def _head_frame_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XZ")
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0.0, -BARREL_LEN / 2.0, 0.0))
    )
    bridge = _span_box(0.008, FRAME_OFFSET + 0.001, 0.018, 0.012)
    ring = _span_box(FRAME_OFFSET, FRAME_OFFSET + FRAME_T, FRAME_W, FRAME_H)
    ring = ring.cut(
        _span_box(
            FRAME_OFFSET - 0.001,
            FRAME_OFFSET + FRAME_T + 0.001,
            FRAME_W - 2.0 * FRAME_BORDER,
            FRAME_H - 2.0 * FRAME_BORDER,
        )
    )

    vertical_bar = _span_box(
        FRAME_OFFSET,
        FRAME_OFFSET + FRAME_T,
        0.012,
        FRAME_H - 2.0 * FRAME_BORDER,
    )
    horizontal_bar = _span_box(
        FRAME_OFFSET,
        FRAME_OFFSET + FRAME_T,
        FRAME_W - 2.0 * FRAME_BORDER,
        0.012,
    )

    return barrel.union(bridge).union(ring).union(vertical_bar).union(horizontal_bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_mount")

    model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("satin_steel", rgba=(0.68, 0.71, 0.74, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="powder_black",
        name="wall_plate_mesh",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((PLATE_T + SPINE_T + INTERFACE_T, PLATE_W, PLATE_H)),
        mass=2.4,
        origin=Origin(xyz=((PLATE_T + SPINE_T + INTERFACE_T) / 2.0, 0.0, 0.0)),
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        mesh_from_cadquery(
            _arm_link_shape(
                length=ARM1_LEN,
                prox_radius=SHOULDER_R,
                dist_radius=ELBOW_R,
                width=ARM1_W,
                height=ARM1_H,
            ),
            "inner_arm",
        ),
        material="graphite",
        name="inner_arm_mesh",
    )
    inner_arm.inertial = Inertial.from_geometry(
        Box((ARM1_LEN, ARM1_W, ARM1_H)),
        mass=1.2,
        origin=Origin(xyz=(ARM1_LEN / 2.0, 0.0, 0.0)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        mesh_from_cadquery(
            _arm_link_shape(
                length=ARM2_LEN,
                prox_radius=ELBOW_R,
                dist_radius=SWIVEL_R,
                width=ARM2_W,
                height=ARM2_H,
            ),
            "outer_arm",
        ),
        material="graphite",
        name="outer_arm_mesh",
    )
    outer_arm.inertial = Inertial.from_geometry(
        Box((ARM2_LEN, ARM2_W, ARM2_H)),
        mass=0.95,
        origin=Origin(xyz=(ARM2_LEN / 2.0, 0.0, 0.0)),
    )

    swivel_carrier = model.part("swivel_carrier")
    swivel_carrier.visual(
        Cylinder(radius=SWIVEL_R, length=INTERFACE_T),
        origin=Origin(xyz=(INTERFACE_T / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="satin_steel",
        name="swivel_disc",
    )
    swivel_carrier.visual(
        Box((0.024, 0.022, 0.016)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material="satin_steel",
        name="swivel_stem",
    )
    swivel_carrier.visual(
        Box((0.012, 0.026, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.013)),
        material="satin_steel",
        name="tilt_upper_bridge",
    )
    swivel_carrier.visual(
        Box((0.012, 0.026, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, -0.013)),
        material="satin_steel",
        name="tilt_lower_bridge",
    )
    swivel_carrier.visual(
        Box((0.016, EAR_T, 0.040)),
        origin=Origin(xyz=(0.046, BARREL_LEN / 2.0 + EAR_T / 2.0, 0.0)),
        material="satin_steel",
        name="tilt_positive_ear",
    )
    swivel_carrier.visual(
        Box((0.016, EAR_T, 0.040)),
        origin=Origin(xyz=(0.046, -(BARREL_LEN / 2.0 + EAR_T / 2.0), 0.0)),
        material="satin_steel",
        name="tilt_negative_ear",
    )
    swivel_carrier.inertial = Inertial.from_geometry(
        Box((TILT_X + 0.006, 0.044, EAR_H)),
        mass=0.55,
        origin=Origin(xyz=((TILT_X + 0.006) / 2.0, 0.0, 0.0)),
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=BARREL_R, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="tilt_barrel",
    )
    head_frame.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material="powder_black",
        name="head_neck",
    )
    head_frame.visual(
        Box((0.015, 0.018, 0.012)),
        origin=Origin(xyz=(0.0215, 0.0, 0.0)),
        material="powder_black",
        name="head_bridge",
    )
    head_frame.visual(
        Box((FRAME_T, FRAME_BORDER, FRAME_H)),
        origin=Origin(
            xyz=(FRAME_OFFSET + FRAME_T / 2.0, (FRAME_W - FRAME_BORDER) / 2.0, 0.0)
        ),
        material="powder_black",
        name="frame_side_pos",
    )
    head_frame.visual(
        Box((FRAME_T, FRAME_BORDER, FRAME_H)),
        origin=Origin(
            xyz=(FRAME_OFFSET + FRAME_T / 2.0, -(FRAME_W - FRAME_BORDER) / 2.0, 0.0)
        ),
        material="powder_black",
        name="frame_side_neg",
    )
    head_frame.visual(
        Box((FRAME_T, FRAME_W, FRAME_BORDER)),
        origin=Origin(
            xyz=(FRAME_OFFSET + FRAME_T / 2.0, 0.0, (FRAME_H - FRAME_BORDER) / 2.0)
        ),
        material="powder_black",
        name="frame_top",
    )
    head_frame.visual(
        Box((FRAME_T, FRAME_W, FRAME_BORDER)),
        origin=Origin(
            xyz=(FRAME_OFFSET + FRAME_T / 2.0, 0.0, -(FRAME_H - FRAME_BORDER) / 2.0)
        ),
        material="powder_black",
        name="frame_bottom",
    )
    head_frame.visual(
        Box((FRAME_T, 0.012, FRAME_H - 2.0 * FRAME_BORDER)),
        origin=Origin(xyz=(FRAME_OFFSET + FRAME_T / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="frame_center_vertical",
    )
    head_frame.visual(
        Box((FRAME_T, FRAME_W - 2.0 * FRAME_BORDER, 0.012)),
        origin=Origin(xyz=(FRAME_OFFSET + FRAME_T / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="frame_center_horizontal",
    )
    head_frame.inertial = Inertial.from_geometry(
        Box((FRAME_OFFSET + FRAME_T, FRAME_W, FRAME_H)),
        mass=0.48,
        origin=Origin(xyz=((FRAME_OFFSET + FRAME_T) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_fold",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=inner_arm,
        origin=Origin(xyz=(PLATE_T + SPINE_T + INTERFACE_T, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.2,
            lower=-2.05,
            upper=2.05,
        ),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(ARM1_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.3,
            lower=-2.65,
            upper=2.65,
        ),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=swivel_carrier,
        origin=Origin(xyz=(ARM2_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_carrier,
        child=head_frame,
        origin=Origin(xyz=(TILT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    required_parts = {
        "wall_plate",
        "inner_arm",
        "outer_arm",
        "swivel_carrier",
        "head_frame",
    }
    required_joints = {
        "shoulder_fold",
        "elbow_fold",
        "head_swivel",
        "head_tilt",
    }
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "prompt_part_set_present",
        required_parts.issubset(part_names),
        f"missing parts: {sorted(required_parts - part_names)}",
    )
    ctx.check(
        "prompt_articulations_present",
        required_joints.issubset(joint_names),
        f"missing articulations: {sorted(required_joints - joint_names)}",
    )

    wall_plate = object_model.get_part("wall_plate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    swivel_carrier = object_model.get_part("swivel_carrier")
    head_frame = object_model.get_part("head_frame")

    shoulder_fold = object_model.get_articulation("shoulder_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_contact(wall_plate, inner_arm, name="shoulder_joint_contact")
    ctx.expect_contact(inner_arm, outer_arm, name="elbow_joint_contact")
    ctx.expect_contact(outer_arm, swivel_carrier, name="swivel_joint_contact")
    ctx.expect_contact(swivel_carrier, head_frame, name="tilt_joint_contact")

    def _aabb_center(part_obj) -> tuple[float, float, float] | None:
        aabb = ctx.part_world_aabb(part_obj)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    rest_inner = _aabb_center(inner_arm)
    rest_outer = _aabb_center(outer_arm)
    rest_head = _aabb_center(head_frame)

    with ctx.pose({shoulder_fold: 0.80}):
        posed_inner = _aabb_center(inner_arm)
        ctx.check(
            "shoulder_positive_rotates_arm_horizontally",
            rest_inner is not None
            and posed_inner is not None
            and posed_inner[1] > rest_inner[1] + 0.05
            and abs(posed_inner[2] - rest_inner[2]) < 0.01,
            f"rest={rest_inner}, posed={posed_inner}",
        )

    with ctx.pose({elbow_fold: 0.95}):
        posed_outer = _aabb_center(outer_arm)
        ctx.check(
            "elbow_positive_folds_outer_arm_in_plan",
            rest_outer is not None
            and posed_outer is not None
            and posed_outer[1] > rest_outer[1] + 0.03
            and abs(posed_outer[2] - rest_outer[2]) < 0.01,
            f"rest={rest_outer}, posed={posed_outer}",
        )

    with ctx.pose({head_swivel: 0.45}):
        posed_head = _aabb_center(head_frame)
        ctx.check(
            "swivel_positive_turns_head_about_vertical",
            rest_head is not None
            and posed_head is not None
            and posed_head[1] > rest_head[1] + 0.01
            and abs(posed_head[2] - rest_head[2]) < 0.01,
            f"rest={rest_head}, posed={posed_head}",
        )

    with ctx.pose({head_tilt: 0.22}):
        tilted_head = _aabb_center(head_frame)
        ctx.expect_contact(swivel_carrier, head_frame, name="tilt_joint_contact_at_up_tilt")
        ctx.check(
            "tilt_positive_raises_head_frame",
            rest_head is not None
            and tilted_head is not None
            and tilted_head[2] > rest_head[2] + 0.003,
            f"rest={rest_head}, tilted={tilted_head}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
