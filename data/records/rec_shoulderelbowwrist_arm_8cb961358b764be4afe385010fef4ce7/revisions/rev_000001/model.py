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


BASE_PLATE_LENGTH = 0.22
BASE_PLATE_WIDTH = 0.18
BASE_PLATE_THICKNESS = 0.018
TOWER_LENGTH = 0.11
TOWER_WIDTH = 0.09
TOWER_HEIGHT = 0.125

SHOULDER_X = 0.03
SHOULDER_Z = BASE_PLATE_THICKNESS + TOWER_HEIGHT + 0.035
SHOULDER_OUTER_WIDTH = 0.060
SHOULDER_GAP = 0.046
SHOULDER_HUB_RADIUS = 0.045
SHOULDER_HUB_WIDTH = SHOULDER_GAP

UPPER_ARM_LENGTH = 0.215
UPPER_ARM_NECK_WIDTH = 0.028
UPPER_ARM_BODY_WIDTH = 0.044
UPPER_ARM_NECK_HEIGHT = 0.052
UPPER_ARM_BODY_HEIGHT = 0.060
ELBOW_OUTER_WIDTH = 0.052
ELBOW_GAP = 0.040
ELBOW_HUB_RADIUS = 0.033
ELBOW_HUB_WIDTH = ELBOW_GAP

FOREARM_LENGTH = 0.175
FOREARM_NECK_WIDTH = 0.022
FOREARM_BODY_WIDTH = 0.032
FOREARM_NECK_HEIGHT = 0.038
FOREARM_BODY_HEIGHT = 0.045
WRIST_OUTER_WIDTH = 0.042
WRIST_GAP = 0.032
WRIST_HUB_RADIUS = 0.022
WRIST_HUB_WIDTH = WRIST_GAP

WRIST_BODY_WIDTH = 0.022
WRIST_BODY_HEIGHT = 0.030
WRIST_STAGE_LENGTH = 0.067


def _y_cylinder(radius: float, width: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(width / 2.0, both=True)


def _x_cylinder(radius: float, length: float, x0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def _x_box(length: float, width: float, height: float, x0: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x0, 0.0, z0))
    )


def _forward_half_hub(radius: float, width: float) -> cq.Workplane:
    return _y_cylinder(radius, width).intersect(_x_box(2.0 * radius + 0.002, width + 0.002, 2.0 * radius + 0.002, x0=0.0))


def _rear_yoke(x_end: float, length: float, outer_width: float, gap: float, height: float, z0: float = 0.0) -> cq.Workplane:
    cheek_thickness = (outer_width - gap) / 2.0
    cheek_x0 = x_end - length
    cheek_offset = gap / 2.0 + cheek_thickness / 2.0
    left = _x_box(length, cheek_thickness, height, cheek_x0, z0=z0).translate((0.0, cheek_offset, 0.0))
    right = _x_box(length, cheek_thickness, height, cheek_x0, z0=z0).translate((0.0, -cheek_offset, 0.0))
    web_length = min(length * 0.45, 0.012)
    back_web = _x_box(web_length, outer_width, height * 0.58, cheek_x0, z0=z0)
    return left.union(right).union(back_web)


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)
        .translate((0.01, 0.0, BASE_PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    tower = (
        cq.Workplane("XY")
        .box(TOWER_LENGTH, TOWER_WIDTH, TOWER_HEIGHT)
        .translate((-0.030, 0.0, BASE_PLATE_THICKNESS + TOWER_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )

    shoulder_spine = _x_box(0.044, 0.040, 0.082, x0=SHOULDER_X - 0.068, z0=SHOULDER_Z - 0.004)
    shoulder_yoke = _rear_yoke(
        x_end=SHOULDER_X,
        length=0.028,
        outer_width=SHOULDER_OUTER_WIDTH,
        gap=SHOULDER_GAP,
        height=0.088,
        z0=SHOULDER_Z,
    )

    return plate.union(tower).union(shoulder_spine).union(shoulder_yoke)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _forward_half_hub(SHOULDER_HUB_RADIUS, SHOULDER_HUB_WIDTH)
    root_neck = _x_box(0.050, UPPER_ARM_NECK_WIDTH, UPPER_ARM_NECK_HEIGHT, x0=0.0)
    body = _x_box(0.108, 0.040, UPPER_ARM_BODY_HEIGHT, x0=0.046)
    taper = _x_box(0.036, 0.036, 0.048, x0=0.154)
    elbow_yoke = _rear_yoke(
        x_end=UPPER_ARM_LENGTH,
        length=0.028,
        outer_width=ELBOW_OUTER_WIDTH,
        gap=ELBOW_GAP,
        height=0.064,
    )

    return shoulder_hub.union(root_neck).union(body).union(taper).union(elbow_yoke)


def _forearm_shape() -> cq.Workplane:
    elbow_hub = _forward_half_hub(ELBOW_HUB_RADIUS, ELBOW_HUB_WIDTH)
    root_neck = _x_box(0.042, FOREARM_NECK_WIDTH, FOREARM_NECK_HEIGHT, x0=0.0)
    body = _x_box(0.088, 0.030, FOREARM_BODY_HEIGHT, x0=0.040)
    taper = _x_box(0.032, 0.026, 0.034, x0=0.123)
    wrist_yoke = _rear_yoke(
        x_end=FOREARM_LENGTH,
        length=0.022,
        outer_width=WRIST_OUTER_WIDTH,
        gap=WRIST_GAP,
        height=0.042,
    )

    return elbow_hub.union(root_neck).union(body).union(taper).union(wrist_yoke)


def _wrist_housing_shape() -> cq.Workplane:
    wrist_hub = _forward_half_hub(WRIST_HUB_RADIUS, WRIST_HUB_WIDTH)
    root_neck = _x_box(0.022, 0.016, 0.024, x0=0.0)
    housing = _x_box(0.042, WRIST_BODY_WIDTH, WRIST_BODY_HEIGHT, x0=0.018)
    flange = _x_cylinder(0.018, 0.014, x0=0.056)

    return wrist_hub.union(root_neck).union(housing).union(flange)


def _tool_plate_shape() -> cq.Workplane:
    stem = _x_box(0.018, 0.014, 0.014, x0=0.056)
    plate = _x_box(0.020, 0.030, 0.030, x0=0.072)
    return stem.union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_wristed_arm")

    model.material("pedestal_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("upper_arm_blue", rgba=(0.36, 0.46, 0.58, 1.0))
    model.material("forearm_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("wrist_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("tool_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base_pedestal"), material="pedestal_dark", name="pedestal")
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, SHOULDER_Z + 0.055)),
        mass=18.0,
        origin=Origin(xyz=(0.01, 0.0, (SHOULDER_Z + 0.055) / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm_link"),
        material="upper_arm_blue",
        name="upper_arm_shell",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.020, ELBOW_OUTER_WIDTH, 0.080)),
        mass=6.2,
        origin=Origin(xyz=((UPPER_ARM_LENGTH + 0.020) / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_link"),
        material="forearm_silver",
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.018, WRIST_OUTER_WIDTH, 0.060)),
        mass=3.4,
        origin=Origin(xyz=((FOREARM_LENGTH + 0.018) / 2.0, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_wrist_housing_shape(), "wrist_housing"),
        material="wrist_graphite",
        name="wrist_housing",
    )
    wrist.visual(
        mesh_from_cadquery(_tool_plate_shape(), "tool_plate"),
        material="tool_steel",
        name="tool_plate",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((WRIST_STAGE_LENGTH, 0.034, 0.034)),
        mass=1.1,
        origin=Origin(xyz=(WRIST_STAGE_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-1.0, upper=1.2),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.7, lower=-1.3, upper=1.35),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.2, lower=-1.4, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("base_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

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
        base,
        upper_arm,
        reason="The shoulder is modeled as a captured trunnion nested inside the tower yoke, so the simplified hub and housing intentionally share volume at the base joint.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "serial revolute axes are parallel pitch joints",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist_joint.axis == (0.0, -1.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist_joint.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_joint: 0.0}):
        ctx.expect_contact(base, upper_arm, contact_tol=0.0015, name="base supports upper arm at shoulder")
        ctx.expect_contact(upper_arm, forearm, contact_tol=0.0015, name="upper arm supports forearm at elbow")
        ctx.expect_contact(forearm, wrist, contact_tol=0.0015, name="forearm supports wrist at wrist joint")

    def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_joint: 0.0}):
        neutral_wrist_origin = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.8, elbow: 0.0, wrist_joint: 0.0}):
        raised_wrist_origin = ctx.part_world_position(wrist)
    ctx.check(
        "shoulder positive motion lifts downstream chain",
        neutral_wrist_origin is not None
        and raised_wrist_origin is not None
        and raised_wrist_origin[2] > neutral_wrist_origin[2] + 0.10,
        details=f"neutral={neutral_wrist_origin}, raised={raised_wrist_origin}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0, wrist_joint: 0.0}):
        straight_wrist_origin = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.35, elbow: 0.85, wrist_joint: 0.0}):
        bent_wrist_origin = ctx.part_world_position(wrist)
    ctx.check(
        "elbow positive motion folds forearm upward",
        straight_wrist_origin is not None
        and bent_wrist_origin is not None
        and bent_wrist_origin[2] > straight_wrist_origin[2] + 0.05,
        details=f"straight={straight_wrist_origin}, bent={bent_wrist_origin}",
    )

    with ctx.pose({shoulder: 0.30, elbow: 0.45, wrist_joint: 0.0}):
        neutral_tool_z = _aabb_center_z(ctx.part_element_world_aabb(wrist, elem="tool_plate"))
    with ctx.pose({shoulder: 0.30, elbow: 0.45, wrist_joint: 0.80}):
        lifted_tool_z = _aabb_center_z(ctx.part_element_world_aabb(wrist, elem="tool_plate"))
    ctx.check(
        "wrist positive motion tips tool plate upward",
        neutral_tool_z is not None and lifted_tool_z is not None and lifted_tool_z > neutral_tool_z + 0.02,
        details=f"neutral_z={neutral_tool_z}, lifted_z={lifted_tool_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
