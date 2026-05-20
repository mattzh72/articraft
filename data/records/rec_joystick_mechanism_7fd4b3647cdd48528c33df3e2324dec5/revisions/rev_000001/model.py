from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


BASE_RADIUS = 0.095
BASE_THICKNESS = 0.018
BASE_PEDESTAL_RADIUS = 0.032
BASE_PEDESTAL_HEIGHT = 0.016
BASE_SADDLE_SIZE = (0.060, 0.050, 0.016)
BASE_CHEEK_X = 0.055
BASE_CHEEK_THICKNESS = 0.012
BASE_CHEEK_DEPTH = 0.052
BASE_CHEEK_HEIGHT = 0.118

GIMBAL_CENTER_Z = 0.100

OUTER_PLATE_X = 0.060
OUTER_PLATE_THICKNESS = 0.010
OUTER_PLATE_Y = 0.024
OUTER_PLATE_HEIGHT = 0.022
OUTER_BRIDGE_X = 0.052
OUTER_BRIDGE_Y = 0.038
OUTER_BRIDGE_Z = -0.026
OUTER_BRIDGE_THICKNESS = 0.010
OUTER_WEB_X = 0.010
OUTER_WEB_Y = 0.042
OUTER_WEB_HEIGHT = 0.032
OUTER_WEB_CENTER_X = 0.022
OUTER_WEB_CENTER_Z = -0.013
OUTER_TRUNNION_RADIUS = 0.0075
OUTER_TRUNNION_LENGTH = 0.022
OUTER_TRUNNION_CENTER_X = 0.038

INNER_TRUNNION_RADIUS = 0.0065
INNER_TRUNNION_LENGTH = 0.010
INNER_TRUNNION_CENTER_Y = 0.014
INNER_EAR_X = 0.018
INNER_EAR_THICKNESS = 0.008
INNER_EAR_CENTER_Y = 0.010
INNER_EAR_HEIGHT = 0.036
INNER_EAR_CENTER_Z = 0.000
INNER_BRIDGE_X = 0.018
INNER_BRIDGE_Y = 0.020
INNER_BRIDGE_THICKNESS = 0.008
INNER_BRIDGE_CENTER_Z = 0.010

STEM_RADIUS = 0.010
STEM_LENGTH = 0.110
STEM_BOTTOM_Z = 0.012
STEM_FORWARD_OFFSET = 0.000

GRIP_SIZE = (0.040, 0.028, 0.074)
GRIP_BOTTOM_Z = 0.094
GRIP_FORWARD_OFFSET = 0.012
GRIP_CAP_RADIUS = 0.016
GRIP_CAP_Z = GRIP_BOTTOM_Z + GRIP_SIZE[2]

OUTER_LIMIT = 0.44
INNER_LIMIT = 0.44


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_control_stick")

    base_finish = model.material("base_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.42, 0.45, 0.48, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.09, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_finish,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=BASE_PEDESTAL_RADIUS, length=BASE_PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_PEDESTAL_HEIGHT / 2.0)),
        material=base_finish,
        name="pedestal",
    )
    base.visual(
        Box(BASE_SADDLE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_SADDLE_SIZE[2] / 2.0)),
        material=base_finish,
        name="saddle",
    )
    base.visual(
        Box((BASE_CHEEK_THICKNESS, BASE_CHEEK_DEPTH, BASE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(BASE_CHEEK_X, 0.0, BASE_THICKNESS + BASE_CHEEK_HEIGHT / 2.0),
        ),
        material=base_finish,
        name="left_base_cheek",
    )
    base.visual(
        Box((BASE_CHEEK_THICKNESS, BASE_CHEEK_DEPTH, BASE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(-BASE_CHEEK_X, 0.0, BASE_THICKNESS + BASE_CHEEK_HEIGHT / 2.0),
        ),
        material=base_finish,
        name="right_base_cheek",
    )

    outer = model.part("outer_gimbal")
    outer.visual(
        Box((OUTER_PLATE_X, OUTER_PLATE_THICKNESS, OUTER_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTER_PLATE_Y, 0.0)),
        material=frame_finish,
        name="front_outer_cheek",
    )
    outer.visual(
        Box((OUTER_PLATE_X, OUTER_PLATE_THICKNESS, OUTER_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, -OUTER_PLATE_Y, 0.0)),
        material=frame_finish,
        name="rear_outer_cheek",
    )
    outer.visual(
        Box((OUTER_BRIDGE_X, OUTER_BRIDGE_Y, OUTER_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_BRIDGE_Z)),
        material=frame_finish,
        name="outer_bridge",
    )
    outer.visual(
        Box((OUTER_WEB_X, OUTER_WEB_Y, OUTER_WEB_HEIGHT)),
        origin=Origin(xyz=(OUTER_WEB_CENTER_X, 0.0, OUTER_WEB_CENTER_Z)),
        material=frame_finish,
        name="left_outer_web",
    )
    outer.visual(
        Box((OUTER_WEB_X, OUTER_WEB_Y, OUTER_WEB_HEIGHT)),
        origin=Origin(xyz=(-OUTER_WEB_CENTER_X, 0.0, OUTER_WEB_CENTER_Z)),
        material=frame_finish,
        name="right_outer_web",
    )
    outer.visual(
        Cylinder(radius=OUTER_TRUNNION_RADIUS, length=OUTER_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(OUTER_TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=frame_finish,
        name="left_outer_trunnion",
    )
    outer.visual(
        Cylinder(radius=OUTER_TRUNNION_RADIUS, length=OUTER_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(-OUTER_TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=frame_finish,
        name="right_outer_trunnion",
    )

    inner = model.part("inner_gimbal")
    inner.visual(
        Box((INNER_EAR_X, INNER_EAR_THICKNESS, INNER_EAR_HEIGHT)),
        origin=Origin(xyz=(0.0, INNER_EAR_CENTER_Y, INNER_EAR_CENTER_Z)),
        material=frame_finish,
        name="left_inner_cheek",
    )
    inner.visual(
        Box((INNER_EAR_X, INNER_EAR_THICKNESS, INNER_EAR_HEIGHT)),
        origin=Origin(xyz=(0.0, -INNER_EAR_CENTER_Y, INNER_EAR_CENTER_Z)),
        material=frame_finish,
        name="right_inner_cheek",
    )
    inner.visual(
        Box((INNER_BRIDGE_X, INNER_BRIDGE_Y, INNER_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, INNER_BRIDGE_CENTER_Z)),
        material=frame_finish,
        name="inner_bridge",
    )
    inner.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, INNER_TRUNNION_CENTER_Y, 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=frame_finish,
        name="left_inner_trunnion",
    )
    inner.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -INNER_TRUNNION_CENTER_Y, 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=frame_finish,
        name="right_inner_trunnion",
    )
    inner.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_LENGTH),
        origin=Origin(
            xyz=(0.0, STEM_FORWARD_OFFSET, STEM_BOTTOM_Z + STEM_LENGTH / 2.0),
        ),
        material=frame_finish,
        name="stem",
    )
    inner.visual(
        Box(GRIP_SIZE),
        origin=Origin(
            xyz=(0.0, GRIP_FORWARD_OFFSET, GRIP_BOTTOM_Z + GRIP_SIZE[2] / 2.0),
        ),
        material=grip_finish,
        name="grip",
    )
    inner.visual(
        Sphere(radius=GRIP_CAP_RADIUS),
        origin=Origin(xyz=(0.0, GRIP_FORWARD_OFFSET, GRIP_CAP_Z)),
        material=grip_finish,
        name="grip_cap",
    )

    model.articulation(
        "base_to_outer_gimbal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-OUTER_LIMIT, upper=OUTER_LIMIT),
    )
    model.articulation(
        "outer_to_inner_gimbal",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-INNER_LIMIT, upper=INNER_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_gimbal")
    inner = object_model.get_part("inner_gimbal")
    outer_joint = object_model.get_articulation("base_to_outer_gimbal")
    inner_joint = object_model.get_articulation("outer_to_inner_gimbal")

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
        "part_tree_matches_nested_two_axis_gimbal",
        (
            outer_joint.parent == "base"
            and outer_joint.child == "outer_gimbal"
            and inner_joint.parent == "outer_gimbal"
            and inner_joint.child == "inner_gimbal"
        ),
        details=(
            f"got outer {outer_joint.parent}->{outer_joint.child}, "
            f"inner {inner_joint.parent}->{inner_joint.child}"
        ),
    )

    dot = sum(a * b for a, b in zip(outer_joint.axis, inner_joint.axis))
    outer_origin = ctx.part_world_position(outer)
    inner_origin = ctx.part_world_position(inner)
    origin_delta = None
    if outer_origin is not None and inner_origin is not None:
        origin_delta = max(abs(a - b) for a, b in zip(outer_origin, inner_origin))
    ctx.check(
        "gimbal_axes_are_orthogonal_and_intersecting",
        abs(dot) < 1e-9 and origin_delta is not None and origin_delta < 1e-9,
        details=f"axis_dot={dot}, origin_delta={origin_delta}, outer_origin={outer_origin}, inner_origin={inner_origin}",
    )

    ctx.expect_contact(base, outer, name="outer_gimbal_is_borne_by_base_cheeks")
    ctx.expect_contact(outer, inner, name="inner_gimbal_is_borne_by_outer_cheeks")
    ctx.expect_overlap(inner, base, axes="xy", min_overlap=0.02, elem_a="grip", name="grip_stays_over_base")

    with ctx.pose({outer_joint: OUTER_LIMIT}):
        ctx.fail_if_parts_overlap_in_current_pose(name="outer_axis_clear_at_limit")

    with ctx.pose({inner_joint: INNER_LIMIT}):
        ctx.fail_if_parts_overlap_in_current_pose(name="inner_axis_clear_at_limit")

    with ctx.pose({outer_joint: OUTER_LIMIT * 0.8, inner_joint: INNER_LIMIT * 0.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_tilt_clear")

    rest_grip_center = _aabb_center(ctx.part_element_world_aabb(inner, elem="grip"))
    with ctx.pose({outer_joint: OUTER_LIMIT}):
        outer_deflected_center = _aabb_center(ctx.part_element_world_aabb(inner, elem="grip"))
    with ctx.pose({inner_joint: INNER_LIMIT}):
        inner_deflected_center = _aabb_center(ctx.part_element_world_aabb(inner, elem="grip"))

    outer_motion_ok = (
        rest_grip_center is not None
        and outer_deflected_center is not None
        and outer_deflected_center[1] > rest_grip_center[1] + 0.035
        and abs(outer_deflected_center[0] - rest_grip_center[0]) < 0.015
    )
    inner_motion_ok = (
        rest_grip_center is not None
        and inner_deflected_center is not None
        and inner_deflected_center[0] > rest_grip_center[0] + 0.035
        and abs(inner_deflected_center[1] - rest_grip_center[1]) < 0.015
    )

    ctx.check(
        "outer_axis_tilts_grip_fore_aft",
        outer_motion_ok,
        details=f"rest={rest_grip_center}, deflected={outer_deflected_center}",
    )
    ctx.check(
        "inner_axis_tilts_grip_side_to_side",
        inner_motion_ok,
        details=f"rest={rest_grip_center}, deflected={inner_deflected_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
