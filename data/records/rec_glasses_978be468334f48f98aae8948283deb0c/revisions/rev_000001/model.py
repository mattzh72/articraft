from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


FRAME_OUTER_W = 0.050
FRAME_OUTER_H = 0.034
LENS_OPEN_W = 0.044
LENS_OPEN_H = 0.028
FRAME_THICK = 0.004
LENS_THICK = 0.0024
LENS_CENTER_X = 0.033
OUTER_HINGE_X = 0.060
OUTER_HINGE_Z = 0.007
RIGHT_STACK_Y = 0.0025


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _frame_mesh(name: str):
    outer = rounded_rect_profile(FRAME_OUTER_W, FRAME_OUTER_H, 0.008, corner_segments=10)
    inner = rounded_rect_profile(LENS_OPEN_W, LENS_OPEN_H, 0.006, corner_segments=10)
    geom = ExtrudeWithHolesGeometry(outer, [inner], height=FRAME_THICK, center=True).rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def _temple_mesh(name: str, side_sign: float):
    profile = rounded_rect_profile(0.0065, 0.0019, 0.00075, corner_segments=5)
    geom = sweep_profile_along_spline(
        [
            (0.0006 * side_sign, 0.003, 0.0000),
            (0.0020 * side_sign, 0.018, 0.0012),
            (0.0045 * side_sign, 0.038, 0.0003),
            (0.0088 * side_sign, 0.058, -0.0085),
        ],
        profile=profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    return _save_mesh(name, geom)


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_reading_glasses")

    frame_black = model.material("frame_black", rgba=(0.08, 0.08, 0.09, 1.0))
    temple_black = model.material("temple_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.64, 0.68, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.78, 0.86, 0.96, 0.35))
    soft_tip = model.material("soft_tip", rgba=(0.16, 0.16, 0.18, 1.0))

    left_frame_mesh = _frame_mesh("left_lens_frame_v2")
    right_frame_mesh = _frame_mesh("right_lens_frame_v2")
    left_temple_mesh = _temple_mesh("left_temple_arm_v2", side_sign=-1.0)
    right_temple_mesh = _temple_mesh("right_temple_arm_v2", side_sign=1.0)

    left_front = model.part("left_front")
    left_front.visual(
        left_frame_mesh,
        origin=Origin(xyz=(-LENS_CENTER_X, 0.0, 0.0)),
        material=frame_black,
        name="frame_rim",
    )
    left_front.visual(
        Box((LENS_OPEN_W * 1.01, LENS_THICK, LENS_OPEN_H * 1.01)),
        origin=Origin(xyz=(-LENS_CENTER_X, 0.0, 0.0)),
        material=lens_clear,
        name="lens_glass",
    )
    left_front.visual(
        Box((0.011, 0.003, 0.005)),
        origin=Origin(xyz=(-0.0065, 0.0, 0.00525)),
        material=frame_black,
        name="bridge_upper_leaf",
    )
    left_front.visual(
        Box((0.011, 0.003, 0.005)),
        origin=Origin(xyz=(-0.0065, 0.0, -0.00525)),
        material=frame_black,
        name="bridge_lower_leaf",
    )
    left_front.visual(
        Cylinder(radius=0.0016, length=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.00525)),
        material=hinge_metal,
        name="bridge_upper_knuckle",
    )
    left_front.visual(
        Cylinder(radius=0.0016, length=0.0055),
        origin=Origin(xyz=(0.0, 0.0, -0.00525)),
        material=hinge_metal,
        name="bridge_lower_knuckle",
    )
    left_front.visual(
        Box((0.010, 0.003, 0.003)),
        origin=Origin(xyz=(-0.056, 0.0, OUTER_HINGE_Z + 0.003)),
        material=frame_black,
        name="outer_hinge_upper_leaf",
    )
    left_front.visual(
        Box((0.010, 0.003, 0.003)),
        origin=Origin(xyz=(-0.056, 0.0, OUTER_HINGE_Z - 0.003)),
        material=frame_black,
        name="outer_hinge_lower_leaf",
    )
    left_front.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, OUTER_HINGE_Z + 0.003)),
        material=hinge_metal,
        name="outer_hinge_upper_knuckle",
    )
    left_front.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, OUTER_HINGE_Z - 0.003)),
        material=hinge_metal,
        name="outer_hinge_lower_knuckle",
    )
    left_front.inertial = Inertial.from_geometry(
        Box((0.070, 0.010, 0.040)),
        mass=0.018,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
    )

    right_front = model.part("right_front")
    right_front.visual(
        right_frame_mesh,
        origin=Origin(xyz=(LENS_CENTER_X, RIGHT_STACK_Y, 0.0)),
        material=frame_black,
        name="frame_rim",
    )
    right_front.visual(
        Box((LENS_OPEN_W * 1.01, LENS_THICK, LENS_OPEN_H * 1.01)),
        origin=Origin(xyz=(LENS_CENTER_X, RIGHT_STACK_Y, 0.0)),
        material=lens_clear,
        name="lens_glass",
    )
    right_front.visual(
        Box((0.013, 0.004, 0.010)),
        origin=Origin(xyz=(0.0055, 0.0013, 0.0)),
        material=frame_black,
        name="bridge_center_leaf",
    )
    right_front.visual(
        Cylinder(radius=0.0016, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="bridge_center_knuckle",
    )
    right_front.visual(
        Box((0.010, 0.003, 0.003)),
        origin=Origin(xyz=(0.056, RIGHT_STACK_Y, OUTER_HINGE_Z + 0.003)),
        material=frame_black,
        name="outer_hinge_upper_leaf",
    )
    right_front.visual(
        Box((0.010, 0.003, 0.003)),
        origin=Origin(xyz=(0.056, RIGHT_STACK_Y, OUTER_HINGE_Z - 0.003)),
        material=frame_black,
        name="outer_hinge_lower_leaf",
    )
    right_front.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(OUTER_HINGE_X, RIGHT_STACK_Y, OUTER_HINGE_Z + 0.003)),
        material=hinge_metal,
        name="outer_hinge_upper_knuckle",
    )
    right_front.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(OUTER_HINGE_X, RIGHT_STACK_Y, OUTER_HINGE_Z - 0.003)),
        material=hinge_metal,
        name="outer_hinge_lower_knuckle",
    )
    right_front.inertial = Inertial.from_geometry(
        Box((0.070, 0.012, 0.040)),
        mass=0.018,
        origin=Origin(xyz=(0.030, 0.0013, 0.0)),
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_knuckle",
    )
    left_temple.visual(
        Box((0.006, 0.003, 0.003)),
        origin=Origin(xyz=(-0.001, 0.0015, 0.0)),
        material=temple_black,
        name="hinge_leaf",
    )
    left_temple.visual(left_temple_mesh, material=temple_black, name="temple_arm")
    left_temple.visual(
        Box((0.008, 0.014, 0.0035)),
        origin=Origin(xyz=(-0.0082, 0.055, -0.0078)),
        material=soft_tip,
        name="tip_pad",
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.018, 0.064, 0.016)),
        mass=0.010,
        origin=Origin(xyz=(-0.004, 0.032, -0.003)),
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Cylinder(radius=0.0015, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_knuckle",
    )
    right_temple.visual(
        Box((0.006, 0.003, 0.003)),
        origin=Origin(xyz=(0.001, 0.0015, 0.0)),
        material=temple_black,
        name="hinge_leaf",
    )
    right_temple.visual(right_temple_mesh, material=temple_black, name="temple_arm")
    right_temple.visual(
        Box((0.008, 0.014, 0.0035)),
        origin=Origin(xyz=(0.0082, 0.055, -0.0078)),
        material=soft_tip,
        name="tip_pad",
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.018, 0.064, 0.016)),
        mass=0.010,
        origin=Origin(xyz=(0.004, 0.032, -0.003)),
    )

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=left_front,
        child=right_front,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=left_front,
        child=left_temple,
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, OUTER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=5.0,
            lower=-math.radians(108.0),
            upper=0.05,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=right_front,
        child=right_temple,
        origin=Origin(xyz=(OUTER_HINGE_X, RIGHT_STACK_Y, OUTER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=5.0,
            lower=-0.05,
            upper=math.radians(108.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_front = object_model.get_part("left_front")
    right_front = object_model.get_part("right_front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")

    bridge_hinge = object_model.get_articulation("bridge_hinge")
    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")

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

    ctx.expect_contact(left_front, right_front, name="bridge_knuckles_contact")
    ctx.expect_contact(left_front, left_temple, name="left_outer_hinge_contact")
    ctx.expect_contact(right_front, right_temple, name="right_outer_hinge_contact")
    ctx.expect_overlap(
        left_front,
        right_front,
        axes="z",
        elem_a="lens_glass",
        elem_b="lens_glass",
        min_overlap=0.024,
        name="lenses_share_reading_height",
    )

    ctx.check(
        "bridge_axis_is_vertical",
        bridge_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={bridge_hinge.axis}",
    )
    ctx.check(
        "left_temple_axis_is_vertical",
        left_temple_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={left_temple_hinge.axis}",
    )
    ctx.check(
        "right_temple_axis_is_vertical",
        right_temple_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={right_temple_hinge.axis}",
    )
    ctx.check(
        "bridge_has_real_fold_range",
        bridge_hinge.motion_limits is not None and (bridge_hinge.motion_limits.upper or 0.0) > 1.8,
        details=f"limits={bridge_hinge.motion_limits}",
    )

    open_right_lens = ctx.part_element_world_aabb(right_front, elem="lens_glass")
    open_left_tip = ctx.part_element_world_aabb(left_temple, elem="tip_pad")
    open_right_tip = ctx.part_element_world_aabb(right_temple, elem="tip_pad")
    assert open_right_lens is not None
    assert open_left_tip is not None
    assert open_right_tip is not None
    open_right_lens_center = _aabb_center(open_right_lens)
    open_left_tip_center = _aabb_center(open_left_tip)
    open_right_tip_center = _aabb_center(open_right_tip)

    with ctx.pose({bridge_hinge: math.radians(68.0)}):
        ctx.expect_contact(left_front, right_front, name="bridge_stays_connected_when_folded")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_bridge_partially_folded")
        folded_right_lens = ctx.part_element_world_aabb(right_front, elem="lens_glass")
        assert folded_right_lens is not None
        folded_right_lens_center = _aabb_center(folded_right_lens)
        ctx.check(
            "bridge_rotation_moves_right_lens_inward",
            folded_right_lens_center[0] < open_right_lens_center[0] - 0.010
            and folded_right_lens_center[1] > open_right_lens_center[1] + 0.020,
            details=f"open={open_right_lens_center}, folded={folded_right_lens_center}",
        )

    with ctx.pose({left_temple_hinge: -math.radians(82.0), right_temple_hinge: math.radians(82.0)}):
        ctx.expect_contact(left_front, left_temple, name="left_hinge_stays_connected_when_folded")
        ctx.expect_contact(right_front, right_temple, name="right_hinge_stays_connected_when_folded")
        folded_left_tip = ctx.part_element_world_aabb(left_temple, elem="tip_pad")
        folded_right_tip = ctx.part_element_world_aabb(right_temple, elem="tip_pad")
        assert folded_left_tip is not None
        assert folded_right_tip is not None
        folded_left_tip_center = _aabb_center(folded_left_tip)
        folded_right_tip_center = _aabb_center(folded_right_tip)
        ctx.check(
            "left_temple_folds_toward_center",
            folded_left_tip_center[0] > open_left_tip_center[0] + 0.040
            and folded_left_tip_center[1] < open_left_tip_center[1] - 0.020,
            details=f"open={open_left_tip_center}, folded={folded_left_tip_center}",
        )
        ctx.check(
            "right_temple_folds_toward_center",
            folded_right_tip_center[0] < open_right_tip_center[0] - 0.040
            and folded_right_tip_center[1] < open_right_tip_center[1] - 0.020,
            details=f"open={open_right_tip_center}, folded={folded_right_tip_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
