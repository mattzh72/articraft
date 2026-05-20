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


BACKPLATE_W = 0.160
BACKPLATE_H = 0.320
BACKPLATE_T = 0.010
MOUNT_SLOT_LEN = 0.040
MOUNT_SLOT_DIA = 0.010
MOUNT_SLOT_Z = 0.110

SPINE_W = 0.040
SPINE_D = 0.024
SPINE_H = 0.250
SPINE_Y = 0.022

POD_SUPPORT_W = 0.054
POD_SUPPORT_D = 0.018
POD_SUPPORT_H = 0.034
POD_SUPPORT_Y = 0.036
PIVOT_Y = 0.045
POD_Z_OFFSET = 0.085

HINGE_TAB_LEN = 0.028
HINGE_TAB_D = 0.008
HINGE_TAB_T = 0.024

ROOT_BLOCK_LEN = 0.042
ROOT_BLOCK_D = 0.018
ROOT_BLOCK_T = 0.018
ROOT_BLOCK_X = 0.034
ROOT_BLOCK_Y = 0.016

ARM_BEAM_LEN = 0.090
ARM_BEAM_D = 0.014
ARM_BEAM_T = 0.010
ARM_BEAM_X = 0.090
ARM_BEAM_Y = 0.027

TIP_NECK_LEN = 0.018
TIP_NECK_D = 0.018
TIP_NECK_T = 0.010
TIP_NECK_X = 0.137
TIP_NECK_Y = 0.036

END_PLATE_T = 0.008
END_PLATE_W = 0.040
END_PLATE_H = 0.078
END_PLATE_X = 0.146
END_PLATE_Y = 0.042
END_PLATE_HOLE_R = 0.005
END_PLATE_HOLE_Z = 0.021

TONGUE_LEN = 0.030
TONGUE_D = 0.014
TONGUE_T = 0.004
TONGUE_X = 0.018
TONGUE_Y = 0.006

OPEN_ANGLE_CHECK = 0.85


def _pod_mount_shape(z_center: float) -> cq.Workplane:
    pod_block = cq.Workplane("XY").box(POD_SUPPORT_W, POD_SUPPORT_D, POD_SUPPORT_H).translate(
        (0.0, POD_SUPPORT_Y, z_center)
    )
    face_pad = cq.Workplane("XY").box(0.034, 0.004, 0.028).translate((0.0, PIVOT_Y - 0.002, z_center))
    return pod_block.union(face_pad)


def _backplate_shape() -> cq.Workplane:
    backplate = (
        cq.Workplane("XY")
        .box(BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)
        .translate((0.0, BACKPLATE_T / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.012)
    )
    slot_cutters = (
        cq.Workplane("XZ")
        .pushPoints([(0.0, MOUNT_SLOT_Z), (0.0, -MOUNT_SLOT_Z)])
        .slot2D(MOUNT_SLOT_LEN, MOUNT_SLOT_DIA, 90)
        .extrude(BACKPLATE_T + 0.004, both=True)
        .translate((0.0, BACKPLATE_T / 2.0, 0.0))
    )
    backplate = backplate.cut(slot_cutters)

    spine = cq.Workplane("XY").box(SPINE_W, SPINE_D, SPINE_H).translate((0.0, SPINE_Y, 0.0))
    web_right = cq.Workplane("XY").box(0.012, 0.016, 0.220).translate((0.019, 0.015, 0.0))
    web_left = cq.Workplane("XY").box(0.012, 0.016, 0.220).translate((-0.019, 0.015, 0.0))

    upper_pod = _pod_mount_shape(POD_Z_OFFSET)
    lower_pod = _pod_mount_shape(-POD_Z_OFFSET)

    return backplate.union(spine).union(web_right).union(web_left).union(upper_pod).union(lower_pod)


def _branch_shape(hand: float) -> cq.Workplane:
    hinge_tab = cq.Workplane("XY").box(HINGE_TAB_LEN, HINGE_TAB_D, HINGE_TAB_T).translate(
        (hand * (HINGE_TAB_LEN / 2.0), HINGE_TAB_D / 2.0, 0.0)
    )
    tongue = cq.Workplane("XY").box(TONGUE_LEN, TONGUE_D, TONGUE_T).translate(
        (hand * TONGUE_X, TONGUE_Y, 0.0)
    )
    root_block = cq.Workplane("XY").box(ROOT_BLOCK_LEN, ROOT_BLOCK_D, ROOT_BLOCK_T).translate(
        (hand * ROOT_BLOCK_X, ROOT_BLOCK_Y, 0.0)
    )
    beam = cq.Workplane("XY").box(ARM_BEAM_LEN, ARM_BEAM_D, ARM_BEAM_T).translate(
        (hand * ARM_BEAM_X, ARM_BEAM_Y, 0.0)
    )
    tip_neck = cq.Workplane("XY").box(TIP_NECK_LEN, TIP_NECK_D, TIP_NECK_T).translate(
        (hand * TIP_NECK_X, TIP_NECK_Y, 0.0)
    )
    end_plate = cq.Workplane("XY").box(END_PLATE_T, END_PLATE_W, END_PLATE_H).translate(
        (hand * END_PLATE_X, END_PLATE_Y, 0.0)
    )

    plate_holes = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, END_PLATE_HOLE_Z), (0.0, -END_PLATE_HOLE_Z)])
        .circle(END_PLATE_HOLE_R)
        .extrude(END_PLATE_T + 0.006, both=True)
        .translate((hand * END_PLATE_X, END_PLATE_Y, 0.0))
    )

    return (
        hinge_tab.union(tongue)
        .union(root_block)
        .union(beam)
        .union(tip_neck)
        .union(end_plate)
        .cut(plate_holes)
    )


def _aabb_close(aabb_a, aabb_b, tol: float) -> bool:
    if aabb_a is None or aabb_b is None:
        return False
    return all(
        abs(aabb_a[bound][axis] - aabb_b[bound][axis]) <= tol
        for bound in range(2)
        for axis in range(3)
    )


def _aabb_changed(aabb_a, aabb_b, min_delta: float) -> bool:
    if aabb_a is None or aabb_b is None:
        return False
    return any(
        abs(aabb_a[bound][axis] - aabb_b[bound][axis]) >= min_delta
        for bound in range(2)
        for axis in range(3)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_two_branch_bracket")

    model.material("powder_coat_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("cast_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        material="powder_coat_dark",
        name="backplate_shell",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((BACKPLATE_W, 0.084, BACKPLATE_H)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(_branch_shape(1.0), "upper_branch"),
        material="cast_aluminum",
        name="upper_branch_shell",
    )
    upper_branch.inertial = Inertial.from_geometry(
        Box((0.160, 0.052, END_PLATE_H)),
        mass=0.95,
        origin=Origin(xyz=(0.078, 0.008, 0.0)),
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(_branch_shape(-1.0), "lower_branch"),
        material="cast_aluminum",
        name="lower_branch_shell",
    )
    lower_branch.inertial = Inertial.from_geometry(
        Box((0.160, 0.052, END_PLATE_H)),
        mass=0.95,
        origin=Origin(xyz=(-0.078, 0.008, 0.0)),
    )

    model.articulation(
        "upper_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=upper_branch,
        origin=Origin(xyz=(0.0, PIVOT_Y, POD_Z_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.5),
    )
    model.articulation(
        "lower_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=lower_branch,
        origin=Origin(xyz=(0.0, PIVOT_Y, -POD_Z_OFFSET)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_hinge = object_model.get_articulation("upper_branch_hinge")
    lower_hinge = object_model.get_articulation("lower_branch_hinge")

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
        "upper_hinge_axis_is_vertical",
        upper_hinge.axis == (0.0, 0.0, 1.0),
        f"expected +Z revolute axis for upper pod, got {upper_hinge.axis}",
    )
    ctx.check(
        "lower_hinge_axis_is_vertical",
        lower_hinge.axis == (0.0, 0.0, -1.0),
        f"expected -Z revolute axis for lower pod, got {lower_hinge.axis}",
    )
    ctx.check(
        "pod_hinges_are_separate",
        abs(upper_hinge.origin.xyz[2] - lower_hinge.origin.xyz[2]) >= 0.16,
        (
            "upper and lower branch pivots should be vertically separated so the bracket "
            "reads as two independent pod joints rather than one shared drive shaft"
        ),
    )

    ctx.expect_contact(backplate, upper_branch, name="upper_branch_seated_in_pod")
    ctx.expect_contact(backplate, lower_branch, name="lower_branch_seated_in_pod")
    ctx.expect_gap(
        upper_branch,
        lower_branch,
        axis="z",
        min_gap=0.07,
        name="branch_stack_has_vertical_clearance",
    )

    rest_upper_aabb = ctx.part_world_aabb(upper_branch)
    rest_lower_aabb = ctx.part_world_aabb(lower_branch)

    with ctx.pose({upper_hinge: OPEN_ANGLE_CHECK}):
        upper_open_aabb = ctx.part_world_aabb(upper_branch)
        lower_steady_aabb = ctx.part_world_aabb(lower_branch)
        ctx.expect_contact(backplate, upper_branch, name="upper_branch_keeps_pod_contact_open")
    ctx.check(
        "upper_hinge_moves_only_upper_branch",
        _aabb_changed(rest_upper_aabb, upper_open_aabb, 0.01)
        and _aabb_close(rest_lower_aabb, lower_steady_aabb, 1e-6),
        "opening the upper hinge should move the upper branch while leaving the lower branch fixed",
    )

    with ctx.pose({lower_hinge: OPEN_ANGLE_CHECK}):
        lower_open_aabb = ctx.part_world_aabb(lower_branch)
        upper_steady_aabb = ctx.part_world_aabb(upper_branch)
        ctx.expect_contact(backplate, lower_branch, name="lower_branch_keeps_pod_contact_open")
    ctx.check(
        "lower_hinge_moves_only_lower_branch",
        _aabb_changed(rest_lower_aabb, lower_open_aabb, 0.01)
        and _aabb_close(rest_upper_aabb, upper_steady_aabb, 1e-6),
        "opening the lower hinge should move the lower branch while leaving the upper branch fixed",
    )

    with ctx.pose({upper_hinge: OPEN_ANGLE_CHECK, lower_hinge: OPEN_ANGLE_CHECK}):
        ctx.expect_gap(
            upper_branch,
            lower_branch,
            axis="z",
            min_gap=0.07,
            name="branches_clear_each_other_in_service_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
