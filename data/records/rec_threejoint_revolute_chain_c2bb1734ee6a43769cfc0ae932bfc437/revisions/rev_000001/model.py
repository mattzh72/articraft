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


ROOT_CHEEK_T = 0.0045
ROOT_INNER_GAP = 0.0085
ROOT_OUTER_W = ROOT_INNER_GAP + 2.0 * ROOT_CHEEK_T

LINK1_LEN = 0.180
LINK1_BODY_T = 0.0080
LINK1_BODY_H = 0.032
LINK1_HUB_R = 0.0140
LINK1_FORK_GAP = 0.0065
LINK1_EYE_T = ROOT_INNER_GAP
LINK1_CHEEK_T = 0.0042
LINK1_FORK_OUTER = LINK1_FORK_GAP + 2.0 * LINK1_CHEEK_T
LINK1_FORK_R = 0.0115

LINK2_LEN = 0.155
LINK2_BODY_T = 0.0065
LINK2_BODY_H = 0.027
LINK2_HUB_R = LINK1_FORK_R
LINK2_EYE_T = LINK1_FORK_GAP
LINK2_FORK_GAP = 0.0055
LINK2_CHEEK_T = 0.0038
LINK2_FORK_OUTER = LINK2_FORK_GAP + 2.0 * LINK2_CHEEK_T
LINK2_FORK_R = 0.0098

NOSE_LEN = 0.085
NOSE_BODY_T = 0.0055
NOSE_BODY_H = 0.022
NOSE_HUB_R = LINK2_FORK_R
NOSE_EYE_T = LINK2_FORK_GAP

PIVOT_R = 0.0062


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _y_cylinder(*, x: float, z: float, radius: float, length: float, y_center: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y_center, z))
    )


def _foot_shape() -> cq.Workplane:
    base = _box((0.114, 0.058, 0.010), (-0.040, 0.0, -0.041))

    cheek_y = 0.5 * ROOT_INNER_GAP + 0.5 * ROOT_CHEEK_T
    cheek_core = _box((0.038, ROOT_CHEEK_T, 0.034), (-0.010, 0.0, -0.001))
    cheek_boss = _y_cylinder(x=0.0, z=0.0, radius=0.0145, length=ROOT_CHEEK_T)
    stop_pad = _box((0.010, ROOT_CHEEK_T, 0.012), (0.011, 0.0, -0.012))
    left_cheek = cheek_core.union(cheek_boss).union(stop_pad).translate((0.0, cheek_y, 0.0))
    right_cheek = cheek_core.union(cheek_boss).union(stop_pad).translate((0.0, -cheek_y, 0.0))

    left_rib = _box((0.022, ROOT_CHEEK_T, 0.020), (-0.022, cheek_y, -0.024))
    right_rib = _box((0.022, ROOT_CHEEK_T, 0.020), (-0.022, -cheek_y, -0.024))
    heel_block = _box((0.020, 0.032, 0.012), (-0.070, 0.0, -0.032))

    foot = base.union(left_cheek).union(right_cheek).union(left_rib).union(right_rib).union(heel_block)

    slot_cuts = (
        cq.Workplane("XY")
        .pushPoints([(-0.052, 0.0), (-0.020, 0.0)])
        .slot2D(0.018, 0.008, angle=0)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.049))
    )
    pivot_cut = _y_cylinder(x=0.0, z=0.0, radius=PIVOT_R, length=ROOT_OUTER_W + 0.004)

    return foot.cut(slot_cuts).cut(pivot_cut)


def _inner_link_shape() -> cq.Workplane:
    eye = _y_cylinder(x=0.0, z=0.0, radius=LINK1_HUB_R, length=LINK1_EYE_T)
    spine = _box((0.116, LINK1_BODY_T, LINK1_BODY_H), (0.072, 0.0, -0.002))
    emboss = _box((0.050, LINK1_BODY_T + 0.001, 0.008), (0.070, 0.0, 0.008))
    stop_pad = _box((0.014, LINK1_BODY_T, 0.008), (0.022, 0.0, -0.010))

    cheek_y = 0.5 * LINK1_FORK_GAP + 0.5 * LINK1_CHEEK_T
    cheek_core = _box((0.040, LINK1_CHEEK_T, 0.022), (0.160, 0.0, 0.0))
    cheek_boss = _y_cylinder(x=LINK1_LEN, z=0.0, radius=LINK1_FORK_R, length=LINK1_CHEEK_T)
    left_cheek = cheek_core.union(cheek_boss).translate((0.0, cheek_y, 0.0))
    right_cheek = cheek_core.union(cheek_boss).translate((0.0, -cheek_y, 0.0))
    left_rail = _box((0.028, LINK1_CHEEK_T, 0.014), (0.126, cheek_y, -0.010))
    right_rail = _box((0.028, LINK1_CHEEK_T, 0.014), (0.126, -cheek_y, -0.010))

    inner_link = eye.union(spine).union(emboss).union(stop_pad).union(left_cheek).union(right_cheek).union(left_rail).union(right_rail)
    prox_cut = _y_cylinder(x=0.0, z=0.0, radius=PIVOT_R, length=LINK1_EYE_T + 0.003)
    dist_cut = _y_cylinder(x=LINK1_LEN, z=0.0, radius=PIVOT_R, length=LINK1_FORK_OUTER + 0.003)
    lightening = _box((0.046, LINK1_BODY_T + 0.002, 0.010), (0.088, 0.0, -0.002))
    return inner_link.cut(prox_cut).cut(dist_cut).cut(lightening)


def _outer_link_shape() -> cq.Workplane:
    eye = _y_cylinder(x=0.0, z=0.0, radius=LINK2_HUB_R, length=LINK2_EYE_T)
    spine = _box((0.100, LINK2_BODY_T, LINK2_BODY_H), (0.064, 0.0, -0.002))
    emboss = _box((0.042, LINK2_BODY_T + 0.001, 0.007), (0.061, 0.0, 0.007))
    stop_pad = _box((0.012, LINK2_BODY_T, 0.008), (0.020, 0.0, -0.010))

    cheek_y = 0.5 * LINK2_FORK_GAP + 0.5 * LINK2_CHEEK_T
    cheek_core = _box((0.036, LINK2_CHEEK_T, 0.019), (0.138, 0.0, 0.0))
    cheek_boss = _y_cylinder(x=LINK2_LEN, z=0.0, radius=LINK2_FORK_R, length=LINK2_CHEEK_T)
    left_cheek = cheek_core.union(cheek_boss).translate((0.0, cheek_y, 0.0))
    right_cheek = cheek_core.union(cheek_boss).translate((0.0, -cheek_y, 0.0))
    left_rail = _box((0.024, LINK2_CHEEK_T, 0.012), (0.110, cheek_y, -0.009))
    right_rail = _box((0.024, LINK2_CHEEK_T, 0.012), (0.110, -cheek_y, -0.009))

    outer_link = eye.union(spine).union(emboss).union(stop_pad).union(left_cheek).union(right_cheek).union(left_rail).union(right_rail)
    prox_cut = _y_cylinder(x=0.0, z=0.0, radius=PIVOT_R, length=LINK2_EYE_T + 0.003)
    dist_cut = _y_cylinder(x=LINK2_LEN, z=0.0, radius=PIVOT_R, length=LINK2_FORK_OUTER + 0.003)
    lightening = _box((0.040, LINK2_BODY_T + 0.002, 0.009), (0.076, 0.0, -0.002))
    return outer_link.cut(prox_cut).cut(dist_cut).cut(lightening)


def _nose_link_shape() -> cq.Workplane:
    eye = _y_cylinder(x=0.0, z=0.0, radius=NOSE_HUB_R, length=NOSE_EYE_T)
    body = _box((0.048, NOSE_BODY_T, NOSE_BODY_H), (0.034, 0.0, -0.002))
    top_crease = _box((0.020, NOSE_BODY_T + 0.001, 0.006), (0.036, 0.0, 0.006))
    stop_pad = _box((0.011, NOSE_BODY_T, 0.007), (0.016, 0.0, -0.009))
    nose_taper = _box((0.020, NOSE_BODY_T, 0.013), (0.064, 0.0, -0.003))
    hook_bar = _box((0.020, NOSE_BODY_T, 0.008), (0.078, 0.0, -0.013))
    hook_lip = _box((0.008, NOSE_BODY_T, 0.018), (0.092, 0.0, -0.020))
    hook_return = _box((0.015, NOSE_BODY_T, 0.006), (0.080, 0.0, -0.027))

    nose = eye.union(body).union(top_crease).union(stop_pad).union(nose_taper).union(hook_bar).union(hook_lip).union(hook_return)
    prox_cut = _y_cylinder(x=0.0, z=0.0, radius=PIVOT_R, length=NOSE_EYE_T + 0.003)
    lighten = _box((0.016, NOSE_BODY_T + 0.002, 0.008), (0.042, 0.0, -0.002))
    return nose.cut(prox_cut).cut(lighten)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bracket_chain")

    foot_mat = model.material("foot_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    link1_mat = model.material("root_link_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    link2_mat = model.material("mid_link_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    nose_mat = model.material("nose_link_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    foot = model.part("foot")
    foot.visual(mesh_from_cadquery(_foot_shape(), "foot_shell"), material=foot_mat, name="foot_shell")

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_cadquery(_inner_link_shape(), "inner_link_shell"),
        material=link1_mat,
        name="inner_link_shell",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(_outer_link_shape(), "outer_link_shell"),
        material=link2_mat,
        name="outer_link_shell",
    )

    nose_link = model.part("nose_link")
    nose_link.visual(
        mesh_from_cadquery(_nose_link_shape(), "nose_link_shell"),
        material=nose_mat,
        name="nose_link_shell",
    )

    model.articulation(
        "foot_to_inner",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=inner_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-1.45, upper=0.18),
    )
    model.articulation(
        "inner_to_outer",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.8, lower=-0.18, upper=1.75),
    )
    model.articulation(
        "outer_to_nose",
        ArticulationType.REVOLUTE,
        parent=outer_link,
        child=nose_link,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-1.35, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    nose_link = object_model.get_part("nose_link")
    foot_to_inner = object_model.get_articulation("foot_to_inner")
    inner_to_outer = object_model.get_articulation("inner_to_outer")
    outer_to_nose = object_model.get_articulation("outer_to_nose")

    ctx.allow_overlap(
        foot,
        inner_link,
        reason="Root eye nests between fixed clevis cheeks; exported hinge meshes use zero-clearance bearing contact and register minute overlap at the pivot.",
    )
    ctx.allow_overlap(
        inner_link,
        outer_link,
        reason="Middle eye nests between formed fork cheeks; the meshed bearing surfaces at the second pivot register as slight overlap.",
    )
    ctx.allow_overlap(
        outer_link,
        nose_link,
        reason="Terminal nose eye sits inside the outer fork with zero-clearance hinge contact, producing a tiny meshed pivot overlap.",
    )

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

    all_axes_parallel = all(
        abs(j.axis[0]) < 1e-9 and abs(j.axis[1] - 1.0) < 1e-9 and abs(j.axis[2]) < 1e-9
        for j in (foot_to_inner, inner_to_outer, outer_to_nose)
    )
    ctx.check(
        "all revolute axes are parallel in the folding plane",
        all_axes_parallel,
        details="Expected all three pivots to rotate about the shared +Y axis.",
    )

    ctx.expect_contact(foot, inner_link, name="foot supports inner link at root pivot")
    ctx.expect_contact(inner_link, outer_link, name="inner link supports outer link at mid pivot")
    ctx.expect_contact(outer_link, nose_link, name="outer link supports nose link at tip pivot")

    with ctx.pose({foot_to_inner: -0.28, inner_to_outer: 0.24, outer_to_nose: -0.18}):
        ctx.fail_if_parts_overlap_in_current_pose(name="shallow arc pose stays clear")
        ctx.expect_contact(foot, inner_link, name="root pivot stays supported in shallow arc")
        ctx.expect_contact(inner_link, outer_link, name="mid pivot stays supported in shallow arc")
        ctx.expect_contact(outer_link, nose_link, name="tip pivot stays supported in shallow arc")
        ctx.expect_origin_gap(
            nose_link,
            foot,
            axis="x",
            min_gap=0.32,
            name="nose projects outward in shallow extended arc",
        )

    with ctx.pose({foot_to_inner: -1.08, inner_to_outer: 1.22, outer_to_nose: -0.92}):
        ctx.fail_if_parts_overlap_in_current_pose(name="packed pose stays clear")
        ctx.expect_contact(foot, inner_link, name="root pivot stays supported when folded")
        ctx.expect_contact(inner_link, outer_link, name="mid pivot stays supported when folded")
        ctx.expect_contact(outer_link, nose_link, name="tip pivot stays supported when folded")
        ctx.expect_origin_gap(
            nose_link,
            foot,
            axis="x",
            min_gap=0.05,
            max_gap=0.25,
            name="packed pose brings the nose back toward the foot",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
