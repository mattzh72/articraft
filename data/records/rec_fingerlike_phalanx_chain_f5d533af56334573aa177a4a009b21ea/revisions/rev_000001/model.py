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


ROOT_BACK = 0.026
ROOT_WIDTH = 0.028
ROOT_HEIGHT = 0.028
ROOT_FORK_LEN = 0.012
ROOT_CHEEK_T = 0.006
ROOT_FORK_GAP = 0.016
ROOT_FORK_HEIGHT = 0.024
ROOT_PIN_R = 0.0032

PROX_LEN = 0.040
PROX_BODY_W = 0.022
PROX_BODY_H = 0.021
PROX_KNUCKLE_LEN = 0.009
PROX_KNUCKLE_W = ROOT_FORK_GAP
PROX_KNUCKLE_H = 0.020
PROX_FORK_LEN = 0.010
PROX_CHEEK_T = 0.004
PROX_FORK_GAP = 0.014
PROX_FORK_HEIGHT = 0.019
PROX_PIN_R = 0.0027

MID_LEN = 0.028
MID_BODY_W = 0.019
MID_BODY_H = 0.018
MID_KNUCKLE_LEN = 0.008
MID_KNUCKLE_W = PROX_FORK_GAP
MID_KNUCKLE_H = 0.017
MID_FORK_LEN = 0.009
MID_CHEEK_T = 0.0035
MID_FORK_GAP = 0.012
MID_FORK_HEIGHT = 0.016
MID_PIN_R = 0.0022

DIST_LEN = 0.022
DIST_BODY_W = 0.016
DIST_BODY_H = 0.015
DIST_KNUCKLE_LEN = 0.007
DIST_KNUCKLE_W = MID_FORK_GAP
DIST_KNUCKLE_H = 0.014

FUSE_OVERLAP = 0.0005


def _box(length: float, width: float, height: float, *, x_start: float, y_center: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x_start, y_center, 0.0))
    )


def _root_shape():
    main_block = _box(
        ROOT_BACK - ROOT_FORK_LEN + FUSE_OVERLAP,
        ROOT_WIDTH,
        ROOT_HEIGHT,
        x_start=-ROOT_BACK,
    )
    cheek_y = (ROOT_FORK_GAP + ROOT_CHEEK_T) / 2.0
    left_cheek = _box(
        ROOT_FORK_LEN,
        ROOT_CHEEK_T,
        ROOT_FORK_HEIGHT,
        x_start=-ROOT_FORK_LEN,
        y_center=cheek_y,
    )
    right_cheek = _box(
        ROOT_FORK_LEN,
        ROOT_CHEEK_T,
        ROOT_FORK_HEIGHT,
        x_start=-ROOT_FORK_LEN,
        y_center=-cheek_y,
    )
    return main_block.union(left_cheek).union(right_cheek)


def _phalanx_with_fork(
    *,
    segment_len: float,
    body_w: float,
    body_h: float,
    rear_knuckle_len: float,
    rear_knuckle_w: float,
    rear_knuckle_h: float,
    fork_len: float,
    cheek_t: float,
    fork_gap: float,
    fork_h: float,
):
    rear_knuckle = _box(
        rear_knuckle_len,
        rear_knuckle_w,
        rear_knuckle_h,
        x_start=0.0,
    )

    body_x_start = rear_knuckle_len - FUSE_OVERLAP
    body_x_end = segment_len - fork_len + FUSE_OVERLAP
    body = _box(body_x_end - body_x_start, body_w, body_h, x_start=body_x_start)
    body = body.edges("|X").fillet(min(body_h, body_w) * 0.10)

    cheek_y = (fork_gap + cheek_t) / 2.0
    left_cheek = _box(
        fork_len,
        cheek_t,
        fork_h,
        x_start=segment_len - fork_len,
        y_center=cheek_y,
    )
    right_cheek = _box(
        fork_len,
        cheek_t,
        fork_h,
        x_start=segment_len - fork_len,
        y_center=-cheek_y,
    )
    return rear_knuckle.union(body).union(left_cheek).union(right_cheek)


def _distal_shape():
    rear_knuckle = _box(
        DIST_KNUCKLE_LEN,
        DIST_KNUCKLE_W,
        DIST_KNUCKLE_H,
        x_start=0.0,
    )

    body_x_start = DIST_KNUCKLE_LEN - FUSE_OVERLAP
    tip_body = _box(
        DIST_LEN - body_x_start + FUSE_OVERLAP,
        DIST_BODY_W,
        DIST_BODY_H,
        x_start=body_x_start,
    )
    tip_body = tip_body.faces(">X").edges().fillet(0.004)
    return rear_knuckle.union(tip_body)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="finger_module")

    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))

    root = model.part("root_block")
    root.visual(mesh_from_cadquery(_root_shape(), "root_block"), material=graphite, name="root_shell")
    root.inertial = Inertial.from_geometry(
        Box((ROOT_BACK, ROOT_WIDTH, ROOT_HEIGHT)),
        mass=0.14,
        origin=Origin(xyz=(-ROOT_BACK / 2.0, 0.0, 0.0)),
    )

    proximal = model.part("proximal")
    proximal.visual(
        mesh_from_cadquery(
            _phalanx_with_fork(
                segment_len=PROX_LEN,
                body_w=PROX_BODY_W,
                body_h=PROX_BODY_H,
                rear_knuckle_len=PROX_KNUCKLE_LEN,
                rear_knuckle_w=PROX_KNUCKLE_W,
                rear_knuckle_h=PROX_KNUCKLE_H,
                fork_len=PROX_FORK_LEN,
                cheek_t=PROX_CHEEK_T,
                fork_gap=PROX_FORK_GAP,
                fork_h=PROX_FORK_HEIGHT,
            ),
            "proximal_phalanx",
        ),
        material=aluminum,
        name="proximal_shell",
    )
    proximal.inertial = Inertial.from_geometry(
        Box((PROX_LEN, PROX_BODY_W, PROX_BODY_H)),
        mass=0.08,
        origin=Origin(xyz=(PROX_LEN / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle")
    middle.visual(
        mesh_from_cadquery(
            _phalanx_with_fork(
                segment_len=MID_LEN,
                body_w=MID_BODY_W,
                body_h=MID_BODY_H,
                rear_knuckle_len=MID_KNUCKLE_LEN,
                rear_knuckle_w=MID_KNUCKLE_W,
                rear_knuckle_h=MID_KNUCKLE_H,
                fork_len=MID_FORK_LEN,
                cheek_t=MID_CHEEK_T,
                fork_gap=MID_FORK_GAP,
                fork_h=MID_FORK_HEIGHT,
            ),
            "middle_phalanx",
        ),
        material=aluminum,
        name="middle_shell",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MID_LEN, MID_BODY_W, MID_BODY_H)),
        mass=0.05,
        origin=Origin(xyz=(MID_LEN / 2.0, 0.0, 0.0)),
    )

    distal = model.part("distal")
    distal.visual(mesh_from_cadquery(_distal_shape(), "distal_phalanx"), material=aluminum, name="distal_shell")
    distal.inertial = Inertial.from_geometry(
        Box((DIST_LEN, DIST_BODY_W, DIST_BODY_H)),
        mass=0.03,
        origin=Origin(xyz=(DIST_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROX_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MID_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_block")
    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")
    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

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
        "all_parts_present",
        all(part is not None for part in (root, proximal, middle, distal)),
        "Root block and all three phalanx links must exist.",
    )
    ctx.check(
        "parallel_hinge_axes",
        root_to_proximal.axis == proximal_to_middle.axis == middle_to_distal.axis == (0.0, 1.0, 0.0),
        "All three joints should share a parallel +Y hinge axis.",
    )

    with ctx.pose({root_to_proximal: 0.0, proximal_to_middle: 0.0, middle_to_distal: 0.0}):
        ctx.expect_contact(root, proximal, name="root_hinge_contact")
        ctx.expect_contact(proximal, middle, name="proximal_hinge_contact")
        ctx.expect_contact(middle, distal, name="middle_hinge_contact")

        prox_aabb = ctx.part_world_aabb(proximal)
        mid_aabb = ctx.part_world_aabb(middle)
        dist_aabb = ctx.part_world_aabb(distal)
        if prox_aabb and mid_aabb and dist_aabb:
            prox_dx = prox_aabb[1][0] - prox_aabb[0][0]
            mid_dx = mid_aabb[1][0] - mid_aabb[0][0]
            dist_dx = dist_aabb[1][0] - dist_aabb[0][0]
            prox_dy = prox_aabb[1][1] - prox_aabb[0][1]
            mid_dy = mid_aabb[1][1] - mid_aabb[0][1]
            dist_dy = dist_aabb[1][1] - dist_aabb[0][1]
            ctx.check(
                "phalanx_sizes_taper",
                prox_dx > mid_dx > dist_dx and prox_dy > mid_dy > dist_dy,
                (
                    "Segments should progressively shrink. "
                    f"x lengths: {(prox_dx, mid_dx, dist_dx)}, "
                    f"widths: {(prox_dy, mid_dy, dist_dy)}"
                ),
            )
        else:
            ctx.fail("phalanx_sizes_taper", "Could not resolve world AABBs for the phalanx links.")

    distal_rest = ctx.part_world_position(distal)
    with ctx.pose(
        {
            root_to_proximal: root_to_proximal.motion_limits.upper,
            proximal_to_middle: proximal_to_middle.motion_limits.upper,
            middle_to_distal: middle_to_distal.motion_limits.upper,
        }
    ):
        distal_flexed = ctx.part_world_position(distal)

    if distal_rest is not None and distal_flexed is not None:
        ctx.check(
            "positive_motion_curls_finger",
            distal_flexed[2] < distal_rest[2] - 0.01,
            f"Expected positive flexion to move the distal link downward: rest={distal_rest}, flexed={distal_flexed}",
        )
    else:
        ctx.fail("positive_motion_curls_finger", "Could not evaluate distal-link motion in posed checks.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
