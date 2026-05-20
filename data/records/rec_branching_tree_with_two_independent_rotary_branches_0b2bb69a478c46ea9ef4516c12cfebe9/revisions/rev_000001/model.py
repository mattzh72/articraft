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


SPINE_W = 0.08
SPINE_D = 0.06
SPINE_H = 1.30
PEDESTAL_H = 0.06
BASE_RAIL_H = 0.035

UPPER_HUB_Z = 1.02
LOWER_HUB_Z = 0.68
HUB_RADIUS = 0.027
HUB_LENGTH = 0.058
BRANCH_HUB_RADIUS = 0.018
BRANCH_HUB_LENGTH = HUB_LENGTH
CHEEK_T = 0.014
CHEEK_LEN_X = 0.055
SADDLE_BLOCK_X = 0.03
CHEEK_OFFSET_Y = HUB_LENGTH / 2.0 + CHEEK_T / 2.0
HUB_X = SPINE_W / 2.0 + SADDLE_BLOCK_X + CHEEK_LEN_X / 2.0


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    x, y, z = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z))


def _side_saddle(sign: float, hub_z: float) -> cq.Workplane:
    hinge_x = sign * HUB_X
    block_x = sign * (SPINE_W / 2.0 + SADDLE_BLOCK_X / 2.0)
    cheek_center_x = hinge_x
    outer_face_y = CHEEK_OFFSET_Y + CHEEK_T / 2.0

    mount_block = _box((SADDLE_BLOCK_X, 0.10, 0.11), (block_x, 0.0, hub_z))

    front_cheek = _box((CHEEK_LEN_X, CHEEK_T, 0.11), (cheek_center_x, CHEEK_OFFSET_Y, hub_z))
    rear_cheek = _box((CHEEK_LEN_X, CHEEK_T, 0.11), (cheek_center_x, -CHEEK_OFFSET_Y, hub_z))

    front_boss = (
        cq.Workplane("XZ")
        .circle(0.031)
        .extrude(0.008)
        .translate((hinge_x, outer_face_y, hub_z))
    )
    rear_boss = (
        cq.Workplane("XZ")
        .circle(0.031)
        .extrude(-0.008)
        .translate((hinge_x, -outer_face_y, hub_z))
    )

    shoulder = _box((0.02, 0.088, 0.06), (sign * (SPINE_W / 2.0 + 0.01), 0.0, hub_z - 0.038))

    saddle = mount_block.union(front_cheek).union(rear_cheek).union(front_boss).union(rear_boss).union(shoulder)
    journal_bore = (
        cq.Workplane("XZ")
        .circle(BRANCH_HUB_RADIUS)
        .extrude(0.18, both=True)
        .translate((hinge_x, 0.0, hub_z))
    )
    return saddle.cut(journal_bore)


def _build_spine_shape() -> cq.Workplane:
    long_rail = _box((0.56, 0.09, BASE_RAIL_H), (0.0, 0.0, BASE_RAIL_H / 2.0))
    cross_rail = _box((0.18, 0.32, 0.03), (0.0, 0.0, 0.015))
    pedestal = _box((0.18, 0.14, PEDESTAL_H), (0.0, 0.0, BASE_RAIL_H + PEDESTAL_H / 2.0))
    spine = _box(
        (SPINE_W, SPINE_D, SPINE_H),
        (0.0, 0.0, BASE_RAIL_H + PEDESTAL_H + SPINE_H / 2.0),
    )
    top_cap = _box((0.09, 0.07, 0.018), (0.0, 0.0, BASE_RAIL_H + PEDESTAL_H + SPINE_H + 0.009))

    return (
        long_rail.union(cross_rail)
        .union(pedestal)
        .union(spine)
        .union(top_cap)
        .union(_side_saddle(-1.0, UPPER_HUB_Z))
        .union(_side_saddle(1.0, LOWER_HUB_Z))
    )


def _build_upper_branch_frame() -> cq.Workplane:
    root_pad = _box((0.06, 0.032, 0.05), (-0.02, 0.0, 0.0))
    hub_eye = cq.Workplane("XZ").circle(BRANCH_HUB_RADIUS).extrude(BRANCH_HUB_LENGTH / 2.0, both=True)
    beam = _box((0.216, 0.024, 0.034), (-0.151, 0.0, 0.0))
    head_block = _box((0.09, 0.045, 0.062), (-0.30, 0.0, 0.0))
    web = (
        cq.Workplane("XZ")
        .polyline([(-0.065, -0.014), (-0.11, -0.042), (-0.23, -0.042), (-0.195, -0.014)])
        .close()
        .extrude(0.010, both=True)
    )
    return root_pad.union(hub_eye).union(beam).union(head_block).union(web)


def _build_upper_pad() -> cq.Workplane:
    return _box((0.032, 0.10, 0.082), (-0.357, 0.0, 0.0))


def _build_lower_branch_frame() -> cq.Workplane:
    root_pad = _box((0.06, 0.032, 0.05), (0.02, 0.0, 0.0))
    hub_eye = cq.Workplane("XZ").circle(BRANCH_HUB_RADIUS).extrude(BRANCH_HUB_LENGTH / 2.0, both=True)
    beam = _box((0.216, 0.024, 0.034), (0.151, 0.0, 0.0))
    head_block = _box((0.09, 0.048, 0.062), (0.30, 0.0, 0.0))
    web = (
        cq.Workplane("XZ")
        .polyline([(0.065, -0.014), (0.11, -0.042), (0.23, -0.042), (0.195, -0.014)])
        .close()
        .extrude(0.010, both=True)
    )
    return root_pad.union(hub_eye).union(beam).union(head_block).union(web)


def _build_lower_fork() -> cq.Workplane:
    bridge = _box((0.040, 0.086, 0.064), (0.362, 0.0, 0.0))
    upper_tine = _box((0.078, 0.086, 0.016), (0.421, 0.0, 0.024))
    lower_tine = _box((0.078, 0.086, 0.016), (0.421, 0.0, -0.024))
    return bridge.union(upper_tine).union(lower_tine)


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_tree")

    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.18, 0.18, 0.20, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_build_spine_shape(), "spine_frame"),
        material=dark_steel,
        name="spine_frame",
    )

    upper_left_branch = model.part("upper_left_branch")
    upper_left_branch.visual(
        mesh_from_cadquery(_build_upper_branch_frame(), "upper_left_branch_frame"),
        material=satin_steel,
        name="branch_frame",
    )
    upper_left_branch.visual(
        mesh_from_cadquery(_build_upper_pad(), "upper_left_pad"),
        material=pad_rubber,
        name="pad_face",
    )

    lower_right_branch = model.part("lower_right_branch")
    lower_right_branch.visual(
        mesh_from_cadquery(_build_lower_branch_frame(), "lower_right_branch_frame"),
        material=satin_steel,
        name="branch_frame",
    )
    lower_right_branch.visual(
        mesh_from_cadquery(_build_lower_fork(), "lower_right_fork"),
        material=satin_steel,
        name="fork_head",
    )

    model.articulation(
        "spine_to_upper_left_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_left_branch,
        origin=Origin(xyz=(-HUB_X, 0.0, UPPER_HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.35, upper=1.15),
    )
    model.articulation(
        "spine_to_lower_right_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_right_branch,
        origin=Origin(xyz=(HUB_X, 0.0, LOWER_HUB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.30, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    upper_left_branch = object_model.get_part("upper_left_branch")
    lower_right_branch = object_model.get_part("lower_right_branch")
    upper_joint = object_model.get_articulation("spine_to_upper_left_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_right_branch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        spine,
        upper_left_branch,
        reason="simplified captured journal: branch hub volume shares the saddle bore region to represent an internal spindle",
    )
    ctx.allow_overlap(
        spine,
        lower_right_branch,
        reason="simplified captured journal: branch hub volume shares the saddle bore region to represent an internal spindle",
    )

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
        "upper branch hinge axis",
        _vec_close(upper_joint.axis, (0.0, 1.0, 0.0)),
        details=f"expected (0, 1, 0), got {upper_joint.axis}",
    )
    ctx.check(
        "lower branch hinge axis",
        _vec_close(lower_joint.axis, (0.0, -1.0, 0.0)),
        details=f"expected (0, -1, 0), got {lower_joint.axis}",
    )

    ctx.expect_contact(upper_left_branch, spine, name="upper branch hub supported by saddle")
    ctx.expect_contact(lower_right_branch, spine, name="lower branch hub supported by saddle")

    ctx.expect_origin_gap(
        spine,
        upper_left_branch,
        axis="x",
        min_gap=0.09,
        max_gap=0.11,
        name="upper branch mounted on left saddle",
    )
    ctx.expect_origin_gap(
        lower_right_branch,
        spine,
        axis="x",
        min_gap=0.09,
        max_gap=0.11,
        name="lower branch mounted on right saddle",
    )
    ctx.expect_origin_gap(
        upper_left_branch,
        lower_right_branch,
        axis="z",
        min_gap=0.30,
        max_gap=0.40,
        name="branches stacked on the upright spine",
    )

    upper_pad_rest = ctx.part_element_world_aabb(upper_left_branch, elem="pad_face")
    lower_fork_rest = ctx.part_element_world_aabb(lower_right_branch, elem="fork_head")
    if upper_pad_rest is not None and lower_fork_rest is not None:
        upper_pad_rest_center = _aabb_center(upper_pad_rest)
        lower_fork_rest_center = _aabb_center(lower_fork_rest)
        ctx.check(
            "rest pose keeps branches on opposite sides",
            upper_pad_rest_center[0] < -0.45 and lower_fork_rest_center[0] > 0.40,
            details=(
                f"upper pad center x={upper_pad_rest_center[0]:.4f}, "
                f"lower fork center x={lower_fork_rest_center[0]:.4f}"
            ),
        )

    with ctx.pose({upper_joint: 0.85}):
        upper_pad_open = ctx.part_element_world_aabb(upper_left_branch, elem="pad_face")
        if upper_pad_rest is not None and upper_pad_open is not None:
            upper_pad_open_center = _aabb_center(upper_pad_open)
            ctx.check(
                "upper branch positive rotation lifts pad",
                upper_pad_open_center[2] > _aabb_center(upper_pad_rest)[2] + 0.18,
                details=(
                    f"rest z={_aabb_center(upper_pad_rest)[2]:.4f}, "
                    f"open z={upper_pad_open_center[2]:.4f}"
                ),
            )
        ctx.expect_contact(
            upper_left_branch,
            spine,
            name="upper branch stays supported while opened",
        )

    with ctx.pose({lower_joint: 0.85}):
        lower_fork_open = ctx.part_element_world_aabb(lower_right_branch, elem="fork_head")
        if lower_fork_rest is not None and lower_fork_open is not None:
            lower_fork_open_center = _aabb_center(lower_fork_open)
            ctx.check(
                "lower branch positive rotation lifts fork",
                lower_fork_open_center[2] > _aabb_center(lower_fork_rest)[2] + 0.15,
                details=(
                    f"rest z={_aabb_center(lower_fork_rest)[2]:.4f}, "
                    f"open z={lower_fork_open_center[2]:.4f}"
                ),
            )
        ctx.expect_contact(
            lower_right_branch,
            spine,
            name="lower branch stays supported while opened",
        )

    with ctx.pose({upper_joint: 0.85, lower_joint: 0.80}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened branches remain clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
