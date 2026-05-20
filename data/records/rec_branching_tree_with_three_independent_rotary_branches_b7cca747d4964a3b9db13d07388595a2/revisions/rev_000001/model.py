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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SPINE_HEIGHT = 1.52
SPINE_WIDTH = 0.08
SPINE_DEPTH = 0.10
RIGHT_HUB_X = 0.082
LEFT_HUB_X = -RIGHT_HUB_X
HUB_HEIGHT = 0.05
LOWER_Z = 0.52
MIDDLE_Z = 0.88
UPPER_Z = 1.22


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _rib_xz(points: list[tuple[float, float]], width_y: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width_y, both=True)


def _make_side_saddle(side: float, z_level: float) -> cq.Workplane:
    axis_x = RIGHT_HUB_X if side > 0.0 else LEFT_HUB_X
    bridge_center_x = side * 0.060

    top_bridge = _box((0.050, 0.102, 0.014), (bridge_center_x, 0.0, z_level + 0.032))
    bottom_bridge = _box((0.050, 0.102, 0.014), (bridge_center_x, 0.0, z_level - 0.032))
    top_disk = _cylinder_z(0.034, 0.014, (axis_x, 0.0, z_level + 0.032))
    bottom_disk = _cylinder_z(0.034, 0.014, (axis_x, 0.0, z_level - 0.032))

    return top_bridge.union(bottom_bridge).union(top_disk).union(bottom_disk)


def _make_spine_shape() -> cq.Workplane:
    long_foot = _box((0.62, 0.10, 0.055), (0.0, 0.0, 0.0275))
    cross_foot = _box((0.18, 0.46, 0.050), (0.0, 0.0, 0.025))
    plinth = _box((0.22, 0.18, 0.080), (0.0, 0.0, 0.090))
    upright = _box((SPINE_WIDTH, SPINE_DEPTH, 1.380), (0.0, 0.0, 0.820))
    top_cap = _box((0.10, 0.12, 0.045), (0.0, 0.0, SPINE_HEIGHT - 0.0225))

    spine = long_foot.union(cross_foot).union(plinth).union(upright).union(top_cap)
    spine = spine.union(_make_side_saddle(1.0, LOWER_Z))
    spine = spine.union(_make_side_saddle(-1.0, MIDDLE_Z))
    spine = spine.union(_make_side_saddle(1.0, UPPER_Z))
    return spine


def _make_branch_core(
    arm_length: float,
    beam_width: float,
    beam_height: float,
    rib_drop: float,
) -> cq.Workplane:
    hub_outer = _cylinder_z(0.028, HUB_HEIGHT, (0.0, 0.0, 0.0))
    hub_bore = _cylinder_z(0.020, HUB_HEIGHT + 0.006, (0.0, 0.0, 0.0))
    rear_relief = _box((0.064, beam_width + 0.090, HUB_HEIGHT + 0.010), (-0.032, 0.0, 0.0))
    hub = hub_outer.cut(hub_bore).cut(rear_relief)
    neck = _box((0.100, beam_width * 0.34, beam_height * 0.56), (0.074, 0.0, 0.0))
    shoulder = _box((0.044, beam_width + 0.014, beam_height + 0.008), (0.143, 0.0, 0.0))
    beam_center_x = 0.165 + arm_length / 2.0
    beam = _box((arm_length, beam_width, beam_height), (beam_center_x, 0.0, 0.0))
    rib = _rib_xz(
        [
            (0.108, -beam_height / 2.0),
            (0.212, -beam_height / 2.0),
            (0.160, -beam_height / 2.0 - rib_drop),
        ],
        beam_width * 0.44,
    )

    return hub.union(neck).union(shoulder).union(beam).union(rib)


def _make_pad_branch() -> cq.Workplane:
    branch = _make_branch_core(arm_length=0.235, beam_width=0.055, beam_height=0.030, rib_drop=0.032)
    pad_block = _box((0.034, 0.076, 0.048), (0.286, 0.0, 0.0))
    pad_face = _box((0.016, 0.108, 0.118), (0.311, 0.0, 0.010))
    return branch.union(pad_block).union(pad_face)


def _make_fork_branch() -> cq.Workplane:
    branch = _make_branch_core(arm_length=0.205, beam_width=0.050, beam_height=0.028, rib_drop=0.028)
    fork_body = _box((0.102, 0.086, 0.050), (0.258, 0.0, 0.0))
    slot = _box((0.078, 0.044, 0.070), (0.284, 0.0, 0.0))
    return branch.union(fork_body).cut(slot)


def _make_tab_branch() -> cq.Workplane:
    hub_barrel = _cylinder_z(0.016, HUB_HEIGHT, (0.0, 0.0, 0.0))
    root_bar = _box((0.064, 0.032, 0.022), (0.032, 0.0, 0.0))
    shoulder = _box((0.048, 0.048, 0.030), (0.086, 0.0, 0.0))
    beam = _box((0.188, 0.046, 0.026), (0.204, 0.0, 0.0))
    rib = _rib_xz(
        [
            (0.050, -0.013),
            (0.170, -0.013),
            (0.112, -0.040),
        ],
        0.018,
    )
    tab_base = _box((0.046, 0.052, 0.036), (0.314, 0.0, 0.0))
    clamp_tab = _box((0.020, 0.072, 0.124), (0.347, 0.0, 0.034))
    tab_slot = _box((0.028, 0.024, 0.064), (0.347, 0.0, 0.034))

    return (
        hub_barrel.union(root_bar)
        .union(shoulder)
        .union(beam)
        .union(rib)
        .union(tab_base)
        .union(clamp_tab)
        .cut(tab_slot)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_style_rotary_tree")

    spine_material = model.material("spine_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    lower_material = model.material("lower_branch_blue", rgba=(0.28, 0.37, 0.50, 1.0))
    middle_material = model.material("middle_branch_olive", rgba=(0.40, 0.43, 0.28, 1.0))
    upper_material = model.material("upper_branch_silver", rgba=(0.63, 0.66, 0.68, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_make_spine_shape(), "spine"),
        material=spine_material,
        name="spine_shell",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(_make_pad_branch(), "lower_branch"),
        material=lower_material,
        name="lower_branch_shell",
    )

    middle_branch = model.part("middle_branch")
    middle_branch.visual(
        mesh_from_cadquery(_make_fork_branch(), "middle_branch"),
        material=middle_material,
        name="middle_branch_shell",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        Cylinder(radius=0.016, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=upper_material,
        name="upper_hub",
    )
    upper_branch.visual(
        Box((0.040, 0.022, 0.016)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=upper_material,
        name="upper_root_neck",
    )
    upper_branch.visual(
        Box((0.046, 0.048, 0.028)),
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        material=upper_material,
        name="upper_shoulder",
    )
    upper_branch.visual(
        Box((0.060, 0.016, 0.024)),
        origin=Origin(xyz=(0.120, 0.0, -0.025)),
        material=upper_material,
        name="upper_gusset",
    )
    upper_branch.visual(
        Box((0.180, 0.046, 0.026)),
        origin=Origin(xyz=(0.192, 0.0, 0.0)),
        material=upper_material,
        name="upper_beam",
    )
    upper_branch.visual(
        Box((0.046, 0.052, 0.036)),
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        material=upper_material,
        name="upper_tab_base",
    )
    upper_branch.visual(
        Box((0.018, 0.072, 0.124)),
        origin=Origin(xyz=(0.337, 0.0, 0.034)),
        material=upper_material,
        name="upper_branch_shell",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=1.2, lower=-1.25, upper=1.25)
    middle_limits = MotionLimits(effort=18.0, velocity=1.2, lower=-1.15, upper=1.15)

    model.articulation(
        "spine_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=(RIGHT_HUB_X, 0.0, LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "spine_to_middle_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=middle_branch,
        origin=Origin(xyz=(LEFT_HUB_X, 0.0, MIDDLE_Z), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=middle_limits,
    )
    model.articulation(
        "spine_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=(RIGHT_HUB_X, 0.0, UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    lower_branch = object_model.get_part("lower_branch")
    middle_branch = object_model.get_part("middle_branch")
    upper_branch = object_model.get_part("upper_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_branch")
    middle_joint = object_model.get_articulation("spine_to_middle_branch")
    upper_joint = object_model.get_articulation("spine_to_upper_branch")

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
        spine,
        upper_branch,
        elem_a="spine_shell",
        elem_b="upper_hub",
        reason="upper branch uses a captured journal-style hub nested inside the upper saddle support on the spine",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_expected_parts_present",
        all(part is not None for part in (spine, lower_branch, middle_branch, upper_branch)),
        details="spine and three branch parts must exist",
    )
    ctx.check(
        "all_branch_joints_are_vertical_revolutes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0)
            for joint in (lower_joint, middle_joint, upper_joint)
        ),
        details="each branch should revolve about its own vertical supported hub axis",
    )
    ctx.check(
        "branch_hubs_are_staggered_along_spine",
        lower_joint.origin.xyz[2] < middle_joint.origin.xyz[2] < upper_joint.origin.xyz[2],
        details="three branch modules should sit on separate saddles at distinct heights",
    )

    ctx.expect_contact(lower_branch, spine, contact_tol=5e-4, name="lower_branch_is_supported_on_saddle")
    ctx.expect_contact(middle_branch, spine, contact_tol=5e-4, name="middle_branch_is_supported_on_saddle")
    ctx.expect_contact(upper_branch, spine, contact_tol=5e-4, name="upper_branch_is_supported_on_saddle")

    with ctx.pose({lower_joint: 0.90}):
        lower_aabb = ctx.part_world_aabb(lower_branch)
        ctx.check(
            "lower_branch_swings_toward_positive_y",
            lower_aabb is not None and lower_aabb[1][1] > 0.18,
            details=f"lower branch AABB at +0.90 rad: {lower_aabb}",
        )

    with ctx.pose({middle_joint: 0.90}):
        middle_aabb = ctx.part_world_aabb(middle_branch)
        ctx.check(
            "middle_branch_swings_toward_negative_y",
            middle_aabb is not None and middle_aabb[0][1] < -0.15,
            details=f"middle branch AABB at +0.90 rad: {middle_aabb}",
        )

    with ctx.pose({upper_joint: -0.85}):
        upper_aabb = ctx.part_world_aabb(upper_branch)
        ctx.check(
            "upper_branch_swings_toward_negative_y",
            upper_aabb is not None and upper_aabb[0][1] < -0.12,
            details=f"upper branch AABB at -0.85 rad: {upper_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
