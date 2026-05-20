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


PIN_RADIUS = 0.0062
BARREL_WIDTH = 0.012
FORK_INNER_GAP = 0.014
CHEEK_THICKNESS = 0.0032
CHEEK_CENTER_Y = FORK_INNER_GAP / 2.0 + CHEEK_THICKNESS / 2.0
WASHER_RADIUS = 0.0078
WASHER_THICKNESS = 0.0014
COLLAR_RADIUS = 0.0072
COLLAR_THICKNESS = 0.001

ROOT_PITCH = 0.038
MID_PITCH = 0.036


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _outer_joint_caps(x_pos: float) -> cq.Workplane:
    cap_pos = CHEEK_CENTER_Y + CHEEK_THICKNESS / 2.0 + WASHER_THICKNESS / 2.0
    return (
        _y_cylinder(WASHER_RADIUS, WASHER_THICKNESS).translate((x_pos, cap_pos, 0.0))
        .union(_y_cylinder(WASHER_RADIUS, WASHER_THICKNESS).translate((x_pos, -cap_pos, 0.0)))
    )


def _joint_barrel(x_pos: float = 0.0) -> cq.Workplane:
    return _y_cylinder(PIN_RADIUS, BARREL_WIDTH).translate((x_pos, 0.0, 0.0))


def _proximal_knuckle(x_pos: float = 0.0) -> cq.Workplane:
    collar_offset = BARREL_WIDTH / 2.0 + COLLAR_THICKNESS / 2.0
    return (
        _joint_barrel(x_pos)
        .union(_y_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS).translate((x_pos, collar_offset, 0.0)))
        .union(_y_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS).translate((x_pos, -collar_offset, 0.0)))
    )


def _fork_cheeks(x_pos: float, cheek_len: float, cheek_height: float, z_pos: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(cheek_len, CHEEK_THICKNESS, cheek_height)
        .translate((x_pos, CHEEK_CENTER_Y, z_pos))
        .union(
            cq.Workplane("XY")
            .box(cheek_len, CHEEK_THICKNESS, cheek_height)
            .translate((x_pos, -CHEEK_CENTER_Y, z_pos))
        )
    )


def _base_shape() -> cq.Workplane:
    back_block = cq.Workplane("XY").box(0.028, 0.026, 0.024).translate((-0.023, 0.0, -0.002))
    foot = cq.Workplane("XY").box(0.038, 0.032, 0.006).translate((-0.024, 0.0, -0.018))
    top_saddle = cq.Workplane("XY").box(0.014, 0.024, 0.010).translate((-0.015, 0.0, 0.008))
    cheek_roots = _fork_cheeks(-0.011, 0.014, 0.016, z_pos=-0.001)
    front_fork = _fork_cheeks(-0.001, 0.014, 0.022)
    return back_block.union(foot).union(top_saddle).union(cheek_roots).union(front_fork).union(_outer_joint_caps(0.0))


def _root_link_shape() -> cq.Workplane:
    proximal_barrel = _proximal_knuckle()
    body = (
        cq.Workplane("XY")
        .box(0.020, 0.0095, 0.014)
        .translate((0.020, 0.0, -0.0005))
        .edges("|Y")
        .fillet(0.0025)
    )
    upper_pad = cq.Workplane("XY").box(0.012, 0.0090, 0.006).translate((0.022, 0.0, 0.0055))
    side_rails = _fork_cheeks(0.022, 0.010, 0.011)
    distal_cheeks = _fork_cheeks(ROOT_PITCH - 0.001, 0.014, 0.018)
    return proximal_barrel.union(body).union(upper_pad).union(side_rails).union(distal_cheeks).union(_outer_joint_caps(ROOT_PITCH))


def _mid_link_shape() -> cq.Workplane:
    proximal_barrel = _proximal_knuckle()
    path = cq.Workplane("XZ").moveTo(0.008, -0.001).threePointArc((0.014, 0.005), (0.020, 0.001))
    curved_body = cq.Workplane("YZ").ellipse(0.0034, 0.0060).sweep(path, transition="round")
    dorsal_pad = cq.Workplane("XY").box(0.008, 0.0070, 0.0045).translate((0.017, 0.0, 0.0048))
    side_rails = _fork_cheeks(0.018, 0.008, 0.010)
    distal_cheeks = _fork_cheeks(MID_PITCH - 0.001, 0.014, 0.017)
    return proximal_barrel.union(curved_body).union(dorsal_pad).union(side_rails).union(distal_cheeks).union(_outer_joint_caps(MID_PITCH))


def _distal_link_shape() -> cq.Workplane:
    proximal_barrel = _proximal_knuckle()
    shank = cq.Workplane("XY").box(0.010, 0.0090, 0.010).translate((0.018, 0.0, -0.001))
    fingertip = cq.Workplane("XY").box(0.016, 0.0140, 0.013).translate((0.030, 0.0, -0.0015))
    top_face = cq.Workplane("XY").box(0.010, 0.0120, 0.006).translate((0.031, 0.0, 0.0038))
    return proximal_barrel.union(shank).union(fingertip).union(top_face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_grasping_finger")

    base_mat = model.material("base_mat", rgba=(0.18, 0.19, 0.21, 1.0))
    root_mat = model.material("root_mat", rgba=(0.31, 0.33, 0.37, 1.0))
    mid_mat = model.material("mid_mat", rgba=(0.43, 0.45, 0.48, 1.0))
    tip_mat = model.material("tip_mat", rgba=(0.25, 0.27, 0.30, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "finger_base"), material=base_mat, name="base_shell")

    root = model.part("root_link")
    root.visual(
        mesh_from_cadquery(_root_link_shape(), "finger_root_link"),
        material=root_mat,
        name="root_shell",
    )

    mid = model.part("mid_link")
    mid.visual(
        mesh_from_cadquery(_mid_link_shape(), "finger_mid_link"),
        material=mid_mat,
        name="mid_shell",
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_distal_link_shape(), "finger_distal_link"),
        material=tip_mat,
        name="distal_shell",
    )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=root,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "root_to_mid",
        ArticulationType.REVOLUTE,
        parent=root,
        child=mid,
        origin=Origin(xyz=(ROOT_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=0.0, upper=1.5),
    )
    model.articulation(
        "mid_to_distal",
        ArticulationType.REVOLUTE,
        parent=mid,
        child=distal,
        origin=Origin(xyz=(MID_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    root = object_model.get_part("root_link")
    mid = object_model.get_part("mid_link")
    distal = object_model.get_part("distal_link")
    joints = [
        object_model.get_articulation("base_to_root"),
        object_model.get_articulation("root_to_mid"),
        object_model.get_articulation("mid_to_distal"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        root,
        reason="Base knuckle captures the root link with idealized coaxial pin and collar volumes instead of separate hardware parts.",
    )
    ctx.allow_overlap(
        root,
        mid,
        reason="Root-to-mid pinned knuckle is modeled with nested hinge barrels and collars to keep the joint visually obvious.",
    )
    ctx.allow_overlap(
        mid,
        distal,
        reason="Mid-to-distal hinge uses overlapping idealized pin geometry rather than separate pin meshes.",
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
        "finger_has_four_parts",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "finger_has_three_serial_joints",
        len(object_model.articulations) == 3,
        f"expected 3 articulations, found {len(object_model.articulations)}",
    )

    for joint in joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_is_lateral",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis was {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_limits_are_flexion_only",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper > 1.0,
            f"{joint.name} limits were {limits}",
        )

    ctx.expect_contact(base, root, contact_tol=1e-4, name="base_contacts_root_at_knuckle")
    ctx.expect_contact(root, mid, contact_tol=1e-4, name="root_contacts_mid_at_knuckle")
    ctx.expect_contact(mid, distal, contact_tol=1e-4, name="mid_contacts_distal_at_knuckle")

    ctx.expect_origin_gap(mid, root, axis="x", min_gap=0.034, max_gap=0.042, name="mid_joint_pitch")
    ctx.expect_origin_gap(
        distal,
        mid,
        axis="x",
        min_gap=0.032,
        max_gap=0.040,
        name="distal_joint_pitch",
    )
    ctx.expect_overlap(base, root, axes="yz", min_overlap=0.012, name="base_root_knuckle_alignment")
    ctx.expect_overlap(root, mid, axes="yz", min_overlap=0.012, name="root_mid_knuckle_alignment")
    ctx.expect_overlap(mid, distal, axes="yz", min_overlap=0.010, name="mid_distal_knuckle_alignment")

    with ctx.pose(base_to_root=0.65, root_to_mid=0.90, mid_to_distal=0.70):
        ctx.expect_origin_gap(
            base,
            distal,
            axis="z",
            min_gap=0.030,
            name="curling_pose_drops_tip_below_base",
        )
        ctx.expect_origin_gap(
            distal,
            base,
            axis="x",
            min_gap=0.010,
            name="curling_pose_keeps_tip_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
