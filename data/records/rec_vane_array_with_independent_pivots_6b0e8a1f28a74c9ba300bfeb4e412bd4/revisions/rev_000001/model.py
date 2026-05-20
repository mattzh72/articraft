from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


FRAME_WIDTH = 1.00
FRAME_HEIGHT = 0.60
FRAME_DEPTH = 0.08
FRAME_BORDER = 0.06
OPENING_WIDTH = FRAME_WIDTH - 2.0 * FRAME_BORDER
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * FRAME_BORDER

VANE_COUNT = 6
VANE_PITCH = 0.078
VANE_DEPTH = 0.055
VANE_THICKNESS = 0.012
TRUNNION_RADIUS = 0.006
TRUNNION_LENGTH = 0.020
PAD_RADIUS = 0.012
PAD_LENGTH = 0.008
VANE_SWEEP = 1.05
CONTACT_X = OPENING_WIDTH / 2.0 - PAD_LENGTH
BLADE_LENGTH = 2.0 * (CONTACT_X - TRUNNION_LENGTH)

VANE_ZS = tuple((index - (VANE_COUNT - 1) / 2.0) * VANE_PITCH for index in range(VANE_COUNT))


def _make_vane_body() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .ellipse(VANE_DEPTH / 2.0, VANE_THICKNESS / 2.0)
        .extrude(BLADE_LENGTH / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louver_vane_bank")

    frame_material = model.material("frame_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    vane_material = model.material("vane_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - FRAME_BORDER / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + FRAME_BORDER / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + FRAME_BORDER / 2.0)),
        material=frame_material,
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_HEIGHT / 2.0 - FRAME_BORDER / 2.0)),
        material=frame_material,
        name="bottom_rail",
    )

    for index, z_center in enumerate(VANE_ZS, start=1):
        left_support_center_x = -OPENING_WIDTH / 2.0 + PAD_LENGTH / 2.0
        right_support_center_x = OPENING_WIDTH / 2.0 - PAD_LENGTH / 2.0
        frame.visual(
            Cylinder(radius=PAD_RADIUS, length=PAD_LENGTH),
            origin=Origin(
                xyz=(left_support_center_x, 0.0, z_center),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=frame_material,
            name=f"left_support_{index}",
        )
        frame.visual(
            Cylinder(radius=PAD_RADIUS, length=PAD_LENGTH),
            origin=Origin(
                xyz=(right_support_center_x, 0.0, z_center),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=frame_material,
            name=f"right_support_{index}",
        )

        vane = model.part(f"vane_{index}")
        vane.visual(
            mesh_from_cadquery(_make_vane_body(), f"vane_{index}_body"),
            material=vane_material,
            name="vane_body",
        )
        vane.visual(
            Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
            origin=Origin(
                xyz=(CONTACT_X - TRUNNION_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=vane_material,
            name="right_trunnion",
        )
        vane.visual(
            Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
            origin=Origin(
                xyz=(-CONTACT_X + TRUNNION_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=vane_material,
            name="left_trunnion",
        )

        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=-VANE_SWEEP,
                upper=VANE_SWEEP,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vanes = [object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

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

    frame_aabb = ctx.part_world_aabb(frame)
    frame_size_ok = False
    if frame_aabb is not None:
        frame_size_ok = (
            abs((frame_aabb[1][0] - frame_aabb[0][0]) - FRAME_WIDTH) < 0.02
            and abs((frame_aabb[1][1] - frame_aabb[0][1]) - FRAME_DEPTH) < 0.01
            and abs((frame_aabb[1][2] - frame_aabb[0][2]) - FRAME_HEIGHT) < 0.02
        )
    ctx.check(
        "frame_has_realistic_overall_size",
        frame_size_ok,
        f"expected about {(FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)}, got {frame_aabb}",
    )

    for vane, joint, z_center in zip(vanes, joints, VANE_ZS):
        axis = joint.axis
        axis_ok = (
            axis is not None
            and abs(axis[0] - 1.0) < 1e-9
            and abs(axis[1]) < 1e-9
            and abs(axis[2]) < 1e-9
        )
        ctx.check(f"{joint.name}_axis_is_x", axis_ok, f"axis={axis}")

        limits = joint.motion_limits
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + VANE_SWEEP) < 1e-9
            and abs(limits.upper - VANE_SWEEP) < 1e-9
        )
        ctx.check(f"{joint.name}_limits_match_vane_sweep", limits_ok, f"limits={limits}")

        vane_pos = ctx.part_world_position(vane)
        placement_ok = (
            vane_pos is not None
            and abs(vane_pos[0]) < 1e-9
            and abs(vane_pos[1]) < 1e-9
            and abs(vane_pos[2] - z_center) < 1e-9
        )
        ctx.check(
            f"{vane.name}_centerline_is_on_expected_row",
            placement_ok,
            f"expected z={z_center}, got position={vane_pos}",
        )

        ctx.expect_contact(
            frame,
            vane,
            elem_a=f"left_support_{int(vane.name.split('_')[-1])}",
            elem_b="left_trunnion",
            contact_tol=1e-3,
            name=f"{vane.name}_left_trunnion_is_supported",
        )
        ctx.expect_contact(
            frame,
            vane,
            elem_a=f"right_support_{int(vane.name.split('_')[-1])}",
            elem_b="right_trunnion",
            contact_tol=1e-3,
            name=f"{vane.name}_right_trunnion_is_supported",
        )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_lower_pose_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_pose_no_floating")
                ctx.expect_contact(
                    frame,
                    vane,
                    elem_a=f"left_support_{int(vane.name.split('_')[-1])}",
                    elem_b="left_trunnion",
                    contact_tol=1e-3,
                    name=f"{vane.name}_lower_pose_left_trunnion_support",
                )
                ctx.expect_contact(
                    frame,
                    vane,
                    elem_a=f"right_support_{int(vane.name.split('_')[-1])}",
                    elem_b="right_trunnion",
                    contact_tol=1e-3,
                    name=f"{vane.name}_lower_pose_right_trunnion_support",
                )

            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_upper_pose_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_pose_no_floating")
                ctx.expect_contact(
                    frame,
                    vane,
                    elem_a=f"left_support_{int(vane.name.split('_')[-1])}",
                    elem_b="left_trunnion",
                    contact_tol=1e-3,
                    name=f"{vane.name}_upper_pose_left_trunnion_support",
                )
                ctx.expect_contact(
                    frame,
                    vane,
                    elem_a=f"right_support_{int(vane.name.split('_')[-1])}",
                    elem_b="right_trunnion",
                    contact_tol=1e-3,
                    name=f"{vane.name}_upper_pose_right_trunnion_support",
                )

    for lower_vane, upper_vane in zip(vanes[:-1], vanes[1:]):
        ctx.expect_gap(
            upper_vane,
            lower_vane,
            axis="z",
            min_gap=0.055,
            max_gap=0.070,
            name=f"{upper_vane.name}_has_rest_gap_above_{lower_vane.name}",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=12,
        name="sampled_pose_no_floating_components",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
