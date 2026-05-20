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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_step")

    painted_metal = model.material("painted_metal", rgba=(0.18, 0.22, 0.28, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.25, 0.28, 0.31, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.09, 0.10, 1.0))
    step_pad = model.material("step_pad", rgba=(0.14, 0.15, 0.16, 1.0))

    bed_surround = model.part("bed_surround")
    bed_surround.visual(
        Box((1.82, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.175, 0.03)),
        material=inner_trim,
        name="bed_sill",
    )
    bed_surround.visual(
        Box((0.11, 0.36, 0.64)),
        origin=Origin(xyz=(-0.855, 0.185, 0.38)),
        material=painted_metal,
        name="left_bed_side",
    )
    bed_surround.visual(
        Box((0.11, 0.36, 0.64)),
        origin=Origin(xyz=(0.855, 0.185, 0.38)),
        material=painted_metal,
        name="right_bed_side",
    )
    bed_surround.visual(
        Box((0.11, 0.08, 0.30)),
        origin=Origin(xyz=(-0.855, 0.045, 0.21)),
        material=painted_metal,
        name="left_jamb",
    )
    bed_surround.visual(
        Box((0.11, 0.08, 0.30)),
        origin=Origin(xyz=(0.855, 0.045, 0.21)),
        material=painted_metal,
        name="right_jamb",
    )
    bed_surround.visual(
        Box((0.070, 0.090, 0.040)),
        origin=Origin(xyz=(-0.815, -0.035, 0.025)),
        material=inner_trim,
        name="left_hinge_arm",
    )
    bed_surround.visual(
        Box((0.070, 0.090, 0.040)),
        origin=Origin(xyz=(0.815, -0.035, 0.025)),
        material=inner_trim,
        name="right_hinge_arm",
    )
    bed_surround.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(-0.772, -0.058, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="left_hinge_boss",
    )
    bed_surround.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.772, -0.058, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="right_hinge_boss",
    )
    bed_surround.inertial = Inertial.from_geometry(
        Box((1.82, 0.36, 0.64)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.18, 0.32)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.44, 0.028, 0.095)),
        origin=Origin(xyz=(-0.54, 0.014, 0.0475)),
        material=painted_metal,
        name="left_lower_frame",
    )
    tailgate.visual(
        Box((0.44, 0.028, 0.095)),
        origin=Origin(xyz=(0.54, 0.014, 0.0475)),
        material=painted_metal,
        name="right_lower_frame",
    )
    tailgate.visual(
        Box((1.58, 0.028, 0.055)),
        origin=Origin(xyz=(0.0, 0.014, 0.5275)),
        material=painted_metal,
        name="top_rail",
    )
    tailgate.visual(
        Box((0.08, 0.028, 0.43)),
        origin=Origin(xyz=(-0.75, 0.014, 0.31)),
        material=painted_metal,
        name="left_frame_rail",
    )
    tailgate.visual(
        Box((0.08, 0.028, 0.43)),
        origin=Origin(xyz=(0.75, 0.014, 0.31)),
        material=painted_metal,
        name="right_frame_rail",
    )
    tailgate.visual(
        Box((1.42, 0.006, 0.45)),
        origin=Origin(xyz=(0.0, -0.010, 0.28)),
        material=painted_metal,
        name="rear_skin",
    )
    tailgate.visual(
        Box((1.04, 0.020, 0.11)),
        origin=Origin(xyz=(0.0, 0.010, 0.445)),
        material=inner_trim,
        name="upper_inner_panel",
    )
    tailgate.visual(
        Box((0.18, 0.020, 0.22)),
        origin=Origin(xyz=(-0.54, 0.010, 0.205)),
        material=inner_trim,
        name="left_step_recess_fill",
    )
    tailgate.visual(
        Box((0.18, 0.020, 0.22)),
        origin=Origin(xyz=(0.54, 0.010, 0.205)),
        material=inner_trim,
        name="right_step_recess_fill",
    )
    tailgate.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(-0.445, -0.014, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="left_step_hinge_boss",
    )
    tailgate.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.445, -0.014, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="right_step_hinge_boss",
    )
    tailgate.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(-0.764, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="left_tailgate_hinge",
    )
    tailgate.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.764, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="right_tailgate_hinge",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((1.58, 0.045, 0.56)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    step_panel = model.part("step_panel")
    step_panel.visual(
        Box((0.82, 0.010, 0.225)),
        origin=Origin(xyz=(0.0, 0.005, 0.1125)),
        material=step_pad,
        name="step_face",
    )
    step_panel.visual(
        Box((0.87, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.004, 0.015)),
        material=inner_trim,
        name="bottom_hinge_beam",
    )
    step_panel.visual(
        Box((0.68, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0105, 0.055)),
        material=handle_black,
        name="tread_strip_lower",
    )
    step_panel.visual(
        Box((0.68, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0105, 0.105)),
        material=handle_black,
        name="tread_strip_mid",
    )
    step_panel.visual(
        Box((0.68, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0105, 0.155)),
        material=handle_black,
        name="tread_strip_upper",
    )
    step_panel.visual(
        Box((0.20, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.010, 0.160)),
        material=inner_trim,
        name="handle_bridge",
    )
    step_panel.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(-0.082, 0.010, 0.174)),
        material=inner_trim,
        name="left_handle_ear",
    )
    step_panel.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(0.082, 0.010, 0.174)),
        material=inner_trim,
        name="right_handle_ear",
    )
    step_panel.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(-0.435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="left_step_hinge",
    )
    step_panel.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="right_step_hinge",
    )
    step_panel.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(-0.082, 0.012, 0.186), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="left_handle_socket",
    )
    step_panel.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.082, 0.012, 0.186), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_trim,
        name="right_handle_socket",
    )
    step_panel.inertial = Inertial.from_geometry(
        Box((0.82, 0.036, 0.225)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.010, 0.1125)),
    )

    release_handle = model.part("release_handle")
    release_handle.visual(
        Cylinder(radius=0.006, length=0.142),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="handle_pivot_barrel",
    )
    release_handle.visual(
        Box((0.118, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.002, -0.018)),
        material=handle_black,
        name="handle_paddle",
    )
    release_handle.visual(
        Cylinder(radius=0.004, length=0.090),
        origin=Origin(xyz=(0.0, 0.003, -0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    release_handle.inertial = Inertial.from_geometry(
        Box((0.142, 0.018, 0.038)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.003, -0.018)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_surround,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.030, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "tailgate_to_step_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=step_panel,
        origin=Origin(xyz=(0.0, 0.014, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=math.radians(-95.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "step_panel_to_release_handle",
        ArticulationType.REVOLUTE,
        parent=step_panel,
        child=release_handle,
        origin=Origin(xyz=(0.0, 0.028, 0.186)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    micro_contact_tol = 1e-5
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol)
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

    bed_surround = object_model.get_part("bed_surround")
    tailgate = object_model.get_part("tailgate")
    step_panel = object_model.get_part("step_panel")
    release_handle = object_model.get_part("release_handle")

    bed_to_tailgate = object_model.get_articulation("bed_to_tailgate")
    tailgate_to_step_panel = object_model.get_articulation("tailgate_to_step_panel")
    step_panel_to_release_handle = object_model.get_articulation("step_panel_to_release_handle")

    ctx.expect_contact(
        tailgate,
        bed_surround,
        contact_tol=micro_contact_tol,
        name="tailgate_touches_bed_supports",
    )
    ctx.expect_contact(
        step_panel,
        tailgate,
        contact_tol=micro_contact_tol,
        name="step_panel_touches_tailgate",
    )
    ctx.expect_contact(
        release_handle,
        step_panel,
        contact_tol=micro_contact_tol,
        name="handle_touches_step_panel",
    )

    tailgate_limits = bed_to_tailgate.motion_limits
    step_limits = tailgate_to_step_panel.motion_limits
    handle_limits = step_panel_to_release_handle.motion_limits
    assert tailgate_limits is not None
    assert step_limits is not None
    assert handle_limits is not None

    with ctx.pose(
        {
            bed_to_tailgate: tailgate_limits.lower,
            tailgate_to_step_panel: step_limits.upper,
            step_panel_to_release_handle: handle_limits.lower,
        }
    ):
        closed_tail_aabb = ctx.part_world_aabb(tailgate)
        closed_bed_aabb = ctx.part_world_aabb(bed_surround)
        closed_step_aabb = ctx.part_world_aabb(step_panel)
        closed_handle_aabb = ctx.part_world_aabb(release_handle)
        closed_tail_pos = ctx.part_world_position(tailgate)
        closed_bed_pos = ctx.part_world_position(bed_surround)
        assert closed_tail_aabb is not None
        assert closed_bed_aabb is not None
        assert closed_step_aabb is not None
        assert closed_handle_aabb is not None
        assert closed_tail_pos is not None
        assert closed_bed_pos is not None

        tailgate_width = closed_tail_aabb[1][0] - closed_tail_aabb[0][0]
        tailgate_height = closed_tail_aabb[1][2] - closed_tail_aabb[0][2]
        side_margin_left = closed_tail_aabb[0][0] - closed_bed_aabb[0][0]
        side_margin_right = closed_bed_aabb[1][0] - closed_tail_aabb[1][0]

        ctx.check(
            "tailgate_is_positioned_between_bed_sides",
            side_margin_left > 0.10 and side_margin_right > 0.10,
            details="Tailgate should sit between the simplified bed-side supports with visible lateral clearance.",
        )
        ctx.check(
            "tailgate_height_matches_bed_opening_scale",
            0.52 <= tailgate_height <= 0.60 and 1.50 <= tailgate_width <= 1.62,
            details="Pickup tailgate should read as a full-width panel with realistic full-size proportions.",
        )
        ctx.check(
            "tailgate_closed_near_bed_opening_plane",
            -0.06 <= closed_tail_pos[1] - closed_bed_pos[1] <= -0.015,
            details="Closed tailgate should sit just behind the bed opening plane.",
        )
        ctx.expect_gap(
            tailgate,
            step_panel,
            axis="z",
            positive_elem="upper_inner_panel",
            negative_elem="step_face",
            min_gap=0.03,
            max_gap=0.06,
            name="step_panel_has_upper_recess_clearance",
        )
        ctx.expect_within(
            step_panel,
            tailgate,
            axes="x",
            margin=0.02,
            name="step_panel_nested_within_tailgate_width",
        )
        ctx.check(
            "handle_sits_flush_in_closed_pose",
            closed_handle_aabb[0][1] >= closed_step_aabb[1][1] - 0.002
            and closed_handle_aabb[1][1] <= closed_step_aabb[1][1] + 0.040,
            details="Closed handle should live near the step panel face instead of protruding far outward.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tailgate_closed_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol, name="tailgate_closed_no_floating")

    with ctx.pose({bed_to_tailgate: tailgate_limits.upper}):
        open_tail_aabb = ctx.part_world_aabb(tailgate)
        assert open_tail_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="tailgate_open_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol, name="tailgate_open_no_floating")
        ctx.check(
            "tailgate_rotates_downward",
            open_tail_aabb[0][1] < closed_tail_aabb[0][1] - 0.40
            and open_tail_aabb[1][2] < closed_tail_aabb[1][2] - 0.45,
            details="Open tailgate should rotate down to a near-horizontal loading position.",
        )

    with ctx.pose({bed_to_tailgate: tailgate_limits.upper, tailgate_to_step_panel: step_limits.upper}):
        stowed_step_aabb = ctx.part_world_aabb(step_panel)
        assert stowed_step_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="step_panel_closed_on_open_tailgate_no_overlap")
        ctx.fail_if_isolated_parts(
            contact_tol=micro_contact_tol,
            name="step_panel_closed_on_open_tailgate_no_floating",
        )
    with ctx.pose({bed_to_tailgate: tailgate_limits.upper, tailgate_to_step_panel: step_limits.lower}):
        deployed_step_aabb = ctx.part_world_aabb(step_panel)
        assert deployed_step_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="step_panel_open_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol, name="step_panel_open_no_floating")
        ctx.check(
            "step_panel_folds_up_from_tailgate_surface",
            deployed_step_aabb[1][2] > stowed_step_aabb[1][2] + 0.16
            and deployed_step_aabb[0][1] > stowed_step_aabb[0][1] + 0.15,
            details="Deployed step panel should rise above the dropped tailgate and move forward toward the cargo bed.",
        )

    with ctx.pose(
        {
            bed_to_tailgate: tailgate_limits.lower,
            tailgate_to_step_panel: step_limits.upper,
            step_panel_to_release_handle: handle_limits.lower,
        }
    ):
        handle_closed_aabb = ctx.part_world_aabb(release_handle)
        assert handle_closed_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_closed_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol, name="handle_closed_no_floating")
    with ctx.pose(
        {
            bed_to_tailgate: tailgate_limits.lower,
            tailgate_to_step_panel: step_limits.upper,
            step_panel_to_release_handle: handle_limits.upper,
        }
    ):
        handle_open_aabb = ctx.part_world_aabb(release_handle)
        assert handle_open_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_open_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=micro_contact_tol, name="handle_open_no_floating")
        ctx.check(
            "release_handle_rotates_outward",
            handle_open_aabb[1][1] > handle_closed_aabb[1][1] + 0.010,
            details="Release handle should pivot outward from the step panel face on its local hinge.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
