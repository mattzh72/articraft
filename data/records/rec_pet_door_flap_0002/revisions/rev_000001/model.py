from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_OUTER_W = 0.240
FRAME_OUTER_H = 0.280
FRAME_DEPTH = 0.028
FRAME_RAIL = 0.030

FRAME_FLANGE_W = 0.300
FRAME_FLANGE_H = 0.340
FRAME_FLANGE_DEPTH = 0.005

OPENING_W = FRAME_OUTER_W - 2.0 * FRAME_RAIL
OPENING_H = FRAME_OUTER_H - 2.0 * FRAME_RAIL

FLAP_W = OPENING_W - 0.012
FLAP_H = OPENING_H - 0.015
FLAP_T = 0.003
HINGE_RADIUS = 0.004
HINGE_AXIS_Z = OPENING_H / 2.0 - 0.008
HINGE_SWING = math.pi / 4.0
HINGE_CAP_W = 0.008
HINGE_CAP_H = 0.042


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_insert", assets=ASSETS)

    frame_plastic = model.material("frame_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    flap_vinyl = model.material("flap_vinyl", rgba=(0.35, 0.42, 0.48, 0.48))
    seal_black = model.material("seal_black", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_OUTER_H)),
        origin=Origin(xyz=(-(OPENING_W / 2.0 + FRAME_RAIL / 2.0), 0.0, 0.0)),
        material=frame_plastic,
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_OUTER_H)),
        origin=Origin(xyz=((OPENING_W / 2.0 + FRAME_RAIL / 2.0), 0.0, 0.0)),
        material=frame_plastic,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_OUTER_W, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_H / 2.0 + FRAME_RAIL / 2.0)),
        material=frame_plastic,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_OUTER_W, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -(OPENING_H / 2.0 + FRAME_RAIL / 2.0))),
        material=frame_plastic,
        name="bottom_rail",
    )
    frame.visual(
        Box((HINGE_CAP_W, 0.010, HINGE_CAP_H)),
        origin=Origin(
            xyz=(
                -(FLAP_W / 2.0 + HINGE_CAP_W / 2.0),
                0.0,
                HINGE_AXIS_Z + (0.014 - HINGE_RADIUS) / 2.0,
            )
        ),
        material=frame_plastic,
        name="left_hinge_cap",
    )
    frame.visual(
        Box((HINGE_CAP_W, 0.010, HINGE_CAP_H)),
        origin=Origin(
            xyz=(
                FLAP_W / 2.0 + HINGE_CAP_W / 2.0,
                0.0,
                HINGE_AXIS_Z + (0.014 - HINGE_RADIUS) / 2.0,
            )
        ),
        material=frame_plastic,
        name="right_hinge_cap",
    )

    flange_side_w = (FRAME_FLANGE_W - FRAME_OUTER_W) / 2.0
    flange_top_h = (FRAME_FLANGE_H - FRAME_OUTER_H) / 2.0
    flange_y = -(FRAME_DEPTH / 2.0 + FRAME_FLANGE_DEPTH / 2.0)

    frame.visual(
        Box((flange_side_w, FRAME_FLANGE_DEPTH, FRAME_FLANGE_H)),
        origin=Origin(xyz=(-(FRAME_OUTER_W / 2.0 + flange_side_w / 2.0), flange_y, 0.0)),
        material=frame_plastic,
        name="left_flange",
    )
    frame.visual(
        Box((flange_side_w, FRAME_FLANGE_DEPTH, FRAME_FLANGE_H)),
        origin=Origin(xyz=((FRAME_OUTER_W / 2.0 + flange_side_w / 2.0), flange_y, 0.0)),
        material=frame_plastic,
        name="right_flange",
    )
    frame.visual(
        Box((FRAME_FLANGE_W, FRAME_FLANGE_DEPTH, flange_top_h)),
        origin=Origin(xyz=(0.0, flange_y, FRAME_OUTER_H / 2.0 + flange_top_h / 2.0)),
        material=frame_plastic,
        name="top_flange",
    )
    frame.visual(
        Box((FRAME_FLANGE_W, FRAME_FLANGE_DEPTH, flange_top_h)),
        origin=Origin(xyz=(0.0, flange_y, -(FRAME_OUTER_H / 2.0 + flange_top_h / 2.0))),
        material=frame_plastic,
        name="bottom_flange",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=HINGE_RADIUS, length=FLAP_W),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="hinge_sleeve",
    )
    flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, 0.0, -(FLAP_H / 2.0 + HINGE_RADIUS - 0.0005))),
        material=flap_vinyl,
        name="flap_panel",
    )
    flap.visual(
        Box((FLAP_W * 0.92, FLAP_T * 1.8, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -(FLAP_H + HINGE_RADIUS - 0.0055))),
        material=seal_black,
        name="bottom_weight",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-HINGE_SWING,
            upper=HINGE_SWING,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    flap_hinge = object_model.get_articulation("frame_to_flap")

    left_jamb = frame.get_visual("left_jamb")
    right_jamb = frame.get_visual("right_jamb")
    top_rail = frame.get_visual("top_rail")
    bottom_rail = frame.get_visual("bottom_rail")
    left_hinge_cap = frame.get_visual("left_hinge_cap")
    right_hinge_cap = frame.get_visual("right_hinge_cap")
    left_flange = frame.get_visual("left_flange")
    right_flange = frame.get_visual("right_flange")
    top_flange = frame.get_visual("top_flange")
    bottom_flange = frame.get_visual("bottom_flange")

    hinge_sleeve = flap.get_visual("hinge_sleeve")
    flap_panel = flap.get_visual("flap_panel")
    bottom_weight = flap.get_visual("bottom_weight")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "frame_visuals_present",
        all(
            visual is not None
            for visual in (
                left_jamb,
                right_jamb,
                top_rail,
                bottom_rail,
                left_hinge_cap,
                right_hinge_cap,
                left_flange,
                right_flange,
                top_flange,
                bottom_flange,
            )
        ),
        details="Frame should include tunnel rails and a front mounting flange ring.",
    )
    ctx.check(
        "flap_visuals_present",
        all(visual is not None for visual in (hinge_sleeve, flap_panel, bottom_weight)),
        details="Flap should include a hinge sleeve, thin panel, and bottom weight strip.",
    )

    axis_ok = all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(flap_hinge.axis, (1.0, 0.0, 0.0))
    )
    limits = flap_hinge.motion_limits
    limit_ok = (
        limits is not None
        and math.isclose(limits.lower, -HINGE_SWING, abs_tol=1e-6)
        and math.isclose(limits.upper, HINGE_SWING, abs_tol=1e-6)
    )
    ctx.check(
        "hinge_axis_is_horizontal",
        axis_ok,
        details=f"Expected flap hinge axis (1, 0, 0), got {flap_hinge.axis!r}.",
    )
    ctx.check(
        "hinge_limits_match_pet_door_swing",
        limit_ok,
        details="Flap should swing through approximately +/-45 degrees.",
    )

    ctx.expect_origin_distance(
        flap,
        frame,
        axes="x",
        max_dist=1e-6,
        name="flap_is_centered_in_frame_width",
    )
    ctx.expect_origin_gap(
        flap,
        frame,
        axis="z",
        min_gap=0.100,
        max_gap=0.104,
        name="hinge_axis_sits_at_upper_opening",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            min_gap=0.005,
            max_gap=0.007,
            positive_elem=right_jamb,
            negative_elem=flap_panel,
            name="right_side_clearance_at_rest",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="x",
            min_gap=0.005,
            max_gap=0.007,
            positive_elem=flap_panel,
            negative_elem=left_jamb,
            name="left_side_clearance_at_rest",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            min_gap=0.003,
            max_gap=0.005,
            positive_elem=top_rail,
            negative_elem=hinge_sleeve,
            name="top_hinge_clearance_at_rest",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            min_gap=0.003,
            max_gap=0.005,
            positive_elem=flap_panel,
            negative_elem=bottom_rail,
            name="bottom_edge_clearance_at_rest",
        )
        ctx.expect_overlap(
            flap,
            frame,
            axes="x",
            min_overlap=0.165,
            elem_a=flap_panel,
            elem_b=top_rail,
            name="flap_width_remains_under_top_rail",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=left_hinge_cap,
            name="left_hinge_cap_supports_flap_at_rest",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=right_hinge_cap,
            name="right_hinge_cap_supports_flap_at_rest",
        )

    with ctx.pose({flap_hinge: math.radians(40.0)}):
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            min_gap=0.003,
            max_gap=0.005,
            positive_elem=top_rail,
            negative_elem=hinge_sleeve,
            name="top_hinge_clearance_swung_forward",
        )
        ctx.expect_overlap(
            flap,
            frame,
            axes="x",
            min_overlap=0.165,
            elem_a=flap_panel,
            elem_b=top_rail,
            name="flap_stays_laterally_aligned_swung_forward",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=left_hinge_cap,
            name="left_hinge_cap_supports_flap_swung_forward",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=right_hinge_cap,
            name="right_hinge_cap_supports_flap_swung_forward",
        )
        forward_panel_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    with ctx.pose({flap_hinge: math.radians(-40.0)}):
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            min_gap=0.003,
            max_gap=0.005,
            positive_elem=top_rail,
            negative_elem=hinge_sleeve,
            name="top_hinge_clearance_swung_backward",
        )
        ctx.expect_overlap(
            flap,
            frame,
            axes="x",
            min_overlap=0.165,
            elem_a=flap_panel,
            elem_b=top_rail,
            name="flap_stays_laterally_aligned_swung_backward",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=left_hinge_cap,
            name="left_hinge_cap_supports_flap_swung_backward",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_sleeve,
            elem_b=right_hinge_cap,
            name="right_hinge_cap_supports_flap_swung_backward",
        )
        backward_panel_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    ctx.check(
        "flap_swings_forward_of_frame",
        forward_panel_aabb is not None and forward_panel_aabb[1][1] > 0.12,
        details="At +40 degrees the flap panel should project clearly to +Y.",
    )
    ctx.check(
        "flap_swings_backward_of_frame",
        backward_panel_aabb is not None and backward_panel_aabb[0][1] < -0.12,
        details="At -40 degrees the flap panel should project clearly to -Y.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
