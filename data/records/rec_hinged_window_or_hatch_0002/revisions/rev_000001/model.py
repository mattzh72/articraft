from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="awning_window", assets=ASSETS)

    frame_white = model.material("frame_white", rgba=(0.92, 0.93, 0.94, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.53, 0.56, 0.60, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.16, 0.18, 0.19, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.67, 0.83, 0.92, 0.38))

    frame_width = 1.20
    frame_height = 0.90
    frame_depth = 0.095
    stile_width = 0.075
    head_height = 0.070
    sill_height = 0.080

    sash_width = 1.038
    sash_depth = 0.048
    sash_top_rail_height = 0.055
    sash_bottom_rail_height = 0.065
    sash_side_stile_width = 0.055
    sash_axis_y = 0.024
    sash_axis_z = 0.394
    sash_center_y = -0.006
    sash_top_rail_center_z = -0.0495
    sash_bottom_rail_center_z = -0.7265
    sash_side_stile_height = 0.652
    sash_side_stile_center_z = -0.368

    frame = model.part("outer_frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=26.0,
        origin=Origin(),
    )
    frame.visual(
        Box((stile_width, frame_depth, frame_height)),
        origin=Origin(xyz=(-(frame_width - stile_width) * 0.5, 0.0, 0.0)),
        material=frame_white,
        name="left_stile",
    )
    frame.visual(
        Box((stile_width, frame_depth, frame_height)),
        origin=Origin(xyz=((frame_width - stile_width) * 0.5, 0.0, 0.0)),
        material=frame_white,
        name="right_stile",
    )
    frame.visual(
        Box((frame_width - 2.0 * stile_width, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.0385, 0.390)),
        material=frame_white,
        name="head",
    )
    frame.visual(
        Box((frame_width - 2.0 * stile_width, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.0355, 0.430)),
        material=frame_white,
        name="head_cap",
    )
    frame.visual(
        Box((frame_width - 2.0 * stile_width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, -(frame_height - sill_height) * 0.5)),
        material=frame_white,
        name="sill",
    )
    frame.visual(
        Box((frame_width - 2.0 * stile_width, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.014, -(frame_height - sill_height) * 0.5 + 0.036)),
        material=gasket_gray,
        name="inner_sill_gasket",
    )

    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        hinge_x = x_sign * 0.360
        frame.visual(
            Box((0.050, 0.010, 0.020)),
            origin=Origin(xyz=(hinge_x, 0.031, 0.448)),
            material=hinge_gray,
            name=f"{side_name}_hinge_mount",
        )
        frame.visual(
            Box((0.024, 0.008, 0.062)),
            origin=Origin(xyz=(hinge_x, 0.024, 0.430)),
            material=hinge_gray,
            name=f"{side_name}_hinge_hanger",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.044),
            origin=Origin(xyz=(hinge_x, sash_axis_y, sash_axis_z + 0.001), rpy=(0.0, pi * 0.5, 0.0)),
            material=hinge_gray,
            name=f"{side_name}_hinge_pin",
        )

    sash = model.part("sash")
    sash.inertial = Inertial.from_geometry(
        Box((sash_width, sash_depth, 0.772)),
        mass=18.0,
        origin=Origin(xyz=(0.0, sash_center_y, -0.376)),
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_top_rail_height)),
        origin=Origin(xyz=(0.0, sash_center_y, sash_top_rail_center_z)),
        material=frame_white,
        name="top_rail",
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_bottom_rail_height)),
        origin=Origin(xyz=(0.0, sash_center_y, sash_bottom_rail_center_z)),
        material=frame_white,
        name="bottom_rail",
    )
    sash.visual(
        Box((sash_side_stile_width, sash_depth, sash_side_stile_height)),
        origin=Origin(xyz=(-(sash_width - sash_side_stile_width) * 0.5, sash_center_y, sash_side_stile_center_z)),
        material=frame_white,
        name="left_stile",
    )
    sash.visual(
        Box((sash_side_stile_width, sash_depth, sash_side_stile_height)),
        origin=Origin(xyz=((sash_width - sash_side_stile_width) * 0.5, sash_center_y, sash_side_stile_center_z)),
        material=frame_white,
        name="right_stile",
    )
    sash.visual(
        Box((0.950, 0.020, 0.650)),
        origin=Origin(xyz=(0.0, -0.004, -0.368)),
        material=glass_blue,
        name="glass_panel",
    )
    sash.visual(
        Box((0.978, 0.006, 0.678)),
        origin=Origin(xyz=(0.0, 0.012, -0.368)),
        material=gasket_gray,
        name="outer_glazing_gasket",
    )

    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        sash.visual(
            Cylinder(radius=0.007, length=0.030),
            origin=Origin(xyz=(x_sign * 0.360, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
            material=hinge_gray,
            name=f"{side_name}_hinge_barrel",
        )
        sash.visual(
            Box((0.048, 0.010, 0.034)),
            origin=Origin(xyz=(x_sign * 0.360, 0.009, -0.017)),
            material=hinge_gray,
            name=f"{side_name}_hinge_leaf",
        )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, sash_axis_y, sash_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=1.047),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    top_hinge = object_model.get_articulation("top_hinge")

    frame_left_stile = frame.get_visual("left_stile")
    frame_right_stile = frame.get_visual("right_stile")
    frame_head = frame.get_visual("head")
    frame_sill = frame.get_visual("sill")
    frame_left_hinge_hanger = frame.get_visual("left_hinge_hanger")
    frame_right_hinge_hanger = frame.get_visual("right_hinge_hanger")
    frame_left_hinge_pin = frame.get_visual("left_hinge_pin")
    frame_right_hinge_pin = frame.get_visual("right_hinge_pin")

    sash_left_stile = sash.get_visual("left_stile")
    sash_right_stile = sash.get_visual("right_stile")
    sash_top_rail = sash.get_visual("top_rail")
    sash_bottom_rail = sash.get_visual("bottom_rail")
    sash_left_hinge_barrel = sash.get_visual("left_hinge_barrel")
    sash_right_hinge_barrel = sash.get_visual("right_hinge_barrel")
    sash_left_hinge_leaf = sash.get_visual("left_hinge_leaf")
    sash_right_hinge_leaf = sash.get_visual("right_hinge_leaf")
    sash_glass = sash.get_visual("glass_panel")

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
    ctx.allow_overlap(
        frame,
        sash,
        elem_a=frame_left_hinge_pin,
        elem_b=sash_left_hinge_barrel,
        reason="left awning hinge pin runs inside the sash hinge barrel",
    )
    ctx.allow_overlap(
        frame,
        sash,
        elem_a=frame_right_hinge_pin,
        elem_b=sash_right_hinge_barrel,
        reason="right awning hinge pin runs inside the sash hinge barrel",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check("outer_frame_present", frame is not None, "missing outer frame part")
    ctx.check("sash_present", sash is not None, "missing glazed sash part")
    ctx.check("top_hinge_present", top_hinge is not None, "missing awning hinge articulation")
    ctx.check(
        "hinge_axis_is_horizontal_x",
        tuple(round(v, 6) for v in top_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected hinge axis (1, 0, 0), got {top_hinge.axis}",
    )
    ctx.check(
        "hinge_opens_about_60_deg",
        abs(top_hinge.motion_limits.lower - 0.0) < 1e-6 and abs(top_hinge.motion_limits.upper - 1.047) < 0.02,
        f"expected 0 to about 1.047 rad travel, got {top_hinge.motion_limits}",
    )

    ctx.expect_contact(frame, sash, elem_a=frame_left_hinge_pin, elem_b=sash_left_hinge_barrel, name="left_hinge_pivot_contact")
    ctx.expect_contact(frame, sash, elem_a=frame_right_hinge_pin, elem_b=sash_right_hinge_barrel, name="right_hinge_pivot_contact")
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem=frame_left_hinge_hanger,
        negative_elem=sash_left_hinge_leaf,
        min_gap=0.004,
        max_gap=0.020,
        name="left_hinge_leaf_below_hanger",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem=frame_right_hinge_hanger,
        negative_elem=sash_right_hinge_leaf,
        min_gap=0.004,
        max_gap=0.020,
        name="right_hinge_leaf_below_hanger",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="x",
        positive_elem=sash_left_stile,
        negative_elem=frame_left_stile,
        min_gap=0.004,
        max_gap=0.012,
        name="left_reveal_gap",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="x",
        positive_elem=frame_right_stile,
        negative_elem=sash_right_stile,
        min_gap=0.004,
        max_gap=0.012,
        name="right_reveal_gap",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem=frame_head,
        negative_elem=sash_top_rail,
        min_gap=0.005,
        max_gap=0.015,
        name="top_reveal_gap",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem=sash_bottom_rail,
        negative_elem=frame_sill,
        min_gap=0.001,
        max_gap=0.006,
        name="bottom_reveal_gap",
    )
    ctx.expect_overlap(sash, frame, axes="xz", min_overlap=0.72, name="closed_sash_within_frame_footprint")
    ctx.expect_within(sash, sash, axes="xz", inner_elem=sash_glass, margin=0.0, name="glass_within_sash_perimeter")

    with ctx.pose({top_hinge: 0.55}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem=sash_bottom_rail,
            min_gap=0.22,
            name="mid_open_bottom_projects_outward",
        )
        ctx.expect_origin_distance(sash, frame, axes="x", max_dist=0.001, name="mid_open_hinge_keeps_sash_centered")

    with ctx.pose({top_hinge: 1.047}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem=sash_bottom_rail,
            min_gap=0.50,
            name="fully_open_bottom_projects_outward",
        )
        ctx.expect_overlap(sash, frame, axes="x", min_overlap=0.95, name="fully_open_still_hung_from_top_edge")
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem=sash_top_rail,
            negative_elem=frame_head,
            min_gap=0.025,
            name="fully_open_top_rail_clears_head_lip",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
