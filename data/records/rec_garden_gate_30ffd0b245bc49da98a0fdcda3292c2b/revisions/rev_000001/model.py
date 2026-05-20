from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="bifold_garden_gate")

    cedar = model.material("cedar", rgba=(0.63, 0.42, 0.24, 1.0))
    cedar_dark = model.material("cedar_dark", rgba=(0.50, 0.31, 0.16, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.71, 1.0))
    blackened = model.material("blackened_steel", rgba=(0.14, 0.14, 0.15, 1.0))

    post_size = 0.09
    post_height = 1.55
    post_half_span = 0.525
    post_inner_x = post_half_span - (post_size / 2.0)

    tie_width = (2.0 * post_half_span) + post_size
    tie_depth = 0.12
    tie_height = 0.10

    gate_bottom = 0.06
    gate_height = 1.14
    panel_thickness = 0.034

    hinge_radius = 0.010
    hinge_barrel_length = 0.14
    upper_hinge_z = 0.94
    lower_hinge_z = 0.20
    hinge_axis_y = (post_size / 2.0) + 0.012

    panel_visible_width = 0.450
    stile_width = 0.045
    top_rail_height = 0.070
    bottom_rail_height = 0.090
    slat_width = 0.026
    slat_thickness = 0.018

    leaf_inner_x = hinge_radius
    leaf_outer_x = leaf_inner_x + panel_visible_width
    fold_hinge_x = 0.470

    hardware_plate_thickness = 0.008
    leaf_center_y = hinge_radius + (panel_thickness / 2.0) + 0.003
    front_plate_y = leaf_center_y + (panel_thickness / 2.0) + (hardware_plate_thickness / 2.0)
    back_plate_y = leaf_center_y - (panel_thickness / 2.0) - (hardware_plate_thickness / 2.0)
    frame_hinge_plate_y = (post_size / 2.0) + (hardware_plate_thickness / 2.0)
    knuckle_width = 0.016
    knuckle_depth = 0.032
    knuckle_center_y = 0.006
    frame_knuckle_center_y = hinge_axis_y - 0.006

    latch_plate_thickness = 0.006
    latch_plate_y = leaf_center_y + (panel_thickness / 2.0) + (latch_plate_thickness / 2.0)
    latch_boss_length = 0.018
    latch_boss_center_y = (
        leaf_center_y
        + (panel_thickness / 2.0)
        + latch_plate_thickness
        + (latch_boss_length / 2.0)
    )
    latch_pivot_y = (
        leaf_center_y
        + (panel_thickness / 2.0)
        + latch_plate_thickness
        + latch_boss_length
    )
    latch_pivot_x = 0.426
    latch_pivot_z = 0.69
    striker_y = (post_size / 2.0) + 0.025
    latch_stop_center_y = leaf_center_y + (panel_thickness / 2.0) + latch_plate_thickness + 0.007

    frame = model.part("frame")
    frame.visual(
        Box((post_size, post_size, post_height)),
        origin=Origin(xyz=(-post_half_span, 0.0, post_height / 2.0)),
        material=cedar_dark,
        name="left_post",
    )
    frame.visual(
        Box((post_size, post_size, post_height)),
        origin=Origin(xyz=(post_half_span, 0.0, post_height / 2.0)),
        material=cedar_dark,
        name="right_post",
    )
    frame.visual(
        Box((tie_width, tie_depth, tie_height)),
        origin=Origin(xyz=(0.0, 0.0, -(tie_height / 2.0))),
        material=cedar_dark,
        name="subgrade_tie",
    )
    frame.visual(
        Box((0.11, 0.11, 0.03)),
        origin=Origin(xyz=(-post_half_span, 0.0, post_height + 0.015)),
        material=cedar_dark,
        name="left_post_cap",
    )
    frame.visual(
        Box((0.11, 0.11, 0.03)),
        origin=Origin(xyz=(post_half_span, 0.0, post_height + 0.015)),
        material=cedar_dark,
        name="right_post_cap",
    )
    for hinge_z, suffix in ((lower_hinge_z, "lower"), (upper_hinge_z, "upper")):
        frame.visual(
            Cylinder(radius=hinge_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(-0.470, hinge_axis_y, gate_bottom + hinge_z)),
            material=galvanized,
            name=f"frame_left_post_hinge_barrel_{suffix}",
        )
        frame.visual(
            Box((0.054, hardware_plate_thickness, 0.036)),
            origin=Origin(
                xyz=(-0.497, frame_hinge_plate_y, gate_bottom + hinge_z),
            ),
            material=galvanized,
            name=f"frame_left_post_hinge_plate_{suffix}",
        )
        frame.visual(
            Box((knuckle_width, knuckle_depth, 0.036)),
            origin=Origin(
                xyz=(-0.488, frame_knuckle_center_y, gate_bottom + hinge_z),
            ),
            material=galvanized,
            name=f"frame_left_post_hinge_knuckle_{suffix}",
        )
    frame.visual(
        Box((0.012, 0.050, 0.160)),
        origin=Origin(xyz=(0.474, striker_y, 0.67)),
        material=blackened,
        name="frame_striker_plate",
    )
    frame.inertial = Inertial.from_geometry(
        Box((tie_width, tie_depth, post_height + 0.03)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + 0.03) / 2.0)),
    )

    def add_leaf(part_name: str, prefix: str, *, with_fold_barrels: bool, with_latch_mount: bool) -> object:
        leaf = model.part(part_name)
        leaf.visual(
            Box((stile_width, panel_thickness, gate_height)),
            origin=Origin(
                xyz=(leaf_inner_x + (stile_width / 2.0), leaf_center_y, gate_height / 2.0),
            ),
            material=cedar,
            name=f"{prefix}_left_stile",
        )
        leaf.visual(
            Box((stile_width, panel_thickness, gate_height)),
            origin=Origin(
                xyz=(leaf_outer_x - (stile_width / 2.0), leaf_center_y, gate_height / 2.0),
            ),
            material=cedar,
            name=f"{prefix}_free_stile" if with_latch_mount else f"{prefix}_right_stile",
        )
        leaf.visual(
            Box((panel_visible_width - (2.0 * stile_width), panel_thickness, bottom_rail_height)),
            origin=Origin(
                xyz=(
                    (leaf_inner_x + leaf_outer_x) / 2.0,
                    leaf_center_y,
                    bottom_rail_height / 2.0,
                ),
            ),
            material=cedar,
            name=f"{prefix}_bottom_rail",
        )
        leaf.visual(
            Box((panel_visible_width - (2.0 * stile_width), panel_thickness, top_rail_height)),
            origin=Origin(
                xyz=(
                    (leaf_inner_x + leaf_outer_x) / 2.0,
                    leaf_center_y,
                    gate_height - (top_rail_height / 2.0),
                ),
            ),
            material=cedar,
            name=f"{prefix}_top_rail",
        )

        slat_height = gate_height - top_rail_height - bottom_rail_height
        slat_center_z = bottom_rail_height + (slat_height / 2.0)
        for index, slat_x in enumerate((0.145, 0.235, 0.325), start=1):
            leaf.visual(
                Box((slat_width, slat_thickness, slat_height)),
                origin=Origin(xyz=(slat_x, leaf_center_y, slat_center_z)),
                material=cedar_dark,
                name=f"{prefix}_slat_{index}",
            )

        for hinge_z, suffix in ((lower_hinge_z, "lower"), (upper_hinge_z, "upper")):
            leaf.visual(
                Box((0.100, hardware_plate_thickness, 0.030)),
                origin=Origin(
                    xyz=(0.060, front_plate_y, hinge_z),
                ),
                material=galvanized,
                name=f"{prefix}_front_hinge_strap_{suffix}",
            )
            leaf.visual(
                Box((0.100, hardware_plate_thickness, 0.030)),
                origin=Origin(
                    xyz=(0.060, back_plate_y, hinge_z),
                ),
                material=galvanized,
                name=f"{prefix}_back_hinge_strap_{suffix}",
            )
            leaf.visual(
                Box((knuckle_width, knuckle_depth, 0.036)),
                origin=Origin(
                    xyz=(0.018, knuckle_center_y, hinge_z),
                ),
                material=galvanized,
                name=f"{prefix}_left_hinge_knuckle_{suffix}",
            )

        if with_fold_barrels:
            for hinge_z, suffix in ((lower_hinge_z, "lower"), (upper_hinge_z, "upper")):
                leaf.visual(
                    Cylinder(radius=hinge_radius, length=hinge_barrel_length),
                    origin=Origin(xyz=(fold_hinge_x, 0.0, hinge_z)),
                    material=galvanized,
                    name=f"{prefix}_right_hinge_barrel_{suffix}",
                )
                leaf.visual(
                    Box((0.100, hardware_plate_thickness, 0.030)),
                    origin=Origin(
                        xyz=(0.410, back_plate_y, hinge_z),
                    ),
                    material=galvanized,
                    name=f"{prefix}_right_hinge_strap_{suffix}",
                )
                leaf.visual(
                    Box((knuckle_width, knuckle_depth, 0.036)),
                    origin=Origin(
                        xyz=(fold_hinge_x - 0.018, knuckle_center_y, hinge_z),
                    ),
                    material=galvanized,
                    name=f"{prefix}_right_hinge_knuckle_{suffix}",
                )

        if with_latch_mount:
            leaf.visual(
                Box((0.060, latch_plate_thickness, 0.190)),
                origin=Origin(xyz=(0.430, latch_plate_y, latch_pivot_z)),
                material=blackened,
                name=f"{prefix}_latch_plate",
            )
            leaf.visual(
                Cylinder(radius=0.010, length=latch_boss_length),
                origin=Origin(
                    xyz=(latch_pivot_x, latch_boss_center_y, latch_pivot_z),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=blackened,
                name=f"{prefix}_latch_pivot_boss",
            )
            leaf.visual(
                Box((0.020, 0.014, 0.040)),
                origin=Origin(xyz=(0.448, latch_stop_center_y, latch_pivot_z - 0.020)),
                material=blackened,
                name=f"{prefix}_latch_stop",
            )

        inertial_width = fold_hinge_x + hinge_radius if with_fold_barrels else leaf_outer_x
        leaf.inertial = Inertial.from_geometry(
            Box((inertial_width, panel_thickness, gate_height)),
            mass=10.0,
            origin=Origin(xyz=(inertial_width / 2.0, leaf_center_y, gate_height / 2.0)),
        )
        return leaf

    outer_panel = add_leaf(
        "outer_panel",
        "outer",
        with_fold_barrels=True,
        with_latch_mount=False,
    )
    inner_panel = add_leaf(
        "inner_panel",
        "inner",
        with_fold_barrels=False,
        with_latch_mount=True,
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blackened,
        name="latch_hub",
    )
    latch_lever.visual(
        Box((0.010, 0.014, 0.042)),
        origin=Origin(xyz=(0.0, 0.009, -0.021)),
        material=blackened,
        name="latch_arm",
    )
    latch_lever.visual(
        Box((0.014, 0.010, 0.074)),
        origin=Origin(xyz=(0.0, 0.012, -0.061)),
        material=blackened,
        name="latch_handle",
    )
    latch_lever.visual(
        Box((0.030, 0.008, 0.012)),
        origin=Origin(xyz=(0.018, 0.010, -0.014)),
        material=blackened,
        name="latch_tongue",
    )
    latch_lever.inertial = Inertial.from_geometry(
        Box((0.036, 0.016, 0.090)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.008, -0.045)),
    )

    model.articulation(
        "post_to_outer_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=outer_panel,
        origin=Origin(xyz=(-0.470, hinge_axis_y, gate_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "outer_panel_to_inner_panel",
        ArticulationType.REVOLUTE,
        parent=outer_panel,
        child=inner_panel,
        origin=Origin(xyz=(fold_hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.90,
            upper=0.0,
        ),
    )
    model.articulation(
        "inner_panel_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=inner_panel,
        child=latch_lever,
        origin=Origin(xyz=(latch_pivot_x, latch_pivot_y, latch_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    outer_panel = object_model.get_part("outer_panel")
    inner_panel = object_model.get_part("inner_panel")
    latch_lever = object_model.get_part("latch_lever")

    outer_hinge = object_model.get_articulation("post_to_outer_panel")
    fold_hinge = object_model.get_articulation("outer_panel_to_inner_panel")
    latch_pivot = object_model.get_articulation("inner_panel_to_latch_lever")

    outer_left_stile = outer_panel.get_visual("outer_left_stile")
    outer_fold_barrel = outer_panel.get_visual("outer_right_hinge_barrel_lower")
    inner_left_stile = inner_panel.get_visual("inner_left_stile")
    inner_free_stile = inner_panel.get_visual("inner_free_stile")
    striker_plate = frame.get_visual("frame_striker_plate")
    post_barrel = frame.get_visual("frame_left_post_hinge_barrel_lower")
    latch_boss = inner_panel.get_visual("inner_latch_pivot_boss")
    latch_hub = latch_lever.get_visual("latch_hub")
    latch_handle = latch_lever.get_visual("latch_handle")
    latch_tongue = latch_lever.get_visual("latch_tongue")

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
        "outer_hinge_is_vertical",
        tuple(outer_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical z-axis, got {outer_hinge.axis}",
    )
    ctx.check(
        "fold_hinge_is_vertical",
        tuple(fold_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical z-axis, got {fold_hinge.axis}",
    )
    ctx.check(
        "latch_pivot_uses_horizontal_axis",
        tuple(latch_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"expected horizontal y-axis, got {latch_pivot.axis}",
    )
    ctx.check(
        "fold_hinge_limits_allow_folding",
        (
            fold_hinge.motion_limits is not None
            and fold_hinge.motion_limits.lower is not None
            and fold_hinge.motion_limits.upper is not None
            and fold_hinge.motion_limits.lower < -1.0
            and abs(fold_hinge.motion_limits.upper) < 1e-9
        ),
        details="fold hinge should close straight at 0 rad and fold back well past 90 degrees",
    )

    with ctx.pose({outer_hinge: 0.0, fold_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            outer_panel,
            frame,
            axis="x",
            positive_elem=outer_left_stile,
            negative_elem=post_barrel,
            max_gap=0.0005,
            max_penetration=1e-6,
            name="outer_panel_seats_on_post_hinge",
        )
        ctx.expect_gap(
            inner_panel,
            outer_panel,
            axis="x",
            positive_elem=inner_left_stile,
            negative_elem=outer_fold_barrel,
            max_gap=0.0005,
            max_penetration=1e-6,
            name="inner_panel_seats_on_fold_hinge",
        )
        ctx.expect_gap(
            frame,
            inner_panel,
            axis="x",
            positive_elem=striker_plate,
            negative_elem=inner_free_stile,
            min_gap=0.006,
            max_gap=0.020,
            name="free_edge_gap_to_striker_is_small",
        )
        ctx.expect_gap(
            latch_lever,
            inner_panel,
            axis="y",
            positive_elem=latch_hub,
            negative_elem=latch_boss,
            max_gap=0.0005,
            max_penetration=1e-6,
            name="latch_handle_is_mounted_on_pivot_boss",
        )
        ctx.expect_overlap(
            inner_panel,
            frame,
            axes="z",
            min_overlap=0.150,
            elem_a=inner_free_stile,
            elem_b=striker_plate,
            name="latch_lines_up_with_striker_height",
        )
        ctx.expect_gap(
            frame,
            latch_lever,
            axis="x",
            positive_elem=striker_plate,
            negative_elem=latch_tongue,
            min_gap=0.006,
            max_gap=0.018,
            name="closed_latch_tongue_sits_near_striker",
        )

    with ctx.pose({latch_pivot: 0.60}):
        ctx.expect_gap(
            frame,
            latch_lever,
            axis="x",
            positive_elem=striker_plate,
            negative_elem=latch_tongue,
            min_gap=0.012,
            name="lifting_latch_retracts_tongue_from_striker",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
