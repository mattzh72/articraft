from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="hopper_window")

    frame_w = 1.00
    frame_h = 0.80
    frame_d = 0.085
    stile_w = 0.07
    head_h = 0.07
    sill_h = 0.09

    opening_w = frame_w - 2.0 * stile_w
    opening_bottom = -frame_h * 0.5 + sill_h
    opening_top = frame_h * 0.5 - head_h
    opening_h = opening_top - opening_bottom
    opening_center_z = 0.5 * (opening_top + opening_bottom)

    sash_w = 0.84
    sash_h = 0.612
    sash_d = 0.051
    sash_side = 0.055
    sash_top = 0.060
    sash_bottom = 0.070
    sash_axis_z = -0.302

    stop_w = 0.018
    stop_d = 0.017
    stop_center_y = frame_d * 0.5 - stop_d * 0.5

    liner_d = 0.015
    liner_center_y = -frame_d * 0.5 + liner_d * 0.5

    hinge_x = 0.24
    hinge_barrel_r = 0.008
    hinge_barrel_len = 0.082

    bronze = model.material("bronze", rgba=(0.28, 0.23, 0.18, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.17, 0.14, 0.11, 1.0))
    gasket = model.material("gasket", rgba=(0.08, 0.08, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.68, 1.0))
    glass = model.material("glass", rgba=(0.67, 0.82, 0.90, 0.38))
    handle_black = model.material("handle_black", rgba=(0.16, 0.16, 0.17, 1.0))

    outer_frame = model.part("outer_frame")
    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_w, frame_d, frame_h)),
        mass=18.0,
        origin=Origin(),
    )
    outer_frame.visual(
        Box((stile_w, frame_d, frame_h)),
        origin=Origin(xyz=(-(frame_w - stile_w) * 0.5, 0.0, 0.0)),
        material=bronze,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((stile_w, frame_d, frame_h)),
        origin=Origin(xyz=((frame_w - stile_w) * 0.5, 0.0, 0.0)),
        material=bronze,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((opening_w, frame_d, head_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_h * 0.5 - head_h * 0.5)),
        material=bronze,
        name="head",
    )
    outer_frame.visual(
        Box((opening_w, frame_d, sill_h)),
        origin=Origin(xyz=(0.0, 0.0, -frame_h * 0.5 + sill_h * 0.5)),
        material=bronze,
        name="sill",
    )

    outer_frame.visual(
        Box((stop_w, stop_d, opening_h)),
        origin=Origin(xyz=(-opening_w * 0.5 + stop_w * 0.5, stop_center_y, opening_center_z)),
        material=dark_bronze,
        name="left_stop",
    )
    outer_frame.visual(
        Box((stop_w, stop_d, opening_h)),
        origin=Origin(xyz=(opening_w * 0.5 - stop_w * 0.5, stop_center_y, opening_center_z)),
        material=dark_bronze,
        name="right_stop",
    )
    outer_frame.visual(
        Box((opening_w - 2.0 * stop_w, stop_d, stop_w)),
        origin=Origin(xyz=(0.0, stop_center_y, opening_top - stop_w * 0.5)),
        material=dark_bronze,
        name="top_stop",
    )
    outer_frame.visual(
        Box((opening_w - 2.0 * stop_w, stop_d, stop_w)),
        origin=Origin(xyz=(0.0, stop_center_y, opening_bottom + stop_w * 0.5)),
        material=dark_bronze,
        name="bottom_stop",
    )

    outer_frame.visual(
        Box((0.036, liner_d, opening_h)),
        origin=Origin(xyz=(-(opening_w - 0.036) * 0.5, liner_center_y, opening_center_z)),
        material=bronze,
        name="left_liner",
    )
    outer_frame.visual(
        Box((0.036, liner_d, opening_h)),
        origin=Origin(xyz=((opening_w - 0.036) * 0.5, liner_center_y, opening_center_z)),
        material=bronze,
        name="right_liner",
    )
    outer_frame.visual(
        Box((opening_w - 0.072, liner_d, 0.036)),
        origin=Origin(xyz=(0.0, liner_center_y, opening_top - 0.018)),
        material=bronze,
        name="top_liner",
    )
    outer_frame.visual(
        Box((opening_w - 0.072, liner_d, 0.050)),
        origin=Origin(xyz=(0.0, liner_center_y, opening_bottom + 0.025)),
        material=bronze,
        name="bottom_liner",
    )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = x_sign * hinge_x
        outer_frame.visual(
            Box((0.102, 0.032, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, sash_axis_z - hinge_barrel_r - 0.003)),
            material=steel,
            name=f"{side}_hinge_base",
        )
        outer_frame.visual(
            Box((0.102, 0.008, 0.020)),
            origin=Origin(xyz=(x_pos, 0.014, sash_axis_z - 0.018)),
            material=steel,
            name=f"{side}_hinge_front_cheek",
        )
        outer_frame.visual(
            Box((0.102, 0.008, 0.020)),
            origin=Origin(xyz=(x_pos, -0.014, sash_axis_z - 0.018)),
            material=steel,
            name=f"{side}_hinge_back_cheek",
        )

    sash = model.part("sash")
    sash.inertial = Inertial.from_geometry(
        Box((sash_w, sash_d, sash_h)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, sash_h * 0.5)),
    )
    sash.visual(
        Box((sash_side, sash_d, sash_h)),
        origin=Origin(xyz=(-(sash_w - sash_side) * 0.5, 0.0, sash_h * 0.5)),
        material=aluminum,
        name="left_stile",
    )
    sash.visual(
        Box((sash_side, sash_d, sash_h)),
        origin=Origin(xyz=((sash_w - sash_side) * 0.5, 0.0, sash_h * 0.5)),
        material=aluminum,
        name="right_stile",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_side, sash_d, sash_top)),
        origin=Origin(xyz=(0.0, 0.0, sash_h - sash_top * 0.5)),
        material=aluminum,
        name="top_rail",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_side, sash_d, sash_bottom)),
        origin=Origin(xyz=(0.0, 0.0, sash_bottom * 0.5)),
        material=aluminum,
        name="bottom_rail",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_side + 0.004, 0.022, sash_h - sash_top - sash_bottom + 0.004)),
        origin=Origin(xyz=(0.0, 0.002, 0.5 * (sash_bottom + sash_h - sash_top))),
        material=glass,
        name="glass_panel",
    )
    sash.visual(
        Box((0.014, 0.008, sash_h - sash_top - sash_bottom)),
        origin=Origin(
            xyz=(
                -0.5 * (sash_w - 2.0 * sash_side - 0.014),
                -0.0215,
                0.5 * (sash_bottom + sash_h - sash_top),
            )
        ),
        material=gasket,
        name="left_interior_glazing_gasket",
    )
    sash.visual(
        Box((0.014, 0.008, sash_h - sash_top - sash_bottom)),
        origin=Origin(
            xyz=(
                0.5 * (sash_w - 2.0 * sash_side - 0.014),
                -0.0215,
                0.5 * (sash_bottom + sash_h - sash_top),
            )
        ),
        material=gasket,
        name="right_interior_glazing_gasket",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_side, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.0215, sash_bottom + 0.007)),
        material=gasket,
        name="bottom_interior_glazing_gasket",
    )
    sash.visual(
        Box((sash_w - 2.0 * sash_side, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.0215, sash_h - sash_top - 0.007)),
        material=gasket,
        name="top_interior_glazing_gasket",
    )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = x_sign * hinge_x
        sash.visual(
            Cylinder(radius=hinge_barrel_r, length=hinge_barrel_len),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=steel,
            name=f"{side}_hinge_barrel",
        )
        sash.visual(
            Box((hinge_barrel_len, 0.014, 0.032)),
            origin=Origin(xyz=(x_pos, 0.0, 0.020)),
            material=aluminum,
            name=f"{side}_hinge_arm",
        )

    latch_handle = model.part("latch_handle")
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.105)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.012, -0.050)),
    )
    latch_handle.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=handle_black,
        name="escutcheon",
    )
    latch_handle.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, -0.002), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    latch_handle.visual(
        Box((0.014, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, -0.014, -0.050)),
        material=handle_black,
        name="lever_stem",
    )
    latch_handle.visual(
        Box((0.050, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.016, -0.092)),
        material=handle_black,
        name="lever_grip",
    )

    model.articulation(
        "sash_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=sash,
        origin=Origin(xyz=(0.0, 0.0, sash_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=0.95),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch_handle,
        origin=Origin(xyz=(0.0, -sash_d * 0.5, 0.564)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    latch_handle = object_model.get_part("latch_handle")
    sash_hinge = object_model.get_articulation("sash_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

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
        "sash_hinge_axis_is_horizontal",
        tuple(float(v) for v in sash_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected sash hinge axis (1,0,0), got {sash_hinge.axis}",
    )
    ctx.check(
        "sash_hinge_limits_match_hopper_motion",
        sash_hinge.motion_limits is not None
        and sash_hinge.motion_limits.lower == 0.0
        and sash_hinge.motion_limits.upper is not None
        and 0.75 <= sash_hinge.motion_limits.upper <= 1.10,
        details=f"unexpected sash hinge limits: {sash_hinge.motion_limits}",
    )
    ctx.check(
        "latch_axis_runs_through_handle_pivot",
        tuple(float(v) for v in latch_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"expected latch axis (0,1,0), got {latch_pivot.axis}",
    )

    ctx.expect_contact(
        sash,
        outer_frame,
        elem_a="left_hinge_barrel",
        elem_b="left_hinge_base",
        contact_tol=0.001,
        name="left_lower_hinge_captures_sash",
    )
    ctx.expect_contact(
        sash,
        outer_frame,
        elem_a="right_hinge_barrel",
        elem_b="right_hinge_base",
        contact_tol=0.001,
        name="right_lower_hinge_captures_sash",
    )
    ctx.expect_contact(
        sash,
        outer_frame,
        elem_a="left_stile",
        elem_b="left_stop",
        contact_tol=0.001,
        name="sash_seats_against_frame_stop",
    )
    ctx.expect_within(
        sash,
        outer_frame,
        axes="xz",
        margin=0.0,
        name="sash_stays_within_outer_frame_envelope",
    )
    ctx.expect_contact(
        latch_handle,
        sash,
        elem_a="escutcheon",
        elem_b="top_rail",
        contact_tol=0.001,
        name="handle_mounts_to_top_rail",
    )

    with ctx.pose({sash_hinge: 0.85}):
        ctx.expect_gap(
            outer_frame,
            sash,
            axis="y",
            positive_elem="top_stop",
            negative_elem="top_rail",
            min_gap=0.10,
            name="hopper_sash_tips_inward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
