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
    model = ArticulatedObject(name="pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.10, 0.19, 0.34, 1.0))
    liner_black = model.material("liner_black", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.50, 0.54, 1.0))

    panel_width = 1.52
    panel_height = 0.52
    stub_width = 0.09
    overall_width = panel_width + 2.0 * stub_width
    hinge_z = 0.08
    hinge_x = panel_width * 0.5 - 0.08

    bed_stub_frame = model.part("bed_stub_frame")
    bed_stub_frame.visual(
        Box((overall_width, 0.16, hinge_z)),
        origin=Origin(xyz=(0.0, 0.115, hinge_z * 0.5)),
        material=liner_black,
        name="lower_sill",
    )
    bed_stub_frame.visual(
        Box((stub_width, 0.23, 0.58)),
        origin=Origin(
            xyz=(-(panel_width + stub_width) * 0.5, 0.145, hinge_z + 0.29)
        ),
        material=body_paint,
        name="left_stub",
    )
    bed_stub_frame.visual(
        Box((stub_width, 0.23, 0.58)),
        origin=Origin(
            xyz=((panel_width + stub_width) * 0.5, 0.145, hinge_z + 0.29)
        ),
        material=body_paint,
        name="right_stub",
    )
    bed_stub_frame.visual(
        Box((0.06, 0.05, 0.08)),
        origin=Origin(xyz=(-(hinge_x + 0.065), 0.035, 0.07)),
        material=steel,
        name="left_hinge_support",
    )
    bed_stub_frame.visual(
        Box((0.06, 0.05, 0.08)),
        origin=Origin(xyz=(hinge_x + 0.065, 0.035, 0.07)),
        material=steel,
        name="right_hinge_support",
    )
    bed_stub_frame.visual(
        Cylinder(radius=0.01, length=0.11),
        origin=Origin(
            xyz=(-hinge_x, 0.0, hinge_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_hinge_pin",
    )
    bed_stub_frame.visual(
        Cylinder(radius=0.01, length=0.11),
        origin=Origin(
            xyz=(hinge_x, 0.0, hinge_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_hinge_pin",
    )
    bed_stub_frame.inertial = Inertial.from_geometry(
        Box((overall_width, 0.26, 0.66)),
        mass=75.0,
        origin=Origin(xyz=(0.0, 0.13, 0.33)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((panel_width, 0.01, panel_height)),
        origin=Origin(xyz=(0.0, -0.023, panel_height * 0.5)),
        material=body_paint,
        name="outer_skin",
    )
    tailgate.visual(
        Box((panel_width, 0.02, 0.014)),
        origin=Origin(xyz=(0.0, -0.01, panel_height - 0.007)),
        material=body_paint,
        name="top_hem",
    )
    tailgate.visual(
        Box((panel_width - 0.08, 0.038, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, panel_height - 0.042)),
        material=liner_black,
        name="upper_inner_rail",
    )
    tailgate.visual(
        Box((panel_width - 0.12, 0.012, 0.06)),
        origin=Origin(xyz=(0.0, -0.016, 0.09)),
        material=liner_black,
        name="lower_outer_beam",
    )
    tailgate.visual(
        Box((0.045, 0.042, panel_height - 0.12)),
        origin=Origin(
            xyz=(-(panel_width * 0.5) + 0.0225, -0.002, (panel_height + 0.10) * 0.5)
        ),
        material=liner_black,
        name="left_inner_rail",
    )
    tailgate.visual(
        Box((0.045, 0.042, panel_height - 0.12)),
        origin=Origin(
            xyz=((panel_width * 0.5) - 0.0225, -0.002, (panel_height + 0.10) * 0.5)
        ),
        material=liner_black,
        name="right_inner_rail",
    )
    tailgate.visual(
        Box((panel_width - 0.16, 0.004, panel_height - 0.18)),
        origin=Origin(xyz=(0.0, 0.021, 0.29)),
        material=liner_black,
        name="inner_panel",
    )

    band_width = 1.10
    opening_width = 0.36
    band_height = 0.118
    opening_height = 0.058
    band_y = -0.03
    band_z = 0.438
    side_band_width = (band_width - opening_width) * 0.5
    strip_height = (band_height - opening_height) * 0.5
    tailgate.visual(
        Box((side_band_width, 0.004, band_height)),
        origin=Origin(
            xyz=(-(opening_width + side_band_width) * 0.5, band_y, band_z)
        ),
        material=trim_black,
        name="band_left",
    )
    tailgate.visual(
        Box((side_band_width, 0.004, band_height)),
        origin=Origin(
            xyz=((opening_width + side_band_width) * 0.5, band_y, band_z)
        ),
        material=trim_black,
        name="band_right",
    )
    tailgate.visual(
        Box((opening_width, 0.004, strip_height)),
        origin=Origin(
            xyz=(0.0, band_y, band_z + (opening_height + strip_height) * 0.5)
        ),
        material=trim_black,
        name="band_top",
    )
    tailgate.visual(
        Box((opening_width, 0.004, strip_height)),
        origin=Origin(
            xyz=(0.0, band_y, band_z - (opening_height + strip_height) * 0.5)
        ),
        material=trim_black,
        name="band_bottom",
    )

    tailgate.visual(
        Box((0.10, 0.014, 0.056)),
        origin=Origin(xyz=(-hinge_x, -0.018, 0.028)),
        material=steel,
        name="left_hinge_ear",
    )
    tailgate.visual(
        Box((0.10, 0.014, 0.056)),
        origin=Origin(xyz=(hinge_x, -0.018, 0.028)),
        material=steel,
        name="right_hinge_ear",
    )
    tailgate.visual(
        Cylinder(radius=0.018, length=0.07),
        origin=Origin(
            xyz=(-hinge_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.018, length=0.07),
        origin=Origin(
            xyz=(hinge_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.01, length=0.032),
        origin=Origin(
            xyz=(-0.115, -0.038, 0.452),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_black,
        name="left_handle_boss",
    )
    tailgate.visual(
        Cylinder(radius=0.01, length=0.032),
        origin=Origin(
            xyz=(0.115, -0.038, 0.452),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_black,
        name="right_handle_boss",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((panel_width, 0.055, panel_height)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(
            xyz=(-0.115, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_spindle",
    )
    latch_handle.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(
            xyz=(0.115, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_spindle",
    )
    latch_handle.visual(
        Box((0.014, 0.012, 0.040)),
        origin=Origin(xyz=(-0.086, 0.0, -0.020)),
        material=trim_black,
        name="left_handle_arm",
    )
    latch_handle.visual(
        Box((0.014, 0.012, 0.040)),
        origin=Origin(xyz=(0.086, 0.0, -0.020)),
        material=trim_black,
        name="right_handle_arm",
    )
    latch_handle.visual(
        Box((0.30, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.004, -0.042)),
        material=trim_black,
        name="handle_paddle",
    )
    latch_handle.visual(
        Box((0.22, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, -0.060)),
        material=trim_black,
        name="finger_lip",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.30, 0.04, 0.07)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_stub_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.0, -0.038, 0.452)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_stub_frame = object_model.get_part("bed_stub_frame")
    tailgate = object_model.get_part("tailgate")
    latch_handle = object_model.get_part("latch_handle")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    outer_skin = tailgate.get_visual("outer_skin")
    inner_panel = tailgate.get_visual("inner_panel")
    left_hinge_pin = bed_stub_frame.get_visual("left_hinge_pin")
    right_hinge_pin = bed_stub_frame.get_visual("right_hinge_pin")
    left_hinge_barrel = tailgate.get_visual("left_hinge_barrel")
    right_hinge_barrel = tailgate.get_visual("right_hinge_barrel")
    left_handle_boss = tailgate.get_visual("left_handle_boss")
    right_handle_boss = tailgate.get_visual("right_handle_boss")
    left_spindle = latch_handle.get_visual("left_spindle")
    right_spindle = latch_handle.get_visual("right_spindle")
    handle_paddle = latch_handle.get_visual("handle_paddle")

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
        bed_stub_frame,
        tailgate,
        elem_a=left_hinge_pin,
        elem_b=left_hinge_barrel,
        reason="The left hinge pin runs inside the tailgate barrel.",
    )
    ctx.allow_overlap(
        bed_stub_frame,
        tailgate,
        elem_a=right_hinge_pin,
        elem_b=right_hinge_barrel,
        reason="The right hinge pin runs inside the tailgate barrel.",
    )
    ctx.allow_overlap(
        tailgate,
        latch_handle,
        elem_a=left_handle_boss,
        elem_b=left_spindle,
        reason="The left handle spindle is seated in the tailgate pivot boss.",
    )
    ctx.allow_overlap(
        tailgate,
        latch_handle,
        elem_a=right_handle_boss,
        elem_b=right_spindle,
        reason="The right handle spindle is seated in the tailgate pivot boss.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "tailgate_hinge_axis_is_transverse",
        tuple(tailgate_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected x-axis hinge, got {tailgate_hinge.axis!r}.",
    )
    ctx.check(
        "handle_pivot_axis_is_transverse",
        tuple(handle_pivot.axis) == (1.0, 0.0, 0.0),
        f"Expected x-axis handle pivot, got {handle_pivot.axis!r}.",
    )

    tailgate_limits = tailgate_hinge.motion_limits
    handle_limits = handle_pivot.motion_limits
    ctx.check(
        "tailgate_hinge_range_realistic",
        tailgate_limits is not None
        and tailgate_limits.lower == 0.0
        and tailgate_limits.upper is not None
        and math.radians(85.0) <= tailgate_limits.upper <= math.radians(100.0),
        f"Unexpected tailgate range: {tailgate_limits!r}.",
    )
    ctx.check(
        "handle_pivot_range_realistic",
        handle_limits is not None
        and handle_limits.lower is not None
        and math.radians(-45.0) <= handle_limits.lower <= math.radians(-20.0)
        and handle_limits.upper == 0.0,
        f"Unexpected handle range: {handle_limits!r}.",
    )

    tailgate_aabb = ctx.part_world_aabb(tailgate)
    frame_aabb = ctx.part_world_aabb(bed_stub_frame)
    handle_aabb = ctx.part_world_aabb(latch_handle)
    if tailgate_aabb is not None:
        tailgate_width = tailgate_aabb[1][0] - tailgate_aabb[0][0]
        tailgate_height = tailgate_aabb[1][2] - tailgate_aabb[0][2]
        tailgate_depth = tailgate_aabb[1][1] - tailgate_aabb[0][1]
        ctx.check(
            "tailgate_panel_size_realistic",
            1.45 <= tailgate_width <= 1.58
            and 0.48 <= tailgate_height <= 0.56
            and 0.04 <= tailgate_depth <= 0.08,
            (
                f"Tailgate dims were {(tailgate_width, tailgate_depth, tailgate_height)!r}; "
                "expected a full-size pickup tailgate."
            ),
        )
    if frame_aabb is not None and tailgate_aabb is not None:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        tailgate_width = tailgate_aabb[1][0] - tailgate_aabb[0][0]
        ctx.check(
            "bed_stub_frame_reads_as_short_stubs",
            0.14 <= frame_width - tailgate_width <= 0.24,
            (
                f"Stub overhang was {frame_width - tailgate_width:.4f} m; "
                "expected short bedside stubs just outside the gate."
            ),
        )

    with ctx.pose({tailgate_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_contact(
            bed_stub_frame,
            tailgate,
            elem_a=left_hinge_pin,
            elem_b=left_hinge_barrel,
            name="left_hinge_contact_closed",
        )
        ctx.expect_contact(
            bed_stub_frame,
            tailgate,
            elem_a=right_hinge_pin,
            elem_b=right_hinge_barrel,
            name="right_hinge_contact_closed",
        )
        ctx.expect_contact(
            tailgate,
            latch_handle,
            elem_a=left_handle_boss,
            elem_b=left_spindle,
            name="left_handle_pivot_contact_closed",
        )
        ctx.expect_contact(
            tailgate,
            latch_handle,
            elem_a=right_handle_boss,
            elem_b=right_spindle,
            name="right_handle_pivot_contact_closed",
        )
        ctx.expect_gap(
            tailgate,
            latch_handle,
            axis="y",
            positive_elem=outer_skin,
            negative_elem=handle_paddle,
            min_gap=0.0,
            max_gap=0.012,
            name="handle_sits_flush_to_outer_skin",
        )
        ctx.expect_overlap(
            latch_handle,
            tailgate,
            axes="x",
            elem_a=handle_paddle,
            elem_b=outer_skin,
            min_overlap=0.28,
            name="handle_width_centered_on_tailgate",
        )
        ctx.expect_within(
            latch_handle,
            tailgate,
            axes="x",
            inner_elem=handle_paddle,
            outer_elem=outer_skin,
            margin=0.0,
            name="handle_stays_within_tailgate_width",
        )

    if tailgate_limits is not None and tailgate_limits.lower is not None and tailgate_limits.upper is not None:
        with ctx.pose({tailgate_hinge: tailgate_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tailgate_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tailgate_hinge_lower_no_floating")
        with ctx.pose({tailgate_hinge: tailgate_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tailgate_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tailgate_hinge_upper_no_floating")
            ctx.expect_contact(
                bed_stub_frame,
                tailgate,
                elem_a=left_hinge_pin,
                elem_b=left_hinge_barrel,
                name="left_hinge_contact_open",
            )
            ctx.expect_contact(
                bed_stub_frame,
                tailgate,
                elem_a=right_hinge_pin,
                elem_b=right_hinge_barrel,
                name="right_hinge_contact_open",
            )
            open_tailgate_aabb = ctx.part_world_aabb(tailgate)
            if tailgate_aabb is not None and open_tailgate_aabb is not None:
                ctx.check(
                    "tailgate_swings_downward",
                    open_tailgate_aabb[1][2] < tailgate_aabb[1][2] - 0.35
                    and open_tailgate_aabb[0][1] < tailgate_aabb[0][1] - 0.35,
                    (
                        f"Closed AABB {tailgate_aabb!r} and open AABB {open_tailgate_aabb!r} "
                        "did not show the panel rotating rearward and downward."
                    ),
                )

    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_pivot: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_pivot_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_pivot_lower_no_floating")
            ctx.expect_contact(
                tailgate,
                latch_handle,
                elem_a=left_handle_boss,
                elem_b=left_spindle,
                name="left_handle_pivot_contact_open",
            )
            ctx.expect_contact(
                tailgate,
                latch_handle,
                elem_a=right_handle_boss,
                elem_b=right_spindle,
                name="right_handle_pivot_contact_open",
            )
            open_handle_aabb = ctx.part_world_aabb(latch_handle)
            if handle_aabb is not None and open_handle_aabb is not None:
                ctx.check(
                    "handle_rotates_outward",
                    open_handle_aabb[0][1] < handle_aabb[0][1] - 0.02,
                    (
                        f"Closed handle AABB {handle_aabb!r} and open handle AABB "
                        f"{open_handle_aabb!r} did not show outward travel."
                    ),
                )
        with ctx.pose({handle_pivot: handle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_pivot_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_pivot_upper_no_floating")

    if tailgate_limits is not None and tailgate_limits.upper is not None and handle_limits is not None and handle_limits.lower is not None:
        with ctx.pose({tailgate_hinge: tailgate_limits.upper, handle_pivot: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="all_articulations_open_no_overlap")
            ctx.fail_if_isolated_parts(name="all_articulations_open_no_floating")
            ctx.expect_contact(
                tailgate,
                latch_handle,
                elem_a=left_handle_boss,
                elem_b=left_spindle,
                name="left_handle_contact_with_tailgate_open",
            )
            ctx.expect_contact(
                tailgate,
                latch_handle,
                elem_a=right_handle_boss,
                elem_b=right_spindle,
                name="right_handle_contact_with_tailgate_open",
            )
            ctx.expect_overlap(
                tailgate,
                bed_stub_frame,
                axes="x",
                elem_a=inner_panel,
                elem_b=bed_stub_frame.get_visual("lower_sill"),
                min_overlap=1.20,
                name="open_tailgate_still_spans_between_stubs",
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
