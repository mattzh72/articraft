from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    track_dark = model.material("track_dark", rgba=(0.45, 0.47, 0.50, 1.0))
    gasket = model.material("gasket", rgba=(0.12, 0.12, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.86, 0.95, 0.28))

    outer_w = 1.20
    outer_h = 1.10
    outer_d = 0.12
    jamb_w = 0.055
    rail_h = 0.060

    clear_w = outer_w - 2.0 * jamb_w
    clear_h = outer_h - 2.0 * rail_h

    guide_wall_t = 0.010
    guide_wall_h = 0.014
    front_track_y = -0.028
    rear_track_y = 0.028
    sash_d = 0.028

    sash_w = 0.528
    sash_h = 0.940
    sash_stile_w = 0.050
    sash_meeting_w = 0.060
    sash_rail_h = 0.050
    sash_closed_x = -0.273
    fixed_x = 0.273
    sash_travel = 0.510

    def box_visual(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
    ) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def build_rect_sash(
        part,
        *,
        prefix: str,
        meeting_on_right: bool,
        add_handle: bool,
    ) -> None:
        left_stile_w = sash_stile_w
        right_stile_w = sash_meeting_w if meeting_on_right else sash_stile_w
        if not meeting_on_right:
            left_stile_w = sash_meeting_w

        left_x = -sash_w / 2.0 + left_stile_w / 2.0
        right_x = sash_w / 2.0 - right_stile_w / 2.0
        top_z = sash_h / 2.0 - sash_rail_h / 2.0
        bottom_z = -sash_h / 2.0 + sash_rail_h / 2.0

        box_visual(
            part,
            f"{prefix}_left_stile",
            (left_stile_w, sash_d, sash_h),
            (left_x, 0.0, 0.0),
            aluminum,
        )
        box_visual(
            part,
            f"{prefix}_right_stile",
            (right_stile_w, sash_d, sash_h),
            (right_x, 0.0, 0.0),
            aluminum,
        )
        box_visual(
            part,
            f"{prefix}_head",
            (sash_w, sash_d, sash_rail_h),
            (0.0, 0.0, top_z),
            aluminum,
        )
        box_visual(
            part,
            f"{prefix}_sill",
            (sash_w, sash_d, sash_rail_h),
            (0.0, 0.0, bottom_z),
            aluminum,
        )

        glass_w = sash_w - left_stile_w - right_stile_w + 0.024
        glass_h = sash_h - 2.0 * sash_rail_h + 0.024
        box_visual(
            part,
            f"{prefix}_glass",
            (glass_w, 0.006, glass_h),
            ((right_stile_w - left_stile_w) / 2.0, 0.0, 0.0),
            glass,
        )

        box_visual(
            part,
            f"{prefix}_glazing_gasket",
            (glass_w + 0.010, 0.010, glass_h + 0.010),
            ((right_stile_w - left_stile_w) / 2.0, 0.0, 0.0),
            gasket,
        )

        for x in (-0.155, 0.155):
            box_visual(
                part,
                f"{prefix}_bottom_guide_{'left' if x < 0.0 else 'right'}",
                (0.070, 0.010, 0.014),
                (x, 0.0, -0.470),
                track_dark,
            )
            box_visual(
                part,
                f"{prefix}_top_guide_{'left' if x < 0.0 else 'right'}",
                (0.070, 0.010, 0.014),
                (x, 0.0, 0.470),
                track_dark,
            )

        if add_handle:
            box_visual(
                part,
                f"{prefix}_pull",
                (0.014, 0.008, 0.260),
                (sash_w / 2.0 - right_stile_w / 2.0, 0.010, 0.0),
                track_dark,
            )

    outer_frame = model.part("outer_frame")
    box_visual(
        outer_frame,
        "left_jamb",
        (jamb_w, outer_d, outer_h),
        (-outer_w / 2.0 + jamb_w / 2.0, 0.0, 0.0),
        aluminum,
    )
    box_visual(
        outer_frame,
        "right_jamb",
        (jamb_w, outer_d, outer_h),
        (outer_w / 2.0 - jamb_w / 2.0, 0.0, 0.0),
        aluminum,
    )
    box_visual(
        outer_frame,
        "head",
        (clear_w, outer_d, rail_h),
        (0.0, 0.0, outer_h / 2.0 - rail_h / 2.0),
        aluminum,
    )
    box_visual(
        outer_frame,
        "sill",
        (clear_w, outer_d, rail_h),
        (0.0, 0.0, -outer_h / 2.0 + rail_h / 2.0),
        aluminum,
    )

    for name, y in (
        ("front", -0.052),
        ("center", 0.0),
        ("rear", 0.052),
    ):
        box_visual(
            outer_frame,
            f"bottom_{name}_guide",
            (clear_w, guide_wall_t, guide_wall_h),
            (0.0, y, -clear_h / 2.0 + guide_wall_h / 2.0 - 0.001),
            track_dark,
        )
        box_visual(
            outer_frame,
            f"top_{name}_guide",
            (clear_w, guide_wall_t, guide_wall_h),
            (0.0, y, clear_h / 2.0 - guide_wall_h / 2.0 + 0.001),
            track_dark,
        )

    for name, y in (("front", front_track_y), ("rear", rear_track_y)):
        box_visual(
            outer_frame,
            f"bottom_{name}_runner",
            (clear_w, 0.022, 0.018),
            (0.0, y, -clear_h / 2.0 + 0.004),
            track_dark,
        )
        box_visual(
            outer_frame,
            f"top_{name}_runner",
            (clear_w, 0.022, 0.018),
            (0.0, y, clear_h / 2.0 - 0.004),
            track_dark,
        )

    outer_frame.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=22.0,
    )

    fixed_segment = model.part("fixed_segment")
    build_rect_sash(
        fixed_segment,
        prefix="fixed",
        meeting_on_right=False,
        add_handle=False,
    )
    fixed_segment.inertial = Inertial.from_geometry(
        Box((sash_w, sash_d, sash_h)),
        mass=9.0,
    )

    sliding_sash = model.part("sliding_sash")
    build_rect_sash(
        sliding_sash,
        prefix="sliding",
        meeting_on_right=True,
        add_handle=True,
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_w, sash_d, sash_h)),
        mass=10.0,
    )

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_segment,
        origin=Origin(xyz=(fixed_x, front_track_y, 0.0)),
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(sash_closed_x, rear_track_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.45,
            lower=0.0,
            upper=sash_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    outer_frame = object_model.get_part("outer_frame")
    fixed_segment = object_model.get_part("fixed_segment")
    sliding_sash = object_model.get_part("sliding_sash")
    slide_joint = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_gap(
        fixed_segment,
        sliding_sash,
        axis="x",
        min_gap=0.008,
        max_gap=0.030,
        name="closed meeting stiles nearly meet",
    )
    ctx.expect_within(
        fixed_segment,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="fixed segment stays inside the outer frame tracks",
    )
    ctx.expect_within(
        sliding_sash,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="closed sliding sash stays inside the outer frame tracks",
    )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="yz",
            margin=0.0,
            name="open sliding sash stays retained in the guide channels",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_segment,
            axes="xz",
            min_overlap=0.45,
            name="open sash overlaps the fixed panel footprint",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_segment,
            axis="y",
            min_gap=0.020,
            max_gap=0.040,
            name="open sash runs in a separate rear track",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash translates to the right when opened",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] > rest_pos[0] + 0.45,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
