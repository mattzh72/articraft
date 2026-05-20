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
    model = ArticulatedObject(name="counterbalanced_studio_easel")

    oak = model.material("oak", rgba=(0.50, 0.35, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.10, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.30, 0.33, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 1.82)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
    )

    # Wide floor frame.
    frame.visual(
        Box((0.08, 0.78, 0.055)),
        origin=Origin(xyz=(-0.29, 0.0, 0.0275)),
        material=oak,
        name="left_base_runner",
    )
    frame.visual(
        Box((0.08, 0.78, 0.055)),
        origin=Origin(xyz=(0.29, 0.0, 0.0275)),
        material=oak,
        name="right_base_runner",
    )
    frame.visual(
        Box((0.66, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, 0.31, 0.0275)),
        material=oak,
        name="front_crossbar",
    )
    frame.visual(
        Box((0.66, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, -0.31, 0.0275)),
        material=oak,
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.12, 0.56, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=oak,
        name="center_floor_spreader",
    )

    # Tall uprights and top bridge.
    frame.visual(
        Box((0.075, 0.085, 1.74)),
        origin=Origin(xyz=(-0.23, -0.25, 0.925)),
        material=oak,
        name="left_upright",
    )
    frame.visual(
        Box((0.075, 0.085, 1.74)),
        origin=Origin(xyz=(0.23, -0.25, 0.925)),
        material=oak,
        name="right_upright",
    )
    frame.visual(
        Box((0.54, 0.10, 0.07)),
        origin=Origin(xyz=(0.0, -0.25, 1.765)),
        material=oak,
        name="top_crossbar",
    )

    # Front-mounted guide rails that the cradle rides on.
    frame.visual(
        Cylinder(radius=0.011, length=1.36),
        origin=Origin(xyz=(-0.23, -0.202, 0.92)),
        material=dark_steel,
        name="left_guide_rail",
    )
    frame.visual(
        Cylinder(radius=0.011, length=1.36),
        origin=Origin(xyz=(0.23, -0.202, 0.92)),
        material=dark_steel,
        name="right_guide_rail",
    )

    # A visible rear counterweight pack to make the easel read as balanced.
    frame.visual(
        Box((0.18, 0.055, 0.05)),
        origin=Origin(xyz=(0.0, -0.305, 1.75)),
        material=graphite,
        name="counterweight_head",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.48),
        origin=Origin(xyz=(0.0, -0.305, 1.515)),
        material=black,
        name="counterweight_cable",
    )
    frame.visual(
        Box((0.10, 0.08, 0.21)),
        origin=Origin(xyz=(0.0, -0.305, 1.17)),
        material=graphite,
        name="counterweight_block",
    )

    cradle = model.part("canvas_cradle")
    cradle.inertial = Inertial.from_geometry(
        Box((0.60, 0.16, 0.88)),
        mass=5.5,
        origin=Origin(xyz=(0.0, -0.11, 0.44)),
    )

    cradle.visual(
        Box((0.075, 0.060, 0.22)),
        origin=Origin(xyz=(-0.23, -0.161, 0.08)),
        material=dark_steel,
        name="left_slider_shoe",
    )
    cradle.visual(
        Box((0.075, 0.060, 0.22)),
        origin=Origin(xyz=(0.23, -0.161, 0.08)),
        material=dark_steel,
        name="right_slider_shoe",
    )
    cradle.visual(
        Box((0.56, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, -0.158, 0.08)),
        material=dark_steel,
        name="lower_crossbar",
    )
    cradle.visual(
        Box((0.05, 0.045, 0.70)),
        origin=Origin(xyz=(-0.195, -0.152, 0.40)),
        material=dark_steel,
        name="left_side_arm",
    )
    cradle.visual(
        Box((0.05, 0.045, 0.70)),
        origin=Origin(xyz=(0.195, -0.152, 0.40)),
        material=dark_steel,
        name="right_side_arm",
    )
    cradle.visual(
        Box((0.06, 0.03, 0.66)),
        origin=Origin(xyz=(0.0, -0.136, 0.39)),
        material=dark_steel,
        name="center_backrest",
    )
    cradle.visual(
        Box((0.45, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.150, 0.745)),
        material=dark_steel,
        name="upper_crossbar",
    )
    cradle.visual(
        Box((0.12, 0.035, 0.11)),
        origin=Origin(xyz=(0.0, -0.132, 0.82)),
        material=graphite,
        name="top_clamp_pad",
    )
    cradle.visual(
        Box((0.58, 0.095, 0.032)),
        origin=Origin(xyz=(0.0, -0.089, 0.027)),
        material=oak,
        name="lower_tray",
    )
    cradle.visual(
        Box((0.56, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.042, 0.0475)),
        material=oak,
        name="tray_lip",
    )
    cradle.visual(
        Box((0.05, 0.065, 0.045)),
        origin=Origin(xyz=(-0.15, -0.120, 0.05)),
        material=dark_steel,
        name="left_tray_bracket",
    )
    cradle.visual(
        Box((0.05, 0.065, 0.045)),
        origin=Origin(xyz=(0.15, -0.120, 0.05)),
        material=dark_steel,
        name="right_tray_bracket",
    )

    model.articulation(
        "frame_to_canvas_cradle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.46,
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
    frame = object_model.get_part("frame")
    cradle = object_model.get_part("canvas_cradle")
    slide = object_model.get_articulation("frame_to_canvas_cradle")

    left_rail = frame.get_visual("left_guide_rail")
    right_rail = frame.get_visual("right_guide_rail")
    front_crossbar = frame.get_visual("front_crossbar")
    top_crossbar = frame.get_visual("top_crossbar")

    left_shoe = cradle.get_visual("left_slider_shoe")
    right_shoe = cradle.get_visual("right_slider_shoe")
    lower_tray = cradle.get_visual("lower_tray")
    top_clamp = cradle.get_visual("top_clamp_pad")

    ctx.check(
        "cradle articulation is vertical prismatic motion",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            cradle,
            frame,
            elem_a=left_shoe,
            elem_b=left_rail,
            contact_tol=0.0015,
            name="left slider shoe bears on left guide rail",
        )
        ctx.expect_contact(
            cradle,
            frame,
            elem_a=right_shoe,
            elem_b=right_rail,
            contact_tol=0.0015,
            name="right slider shoe bears on right guide rail",
        )
        ctx.expect_gap(
            cradle,
            frame,
            axis="z",
            positive_elem=lower_tray,
            negative_elem=front_crossbar,
            min_gap=0.22,
            max_gap=0.34,
            name="lower tray clears the floor frame at the low setting",
        )

    rest_pos = ctx.part_world_position(cradle)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    with ctx.pose({slide: upper if upper is not None else 0.46}):
        ctx.expect_contact(
            cradle,
            frame,
            elem_a=left_shoe,
            elem_b=left_rail,
            contact_tol=0.0015,
            name="left slider shoe stays engaged at full height",
        )
        ctx.expect_contact(
            cradle,
            frame,
            elem_a=right_shoe,
            elem_b=right_rail,
            contact_tol=0.0015,
            name="right slider shoe stays engaged at full height",
        )
        ctx.expect_gap(
            frame,
            cradle,
            axis="z",
            positive_elem=top_crossbar,
            negative_elem=top_clamp,
            min_gap=0.06,
            max_gap=0.20,
            name="top clamp remains below the top bridge at full extension",
        )
        raised_pos = ctx.part_world_position(cradle)

    ctx.check(
        "canvas cradle raises upward on the guide rails",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.40,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
