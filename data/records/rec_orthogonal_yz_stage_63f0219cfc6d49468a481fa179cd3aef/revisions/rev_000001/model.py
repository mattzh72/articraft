from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BEAM_WIDTH = 0.22
BEAM_LENGTH = 0.82
BEAM_HEIGHT = 0.10

RAIL_WIDTH = 0.032
RAIL_LENGTH = 0.70
RAIL_HEIGHT = 0.03
RAIL_X = 0.075

CROSSHEAD_TRAVEL = 0.24
RAM_TRAVEL = 0.18


def _add_box(
    part,
    size,
    xyz,
    material,
    *,
    name: str | None = None,
    rpy=(0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz,
    material,
    name: str | None = None,
    rpy=(0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_beam_yz_axis")

    model.material("frame_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("dark_housing", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("slide_silver", rgba=(0.83, 0.85, 0.88, 1.0))
    model.material("rail_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    model.material("tool_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    model.material("accent_blue", rgba=(0.18, 0.37, 0.73, 1.0))

    top_support = model.part("top_support")
    _add_box(
        top_support,
        (BEAM_WIDTH, BEAM_LENGTH, BEAM_HEIGHT),
        (0.0, 0.0, 0.0),
        "frame_gray",
        name="beam_body",
    )
    _add_box(
        top_support,
        (0.30, 0.06, 0.18),
        (0.0, -(BEAM_LENGTH / 2.0 - 0.03), -0.005),
        "frame_gray",
        name="left_end_mount",
    )
    _add_box(
        top_support,
        (0.30, 0.06, 0.18),
        (0.0, BEAM_LENGTH / 2.0 - 0.03, -0.005),
        "frame_gray",
        name="right_end_mount",
    )
    _add_box(
        top_support,
        (0.09, 0.14, 0.025),
        (0.0, -(BEAM_LENGTH / 2.0 - 0.11), 0.0625),
        "frame_gray",
        name="left_top_pad",
    )
    _add_box(
        top_support,
        (0.09, 0.14, 0.025),
        (0.0, BEAM_LENGTH / 2.0 - 0.11, 0.0625),
        "frame_gray",
        name="right_top_pad",
    )
    _add_box(
        top_support,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT),
        (-RAIL_X, 0.0, -(BEAM_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        "rail_steel",
        name="left_rail",
    )
    _add_box(
        top_support,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT),
        (RAIL_X, 0.0, -(BEAM_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        "rail_steel",
        name="right_rail",
    )
    _add_box(
        top_support,
        (0.08, 0.28, 0.018),
        (0.0, 0.0, 0.059),
        "accent_blue",
        name="nameplate",
    )

    crosshead = model.part("crosshead")
    _add_box(
        crosshead,
        (0.208, 0.13, 0.09),
        (0.0, 0.0, -0.045),
        "dark_housing",
        name="upper_carriage",
    )
    _add_box(
        crosshead,
        (0.020, 0.15, 0.12),
        (-0.101, 0.0, -0.145),
        "dark_housing",
        name="left_hanger_cheek",
    )
    _add_box(
        crosshead,
        (0.020, 0.15, 0.12),
        (0.101, 0.0, -0.145),
        "dark_housing",
        name="right_hanger_cheek",
    )
    _add_box(
        crosshead,
        (0.020, 0.09, 0.26),
        (-0.0575, 0.0, -0.30),
        "slide_silver",
        name="left_ram_guide",
    )
    _add_box(
        crosshead,
        (0.020, 0.09, 0.26),
        (0.0575, 0.0, -0.30),
        "slide_silver",
        name="right_ram_guide",
    )
    _add_box(
        crosshead,
        (0.13, 0.015, 0.26),
        (0.0, -0.0525, -0.30),
        "dark_housing",
        name="rear_bridge",
    )
    _add_box(
        crosshead,
        (0.13, 0.015, 0.26),
        (0.0, 0.0525, -0.30),
        "dark_housing",
        name="front_bridge",
    )
    _add_box(
        crosshead,
        (0.12, 0.09, 0.03),
        (0.0, 0.0, -0.17),
        "dark_housing",
        name="ram_saddle",
    )
    _add_box(
        crosshead,
        (0.09, 0.08, 0.07),
        (0.0, 0.0, -0.125),
        "dark_housing",
        name="center_spine",
    )
    _add_box(
        crosshead,
        (0.03, 0.10, 0.045),
        (-0.065, 0.0, -0.4475),
        "dark_housing",
        name="left_lower_jaw",
    )
    _add_box(
        crosshead,
        (0.03, 0.10, 0.045),
        (0.065, 0.0, -0.4475),
        "dark_housing",
        name="right_lower_jaw",
    )

    ram = model.part("ram")
    _add_box(
        ram,
        (0.09, 0.08, 0.20),
        (0.0, 0.0, -0.10),
        "slide_silver",
        name="guide_block",
    )
    _add_box(
        ram,
        (0.075, 0.065, 0.24),
        (0.0, 0.0, -0.32),
        "slide_silver",
        name="quill",
    )
    _add_box(
        ram,
        (0.11, 0.085, 0.04),
        (0.0, 0.0, -0.46),
        "dark_housing",
        name="tool_plate",
    )
    _add_cylinder(
        ram,
        radius=0.018,
        length=0.07,
        xyz=(0.0, 0.0, -0.515),
        material="tool_steel",
        name="tool_stub",
    )

    model.articulation(
        "support_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=crosshead,
        origin=Origin(xyz=(0.0, 0.0, -(BEAM_HEIGHT / 2.0 + RAIL_HEIGHT))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.8,
            lower=-CROSSHEAD_TRAVEL,
            upper=CROSSHEAD_TRAVEL,
        ),
    )
    model.articulation(
        "crosshead_to_ram",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.45,
            lower=0.0,
            upper=RAM_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    crosshead = object_model.get_part("crosshead")
    ram = object_model.get_part("ram")
    y_axis = object_model.get_articulation("support_to_crosshead")
    z_axis = object_model.get_articulation("crosshead_to_ram")

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
        "parts_present",
        top_support is not None and crosshead is not None and ram is not None,
        "Expected top_support, crosshead, and ram parts.",
    )
    ctx.check(
        "prismatic_axis_types",
        y_axis.articulation_type == ArticulationType.PRISMATIC
        and z_axis.articulation_type == ArticulationType.PRISMATIC,
        "Both mechanisms should be authored as prismatic joints.",
    )
    ctx.check(
        "crosshead_axis_is_y",
        tuple(round(v, 6) for v in y_axis.axis) == (0.0, 1.0, 0.0),
        f"Expected Y-axis slide, got {y_axis.axis}.",
    )
    ctx.check(
        "ram_axis_is_negative_z",
        tuple(round(v, 6) for v in z_axis.axis) == (0.0, 0.0, -1.0),
        f"Expected downward Z-axis slide, got {z_axis.axis}.",
    )

    ctx.expect_contact(
        crosshead,
        top_support,
        contact_tol=5e-5,
        name="crosshead_supported_by_underbeam",
    )
    ctx.expect_contact(
        ram,
        crosshead,
        contact_tol=5e-5,
        name="ram_guided_in_crosshead",
    )

    with ctx.pose({y_axis: y_axis.motion_limits.upper}):
        ctx.expect_within(
            crosshead,
            top_support,
            axes="y",
            margin=0.0,
            name="crosshead_stays_within_beam_span_upper",
        )
    with ctx.pose({y_axis: y_axis.motion_limits.lower}):
        ctx.expect_within(
            crosshead,
            top_support,
            axes="y",
            margin=0.0,
            name="crosshead_stays_within_beam_span_lower",
        )

    crosshead_home = ctx.part_world_position(crosshead)
    with ctx.pose({y_axis: y_axis.motion_limits.upper}):
        crosshead_shifted = ctx.part_world_position(crosshead)
    ctx.check(
        "crosshead_positive_q_moves_along_positive_y",
        crosshead_home is not None
        and crosshead_shifted is not None
        and crosshead_shifted[1] > crosshead_home[1] + 0.15
        and abs(crosshead_shifted[0] - crosshead_home[0]) < 1e-5
        and abs(crosshead_shifted[2] - crosshead_home[2]) < 1e-5,
        f"Crosshead home={crosshead_home}, shifted={crosshead_shifted}.",
    )

    ram_home = ctx.part_world_position(ram)
    with ctx.pose({z_axis: z_axis.motion_limits.upper}):
        ram_extended = ctx.part_world_position(ram)
    ctx.check(
        "ram_positive_q_moves_downward",
        ram_home is not None
        and ram_extended is not None
        and ram_extended[2] < ram_home[2] - 0.12
        and abs(ram_extended[0] - ram_home[0]) < 1e-5
        and abs(ram_extended[1] - ram_home[1]) < 1e-5,
        f"Ram home={ram_home}, extended={ram_extended}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
