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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 40,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos((index * math.tau) / segments),
            radius * math.sin((index * math.tau) / segments),
        )
        for index in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _ring_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 40,
    name: str,
):
    geometry = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_drill_guide_stand")

    coated_steel = model.material("coated_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    cast_alloy = model.material("cast_alloy", rgba=(0.32, 0.34, 0.36, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.79, 0.80, 0.82, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    signal_red = model.material("signal_red", rgba=(0.68, 0.12, 0.10, 1.0))

    base_plate = model.part("base_plate")
    base_plate_geometry = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.280, 0.220, 0.022),
        [
            _translate_profile(rounded_rect_profile(0.094, 0.036, 0.018), 0.0, 0.016),
            _translate_profile(rounded_rect_profile(0.050, 0.012, 0.006), -0.082, 0.040),
            _translate_profile(rounded_rect_profile(0.050, 0.012, 0.006), 0.082, 0.040),
        ],
        0.012,
    )
    base_plate_geometry.translate(0.0, 0.0, 0.006)
    base_plate.visual(
        mesh_from_geometry(base_plate_geometry, "drill_guide_base_plate"),
        material=coated_steel,
        name="plate",
    )
    base_plate.visual(
        Box((0.034, 0.032, 0.044)),
        origin=Origin(xyz=(-0.053, -0.084, 0.034)),
        material=cast_alloy,
        name="left_cheek",
    )
    base_plate.visual(
        Box((0.034, 0.032, 0.044)),
        origin=Origin(xyz=(0.053, -0.084, 0.034)),
        material=cast_alloy,
        name="right_cheek",
    )
    base_plate.visual(
        Box((0.126, 0.020, 0.028)),
        origin=Origin(xyz=(0.000, -0.104, 0.020)),
        material=cast_alloy,
        name="rear_bridge",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base_plate.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(
                    xyz=(0.100 * x_sign, 0.070 * y_sign, -0.003),
                ),
                material=black_plastic,
            )

    tilt_base = model.part("tilt_base")
    tilt_base.visual(
        Cylinder(radius=0.010, length=0.072),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_alloy,
        name="trunnion",
    )
    tilt_base.visual(
        Box((0.068, 0.052, 0.036)),
        origin=Origin(xyz=(0.000, 0.020, 0.018)),
        material=cast_alloy,
        name="mount_block",
    )
    tilt_base.visual(
        Box((0.036, 0.016, 0.048)),
        origin=Origin(xyz=(0.000, -0.018, 0.024)),
        material=cast_alloy,
        name="rear_brace",
    )
    tilt_base.visual(
        Box((0.052, 0.016, 0.030)),
        origin=Origin(xyz=(0.000, 0.040, 0.015)),
        material=cast_alloy,
        name="front_pad",
    )

    column_sleeve = model.part("column_sleeve")
    column_sleeve.visual(
        _ring_mesh(
            0.0175,
            0.0136,
            0.340,
            segments=44,
            name="drill_guide_column_sleeve",
        ),
        material=bright_steel,
        name="outer_tube",
    )
    column_sleeve.visual(
        _ring_mesh(
            0.025,
            0.0136,
            0.020,
            segments=44,
            name="drill_guide_column_lower_collar",
        ),
        material=cast_alloy,
        name="lower_collar",
    )
    column_sleeve.visual(
        _ring_mesh(
            0.022,
            0.0136,
            0.016,
            segments=44,
            name="drill_guide_column_top_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.324)),
        material=cast_alloy,
        name="top_cap",
    )
    column_sleeve.visual(
        Box((0.004, 0.010, 0.220)),
        origin=Origin(xyz=(0.0195, 0.0, 0.170)),
        material=cast_alloy,
        name="guide_key",
    )

    column_extension = model.part("column_extension")
    column_extension.visual(
        Cylinder(radius=0.0125, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=bright_steel,
        name="extension_rod",
    )
    column_extension.visual(
        Cylinder(radius=0.0128, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=bright_steel,
        name="guide_bushing",
    )
    column_extension.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=cast_alloy,
        name="stop_collar",
    )
    column_extension.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.293)),
        material=black_plastic,
        name="top_knob",
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        _ring_mesh(
            0.034,
            0.0186,
            0.055,
            segments=44,
            name="drill_guide_head_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=cast_alloy,
        name="collar_bushing",
    )
    clamp_head.visual(
        Box((0.050, 0.044, 0.046)),
        origin=Origin(xyz=(0.000, 0.042, 0.000)),
        material=cast_alloy,
        name="carriage_body",
    )
    clamp_head.visual(
        Box((0.024, 0.070, 0.024)),
        origin=Origin(xyz=(0.000, 0.075, -0.004)),
        material=cast_alloy,
        name="forward_arm",
    )
    clamp_head.visual(
        _ring_mesh(
            0.040,
            0.022,
            0.028,
            segments=44,
            name="drill_guide_clamp_ring",
        ),
        origin=Origin(xyz=(0.0, 0.108, -0.004)),
        material=cast_alloy,
        name="drill_clamp_ring",
    )
    clamp_head.visual(
        Box((0.018, 0.020, 0.022)),
        origin=Origin(xyz=(0.000, 0.144, 0.010)),
        material=cast_alloy,
        name="split_lug",
    )
    clamp_head.visual(
        Box((0.006, 0.014, 0.090)),
        origin=Origin(xyz=(0.0245, 0.000, -0.006)),
        material=black_plastic,
        name="guide_shoe",
    )
    clamp_head.visual(
        Cylinder(radius=0.005, length=0.052),
        origin=Origin(
            xyz=(0.0, 0.144, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=signal_red,
        name="clamp_screw",
    )
    clamp_head.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(-0.026, 0.144, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=signal_red,
        name="left_knob",
    )
    clamp_head.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(0.026, 0.144, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=signal_red,
        name="right_knob",
    )

    model.articulation(
        "base_tilt",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=tilt_base,
        origin=Origin(xyz=(0.0, -0.084, 0.034)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "tilt_to_column",
        ArticulationType.FIXED,
        parent=tilt_base,
        child=column_sleeve,
        origin=Origin(xyz=(0.0, 0.020, 0.036)),
    )
    model.articulation(
        "column_telescopic",
        ArticulationType.PRISMATIC,
        parent=column_sleeve,
        child=column_extension,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=column_sleeve,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.25,
            lower=0.0,
            upper=0.175,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base_plate = object_model.get_part("base_plate")
    tilt_base = object_model.get_part("tilt_base")
    column_sleeve = object_model.get_part("column_sleeve")
    column_extension = object_model.get_part("column_extension")
    clamp_head = object_model.get_part("clamp_head")

    base_tilt = object_model.get_articulation("base_tilt")
    tilt_to_column = object_model.get_articulation("tilt_to_column")
    column_telescopic = object_model.get_articulation("column_telescopic")
    head_slide = object_model.get_articulation("head_slide")

    mount_block = tilt_base.get_visual("mount_block")
    base_plate_surface = base_plate.get_visual("plate")
    lower_collar = column_sleeve.get_visual("lower_collar")
    top_cap = column_sleeve.get_visual("top_cap")
    stop_collar = column_extension.get_visual("stop_collar")
    head_collar = clamp_head.get_visual("collar_bushing")
    guide_key = column_sleeve.get_visual("guide_key")
    guide_shoe = clamp_head.get_visual("guide_shoe")

    ctx.check(
        "tilt_joint_axis",
        base_tilt.axis == (-1.0, 0.0, 0.0),
        details=f"Expected tilt axis (-1, 0, 0), got {base_tilt.axis}",
    )
    ctx.check(
        "telescoping_column_axis",
        column_telescopic.axis == (0.0, 0.0, 1.0),
        details=f"Expected column axis (0, 0, 1), got {column_telescopic.axis}",
    )
    ctx.check(
        "head_slide_axis",
        head_slide.axis == (0.0, 0.0, -1.0),
        details=f"Expected head slide axis (0, 0, -1), got {head_slide.axis}",
    )
    ctx.check(
        "fixed_column_mount",
        tilt_to_column.articulation_type == ArticulationType.FIXED,
        details=f"Expected fixed sleeve mount, got {tilt_to_column.articulation_type}",
    )

    ctx.expect_contact(base_plate, tilt_base, name="tilt_base_supported_on_base")
    ctx.expect_contact(
        column_sleeve,
        tilt_base,
        elem_a=lower_collar,
        elem_b=mount_block,
        name="column_sleeve_seated_in_tilt_block",
    )
    ctx.expect_contact(
        column_extension,
        column_sleeve,
        elem_a=stop_collar,
        elem_b=top_cap,
        name="telescoping_column_retracted_stop_contacts_cap",
    )
    ctx.expect_within(
        column_sleeve,
        clamp_head,
        axes="xy",
        inner_elem="outer_tube",
        outer_elem=head_collar,
        margin=0.0015,
        name="head_carriage_coaxial_with_column",
    )
    ctx.expect_contact(
        clamp_head,
        column_sleeve,
        elem_a=guide_shoe,
        elem_b=guide_key,
        name="head_carriage_guided_by_column_key",
    )
    ctx.expect_within(
        column_extension,
        column_sleeve,
        axes="xy",
        inner_elem="guide_bushing",
        outer_elem="outer_tube",
        margin=0.0015,
        name="telescoping_rod_guided_in_sleeve",
    )
    ctx.expect_overlap(
        clamp_head,
        base_plate,
        axes="xy",
        min_overlap=0.040,
        name="clamp_head_projects_over_base_plate",
    )

    extension_rest = ctx.part_world_position(column_extension)
    head_rest = ctx.part_world_position(clamp_head)
    assert extension_rest is not None
    assert head_rest is not None

    with ctx.pose({column_telescopic: 0.160}):
        extension_up = ctx.part_world_position(column_extension)
        assert extension_up is not None
        ctx.check(
            "column_extension_rises",
            extension_up[2] > extension_rest[2] + 0.150,
            details=f"Expected >0.150 m lift, got {extension_up[2] - extension_rest[2]:.4f} m",
        )
        ctx.expect_gap(
            column_extension,
            column_sleeve,
            axis="z",
            min_gap=0.150,
            positive_elem=stop_collar,
            negative_elem=top_cap,
            name="extended_column_collar_clears_top_cap",
        )

    with ctx.pose({head_slide: 0.170}):
        head_low = ctx.part_world_position(clamp_head)
        assert head_low is not None
        ctx.check(
            "head_slide_descends",
            head_low[2] < head_rest[2] - 0.160,
            details=f"Expected >0.160 m descent, got {head_rest[2] - head_low[2]:.4f} m",
        )
        ctx.expect_gap(
            clamp_head,
            base_plate,
            axis="z",
            min_gap=0.110,
            negative_elem=base_plate_surface,
            name="head_clear_of_base_in_low_position",
        )

    with ctx.pose({base_tilt: math.radians(35.0)}):
        head_tilted = ctx.part_world_position(clamp_head)
        assert head_tilted is not None
        ctx.check(
            "tilt_swings_head_forward",
            head_tilted[1] > head_rest[1] + 0.140,
            details=f"Expected forward swing >0.140 m, got {head_tilted[1] - head_rest[1]:.4f} m",
        )
        ctx.expect_contact(
            column_sleeve,
            tilt_base,
            elem_a=lower_collar,
            elem_b=mount_block,
            name="column_remains_seated_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
