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
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    back_plastic = model.material("back_plastic", rgba=(0.44, 0.45, 0.48, 1.0))
    cover_plastic = model.material("cover_plastic", rgba=(0.92, 0.92, 0.89, 1.0))
    dial_plastic = model.material("dial_plastic", rgba=(0.83, 0.84, 0.80, 1.0))
    marking = model.material("marking", rgba=(0.18, 0.19, 0.20, 1.0))
    retainer_metal = model.material("retainer_metal", rgba=(0.75, 0.77, 0.79, 1.0))

    backplate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.088, 0.088, 0.010, corner_segments=8),
            [
                _circle_profile(0.0040, center=(0.0, 0.0260)),
                _circle_profile(0.0040, center=(0.0, -0.0260)),
            ],
            height=0.003,
            center=True,
        ),
        "thermostat_backplate",
    )
    fascia_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.110, 0.110, 0.014, corner_segments=8),
            [_circle_profile(0.0290, segments=56)],
            height=0.003,
            center=True,
        ),
        "thermostat_fascia",
    )
    dial_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0095, 0.0005),
                (0.0225, 0.0005),
                (0.0305, 0.0035),
                (0.0380, 0.0100),
                (0.0365, 0.0140),
                (0.0320, 0.0155),
            ],
            [
                (0.0080, 0.0005),
                (0.0080, 0.0095),
                (0.0130, 0.0125),
                (0.0095, 0.0155),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial",
    )

    base_plate = model.part("base_plate")
    base_plate.visual(
        backplate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=back_plastic,
        name="backplate",
    )
    base_plate.visual(
        Box((0.003, 0.089, 0.020)),
        origin=Origin(xyz=(-0.0445, 0.0, 0.0130)),
        material=back_plastic,
        name="left_wall",
    )
    base_plate.visual(
        Box((0.003, 0.089, 0.020)),
        origin=Origin(xyz=(0.0445, 0.0, 0.0130)),
        material=back_plastic,
        name="right_wall",
    )
    base_plate.visual(
        Box((0.086, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0445, 0.0130)),
        material=back_plastic,
        name="top_wall",
    )
    base_plate.visual(
        Box((0.086, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.0445, 0.0130)),
        material=back_plastic,
        name="bottom_wall",
    )
    base_plate.visual(
        Cylinder(radius=0.0048, length=0.0310),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=back_plastic,
        name="central_post",
    )
    base_plate.visual(
        Cylinder(radius=0.0035, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0340)),
        material=back_plastic,
        name="post_tip",
    )
    base_plate.visual(
        Box((0.003, 0.014, 0.002)),
        origin=Origin(xyz=(-0.0465, 0.0, 0.0180)),
        material=back_plastic,
        name="left_snap",
    )
    base_plate.visual(
        Box((0.003, 0.014, 0.002)),
        origin=Origin(xyz=(0.0465, 0.0, 0.0180)),
        material=back_plastic,
        name="right_snap",
    )

    cover = model.part("cover")
    cover.visual(
        fascia_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0245)),
        material=cover_plastic,
        name="fascia",
    )
    cover.visual(
        Box((0.003, 0.099, 0.020)),
        origin=Origin(xyz=(-0.0480, 0.0, 0.0130)),
        material=cover_plastic,
        name="left_skirt",
    )
    cover.visual(
        Box((0.003, 0.099, 0.020)),
        origin=Origin(xyz=(0.0480, 0.0, 0.0130)),
        material=cover_plastic,
        name="right_skirt",
    )
    cover.visual(
        Box((0.093, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0480, 0.0130)),
        material=cover_plastic,
        name="top_skirt",
    )
    cover.visual(
        Box((0.093, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.0480, 0.0130)),
        material=cover_plastic,
        name="bottom_skirt",
    )
    cover.visual(
        Box((0.0025, 0.014, 0.002)),
        origin=Origin(xyz=(-0.04625, 0.0, 0.0200)),
        material=cover_plastic,
        name="left_tab",
    )
    cover.visual(
        Box((0.0025, 0.014, 0.002)),
        origin=Origin(xyz=(0.04625, 0.0, 0.0200)),
        material=cover_plastic,
        name="right_tab",
    )
    cover.visual(
        Box((0.004, 0.010, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0360, 0.02675)),
        material=marking,
        name="top_tick",
    )

    dial = model.part("dial")
    dial.visual(dial_mesh, material=dial_plastic, name="dial_shell")
    dial.visual(
        Box((0.004, 0.010, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0255, 0.01625)),
        material=marking,
        name="pointer_ridge",
    )

    retainer = model.part("retainer")
    retainer.visual(
        Cylinder(radius=0.0035, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=retainer_metal,
        name="stem",
    )
    retainer.visual(
        Cylinder(radius=0.0105, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00325)),
        material=retainer_metal,
        name="head",
    )

    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base_plate,
        child=cover,
        origin=Origin(),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "base_to_retainer",
        ArticulationType.FIXED,
        parent=base_plate,
        child=retainer,
        origin=Origin(xyz=(0.0, 0.0, 0.0370)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    cover = object_model.get_part("cover")
    dial = object_model.get_part("dial")
    retainer = object_model.get_part("retainer")
    dial_joint = object_model.get_articulation("base_to_dial")
    dial_limits = dial_joint.motion_limits

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

    ctx.expect_contact(
        cover,
        base_plate,
        elem_a="left_tab",
        elem_b="left_snap",
        name="left_snap_tab_engaged",
    )
    ctx.expect_contact(
        cover,
        base_plate,
        elem_a="right_tab",
        elem_b="right_snap",
        name="right_snap_tab_engaged",
    )
    ctx.expect_contact(
        retainer,
        base_plate,
        elem_a="stem",
        elem_b="post_tip",
        name="retainer_stem_clamps_onto_post_tip",
    )
    ctx.expect_contact(
        dial,
        retainer,
        elem_a="dial_shell",
        elem_b="head",
        name="retainer_captures_dial_front_lip",
    )
    ctx.expect_origin_distance(
        dial,
        base_plate,
        axes="xy",
        max_dist=1e-6,
        name="dial_axis_is_centered_on_body",
    )
    ctx.expect_origin_distance(
        retainer,
        dial,
        axes="xy",
        max_dist=1e-6,
        name="retainer_stays_centered_on_dial_axis",
    )
    ctx.expect_overlap(
        dial,
        cover,
        axes="xy",
        min_overlap=0.050,
        name="dial_stays_registered_over_body_face",
    )
    ctx.check(
        "dial_joint_axis_is_front_normal",
        dial_joint.axis == (0.0, 0.0, 1.0),
        f"axis={dial_joint.axis}",
    )
    ctx.check(
        "dial_joint_limits_span_realistic_rotation",
        dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None
        and dial_limits.lower < -2.0
        and dial_limits.upper > 2.0,
        f"limits={None if dial_limits is None else (dial_limits.lower, dial_limits.upper)}",
    )

    with ctx.pose({dial_joint: 1.8}):
        ctx.expect_contact(
            dial,
            retainer,
            elem_a="dial_shell",
            elem_b="head",
            name="retainer_capture_persists_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
