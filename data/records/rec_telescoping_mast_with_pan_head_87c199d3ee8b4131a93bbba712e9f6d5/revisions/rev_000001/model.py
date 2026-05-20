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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SUPPORT_SADDLE_TOP = 0.056

OUTER_STAGE_HOME_Z = 0.060
OUTER_STAGE_TRAVEL = 0.260

INNER_STAGE_HOME_Z = 0.050
INNER_STAGE_TRAVEL = 0.200

FACEPLATE_MOUNT_Z = 0.450


def _add_box(
    part,
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material: str | None = None,
    name: str | None = None,
):
    return part.visual(
        Box((size_x, size_y, size_z)),
        origin=Origin(xyz=(x, y, z + size_z / 2.0)),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material: str | None = None,
    name: str | None = None,
):
    return part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=(x, y, z + height / 2.0), rpy=rpy),
        material=material,
        name=name,
    )


def _add_rect_tube(
    part,
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    *,
    z: float = 0.0,
    material: str | None = None,
    name_prefix: str,
) -> None:
    side_x = (outer_x - wall) / 2.0
    side_y = (outer_y - wall) / 2.0
    span_x = outer_x - 2.0 * wall

    _add_box(part, wall, outer_y, height, x=-side_x, z=z, material=material, name=f"{name_prefix}_left")
    _add_box(part, wall, outer_y, height, x=side_x, z=z, material=material, name=f"{name_prefix}_right")
    _add_box(part, span_x, wall, height, y=-side_y, z=z, material=material, name=f"{name_prefix}_rear")
    _add_box(part, span_x, wall, height, y=side_y, z=z, material=material, name=f"{name_prefix}_front")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_mast")

    model.material("powder_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("mast_gray", rgba=(0.48, 0.51, 0.55, 1.0))
    model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("machined_dark", rgba=(0.17, 0.18, 0.20, 1.0))

    support_frame = model.part("support_frame")
    _add_box(support_frame, 0.280, 0.180, 0.016, material="powder_gray", name="base_plate")
    _add_box(
        support_frame,
        0.110,
        0.012,
        0.250,
        y=-0.040,
        z=0.016,
        material="powder_gray",
        name="rear_spine",
    )
    _add_box(
        support_frame,
        0.012,
        0.082,
        0.080,
        x=-0.041,
        y=-0.009,
        z=0.016,
        material="powder_gray",
        name="left_fork",
    )
    _add_box(
        support_frame,
        0.012,
        0.082,
        0.080,
        x=0.041,
        y=-0.009,
        z=0.016,
        material="powder_gray",
        name="right_fork",
    )
    _add_box(
        support_frame,
        0.070,
        0.064,
        0.008,
        z=0.048,
        material="powder_gray",
        name="saddle",
    )

    outer_stage = model.part("outer_stage")
    _add_box(outer_stage, 0.068, 0.060, 0.040, material="mast_gray", name="outer_shoe")
    _add_rect_tube(
        outer_stage,
        0.084,
        0.064,
        0.500,
        0.005,
        z=0.040,
        material="mast_gray",
        name_prefix="outer_tube",
    )

    middle_stage = model.part("middle_stage")
    _add_rect_tube(
        middle_stage,
        0.072,
        0.052,
        0.480,
        0.004,
        material="aluminum",
        name_prefix="middle_tube",
    )
    _add_box(
        middle_stage,
        0.001,
        0.028,
        0.140,
        x=-0.0365,
        z=0.030,
        material="aluminum",
        name="middle_pad_left",
    )
    _add_box(
        middle_stage,
        0.001,
        0.028,
        0.140,
        x=0.0365,
        z=0.030,
        material="aluminum",
        name="middle_pad_right",
    )
    _add_box(
        middle_stage,
        0.022,
        0.001,
        0.140,
        y=-0.0265,
        z=0.030,
        material="aluminum",
        name="middle_pad_rear",
    )
    _add_box(
        middle_stage,
        0.022,
        0.001,
        0.140,
        y=0.0265,
        z=0.030,
        material="aluminum",
        name="middle_pad_front",
    )

    inner_stage = model.part("inner_stage")
    _add_rect_tube(
        inner_stage,
        0.062,
        0.042,
        0.400,
        0.004,
        material="aluminum",
        name_prefix="inner_tube",
    )
    _add_box(inner_stage, 0.062, 0.042, 0.010, z=0.400, material="aluminum", name="inner_cap")
    _add_cylinder(inner_stage, 0.009, 0.040, z=0.410, material="aluminum", name="faceplate_post")
    _add_box(
        inner_stage,
        0.001,
        0.024,
        0.120,
        x=-0.0315,
        z=0.030,
        material="aluminum",
        name="inner_pad_left",
    )
    _add_box(
        inner_stage,
        0.001,
        0.024,
        0.120,
        x=0.0315,
        z=0.030,
        material="aluminum",
        name="inner_pad_right",
    )
    _add_box(
        inner_stage,
        0.018,
        0.001,
        0.120,
        y=-0.0215,
        z=0.030,
        material="aluminum",
        name="inner_pad_rear",
    )
    _add_box(
        inner_stage,
        0.018,
        0.001,
        0.120,
        y=0.0215,
        z=0.030,
        material="aluminum",
        name="inner_pad_front",
    )

    faceplate = model.part("faceplate")
    _add_cylinder(faceplate, 0.040, 0.006, material="machined_dark", name="faceplate_disk")
    _add_cylinder(faceplate, 0.015, 0.010, z=0.006, material="machined_dark", name="faceplate_hub")
    _add_cylinder(faceplate, 0.004, 0.006, x=0.025, z=0.006, material="machined_dark", name="bolt_a")
    _add_cylinder(faceplate, 0.004, 0.006, x=-0.025, z=0.006, material="machined_dark", name="bolt_b")
    _add_cylinder(faceplate, 0.004, 0.006, y=0.025, z=0.006, material="machined_dark", name="bolt_c")
    _add_cylinder(faceplate, 0.004, 0.006, y=-0.025, z=0.006, material="machined_dark", name="bolt_d")

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support_frame,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_SADDLE_TOP)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_STAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=0.0,
            upper=OUTER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.30,
            lower=0.0,
            upper=INNER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=inner_stage,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, FACEPLATE_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-pi,
            upper=pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    outer_stage = object_model.get_part("outer_stage")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    faceplate = object_model.get_part("faceplate")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_faceplate = object_model.get_articulation("inner_to_faceplate")

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
        "mast_prismatic_axes_are_vertical",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and outer_to_middle.axis == (0.0, 0.0, 1.0)
        and middle_to_inner.axis == (0.0, 0.0, 1.0),
        "The nested mast stages must translate vertically along +Z.",
    )
    ctx.check(
        "faceplate_revolves_about_mast_axis",
        inner_to_faceplate.articulation_type == ArticulationType.REVOLUTE
        and inner_to_faceplate.axis == (0.0, 0.0, 1.0),
        "The top faceplate should rotate about the mast centerline.",
    )

    ctx.expect_contact(support_frame, outer_stage, name="outer_stage_captured_by_support")
    ctx.expect_contact(outer_stage, middle_stage, name="middle_stage_supported_in_outer")
    ctx.expect_contact(middle_stage, inner_stage, name="inner_stage_supported_in_middle")
    ctx.expect_contact(inner_stage, faceplate, name="faceplate_seated_on_inner_stage")

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_within(middle_stage, outer_stage, axes="xy", margin=0.0, name="middle_stage_nested_in_outer")
        ctx.expect_within(inner_stage, middle_stage, axes="xy", margin=0.0, name="inner_stage_nested_in_middle")

    with ctx.pose(
        {
            outer_to_middle: OUTER_STAGE_TRAVEL,
            middle_to_inner: INNER_STAGE_TRAVEL,
            inner_to_faceplate: pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_extension")
        ctx.expect_origin_gap(
            middle_stage,
            outer_stage,
            axis="z",
            min_gap=OUTER_STAGE_HOME_Z + OUTER_STAGE_TRAVEL - 0.001,
            max_gap=OUTER_STAGE_HOME_Z + OUTER_STAGE_TRAVEL + 0.001,
            name="middle_stage_extends_upward",
        )
        ctx.expect_origin_gap(
            inner_stage,
            middle_stage,
            axis="z",
            min_gap=INNER_STAGE_HOME_Z + INNER_STAGE_TRAVEL - 0.001,
            max_gap=INNER_STAGE_HOME_Z + INNER_STAGE_TRAVEL + 0.001,
            name="inner_stage_extends_upward",
        )
        ctx.expect_contact(outer_stage, middle_stage, name="middle_stage_remains_guided_when_extended")
        ctx.expect_contact(middle_stage, inner_stage, name="inner_stage_remains_guided_when_extended")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
