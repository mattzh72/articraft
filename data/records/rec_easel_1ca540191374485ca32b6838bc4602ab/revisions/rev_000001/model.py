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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_sided_display_easel")

    model.material("powder_coat", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.09, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="powder_coat",
        name="main_foot",
    )
    base.visual(
        Box((0.09, 0.52, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="powder_coat",
        name="cross_foot",
    )
    base.visual(
        Box((0.16, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material="powder_coat",
        name="center_plinth",
    )
    base.visual(
        Box((0.009, 0.062, 0.62)),
        origin=Origin(xyz=(-0.0455, 0.0, 0.455)),
        material="powder_coat",
        name="sleeve_left_wall",
    )
    base.visual(
        Box((0.009, 0.062, 0.62)),
        origin=Origin(xyz=(0.0455, 0.0, 0.455)),
        material="powder_coat",
        name="sleeve_right_wall",
    )
    base.visual(
        Box((0.082, 0.009, 0.62)),
        origin=Origin(xyz=(0.0, 0.0265, 0.455)),
        material="powder_coat",
        name="sleeve_front_wall",
    )
    base.visual(
        Box((0.082, 0.009, 0.62)),
        origin=Origin(xyz=(0.0, -0.0265, 0.455)),
        material="powder_coat",
        name="sleeve_back_wall",
    )
    base.visual(
        Box((0.05, 0.003, 0.62)),
        origin=Origin(xyz=(0.0, 0.0175, 0.455)),
        material="aluminum",
        name="front_guide",
    )
    base.visual(
        Box((0.05, 0.003, 0.62)),
        origin=Origin(xyz=(0.0, -0.0175, 0.455)),
        material="aluminum",
        name="rear_guide",
    )
    base.visual(
        Box((0.018, 0.072, 0.028)),
        origin=Origin(xyz=(-0.046, 0.0, 0.751)),
        material="powder_coat",
        name="collar_left_wall",
    )
    base.visual(
        Box((0.018, 0.072, 0.028)),
        origin=Origin(xyz=(0.046, 0.0, 0.751)),
        material="powder_coat",
        name="collar_right_wall",
    )
    base.visual(
        Box((0.074, 0.019, 0.028)),
        origin=Origin(xyz=(0.0, 0.0265, 0.751)),
        material="powder_coat",
        name="collar_front_wall",
    )
    base.visual(
        Box((0.074, 0.019, 0.028)),
        origin=Origin(xyz=(0.0, -0.0265, 0.751)),
        material="powder_coat",
        name="collar_back_wall",
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Box((0.072, 0.032, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material="aluminum",
        name="inner_post",
    )
    upper_column.visual(
        Box((0.14, 0.07, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material="powder_coat",
        name="head_block",
    )
    upper_column.visual(
        Box((0.16, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.846)),
        material="powder_coat",
        name="head_cap",
    )
    upper_column.visual(
        Box((0.12, 0.014, 0.07)),
        origin=Origin(xyz=(0.0, 0.028, 0.875)),
        material="powder_coat",
        name="front_clip_bracket",
    )
    upper_column.visual(
        Box((0.12, 0.014, 0.07)),
        origin=Origin(xyz=(0.0, -0.028, 0.875)),
        material="powder_coat",
        name="back_clip_bracket",
    )

    def add_tray(name: str, sign: float) -> None:
        roll = -0.34 * sign
        tray = model.part(name)
        tray.visual(
            Cylinder(radius=0.011, length=0.44),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material="aluminum",
            name="hinge_barrel",
        )
        tray.visual(
            Box((0.14, 0.044, 0.024)),
            origin=Origin(xyz=(0.0, 0.022 * sign, -0.006)),
            material="powder_coat",
            name="hinge_block",
        )
        tray.visual(
            Box((0.22, 0.10, 0.04)),
            origin=Origin(xyz=(0.0, 0.070 * sign, -0.028), rpy=(roll, 0.0, 0.0)),
            material="powder_coat",
            name="tray_spine",
        )
        tray.visual(
            Box((0.46, 0.28, 0.012)),
            origin=Origin(xyz=(0.0, 0.160 * sign, -0.060), rpy=(roll, 0.0, 0.0)),
            material="powder_coat",
            name="tray_panel",
        )
        tray.visual(
            Box((0.46, 0.014, 0.028)),
            origin=Origin(xyz=(0.0, 0.297 * sign, -0.104), rpy=(roll, 0.0, 0.0)),
            material="powder_coat",
            name="tray_lip",
        )
        tray.visual(
            Box((0.018, 0.20, 0.055)),
            origin=Origin(xyz=(-0.185, 0.150 * sign, -0.073), rpy=(roll, 0.0, 0.0)),
            material="powder_coat",
            name="left_rib",
        )
        tray.visual(
            Box((0.018, 0.20, 0.055)),
            origin=Origin(xyz=(0.185, 0.150 * sign, -0.073), rpy=(roll, 0.0, 0.0)),
            material="powder_coat",
            name="right_rib",
        )

        model.articulation(
            f"upper_column_to_{name}",
            ArticulationType.REVOLUTE,
            parent=upper_column,
            child=tray,
            origin=Origin(xyz=(0.0, 0.046 * sign, 0.79)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=-0.20,
                upper=0.85,
            ),
        )

    def add_clip(name: str, sign: float) -> None:
        roll = -0.42 * sign
        clip = model.part(name)
        clip.visual(
            Cylinder(radius=0.007, length=0.38),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material="aluminum",
            name="hinge_barrel",
        )
        clip.visual(
            Box((0.08, 0.020, 0.016)),
            origin=Origin(xyz=(0.0, 0.010 * sign, -0.004)),
            material="aluminum",
            name="hinge_block",
        )
        clip.visual(
            Box((0.022, 0.09, 0.012)),
            origin=Origin(xyz=(-0.155, 0.050 * sign, -0.014), rpy=(roll, 0.0, 0.0)),
            material="aluminum",
            name="left_link",
        )
        clip.visual(
            Box((0.022, 0.09, 0.012)),
            origin=Origin(xyz=(0.155, 0.050 * sign, -0.014), rpy=(roll, 0.0, 0.0)),
            material="aluminum",
            name="right_link",
        )
        clip.visual(
            Box((0.40, 0.018, 0.028)),
            origin=Origin(xyz=(0.0, 0.095 * sign, -0.045), rpy=(roll, 0.0, 0.0)),
            material="aluminum",
            name="clip_rail",
        )
        clip.visual(
            Box((0.40, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, 0.095 * sign, -0.063), rpy=(roll, 0.0, 0.0)),
            material="rubber",
            name="clip_pad",
        )

        model.articulation(
            f"upper_column_to_{name}",
            ArticulationType.REVOLUTE,
            parent=upper_column,
            child=clip,
            origin=Origin(xyz=(0.0, 0.042 * sign, 0.892)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.5,
                lower=-0.25,
                upper=1.10,
            ),
        )

    add_tray("front_tray", 1.0)
    add_tray("back_tray", -1.0)
    add_clip("front_clip", 1.0)
    add_clip("back_clip", -1.0)

    model.articulation(
        "base_to_upper_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_column = object_model.get_part("upper_column")
    front_tray = object_model.get_part("front_tray")
    back_tray = object_model.get_part("back_tray")
    front_clip = object_model.get_part("front_clip")
    back_clip = object_model.get_part("back_clip")

    column_slide = object_model.get_articulation("base_to_upper_column")
    front_tray_hinge = object_model.get_articulation("upper_column_to_front_tray")
    back_tray_hinge = object_model.get_articulation("upper_column_to_back_tray")
    front_clip_hinge = object_model.get_articulation("upper_column_to_front_clip")
    back_clip_hinge = object_model.get_articulation("upper_column_to_back_clip")

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

    def element_center(part_name: str, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
        )

    ctx.expect_gap(
        base,
        upper_column,
        axis="y",
        positive_elem="front_guide",
        negative_elem="inner_post",
        max_gap=0.0005,
        max_penetration=0.0,
        name="front guide rides against the inner post",
    )
    ctx.expect_gap(
        upper_column,
        base,
        axis="y",
        positive_elem="inner_post",
        negative_elem="rear_guide",
        max_gap=0.0005,
        max_penetration=0.0,
        name="rear guide rides against the inner post",
    )
    ctx.expect_overlap(
        base,
        upper_column,
        axes="z",
        elem_a="front_guide",
        elem_b="inner_post",
        min_overlap=0.30,
        name="inner post remains deeply engaged in the sleeve at rest",
    )

    ctx.expect_contact(
        upper_column,
        front_tray,
        elem_a="head_block",
        elem_b="hinge_barrel",
        contact_tol=5e-4,
        name="front tray hinge bears on the column head",
    )
    ctx.expect_contact(
        upper_column,
        back_tray,
        elem_a="head_block",
        elem_b="hinge_barrel",
        contact_tol=5e-4,
        name="back tray hinge bears on the column head",
    )
    ctx.expect_contact(
        upper_column,
        front_clip,
        elem_a="front_clip_bracket",
        elem_b="hinge_barrel",
        contact_tol=5e-4,
        name="front clip hinge bears on its bracket",
    )
    ctx.expect_contact(
        upper_column,
        back_clip,
        elem_a="back_clip_bracket",
        elem_b="hinge_barrel",
        contact_tol=5e-4,
        name="back clip hinge bears on its bracket",
    )

    front_center = element_center("front_tray", "tray_panel")
    back_center = element_center("back_tray", "tray_panel")
    ctx.check(
        "display trays face opposite directions",
        front_center is not None
        and back_center is not None
        and front_center[1] > 0.10
        and back_center[1] < -0.10,
        details=f"front_center={front_center}, back_center={back_center}",
    )

    rest_column_pos = ctx.part_world_position(upper_column)
    slide_upper = column_slide.motion_limits.upper
    with ctx.pose({column_slide: slide_upper}):
        extended_column_pos = ctx.part_world_position(upper_column)
        ctx.expect_overlap(
            base,
            upper_column,
            axes="z",
            elem_a="front_guide",
            elem_b="inner_post",
            min_overlap=0.09,
            name="inner post keeps retained insertion at full extension",
        )
    ctx.check(
        "upper column extends upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and slide_upper is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.20,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}, upper={slide_upper}",
    )

    rest_front_tray = element_center("front_tray", "tray_panel")
    with ctx.pose({front_tray_hinge: 0.55}):
        opened_front_tray = element_center("front_tray", "tray_panel")
    ctx.check(
        "front tray folds upward with positive rotation",
        rest_front_tray is not None
        and opened_front_tray is not None
        and opened_front_tray[2] > rest_front_tray[2] + 0.06,
        details=f"rest={rest_front_tray}, opened={opened_front_tray}",
    )

    rest_back_tray = element_center("back_tray", "tray_panel")
    with ctx.pose({back_tray_hinge: 0.55}):
        opened_back_tray = element_center("back_tray", "tray_panel")
    ctx.check(
        "back tray folds upward with positive rotation",
        rest_back_tray is not None
        and opened_back_tray is not None
        and opened_back_tray[2] > rest_back_tray[2] + 0.06,
        details=f"rest={rest_back_tray}, opened={opened_back_tray}",
    )

    rest_front_clip = element_center("front_clip", "clip_rail")
    with ctx.pose({front_clip_hinge: 0.65}):
        opened_front_clip = element_center("front_clip", "clip_rail")
    ctx.check(
        "front clip rail opens upward with positive rotation",
        rest_front_clip is not None
        and opened_front_clip is not None
        and opened_front_clip[2] > rest_front_clip[2] + 0.05,
        details=f"rest={rest_front_clip}, opened={opened_front_clip}",
    )

    rest_back_clip = element_center("back_clip", "clip_rail")
    with ctx.pose({back_clip_hinge: 0.65}):
        opened_back_clip = element_center("back_clip", "clip_rail")
    ctx.check(
        "back clip rail opens upward with positive rotation",
        rest_back_clip is not None
        and opened_back_clip is not None
        and opened_back_clip[2] > rest_back_clip[2] + 0.05,
        details=f"rest={rest_back_clip}, opened={opened_back_clip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
