from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_SUPPORT_Z = 0.215
LOWER_TO_MIDDLE_Z = 0.093
MIDDLE_TO_UPPER_Z = 0.074
HOUSING_BODY_TOP = 0.178
LOWER_BODY_TOP = 0.070
MIDDLE_BODY_TOP = 0.048


def _make_housing_base() -> cq.Workplane:
    body_profile = [
        (-0.28, 0.0),
        (-0.28, 0.055),
        (-0.23, 0.105),
        (-0.17, 0.145),
        (-0.09, 0.17),
        (-0.03, 0.155),
        (0.0, 0.13),
        (0.03, 0.155),
        (0.09, 0.17),
        (0.17, 0.145),
        (0.23, 0.105),
        (0.28, 0.055),
        (0.28, 0.0),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(body_profile)
        .close()
        .extrude(0.17, both=True)
    )

    front_pocket = cq.Workplane("XY").box(0.24, 0.08, 0.085).translate((0.0, 0.13, 0.08))
    rear_pocket = cq.Workplane("XY").box(0.24, 0.08, 0.085).translate((0.0, -0.13, 0.08))
    body = body.cut(front_pocket).cut(rear_pocket)

    pedestal = (
        cq.Workplane("XY")
        .circle(0.125)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.128))
        .union(
            cq.Workplane("XY")
            .circle(0.10)
            .extrude(HOUSING_BODY_TOP - 0.146)
            .translate((0.0, 0.0, 0.146))
        )
    )

    return body.union(pedestal)


def _make_housing_support() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.068)
        .extrude(0.015)
        .translate((0.0, 0.0, HOUSING_BODY_TOP))
        .union(
            cq.Workplane("XY")
            .circle(0.052)
            .extrude(HOUSING_SUPPORT_Z - (HOUSING_BODY_TOP + 0.015))
            .translate((0.0, 0.0, HOUSING_BODY_TOP + 0.015))
        )
    )


def _make_lower_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.17)
        .extrude(0.038)
        .faces(">Z")
        .workplane()
        .circle(0.156)
        .extrude(0.016)
        .faces(">Z")
        .workplane()
        .circle(0.128)
        .extrude(0.008)
        .faces(">Z")
        .workplane()
        .center(0.052, 0.0)
        .rect(0.082, 0.132)
        .extrude(0.008)
    )


def _make_lower_support() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.054)
        .extrude(0.014)
        .translate((0.0, 0.0, LOWER_BODY_TOP))
        .union(
            cq.Workplane("XY")
            .circle(0.041)
            .extrude(LOWER_TO_MIDDLE_Z - (LOWER_BODY_TOP + 0.014))
            .translate((0.0, 0.0, LOWER_BODY_TOP + 0.014))
        )
    )


def _make_middle_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.118)
        .extrude(0.030)
        .faces(">Z")
        .workplane()
        .circle(0.105)
        .extrude(0.012)
        .faces(">Z")
        .workplane()
        .center(-0.03, 0.018)
        .rect(0.058, 0.076)
        .extrude(0.006)
    )


def _make_middle_support() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.014)
        .translate((0.0, 0.0, MIDDLE_BODY_TOP))
        .union(
            cq.Workplane("XY")
            .circle(0.030)
            .extrude(MIDDLE_TO_UPPER_Z - (MIDDLE_BODY_TOP + 0.014))
            .translate((0.0, 0.0, MIDDLE_BODY_TOP + 0.014))
        )
    )


def _make_upper_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.080)
        .extrude(0.028)
        .faces(">Z")
        .workplane()
        .circle(0.066)
        .extrude(0.014)
        .faces(">Z")
        .workplane()
        .center(0.022, -0.004)
        .rect(0.028, 0.072)
        .extrude(0.013)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_coaxial_stack")

    dark_shell = model.material("dark_shell", rgba=(0.18, 0.20, 0.23, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.31, 0.34, 1.0))
    machined = model.material("machined", rgba=(0.63, 0.65, 0.69, 1.0))
    light_cap = model.material("light_cap", rgba=(0.76, 0.78, 0.80, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_base(), "housing_base"),
        material=dark_shell,
        name="housing_base",
    )
    housing.visual(
        mesh_from_cadquery(_make_housing_support(), "housing_support"),
        material=machined,
        name="housing_support",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_body(), "lower_body"),
        material=graphite,
        name="lower_body",
    )
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_support(), "lower_support"),
        material=machined,
        name="lower_support",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(_make_middle_body(), "middle_body"),
        material=machined,
        name="middle_body",
    )
    middle_stage.visual(
        mesh_from_cadquery(_make_middle_support(), "middle_support"),
        material=light_cap,
        name="middle_support",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_make_upper_body(), "upper_body"),
        material=light_cap,
        name="upper_body",
    )

    model.articulation(
        "housing_to_lower",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_SUPPORT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TO_MIDDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.2,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.REVOLUTE,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.6,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("housing_to_lower")
    middle_joint = object_model.get_articulation("lower_to_middle")
    upper_joint = object_model.get_articulation("middle_to_upper")

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

    axes_vertical = all(
        tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0)
        for joint in (lower_joint, middle_joint, upper_joint)
    )
    ctx.check(
        "coaxial_rotary_axes_are_vertical",
        axes_vertical,
        "All rotary stages should use the shared vertical axis.",
    )

    with ctx.pose(
        {
            lower_joint: 1.1,
            middle_joint: -0.8,
            upper_joint: 0.55,
        }
    ):
        ctx.expect_contact(
            housing,
            housing,
            elem_a="housing_support",
            elem_b="housing_support",
            contact_tol=1e-4,
            name="placeholder",
        )
        ctx.expect_origin_distance(
            lower_stage,
            middle_stage,
            axes="xy",
            max_dist=0.0005,
            name="lower_and_middle_share_coaxial_centerline",
        )
        ctx.expect_origin_distance(
            middle_stage,
            upper_stage,
            axes="xy",
            max_dist=0.0005,
            name="middle_and_upper_share_coaxial_centerline",
        )

    ctx.expect_contact(
        housing,
        lower_stage,
        elem_a="housing_support",
        elem_b="lower_body",
        contact_tol=1e-4,
        name="lower_stage_is_supported_by_housing",
    )
    ctx.expect_contact(
        lower_stage,
        middle_stage,
        elem_a="lower_support",
        elem_b="middle_body",
        contact_tol=1e-4,
        name="middle_stage_is_supported_by_lower_stage",
    )
    ctx.expect_contact(
        middle_stage,
        upper_stage,
        elem_a="middle_support",
        elem_b="upper_body",
        contact_tol=1e-4,
        name="upper_stage_is_supported_by_middle_stage",
    )
    ctx.expect_gap(
        middle_stage,
        lower_stage,
        axis="z",
        positive_elem="middle_body",
        negative_elem="lower_body",
        min_gap=0.020,
        max_gap=0.030,
        name="middle_stage_body_visibly_clears_lower_stage_body",
    )
    ctx.expect_gap(
        upper_stage,
        middle_stage,
        axis="z",
        positive_elem="upper_body",
        negative_elem="middle_body",
        min_gap=0.024,
        max_gap=0.032,
        name="upper_stage_body_visibly_clears_middle_stage_body",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
