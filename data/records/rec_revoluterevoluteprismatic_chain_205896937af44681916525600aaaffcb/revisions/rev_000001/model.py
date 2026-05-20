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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _rear_support_shape() -> cq.Workplane:
    base = _box((0.24, 0.18, 0.03), (-0.12, 0.0, -0.165))
    mast = _box((0.12, 0.12, 0.17), (-0.12, 0.0, -0.075))
    fork_outer = _box((0.11, 0.09, 0.09), (-0.055, 0.0, 0.0))
    fork_slot = _box((0.055, 0.040, 0.060), (-0.0275, 0.0, 0.0))
    return base.union(mast).union(fork_outer).cut(fork_slot)


def _first_link_shape() -> cq.Workplane:
    root_lug = _box((0.024, 0.036, 0.040), (0.012, 0.0, 0.0))
    root_neck = _box((0.038, 0.046, 0.048), (0.041, 0.0, 0.0))
    beam_outer = _box((0.304, 0.054, 0.048), (0.210, 0.0, 0.0))
    clevis_outer = _box((0.060, 0.078, 0.068), (0.360, 0.0, 0.0))
    clevis_slot = _box((0.032, 0.036, 0.052), (0.374, 0.0, 0.0))
    return root_lug.union(root_neck).union(beam_outer).union(clevis_outer).cut(clevis_slot)


def _second_link_shape() -> cq.Workplane:
    root_lug = _box((0.024, 0.032, 0.040), (0.012, 0.0, 0.0))
    root_neck = _box((0.036, 0.042, 0.046), (0.040, 0.0, 0.0))
    beam = _box((0.168, 0.050, 0.044), (0.138, 0.0, 0.0))
    sleeve_outer = _box((0.196, 0.058, 0.050), (0.312, 0.0, 0.0))
    sleeve_inner = _box((0.17, 0.046, 0.038), (0.325, 0.0, 0.0))
    return root_lug.union(root_neck).union(beam).union(sleeve_outer).cut(sleeve_inner)


def _nose_extension_shape() -> cq.Workplane:
    slider = _box((0.16, 0.040, 0.032), (0.08, 0.0, 0.0))
    nose = _box((0.08, 0.032, 0.026), (0.20, 0.0, 0.0))
    return slider.union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_arm")

    support_gray = model.material("support_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    arm_yellow = model.material("arm_yellow", rgba=(0.92, 0.74, 0.18, 1.0))
    extension_metal = model.material("extension_metal", rgba=(0.67, 0.70, 0.74, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        origin=Origin(),
        material=support_gray,
        name="support_shell",
    )

    arm_link_1 = model.part("arm_link_1")
    arm_link_1.visual(
        mesh_from_cadquery(_first_link_shape(), "arm_link_1"),
        origin=Origin(),
        material=arm_yellow,
        name="link_1_shell",
    )

    arm_link_2 = model.part("arm_link_2")
    arm_link_2.visual(
        mesh_from_cadquery(_second_link_shape(), "arm_link_2"),
        origin=Origin(),
        material=arm_yellow,
        name="link_2_shell",
    )

    nose_extension = model.part("nose_extension")
    nose_extension.visual(
        mesh_from_cadquery(_nose_extension_shape(), "nose_extension"),
        origin=Origin(),
        material=extension_metal,
        name="extension_shell",
    )

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=arm_link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.4,
            lower=-0.55,
            upper=0.4,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=arm_link_1,
        child=arm_link_2,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "link_2_to_extension",
        ArticulationType.PRISMATIC,
        parent=arm_link_2,
        child=nose_extension,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    arm_link_1 = object_model.get_part("arm_link_1")
    arm_link_2 = object_model.get_part("arm_link_2")
    nose_extension = object_model.get_part("nose_extension")

    shoulder = object_model.get_articulation("support_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")
    telescope = object_model.get_articulation("link_2_to_extension")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        arm_link_2,
        nose_extension,
        reason="Telescoping extension intentionally nests inside the second-link sleeve.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.003)
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
        "joint_layout_and_types",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and telescope.articulation_type == ArticulationType.PRISMATIC
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and telescope.axis == (1.0, 0.0, 0.0),
        "Expected two pitch joints followed by one telescoping prismatic joint.",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, telescope: 0.0}):
        ctx.expect_contact(
            arm_link_1,
            rear_support,
            contact_tol=0.003,
            name="rear_support_captures_first_link",
        )
        ctx.expect_contact(
            arm_link_1,
            arm_link_2,
            contact_tol=0.003,
            name="first_link_captures_second_link",
        )
        ctx.expect_contact(
            arm_link_2,
            nose_extension,
            contact_tol=0.003,
            name="slider_contacts_second_link_sleeve",
        )
        ctx.expect_origin_gap(
            arm_link_2,
            arm_link_1,
            axis="x",
            min_gap=0.38,
            max_gap=0.40,
            name="second_link_starts_at_first_link_tip",
        )
        ctx.expect_origin_gap(
            nose_extension,
            arm_link_2,
            axis="x",
            min_gap=0.21,
            max_gap=0.23,
            name="extension_starts_at_front_sleeve",
        )
        ctx.expect_within(
            nose_extension,
            arm_link_2,
            axes="yz",
            margin=0.0,
            name="extension_stays_within_sleeve_cross_section_retracted",
        )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, telescope: 0.18}):
        ctx.expect_origin_gap(
            nose_extension,
            arm_link_2,
            axis="x",
            min_gap=0.39,
            max_gap=0.41,
            name="extension_moves_forward_when_extended",
        )
        ctx.expect_within(
            nose_extension,
            arm_link_2,
            axes="yz",
            margin=0.0,
            name="extension_stays_within_sleeve_cross_section_extended",
        )

    with ctx.pose({shoulder: 0.4, elbow: 0.55, telescope: 0.12}):
        ctx.expect_origin_gap(
            arm_link_2,
            rear_support,
            axis="z",
            min_gap=0.14,
            name="shoulder_lifts_the_elbow",
        )
        ctx.expect_origin_gap(
            nose_extension,
            arm_link_2,
            axis="z",
            min_gap=0.08,
            name="elbow_and_extension_raise_the_nose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_work_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
