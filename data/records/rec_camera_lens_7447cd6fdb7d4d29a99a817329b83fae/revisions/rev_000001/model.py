from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def shell_from_outer_profile(
    outer_profile: list[tuple[float, float]],
    *,
    wall: float,
    segments: int = 64,
) -> object:
    inner_profile = [(max(0.001, radius - wall), z) for radius, z in outer_profile]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_shift_architecture_lens")

    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.21, 1.0))
    matte_rubber = model.material("matte_rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    rear_body = model.part("rear_body")
    shift_carriage = model.part("shift_carriage")
    front_group = model.part("front_group")

    mount_plate_geom = shell_from_outer_profile(
        [(0.032, -0.004), (0.032, 0.0)],
        wall=0.013,
    )
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        lug = BoxGeometry((0.011, 0.004, 0.002))
        lug.rotate_z(angle).translate(
            0.0285 * math.cos(angle),
            0.0285 * math.sin(angle),
            -0.001,
        )
        mount_plate_geom.merge(lug)

    rear_barrel_geom = shell_from_outer_profile(
        [
            (0.032, 0.0),
            (0.034, 0.010),
            (0.036, 0.024),
            (0.036, 0.052),
        ],
        wall=0.0035,
    )

    fixed_slide_frame_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.072, 0.060, 0.010, corner_segments=8),
        [circle_profile(0.025, segments=40)],
        0.004,
        center=True,
    ).translate(0.0, 0.0, 0.054)

    rear_body.visual(
        mesh_from_geometry(mount_plate_geom, "rear_mount_plate"),
        material=mount_steel,
        name="rear_mount_plate",
    )
    rear_body.visual(
        mesh_from_geometry(rear_barrel_geom, "rear_barrel_shell"),
        material=anodized_black,
        name="rear_barrel_shell",
    )
    rear_body.visual(
        mesh_from_geometry(fixed_slide_frame_geom, "fixed_slide_frame"),
        material=gunmetal,
        name="fixed_slide_frame",
    )
    rear_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.060),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    shift_slide_frame_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.082, 0.064, 0.010, corner_segments=8),
        [circle_profile(0.026, segments=40)],
        0.008,
        center=True,
    ).translate(0.0, 0.0, 0.004)

    shift_housing_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.094, 0.072, 0.012, corner_segments=8),
        [circle_profile(0.028, segments=44)],
        0.020,
        center=True,
    ).translate(0.0, 0.0, 0.018)

    shift_carriage.visual(
        mesh_from_geometry(shift_slide_frame_geom, "shift_slide_frame"),
        material=gunmetal,
        name="shift_slide_frame",
    )
    shift_carriage.visual(
        mesh_from_geometry(shift_housing_geom, "shift_housing"),
        material=anodized_black,
        name="shift_housing",
    )
    shift_carriage.visual(
        Box((0.006, 0.026, 0.018)),
        origin=Origin(xyz=(-0.049, 0.0, 0.037)),
        material=gunmetal,
        name="left_cheek",
    )
    shift_carriage.visual(
        Box((0.006, 0.026, 0.018)),
        origin=Origin(xyz=(0.049, 0.0, 0.037)),
        material=gunmetal,
        name="right_cheek",
    )
    shift_carriage.inertial = Inertial.from_geometry(
        Box((0.094, 0.072, 0.046)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    front_barrel_geom = shell_from_outer_profile(
        [
            (0.029, -0.004),
            (0.031, 0.006),
            (0.034, 0.020),
            (0.039, 0.042),
            (0.042, 0.058),
        ],
        wall=0.004,
    )
    focus_ring_geom = shell_from_outer_profile(
        [(0.037, 0.016), (0.037, 0.034)],
        wall=0.0025,
    )

    front_group.visual(
        mesh_from_geometry(front_barrel_geom, "front_group_barrel"),
        material=anodized_black,
        name="front_group_barrel",
    )
    front_group.visual(
        mesh_from_geometry(focus_ring_geom, "front_focus_ring"),
        material=matte_rubber,
        name="front_focus_ring",
    )
    front_group.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion_arm",
    )
    front_group.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion_arm",
    )
    front_group.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_steel,
        name="left_pivot_disc",
    )
    front_group.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_steel,
        name="right_pivot_disc",
    )
    front_group.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.062),
        mass=0.33,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    model.articulation(
        "rear_body_to_shift_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_body,
        child=shift_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.04,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "shift_carriage_to_front_group",
        ArticulationType.REVOLUTE,
        parent=shift_carriage,
        child=front_group,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.14,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_body = object_model.get_part("rear_body")
    shift_carriage = object_model.get_part("shift_carriage")
    front_group = object_model.get_part("front_group")
    shift_joint = object_model.get_articulation("rear_body_to_shift_carriage")
    tilt_joint = object_model.get_articulation("shift_carriage_to_front_group")

    fixed_slide_frame = rear_body.get_visual("fixed_slide_frame")
    shift_slide_frame = shift_carriage.get_visual("shift_slide_frame")
    left_cheek = shift_carriage.get_visual("left_cheek")
    right_cheek = shift_carriage.get_visual("right_cheek")
    left_pivot_disc = front_group.get_visual("left_pivot_disc")
    right_pivot_disc = front_group.get_visual("right_pivot_disc")

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
        "single_root_body",
        len(object_model.root_parts()) == 1,
        details="Lens assembly should resolve to one rooted rear body.",
    )
    ctx.check(
        "shift_joint_type_and_axis",
        shift_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(shift_joint.axis) == (1.0, 0.0, 0.0),
        details="Shift mechanism should slide laterally on an x-axis prismatic joint.",
    )
    ctx.check(
        "tilt_joint_type_and_axis",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        details="Front group should pivot on a horizontal x-axis revolute joint.",
    )

    with ctx.pose({shift_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_contact(
            shift_carriage,
            rear_body,
            elem_a=shift_slide_frame,
            elem_b=fixed_slide_frame,
            name="shift_slide_faces_contact_at_rest",
        )
        ctx.expect_overlap(
            shift_carriage,
            rear_body,
            axes="xy",
            elem_a=shift_slide_frame,
            elem_b=fixed_slide_frame,
            min_overlap=0.050,
            name="shift_slide_faces_have_bearing_footprint_at_rest",
        )
        ctx.expect_contact(
            front_group,
            shift_carriage,
            elem_a=left_pivot_disc,
            elem_b=left_cheek,
            name="left_tilt_pivot_is_supported",
        )
        ctx.expect_contact(
            front_group,
            shift_carriage,
            elem_a=right_pivot_disc,
            elem_b=right_cheek,
            name="right_tilt_pivot_is_supported",
        )
        ctx.expect_origin_gap(
            front_group,
            shift_carriage,
            axis="z",
            min_gap=0.044,
            max_gap=0.048,
            name="front_group_sits_forward_of_shift_housing",
        )

    with ctx.pose({shift_joint: 0.012, tilt_joint: 0.0}):
        ctx.expect_contact(
            shift_carriage,
            rear_body,
            elem_a=shift_slide_frame,
            elem_b=fixed_slide_frame,
            name="shift_carriage_remains_supported_at_max_shift",
        )
        ctx.expect_overlap(
            shift_carriage,
            rear_body,
            axes="xy",
            elem_a=shift_slide_frame,
            elem_b=fixed_slide_frame,
            min_overlap=0.050,
            name="shift_slide_faces_keep_bearing_footprint_at_max_shift",
        )
        ctx.expect_origin_distance(
            shift_carriage,
            rear_body,
            axes="x",
            min_dist=0.0115,
            max_dist=0.0125,
            name="shift_stage_reaches_lateral_travel",
        )

    with ctx.pose({shift_joint: 0.0, tilt_joint: 0.14}):
        ctx.expect_contact(
            front_group,
            shift_carriage,
            elem_a=left_pivot_disc,
            elem_b=left_cheek,
            name="left_pivot_contacts_at_max_tilt",
        )
        ctx.expect_contact(
            front_group,
            shift_carriage,
            elem_a=right_pivot_disc,
            elem_b=right_cheek,
            name="right_pivot_contacts_at_max_tilt",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
