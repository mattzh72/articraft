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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


FRAME_THICKNESS = 0.0064
HINGE_BARREL_RADIUS = 0.0020
HINGE_BARREL_LENGTH = 0.0045
HINGE_BARREL_PITCH = 0.0045
HINGE_Z = 0.010
LEFT_HINGE_X = -0.074
RIGHT_HINGE_X = 0.074
LEFT_HINGE_Y = -0.0118
RIGHT_HINGE_Y = -0.0064
TEMPLE_FOLD_ANGLE = math.radians(85.0)


def _mirror_profile_x(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, z) for x, z in points]


def _temple_section(
    y_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    radius = min(width, height) * 0.28
    profile = rounded_rect_profile(width, height, radius)
    return [(x, y_pos, z + z_center) for x, z in profile]


def _build_front_frame_mesh():
    outer_profile = [
        (0.000, 0.025),
        (0.018, 0.025),
        (0.046, 0.025),
        (0.069, 0.025),
        (0.077, 0.016),
        (0.079, 0.004),
        (0.079, -0.008),
        (0.073, -0.019),
        (0.058, -0.025),
        (0.037, -0.027),
        (0.018, -0.023),
        (0.008, -0.014),
        (0.006, -0.004),
        (0.008, 0.009),
        (0.000, 0.015),
    ]
    outer_profile = list(reversed(_mirror_profile_x(outer_profile[1:]))) + outer_profile

    right_lens = [
        (0.015, 0.014),
        (0.037, 0.014),
        (0.051, 0.012),
        (0.057, 0.003),
        (0.056, -0.010),
        (0.048, -0.017),
        (0.028, -0.018),
        (0.011, -0.015),
        (0.007, -0.006),
        (0.008, 0.008),
    ]
    left_lens = _mirror_profile_x(right_lens)

    front_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [left_lens, right_lens],
        FRAME_THICKNESS,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(front_geom, "wayfarer_front_frame")


def _build_temple_mesh(*, stack_offset: float):
    temple_sections = [
        _temple_section(-0.001, width=0.0046, height=0.0135, z_center=0.0000),
        _temple_section(-0.010, width=0.0086, height=0.0150, z_center=0.0015 * (1.0 if stack_offset >= 0.0 else -1.0)),
        _temple_section(-0.030, width=0.0095, height=0.0140, z_center=stack_offset * 0.55),
        _temple_section(-0.070, width=0.0088, height=0.0120, z_center=stack_offset),
        _temple_section(-0.110, width=0.0072, height=0.0090, z_center=stack_offset - 0.0025),
        _temple_section(-0.145, width=0.0056, height=0.0070, z_center=stack_offset - 0.0060),
    ]
    temple_geom = section_loft(temple_sections)
    mesh_name = "broad_temple_arm_upper" if stack_offset > 0.0 else "broad_temple_arm_lower"
    return mesh_from_geometry(temple_geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wayfarer_glasses")

    acetate_black = model.material("acetate_black", rgba=(0.06, 0.06, 0.07, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.64, 0.67, 1.0))

    frame_mesh = _build_front_frame_mesh()
    left_temple_mesh = _build_temple_mesh(stack_offset=0.0075)
    right_temple_mesh = _build_temple_mesh(stack_offset=-0.0075)

    front_frame = model.part("front_frame")
    front_frame.visual(frame_mesh, material=acetate_black, name="front_rim")
    front_frame.visual(
        Box((0.006, 0.011, 0.020)),
        origin=Origin(xyz=(LEFT_HINGE_X, -0.0087, HINGE_Z)),
        material=acetate_black,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.006, 0.008, 0.020)),
        origin=Origin(xyz=(RIGHT_HINGE_X, -0.0066, HINGE_Z)),
        material=acetate_black,
        name="right_hinge_block",
    )
    for side_name, hinge_x, hinge_y in (
        ("left", LEFT_HINGE_X, LEFT_HINGE_Y),
        ("right", RIGHT_HINGE_X, RIGHT_HINGE_Y),
    ):
        front_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(hinge_x, hinge_y, HINGE_Z - HINGE_BARREL_PITCH)),
            material=hinge_metal,
            name=f"{side_name}_lower_barrel",
        )
        front_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(hinge_x, hinge_y, HINGE_Z + HINGE_BARREL_PITCH)),
            material=hinge_metal,
            name=f"{side_name}_upper_barrel",
        )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        material=hinge_metal,
        name="hinge_barrel",
    )
    left_temple.visual(
        left_temple_mesh,
        material=acetate_black,
        name="temple_arm",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        material=hinge_metal,
        name="hinge_barrel",
    )
    right_temple.visual(
        right_temple_mesh,
        material=acetate_black,
        name="temple_arm",
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(LEFT_HINGE_X, LEFT_HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=TEMPLE_FOLD_ANGLE,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(RIGHT_HINGE_X, RIGHT_HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=TEMPLE_FOLD_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

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
        "left_hinge_axis_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        f"unexpected left hinge axis {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        f"unexpected right hinge axis {right_hinge.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_lower_barrel",
            elem_b="hinge_barrel",
            name="left_lower_barrel_contact_open",
        )
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_upper_barrel",
            elem_b="hinge_barrel",
            name="left_upper_barrel_contact_open",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_lower_barrel",
            elem_b="hinge_barrel",
            name="right_lower_barrel_contact_open",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_upper_barrel",
            elem_b="hinge_barrel",
            name="right_upper_barrel_contact_open",
        )

    with ctx.pose({left_hinge: TEMPLE_FOLD_ANGLE, right_hinge: TEMPLE_FOLD_ANGLE}):
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_lower_barrel",
            elem_b="hinge_barrel",
            name="left_lower_barrel_contact_folded",
        )
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_upper_barrel",
            elem_b="hinge_barrel",
            name="left_upper_barrel_contact_folded",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_lower_barrel",
            elem_b="hinge_barrel",
            name="right_lower_barrel_contact_folded",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_upper_barrel",
            elem_b="hinge_barrel",
            name="right_upper_barrel_contact_folded",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_has_no_overlaps")

        left_folded_aabb = ctx.part_world_aabb(left_temple)
        right_folded_aabb = ctx.part_world_aabb(right_temple)
        if left_folded_aabb is None or right_folded_aabb is None:
            ctx.fail("folded_temple_pose_available", "Unable to evaluate folded temple AABBs.")
        else:
            ctx.check(
                "left_temple_folds_inward",
                left_folded_aabb[1][0] > -0.005,
                f"left temple stayed too far left when folded: max_x={left_folded_aabb[1][0]:.4f}",
            )
            ctx.check(
                "right_temple_folds_inward",
                right_folded_aabb[0][0] < 0.005,
                f"right temple stayed too far right when folded: min_x={right_folded_aabb[0][0]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
