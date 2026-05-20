from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _shield_section(
    *,
    x_pos: float,
    y_center: float,
    z_center: float,
    height: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        thickness,
        height,
        radius=min(thickness * 0.45, height * 0.12),
        corner_segments=6,
    )
    section = []
    for y_off, z_off in profile:
        section.append((x_pos, y_center + y_off - (0.10 * z_off), z_center + z_off))
    return section


def _build_shield_mesh():
    stations = [
        (-0.076, -0.028, 0.0275, 0.040, 0.0024),
        (-0.056, -0.020, 0.0288, 0.044, 0.0026),
        (-0.028, -0.009, 0.0302, 0.048, 0.0027),
        (0.000, 0.000, 0.0310, 0.051, 0.0028),
        (0.028, -0.009, 0.0302, 0.048, 0.0027),
        (0.056, -0.020, 0.0288, 0.044, 0.0026),
        (0.076, -0.028, 0.0275, 0.040, 0.0024),
    ]
    return section_loft(
        [
            _shield_section(
                x_pos=x_pos,
                y_center=y_center,
                z_center=z_center,
                height=height,
                thickness=thickness,
            )
            for x_pos, y_center, z_center, height, thickness in stations
        ]
    )


def _temple_section(
    *,
    y_pos: float,
    z_center: float,
    width: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        thickness,
        radius=min(width * 0.28, thickness * 0.48),
        corner_segments=5,
    )
    return [(x_off, y_pos, z_center + z_off) for x_off, z_off in profile]


def _build_temple_arm_mesh(*, arm_z_offset: float):
    return section_loft(
        [
            _temple_section(y_pos=-0.012, z_center=arm_z_offset * 0.35, width=0.0068, thickness=0.0048),
            _temple_section(y_pos=-0.030, z_center=arm_z_offset * 0.55, width=0.0067, thickness=0.0042),
            _temple_section(y_pos=-0.066, z_center=arm_z_offset * 0.82, width=0.0061, thickness=0.0036),
            _temple_section(y_pos=-0.104, z_center=arm_z_offset, width=0.0058, thickness=0.0031),
            _temple_section(y_pos=-0.120, z_center=arm_z_offset, width=0.0055, thickness=0.0028),
        ]
    )


def _build_temple_tip_mesh(*, arm_z_offset: float):
    return section_loft(
        [
            _temple_section(y_pos=-0.118, z_center=arm_z_offset, width=0.0055, thickness=0.0029),
            _temple_section(y_pos=-0.138, z_center=arm_z_offset, width=0.0053, thickness=0.0028),
            _temple_section(y_pos=-0.156, z_center=arm_z_offset, width=0.0049, thickness=0.0026),
        ]
    )


def _build_temple(part, *, arm_z_offset: float, temple_material, tip_material) -> None:
    part.visual(
        Box((0.0056, 0.0078, 0.0054)),
        origin=Origin(xyz=(0.0, -0.0039, 0.0)),
        material=temple_material,
        name="hinge_tongue",
    )
    part.visual(
        Box((0.0063, 0.0120, 0.0048)),
        origin=Origin(xyz=(0.0, -0.0110, arm_z_offset * 0.5)),
        material=temple_material,
        name="hinge_transition",
    )
    part.visual(
        mesh_from_geometry(
            _build_temple_arm_mesh(arm_z_offset=arm_z_offset),
            f"temple_arm_{'upper' if arm_z_offset > 0.0 else 'lower'}",
        ),
        material=temple_material,
        name="arm_body",
    )
    part.visual(
        mesh_from_geometry(
            _build_temple_tip_mesh(arm_z_offset=arm_z_offset),
            f"temple_tip_{'upper' if arm_z_offset > 0.0 else 'lower'}",
        ),
        material=tip_material,
        name="tip_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wraparound_sport_glasses")

    frame_black = model.material("frame_black", rgba=(0.08, 0.08, 0.09, 1.0))
    temple_black = model.material("temple_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.14, 0.14, 0.15, 1.0))
    lens_smoke = model.material("lens_smoke", rgba=(0.12, 0.17, 0.20, 0.58))

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_geometry(_build_shield_mesh(), "shield_lens"),
        material=lens_smoke,
        name="shield_lens",
    )

    brow_bar = tube_from_spline_points(
        [
            (-0.074, -0.026, 0.044),
            (-0.048, -0.013, 0.048),
            (-0.020, -0.003, 0.050),
            (0.000, 0.003, 0.051),
            (0.020, -0.003, 0.050),
            (0.048, -0.013, 0.048),
            (0.074, -0.026, 0.044),
        ],
        radius=0.0042,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    front_frame.visual(
        mesh_from_geometry(brow_bar, "brow_bar"),
        material=frame_black,
        name="brow_bar",
    )

    nosepiece = tube_from_spline_points(
        [
            (-0.010, -0.003, 0.015),
            (-0.008, -0.008, 0.010),
            (-0.005, -0.011, 0.004),
            (0.000, -0.013, 0.000),
            (0.005, -0.011, 0.004),
            (0.008, -0.008, 0.010),
            (0.010, -0.003, 0.015),
        ],
        radius=0.0028,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    front_frame.visual(
        mesh_from_geometry(nosepiece, "integrated_nosepiece"),
        material=rubber_black,
        name="nosepiece",
    )

    hinge_origin_y = -0.029
    hinge_origin_z = 0.029
    lug_x = 0.082
    corner_x = 0.078
    corner_center_y = -0.0225
    upper_lug_center_z = hinge_origin_z + 0.0043
    lower_lug_center_z = hinge_origin_z - 0.0043
    lug_center_y = hinge_origin_y - 0.0018

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        front_frame.visual(
            Box((0.0110, 0.0100, 0.0210)),
            origin=Origin(xyz=(sign * corner_x, corner_center_y, 0.0295)),
            material=frame_black,
            name=f"{side_name}_corner_block",
        )
        front_frame.visual(
            Box((0.0068, 0.0086, 0.0032)),
            origin=Origin(xyz=(sign * lug_x, lug_center_y, upper_lug_center_z)),
            material=frame_black,
            name=f"{side_name}_upper_lug",
        )
        front_frame.visual(
            Box((0.0068, 0.0086, 0.0032)),
            origin=Origin(xyz=(sign * lug_x, lug_center_y, lower_lug_center_z)),
            material=frame_black,
            name=f"{side_name}_lower_lug",
        )

    left_temple = model.part("left_temple")
    _build_temple(
        left_temple,
        arm_z_offset=0.0020,
        temple_material=temple_black,
        tip_material=rubber_black,
    )

    right_temple = model.part("right_temple")
    _build_temple(
        right_temple,
        arm_z_offset=-0.0020,
        temple_material=temple_black,
        tip_material=rubber_black,
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-lug_x, hinge_origin_y, hinge_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(lug_x, hinge_origin_y, hinge_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=math.radians(-102.0),
            upper=0.0,
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
        "left_hinge_axis",
        tuple(left_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected +Z hinge axis, got {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis",
        tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected +Z hinge axis, got {right_hinge.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left_hinge_limits",
        left_limits is not None
        and math.isclose(left_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and left_limits.upper is not None
        and left_limits.upper > 1.5,
        details=f"Unexpected left hinge limits: {left_limits}",
    )
    ctx.check(
        "right_hinge_limits",
        right_limits is not None
        and right_limits.lower is not None
        and right_limits.lower < -1.5
        and math.isclose(right_limits.upper or 0.0, 0.0, abs_tol=1e-9),
        details=f"Unexpected right hinge limits: {right_limits}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="left_upper_lug",
        )
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="left_lower_lug",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="right_upper_lug",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="right_lower_lug",
        )

    with ctx.pose({left_hinge: 1.10, right_hinge: -1.10}):
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="left_upper_lug",
        )
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="left_lower_lug",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="right_upper_lug",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a="hinge_tongue",
            elem_b="right_lower_lug",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
