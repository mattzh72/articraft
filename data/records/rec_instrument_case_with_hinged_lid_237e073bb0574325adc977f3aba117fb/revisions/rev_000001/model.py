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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_hardware_case")

    shell_black = model.material("shell_black", rgba=(0.16, 0.17, 0.18, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    case_length = 1.18
    case_width = 0.42
    lower_height = 0.22
    lid_height = 0.11
    wall = 0.012
    rear_hinge_radius = 0.010

    front_face_y = case_width * 0.5
    back_face_y = -case_width * 0.5
    left_face_x = -case_length * 0.5
    right_face_x = case_length * 0.5

    hinge_axis_y = back_face_y - 0.006
    hinge_axis_z = lower_height + 0.004

    handle_opening_width = 0.24
    handle_opening_bottom = 0.06
    handle_opening_top = 0.16
    pocket_depth = 0.040

    latch_x_positions = (-0.47, -0.27, 0.27, 0.47)
    latch_mount_front_y = front_face_y + 0.010
    latch_pivot_z = lower_height + 0.001

    def lid_local(x: float, y: float, z: float) -> tuple[float, float, float]:
        return (x, y - hinge_axis_y, z - hinge_axis_z)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((case_length, case_width, lower_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, lower_height * 0.5)),
    )

    body.visual(
        Box((case_length - 2.0 * wall, case_width - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=shell_black,
        name="body_floor",
    )
    body.visual(
        Box((case_length, wall, lower_height)),
        origin=Origin(xyz=(0.0, back_face_y + wall * 0.5, lower_height * 0.5)),
        material=shell_black,
        name="body_back_shell",
    )
    body.visual(
        Box((wall, case_width - 2.0 * wall, lower_height)),
        origin=Origin(xyz=(left_face_x + wall * 0.5, 0.0, lower_height * 0.5)),
        material=shell_black,
        name="body_left_shell",
    )
    body.visual(
        Box((wall, case_width - 2.0 * wall, lower_height)),
        origin=Origin(xyz=(right_face_x - wall * 0.5, 0.0, lower_height * 0.5)),
        material=shell_black,
        name="body_right_shell",
    )

    front_side_span = (case_length - handle_opening_width) * 0.5
    front_side_center = handle_opening_width * 0.5 + front_side_span * 0.5
    front_center_y = front_face_y - wall * 0.5

    body.visual(
        Box((front_side_span, wall, lower_height)),
        origin=Origin(xyz=(-front_side_center, front_center_y, lower_height * 0.5)),
        material=shell_black,
        name="body_front_left_segment",
    )
    body.visual(
        Box((front_side_span, wall, lower_height)),
        origin=Origin(xyz=(front_side_center, front_center_y, lower_height * 0.5)),
        material=shell_black,
        name="body_front_right_segment",
    )
    body.visual(
        Box((handle_opening_width, wall, lower_height - handle_opening_top)),
        origin=Origin(
            xyz=(
                0.0,
                front_center_y,
                (handle_opening_top + lower_height) * 0.5,
            )
        ),
        material=shell_black,
        name="body_front_upper_strip",
    )
    body.visual(
        Box((handle_opening_width, wall, handle_opening_bottom)),
        origin=Origin(xyz=(0.0, front_center_y, handle_opening_bottom * 0.5)),
        material=shell_black,
        name="body_front_lower_strip",
    )

    pocket_height = handle_opening_top - handle_opening_bottom + 0.008
    pocket_side_thickness = 0.018
    pocket_side_center_x = handle_opening_width * 0.5 - pocket_side_thickness * 0.5
    pocket_side_center_y = front_face_y - wall - pocket_depth * 0.5
    pocket_center_z = (handle_opening_bottom + handle_opening_top) * 0.5
    pocket_back_y = front_face_y - wall - pocket_depth + 0.004

    body.visual(
        Box((pocket_side_thickness, pocket_depth, pocket_height)),
        origin=Origin(xyz=(-pocket_side_center_x, pocket_side_center_y, pocket_center_z)),
        material=liner_gray,
        name="handle_pocket_left",
    )
    body.visual(
        Box((pocket_side_thickness, pocket_depth, pocket_height)),
        origin=Origin(xyz=(pocket_side_center_x, pocket_side_center_y, pocket_center_z)),
        material=liner_gray,
        name="handle_pocket_right",
    )
    body.visual(
        Box((handle_opening_width - 0.036, 0.008, pocket_height)),
        origin=Origin(xyz=(0.0, pocket_back_y, pocket_center_z)),
        material=liner_gray,
        name="handle_pocket_back",
    )
    body.visual(
        Box((0.020, 0.016, 0.022)),
        origin=Origin(xyz=(-0.092, pocket_side_center_y, pocket_center_z)),
        material=dark_steel,
        name="handle_mount_left",
    )
    body.visual(
        Box((0.020, 0.016, 0.022)),
        origin=Origin(xyz=(0.092, pocket_side_center_y, pocket_center_z)),
        material=dark_steel,
        name="handle_mount_right",
    )
    body.visual(
        Box((0.164, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, pocket_side_center_y, pocket_center_z)),
        material=rubber,
        name="handle_grip",
    )

    body.visual(
        Box((case_length - 0.08, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, front_face_y + 0.003, lower_height - 0.009)),
        material=aluminum,
        name="front_valance",
    )
    body.visual(
        Box((case_length - 0.08, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, back_face_y - 0.003, lower_height - 0.018)),
        material=aluminum,
        name="rear_valance",
    )
    body.visual(
        Box((0.006, case_width - 0.08, 0.018)),
        origin=Origin(xyz=(left_face_x + 0.003, 0.0, lower_height - 0.009)),
        material=aluminum,
        name="left_valance",
    )
    body.visual(
        Box((0.006, case_width - 0.08, 0.018)),
        origin=Origin(xyz=(right_face_x - 0.003, 0.0, lower_height - 0.009)),
        material=aluminum,
        name="right_valance",
    )

    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.47, -0.15),
            (-0.47, 0.15),
            (0.47, -0.15),
            (0.47, 0.15),
        ),
        start=1,
    ):
        body.visual(
            Box((0.070, 0.032, 0.010)),
            origin=Origin(xyz=(foot_x, foot_y, 0.005)),
            material=rubber,
            name=f"foot_{foot_index}",
        )

    body_hinge_segments = (
        ("body_hinge_left", -0.44, 0.16),
        ("body_hinge_center", 0.0, 0.16),
        ("body_hinge_right", 0.44, 0.16),
    )
    for hinge_name, hinge_x, hinge_len in body_hinge_segments:
        body.visual(
            Cylinder(radius=rear_hinge_radius, length=hinge_len),
            origin=Origin(
                xyz=(hinge_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=dark_steel,
            name=hinge_name,
        )
        body.visual(
            Box((hinge_len * 0.84, 0.012, 0.020)),
            origin=Origin(xyz=(hinge_x, hinge_axis_y - 0.008, lower_height + 0.004)),
            material=dark_steel,
            name=f"{hinge_name}_bracket",
        )

    for latch_index, latch_x in enumerate(latch_x_positions, start=1):
        body.visual(
            Box((0.070, 0.010, 0.018)),
            origin=Origin(xyz=(latch_x, front_face_y + 0.005, lower_height - 0.008)),
            material=dark_steel,
            name=f"latch_mount_{latch_index}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((case_length, case_width, lid_height)),
        mass=7.0,
        origin=Origin(
            xyz=lid_local(
                0.0,
                0.0,
                lower_height + lid_height * 0.5,
            )
        ),
    )

    lid.visual(
        Box((case_length - 2.0 * wall, case_width - 2.0 * wall, wall)),
        origin=Origin(
            xyz=lid_local(
                0.0,
                0.0,
                lower_height + lid_height - wall * 0.5,
            )
        ),
        material=shell_black,
        name="lid_top_panel",
    )
    lid.visual(
        Box((case_length, wall, lid_height)),
        origin=Origin(
            xyz=lid_local(
                0.0,
                front_face_y - wall * 0.5,
                lower_height + lid_height * 0.5,
            )
        ),
        material=shell_black,
        name="lid_front_shell",
    )
    lid.visual(
        Box((case_length, wall, lid_height)),
        origin=Origin(
            xyz=lid_local(
                0.0,
                back_face_y + wall * 0.5,
                lower_height + lid_height * 0.5,
            )
        ),
        material=shell_black,
        name="lid_back_shell",
    )
    lid.visual(
        Box((wall, case_width - 2.0 * wall, lid_height)),
        origin=Origin(
            xyz=lid_local(
                left_face_x + wall * 0.5,
                0.0,
                lower_height + lid_height * 0.5,
            )
        ),
        material=shell_black,
        name="lid_left_shell",
    )
    lid.visual(
        Box((wall, case_width - 2.0 * wall, lid_height)),
        origin=Origin(
            xyz=lid_local(
                right_face_x - wall * 0.5,
                0.0,
                lower_height + lid_height * 0.5,
            )
        ),
        material=shell_black,
        name="lid_right_shell",
    )

    lid.visual(
        Box((case_length - 0.06, 0.006, 0.018)),
        origin=Origin(
            xyz=lid_local(
                0.0,
                front_face_y + 0.003,
                lower_height + 0.012,
            )
        ),
        material=aluminum,
        name="lid_front_trim",
    )
    lid.visual(
        Box((0.006, case_width - 0.06, 0.018)),
        origin=Origin(
            xyz=lid_local(
                left_face_x + 0.003,
                0.0,
                lower_height + 0.012,
            )
        ),
        material=aluminum,
        name="lid_left_trim",
    )
    lid.visual(
        Box((0.006, case_width - 0.06, 0.018)),
        origin=Origin(
            xyz=lid_local(
                right_face_x - 0.003,
                0.0,
                lower_height + 0.012,
            )
        ),
        material=aluminum,
        name="lid_right_trim",
    )

    lid_hinge_segments = (
        ("lid_hinge_left", -0.22, 0.20),
        ("lid_hinge_right", 0.22, 0.20),
    )
    for hinge_name, hinge_x, hinge_len in lid_hinge_segments:
        lid.visual(
            Cylinder(radius=rear_hinge_radius, length=hinge_len),
            origin=Origin(
                xyz=lid_local(hinge_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=dark_steel,
            name=hinge_name,
        )
        lid.visual(
            Box((hinge_len * 0.84, 0.010, 0.022)),
            origin=Origin(
                xyz=lid_local(hinge_x, hinge_axis_y + 0.001, lower_height + 0.006)
            ),
            material=dark_steel,
            name=f"{hinge_name}_bracket",
        )

    for latch_index, latch_x in enumerate(latch_x_positions, start=1):
        lid.visual(
            Box((0.048, 0.010, 0.026)),
            origin=Origin(
                xyz=lid_local(
                    latch_x,
                    front_face_y + 0.005,
                    lower_height + 0.014,
                )
            ),
            material=aluminum,
            name=f"lid_striker_{latch_index}",
        )

    lid_joint = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    for latch_index, latch_x in enumerate(latch_x_positions, start=1):
        latch = model.part(f"latch_{latch_index}")
        latch.inertial = Inertial.from_geometry(
            Box((0.060, 0.020, 0.100)),
            mass=0.24,
            origin=Origin(xyz=(0.0, 0.010, -0.050)),
        )
        latch.visual(
            Box((0.056, 0.008, 0.088)),
            origin=Origin(xyz=(0.0, 0.004, -0.044)),
            material=dark_steel,
            name="blade",
        )
        latch.visual(
            Box((0.040, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.007, -0.083)),
            material=dark_steel,
            name="pull_tab",
        )
        latch.visual(
            Box((0.044, 0.006, 0.016)),
            origin=Origin(xyz=(0.0, 0.003, 0.008)),
            material=aluminum,
            name="face_plate",
        )
        model.articulation(
            f"latch_{latch_index}_pivot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(latch_x, latch_mount_front_y, latch_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.20),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_parts = [object_model.get_part(f"latch_{index}") for index in range(1, 5)]
    latch_joints = [object_model.get_articulation(f"latch_{index}_pivot") for index in range(1, 5)]

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
        "lid_hinge_axis_aligned_with_case_length",
        lid_hinge.axis == (1.0, 0.0, 0.0),
        details=f"lid hinge axis was {lid_hinge.axis}",
    )
    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge_has_realistic_open_range",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper >= 1.2,
        details=f"lid hinge limits were {lid_limits}",
    )

    for latch_index, latch_joint in enumerate(latch_joints, start=1):
        ctx.check(
            f"latch_{latch_index}_axis_aligned_with_case_length",
            latch_joint.axis == (1.0, 0.0, 0.0),
            details=f"latch axis was {latch_joint.axis}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_shell",
            negative_elem="body_front_left_segment",
            min_gap=0.0,
            max_gap=0.001,
            name="lid_front_seam_sits_on_lower_shell",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="lid_back_shell",
            elem_b="body_back_shell",
            name="lid_remains_captured_at_rear_hinge_line",
        )
        for latch_index, latch in enumerate(latch_parts, start=1):
            ctx.expect_gap(
                latch,
                body,
                axis="y",
                positive_elem="blade",
                negative_elem=f"latch_mount_{latch_index}",
                min_gap=0.0,
                max_gap=0.001,
                name=f"latch_{latch_index}_sits_on_front_mount",
            )

    with ctx.pose({lid_hinge: 1.10}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_shell",
            negative_elem="body_front_left_segment",
            min_gap=0.20,
            name="lid_front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
