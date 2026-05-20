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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter_panel")

    painted_wood = model.material("painted_wood", rgba=(0.95, 0.95, 0.92, 1.0))
    painted_shadow = model.material("painted_shadow", rgba=(0.86, 0.87, 0.84, 1.0))
    tilt_rod_paint = model.material("tilt_rod_paint", rgba=(0.91, 0.92, 0.88, 1.0))
    hardware = model.material("hardware", rgba=(0.76, 0.78, 0.80, 1.0))

    panel_width = 0.66
    panel_height = 1.42
    panel_thickness = 0.034
    stile_width = 0.075
    rail_height = 0.09
    opening_width = panel_width - 2.0 * stile_width
    opening_half_width = opening_width * 0.5
    opening_bottom = rail_height
    opening_top = panel_height - rail_height
    opening_height = opening_top - opening_bottom

    louver_count = 9
    louver_bottom_clear = 0.11
    louver_pitch = (opening_height - 2.0 * louver_bottom_clear) / (louver_count - 1)
    louver_depth = 0.068
    louver_thickness = 0.011
    louver_span = 0.456
    pivot_radius = 0.0035
    pivot_pin_length = 0.018
    pivot_line_y = -0.008
    pivot_pin_center_x = louver_span * 0.5 + pivot_pin_length * 0.5
    pivot_pin_outer_face_x = louver_span * 0.5 + pivot_pin_length
    pivot_cap_length = 0.008
    pivot_cap_radius = 0.0052
    pivot_cap_center_x = pivot_pin_outer_face_x + pivot_cap_length * 0.5

    tilt_rod_x = opening_half_width - 0.080
    tilt_rod_y = 0.032
    tilt_rod_width = 0.014
    tilt_rod_depth = 0.005
    tilt_rod_length = 1.19
    tilt_rod_center_z = panel_height * 0.5
    tilt_rod_travel = 0.025

    louver_profile = superellipse_profile(
        louver_thickness,
        louver_depth,
        exponent=2.1,
        segments=40,
    )
    louver_geometry = ExtrudeGeometry.centered(louver_profile, louver_span).rotate_y(pi / 2.0)
    louver_mesh = _save_mesh("plantation_shutter_louver", louver_geometry)

    rod_profile = rounded_rect_profile(
        tilt_rod_width,
        tilt_rod_depth,
        radius=0.0015,
        corner_segments=6,
    )
    rod_mesh = _save_mesh(
        "plantation_shutter_tilt_rod",
        ExtrudeGeometry.centered(rod_profile, tilt_rod_length),
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness, panel_height)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
    )
    frame.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(-(panel_width * 0.5 - stile_width * 0.5), 0.0, panel_height * 0.5)),
        material=painted_wood,
        name="left_stile",
    )
    frame.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=((panel_width * 0.5 - stile_width * 0.5), 0.0, panel_height * 0.5)),
        material=painted_wood,
        name="right_stile",
    )
    frame.visual(
        Box((opening_width, panel_thickness, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, panel_height - rail_height * 0.5)),
        material=painted_wood,
        name="top_rail",
    )
    frame.visual(
        Box((opening_width, panel_thickness, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_height * 0.5)),
        material=painted_wood,
        name="bottom_rail",
    )

    frame.visual(
        Box((opening_width, 0.009, 0.016)),
        origin=Origin(xyz=(0.0, 0.009, panel_height - rail_height + 0.012)),
        material=painted_shadow,
        name="top_inner_stop",
    )
    frame.visual(
        Box((opening_width, 0.009, 0.016)),
        origin=Origin(xyz=(0.0, 0.009, rail_height - 0.012)),
        material=painted_shadow,
        name="bottom_inner_stop",
    )

    louver_centers: list[float] = []
    for index in range(louver_count):
        z = opening_bottom + louver_bottom_clear + index * louver_pitch
        louver_centers.append(z)
        for sign, label in ((-1.0, "left"), (1.0, "right")):
            frame.visual(
                Cylinder(radius=pivot_cap_radius, length=pivot_cap_length),
                origin=Origin(
                    xyz=(sign * pivot_cap_center_x, pivot_line_y, z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=painted_shadow,
                name=f"{label}_pivot_cap_{index + 1:02d}",
            )
            frame.visual(
                Box((0.005, 0.012, 0.012)),
                origin=Origin(xyz=(sign * (opening_half_width + 0.0005), pivot_line_y, z)),
                material=painted_shadow,
                name=f"{label}_pivot_bridge_{index + 1:02d}",
            )

    for guide_label, guide_z in (("bottom", 0.105), ("top", panel_height - 0.105)):
        frame.visual(
            Box((0.032, 0.013, 0.030)),
            origin=Origin(xyz=(tilt_rod_x, 0.0205, guide_z)),
            material=painted_shadow,
            name=f"{guide_label}_rod_guide_mount",
        )
        for side_sign, side_label in ((-1.0, "left"), (1.0, "right")):
            frame.visual(
                Box((0.004, 0.012, 0.050)),
                origin=Origin(
                    xyz=(
                        tilt_rod_x + side_sign * (tilt_rod_width * 0.5 + 0.002),
                        tilt_rod_y,
                        guide_z,
                    )
                ),
                material=painted_shadow,
                name=f"{guide_label}_rod_guide_{side_label}",
            )

    tilt_rod = model.part("tilt_rod")
    tilt_rod.visual(
        rod_mesh,
        material=tilt_rod_paint,
        name="rod_body",
    )
    for index, z in enumerate(louver_centers):
        tilt_rod.visual(
            Cylinder(radius=0.0048, length=0.006),
            origin=Origin(
                xyz=(0.0, tilt_rod_depth * 0.5 + 0.001, z - tilt_rod_center_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"link_button_{index + 1:02d}",
        )
    tilt_rod.inertial = Inertial.from_geometry(
        Box((tilt_rod_width, tilt_rod_depth, tilt_rod_length)),
        mass=0.5,
    )

    model.articulation(
        "frame_to_tilt_rod",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tilt_rod,
        origin=Origin(xyz=(tilt_rod_x, tilt_rod_y, tilt_rod_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=0.10,
            lower=-tilt_rod_travel,
            upper=tilt_rod_travel,
        ),
    )

    slot_clear_width = 0.018
    slot_side_thickness = 0.002
    slot_depth = 0.010
    slot_height = 0.055
    slot_side_y = louver_depth * 0.5 + slot_depth * 0.5
    slot_front_y = louver_depth * 0.5 + slot_depth + 0.001

    for index, z in enumerate(louver_centers):
        louver_name = f"louver_{index + 1:02d}"
        louver = model.part(louver_name)
        louver.visual(louver_mesh, material=painted_wood, name="blade")
        louver.visual(
            Cylinder(radius=pivot_radius, length=pivot_pin_length),
            origin=Origin(xyz=(-pivot_pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="left_pivot_pin",
        )
        louver.visual(
            Cylinder(radius=pivot_radius, length=pivot_pin_length),
            origin=Origin(xyz=(pivot_pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="right_pivot_pin",
        )
        louver.visual(
            Box((slot_side_thickness, slot_depth, slot_height)),
            origin=Origin(
                xyz=(
                    tilt_rod_x - (slot_clear_width * 0.5 + slot_side_thickness * 0.5),
                    slot_side_y,
                    0.0,
                )
            ),
            material=hardware,
            name="tilt_link_left_cheek",
        )
        louver.visual(
            Box((slot_side_thickness, slot_depth, slot_height)),
            origin=Origin(
                xyz=(
                    tilt_rod_x + (slot_clear_width * 0.5 + slot_side_thickness * 0.5),
                    slot_side_y,
                    0.0,
                )
            ),
            material=hardware,
            name="tilt_link_right_cheek",
        )
        louver.visual(
            Box((slot_clear_width + 2.0 * slot_side_thickness, 0.002, slot_height)),
            origin=Origin(xyz=(tilt_rod_x, slot_front_y, 0.0)),
            material=hardware,
            name="tilt_link_bridge",
        )
        louver.inertial = Inertial.from_geometry(
            Box((louver_span + 2.0 * pivot_pin_length, louver_depth + 0.012, 0.060)),
            mass=0.55,
        )

        model.articulation(
            f"frame_to_{louver_name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, pivot_line_y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=1.4,
                lower=-0.75,
                upper=0.75,
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

    frame = object_model.get_part("frame")
    tilt_rod = object_model.get_part("tilt_rod")
    tilt_rod_joint = object_model.get_articulation("frame_to_tilt_rod")
    louver_parts = [object_model.get_part(f"louver_{index + 1:02d}") for index in range(9)]
    louver_joints = [object_model.get_articulation(f"frame_to_louver_{index + 1:02d}") for index in range(9)]

    ctx.check(
        "tilt_rod_axis_is_vertical",
        tuple(tilt_rod_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical prismatic axis, got {tilt_rod_joint.axis}.",
    )
    for index, joint in enumerate(louver_joints):
        ctx.check(
            f"louver_{index + 1:02d}_axis_is_horizontal",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"Expected x-axis louver pivot, got {joint.axis}.",
        )

    ctx.expect_contact(tilt_rod, frame, name="tilt_rod_guided_by_frame")
    for index, louver in enumerate(louver_parts):
        ctx.expect_contact(louver, frame, name=f"louver_{index + 1:02d}_captured_in_frame")

    rod_rest = ctx.part_world_position(tilt_rod)
    assert rod_rest is not None

    open_pose = {tilt_rod_joint: 0.022}
    open_pose.update({joint: 0.45 for joint in louver_joints})
    with ctx.pose(open_pose):
        rod_open = ctx.part_world_position(tilt_rod)
        assert rod_open is not None
        ctx.check(
            "tilt_rod_moves_upward",
            rod_open[2] > rod_rest[2] + 0.015,
            details=f"Expected upward travel from {rod_rest} to {rod_open}.",
        )
        ctx.expect_contact(tilt_rod, frame, name="tilt_rod_guided_when_open")
        for index, louver in enumerate(louver_parts):
            ctx.expect_contact(louver, frame, name=f"louver_{index + 1:02d}_captured_when_open")
        for index in range(len(louver_parts) - 1):
            ctx.expect_gap(
                louver_parts[index + 1],
                louver_parts[index],
                axis="z",
                min_gap=0.030,
                name=f"open_gap_between_louver_{index + 1:02d}_and_{index + 2:02d}",
            )

    closed_pose = {tilt_rod_joint: -0.022}
    closed_pose.update({joint: -0.45 for joint in louver_joints})
    with ctx.pose(closed_pose):
        rod_closed = ctx.part_world_position(tilt_rod)
        assert rod_closed is not None
        ctx.check(
            "tilt_rod_moves_downward",
            rod_closed[2] < rod_rest[2] - 0.015,
            details=f"Expected downward travel from {rod_rest} to {rod_closed}.",
        )
        ctx.expect_contact(tilt_rod, frame, name="tilt_rod_guided_when_closed")
        ctx.expect_contact(louver_parts[0], frame, name="bottom_louver_captured_when_closed")
        ctx.expect_contact(louver_parts[-1], frame, name="top_louver_captured_when_closed")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
