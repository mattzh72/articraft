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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="package_delivery_hand_truck")

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def _add_wheel_visuals(
        part,
        prefix: str,
        *,
        tire_radius: float,
        tire_width: float,
    ) -> None:
        tire_half = tire_width * 0.5
        tire_profile = [
            (tire_radius * 0.50, -tire_half * 0.94),
            (tire_radius * 0.71, -tire_half * 0.98),
            (tire_radius * 0.90, -tire_half * 0.82),
            (tire_radius * 0.98, -tire_half * 0.42),
            (tire_radius, 0.0),
            (tire_radius * 0.98, tire_half * 0.42),
            (tire_radius * 0.90, tire_half * 0.82),
            (tire_radius * 0.71, tire_half * 0.98),
            (tire_radius * 0.50, tire_half * 0.94),
            (tire_radius * 0.42, tire_half * 0.28),
            (tire_radius * 0.39, 0.0),
            (tire_radius * 0.42, -tire_half * 0.28),
            (tire_radius * 0.50, -tire_half * 0.94),
        ]
        part.visual(
            _mesh(
                f"{prefix}_tire",
                LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0),
            ),
            material=rubber,
            name="tire",
        )

        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
        part.visual(
            Cylinder(radius=tire_radius * 0.67, length=tire_width * 0.76),
            origin=spin_origin,
            material=steel,
            name="rim",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.46, length=tire_width * 0.82),
            origin=spin_origin,
            material=dark_steel,
            name="hub",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.24, length=tire_width * 0.70),
            origin=spin_origin,
            material=steel,
            name="cap",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.55, length=tire_width * 0.08),
            origin=Origin(xyz=(tire_width * 0.22, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.55, length=tire_width * 0.08),
            origin=Origin(xyz=(-tire_width * 0.22, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 1.12)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.03, 0.56)),
    )

    rail_radius = 0.017
    frame_half_width = 0.145
    axle_y = 0.02
    axle_z = 0.155
    hinge_y = 0.0
    hinge_z = 1.055

    frame.visual(
        Cylinder(radius=rail_radius, length=0.88),
        origin=Origin(xyz=(-frame_half_width, 0.0, 0.62)),
        material=frame_paint,
        name="left_upright",
    )
    frame.visual(
        Cylinder(radius=rail_radius, length=0.88),
        origin=Origin(xyz=(frame_half_width, 0.0, 0.62)),
        material=frame_paint,
        name="right_upright",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.324),
        origin=Origin(xyz=(0.0, 0.0, 0.800), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="top_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.312),
        origin=Origin(xyz=(0.0, 0.0, 0.63), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="mid_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.318),
        origin=Origin(xyz=(0.0, 0.0, 0.36), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="lower_crossbar",
    )
    frame.visual(
        Box((0.030, 0.050, 0.090)),
        origin=Origin(xyz=(-frame_half_width, 0.008, 0.145)),
        material=frame_paint,
        name="left_lower_stanchion",
    )
    frame.visual(
        Box((0.030, 0.050, 0.090)),
        origin=Origin(xyz=(frame_half_width, 0.008, 0.145)),
        material=frame_paint,
        name="right_lower_stanchion",
    )
    frame.visual(
        Box((0.34, 0.24, 0.008)),
        origin=Origin(xyz=(0.0, -0.125, 0.024)),
        material=frame_paint,
        name="nose_plate",
    )
    frame.visual(
        Box((0.34, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.239, 0.015)),
        material=frame_paint,
        name="nose_lip",
    )
    frame.visual(
        Box((0.18, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, -0.016, 0.046)),
        material=frame_paint,
        name="nose_backer",
    )

    for mesh_name, points in [
        (
            "left_nose_brace",
            [(-0.118, axle_y, axle_z), (-0.120, -0.055, 0.106), (-0.120, -0.175, 0.050)],
        ),
        (
            "right_nose_brace",
            _mirror_x([(-0.118, axle_y, axle_z), (-0.120, -0.055, 0.106), (-0.120, -0.175, 0.050)]),
        ),
    ]:
        frame.visual(
            _mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=0.012,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
        )
    frame.visual(
        Box((0.022, 0.180, 0.140)),
        origin=Origin(xyz=(-0.120, -0.105, 0.085)),
        material=frame_paint,
        name="left_toe_gusset",
    )
    frame.visual(
        Box((0.022, 0.180, 0.140)),
        origin=Origin(xyz=(0.120, -0.105, 0.085)),
        material=frame_paint,
        name="right_toe_gusset",
    )

    frame.visual(
        Cylinder(radius=0.014, length=0.344),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_beam",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.172, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_spacer",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.172, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_spacer",
    )

    for side in (-1.0, 1.0):
        sign_name = "left" if side < 0 else "right"
        frame.visual(
            Box((0.020, 0.020, 0.060)),
            origin=Origin(xyz=(side * 0.166, 0.002, 1.026)),
            material=dark_steel,
            name=f"{sign_name}_hinge_tab",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.024),
            origin=Origin(xyz=(side * 0.188, 0.0, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"{sign_name}_pivot_pin",
        )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.052),
        mass=3.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        "left_wheel",
        tire_radius=0.155,
        tire_width=0.052,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.052),
        mass=3.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        "right_wheel",
        tire_radius=0.155,
        tire_width=0.052,
    )

    upper_handle = model.part("upper_handle")
    upper_handle.inertial = Inertial.from_geometry(
        Box((0.40, 0.14, 0.14)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.050, 0.060)),
    )
    upper_handle.visual(
        Cylinder(radius=0.0085, length=0.022),
        origin=Origin(xyz=(-0.188, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_hinge_barrel",
    )
    upper_handle.visual(
        Cylinder(radius=0.0085, length=0.022),
        origin=Origin(xyz=(0.188, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_hinge_barrel",
    )
    upper_handle.visual(
        _mesh(
            "upper_handle_left_link",
            tube_from_spline_points(
                [
                    (-0.188, 0.010, 0.0),
                    (-0.182, 0.022, 0.006),
                    (-0.168, 0.036, 0.018),
                ],
                radius=0.005,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=steel,
        name="left_hinge_link",
    )
    upper_handle.visual(
        _mesh(
            "upper_handle_right_link",
            tube_from_spline_points(
                [
                    (0.188, 0.010, 0.0),
                    (0.182, 0.022, 0.006),
                    (0.168, 0.036, 0.018),
                ],
                radius=0.005,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=steel,
        name="right_hinge_link",
    )
    upper_handle.visual(
        _mesh(
            "upper_handle_loop",
            wire_from_points(
                [
                    (-0.168, 0.036, 0.018),
                    (-0.160, 0.052, 0.046),
                    (-0.060, 0.056, 0.094),
                    (0.060, 0.056, 0.094),
                    (0.160, 0.052, 0.046),
                    (0.168, 0.036, 0.018),
                ],
                radius=0.008,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.026,
                corner_segments=10,
            ),
        ),
        material=steel,
        name="handle_loop",
    )
    upper_handle.visual(
        Cylinder(radius=0.013, length=0.116),
        origin=Origin(xyz=(0.0, 0.056, 0.094), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="grip_sleeve",
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.195, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(0.195, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "upper_handle_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_handle,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.60, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    upper_handle = object_model.get_part("upper_handle")
    upper_handle_fold = object_model.get_articulation("upper_handle_fold")

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
    ctx.allow_overlap(
        frame,
        upper_handle,
        elem_a="left_pivot_pin",
        elem_b="left_hinge_barrel",
        reason="Real hinge pin passes through the rotating left barrel.",
    )
    ctx.allow_overlap(
        frame,
        upper_handle,
        elem_a="right_pivot_pin",
        elem_b="right_hinge_barrel",
        reason="Real hinge pin passes through the rotating right barrel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_wheel, frame, name="left_wheel_contacts_axle")
    ctx.expect_contact(right_wheel, frame, name="right_wheel_contacts_axle")
    ctx.expect_contact(
        frame,
        upper_handle,
        elem_a="left_pivot_pin",
        elem_b="left_hinge_barrel",
        name="left_handle_pin_contact",
    )
    ctx.expect_contact(
        frame,
        upper_handle,
        elem_a="right_pivot_pin",
        elem_b="right_hinge_barrel",
        name="right_handle_pin_contact",
    )
    handle_pos = ctx.part_world_position(upper_handle)
    if handle_pos is not None:
        ctx.check(
            "upper_handle_pivots_at_frame_top",
            abs(handle_pos[0]) < 1e-6 and abs(handle_pos[1] - 0.0) < 1e-6 and abs(handle_pos[2] - 1.055) < 1e-6,
            f"handle origin={handle_pos}",
        )

    frame_aabb = ctx.part_world_aabb(frame)
    handle_aabb = ctx.part_world_aabb(upper_handle)
    if frame_aabb is not None and handle_aabb is not None:
        ctx.check(
            "open_handle_extends_above_frame",
            handle_aabb[1][2] > frame_aabb[1][2] + 0.08,
            f"handle top z={handle_aabb[1][2]:.3f}, frame top z={frame_aabb[1][2]:.3f}",
        )

    with ctx.pose({upper_handle_fold: -1.20}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlaps")
        ctx.expect_overlap(
            upper_handle,
            frame,
            axes="x",
            min_overlap=0.25,
            name="folded_handle_tracks_frame_width",
        )
        folded_aabb = ctx.part_world_aabb(upper_handle)
        folded_pos = ctx.part_world_position(upper_handle)
        if folded_pos is not None:
            ctx.check(
                "folded_handle_keeps_same_pivot_origin",
                abs(folded_pos[0]) < 1e-6 and abs(folded_pos[1] - 0.0) < 1e-6 and abs(folded_pos[2] - 1.055) < 1e-6,
                f"folded handle origin={folded_pos}",
            )
        if frame_aabb is not None and folded_aabb is not None:
            ctx.check(
                "folded_handle_stows_below_frame_top",
                folded_aabb[1][2] < frame_aabb[1][2] + 0.03,
                f"folded handle top z={folded_aabb[1][2]:.3f}, frame top z={frame_aabb[1][2]:.3f}",
            )
            ctx.check(
                "folded_handle_spans_upper_frame_zone",
                folded_aabb[0][2] > frame_aabb[1][2] - 0.16,
                f"folded handle bottom z={folded_aabb[0][2]:.3f}, frame top z={frame_aabb[1][2]:.3f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
