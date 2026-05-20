from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rolling_walker")

    tube_finish = model.material("tube_finish", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    rim_gray = model.material("rim_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    accent = model.material("accent", rgba=(0.31, 0.55, 0.82, 1.0))

    tube_radius = 0.014

    def _tube(points, radius: float = tube_radius, samples: int = 8, radial: int = 16):
        return tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial,
            cap_ends=True,
        )

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _box_geom(size: tuple[float, float, float], center: tuple[float, float, float]):
        return BoxGeometry(size).translate(*center)

    def _cyl_x(radius: float, length: float, center: tuple[float, float, float]):
        return CylinderGeometry(radius, length).rotate_y(pi / 2.0).translate(*center)

    def _cyl_y(radius: float, length: float, center: tuple[float, float, float]):
        return CylinderGeometry(radius, length).rotate_x(pi / 2.0).translate(*center)

    def _build_center_hub():
        geom = _cyl_x(0.022, 0.38, (0.0, 0.0, 0.48))
        geom.merge(CylinderGeometry(0.020, 0.20).translate(0.0, 0.0, 0.58))
        geom.merge(_cyl_y(0.015, 0.20, (0.0, 0.0, 0.42)))
        geom.merge(_box_geom((0.10, 0.12, 0.05), (0.0, 0.0, 0.39)))
        geom.merge(_box_geom((0.05, 0.14, 0.025), (0.0, 0.0, 0.67)))
        geom.merge(
            _tube(
                [
                    (0.0, 0.05, 0.665),
                    (0.0, 0.0, 0.705),
                    (0.0, -0.05, 0.665),
                ],
                radius=0.007,
                samples=10,
                radial=14,
            )
        )
        geom.merge(_box_geom((0.13, 0.06, 0.035), (0.0, 0.0, 0.515)))
        return geom

    def _build_side_frame(sign: float):
        root_block = (0.055 * sign, 0.0, 0.0)
        front_socket = (0.070 * sign, 0.180, 0.228)
        rear_leg_low = (0.050 * sign, -0.205, -0.410)
        rear_leg_mid = (0.050 * sign, -0.205, 0.180)
        front_top = (0.064 * sign, 0.155, 0.785)
        rear_top = (0.064 * sign, -0.165, 0.815)

        geom = _box_geom((0.010, 0.070, 0.046), root_block)
        geom.merge(
            _tube(
                [
                    (0.055 * sign, 0.028, 0.0),
                    (0.060 * sign, 0.100, 0.095),
                    front_socket,
                ],
                samples=8,
            )
        )
        geom.merge(
            _tube(
                [
                    (0.055 * sign, -0.028, 0.0),
                    (0.056 * sign, -0.110, 0.100),
                    rear_leg_mid,
                ],
                samples=8,
            )
        )
        geom.merge(
            _tube(
                [
                    front_socket,
                    (0.062 * sign, 0.170, 0.500),
                    front_top,
                ],
                radius=0.013,
                samples=8,
            )
        )
        geom.merge(
            _tube(
                [
                    rear_leg_low,
                    rear_leg_mid,
                    rear_top,
                ],
                radius=0.012,
                samples=6,
            )
        )
        geom.merge(
            _tube(
                [
                    front_top,
                    (0.072 * sign, -0.030, 0.845),
                    rear_top,
                ],
                samples=10,
            )
        )
        geom.merge(
            _tube(
                [
                    (0.057 * sign, 0.010, 0.010),
                    (0.060 * sign, -0.085, 0.250),
                    (0.061 * sign, -0.145, 0.530),
                ],
                radius=0.010,
                samples=8,
            )
        )
        geom.merge(_box_geom((0.028, 0.032, 0.026), front_socket))
        geom.merge(_box_geom((0.026, 0.034, 0.036), (0.064 * sign, 0.160, 0.770)))
        geom.merge(_box_geom((0.030, 0.038, 0.040), (0.064 * sign, -0.165, 0.805)))
        geom.merge(_cyl_x(0.005, 0.016, (0.060 * sign, -0.205, -0.410)))
        return geom

    center_hub = model.part("center_hub")
    center_hub.visual(
        Box((0.088, 0.100, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=dark_trim,
        name="hub_body",
    )
    center_hub.visual(
        Cylinder(radius=0.024, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.48), rpy=(0.0, pi / 2.0, 0.0)),
        material=tube_finish,
        name="slider_sleeve",
    )
    center_hub.visual(
        Cylinder(radius=0.020, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=tube_finish,
        name="upright_post",
    )
    center_hub.visual(
        Cylinder(radius=0.015, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.455), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tube_finish,
        name="lower_cross_tube",
    )
    center_hub.visual(
        Box((0.050, 0.120, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.652)),
        material=tube_finish,
        name="top_cap",
    )
    center_hub.visual(
        Box((0.060, 0.082, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_trim,
        name="hub_block",
    )
    center_hub.visual(
        Box((0.082, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.458)),
        material=accent,
        name="fold_release_plate",
    )
    center_hub.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.28)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    for side_name, sign in (("left_side_frame", 1.0), ("right_side_frame", -1.0)):
        side_frame = model.part(side_name)
        side_frame.visual(
            _mesh(
                f"{side_name}_front_lower",
                _tube(
                    [
                        (0.052 * sign, 0.0, -0.002),
                        (0.058 * sign, 0.060, -0.082),
                        (0.064 * sign, 0.120, -0.165),
                        (0.070 * sign, 0.180, -0.245),
                    ],
                    radius=0.011,
                    samples=8,
                ),
            ),
            material=tube_finish,
            name="front_lower_rail",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_rear_lower",
                _tube(
                    [
                        (0.052 * sign, 0.0, -0.002),
                        (0.054 * sign, -0.070, -0.100),
                        (0.048 * sign, -0.150, -0.240),
                        (0.050 * sign, -0.205, -0.395),
                    ],
                    radius=0.011,
                    samples=8,
                ),
            ),
            material=tube_finish,
            name="rear_lower_rail",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_front_upright",
                _tube(
                    [
                        (0.070 * sign, 0.180, -0.245),
                        (0.072 * sign, 0.165, -0.020),
                        (0.072 * sign, 0.155, 0.230),
                    ],
                    radius=0.013,
                    samples=8,
                ),
            ),
            material=tube_finish,
            name="front_upright",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_rear_upright",
                _tube(
                    [
                        (0.050 * sign, -0.205, -0.395),
                        (0.056 * sign, -0.185, -0.020),
                        (0.064 * sign, -0.170, 0.200),
                        (0.070 * sign, -0.160, 0.355),
                    ],
                    radius=0.012,
                    samples=8,
                ),
            ),
            material=tube_finish,
            name="rear_upright",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_top_rail",
                _tube(
                    [
                        (0.072 * sign, 0.155, 0.230),
                        (0.082 * sign, -0.020, 0.332),
                        (0.070 * sign, -0.160, 0.355),
                    ],
                    radius=0.011,
                    samples=10,
                ),
            ),
            material=tube_finish,
            name="top_rail",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_brace",
                _tube(
                    [
                        (0.054 * sign, 0.010, -0.002),
                        (0.060 * sign, -0.090, 0.110),
                        (0.066 * sign, -0.145, 0.210),
                    ],
                    radius=0.009,
                    samples=8,
                ),
            ),
            material=tube_finish,
            name="diagonal_brace",
        )
        side_frame.visual(
            Box((0.060, 0.060, 0.040)),
            origin=Origin(xyz=(0.026 * sign, 0.0, -0.004)),
            material=dark_trim,
            name="root_block",
        )
        side_frame.visual(
            Box((0.028, 0.028, 0.020)),
            origin=Origin(xyz=(0.070 * sign, 0.180, -0.255)),
            material=dark_trim,
            name="front_socket",
        )
        side_frame.visual(
            Box((0.026, 0.032, 0.032)),
            origin=Origin(xyz=(0.055 * sign, -0.205, -0.410)),
            material=dark_trim,
            name="rear_axle_block",
        )
        side_frame.visual(
            Cylinder(radius=0.016, length=0.10),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_trim,
            name="slider_stub",
        )
        side_frame.visual(
            _mesh(
                f"{side_name}_grip_neck",
                _tube(
                    [
                        (0.070 * sign, -0.160, 0.355),
                        (0.070 * sign, -0.160, 0.390),
                    ],
                    radius=0.011,
                    samples=4,
                ),
            ),
            material=tube_finish,
            name="grip_neck",
        )
        side_frame.visual(
            Cylinder(radius=0.021, length=0.120),
            origin=Origin(xyz=(0.070 * sign, -0.160, 0.410), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name="hand_grip",
        )
        side_frame.visual(
            Box((0.018, 0.032, 0.020)),
            origin=Origin(xyz=(0.070 * sign, -0.108, 0.356)),
            material=dark_trim,
            name="brake_lever_blade",
        )
        side_frame.inertial = Inertial.from_geometry(
            Box((0.18, 0.50, 0.84)),
            mass=2.7,
            origin=Origin(xyz=(0.04 * sign, -0.01, 0.00)),
        )

    for fork_name in ("front_left_caster_fork", "front_right_caster_fork"):
        fork = model.part(fork_name)
        fork.visual(
            Cylinder(radius=0.010, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, -0.045)),
            material=dark_trim,
            name="caster_stem",
        )
        fork.visual(
            Box((0.040, 0.028, 0.016)),
            origin=Origin(xyz=(0.0, -0.018, -0.088)),
            material=dark_trim,
            name="fork_crown",
        )
        fork.visual(
            Box((0.004, 0.022, 0.074)),
            origin=Origin(xyz=(0.022, -0.030, -0.126)),
            material=dark_trim,
            name="outer_leg",
        )
        fork.visual(
            Box((0.004, 0.022, 0.074)),
            origin=Origin(xyz=(-0.022, -0.030, -0.126)),
            material=dark_trim,
            name="inner_leg",
        )
        fork.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(0.019, -0.030, -0.160), rpy=(0.0, pi / 2.0, 0.0)),
            material=rim_gray,
            name="outer_axle_pin",
        )
        fork.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(-0.019, -0.030, -0.160), rpy=(0.0, pi / 2.0, 0.0)),
            material=rim_gray,
            name="inner_axle_pin",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 0.18)),
            mass=0.35,
            origin=Origin(xyz=(0.0, -0.020, -0.090)),
        )

    for wheel_name, radius, width in (
        ("front_left_caster_wheel", 0.058, 0.025),
        ("front_right_caster_wheel", 0.058, 0.025),
        ("left_rear_wheel", 0.070, 0.030),
        ("right_rear_wheel", 0.070, 0.030),
    ):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=radius, length=width),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.73, length=width * 1.08),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rim_gray,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.24, length=width * 1.20),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_trim,
            name="hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=width),
            mass=0.60 if "front" in wheel_name else 0.82,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )

    left_frame = model.get_part("left_side_frame")
    right_frame = model.get_part("right_side_frame")
    front_left_fork = model.get_part("front_left_caster_fork")
    front_right_fork = model.get_part("front_right_caster_fork")
    front_left_wheel = model.get_part("front_left_caster_wheel")
    front_right_wheel = model.get_part("front_right_caster_wheel")
    left_rear_wheel = model.get_part("left_rear_wheel")
    right_rear_wheel = model.get_part("right_rear_wheel")

    model.articulation(
        "left_frame_slide",
        ArticulationType.PRISMATIC,
        parent=center_hub,
        child=left_frame,
        origin=Origin(xyz=(0.180, 0.0, 0.48)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.08),
    )
    model.articulation(
        "right_frame_slide",
        ArticulationType.PRISMATIC,
        parent=center_hub,
        child=right_frame,
        origin=Origin(xyz=(-0.180, 0.0, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.08),
    )

    model.articulation(
        "front_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=left_frame,
        child=front_left_fork,
        origin=Origin(xyz=(0.070, 0.180, -0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "front_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=front_right_fork,
        origin=Origin(xyz=(-0.070, 0.180, -0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.086, -0.205, -0.410)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=16.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.086, -0.205, -0.410)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_hub = object_model.get_part("center_hub")
    left_frame = object_model.get_part("left_side_frame")
    right_frame = object_model.get_part("right_side_frame")
    front_left_fork = object_model.get_part("front_left_caster_fork")
    front_right_fork = object_model.get_part("front_right_caster_fork")
    front_left_wheel = object_model.get_part("front_left_caster_wheel")
    front_right_wheel = object_model.get_part("front_right_caster_wheel")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")

    left_slide = object_model.get_articulation("left_frame_slide")
    right_slide = object_model.get_articulation("right_frame_slide")
    left_swivel = object_model.get_articulation("front_left_caster_swivel")
    right_swivel = object_model.get_articulation("front_right_caster_swivel")
    left_caster_spin = object_model.get_articulation("front_left_wheel_spin")
    right_caster_spin = object_model.get_articulation("front_right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        center_hub,
        left_frame,
        reason="Left telescoping side rail intentionally nests inside the central sleeve during width collapse.",
        elem_a="slider_sleeve",
        elem_b="slider_stub",
    )
    ctx.allow_overlap(
        center_hub,
        left_frame,
        reason="Left fold carriage root block is intentionally shrouded by the central sleeve at the fully folded storage limit.",
        elem_a="slider_sleeve",
        elem_b="root_block",
    )
    ctx.allow_overlap(
        center_hub,
        right_frame,
        reason="Right telescoping side rail intentionally nests inside the central sleeve during width collapse.",
        elem_a="slider_sleeve",
        elem_b="slider_stub",
    )
    ctx.allow_overlap(
        center_hub,
        right_frame,
        reason="Right fold carriage root block is intentionally shrouded by the central sleeve at the fully folded storage limit.",
        elem_a="slider_sleeve",
        elem_b="root_block",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_axes_drive_inward_collapse",
        tuple(left_slide.axis) == (-1.0, 0.0, 0.0) and tuple(right_slide.axis) == (1.0, 0.0, 0.0),
        details=f"Unexpected slide axes: left={left_slide.axis}, right={right_slide.axis}",
    )
    ctx.check(
        "front_caster_axes_are_plausible",
        tuple(left_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(right_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(left_caster_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_caster_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"Swivel axes=({left_swivel.axis}, {right_swivel.axis}), "
            f"wheel spin axes=({left_caster_spin.axis}, {right_caster_spin.axis})"
        ),
    )

    ctx.expect_contact(front_left_fork, left_frame, contact_tol=0.0015, name="left_fork_mount_contact")
    ctx.expect_contact(front_right_fork, right_frame, contact_tol=0.0015, name="right_fork_mount_contact")
    ctx.expect_contact(front_left_wheel, front_left_fork, contact_tol=0.0015, name="left_caster_axle_contact")
    ctx.expect_contact(front_right_wheel, front_right_fork, contact_tol=0.0015, name="right_caster_axle_contact")
    ctx.expect_contact(left_rear_wheel, left_frame, contact_tol=0.0015, name="left_rear_wheel_mount_contact")
    ctx.expect_contact(right_rear_wheel, right_frame, contact_tol=0.0015, name="right_rear_wheel_mount_contact")
    ctx.expect_origin_gap(front_left_fork, left_rear_wheel, axis="y", min_gap=0.30, name="left_wheelbase_length")
    ctx.expect_origin_gap(front_right_fork, right_rear_wheel, axis="y", min_gap=0.30, name="right_wheelbase_length")
    ctx.expect_origin_distance(left_frame, right_frame, axes="x", min_dist=0.34, name="open_frame_width")

    def _overall_width() -> float | None:
        min_x: float | None = None
        max_x: float | None = None
        for part in (
            center_hub,
            left_frame,
            right_frame,
            front_left_fork,
            front_right_fork,
            front_left_wheel,
            front_right_wheel,
            left_rear_wheel,
            right_rear_wheel,
        ):
            aabb = ctx.part_world_aabb(part)
            if aabb is None:
                return None
            low, high = aabb
            min_x = low[0] if min_x is None else min(min_x, low[0])
            max_x = high[0] if max_x is None else max(max_x, high[0])
        if min_x is None or max_x is None:
            return None
        return max_x - min_x

    open_width = _overall_width()
    with ctx.pose({left_slide: 0.08, right_slide: 0.08}):
        folded_width = _overall_width()
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_overlap_clearance")
        ctx.check(
            "frame_collapses_for_storage",
            open_width is not None
            and folded_width is not None
            and folded_width < open_width - 0.13
            and folded_width < 0.41,
            details=f"open_width={open_width}, folded_width={folded_width}",
        )

    with ctx.pose(front_left_caster_swivel=0.70, front_right_caster_swivel=-0.70):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned_caster_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
