from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


SEAT_COUNT = 8
AXLE_HEIGHT = 2.65
RIM_RADIUS = 1.58
RIM_TUBE_RADIUS = 0.055
RIM_HALF_SPACING = 0.22
CROSSBEAM_DEPTH = 0.06
CROSSBEAM_CENTER_RADIUS = RIM_RADIUS + 0.01
HANGER_MOUNT_RADIUS = CROSSBEAM_CENTER_RADIUS + 0.05
HANGER_DROP = 0.24


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz(radius: float, angle: float) -> tuple[float, float]:
    return radius * cos(angle), radius * sin(angle)


def _seat_angle(index: int) -> float:
    return -pi / 2.0 + index * (2.0 * pi / SEAT_COUNT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_observation_wheel")

    trailer_red = model.material("trailer_red", rgba=(0.63, 0.18, 0.16, 1.0))
    cream = model.material("cream", rgba=(0.90, 0.89, 0.84, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.94, 0.94, 0.95, 1.0))
    seat_teal = model.material("seat_teal", rgba=(0.20, 0.58, 0.65, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.68, 0.29, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    tire_black = model.material("tire_black", rgba=(0.06, 0.06, 0.07, 1.0))

    trailer_base = model.part("trailer_base")
    trailer_base.inertial = Inertial.from_geometry(
        Box((2.95, 1.70, 0.50)),
        mass=420.0,
        origin=Origin(xyz=(0.22, 0.0, 0.18)),
    )
    trailer_base.visual(
        Box((2.20, 1.35, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=trailer_red,
        name="deck",
    )
    trailer_base.visual(
        Box((1.55, 0.34, 0.10)),
        origin=Origin(xyz=(-0.05, 0.0, 0.13)),
        material=dark_steel,
        name="undercarriage",
    )
    trailer_base.visual(
        Box((0.78, 0.18, 0.10)),
        origin=Origin(xyz=(1.49, 0.0, 0.16)),
        material=dark_steel,
        name="tow_bar",
    )
    trailer_base.visual(
        Box((0.22, 0.24, 0.08)),
        origin=Origin(xyz=(1.86, 0.0, 0.12)),
        material=dark_steel,
        name="hitch_block",
    )
    trailer_base.visual(
        Box((0.14, 1.50, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, 0.23)),
        material=dark_steel,
        name="axle_beam",
    )
    for side_y, name in ((-0.80, "left_road_wheel"), (0.80, "right_road_wheel")):
        trailer_base.visual(
            Cylinder(radius=0.26, length=0.14),
            origin=Origin(xyz=(-0.08, side_y, 0.28), rpy=(0.0, pi / 2.0, 0.0)),
            material=tire_black,
            name=name,
        )
        trailer_base.visual(
            Box((0.26, 0.10, 0.08)),
            origin=Origin(xyz=(-0.02, side_y, 0.54)),
            material=trailer_red,
            name=f"fender_{'left' if side_y < 0.0 else 'right'}",
        )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            trailer_base.visual(
                Box((0.10, 0.10, 0.32)),
                origin=Origin(xyz=(0.86 * x_sign, 0.52 * y_sign, 0.16)),
                material=dark_steel,
                name=f"stabilizer_post_{'f' if x_sign > 0 else 'r'}_{'r' if y_sign > 0 else 'l'}",
            )
            trailer_base.visual(
                Box((0.18, 0.18, 0.04)),
                origin=Origin(xyz=(0.86 * x_sign, 0.52 * y_sign, 0.02)),
                material=dark_steel,
                name=f"stabilizer_pad_{'f' if x_sign > 0 else 'r'}_{'r' if y_sign > 0 else 'l'}",
            )

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((1.00, 0.90, 2.90)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )
    support_frame.visual(
        Box((0.92, 0.70, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=dark_steel,
        name="base_saddle",
    )
    for x_sign in (-1.0, 1.0):
        x = 0.40 * x_sign
        for y in (-0.30, 0.30):
            support_frame.visual(
                Box((0.16, 0.14, 0.06)),
                origin=Origin(xyz=(x, y, 0.27)),
                material=dark_steel,
                name=f"foot_{'left' if x_sign < 0 else 'right'}_{'front' if y < 0 else 'rear'}",
            )
        for mesh_name, points in (
            (
                f"support_leg_front_{'left' if x_sign < 0 else 'right'}",
                [(x, -0.30, 0.30), (x, -0.10, 1.65), (x, 0.00, AXLE_HEIGHT - 0.06)],
            ),
            (
                f"support_leg_rear_{'left' if x_sign < 0 else 'right'}",
                [(x, 0.30, 0.30), (x, 0.10, 1.65), (x, 0.00, AXLE_HEIGHT - 0.06)],
            ),
            (
                f"support_brace_{'left' if x_sign < 0 else 'right'}",
                [(x, -0.22, 0.86), (x, 0.18, 1.56)],
            ),
        ):
            support_frame.visual(
                _save_mesh(
                    mesh_name,
                    tube_from_spline_points(
                        points,
                        radius=0.045 if "leg" in mesh_name else 0.028,
                        samples_per_segment=10,
                        radial_segments=18,
                    ),
                ),
                material=cream,
            )
    support_frame.visual(
        Box((0.80, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.30, 0.34)),
        material=dark_steel,
        name="base_tie_front",
    )
    support_frame.visual(
        Box((0.80, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.30, 0.34)),
        material=dark_steel,
        name="base_tie_rear",
    )
    support_frame.visual(
        Box((0.10, 0.18, 0.18)),
        origin=Origin(xyz=(-0.40, 0.0, AXLE_HEIGHT)),
        material=dark_steel,
        name="bearing_left",
    )
    support_frame.visual(
        Box((0.10, 0.18, 0.18)),
        origin=Origin(xyz=(0.40, 0.0, AXLE_HEIGHT)),
        material=dark_steel,
        name="bearing_right",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.66, length=0.70),
        mass=260.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    rim_mesh = _save_mesh(
        "observation_wheel_rim",
        TorusGeometry(RIM_RADIUS, RIM_TUBE_RADIUS, radial_segments=18, tubular_segments=88).rotate_y(pi / 2.0),
    )
    wheel.visual(rim_mesh, origin=Origin(xyz=(-RIM_HALF_SPACING, 0.0, 0.0)), material=wheel_white, name="rim_left")
    wheel.visual(rim_mesh, origin=Origin(xyz=(RIM_HALF_SPACING, 0.0, 0.0)), material=wheel_white, name="rim_right")
    wheel.visual(
        Cylinder(radius=0.035, length=0.70),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.46),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    for side_x in (-0.18, 0.18):
        for index in range(SEAT_COUNT):
            angle = _seat_angle(index)
            spoke_y, spoke_z = _yz(RIM_RADIUS, angle)
            wheel.visual(
                _save_mesh(
                    f"wheel_spoke_{'left' if side_x < 0.0 else 'right'}_{index}",
                    tube_from_spline_points(
                        [(side_x * 0.45, 0.0, 0.0), (side_x, spoke_y, spoke_z)],
                        radius=0.020,
                        samples_per_segment=2,
                        radial_segments=14,
                    ),
                ),
                material=wheel_white,
            )
    for index in range(SEAT_COUNT):
        angle = _seat_angle(index)
        beam_y, beam_z = _yz(CROSSBEAM_CENTER_RADIUS, angle)
        wheel.visual(
            Box((0.48, CROSSBEAM_DEPTH, 0.04)),
            origin=Origin(xyz=(0.0, beam_y, beam_z), rpy=(angle, 0.0, 0.0)),
            material=brass,
            name=f"crossbeam_{index}",
        )

    model.articulation(
        "trailer_to_support",
        ArticulationType.FIXED,
        parent=trailer_base,
        child=support_frame,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )

    for index in range(SEAT_COUNT):
        angle = _seat_angle(index)
        mount_y, mount_z = _yz(HANGER_MOUNT_RADIUS, angle)

        hanger = model.part(f"hanger_{index}")
        hanger.inertial = Inertial.from_geometry(
            Box((0.24, 0.34, 0.10)),
            mass=1.8,
            origin=Origin(xyz=(0.0, 0.10, 0.0)),
        )
        hanger.visual(
            Box((0.18, 0.04, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=cream,
            name="mount_block",
        )
        hanger.visual(
            Box((0.04, HANGER_DROP - 0.04, 0.04)),
            origin=Origin(xyz=(0.0, HANGER_DROP * 0.5, 0.0)),
            material=cream,
            name="stem",
        )
        hanger.visual(
            Box((0.22, 0.04, 0.03)),
            origin=Origin(xyz=(0.0, HANGER_DROP, 0.055)),
            material=cream,
            name="clip_bar",
        )
        hanger.visual(
            Box((0.02, 0.04, 0.08)),
            origin=Origin(xyz=(-0.11, HANGER_DROP, 0.0)),
            material=dark_steel,
            name="left_jaw",
        )
        hanger.visual(
            Box((0.02, 0.04, 0.08)),
            origin=Origin(xyz=(0.11, HANGER_DROP, 0.0)),
            material=dark_steel,
            name="right_jaw",
        )
        hanger.visual(
            Cylinder(radius=0.010, length=0.20),
            origin=Origin(xyz=(0.0, HANGER_DROP, -0.02), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="pivot_rod",
        )
        model.articulation(
            f"wheel_to_hanger_{index}",
            ArticulationType.FIXED,
            parent=wheel,
            child=hanger,
            origin=Origin(xyz=(0.0, mount_y, mount_z), rpy=(angle, 0.0, 0.0)),
        )

        seat = model.part(f"seat_{index}")
        seat.inertial = Inertial.from_geometry(
            Box((0.34, 0.24, 0.22)),
            mass=5.0,
            origin=Origin(xyz=(0.0, 0.10, 0.0)),
        )
        seat.visual(
            Box((0.20, 0.04, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_block",
        )
        seat.visual(
            Box((0.24, 0.02, 0.16)),
            origin=Origin(xyz=(0.0, 0.17, 0.03)),
            material=seat_teal,
            name="bench",
        )
        seat.visual(
            Box((0.24, 0.12, 0.02)),
            origin=Origin(xyz=(0.0, 0.08, -0.07)),
            material=seat_teal,
            name="backrest",
        )
        seat.visual(
            Box((0.03, 0.18, 0.16)),
            origin=Origin(xyz=(-0.075, 0.11, 0.02)),
            material=seat_teal,
            name="left_side",
        )
        seat.visual(
            Box((0.03, 0.18, 0.16)),
            origin=Origin(xyz=(0.075, 0.11, 0.02)),
            material=seat_teal,
            name="right_side",
        )
        seat.visual(
            Cylinder(radius=0.012, length=0.22),
            origin=Origin(xyz=(0.0, 0.12, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name="front_guard",
        )
        seat.visual(
            Cylinder(radius=0.010, length=0.22),
            origin=Origin(xyz=(0.0, 0.23, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="foot_bar",
        )
        model.articulation(
            f"hanger_to_seat_{index}",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=seat,
            origin=Origin(xyz=(0.0, HANGER_DROP, 0.0), rpy=(-angle, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=14.0, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for index in range(SEAT_COUNT):
        ctx.allow_overlap(
            f"hanger_{index}",
            "wheel",
            reason="Simplified hanger clamp geometry shares envelope with the wheel's seat-mount crossbeam.",
        )
        ctx.allow_overlap(
            f"hanger_{index}",
            f"seat_{index}",
            reason="Simplified seat pivot uses an intentionally interpenetrating captured pin and yoke envelope.",
        )

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

    def _require_part(name: str):
        try:
            part = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - test scaffolding guard
            ctx.fail(f"part_present_{name}", str(exc))
            return None
        ctx.check(f"part_present_{name}", True)
        return part

    def _require_joint(name: str):
        try:
            articulation = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - test scaffolding guard
            ctx.fail(f"joint_present_{name}", str(exc))
            return None
        ctx.check(f"joint_present_{name}", True)
        return articulation

    trailer_base = _require_part("trailer_base")
    support_frame = _require_part("support_frame")
    wheel = _require_part("wheel")
    wheel_spin = _require_joint("wheel_spin")

    for index in range(SEAT_COUNT):
        _require_part(f"hanger_{index}")
        _require_part(f"seat_{index}")
        _require_joint(f"wheel_to_hanger_{index}")
        _require_joint(f"hanger_to_seat_{index}")

    if trailer_base and support_frame:
        ctx.expect_contact(
            support_frame,
            trailer_base,
            name="support_frame_contacts_trailer_base",
        )
    if wheel and support_frame:
        ctx.expect_contact(
            wheel,
            support_frame,
            elem_a="axle",
            elem_b="bearing_left",
            name="axle_contacts_left_bearing",
        )
        ctx.expect_contact(
            wheel,
            support_frame,
            elem_a="axle",
            elem_b="bearing_right",
            name="axle_contacts_right_bearing",
        )
    if wheel and trailer_base:
        ctx.expect_origin_gap(
            wheel,
            trailer_base,
            axis="z",
            min_gap=2.4,
            max_gap=2.9,
            name="wheel_axle_height_above_trailer",
        )

    if wheel_spin is not None:
        ctx.check(
            "wheel_spin_is_continuous_x_axis",
            wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
            f"Expected continuous x-axis wheel spin, got type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
        )

    for index in range(SEAT_COUNT):
        hanger = object_model.get_part(f"hanger_{index}")
        seat = object_model.get_part(f"seat_{index}")
        seat_joint = object_model.get_articulation(f"hanger_to_seat_{index}")

        ctx.expect_contact(
            hanger,
            wheel,
            elem_a="mount_block",
            elem_b=f"crossbeam_{index}",
            name=f"hanger_{index}_mounted_to_crossbeam",
        )
        ctx.expect_contact(
            seat,
            hanger,
            elem_a="pivot_block",
            elem_b="left_jaw",
            name=f"seat_{index}_retained_by_hanger",
        )
        ctx.check(
            f"seat_joint_{index}_is_continuous_x_axis",
            seat_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(seat_joint.axis) == (1.0, 0.0, 0.0),
            f"Expected seat joint {index} to be continuous about x, got type={seat_joint.articulation_type}, axis={seat_joint.axis}",
        )

    if wheel_spin is not None:
        seat_joint_0 = object_model.get_articulation("hanger_to_seat_0")
        with ctx.pose({wheel_spin: pi / 2.0, seat_joint_0: -pi / 2.0}):
            ctx.expect_contact(
                "seat_0",
                "hanger_0",
                elem_a="pivot_block",
                elem_b="left_jaw",
                name="seat_0_remains_captured_when_wheel_turns",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
