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
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


SEGMENTS = 72


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 48, reverse: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]
    return list(reversed(points)) if reverse else points


def _annular_ring(outer_radius: float, inner_radius: float, height: float, z_center: float) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius, reverse=True)],
        height,
        center=True,
    ).translate(0.0, 0.0, z_center)


def _build_bottle_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.038, 0.004),
            (0.043, 0.018),
            (0.046, 0.055),
            (0.047, 0.110),
            (0.046, 0.130),
            (0.042, 0.137),
            (0.036, 0.143),
            (0.033, 0.145),
        ],
        [
            (0.000, 0.003),
            (0.034, 0.010),
            (0.039, 0.022),
            (0.042, 0.055),
            (0.043, 0.110),
            (0.042, 0.130),
            (0.038, 0.137),
            (0.032, 0.143),
            (0.029, 0.145),
        ],
        segments=SEGMENTS,
        start_cap="flat",
        end_cap="flat",
    )


def _build_neck_base() -> MeshGeometry:
    return _annular_ring(0.0300, 0.0100, 0.004, 0.147)


def _build_neck_shaft() -> MeshGeometry:
    shaft = _annular_ring(0.0195, 0.0095, 0.042, 0.170)
    top_seat = _annular_ring(0.0210, 0.0095, 0.004, 0.190)
    shaft.merge(top_seat)
    return shaft


def _build_neck_bead() -> MeshGeometry:
    return _annular_ring(0.0245, 0.0095, 0.004, 0.161)


def _build_pump_stem() -> MeshGeometry:
    return CylinderGeometry(radius=0.0065, height=0.052, radial_segments=SEGMENTS).translate(
        0.0,
        0.0,
        0.170,
    )


def _build_pump_boss() -> MeshGeometry:
    return CylinderGeometry(radius=0.016, height=0.009, radial_segments=SEGMENTS).translate(
        0.0,
        0.0,
        0.1965,
    )


def _build_actuator_head() -> MeshGeometry:
    head = ExtrudeGeometry(
        rounded_rect_profile(0.082, 0.050, 0.010, corner_segments=8),
        0.014,
    ).translate(0.0, 0.0, 0.205)
    thumb_pad = ExtrudeGeometry(
        rounded_rect_profile(0.064, 0.044, 0.009, corner_segments=8),
        0.004,
    ).translate(-0.006, 0.0, 0.214)
    spout_block = BoxGeometry((0.024, 0.018, 0.009)).translate(0.046, 0.0, 0.205)
    nozzle = (
        CylinderGeometry(radius=0.0045, height=0.012, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(0.060, 0.0, 0.204)
    )
    head.merge(thumb_pad)
    head.merge(spout_block)
    head.merge(nozzle)
    return head


def _build_lock_collar() -> MeshGeometry:
    outer_skirt = _annular_ring(0.0335, 0.0265, 0.009, 0.1545)
    grip_band = _annular_ring(0.0355, 0.0265, 0.005, 0.1475)
    outer_skirt.merge(grip_band)
    return outer_skirt


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foaming_soap_pump_bottle")

    bottle_resin = model.material("bottle_resin", rgba=(0.82, 0.90, 0.86, 1.0))
    pump_white = model.material("pump_white", rgba=(0.96, 0.96, 0.97, 1.0))
    collar_satin = model.material("collar_satin", rgba=(0.82, 0.84, 0.86, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _mesh(_build_bottle_shell(), "bottle_shell"),
        material=bottle_resin,
        name="body_shell",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.190),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    neck_collar = model.part("neck_collar")
    neck_collar.visual(
        _mesh(_build_neck_base(), "neck_base"),
        material=pump_white,
        name="neck_base",
    )
    neck_collar.visual(
        _mesh(_build_neck_shaft(), "neck_shaft"),
        material=pump_white,
        name="neck_shaft",
    )
    neck_collar.visual(
        _mesh(_build_neck_bead(), "neck_bead"),
        material=pump_white,
        name="neck_bead",
    )
    neck_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.050),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        _mesh(_build_pump_stem(), "pump_stem"),
        material=pump_white,
        name="pump_stem",
    )
    actuator.visual(
        _mesh(_build_pump_boss(), "pump_boss"),
        material=pump_white,
        name="pump_boss",
    )
    actuator.visual(
        _mesh(_build_actuator_head(), "actuator_head"),
        material=pump_white,
        name="actuator_head",
    )
    actuator.inertial = Inertial.from_geometry(
        Box((0.086, 0.050, 0.063)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.1815)),
    )

    lock_collar = model.part("lock_collar")
    lock_collar.visual(
        _mesh(_build_lock_collar(), "lock_collar"),
        material=collar_satin,
        name="lock_shell",
    )
    lock_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.013),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
    )

    model.articulation(
        "body_to_neck_collar",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=neck_collar,
        origin=Origin(),
    )
    model.articulation(
        "neck_to_actuator",
        ArticulationType.PRISMATIC,
        parent=neck_collar,
        child=actuator,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.06,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "neck_to_lock_collar",
        ArticulationType.CONTINUOUS,
        parent=neck_collar,
        child=lock_collar,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    neck_collar = object_model.get_part("neck_collar")
    actuator = object_model.get_part("actuator")
    lock_collar = object_model.get_part("lock_collar")

    actuator_slide = object_model.get_articulation("neck_to_actuator")
    collar_turn = object_model.get_articulation("neck_to_lock_collar")

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
        "actuator_joint_is_vertical_prismatic",
        actuator_slide.articulation_type == ArticulationType.PRISMATIC
        and actuator_slide.axis == (0.0, 0.0, -1.0)
        and actuator_slide.motion_limits is not None
        and actuator_slide.motion_limits.lower == 0.0
        and actuator_slide.motion_limits.upper == 0.006,
        details="Actuator should slide vertically downward through a short pump stroke.",
    )
    ctx.check(
        "lock_collar_joint_is_vertical_rotation",
        collar_turn.articulation_type == ArticulationType.CONTINUOUS
        and collar_turn.axis == (0.0, 0.0, 1.0)
        and collar_turn.motion_limits is not None
        and collar_turn.motion_limits.lower is None
        and collar_turn.motion_limits.upper is None,
        details="The lock collar should rotate freely around the bottle neck axis.",
    )

    ctx.expect_contact(
        neck_collar,
        bottle_body,
        elem_a="neck_base",
        elem_b="body_shell",
        name="neck_collar_seated_on_bottle",
    )
    ctx.expect_within(
        actuator,
        neck_collar,
        axes="xy",
        inner_elem="pump_stem",
        outer_elem="neck_shaft",
        margin=0.0,
        name="pump_stem_runs_inside_neck",
    )
    ctx.expect_contact(
        lock_collar,
        bottle_body,
        elem_a="lock_shell",
        elem_b="body_shell",
        name="lock_collar_rests_on_bottle_shoulder",
    )
    ctx.expect_overlap(
        lock_collar,
        neck_collar,
        axes="xy",
        min_overlap=0.040,
        elem_a="lock_shell",
        elem_b="neck_shaft",
        name="lock_collar_encircles_neck",
    )
    ctx.expect_overlap(
        actuator,
        neck_collar,
        axes="xy",
        min_overlap=0.013,
        elem_a="pump_stem",
        elem_b="neck_shaft",
        name="actuator_centered_over_neck",
    )
    ctx.expect_gap(
        neck_collar,
        lock_collar,
        axis="z",
        positive_elem="neck_bead",
        negative_elem="lock_shell",
        min_gap=0.0,
        max_gap=0.0045,
        name="retaining_bead_sits_just_above_lock_collar",
    )
    ctx.expect_gap(
        lock_collar,
        bottle_body,
        axis="z",
        positive_elem="lock_shell",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="lock_collar_supported_by_bottle_shoulder",
    )
    ctx.expect_gap(
        actuator,
        lock_collar,
        axis="z",
        positive_elem="actuator_head",
        negative_elem="lock_shell",
        min_gap=0.020,
        name="actuator_head_sits_above_lock_collar",
    )

    actuator_rest = ctx.part_world_position(actuator)
    ctx.check(
        "actuator_present",
        actuator_rest is not None,
        details="Actuator part should resolve in the assembled model.",
    )
    with ctx.pose({actuator_slide: 0.006}):
        actuator_pressed = ctx.part_world_position(actuator)
        ctx.check(
            "actuator_moves_down_when_pressed",
            actuator_rest is not None
            and actuator_pressed is not None
            and actuator_pressed[2] < actuator_rest[2] - 0.005,
            details="Pressing the head should move the actuator downward by a short stroke.",
        )
        ctx.expect_contact(
            lock_collar,
            bottle_body,
            elem_a="lock_shell",
            elem_b="body_shell",
            name="lock_collar_stays_supported_when_pressed",
        )
        ctx.expect_gap(
            actuator,
            lock_collar,
            axis="z",
            positive_elem="actuator_head",
            negative_elem="lock_shell",
            min_gap=0.014,
            name="actuator_head_clears_lock_collar_when_pressed",
        )

    collar_rest = ctx.part_world_position(lock_collar)
    with ctx.pose({collar_turn: math.pi / 2.0}):
        collar_turned = ctx.part_world_position(lock_collar)
        ctx.check(
            "lock_collar_turns_without_wandering",
            collar_rest is not None
            and collar_turned is not None
            and abs(collar_turned[0] - collar_rest[0]) < 1e-9
            and abs(collar_turned[1] - collar_rest[1]) < 1e-9
            and abs(collar_turned[2] - collar_rest[2]) < 1e-9,
            details="The lock collar should stay centered on the bottle neck while rotating.",
        )
        ctx.expect_contact(
            lock_collar,
            bottle_body,
            elem_a="lock_shell",
            elem_b="body_shell",
            name="lock_collar_stays_supported_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
