from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent
MESH_DIR = HERE / "assets" / "meshes"


BODY_WIDTH = 0.160
BACKPLATE_THICKNESS = 0.008
MAIN_FRAME_OUTER = 0.142
MAIN_FRAME_INNER = 0.114
MAIN_FRAME_DEPTH = 0.028
UPPER_FRAME_OUTER = 0.132
UPPER_FRAME_INNER = 0.096
UPPER_FRAME_DEPTH = 0.016

BEZEL_OUTER_RADIUS = 0.054
BEZEL_INNER_RADIUS = 0.043
BEZEL_THICKNESS = 0.006

KNOB_OUTER_RADIUS = 0.041
KNOB_INNER_RADIUS = 0.0220
KNOB_THICKNESS = 0.012
AXIS_Z = 0.046

SPINDLE_BASE_RADIUS = 0.018
SPINDLE_SHOULDER_RADIUS = 0.0135
SPINDLE_JOURNAL_RADIUS = 0.0090
SPINDLE_CAP_RADIUS = 0.0130


def _save_mesh(name: str, geometry: MeshGeometry):
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _circle_profile(
    radius: float,
    *,
    segments: int = 64,
    phase: float = 0.0,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = phase + (math.tau * index / segments)
        if clockwise:
            angle = -angle
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _chamfered_square_profile(width: float, chamfer: float) -> list[tuple[float, float]]:
    half = width * 0.5
    chamfer = min(chamfer, half * 0.45)
    return [
        (-half + chamfer, -half),
        (half - chamfer, -half),
        (half, -half + chamfer),
        (half, half - chamfer),
        (half - chamfer, half),
        (-half + chamfer, half),
        (-half, half - chamfer),
        (-half, -half + chamfer),
    ]


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments, clockwise=True)],
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
    )


def _build_spindle_mesh() -> MeshGeometry:
    radial_segments = 56
    base = CylinderGeometry(
        radius=SPINDLE_BASE_RADIUS,
        height=0.022,
        radial_segments=radial_segments,
    ).translate(0.0, 0.0, -0.026)
    shoulder = CylinderGeometry(
        radius=SPINDLE_SHOULDER_RADIUS,
        height=0.010,
        radial_segments=radial_segments,
    ).translate(0.0, 0.0, -0.011)
    journal = CylinderGeometry(
        radius=SPINDLE_JOURNAL_RADIUS,
        height=KNOB_THICKNESS,
        radial_segments=radial_segments,
    )
    cap = CylinderGeometry(
        radius=SPINDLE_CAP_RADIUS,
        height=0.004,
        radial_segments=radial_segments,
    ).translate(0.0, 0.0, 0.008)
    return _merge_geometries(base, shoulder, journal, cap)


def _build_impact_shroud_mesh():
    return _save_mesh(
        "thermostat_impact_shroud.obj",
        ExtrudeWithHolesGeometry(
            _chamfered_square_profile(0.136, 0.012),
            [_circle_profile(0.0455, segments=72, clockwise=True)],
            0.016,
            cap=True,
            center=True,
            closed=True,
        ),
    )


def _add_square_frame(
    part,
    *,
    outer: float,
    inner: float,
    depth: float,
    z_bottom: float,
    material,
    prefix: str,
) -> None:
    wall = (outer - inner) * 0.5
    z_center = z_bottom + depth * 0.5
    x_pos = inner * 0.5 + wall * 0.5
    y_pos = x_pos

    part.visual(
        Box((wall, outer, depth)),
        origin=Origin(xyz=(-x_pos, 0.0, z_center)),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((wall, outer, depth)),
        origin=Origin(xyz=(x_pos, 0.0, z_center)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((inner, wall, depth)),
        origin=Origin(xyz=(0.0, y_pos, z_center)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((inner, wall, depth)),
        origin=Origin(xyz=(0.0, -y_pos, z_center)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_wall_thermostat", assets=ASSETS)

    housing_paint = model.material("housing_paint", rgba=(0.47, 0.50, 0.48, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.24, 0.26, 0.25, 1.0))
    dial_rubber = model.material("dial_rubber", rgba=(0.13, 0.14, 0.15, 1.0))
    zinc_fastener = model.material("zinc_fastener", rgba=(0.71, 0.73, 0.76, 1.0))
    signal_orange = model.material("signal_orange", rgba=(0.90, 0.46, 0.12, 1.0))

    bezel_mesh = _annulus_mesh(
        "thermostat_bezel_ring.obj",
        outer_radius=BEZEL_OUTER_RADIUS,
        inner_radius=BEZEL_INNER_RADIUS,
        thickness=BEZEL_THICKNESS,
    )
    knob_mesh = _annulus_mesh(
        "thermostat_control_dial.obj",
        outer_radius=KNOB_OUTER_RADIUS,
        inner_radius=KNOB_INNER_RADIUS,
        thickness=KNOB_THICKNESS,
    )
    spindle_mesh = _save_mesh("thermostat_spindle.obj", _build_spindle_mesh())
    shroud_mesh = _build_impact_shroud_mesh()

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_WIDTH, BACKPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BACKPLATE_THICKNESS * 0.5)),
        material=housing_dark,
        name="backplate",
    )
    body.visual(
        Box((0.114, 0.114, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, BACKPLATE_THICKNESS + 0.002)),
        material=housing_dark,
        name="backplate_reinforcement_pad",
    )
    _add_square_frame(
        body,
        outer=MAIN_FRAME_OUTER,
        inner=MAIN_FRAME_INNER,
        depth=MAIN_FRAME_DEPTH,
        z_bottom=BACKPLATE_THICKNESS,
        material=housing_paint,
        prefix="main_frame",
    )
    _add_square_frame(
        body,
        outer=UPPER_FRAME_OUTER,
        inner=UPPER_FRAME_INNER,
        depth=UPPER_FRAME_DEPTH,
        z_bottom=BACKPLATE_THICKNESS + MAIN_FRAME_DEPTH,
        material=housing_paint,
        prefix="upper_frame",
    )
    body.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, 0.0, BACKPLATE_THICKNESS + MAIN_FRAME_DEPTH + UPPER_FRAME_DEPTH * 0.5)),
        material=housing_paint,
        name="impact_shroud",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.024, 0.024, UPPER_FRAME_DEPTH)),
                origin=Origin(
                    xyz=(
                        x_sign * 0.051,
                        y_sign * 0.051,
                        BACKPLATE_THICKNESS + MAIN_FRAME_DEPTH + UPPER_FRAME_DEPTH * 0.5,
                    )
                ),
                material=housing_paint,
                name=f"corner_gusset_{int((x_sign + 1) * 0.5)}_{int((y_sign + 1) * 0.5)}",
            )
    for x_sign in (-1.0, 1.0):
        body.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(x_sign * 0.042, 0.053, BACKPLATE_THICKNESS + MAIN_FRAME_DEPTH + 0.002),
            ),
            material=housing_dark,
            name=f"service_boss_upper_{'left' if x_sign < 0.0 else 'right'}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(x_sign * 0.042, -0.053, BACKPLATE_THICKNESS + MAIN_FRAME_DEPTH + 0.002),
            ),
            material=housing_dark,
            name=f"service_boss_lower_{'left' if x_sign < 0.0 else 'right'}",
        )
    body.visual(
        Box((0.050, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, -0.058, 0.0515)),
        material=housing_dark,
        name="service_label_pad",
    )
    for index, x_pos in enumerate((-0.034, 0.034)):
        body.visual(
            Cylinder(radius=0.0055, length=0.002),
            origin=Origin(xyz=(x_pos, -0.058, 0.053)),
            material=zinc_fastener,
            name=f"service_label_fastener_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_WIDTH, 0.058)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    dial_face_ring = model.part("dial_face_ring")
    dial_face_ring.visual(
        bezel_mesh,
        material=housing_dark,
        name="bezel_ring",
    )
    for tick_index, angle in enumerate((-2.1, -1.4, -0.7, 0.0, 0.7, 1.4, 2.1)):
        radius = 0.048
        dial_face_ring.visual(
            Box((0.0025, 0.009, 0.0012)),
            origin=Origin(
                xyz=(radius * math.sin(angle), radius * math.cos(angle), 0.0024),
                rpy=(0.0, 0.0, angle),
            ),
            material=zinc_fastener,
            name=f"tick_{tick_index}",
        )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        x_pos = 0.048 * math.cos(angle)
        y_pos = 0.048 * math.sin(angle)
        dial_face_ring.visual(
            Cylinder(radius=0.0045, length=0.002),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=zinc_fastener,
            name=f"bezel_fastener_head_{index}",
        )
        dial_face_ring.visual(
            Box((0.006, 0.0012, 0.0008)),
            origin=Origin(
                xyz=(x_pos, y_pos, 0.0048),
                rpy=(0.0, 0.0, angle + math.pi / 4.0),
            ),
            material=housing_dark,
            name=f"bezel_fastener_slot_{index}",
        )
    dial_face_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=BEZEL_OUTER_RADIUS, length=BEZEL_THICKNESS),
        mass=0.12,
        origin=Origin(),
    )

    dial_spindle = model.part("dial_spindle")
    dial_spindle.visual(
        spindle_mesh,
        material=zinc_fastener,
        name="spindle_body",
    )
    dial_spindle.visual(
        Box((0.006, 0.0012, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0102)),
        material=housing_dark,
        name="spindle_slot",
    )
    dial_spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_BASE_RADIUS, length=0.052),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        knob_mesh,
        material=dial_rubber,
        name="dial_ring",
    )
    for lug_index in range(8):
        angle = lug_index * math.tau / 8.0
        radius = 0.036
        control_knob.visual(
            Box((0.011, 0.0075, KNOB_THICKNESS)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dial_rubber,
            name=f"grip_lug_{lug_index}",
        )
    for pad_index, angle in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        control_knob.visual(
            Box((0.005, 0.005, 0.001)),
            origin=Origin(
                xyz=(0.0405 * math.cos(angle), 0.0405 * math.sin(angle), 0.0055),
                rpy=(0.0, 0.0, angle),
            ),
            material=housing_dark,
            name=f"bearing_pad_{pad_index}",
        )
    control_knob.visual(
        Box((0.010, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.029, 0.005)),
        material=signal_orange,
        name="setpoint_pointer",
    )
    control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_OUTER_RADIUS, length=KNOB_THICKNESS),
        mass=0.16,
        origin=Origin(),
    )

    model.articulation(
        "body_to_dial_face_ring",
        ArticulationType.FIXED,
        parent=body,
        child=dial_face_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    model.articulation(
        "body_to_dial_spindle",
        ArticulationType.FIXED,
        parent=body,
        child=dial_spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
    )
    model.articulation(
        "body_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_knob,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.6,
            velocity=3.5,
            lower=-2.1,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    dial_face_ring = object_model.get_part("dial_face_ring")
    dial_spindle = object_model.get_part("dial_spindle")
    control_knob = object_model.get_part("control_knob")
    dial_joint = object_model.get_articulation("body_to_control_knob")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check("body_present", body is not None, "Missing wall-mount body.")
    ctx.check("dial_face_ring_present", dial_face_ring is not None, "Missing dial face ring.")
    ctx.check("dial_spindle_present", dial_spindle is not None, "Missing center spindle support.")
    ctx.check("control_knob_present", control_knob is not None, "Missing rotating control knob.")

    ctx.check(
        "dial_joint_axis_is_centered_z",
        tuple(dial_joint.axis) == (0.0, 0.0, 1.0),
        f"Dial axis should be +Z, got {dial_joint.axis!r}.",
    )
    limits = dial_joint.motion_limits
    ctx.check(
        "dial_joint_has_practical_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and 4.0 <= (limits.upper - limits.lower) <= 4.4,
        "Dial should have a realistic bounded rotation range near 240 degrees.",
    )

    ctx.expect_contact(dial_face_ring, body, name="dial_face_ring_seated_on_body")
    ctx.expect_contact(dial_spindle, body, name="dial_spindle_anchored_to_body")
    ctx.expect_contact(control_knob, dial_face_ring, name="control_knob_bearing_contact")
    ctx.expect_origin_distance(
        control_knob,
        dial_spindle,
        axes="xy",
        max_dist=1e-6,
        name="control_knob_is_coaxial_with_spindle",
    )
    ctx.expect_origin_distance(
        dial_face_ring,
        dial_spindle,
        axes="xy",
        max_dist=1e-6,
        name="dial_face_ring_is_coaxial_with_spindle",
    )
    ctx.expect_gap(
        dial_face_ring,
        control_knob,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        name="dial_face_ring_sits_proud_of_knob",
    )
    ctx.expect_overlap(
        dial_face_ring,
        body,
        axes="xy",
        min_overlap=0.100,
        name="dial_face_ring_has_broad_mounting_land",
    )
    ctx.expect_overlap(
        control_knob,
        dial_face_ring,
        axes="xy",
        min_overlap=0.080,
        name="control_knob_reads_inside_bezel_footprint",
    )

    body_size = _aabb_size(ctx.part_world_aabb(body))
    knob_size = _aabb_size(ctx.part_world_aabb(control_knob))
    ring_size = _aabb_size(ctx.part_world_aabb(dial_face_ring))
    ctx.check(
        "overall_body_size_is_realistic",
        body_size is not None
        and 0.150 <= body_size[0] <= 0.165
        and 0.150 <= body_size[1] <= 0.165
        and 0.050 <= body_size[2] <= 0.060,
        f"Expected roughly 160 x 160 x 58 mm body, got {body_size!r}.",
    )
    ctx.check(
        "dial_size_is_realistic",
        knob_size is not None
        and 0.082 <= knob_size[0] <= 0.088
        and 0.082 <= knob_size[1] <= 0.088
        and 0.011 <= knob_size[2] <= 0.0135,
        f"Expected rugged dial near 85 mm diameter, got {knob_size!r}.",
    )
    ctx.check(
        "bezel_exceeds_knob_for_protection",
        ring_size is not None and knob_size is not None and ring_size[0] > knob_size[0] and ring_size[1] > knob_size[1],
        f"Bezel should protect knob footprint, got ring={ring_size!r}, knob={knob_size!r}.",
    )

    with ctx.pose({dial_joint: 0.0}):
        ctx.fail_if_isolated_parts(name="dial_rest_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="dial_rest_no_overlap")
        ctx.expect_contact(control_knob, dial_face_ring, name="dial_rest_support_contact")

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({dial_joint: limits.lower}):
            ctx.fail_if_isolated_parts(name="dial_lower_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="dial_lower_no_overlap")
            ctx.expect_contact(control_knob, dial_face_ring, name="dial_lower_support_contact")
        with ctx.pose({dial_joint: limits.upper}):
            ctx.fail_if_isolated_parts(name="dial_upper_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name="dial_upper_no_overlap")
            ctx.expect_contact(control_knob, dial_face_ring, name="dial_upper_support_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
