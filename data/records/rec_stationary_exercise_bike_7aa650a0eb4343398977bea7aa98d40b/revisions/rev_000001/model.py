from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _tube_origin(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an origin and length for a cylinder whose local Z runs p0 -> p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    horizontal = sqrt(dx * dx + dy * dy)
    pitch = atan2(horizontal, dz)
    yaw = atan2(dy, dx) if horizontal > 1e-9 else 0.0
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
) -> None:
    origin, length = _tube_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _flywheel_guard_mesh(name: str):
    outer = superellipse_profile(0.66, 0.66, exponent=2.0, segments=64)
    inner = superellipse_profile(0.39, 0.39, exponent=2.0, segments=48)
    geom = ExtrudeWithHolesGeometry(outer, [inner], height=0.018, center=True).rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _saddle_mesh():
    # Top-view teardrop saddle profile, extruded into a thin cushion.
    profile = [
        (0.25, 0.000),
        (0.20, 0.050),
        (0.06, 0.085),
        (-0.14, 0.120),
        (-0.25, 0.095),
        (-0.30, 0.000),
        (-0.25, -0.095),
        (-0.14, -0.120),
        (0.06, -0.085),
        (0.20, -0.050),
    ]
    return mesh_from_geometry(ExtrudeGeometry(profile, height=0.065, center=True), "saddle_cushion")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_paint = model.material("satin_black_frame", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    housing = model.material("graphite_housing", rgba=(0.18, 0.19, 0.20, 1.0))
    accent = model.material("red_accent", rgba=(0.85, 0.05, 0.03, 1.0))

    frame = model.part("frame")

    # Floor-contact frame: two broad stabilizer feet tied by a central spine.
    frame.visual(Box((0.14, 0.74, 0.055)), origin=Origin(xyz=(-0.58, 0.0, 0.028)), material=rubber, name="rear_foot")
    frame.visual(Box((0.14, 0.74, 0.055)), origin=Origin(xyz=(0.58, 0.0, 0.028)), material=rubber, name="front_foot")
    _add_tube(frame, "floor_spine", (-0.58, 0.0, 0.045), (0.58, 0.0, 0.045), radius=0.022, material=frame_paint)

    # Main supports: seat column, bottom bracket support, flywheel fork, and handlebar mast.
    frame.visual(
        Cylinder(radius=0.040, length=0.560),
        origin=Origin(xyz=(-0.32, 0.0, 0.44)),
        material=frame_paint,
        name="seat_sleeve",
    )
    _add_tube(frame, "crank_down_tube", (0.10, 0.0, 0.09), (0.02, 0.0, 0.36), radius=0.034, material=frame_paint)
    _add_tube(frame, "top_tube", (-0.32, 0.060, 0.64), (0.92, 0.0, 0.78), radius=0.028, material=frame_paint)
    _add_tube(frame, "handlebar_mast", (0.84, 0.0, 0.085), (0.96, 0.0, 1.08), radius=0.033, material=frame_paint)
    _add_tube(frame, "front_mast_foot", (0.58, 0.0, 0.050), (0.84, 0.0, 0.055), radius=0.015, material=frame_paint)
    _add_tube(frame, "flywheel_stay_0", (0.30, 0.085, 0.09), (0.46, 0.085, 0.34), radius=0.022, material=frame_paint)
    _add_tube(frame, "flywheel_stay_1", (0.62, 0.085, 0.09), (0.46, 0.085, 0.34), radius=0.022, material=frame_paint)
    _add_tube(frame, "flywheel_stay_2", (0.30, -0.085, 0.09), (0.46, -0.085, 0.34), radius=0.022, material=frame_paint)
    _add_tube(frame, "flywheel_stay_3", (0.62, -0.085, 0.09), (0.46, -0.085, 0.34), radius=0.022, material=frame_paint)
    _add_tube(frame, "base_brace", (-0.31, 0.0, 0.23), (0.20, 0.0, 0.12), radius=0.026, material=frame_paint)
    frame.visual(Box((0.160, 0.100, 0.080)), origin=Origin(xyz=(0.13, 0.0, 0.095)), material=frame_paint, name="base_joint")

    # Bearing shell for the crank set.  The moving crank axle is intentionally
    # nested inside this fixed bearing proxy.
    frame.visual(
        Cylinder(radius=0.055, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.43), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="bottom_bracket_shell",
    )
    frame.visual(Box((0.100, 0.130, 0.045)), origin=Origin(xyz=(0.018, 0.0, 0.382)), material=frame_paint, name="bottom_bracket_lug")

    # Protective flywheel housing: two annular side plates with center bosses
    # and spoke-like webs, leaving the rotating disk visible through the guards.
    guard_mesh = _flywheel_guard_mesh("flywheel_side_guard")
    for side_index, y in enumerate((-0.064, 0.064)):
        frame.visual(guard_mesh, origin=Origin(xyz=(0.46, y, 0.34)), material=housing, name=f"flywheel_guard_{side_index}")
        frame.visual(
            Cylinder(radius=0.060, length=0.020),
            origin=Origin(xyz=(0.46, y, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"axle_boss_{side_index}",
        )
        frame.visual(Box((0.37, 0.018, 0.030)), origin=Origin(xyz=(0.46, y, 0.34)), material=housing, name=f"guard_web_x_{side_index}")
        frame.visual(Box((0.030, 0.018, 0.37)), origin=Origin(xyz=(0.46, y, 0.34)), material=housing, name=f"guard_web_z_{side_index}")
    frame.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.46, -0.049, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="flywheel_bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.46, 0.049, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="flywheel_bearing_1",
    )

    # Static handlebars and console mounted to the mast.
    _add_tube(frame, "handlebar_cross", (0.98, -0.30, 1.11), (0.98, 0.30, 1.11), radius=0.017, material=dark_metal)
    _add_tube(frame, "handlebar_stem", (0.95, 0.0, 1.02), (0.98, 0.0, 1.11), radius=0.021, material=dark_metal)
    frame.visual(Cylinder(radius=0.022, length=0.16), origin=Origin(xyz=(0.98, -0.33, 1.11), rpy=(pi / 2.0, 0.0, 0.0)), material=rubber, name="grip_0")
    frame.visual(Cylinder(radius=0.022, length=0.16), origin=Origin(xyz=(0.98, 0.33, 1.11), rpy=(pi / 2.0, 0.0, 0.0)), material=rubber, name="grip_1")
    frame.visual(Box((0.15, 0.06, 0.08)), origin=Origin(xyz=(0.94, 0.0, 1.02)), material=dark_metal, name="console")

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.255, length=0.046),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="flywheel_disk",
    )
    flywheel.visual(
        Cylinder(radius=0.180, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="flywheel_shadow",
    )
    flywheel.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="flywheel_axle",
    )
    flywheel.visual(Box((0.025, 0.052, 0.185)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=accent, name="flywheel_stripe")

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.025, length=0.255),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="crank_axle",
    )
    crank.visual(
        Cylinder(radius=0.130, length=0.018),
        origin=Origin(xyz=(0.0, 0.120, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="chainring",
    )
    _add_tube(crank, "crank_arm_0", (0.0, 0.115, 0.0), (0.305, 0.220, 0.0), radius=0.014, material=brushed)
    _add_tube(crank, "crank_arm_1", (0.0, -0.115, 0.0), (-0.305, -0.220, 0.0), radius=0.014, material=brushed)
    crank.visual(Cylinder(radius=0.014, length=0.095), origin=Origin(xyz=(0.305, 0.220, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=brushed, name="pedal_spindle_0")
    crank.visual(Cylinder(radius=0.014, length=0.095), origin=Origin(xyz=(-0.305, -0.220, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=brushed, name="pedal_spindle_1")
    crank.visual(Box((0.115, 0.055, 0.030)), origin=Origin(xyz=(0.345, 0.285, 0.0)), material=rubber, name="pedal_0")
    crank.visual(Box((0.115, 0.055, 0.030)), origin=Origin(xyz=(-0.345, -0.285, 0.0)), material=rubber, name="pedal_1")

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.022, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed,
        name="inner_post",
    )
    seat_post.visual(Box((0.090, 0.075, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.340)), material=dark_metal, name="seat_clamp")
    seat_post.visual(Box((0.265, 0.040, 0.026)), origin=Origin(xyz=(-0.020, 0.0, 0.373)), material=brushed, name="seat_rail")
    seat_post.visual(_saddle_mesh(), origin=Origin(xyz=(-0.020, 0.0, 0.414)), material=rubber, name="saddle")

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.46, 0.0, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=35.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.32, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank")
    flywheel = object_model.get_part("flywheel")
    seat_post = object_model.get_part("seat_post")
    crank_joint = object_model.get_articulation("crank_spin")
    flywheel_joint = object_model.get_articulation("flywheel_spin")
    seat_joint = object_model.get_articulation("seat_height")

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The adjustable seat post is intentionally shown as a nested sliding tube inside the fixed column.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        reason="The crank axle is intentionally captured inside the fixed bottom-bracket bearing shell.",
    )
    ctx.allow_overlap(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="flywheel_bearing_0",
        reason="The flywheel axle is intentionally captured in the fixed side bearing bushing.",
    )
    ctx.allow_overlap(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="flywheel_bearing_1",
        reason="The flywheel axle is intentionally captured in the fixed side bearing bushing.",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.001,
        name="seat post stays centered in the column",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.20,
        name="seat post remains inserted at low height",
    )
    with ctx.pose({seat_joint: 0.18}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.001,
            name="raised seat post stays centered",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.035,
            name="raised seat post retains insertion",
        )

    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="crank_axle",
        outer_elem="bottom_bracket_shell",
        margin=0.001,
        name="crank axle sits inside bottom bracket shell",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="crank_axle",
        elem_b="bottom_bracket_shell",
        min_overlap=0.10,
        name="bottom bracket captures the crank axle",
    )
    for bearing_name in ("flywheel_bearing_0", "flywheel_bearing_1"):
        ctx.expect_within(
            flywheel,
            frame,
            axes="xz",
            inner_elem="flywheel_axle",
            outer_elem=bearing_name,
            margin=0.001,
            name=f"flywheel axle centered in {bearing_name}",
        )
        ctx.expect_overlap(
            flywheel,
            frame,
            axes="y",
            elem_a="flywheel_axle",
            elem_b=bearing_name,
            min_overlap=0.006,
            name=f"flywheel axle retained by {bearing_name}",
        )

    ctx.check(
        "crank and flywheel are continuous rotations",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and flywheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"crank={crank_joint.articulation_type}, flywheel={flywheel_joint.articulation_type}",
    )
    ctx.check(
        "seat height uses a prismatic column",
        seat_joint.articulation_type == ArticulationType.PRISMATIC and tuple(seat_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_joint.articulation_type}, axis={seat_joint.axis}",
    )

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_joint: 0.18}):
        raised_pos = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.17,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
