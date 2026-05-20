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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_rod(part, a, b, radius: float, *, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disc_seat_swing_with_footrest")

    beam_wood = model.material("beam_wood", rgba=(0.54, 0.39, 0.22, 1.0))
    hardware = model.material("hardware", rgba=(0.22, 0.23, 0.25, 1.0))
    strap_black = model.material("strap_black", rgba=(0.10, 0.10, 0.11, 1.0))
    seat_black = model.material("seat_black", rgba=(0.14, 0.14, 0.15, 1.0))
    footrest_gray = model.material("footrest_gray", rgba=(0.34, 0.35, 0.37, 1.0))

    top_beam = model.part("top_beam")
    top_beam.visual(
        Box((0.12, 0.72, 0.08)),
        material=beam_wood,
        name="beam",
    )
    top_beam.visual(
        Box((0.060, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        material=hardware,
        name="hanger_mount",
    )
    top_beam.visual(
        Box((0.038, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.015, -0.125)),
        material=hardware,
        name="hanger_cheek_left",
    )
    top_beam.visual(
        Box((0.038, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, -0.125)),
        material=hardware,
        name="hanger_cheek_right",
    )
    top_beam.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.021, -0.112),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="pivot_cap_left",
    )
    top_beam.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.021, -0.112),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="pivot_cap_right",
    )
    top_beam.inertial = Inertial.from_geometry(
        Box((0.12, 0.72, 0.22)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Box((0.032, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=hardware,
        name="hanger_tongue",
    )
    seat_assembly.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=hardware,
        name="upper_bushing",
    )
    seat_assembly.visual(
        Box((0.030, 0.014, 0.455)),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=strap_black,
        name="suspension_strap",
    )
    seat_assembly.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.478)),
        material=hardware,
        name="seat_hub",
    )
    seat_assembly.visual(
        Cylinder(radius=0.160, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.510)),
        material=seat_black,
        name="seat_disk",
    )
    seat_assembly.visual(
        Cylinder(radius=0.148, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.491)),
        material=strap_black,
        name="seat_pad",
    )
    seat_assembly.visual(
        Box((0.070, 0.036, 0.018)),
        origin=Origin(xyz=(0.078, 0.0, -0.530)),
        material=hardware,
        name="footrest_bracket_base",
    )
    seat_assembly.visual(
        Box((0.018, 0.008, 0.022)),
        origin=Origin(xyz=(0.095, 0.013, -0.548)),
        material=hardware,
        name="footrest_lug_left",
    )
    seat_assembly.visual(
        Box((0.018, 0.008, 0.022)),
        origin=Origin(xyz=(0.095, -0.013, -0.548)),
        material=hardware,
        name="footrest_lug_right",
    )
    seat_assembly.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.56)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.280)),
    )

    footrest_ring = model.part("footrest_ring")
    footrest_ring.visual(
        Box((0.016, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=hardware,
        name="footrest_hinge_tongue",
    )
    _add_rod(
        footrest_ring,
        (0.0, 0.0, -0.018),
        (-0.020, 0.062, -0.135),
        0.0055,
        material=footrest_gray,
        name="footrest_arm_left",
    )
    _add_rod(
        footrest_ring,
        (0.0, 0.0, -0.018),
        (-0.020, -0.062, -0.135),
        0.0055,
        material=footrest_gray,
        name="footrest_arm_right",
    )
    footrest_ring.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.100,
                tube=0.007,
                radial_segments=18,
                tubular_segments=52,
            ),
            "folding_footrest_ring",
        ),
        origin=Origin(xyz=(-0.095, 0.0, -0.135)),
        material=footrest_gray,
        name="footrest_loop",
    )
    footrest_ring.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.16)),
        mass=0.7,
        origin=Origin(xyz=(-0.065, 0.0, -0.085)),
    )

    model.articulation(
        "beam_to_seat",
        ArticulationType.REVOLUTE,
        parent=top_beam,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat_assembly,
        child=footrest_ring,
        origin=Origin(xyz=(0.095, 0.0, -0.536)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    top_beam = object_model.get_part("top_beam")
    seat_assembly = object_model.get_part("seat_assembly")
    footrest_ring = object_model.get_part("footrest_ring")
    seat_swing = object_model.get_articulation("beam_to_seat")
    footrest_fold = object_model.get_articulation("seat_to_footrest")

    ctx.expect_gap(
        top_beam,
        seat_assembly,
        axis="z",
        min_gap=0.50,
        positive_elem="beam",
        negative_elem="seat_disk",
        name="seat hangs clearly below the top beam",
    )
    ctx.expect_gap(
        seat_assembly,
        footrest_ring,
        axis="z",
        min_gap=0.12,
        positive_elem="seat_disk",
        negative_elem="footrest_loop",
        name="footrest ring sits below the seat disk at rest",
    )
    ctx.expect_overlap(
        footrest_ring,
        seat_assembly,
        axes="xy",
        min_overlap=0.14,
        elem_a="footrest_loop",
        elem_b="seat_disk",
        name="footrest remains centered beneath the disc seat footprint",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat_assembly)
    with ctx.pose({seat_swing: 0.45}):
        swung_seat_aabb = ctx.part_world_aabb(seat_assembly)

    rest_seat_center_x = None
    swung_seat_center_x = None
    if rest_seat_aabb is not None:
        rest_seat_center_x = (rest_seat_aabb[0][0] + rest_seat_aabb[1][0]) * 0.5
    if swung_seat_aabb is not None:
        swung_seat_center_x = (swung_seat_aabb[0][0] + swung_seat_aabb[1][0]) * 0.5
    ctx.check(
        "positive seat swing moves the seat forward",
        rest_seat_center_x is not None
        and swung_seat_center_x is not None
        and swung_seat_center_x > rest_seat_center_x + 0.16,
        details=f"rest_x={rest_seat_center_x}, swung_x={swung_seat_center_x}",
    )

    rest_footrest_aabb = ctx.part_element_world_aabb(footrest_ring, elem="footrest_loop")
    with ctx.pose({footrest_fold: footrest_fold.motion_limits.upper}):
        folded_footrest_aabb = ctx.part_element_world_aabb(footrest_ring, elem="footrest_loop")
        ctx.expect_gap(
            seat_assembly,
            footrest_ring,
            axis="z",
            min_gap=0.006,
            positive_elem="seat_disk",
            negative_elem="footrest_loop",
            name="folded footrest still clears the underside of the seat",
        )

    rest_footrest_top = None
    folded_footrest_top = None
    if rest_footrest_aabb is not None:
        rest_footrest_top = rest_footrest_aabb[1][2]
    if folded_footrest_aabb is not None:
        folded_footrest_top = folded_footrest_aabb[1][2]
    ctx.check(
        "footrest rotates upward toward the seat",
        rest_footrest_top is not None
        and folded_footrest_top is not None
        and folded_footrest_top > rest_footrest_top + 0.11,
        details=f"rest_top={rest_footrest_top}, folded_top={folded_footrest_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
