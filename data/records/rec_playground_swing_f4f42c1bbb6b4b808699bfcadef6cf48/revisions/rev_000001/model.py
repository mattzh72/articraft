from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disc_seat_swing")

    powder_blue = model.material("powder_blue", rgba=(0.16, 0.37, 0.70, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.67, 1.0))
    dark_pin = model.material("dark_pin", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rope = model.material("black_rope", rgba=(0.02, 0.025, 0.025, 1.0))
    warm_seat = model.material("warm_seat", rgba=(0.96, 0.62, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))

    seat_shell = mesh_from_geometry(
        LatheGeometry(
            [
                (0.000, -0.030),
                (0.145, -0.030),
                (0.185, -0.026),
                (0.205, -0.010),
                (0.198, 0.018),
                (0.150, 0.034),
                (0.030, 0.028),
                (0.000, 0.020),
            ],
            segments=80,
        ),
        "rounded_disc_seat",
    )
    seat_rim = mesh_from_geometry(
        TorusGeometry(radius=0.194, tube=0.012, radial_segments=18, tubular_segments=80),
        "seat_soft_rim",
    )
    hanger_eye = mesh_from_geometry(
        TorusGeometry(radius=0.027, tube=0.0145, radial_segments=18, tubular_segments=56),
        "top_swing_eye",
    )
    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.265, tube=0.010, radial_segments=14, tubular_segments=96),
        "narrow_foot_ring",
    )

    top_beam = model.part("top_beam")
    top_beam.visual(
        Box((0.90, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=powder_blue,
        name="main_beam",
    )
    top_beam.visual(
        Box((0.18, 0.075, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=galvanized,
        name="hanger_clamp_plate",
    )
    for index, x_pos in enumerate((-0.058, 0.058)):
        top_beam.visual(
            Box((0.026, 0.070, 0.118)),
            origin=Origin(xyz=(x_pos, 0.0, 0.018)),
            material=galvanized,
            name=f"hanger_cheek_{index}",
        )
    top_beam.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_pin,
        name="hanger_pivot_pin",
    )
    top_beam.visual(
        Box((0.055, 0.080, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=galvanized,
        name="center_hanger_block",
    )

    seat = model.part("seat")
    seat.visual(
        hanger_eye,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="top_swing_eye",
    )
    seat.visual(
        Cylinder(radius=0.014, length=1.100),
        origin=Origin(xyz=(0.0, 0.0, -0.585)),
        material=black_rope,
        name="center_hanger_rope",
    )
    seat.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=galvanized,
        name="rope_thimble",
    )
    seat.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -1.123)),
        material=galvanized,
        name="seat_top_boss",
    )
    seat.visual(
        seat_shell,
        origin=Origin(xyz=(0.0, 0.0, -1.160)),
        material=warm_seat,
        name="seat_disc",
    )
    seat.visual(
        seat_rim,
        origin=Origin(xyz=(0.0, 0.0, -1.148)),
        material=rubber,
        name="seat_rim",
    )
    seat.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -1.218)),
        material=galvanized,
        name="underside_hub",
    )
    for index, x_pos in enumerate((-0.078, 0.078)):
        seat.visual(
            Box((0.028, 0.050, 0.050)),
            origin=Origin(xyz=(x_pos, -0.218, -1.205)),
            material=galvanized,
            name=f"footrest_hinge_lug_{index}",
        )
    seat.visual(
        Box((0.170, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.205, -1.178)),
        material=galvanized,
        name="hinge_mount_plate",
    )

    footrest_ring = model.part("footrest_ring")
    footrest_ring.visual(
        Cylinder(radius=0.012, length=0.134),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_pin,
        name="hinge_barrel",
    )
    footrest_ring.visual(
        Box((0.130, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, 0.018, -0.040)),
        material=galvanized,
        name="hinge_web",
    )
    footrest_ring.visual(
        Box((0.018, 0.110, 0.010)),
        origin=Origin(xyz=(-0.048, 0.050, -0.035), rpy=(-0.56, 0.0, 0.0)),
        material=galvanized,
        name="ring_strut_0",
    )
    footrest_ring.visual(
        Box((0.018, 0.110, 0.010)),
        origin=Origin(xyz=(0.048, 0.050, -0.035), rpy=(-0.56, 0.0, 0.0)),
        material=galvanized,
        name="ring_strut_1",
    )
    footrest_ring.visual(
        Box((0.130, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.026, -0.078)),
        material=galvanized,
        name="rear_ring_bridge",
    )
    footrest_ring.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.265, -0.080)),
        material=rubber,
        name="foot_ring",
    )

    model.articulation(
        "hanger_pivot",
        ArticulationType.REVOLUTE,
        parent=top_beam,
        child=seat,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=footrest_ring,
        origin=Origin(xyz=(0.0, -0.218, -1.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_beam = object_model.get_part("top_beam")
    seat = object_model.get_part("seat")
    footrest_ring = object_model.get_part("footrest_ring")
    hanger_pivot = object_model.get_articulation("hanger_pivot")
    footrest_hinge = object_model.get_articulation("footrest_hinge")

    ctx.allow_overlap(
        top_beam,
        seat,
        elem_a="hanger_pivot_pin",
        elem_b="top_swing_eye",
        reason="The swing eye is intentionally captured on the hanger pivot pin with a small bushing interference.",
    )
    ctx.expect_overlap(
        top_beam,
        seat,
        axes="xyz",
        elem_a="hanger_pivot_pin",
        elem_b="top_swing_eye",
        min_overlap=0.004,
        name="hanger pivot pin is captured by the top swing eye",
    )
    for lug_name in ("footrest_hinge_lug_0", "footrest_hinge_lug_1"):
        ctx.allow_overlap(
            seat,
            footrest_ring,
            elem_a=lug_name,
            elem_b="hinge_barrel",
            reason="The footrest hinge barrel is seated into the underside hinge lug as a captured pin proxy.",
        )
        ctx.expect_overlap(
            seat,
            footrest_ring,
            axes="xyz",
            elem_a=lug_name,
            elem_b="hinge_barrel",
            min_overlap=0.002,
            name=f"{lug_name} captures the footrest hinge barrel",
        )

    ctx.expect_gap(
        top_beam,
        seat,
        axis="z",
        positive_elem="main_beam",
        negative_elem="seat_disc",
        min_gap=1.15,
        name="disc seat hangs well below the beam",
    )
    ctx.expect_gap(
        seat,
        footrest_ring,
        axis="z",
        positive_elem="seat_disc",
        negative_elem="foot_ring",
        min_gap=0.040,
        max_gap=0.120,
        name="deployed foot ring sits below the seat underside",
    )
    ctx.expect_overlap(
        seat,
        footrest_ring,
        axes="xy",
        elem_a="seat_disc",
        elem_b="foot_ring",
        min_overlap=0.14,
        name="foot ring is centered under the circular seat",
    )

    def _element_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[1] + upper[1]) / 2.0

    def _element_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[2] + upper[2]) / 2.0

    rest_seat_y = _element_center_y(seat, "seat_disc")
    with ctx.pose({hanger_pivot: 0.45}):
        swung_seat_y = _element_center_y(seat, "seat_disc")
    ctx.check(
        "seat swings forward on the hanger pivot",
        rest_seat_y is not None
        and swung_seat_y is not None
        and swung_seat_y > rest_seat_y + 0.40,
        details=f"rest_y={rest_seat_y}, swung_y={swung_seat_y}",
    )

    deployed_ring_z = _element_center_z(footrest_ring, "foot_ring")
    with ctx.pose({footrest_hinge: 1.0}):
        folded_ring_z = _element_center_z(footrest_ring, "foot_ring")
    ctx.check(
        "footrest ring folds upward on its underside hinge",
        deployed_ring_z is not None
        and folded_ring_z is not None
        and folded_ring_z > deployed_ring_z + 0.18,
        details=f"deployed_z={deployed_ring_z}, folded_z={folded_ring_z}",
    )

    return ctx.report()


object_model = build_object_model()
