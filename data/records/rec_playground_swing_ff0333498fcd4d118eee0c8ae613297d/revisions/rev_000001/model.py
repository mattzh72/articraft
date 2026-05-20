from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_swing_swivel")

    wood = model.material("warm_oiled_wood", rgba=(0.60, 0.36, 0.16, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    galvanized = model.material("brushed_galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = model.material("black_rubber_edge", rgba=(0.01, 0.01, 0.012, 1.0))

    # Root frame: the vertical swivel axis at the underside of the fixed bearing.
    beam = model.part("beam")
    beam.visual(
        Box((1.20, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=wood,
        name="timber_beam",
    )
    beam.visual(
        Box((0.26, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=dark_steel,
        name="mount_plate",
    )
    beam.visual(
        Cylinder(radius=0.052, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="fixed_bearing",
    )
    for ix, x in enumerate((-0.085, 0.085)):
        for iy, y in enumerate((-0.045, 0.045)):
            beam.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, 0.058)),
                material=dark_steel,
                name=f"bolt_{ix}_{iy}",
            )

    swivel = model.part("swivel_block")
    swivel.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_steel,
        name="swivel_cap",
    )
    swivel.visual(
        Box((0.22, 0.15, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=dark_steel,
        name="top_web",
    )
    swivel.visual(
        Cylinder(radius=0.045, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, -0.100), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_steel,
        name="upper_bushing",
    )
    for i, y in enumerate((-0.046, 0.046)):
        swivel.visual(
            Box((0.22, 0.028, 0.100)),
            origin=Origin(xyz=(0.0, y, -0.100)),
            material=dark_steel,
            name=f"clevis_cheek_{i}",
        )
    swivel.visual(
        Box((0.22, 0.15, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=dark_steel,
        name="keeper_web",
    )

    hanger = model.part("seat_hanger")
    hanger.visual(
        Cylinder(radius=0.020, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=galvanized,
        name="upper_pin",
    )
    for i, x in enumerate((-0.300, 0.300)):
        hanger.visual(
            Box((0.042, 0.018, 1.110)),
            origin=Origin(xyz=(x, 0.0, -0.560)),
            material=galvanized,
            name=f"side_link_{i}",
        )
        hanger.visual(
            Cylinder(radius=0.046, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=galvanized,
            name=f"upper_eye_{i}",
        )
        hanger.visual(
            Cylinder(radius=0.043, length=0.050),
            origin=Origin(xyz=(x, 0.0, -1.100), rpy=(0.0, 1.57079632679, 0.0)),
            material=galvanized,
            name=f"lower_eye_{i}",
        )
        hanger.visual(
            Box((0.052, 0.155, 0.135)),
            origin=Origin(xyz=(x, 0.0, -1.145)),
            material=galvanized,
            name=f"seat_bracket_{i}",
        )
    hanger.visual(
        Cylinder(radius=0.020, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, -1.100), rpy=(0.0, 1.57079632679, 0.0)),
        material=galvanized,
        name="lower_pin",
    )
    seat_profile = rounded_rect_profile(0.58, 0.27, 0.040, corner_segments=8)
    seat_mesh = mesh_from_geometry(
        ExtrudeGeometry(seat_profile, 0.045, cap=True, center=True),
        "rounded_seat_board",
    )
    hanger.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.185)),
        material=wood,
        name="seat_board",
    )
    for i, y in enumerate((-0.136, 0.136)):
        hanger.visual(
            Box((0.52, 0.018, 0.035)),
            origin=Origin(xyz=(0.0, y, -1.185)),
            material=black_rubber,
            name=f"seat_edge_{i}",
        )

    model.articulation(
        "beam_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=beam,
        child=swivel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5),
    )
    model.articulation(
        "swivel_to_hanger",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=hanger,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    swivel = object_model.get_part("swivel_block")
    hanger = object_model.get_part("seat_hanger")
    top_swivel = object_model.get_articulation("beam_to_swivel")
    swing = object_model.get_articulation("swivel_to_hanger")

    ctx.allow_overlap(
        swivel,
        hanger,
        elem_a="upper_bushing",
        elem_b="upper_pin",
        reason="The hanger pin is intentionally captured through the swivel block bushing.",
    )
    ctx.expect_within(
        hanger,
        swivel,
        axes="yz",
        inner_elem="upper_pin",
        outer_elem="upper_bushing",
        margin=0.001,
        name="upper pin is centered in the swivel bushing",
    )
    ctx.expect_overlap(
        hanger,
        swivel,
        axes="x",
        elem_a="upper_pin",
        elem_b="upper_bushing",
        min_overlap=0.20,
        name="upper pin remains retained through the bushing",
    )

    rest_board_aabb = ctx.part_element_world_aabb(hanger, elem="seat_board")
    with ctx.pose({swing: 0.55}):
        swung_board_aabb = ctx.part_element_world_aabb(hanger, elem="seat_board")

    if rest_board_aabb is not None and swung_board_aabb is not None:
        rest_center_y = (rest_board_aabb[0][1] + rest_board_aabb[1][1]) * 0.5
        swung_center_y = (swung_board_aabb[0][1] + swung_board_aabb[1][1]) * 0.5
        rest_center_z = (rest_board_aabb[0][2] + rest_board_aabb[1][2]) * 0.5
        swung_center_z = (swung_board_aabb[0][2] + swung_board_aabb[1][2]) * 0.5
        ctx.check(
            "seat swings forward and upward on hanger pivots",
            swung_center_y > rest_center_y + 0.45 and swung_center_z > rest_center_z + 0.12,
            details=f"rest_yz=({rest_center_y:.3f}, {rest_center_z:.3f}), swung_yz=({swung_center_y:.3f}, {swung_center_z:.3f})",
        )
    else:
        ctx.fail("seat swing pose is measurable", "seat_board AABB was unavailable")

    rest_yaw_aabb = ctx.part_element_world_aabb(hanger, elem="seat_board")
    with ctx.pose({top_swivel: 1.0}):
        yawed_board_aabb = ctx.part_element_world_aabb(hanger, elem="seat_board")

    if rest_yaw_aabb is not None and yawed_board_aabb is not None:
        rest_dy = rest_yaw_aabb[1][1] - rest_yaw_aabb[0][1]
        yawed_dy = yawed_board_aabb[1][1] - yawed_board_aabb[0][1]
        ctx.check(
            "top swivel rotates the hanging seat about the vertical pivot",
            yawed_dy > rest_dy + 0.25,
            details=f"rest_board_dy={rest_dy:.3f}, yawed_board_dy={yawed_dy:.3f}",
        )
    else:
        ctx.fail("top swivel pose is measurable", "seat_board AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
