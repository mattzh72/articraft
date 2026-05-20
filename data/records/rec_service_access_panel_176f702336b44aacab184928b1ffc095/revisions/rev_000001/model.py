from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


HINGE_X = -0.43
HINGE_Z = 0.105
DOOR_WIDTH = 0.84
DOOR_HEIGHT = 1.08
DOOR_THICKNESS = 0.035
DOOR_HINGE_CLEARANCE = 0.045
LATCH_X = 0.72


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    diameter: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 48,
) -> list[tuple[float, float]]:
    return _offset_profile(
        superellipse_profile(diameter, diameter, exponent=2.0, segments=segments),
        cx,
        cy,
    )


def _ring_mesh(name: str, outer_diameter: float, inner_diameter: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_diameter, segments=64),
            [_circle_profile(inner_diameter, segments=48)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_access_panel")

    frame_paint = model.material("powder_coated_frame", rgba=(0.16, 0.18, 0.18, 1.0))
    door_paint = model.material("safety_yellow_door", rgba=(0.95, 0.64, 0.12, 1.0))
    dark_steel = model.material("dark_zinc_hardware", rgba=(0.23, 0.24, 0.25, 1.0))
    bright_steel = model.material("worn_bright_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    rubber = model.material("black_replaceable_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    brass = model.material("oiled_bronze_wear", rgba=(0.68, 0.48, 0.20, 1.0))
    latch_red = model.material("red_latch_handle", rgba=(0.78, 0.08, 0.06, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.72, 0.95),
                (1.08, 1.30),
                0.080,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.035,
            ),
            "service_frame_bezel",
        ),
        material=frame_paint,
        name="frame_shell",
    )
    # Replaceable rubber compression strips around the clear service opening.
    frame.visual(
        Box((0.78, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, 0.492, 0.044)),
        material=rubber,
        name="top_gasket",
    )
    frame.visual(
        Box((0.78, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, -0.492, 0.044)),
        material=rubber,
        name="bottom_gasket",
    )
    frame.visual(
        Box((0.035, 0.98, 0.014)),
        origin=Origin(xyz=(-0.378, 0.0, 0.044)),
        material=rubber,
        name="hinge_gasket",
    )
    frame.visual(
        Box((0.035, 0.98, 0.014)),
        origin=Origin(xyz=(0.378, 0.0, 0.044)),
        material=rubber,
        name="latch_gasket",
    )
    # Chunky welded hinge and latch side rails carry the field hardware.
    frame.visual(
        Box((0.070, 1.16, 0.080)),
        origin=Origin(xyz=(-0.505, 0.0, 0.075)),
        material=dark_steel,
        name="hinge_rail",
    )
    frame.visual(
        Box((0.060, 1.06, 0.120)),
        origin=Origin(xyz=(0.505, 0.0, 0.100)),
        material=dark_steel,
        name="latch_rail",
    )
    frame.visual(
        Box((0.070, 0.180, 0.050)),
        origin=Origin(xyz=(0.505, 0.0, 0.142)),
        material=dark_steel,
        name="keeper_base",
    )
    frame.visual(
        Box((0.035, 0.130, 0.045)),
        origin=Origin(xyz=(0.490, 0.0, 0.150)),
        material=bright_steel,
        name="striker_post",
    )
    for idx, y in enumerate((-0.40, 0.0, 0.40)):
        frame.visual(
            Box((0.110, 0.180, 0.012)),
            origin=Origin(xyz=(HINGE_X - 0.050, y, HINGE_Z - 0.010)),
            material=bright_steel,
            name=f"frame_leaf_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.032, length=0.160),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"frame_knuckle_{idx}",
        )
    # Bronze replaceable wear pads show serviceable closure points, not decorative dots.
    for idx, y in enumerate((-0.34, 0.34)):
        frame.visual(
            Box((0.060, 0.105, 0.010)),
            origin=Origin(xyz=(0.475, y, 0.140)),
            material=brass,
            name=f"strike_wear_pad_{idx}",
        )
    for idx, (x, y) in enumerate(
        (
            (-0.505, -0.54),
            (-0.505, 0.54),
            (0.505, -0.47),
            (0.505, 0.47),
            (-0.28, 0.585),
            (0.28, 0.585),
            (-0.28, -0.585),
            (0.28, -0.585),
        )
    ):
        frame.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.042)),
            material=bright_steel,
            name=f"frame_bolt_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _offset_profile(
                    rounded_rect_profile(DOOR_WIDTH, DOOR_HEIGHT, 0.026, corner_segments=8),
                    DOOR_HINGE_CLEARANCE + DOOR_WIDTH / 2.0,
                    0.0,
                ),
                [_circle_profile(0.050, cx=LATCH_X, cy=0.0, segments=56)],
                DOOR_THICKNESS,
                center=True,
            ),
            "service_door_slab",
        ),
        material=door_paint,
        name="door_slab",
    )
    # Bolt-on perimeter wear strips make the door read as a repeatedly serviced part.
    door.visual(
        Box((0.780, 0.035, 0.012)),
        origin=Origin(xyz=(0.455, 0.500, 0.023)),
        material=rubber,
        name="top_wear_strip",
    )
    door.visual(
        Box((0.780, 0.035, 0.012)),
        origin=Origin(xyz=(0.455, -0.500, 0.023)),
        material=rubber,
        name="bottom_wear_strip",
    )
    door.visual(
        Box((0.035, 0.900, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.023)),
        material=rubber,
        name="hinge_wear_strip",
    )
    door.visual(
        Box((0.035, 0.900, 0.012)),
        origin=Origin(xyz=(0.790, 0.0, 0.023)),
        material=rubber,
        name="latch_wear_strip",
    )
    door.visual(
        _ring_mesh("front_latch_bushing", 0.110, 0.052, 0.018),
        origin=Origin(xyz=(LATCH_X, 0.0, 0.027)),
        material=bright_steel,
        name="front_bushing",
    )
    door.visual(
        _ring_mesh("rear_latch_bushing", 0.092, 0.052, 0.014),
        origin=Origin(xyz=(LATCH_X, 0.0, -0.0235)),
        material=bright_steel,
        name="rear_bushing",
    )
    for idx, y in enumerate((-0.20, 0.20)):
        door.visual(
            Box((0.095, 0.220, 0.012)),
            origin=Origin(xyz=(0.045, y, -0.010)),
            material=bright_steel,
            name=f"door_leaf_{idx}",
        )
        door.visual(
            Cylinder(radius=0.032, length=0.220),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"door_knuckle_{idx}",
        )
    for idx, (x, y) in enumerate(
        (
            (0.13, 0.46),
            (0.43, 0.46),
            (0.73, 0.46),
            (0.13, -0.46),
            (0.43, -0.46),
            (0.73, -0.46),
            (0.09, 0.26),
            (0.09, -0.26),
            (0.79, 0.26),
            (0.79, -0.26),
        )
    ):
        door.visual(
            Cylinder(radius=0.014, length=0.009),
            origin=Origin(xyz=(x, y, 0.022)),
            material=dark_steel,
            name=f"door_fastener_{idx}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bright_steel,
        name="latch_shaft",
    )
    latch.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="front_hub",
    )
    latch.visual(
        Box((0.055, 0.245, 0.026)),
        origin=Origin(xyz=(0.0, -0.095, 0.040)),
        material=latch_red,
        name="handle_lever",
    )
    latch.visual(
        Cylinder(radius=0.022, length=0.135),
        origin=Origin(xyz=(0.0, -0.205, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    latch.visual(
        Box((0.170, 0.045, 0.024)),
        origin=Origin(xyz=(0.085, 0.0, 0.000)),
        material=bright_steel,
        name="cam_dog",
    )
    latch.visual(
        Box((0.048, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark_steel,
        name="cam_hub",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=math.radians(105.0)),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(LATCH_X, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=math.radians(90.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.check("main_parts_present", all((frame, door, latch, hinge, latch_turn)), "Frame, door, latch, and both joints are required.")
    if not all((frame, door, latch, hinge, latch_turn)):
        return ctx.report()

    with ctx.pose({hinge: 0.0, latch_turn: 0.0}):
        ctx.expect_gap(
            door,
            frame,
            axis="z",
            positive_elem="door_slab",
            negative_elem="frame_shell",
            min_gap=0.035,
            max_gap=0.060,
            name="door stands proud of the frame opening",
        )
        ctx.expect_overlap(
            door,
            frame,
            axes="xy",
            elem_a="door_slab",
            elem_b="frame_shell",
            min_overlap=0.72,
            name="closed door covers the framed service opening",
        )
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="striker_post",
            negative_elem="cam_dog",
            min_gap=0.004,
            max_gap=0.020,
            name="closed cam dog lands just behind the latch-side striker",
        )

    rest_latch_aabb = ctx.part_element_world_aabb(latch, elem="cam_dog")
    with ctx.pose({latch_turn: math.radians(90.0)}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="cam_dog")
    ctx.check(
        "quarter_turn_latch_retracts_from_keeper",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and float(rest_latch_aabb[1][0] - open_latch_aabb[1][0]) > 0.10,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )

    rest_latch_pos = ctx.part_world_position(latch)
    with ctx.pose({hinge: math.radians(95.0)}):
        swung_latch_pos = ctx.part_world_position(latch)
    ctx.check(
        "door_swing_moves_latch_side_outward",
        rest_latch_pos is not None
        and swung_latch_pos is not None
        and float(swung_latch_pos[2] - rest_latch_pos[2]) > 0.55,
        details=f"rest={rest_latch_pos}, swung={swung_latch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
