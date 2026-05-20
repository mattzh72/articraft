from __future__ import annotations

from math import pi

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
)


FACE_WIDTH = 0.92
FACE_HEIGHT = 1.05
FACE_THICKNESS = 0.040
OPENING_WIDTH = 0.56
OPENING_HEIGHT = 0.72
FRAME_WIDTH = 0.70
FRAME_HEIGHT = 0.84
FRAME_DEPTH = 0.026
HINGE_X = -0.385
HINGE_Y = 0.070
DOOR_WIDTH = 0.700
DOOR_HEIGHT = 0.860
DOOR_THICKNESS = 0.024


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    return [
        (-width / 2.0, -height / 2.0),
        (width / 2.0, -height / 2.0),
        (width / 2.0, height / 2.0),
        (-width / 2.0, height / 2.0),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.27, 0.30, 0.32, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.43, 0.47, 0.49, 1.0))
    door_paint = model.material("door_paint", rgba=(0.62, 0.68, 0.70, 1.0))
    raised_door_paint = model.material("raised_door_paint", rgba=(0.70, 0.75, 0.76, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.035, 0.038, 0.040, 1.0))
    zinc_hardware = model.material("zinc_hardware", rgba=(0.67, 0.69, 0.70, 1.0))
    latch_black = model.material("latch_black", rgba=(0.05, 0.055, 0.06, 1.0))

    equipment_face = model.part("equipment_face")

    face_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(FACE_WIDTH, FACE_HEIGHT, 0.018, corner_segments=6),
        [rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.010, corner_segments=5)],
        FACE_THICKNESS,
        center=True,
    ).rotate_x(pi / 2.0)
    equipment_face.visual(
        mesh_from_geometry(face_plate, "equipment_face_plate"),
        material=cabinet_paint,
        name="face_plate",
    )

    service_frame = BezelGeometry(
        (OPENING_WIDTH, OPENING_HEIGHT),
        (FRAME_WIDTH, FRAME_HEIGHT),
        FRAME_DEPTH,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.012,
        outer_corner_radius=0.018,
    ).rotate_x(pi / 2.0)
    equipment_face.visual(
        mesh_from_geometry(service_frame, "service_frame"),
        origin=Origin(xyz=(0.0, FACE_THICKNESS / 2.0 + FRAME_DEPTH / 2.0 - 0.001, 0.0)),
        material=frame_paint,
        name="service_frame",
    )

    # A thin black gasket line just inside the access opening makes the cutout
    # read as a real service aperture without filling the opening solid.
    gasket = BezelGeometry(
        (OPENING_WIDTH - 0.040, OPENING_HEIGHT - 0.040),
        (OPENING_WIDTH + 0.010, OPENING_HEIGHT + 0.010),
        0.008,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.014,
    ).rotate_x(pi / 2.0)
    equipment_face.visual(
        mesh_from_geometry(gasket, "opening_gasket"),
        origin=Origin(xyz=(0.0, FACE_THICKNESS / 2.0 + 0.003, 0.0)),
        material=dark_gasket,
        name="opening_gasket",
    )

    for index, (sx, sz) in enumerate(
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    ):
        equipment_face.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(
                xyz=(sx * (FRAME_WIDTH / 2.0 - 0.060), FACE_THICKNESS / 2.0 + FRAME_DEPTH - 0.003, sz * (FRAME_HEIGHT / 2.0 - 0.055)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=zinc_hardware,
            name=f"frame_screw_{index}",
        )

    # Fixed-side hinge knuckles and strike hardware are included on the root
    # equipment face; the moving center knuckle is part of the door.
    for name, zc in (("lower_hinge_barrel", -0.300), ("upper_hinge_barrel", 0.300)):
        equipment_face.visual(
            Cylinder(radius=0.018, length=0.220),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, zc)),
            material=zinc_hardware,
            name=name,
        )
        equipment_face.visual(
            Box((0.040, 0.022, 0.160)),
            origin=Origin(xyz=(HINGE_X + 0.028, HINGE_Y - 0.027, zc)),
            material=zinc_hardware,
            name=f"{name}_leaf",
        )

    equipment_face.visual(
        Cylinder(radius=0.007, length=0.860),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        material=zinc_hardware,
        name="hinge_pin",
    )

    equipment_face.visual(
        Box((0.020, 0.018, 0.180)),
        origin=Origin(xyz=(FRAME_WIDTH / 2.0 - 0.004, FACE_THICKNESS / 2.0 + FRAME_DEPTH - 0.004, 0.0)),
        material=zinc_hardware,
        name="latch_strike",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.390, 0.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )

    # A simple raised perimeter gives the door a light formed-shell look while
    # keeping the secondary ribbing intentionally sparse.
    door.visual(
        Box((0.610, 0.012, 0.046)),
        origin=Origin(xyz=(0.395, DOOR_THICKNESS / 2.0 + 0.006, DOOR_HEIGHT / 2.0 - 0.055)),
        material=raised_door_paint,
        name="top_rail",
    )
    door.visual(
        Box((0.610, 0.012, 0.046)),
        origin=Origin(xyz=(0.395, DOOR_THICKNESS / 2.0 + 0.006, -DOOR_HEIGHT / 2.0 + 0.055)),
        material=raised_door_paint,
        name="bottom_rail",
    )
    door.visual(
        Box((0.046, 0.012, 0.745)),
        origin=Origin(xyz=(0.090, DOOR_THICKNESS / 2.0 + 0.006, 0.0)),
        material=raised_door_paint,
        name="hinge_stile",
    )
    door.visual(
        Box((0.046, 0.012, 0.745)),
        origin=Origin(xyz=(0.700, DOOR_THICKNESS / 2.0 + 0.006, 0.0)),
        material=raised_door_paint,
        name="latch_stile",
    )
    for index, zc in enumerate((-0.155, 0.155)):
        door.visual(
            Box((0.500, 0.010, 0.026)),
            origin=Origin(xyz=(0.395, DOOR_THICKNESS / 2.0 + 0.005, zc)),
            material=raised_door_paint,
            name=f"shallow_rib_{index}",
        )

    door.visual(
        Cylinder(radius=0.018, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc_hardware,
        name="center_hinge_barrel",
    )
    door.visual(
        Box((0.075, 0.012, 0.205)),
        origin=Origin(xyz=(0.037, -0.015, 0.0)),
        material=zinc_hardware,
        name="hinge_leaf",
    )

    door.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.665, DOOR_THICKNESS / 2.0 + 0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=latch_black,
        name="latch_boss",
    )
    door.visual(
        Box((0.022, 0.014, 0.150)),
        origin=Origin(xyz=(0.665, DOOR_THICKNESS / 2.0 + 0.033, 0.0)),
        material=latch_black,
        name="latch_paddle",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=equipment_face,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    equipment_face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        equipment_face,
        door,
        elem_a="hinge_pin",
        elem_b="center_hinge_barrel",
        reason="The fixed hinge pin is intentionally captured inside the moving hinge barrel proxy.",
    )

    ctx.check("door_is_single_revolute_panel", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("hinge_axis_is_vertical", abs(hinge.axis[2] - 1.0) < 1e-6, details=f"axis={hinge.axis}")
    ctx.check(
        "hinge_limits_open_outward",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.5,
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            equipment_face,
            axis="y",
            min_gap=0.006,
            max_gap=0.030,
            positive_elem="door_panel",
            negative_elem="service_frame",
            name="closed door sits proud of frame",
        )
        ctx.expect_overlap(
            door,
            equipment_face,
            axes="xz",
            min_overlap=0.60,
            elem_a="door_panel",
            elem_b="service_frame",
            name="closed door covers framed opening",
        )
        ctx.expect_within(
            equipment_face,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="center_hinge_barrel",
            margin=0.001,
            name="hinge pin is centered in moving barrel",
        )
        ctx.expect_overlap(
            equipment_face,
            door,
            axes="z",
            min_overlap=0.25,
            elem_a="hinge_pin",
            elem_b="center_hinge_barrel",
            name="hinge pin passes through moving barrel",
        )
        rest_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    rest_center_y = None if rest_aabb is None else float((rest_aabb[0][1] + rest_aabb[1][1]) * 0.5)
    open_center_y = None if open_aabb is None else float((open_aabb[0][1] + open_aabb[1][1]) * 0.5)
    ctx.check(
        "door_swings_outward",
        rest_center_y is not None and open_center_y is not None and open_center_y > rest_center_y + 0.20,
        details=f"rest_center_y={rest_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
