from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


FACE_WIDTH = 0.90
FACE_HEIGHT = 1.20
FACE_CENTER_Z = 0.60
SHELL_THICKNESS = 0.060

OPENING_WIDTH = 0.460
OPENING_HEIGHT = 0.640
FRAME_DEPTH = 0.018
FRAME_Y = -SHELL_THICKNESS / 2.0 - FRAME_DEPTH / 2.0

HINGE_X = -0.275
HINGE_Y = -0.061
HINGE_Z = FACE_CENTER_Z

DOOR_START_X = 0.016
DOOR_WIDTH = 0.520
DOOR_HEIGHT = 0.720
DOOR_THICKNESS = 0.022


def _vertical_plate_with_opening():
    """Thin equipment face plate in the X/Z plane with a through service opening."""
    shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(FACE_WIDTH, FACE_HEIGHT, 0.012, corner_segments=4),
        [rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.018, corner_segments=6)],
        SHELL_THICKNESS,
    )
    # The mesh extrudes along local Z.  Rotate it so extrusion depth becomes world Y
    # and the 2-D profile reads as the visible equipment face in world X/Z.
    shell.rotate_x(math.pi / 2.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_shell = model.material("painted_shell", rgba=(0.43, 0.48, 0.50, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.32, 0.36, 0.38, 1.0))
    reinforced_steel = model.material("reinforced_steel", rgba=(0.26, 0.29, 0.31, 1.0))
    door_paint = model.material("door_paint", rgba=(0.18, 0.33, 0.45, 1.0))
    hinge_metal = model.material("hinge_dark_zinc", rgba=(0.10, 0.11, 0.12, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.015, 0.016, 0.014, 1.0))
    latch_metal = model.material("latch_brushed_metal", rgba=(0.67, 0.68, 0.65, 1.0))
    cavity_dark = model.material("cavity_shadow", rgba=(0.02, 0.024, 0.026, 1.0))

    main_shell = model.part("main_shell")
    main_shell.visual(
        mesh_from_geometry(_vertical_plate_with_opening(), "face_plate"),
        origin=Origin(xyz=(0.0, 0.0, FACE_CENTER_Z)),
        material=painted_shell,
        name="face_plate",
    )

    # Raised frame bars around the service opening.  The hinge side is intentionally
    # omitted here because it is a separate reinforced band mounted to the shell.
    frame_x0 = HINGE_X
    frame_x1 = 0.310
    frame_len = frame_x1 - frame_x0
    frame_cx = (frame_x0 + frame_x1) / 2.0
    top_z = FACE_CENTER_Z + OPENING_HEIGHT / 2.0 + 0.025
    bottom_z = FACE_CENTER_Z - OPENING_HEIGHT / 2.0 - 0.025
    for name, z in (("top_frame", top_z), ("bottom_frame", bottom_z)):
        main_shell.visual(
            Box((frame_len, FRAME_DEPTH, 0.050)),
            origin=Origin(xyz=(frame_cx, FRAME_Y, z)),
            material=frame_paint,
            name=name,
        )
    main_shell.visual(
        Box((0.040, FRAME_DEPTH, OPENING_HEIGHT + 0.100)),
        origin=Origin(xyz=(frame_x1 - 0.020, FRAME_Y, FACE_CENTER_Z)),
        material=frame_paint,
        name="latch_frame",
    )
    main_shell.visual(
        Box((0.020, 0.010, 0.120)),
        origin=Origin(xyz=(0.300, FRAME_Y - 0.006, FACE_CENTER_Z)),
        material=latch_metal,
        name="strike_keeper",
    )

    # Recessed, dark service cavity visible when the door swings open.
    recess_y = SHELL_THICKNESS / 2.0 + 0.025
    main_shell.visual(
        Box((0.012, 0.050, OPENING_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - 0.006, recess_y, FACE_CENTER_Z)),
        material=cavity_dark,
        name="cavity_side_0",
    )
    main_shell.visual(
        Box((0.012, 0.050, OPENING_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + 0.006, recess_y, FACE_CENTER_Z)),
        material=cavity_dark,
        name="cavity_side_1",
    )
    main_shell.visual(
        Box((OPENING_WIDTH + 0.024, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, recess_y, FACE_CENTER_Z + OPENING_HEIGHT / 2.0 + 0.006)),
        material=cavity_dark,
        name="cavity_top",
    )
    main_shell.visual(
        Box((OPENING_WIDTH + 0.024, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, recess_y, FACE_CENTER_Z - OPENING_HEIGHT / 2.0 - 0.006)),
        material=cavity_dark,
        name="cavity_bottom",
    )
    main_shell.visual(
        Box((OPENING_WIDTH, 0.018, OPENING_HEIGHT)),
        origin=Origin(xyz=(0.0, SHELL_THICKNESS / 2.0 + 0.059, FACE_CENTER_Z)),
        material=cavity_dark,
        name="cavity_back",
    )

    hinge_band = model.part("hinge_band")
    hinge_band.visual(
        Box((0.090, 0.018, 0.820)),
        origin=Origin(xyz=(-0.045, 0.022, 0.0)),
        material=reinforced_steel,
        name="band_plate",
    )
    stationary_knuckles = (-0.270, 0.0, 0.270)
    for i, z in enumerate(stationary_knuckles):
        hinge_band.visual(
            Box((0.050, 0.012, 0.120)),
            origin=Origin(xyz=(-0.025, 0.011, z)),
            material=hinge_metal,
            name=f"fixed_leaf_{i}",
        )
        hinge_band.visual(
            Cylinder(radius=0.011, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"fixed_knuckle_{i}",
        )
    hinge_band.visual(
        Cylinder(radius=0.005, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=latch_metal,
        name="hinge_pin",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH / 2.0, 0.0, 0.0)),
        material=door_paint,
        name="door_slab",
    )
    door.visual(
        Box((0.430, 0.004, 0.520)),
        origin=Origin(xyz=(DOOR_START_X + 0.255, -0.013, 0.0)),
        material=door_paint,
        name="pressed_center",
    )
    door.visual(
        Box((0.035, 0.006, 0.670)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH - 0.018, -0.014, 0.0)),
        material=reinforced_steel,
        name="latch_edge",
    )
    door.visual(
        Box((0.450, 0.005, 0.018)),
        origin=Origin(xyz=(DOOR_START_X + 0.255, -0.0135, 0.290)),
        material=reinforced_steel,
        name="top_stiffener",
    )
    door.visual(
        Box((0.450, 0.005, 0.018)),
        origin=Origin(xyz=(DOOR_START_X + 0.255, -0.0135, -0.290)),
        material=reinforced_steel,
        name="bottom_stiffener",
    )
    door.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(
            xyz=(DOOR_START_X + DOOR_WIDTH - 0.065, -0.015, 0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=latch_metal,
        name="quarter_turn_latch",
    )
    door.visual(
        Box((0.038, 0.002, 0.006)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH - 0.065, -0.020, 0.020)),
        material=gasket_black,
        name="latch_slot",
    )
    for i, z in enumerate((-0.135, 0.135)):
        door.visual(
            Box((0.064, 0.006, 0.120)),
            origin=Origin(xyz=(0.0425, 0.0, z)),
            material=hinge_metal,
            name=f"moving_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.011, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"moving_knuckle_{i}",
        )

    model.articulation(
        "shell_to_hinge_band",
        ArticulationType.FIXED,
        parent=main_shell,
        child=hinge_band,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_band,
        child=door,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_shell = object_model.get_part("main_shell")
    hinge_band = object_model.get_part("hinge_band")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.check("main_shell_present", main_shell is not None, "Expected main shell.")
    ctx.check("hinge_band_present", hinge_band is not None, "Expected separate hinge-side reinforcement band.")
    ctx.check("door_present", door is not None, "Expected hinged panel door.")
    ctx.check(
        "single_revolute_door_hinge",
        hinge is not None and hinge.articulation_type == ArticulationType.REVOLUTE,
        "Door must be carried by one revolute side hinge.",
    )
    if main_shell is None or hinge_band is None or door is None or hinge is None:
        return ctx.report()

    for knuckle_name in ("moving_knuckle_0", "moving_knuckle_1"):
        ctx.allow_overlap(
            hinge_band,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The hinge pin is intentionally captured through the moving hinge knuckle.",
        )
        ctx.expect_within(
            hinge_band,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.0,
            name=f"hinge pin centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            hinge_band,
            door,
            axes="z",
            min_overlap=0.100,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            name=f"hinge pin runs through {knuckle_name}",
        )

    ctx.expect_gap(
        main_shell,
        hinge_band,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="face_plate",
        negative_elem="band_plate",
        name="reinforcement band seats on shell face",
    )
    ctx.expect_overlap(
        hinge_band,
        main_shell,
        axes="xz",
        min_overlap=0.050,
        elem_a="band_plate",
        elem_b="face_plate",
        name="reinforcement band overlaps face footprint",
    )
    ctx.expect_gap(
        main_shell,
        door,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="latch_frame",
        negative_elem="door_slab",
        name="closed door clears raised frame",
    )
    ctx.expect_overlap(
        door,
        main_shell,
        axes="xz",
        min_overlap=0.400,
        elem_a="door_slab",
        elem_b="face_plate",
        name="closed door covers service opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({hinge: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward",
        closed_aabb is not None
        and opened_aabb is not None
        and float(opened_aabb[0][1]) < float(closed_aabb[0][1]) - 0.150,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
