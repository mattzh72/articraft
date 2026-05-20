from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_service_access_panel")

    outer_width = 0.52
    outer_height = 0.62
    shell_depth = 0.04
    band_width = 0.09
    latch_jamb_width = 0.09
    top_bar_height = 0.10
    bottom_bar_height = 0.10
    opening_width = outer_width - band_width - latch_jamb_width
    opening_height = outer_height - top_bar_height - bottom_bar_height

    face_gray = model.material("face_gray", rgba=(0.68, 0.70, 0.72, 1.0))
    band_gray = model.material("band_gray", rgba=(0.54, 0.56, 0.60, 1.0))
    door_gray = model.material("door_gray", rgba=(0.82, 0.83, 0.84, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    main_shell = model.part("main_shell")
    top_bar_width = outer_width - band_width
    top_bar_x = (band_width / 2.0)
    top_bar_z = outer_height / 2.0 - top_bar_height / 2.0
    bottom_bar_z = -top_bar_z
    latch_jamb_x = outer_width / 2.0 - latch_jamb_width / 2.0

    main_shell.visual(
        Box((top_bar_width, shell_depth, top_bar_height)),
        origin=Origin(xyz=(top_bar_x, -shell_depth / 2.0, top_bar_z)),
        material=face_gray,
        name="top_bar",
    )
    main_shell.visual(
        Box((top_bar_width, shell_depth, bottom_bar_height)),
        origin=Origin(xyz=(top_bar_x, -shell_depth / 2.0, bottom_bar_z)),
        material=face_gray,
        name="bottom_bar",
    )
    main_shell.visual(
        Box((latch_jamb_width, shell_depth, opening_height)),
        origin=Origin(xyz=(latch_jamb_x, -shell_depth / 2.0, 0.0)),
        material=face_gray,
        name="latch_jamb",
    )
    main_shell.visual(
        Box((top_bar_width, 0.012, outer_height)),
        origin=Origin(xyz=(top_bar_x, -shell_depth + 0.006, 0.0)),
        material=face_gray,
        name="rear_return",
    )

    reinforcement_band = model.part("reinforcement_band")
    band_x = -outer_width / 2.0 + band_width / 2.0
    band_front_y = -shell_depth / 2.0
    reinforcement_band.visual(
        Box((band_width, shell_depth, outer_height)),
        origin=Origin(xyz=(band_x, band_front_y, 0.0)),
        material=band_gray,
        name="band_plate",
    )
    reinforcement_band.visual(
        Box((band_width * 0.62, 0.012, outer_height - 0.06)),
        origin=Origin(xyz=(band_x - 0.01, -shell_depth + 0.006, 0.0)),
        material=band_gray,
        name="band_doubler",
    )

    hinge_axis_x = -opening_width / 2.0 - 0.002
    hinge_axis_y = 0.006
    hinge_leaf_width = 0.014
    hinge_leaf_depth = 0.004
    hinge_barrel_radius = 0.006
    parent_leaf_center_x = hinge_axis_x - hinge_leaf_width / 2.0 + hinge_barrel_radius * 0.3

    reinforcement_band.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, opening_height + 0.03)),
        origin=Origin(
            xyz=(parent_leaf_center_x, hinge_axis_y, 0.0),
        ),
        material=hinge_dark,
        name="hinge_leaf_parent",
    )
    for name, zc, length in (
        ("hinge_barrel_upper", 0.16, 0.09),
        ("hinge_barrel_lower", -0.16, 0.09),
    ):
        reinforcement_band.visual(
            Cylinder(radius=hinge_barrel_radius, length=length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, zc)),
            material=hinge_dark,
            name=name,
        )

    door = model.part("door")
    door_width = opening_width + 0.01
    door_height = opening_height + 0.01
    door_thickness = 0.008
    door_leaf_start_x = 0.010

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                door_leaf_start_x + door_width / 2.0,
                hinge_axis_y,
                0.0,
            )
        ),
        material=door_gray,
        name="door_leaf",
    )
    door.visual(
        Box((0.018, door_thickness * 1.4, door_height - 0.09)),
        origin=Origin(
            xyz=(
                door_leaf_start_x + door_width - 0.010,
                hinge_axis_y + 0.001,
                0.0,
            )
        ),
        material=door_gray,
        name="latch_edge",
    )
    door.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, door_height)),
        origin=Origin(
            xyz=(hinge_leaf_width / 2.0 - hinge_barrel_radius * 0.3, hinge_axis_y, 0.0)
        ),
        material=hinge_dark,
        name="hinge_leaf_door",
    )
    door.visual(
        Cylinder(radius=hinge_barrel_radius, length=0.22),
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.0)),
        material=hinge_dark,
        name="hinge_barrel_center",
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=main_shell,
        child=reinforcement_band,
    )
    model.articulation(
        "band_to_door",
        ArticulationType.REVOLUTE,
        parent=reinforcement_band,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_shell = object_model.get_part("main_shell")
    reinforcement_band = object_model.get_part("reinforcement_band")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("band_to_door")

    ctx.expect_contact(
        reinforcement_band,
        main_shell,
        name="reinforcement band is mounted to the shell",
    )
    ctx.expect_gap(
        door,
        main_shell,
        axis="y",
        positive_elem="door_leaf",
        max_gap=0.012,
        max_penetration=0.0,
        name="closed door sits just proud of the equipment face",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    with ctx.pose({hinge: upper if upper is not None else (pi * 0.5)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")

    closed_center_y = None
    open_center_y = None
    if closed_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    if open_aabb is not None:
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0

    ctx.check(
        "door opens outward about the side hinge",
        closed_center_y is not None
        and open_center_y is not None
        and open_center_y > closed_center_y + 0.12,
        details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
