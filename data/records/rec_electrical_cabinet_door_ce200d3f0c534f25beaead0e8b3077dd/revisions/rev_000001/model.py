from __future__ import annotations

import math

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
    model = ArticulatedObject(name="surface_mount_electrical_enclosure")

    powder_steel = Material("light_gray_powder_coated_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    bare_steel = Material("brushed_hinge_pin_steel", rgba=(0.56, 0.57, 0.55, 1.0))
    dark_rubber = Material("black_neoprene_gasket", rgba=(0.02, 0.02, 0.018, 1.0))
    dark_fastener = Material("dark_zinc_fasteners", rgba=(0.05, 0.055, 0.06, 1.0))

    # Dimensions are typical of a small wall/surface-mounted electrical enclosure.
    box_width = 0.38
    box_height = 0.50
    box_depth = 0.12
    wall = 0.004

    box = model.part("box")
    # Hollow shallow steel tray: a back sheet and four folded side walls.
    box.visual(
        Box((box_width, wall, box_height)),
        origin=Origin(xyz=(0.0, wall / 2.0, 0.0)),
        material=powder_steel,
        name="back_pan",
    )
    box.visual(
        Box((wall, box_depth, box_height)),
        origin=Origin(xyz=(-box_width / 2.0 + wall / 2.0, box_depth / 2.0, 0.0)),
        material=powder_steel,
        name="side_wall_0",
    )
    box.visual(
        Box((wall, box_depth, box_height)),
        origin=Origin(xyz=(box_width / 2.0 - wall / 2.0, box_depth / 2.0, 0.0)),
        material=powder_steel,
        name="side_wall_1",
    )
    box.visual(
        Box((box_width, box_depth, wall)),
        origin=Origin(xyz=(0.0, box_depth / 2.0, box_height / 2.0 - wall / 2.0)),
        material=powder_steel,
        name="top_wall",
    )
    box.visual(
        Box((box_width, box_depth, wall)),
        origin=Origin(xyz=(0.0, box_depth / 2.0, -box_height / 2.0 + wall / 2.0)),
        material=powder_steel,
        name="bottom_wall",
    )
    # Narrow front return lip around the open tray.
    box.visual(
        Box((box_width, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, box_depth - 0.004, box_height / 2.0 - 0.009)),
        material=powder_steel,
        name="front_lip_top",
    )
    box.visual(
        Box((box_width, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, box_depth - 0.004, -box_height / 2.0 + 0.009)),
        material=powder_steel,
        name="front_lip_bottom",
    )
    box.visual(
        Box((0.018, 0.008, box_height)),
        origin=Origin(xyz=(-box_width / 2.0 + 0.009, box_depth - 0.004, 0.0)),
        material=powder_steel,
        name="front_lip_side_0",
    )
    box.visual(
        Box((0.018, 0.008, box_height)),
        origin=Origin(xyz=(box_width / 2.0 - 0.009, box_depth - 0.004, 0.0)),
        material=powder_steel,
        name="front_lip_side_1",
    )
    # Surface-mount ears with dark screw holes on the back flange.
    for x, z, tab_name, hole_name in (
        (-0.12, 0.266, "mount_tab_0", "mount_hole_0"),
        (0.12, 0.266, "mount_tab_1", "mount_hole_1"),
        (-0.12, -0.266, "mount_tab_2", "mount_hole_2"),
        (0.12, -0.266, "mount_tab_3", "mount_hole_3"),
    ):
        box.visual(
            Box((0.07, 0.004, 0.052)),
            origin=Origin(xyz=(x, 0.002, z)),
            material=powder_steel,
            name=tab_name,
        )
        box.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(x, -0.0002, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_fastener,
            name=hole_name,
        )

    hinge_x = -0.215
    hinge_y = 0.132
    hinge_length = 0.52
    # The fixed leaf is welded/screwed to the left side of the steel tray.  The
    # visible pin runs through the two moving door knuckles.
    box.visual(
        Box((0.028, 0.005, 0.47)),
        origin=Origin(xyz=(-0.203, 0.1215, 0.0)),
        material=bare_steel,
        name="hinge_leaf",
    )
    box.visual(
        Cylinder(radius=0.003, length=hinge_length),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=bare_steel,
        name="hinge_pin",
    )
    box.visual(
        Box((0.010, 0.005, 0.120)),
        origin=Origin(xyz=(hinge_x, 0.1265, 0.0)),
        material=bare_steel,
        name="center_knuckle",
    )

    door = model.part("door")
    # Child frame lies on the hinge axis.  In the closed pose the door extends
    # along +X from that line and sits slightly proud of the tray lip.
    door_width = 0.41
    door_height = 0.54
    door_thickness = 0.008
    panel_x = 0.010 + door_width / 2.0
    panel_y = 0.008
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(panel_x, panel_y, 0.0)),
        material=powder_steel,
        name="door_panel",
    )
    # Folded returns and a dark compression gasket on the inside of the door.
    door.visual(
        Box((door_width - 0.028, 0.006, 0.014)),
        origin=Origin(xyz=(panel_x, 0.002, door_height / 2.0 - 0.016)),
        material=powder_steel,
        name="door_return_top",
    )
    door.visual(
        Box((door_width - 0.028, 0.006, 0.014)),
        origin=Origin(xyz=(panel_x, 0.002, -door_height / 2.0 + 0.016)),
        material=powder_steel,
        name="door_return_bottom",
    )
    door.visual(
        Box((0.014, 0.006, door_height - 0.028)),
        origin=Origin(xyz=(0.010 + 0.007, 0.002, 0.0)),
        material=powder_steel,
        name="door_return_hinge",
    )
    door.visual(
        Box((0.014, 0.006, door_height - 0.028)),
        origin=Origin(xyz=(0.010 + door_width - 0.007, 0.002, 0.0)),
        material=powder_steel,
        name="door_return_latch",
    )
    door.visual(
        Box((door_width - 0.070, 0.0025, 0.012)),
        origin=Origin(xyz=(panel_x, 0.003, door_height / 2.0 - 0.035)),
        material=dark_rubber,
        name="gasket_top",
    )
    door.visual(
        Box((door_width - 0.070, 0.0025, 0.012)),
        origin=Origin(xyz=(panel_x, 0.003, -door_height / 2.0 + 0.035)),
        material=dark_rubber,
        name="gasket_bottom",
    )
    door.visual(
        Box((0.012, 0.0025, door_height - 0.070)),
        origin=Origin(xyz=(0.045, 0.003, 0.0)),
        material=dark_rubber,
        name="gasket_hinge",
    )
    door.visual(
        Box((0.012, 0.0025, door_height - 0.070)),
        origin=Origin(xyz=(door_width - 0.025, 0.003, 0.0)),
        material=dark_rubber,
        name="gasket_latch",
    )
    # Two exposed piano-hinge knuckles on the door side.
    for z, name in ((0.155, "top_knuckle"), (-0.155, "bottom_knuckle")):
        door.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bare_steel,
            name=name,
        )
        door.visual(
            Box((0.038, 0.004, 0.095)),
            origin=Origin(xyz=(0.020, 0.008, z)),
            material=bare_steel,
            name=f"{name}_leaf",
        )
    # Four shallow captive screw heads on the full-front door panel.
    for x, z, name in (
        (0.060, 0.205, "door_screw_0"),
        (0.360, 0.205, "door_screw_1"),
        (0.060, -0.205, "door_screw_2"),
        (0.360, -0.205, "door_screw_3"),
    ):
        door.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(x, panel_y + door_thickness / 2.0 + 0.001, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_fastener,
            name=name,
        )

    model.articulation(
        "box_to_door",
        ArticulationType.REVOLUTE,
        parent=box,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box = object_model.get_part("box")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("box_to_door")

    for knuckle in ("top_knuckle", "bottom_knuckle"):
        ctx.allow_overlap(
            box,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The steel hinge pin is intentionally captured inside the door-side piano-hinge knuckle proxy.",
        )
        ctx.expect_within(
            box,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle,
            margin=0.001,
            name=f"{knuckle} surrounds the pin in plan",
        )
        ctx.expect_overlap(
            box,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.090,
            name=f"{knuckle} has retained pin length",
        )

    ctx.expect_overlap(
        door,
        box,
        axes="xz",
        elem_a="door_panel",
        elem_b="back_pan",
        min_overlap=0.35,
        name="full-front door covers the enclosure footprint",
    )
    ctx.expect_gap(
        door,
        box,
        axis="y",
        positive_elem="door_panel",
        negative_elem="front_lip_top",
        min_gap=0.010,
        max_gap=0.020,
        name="closed door sits just proud of front lip",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "revolute hinge swings door outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
