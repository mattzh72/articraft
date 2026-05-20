from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_4u_desktop_chassis")

    # A 4U rack chassis is about 7 in / 0.178 m tall and 19 in / 0.483 m wide.
    width = 0.483
    depth = 0.430
    height = 0.178
    sheet = 0.004

    metal = model.material("dark_zinc_sheet", rgba=(0.18, 0.19, 0.20, 1.0))
    edge_metal = model.material("worn_black_edge", rgba=(0.05, 0.055, 0.060, 1.0))
    plastic = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    vent_black = model.material("shadow_vent_black", rgba=(0.0, 0.0, 0.0, 1.0))
    hinge_steel = model.material("brushed_hinge_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    label_blue = model.material("status_lens_blue", rgba=(0.05, 0.25, 0.85, 1.0))

    chassis = model.part("chassis")

    # Sheet-metal shell: bottom pan, two side walls, rear wall, front frame, and
    # narrow top rails.  The center top bay is intentionally open for access.
    chassis.visual(
        Box((depth, width, sheet)),
        origin=Origin(xyz=(0.0, 0.0, sheet / 2.0)),
        material=metal,
        name="bottom_pan",
    )
    for y, name, rail_name in (
        (-(width / 2.0 - sheet / 2.0), "side_wall_0", "top_rail_0"),
        ((width / 2.0 - sheet / 2.0), "side_wall_1", "top_rail_1"),
    ):
        chassis.visual(
            Box((depth, sheet, height)),
            origin=Origin(xyz=(0.0, y, height / 2.0)),
            material=metal,
            name=name,
        )
        chassis.visual(
            Box((depth - 0.018, 0.018, sheet)),
            origin=Origin(xyz=(0.006, y * 0.96, height + sheet / 2.0)),
            material=edge_metal,
            name=rail_name,
        )

    chassis.visual(
        Box((sheet, width, height)),
        origin=Origin(xyz=(depth / 2.0 - sheet / 2.0, 0.0, height / 2.0)),
        material=metal,
        name="rear_panel",
    )
    chassis.visual(
        Box((sheet, width - 0.020, 0.018)),
        origin=Origin(xyz=(-depth / 2.0 + sheet / 2.0, 0.0, 0.009)),
        material=edge_metal,
        name="front_frame_bottom",
    )
    chassis.visual(
        Box((sheet, width - 0.020, 0.018)),
        origin=Origin(xyz=(-depth / 2.0 + sheet / 2.0, 0.0, height - 0.009)),
        material=edge_metal,
        name="front_frame_top",
    )
    for y, name in (
        (-(width / 2.0 - 0.012), "front_jamb_0"),
        ((width / 2.0 - 0.012), "front_jamb_1"),
    ):
        chassis.visual(
            Box((sheet, 0.024, height)),
            origin=Origin(xyz=(-depth / 2.0 + sheet / 2.0, y, height / 2.0)),
            material=edge_metal,
            name=name,
        )

    # Rack ears outside the bezel footprint make the desktop chassis read as
    # rack-mountable without intersecting the hinged front door.
    for y, name in (
        (-(width / 2.0 + 0.010), "rack_ear_0"),
        ((width / 2.0 + 0.010), "rack_ear_1"),
    ):
        chassis.visual(
            Box((0.020, 0.024, height)),
            origin=Origin(xyz=(-depth / 2.0 - 0.002, y, height / 2.0)),
            material=metal,
            name=name,
        )
        for z in (0.040, height - 0.040):
            chassis.visual(
                Cylinder(radius=0.0055, length=0.0015),
                origin=Origin(
                    xyz=(-depth / 2.0 - 0.0125, y, z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=vent_black,
                name=f"{name}_screw_shadow_{int(z * 1000)}",
            )

    # Fixed rear hinge saddles below the access-panel pins.
    hinge_x = depth / 2.0 - 0.025
    hinge_z = height + 0.015
    for y, name in ((-0.135, "rear_hinge_saddle_0"), (0.135, "rear_hinge_saddle_1")):
        chassis.visual(
            Box((0.018, 0.082, 0.010)),
            origin=Origin(xyz=(depth / 2.0 - 0.011, y, height + 0.004)),
            material=hinge_steel,
            name=name,
        )

    # A little visible internal structure so the raised top has a purpose.
    chassis.visual(
        Box((0.260, 0.300, 0.004)),
        origin=Origin(xyz=(-0.025, 0.040, sheet + 0.002)),
        material=metal,
        name="motherboard_tray",
    )
    chassis.visual(
        Box((0.090, 0.150, 0.055)),
        origin=Origin(xyz=(-0.125, -0.125, sheet + 0.0275)),
        material=edge_metal,
        name="drive_cage",
    )

    # Hinged front bezel.  Its part frame is the left vertical hinge line; the
    # closed panel spans local +Y and opens outward toward -X for positive q.
    bezel = model.part("front_bezel")
    bezel_width = width - 0.033
    bezel_height = height - 0.022
    bezel_thickness = 0.016
    bezel.visual(
        Box((bezel_thickness, bezel_width, bezel_height)),
        origin=Origin(xyz=(0.0, bezel_width / 2.0, 0.0)),
        material=plastic,
        name="bezel_panel",
    )
    bezel.visual(
        Cylinder(radius=0.0065, length=bezel_height + 0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="front_hinge_pin",
    )
    for z, name in ((0.047, "front_leaf_0"), (-0.047, "front_leaf_1")):
        bezel.visual(
            Box((0.006, 0.040, 0.030)),
            origin=Origin(xyz=(-0.001, 0.015, z)),
            material=hinge_steel,
            name=name,
        )
    # Low, wide service grille and small status lens on the front door.
    bezel.visual(
        Box((0.0025, 0.300, 0.078)),
        origin=Origin(xyz=(-bezel_thickness / 2.0 - 0.001, 0.235, 0.018)),
        material=vent_black,
        name="front_grille_shadow",
    )
    for i in range(9):
        bezel.visual(
            Box((0.003, 0.010, 0.088)),
            origin=Origin(
                xyz=(-bezel_thickness / 2.0 - 0.002, 0.110 + i * 0.030, 0.018)
            ),
            material=edge_metal,
            name=f"grille_bar_{i}",
        )
    bezel.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(
            xyz=(-bezel_thickness / 2.0 - 0.0005, 0.405, -0.050),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=label_blue,
        name="status_lens",
    )

    # Top access panel.  Its frame is the rear hinge axis.  The panel extends
    # forward along local -X and rotates upward about the two visible pins.
    top_panel = model.part("top_panel")
    top_depth = depth - 0.045
    top_width = width - 0.040
    top_thickness = 0.006
    top_center_z = height + sheet + top_thickness / 2.0 - hinge_z
    top_panel.visual(
        Box((top_depth, top_width, top_thickness)),
        origin=Origin(xyz=(-top_depth / 2.0, 0.0, top_center_z)),
        material=metal,
        name="top_skin",
    )
    top_panel.visual(
        Box((top_depth - 0.030, 0.010, 0.004)),
        origin=Origin(xyz=(-(top_depth / 2.0 + 0.004), -top_width / 2.0 + 0.018, top_center_z - 0.005)),
        material=edge_metal,
        name="top_stiffener_0",
    )
    top_panel.visual(
        Box((top_depth - 0.030, 0.010, 0.004)),
        origin=Origin(xyz=(-(top_depth / 2.0 + 0.004), top_width / 2.0 - 0.018, top_center_z - 0.005)),
        material=edge_metal,
        name="top_stiffener_1",
    )
    for y, name in ((-0.135, "rear_hinge_pin_0"), (0.135, "rear_hinge_pin_1")):
        top_panel.visual(
            Cylinder(radius=0.006, length=0.075),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=name,
        )
        top_panel.visual(
            Box((0.024, 0.066, 0.012)),
            origin=Origin(xyz=(-0.008, y, -0.004)),
            material=hinge_steel,
            name=f"hinge_leaf_{name[-1]}",
        )
    # Shallow pressed ribs on the top panel.
    for y, name in ((-0.075, "pressed_rib_0"), (0.075, "pressed_rib_1")):
        top_panel.visual(
            Box((0.270, 0.012, 0.003)),
            origin=Origin(xyz=(-0.205, y, top_center_z + 0.0045)),
            material=edge_metal,
            name=name,
        )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=bezel,
        origin=Origin(
            xyz=(
                -depth / 2.0 - bezel_thickness / 2.0,
                -bezel_width / 2.0,
                height / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    front_bezel = object_model.get_part("front_bezel")
    top_panel = object_model.get_part("top_panel")
    front_hinge = object_model.get_articulation("front_hinge")
    top_hinge = object_model.get_articulation("top_hinge")

    with ctx.pose({front_hinge: 0.0, top_hinge: 0.0}):
        ctx.expect_gap(
            chassis,
            front_bezel,
            axis="x",
            positive_elem="front_frame_top",
            negative_elem="bezel_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="front bezel closes flush to the front frame",
        )
        ctx.expect_overlap(
            front_bezel,
            chassis,
            axes="yz",
            elem_a="bezel_panel",
            elem_b="front_frame_top",
            min_overlap=0.006,
            name="front bezel spans the 4U frame opening",
        )
        ctx.expect_gap(
            top_panel,
            chassis,
            axis="z",
            positive_elem="top_skin",
            negative_elem="top_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="top access panel rests on the top rail",
        )
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="xy",
            elem_a="top_skin",
            elem_b="bottom_pan",
            min_overlap=0.30,
            name="top access panel covers the chassis bay",
        )
        closed_front_aabb = ctx.part_world_aabb(front_bezel)
        closed_top_aabb = ctx.part_world_aabb(top_panel)

    with ctx.pose({front_hinge: 1.2}):
        open_front_aabb = ctx.part_world_aabb(front_bezel)

    with ctx.pose({top_hinge: 1.0}):
        open_top_aabb = ctx.part_world_aabb(top_panel)

    def _coord(aabb, corner_index: int, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return tuple(aabb[corner_index])[axis_index]

    closed_front_min_x = _coord(closed_front_aabb, 0, 0)
    open_front_min_x = _coord(open_front_aabb, 0, 0)
    ctx.check(
        "front bezel hinge opens outward",
        closed_front_min_x is not None
        and open_front_min_x is not None
        and open_front_min_x < closed_front_min_x - 0.25,
        details=f"closed={closed_front_aabb}, open={open_front_aabb}",
    )

    closed_top_max_z = _coord(closed_top_aabb, 1, 2)
    open_top_max_z = _coord(open_top_aabb, 1, 2)
    ctx.check(
        "top access panel lifts above the chassis",
        closed_top_max_z is not None
        and open_top_max_z is not None
        and open_top_max_z > closed_top_max_z + 0.20,
        details=f"closed={closed_top_aabb}, open={open_top_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
