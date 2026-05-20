from __future__ import annotations

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
    model = ArticulatedObject(name="tower_rackmount_server")

    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("black_powdercoat", rgba=(0.01, 0.012, 0.014, 1.0))
    rail_metal = model.material("brushed_slide_metal", rgba=(0.52, 0.54, 0.55, 1.0))
    smoked = model.material("smoked_perforated_door", rgba=(0.02, 0.025, 0.03, 0.45))
    drawer_face = model.material("drive_caddy_black", rgba=(0.025, 0.027, 0.03, 1.0))
    green_led = model.material("green_status_lens", rgba=(0.1, 0.85, 0.24, 1.0))
    amber_led = model.material("amber_status_lens", rgba=(1.0, 0.58, 0.08, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.004, 0.004, 0.004, 1.0))

    # Tower orientation for a rack chassis: tall vertical posture, narrow
    # former rack height as width, and the long rack depth running front-back.
    depth = 0.55
    width = 0.26
    height = 0.72
    wall = 0.018
    front_x = 0.0
    rear_x = -depth

    chassis = model.part("chassis")

    # A hollow metal shell assembled from touching/overlapping sheet panels.
    chassis.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(-depth / 2, -width / 2 + wall / 2, height / 2)),
        material=dark_steel,
        name="side_panel_0",
    )
    chassis.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(-depth / 2, width / 2 - wall / 2, height / 2)),
        material=dark_steel,
        name="side_panel_1",
    )
    chassis.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(-depth / 2, 0.0, wall / 2)),
        material=dark_steel,
        name="bottom_panel",
    )
    chassis.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(-depth / 2, 0.0, height - wall / 2)),
        material=dark_steel,
        name="top_panel",
    )
    chassis.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(rear_x + wall / 2, 0.0, height / 2)),
        material=dark_steel,
        name="rear_panel",
    )

    # Front rack flange and drive-bay frame, leaving the front cavity open.
    for y, name in ((-width / 2 + wall / 2, "front_upright_0"), (width / 2 - wall / 2, "front_upright_1")):
        chassis.visual(
            Box((0.032, 0.028, height)),
            origin=Origin(xyz=(-0.016, y, height / 2)),
            material=black,
            name=name,
        )
    for z, name in ((0.225, "lower_bay_lip"), (0.350, "center_bay_lip"), (0.475, "upper_bay_lip")):
        chassis.visual(
            Box((0.030, width - 0.032, 0.014)),
            origin=Origin(xyz=(-0.015, 0.0, z)),
            material=black,
            name=name,
        )

    # Mid-shelf and fixed slide rails that physically support the two drawers.
    chassis.visual(
        Box((depth - 0.04, width - 0.028, 0.014)),
        origin=Origin(xyz=(-0.28, 0.0, 0.350)),
        material=dark_steel,
        name="bay_shelf",
    )
    drawer_zs = (0.290, 0.410)
    for idx, z in enumerate(drawer_zs):
        for side, y in (("left", -0.103), ("right", 0.103)):
            chassis.visual(
                Box((0.455, 0.018, 0.014)),
                origin=Origin(xyz=(-0.235, y, z)),
                material=rail_metal,
                name=f"bay_rail_{idx}_{side}",
            )

    # Side ventilation louvers and bottom feet reinforce the tower reading.
    for i, z in enumerate((0.535, 0.565, 0.595, 0.625)):
        chassis.visual(
            Box((0.220, 0.006, 0.012)),
            origin=Origin(xyz=(-0.300, width / 2 + 0.001, z)),
            material=black,
            name=f"side_louver_{i}",
        )
    for i, (x, y) in enumerate(((-0.46, -0.085), (-0.46, 0.085), (-0.09, -0.085), (-0.09, 0.085))):
        chassis.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(x, y, -0.007)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Alternating exposed hinge knuckles mounted to the chassis side flange.
    hinge_x = front_x + 0.018
    hinge_y = -width / 2 - 0.014
    chassis.visual(
        Cylinder(radius=0.004, length=0.610),
        origin=Origin(xyz=(hinge_x, hinge_y, height / 2)),
        material=rail_metal,
        name="hinge_pin",
    )
    for i, z in enumerate((0.105, 0.360, 0.615)):
        chassis.visual(
            Cylinder(radius=0.0085, length=0.090),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=rail_metal,
            name=f"fixed_hinge_knuckle_{i}",
        )
        chassis.visual(
            Box((0.040, 0.024, 0.080)),
            origin=Origin(xyz=(0.0, -width / 2 - 0.010, z)),
            material=rail_metal,
            name=f"fixed_hinge_leaf_{i}",
        )

    # Front door: frame, smoked mesh/glass insert, latch handle, and moving
    # hinge knuckles.  Its part frame is exactly the hinge line.
    door = model.part("front_door")
    door_width = 0.280
    door_height = 0.665
    door_thick = 0.022
    door_y_min = 0.022
    door_center_y = door_y_min + door_width / 2

    door.visual(
        Box((door_thick, door_width, door_height)),
        origin=Origin(xyz=(0.0, door_center_y, 0.0)),
        material=smoked,
        name="panel",
    )
    # Rigid rectangular door frame over the transparent panel.
    for name, y in (("hinge_stile", door_y_min + 0.010), ("latch_stile", door_y_min + door_width - 0.010)):
        door.visual(
            Box((door_thick + 0.006, 0.020, door_height + 0.020)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=black,
            name=name,
        )
    for name, z in (("top_rail", door_height / 2 - 0.010), ("bottom_rail", -door_height / 2 + 0.010)):
        door.visual(
            Box((door_thick + 0.006, door_width, 0.020)),
            origin=Origin(xyz=(0.0, door_center_y, z)),
            material=black,
            name=name,
        )
    for i, z in enumerate((-0.180, -0.120, -0.060, 0.000, 0.060, 0.120, 0.180)):
        door.visual(
            Box((0.006, door_width - 0.070, 0.009)),
            origin=Origin(xyz=(door_thick / 2 + 0.002, door_center_y + 0.010, z)),
            material=black,
            name=f"vent_bar_{i}",
        )
    door.visual(
        Box((0.030, 0.014, 0.135)),
        origin=Origin(xyz=(door_thick / 2 + 0.018, door_y_min + door_width - 0.046, 0.000)),
        material=rail_metal,
        name="handle_bar",
    )
    for z in (-0.050, 0.050):
        door.visual(
            Box((0.022, 0.014, 0.018)),
            origin=Origin(xyz=(door_thick / 2 + 0.006, door_y_min + door_width - 0.046, z)),
            material=rail_metal,
            name=f"handle_post_{z:+.2f}",
        )
    for i, z in enumerate((-0.140, 0.140)):
        door.visual(
            Cylinder(radius=0.0085, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rail_metal,
            name=f"door_hinge_knuckle_{i}",
        )
        door.visual(
            Box((0.014, 0.020, 0.090)),
            origin=Origin(xyz=(0.0, 0.014, z)),
            material=rail_metal,
            name=f"door_hinge_leaf_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, height / 2)),
        # The closed door extends from the hinge toward +Y; -Z makes positive
        # motion swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.85),
    )

    def add_drawer(idx: int, z: float):
        drawer = model.part(f"drive_drawer_{idx}")
        drawer.visual(
            Box((0.450, 0.170, 0.070)),
            origin=Origin(xyz=(-0.230, 0.0, 0.0)),
            material=dark_steel,
            name="tray",
        )
        drawer.visual(
            Box((0.022, 0.194, 0.092)),
            origin=Origin(xyz=(-0.016, 0.0, 0.0)),
            material=drawer_face,
            name="front_face",
        )
        # Caddy side flanges engage the fixed rails without intersecting them.
        for side, y in (("left", -0.090), ("right", 0.090)):
            drawer.visual(
                Box((0.420, 0.006, 0.058)),
                origin=Origin(xyz=(-0.235, y, 0.0)),
                material=rail_metal,
                name=f"side_flange_{side}",
            )
        # Fold-out looking pull handle, modeled fixed to the caddy face.
        drawer.visual(
            Box((0.008, 0.118, 0.012)),
            origin=Origin(xyz=(-0.002, 0.0, -0.012)),
            material=rail_metal,
            name="pull_bar",
        )
        for y in (-0.050, 0.050):
            drawer.visual(
                Box((0.012, 0.010, 0.034)),
                origin=Origin(xyz=(-0.008, y, -0.002)),
                material=rail_metal,
                name=f"pull_post_{y:+.2f}",
            )
        # Drive health/activity indicators are flush lenses, not controls.
        drawer.visual(
            Box((0.004, 0.014, 0.014)),
            origin=Origin(xyz=(-0.004, -0.072, 0.024)),
            material=green_led,
            name="status_lens",
        )
        drawer.visual(
            Box((0.004, 0.014, 0.014)),
            origin=Origin(xyz=(-0.004, -0.052, 0.024)),
            material=amber_led,
            name="activity_lens",
        )
        joint = model.articulation(
            f"drawer_rail_{idx}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=drawer,
            origin=Origin(xyz=(front_x, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.340),
        )
        return drawer, joint

    add_drawer(0, drawer_zs[0])
    add_drawer(1, drawer_zs[1])

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("front_door")
    drawer_0 = object_model.get_part("drive_drawer_0")
    drawer_1 = object_model.get_part("drive_drawer_1")
    door_hinge = object_model.get_articulation("door_hinge")
    rail_0 = object_model.get_articulation("drawer_rail_0")
    rail_1 = object_model.get_articulation("drawer_rail_1")

    for idx in (0, 1):
        ctx.allow_overlap(
            chassis,
            door,
            elem_a="hinge_pin",
            elem_b=f"door_hinge_knuckle_{idx}",
            reason="The fixed hinge pin intentionally passes through the moving door knuckle.",
        )
        ctx.expect_within(
            chassis,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=f"door_hinge_knuckle_{idx}",
            margin=0.0005,
            name=f"hinge pin centered in door knuckle {idx}",
        )
        ctx.expect_overlap(
            chassis,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=f"door_hinge_knuckle_{idx}",
            min_overlap=0.10,
            name=f"hinge pin captured by door knuckle {idx}",
        )

    # Closed door sits proud of the drive faces instead of intersecting them.
    ctx.expect_gap(
        door,
        drawer_0,
        axis="x",
        min_gap=0.003,
        positive_elem="panel",
        negative_elem="pull_bar",
        name="closed door clears lower drive handle",
    )
    ctx.expect_gap(
        door,
        drawer_1,
        axis="x",
        min_gap=0.003,
        positive_elem="panel",
        negative_elem="pull_bar",
        name="closed door clears upper drive handle",
    )

    # Both hot-swap trays remain captured by the chassis slide rails.
    for drawer, idx in ((drawer_0, 0), (drawer_1, 1)):
        ctx.expect_within(
            drawer,
            chassis,
            axes="yz",
            margin=0.002,
            inner_elem="tray",
            name=f"drawer {idx} tray remains inside chassis envelope",
        )
        ctx.expect_overlap(
            drawer,
            chassis,
            axes="x",
            min_overlap=0.42,
            elem_a="tray",
            elem_b=f"bay_rail_{idx}_left",
            name=f"drawer {idx} seated rail engagement",
        )

    rest_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: 1.45}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    ctx.check(
        "front door swings outward",
        rest_handle is not None
        and open_handle is not None
        and open_handle[1][0] > rest_handle[1][0] + 0.12,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    for drawer, joint, idx in ((drawer_0, rail_0, 0), (drawer_1, rail_1, 1)):
        rest_front = ctx.part_element_world_aabb(drawer, elem="front_face")
        with ctx.pose({door_hinge: 1.45, joint: 0.340}):
            extended_front = ctx.part_element_world_aabb(drawer, elem="front_face")
            ctx.expect_overlap(
                drawer,
                chassis,
                axes="x",
                min_overlap=0.10,
                elem_a="tray",
                elem_b=f"bay_rail_{idx}_left",
                name=f"drawer {idx} retained at full extension",
            )
        ctx.check(
            f"drawer {idx} extends forward on rails",
            rest_front is not None
            and extended_front is not None
            and extended_front[1][0] > rest_front[1][0] + 0.30,
            details=f"rest_front={rest_front}, extended_front={extended_front}",
        )

    return ctx.report()


object_model = build_object_model()
