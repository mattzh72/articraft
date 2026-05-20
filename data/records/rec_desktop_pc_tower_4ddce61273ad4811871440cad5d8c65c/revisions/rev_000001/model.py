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


TRAY_COUNT = 5
TRAY_PITCH = 0.128
TRAY_Z0 = 0.210
TRAY_ZS = tuple(TRAY_Z0 + i * TRAY_PITCH for i in range(TRAY_COUNT))
TRAY_TRAVEL = 0.220


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nas_storage_tower_chassis")

    dark_metal = model.material("dark_metal", rgba=(0.055, 0.060, 0.065, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.010, 0.012, 0.014, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.090, 0.095, 0.105, 1.0))
    tray_trim = model.material("tray_trim", rgba=(0.020, 0.023, 0.027, 1.0))
    smoked_door = model.material("smoked_door", rgba=(0.035, 0.045, 0.055, 0.42))
    blue_led = model.material("blue_led", rgba=(0.1, 0.45, 1.0, 1.0))
    amber_led = model.material("amber_led", rgba=(1.0, 0.58, 0.10, 1.0))

    chassis = model.part("chassis")

    # A tall, narrow tower shell: separate walls leave the drive bay visibly hollow.
    chassis.visual(Box((0.018, 0.550, 0.900)), origin=Origin(xyz=(-0.141, 0.000, 0.450)), material=dark_metal, name="side_wall_0")
    chassis.visual(Box((0.018, 0.550, 0.900)), origin=Origin(xyz=(0.141, 0.000, 0.450)), material=dark_metal, name="side_wall_1")
    chassis.visual(Box((0.300, 0.550, 0.026)), origin=Origin(xyz=(0.000, 0.000, 0.887)), material=dark_metal, name="top_wall")
    chassis.visual(Box((0.300, 0.550, 0.026)), origin=Origin(xyz=(0.000, 0.000, 0.013)), material=dark_metal, name="bottom_wall")
    chassis.visual(Box((0.300, 0.026, 0.900)), origin=Origin(xyz=(0.000, 0.262, 0.450)), material=dark_metal, name="rear_wall")

    chassis.visual(Box((0.300, 0.020, 0.060)), origin=Origin(xyz=(0.000, -0.265, 0.095)), material=black_plastic, name="front_sill")
    chassis.visual(Box((0.300, 0.020, 0.060)), origin=Origin(xyz=(0.000, -0.265, 0.815)), material=black_plastic, name="front_header")
    chassis.visual(Box((0.020, 0.025, 0.760)), origin=Origin(xyz=(-0.140, -0.264, 0.455)), material=black_plastic, name="front_post_0")
    chassis.visual(Box((0.020, 0.025, 0.760)), origin=Origin(xyz=(0.140, -0.264, 0.455)), material=black_plastic, name="front_post_1")

    # Front drive-bay separators and side-mounted guide rails.
    for i, z in enumerate(TRAY_ZS):
        chassis.visual(
            Box((0.012, 0.420, 0.014)),
            origin=Origin(xyz=(-0.126, -0.045, z - 0.066)),
            material=satin_steel,
            name=f"guide_{i}_0",
        )
        chassis.visual(
            Box((0.012, 0.420, 0.014)),
            origin=Origin(xyz=(0.126, -0.045, z - 0.066)),
            material=satin_steel,
            name=f"guide_{i}_1",
        )
        if i < TRAY_COUNT - 1:
            chassis.visual(
                Box((0.270, 0.018, 0.010)),
                origin=Origin(xyz=(0.000, -0.264, z + TRAY_PITCH / 2.0)),
                material=black_plastic,
                name=f"bay_divider_{i}",
            )

    # Rear vent ribs and status light strip make the tower read as a NAS chassis.
    for i, z in enumerate((0.245, 0.285, 0.325, 0.365, 0.405, 0.445, 0.485, 0.525, 0.565, 0.605)):
        chassis.visual(
            Box((0.004, 0.112, 0.010)),
            origin=Origin(xyz=(0.151, 0.035, z)),
            material=black_plastic,
            name=f"side_vent_{i}",
        )
    for i, z in enumerate((0.705, 0.735, 0.765)):
        chassis.visual(
            Box((0.012, 0.005, 0.012)),
            origin=Origin(xyz=(0.139, -0.279, z)),
            material=blue_led if i != 1 else amber_led,
            name=f"status_light_{i}",
        )
    chassis.visual(
        Box((0.018, 0.007, 0.100)),
        origin=Origin(xyz=(0.139, -0.275, 0.735)),
        material=black_plastic,
        name="status_strip",
    )

    # Chassis-side hinge knuckles with mounting leaves fixed into the front post.
    chassis.visual(Cylinder(radius=0.003, length=0.780), origin=Origin(xyz=(-0.151, -0.305, 0.450)), material=satin_steel, name="hinge_pin")
    for i, z in enumerate((0.330, 0.570)):
        chassis.visual(Cylinder(radius=0.008, length=0.110), origin=Origin(xyz=(-0.151, -0.305, z)), material=satin_steel, name=f"hinge_knuckle_{i}")
        chassis.visual(Box((0.016, 0.034, 0.070)), origin=Origin(xyz=(-0.157, -0.288, z)), material=satin_steel, name=f"hinge_leaf_{i}")

    door = model.part("door")
    door.visual(Box((0.274, 0.016, 0.765)), origin=Origin(xyz=(0.148, 0.000, 0.000)), material=smoked_door, name="door_panel")
    door.visual(Box((0.290, 0.010, 0.026)), origin=Origin(xyz=(0.145, -0.013, 0.382)), material=black_plastic, name="door_top_rail")
    door.visual(Box((0.290, 0.010, 0.026)), origin=Origin(xyz=(0.145, -0.013, -0.382)), material=black_plastic, name="door_bottom_rail")
    door.visual(Box((0.026, 0.010, 0.765)), origin=Origin(xyz=(0.014, -0.013, 0.000)), material=black_plastic, name="door_hinge_stile")
    door.visual(Box((0.026, 0.010, 0.765)), origin=Origin(xyz=(0.274, -0.013, 0.000)), material=black_plastic, name="door_pull_stile")
    door.visual(Box((0.018, 0.014, 0.220)), origin=Origin(xyz=(0.253, -0.022, 0.000)), material=satin_steel, name="front_handle")
    for i, z in enumerate((-0.240, 0.000, 0.240)):
        door.visual(Cylinder(radius=0.0075, length=0.105), origin=Origin(xyz=(0.000, 0.000, z)), material=satin_steel, name=f"door_knuckle_{i}")
        door.visual(Box((0.026, 0.012, 0.072)), origin=Origin(xyz=(0.012, -0.001, z)), material=satin_steel, name=f"door_leaf_{i}")

    model.articulation(
        "chassis_to_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(-0.151, -0.305, 0.450)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    for i, z in enumerate(TRAY_ZS):
        tray = model.part(f"tray_{i}")
        tray.visual(Box((0.225, 0.016, 0.105)), origin=Origin(xyz=(0.000, 0.000, 0.000)), material=tray_plastic, name="fascia")
        tray.visual(Box((0.226, 0.350, 0.082)), origin=Origin(xyz=(0.000, 0.183, 0.000)), material=dark_metal, name="sled")
        tray.visual(Box((0.012, 0.340, 0.020)), origin=Origin(xyz=(-0.118, 0.190, -0.0495)), material=satin_steel, name="runner_0")
        tray.visual(Box((0.012, 0.340, 0.020)), origin=Origin(xyz=(0.118, 0.190, -0.0495)), material=satin_steel, name="runner_1")
        tray.visual(Box((0.105, 0.006, 0.018)), origin=Origin(xyz=(-0.020, -0.005, 0.025)), material=tray_trim, name="pull_recess")
        tray.visual(Box((0.018, 0.006, 0.030)), origin=Origin(xyz=(0.088, -0.005, 0.028)), material=tray_trim, name="release_tab")
        tray.visual(
            Box((0.012, 0.006, 0.012)),
            origin=Origin(xyz=(-0.095, -0.005, 0.030)),
            material=blue_led if i % 2 == 0 else amber_led,
            name="activity_light",
        )

        model.articulation(
            f"chassis_to_tray_{i}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=tray,
            origin=Origin(xyz=(0.000, -0.287, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=TRAY_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("chassis_to_door")

    for i in range(3):
        ctx.allow_overlap(
            chassis,
            door,
            elem_a="hinge_pin",
            elem_b=f"door_knuckle_{i}",
            reason="The continuous hinge pin is intentionally captured inside each door hinge knuckle.",
        )
        ctx.expect_overlap(
            chassis,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=f"door_knuckle_{i}",
            min_overlap=0.080,
            name=f"hinge pin passes through door knuckle {i}",
        )

    ctx.expect_gap(
        chassis,
        door,
        axis="y",
        positive_elem="front_header",
        negative_elem="door_panel",
        min_gap=0.010,
        name="closed door stands proud of chassis front",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.20}):
        opened_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "front door swings outward on vertical hinge",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[0][1] < closed_panel[0][1] - 0.050,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    for i in range(TRAY_COUNT):
        tray = object_model.get_part(f"tray_{i}")
        slide = object_model.get_articulation(f"chassis_to_tray_{i}")
        ctx.expect_within(
            tray,
            chassis,
            axes="xz",
            inner_elem="sled",
            margin=0.001,
            name=f"tray {i} sled fits within tower bay",
        )
        ctx.expect_overlap(
            tray,
            chassis,
            axes="y",
            elem_a="sled",
            elem_b=f"guide_{i}_0",
            min_overlap=0.250,
            name=f"tray {i} is deeply engaged with guide rail",
        )
        rest_pos = ctx.part_world_position(tray)
        with ctx.pose({slide: TRAY_TRAVEL}):
            ctx.expect_overlap(
                tray,
                chassis,
                axes="y",
                elem_a="sled",
                elem_b=f"guide_{i}_0",
                min_overlap=0.080,
                name=f"tray {i} remains captured when extended",
            )
            extended_pos = ctx.part_world_position(tray)
        ctx.check(
            f"tray {i} slides outward from the front",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.150,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
