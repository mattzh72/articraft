from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_microwave")

    stainless = Material("brushed_stainless", rgba=(0.66, 0.67, 0.64, 1.0))
    dark_trim = Material("black_glass_trim", rgba=(0.015, 0.018, 0.020, 1.0))
    cavity_mat = Material("dark_oven_cavity", rgba=(0.055, 0.057, 0.060, 1.0))
    glass = Material("smoky_transparent_glass", rgba=(0.25, 0.42, 0.50, 0.36))
    soft_black = Material("soft_black_plastic", rgba=(0.02, 0.022, 0.024, 1.0))
    white_mark = Material("white_tick_marks", rgba=(0.92, 0.92, 0.86, 1.0))

    cabinet = model.part("cabinet")

    width = 0.56
    depth = 0.42
    height = 0.34
    wall = 0.025
    front_y = -depth / 2.0
    back_y = depth / 2.0

    # Structural outer shell: a single fixed housing with an open front and an
    # inset cooking cavity above a bottom control strip.
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=stainless,
        name="bottom_plate",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=stainless,
        name="top_plate",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, height / 2.0)),
        material=stainless,
        name="rear_shell",
    )

    # Fixed dark oven liner visible through the smoked door glass.
    cabinet.visual(
        Box((0.48, 0.37, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, 0.105)),
        material=cavity_mat,
        name="cavity_floor",
    )
    cabinet.visual(
        Box((0.48, 0.012, 0.205)),
        origin=Origin(xyz=(0.0, 0.179, 0.205)),
        material=cavity_mat,
        name="cavity_back",
    )
    cabinet.visual(
        Box((0.010, 0.37, 0.205)),
        origin=Origin(xyz=(-0.24, 0.000, 0.205)),
        material=cavity_mat,
        name="cavity_wall_0",
    )
    cabinet.visual(
        Box((0.010, 0.37, 0.205)),
        origin=Origin(xyz=(0.24, 0.000, 0.205)),
        material=cavity_mat,
        name="cavity_wall_1",
    )
    cabinet.visual(
        Box((0.48, 0.37, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, 0.305)),
        material=cavity_mat,
        name="cavity_ceiling",
    )

    # Front face frame, bottom-row control panel, and a few subtle ventilation
    # slots so the housing reads like a commercial appliance rather than a box.
    cabinet.visual(
        Box((width, 0.016, 0.088)),
        origin=Origin(xyz=(0.0, front_y - 0.008, 0.061)),
        material=dark_trim,
        name="control_panel",
    )
    cabinet.visual(
        Box((width, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, front_y - 0.008, 0.105)),
        material=stainless,
        name="lower_door_rail",
    )
    cabinet.visual(
        Box((width, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, front_y - 0.008, 0.323)),
        material=stainless,
        name="upper_door_rail",
    )
    cabinet.visual(
        Box((0.026, 0.016, 0.218)),
        origin=Origin(xyz=(-0.267, front_y - 0.008, 0.214)),
        material=stainless,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.026, 0.016, 0.218)),
        origin=Origin(xyz=(0.267, front_y - 0.008, 0.214)),
        material=stainless,
        name="front_stile_1",
    )
    for i, x in enumerate((-0.205, -0.165, -0.125, 0.125, 0.165, 0.205)):
        cabinet.visual(
            Box((0.024, 0.004, 0.004)),
            origin=Origin(xyz=(x, front_y - 0.018, 0.088)),
            material=white_mark,
            name=f"panel_tick_{i}",
        )
    for i, z in enumerate((0.205, 0.225, 0.245, 0.265)):
        cabinet.visual(
            Box((0.003, 0.145, 0.006)),
            origin=Origin(xyz=(width / 2.0 - 0.002, -0.015, z)),
            material=dark_trim,
            name=f"side_vent_{i}",
        )

    # Fixed center spindle that the glass turntable clips around and bears on.
    cabinet.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.120)),
        material=soft_black,
        name="turntable_spindle",
    )

    # A visible hinge pin on the left side visually explains the door motion.
    cabinet.visual(
        Cylinder(radius=0.006, length=0.240),
        origin=Origin(xyz=(-0.255, front_y - 0.035, 0.215)),
        material=stainless,
        name="hinge_pin",
    )
    cabinet.visual(
        Box((0.026, 0.028, 0.020)),
        origin=Origin(xyz=(-0.260, front_y - 0.026, 0.101)),
        material=stainless,
        name="lower_hinge_block",
    )
    cabinet.visual(
        Box((0.026, 0.028, 0.020)),
        origin=Origin(xyz=(-0.260, front_y - 0.026, 0.329)),
        material=stainless,
        name="upper_hinge_block",
    )

    door = model.part("door")
    door.visual(
        Box((0.475, 0.030, 0.216)),
        origin=Origin(xyz=(0.2625, 0.0, 0.0)),
        material=dark_trim,
        name="door_panel",
    )
    door.visual(
        Box((0.375, 0.004, 0.140)),
        origin=Origin(xyz=(0.220, -0.0170, 0.000)),
        material=glass,
        name="viewing_glass",
    )
    door.visual(
        Box((0.475, 0.008, 0.018)),
        origin=Origin(xyz=(0.2625, -0.019, 0.099)),
        material=stainless,
        name="door_top_trim",
    )
    door.visual(
        Box((0.475, 0.008, 0.018)),
        origin=Origin(xyz=(0.2625, -0.019, -0.099)),
        material=stainless,
        name="door_bottom_trim",
    )
    door.visual(
        Box((0.014, 0.026, 0.216)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=stainless,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.026, 0.008, 0.216)),
        origin=Origin(xyz=(0.487, -0.019, 0.0)),
        material=stainless,
        name="latch_side_trim",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.172),
        origin=Origin(xyz=(0.440, -0.055, 0.0)),
        material=stainless,
        name="pull_grip",
    )
    door.visual(
        Box((0.044, 0.043, 0.020)),
        origin=Origin(xyz=(0.440, -0.036, 0.064)),
        material=stainless,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.044, 0.043, 0.020)),
        origin=Origin(xyz=(0.440, -0.036, -0.064)),
        material=stainless,
        name="handle_mount_1",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.255, front_y - 0.035, 0.215)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="glass_disk",
    )
    turntable.visual(
        mesh_from_geometry(TorusGeometry(radius=0.152, tube=0.004), "turntable_outer_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass,
        name="outer_rim",
    )
    turntable.visual(
        mesh_from_geometry(TorusGeometry(radius=0.022, tube=0.004), "turntable_clip_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=soft_black,
        name="clip_ring",
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=turntable,
        origin=Origin(xyz=(0.0, -0.015, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.066,
            0.032,
            body_style="skirted",
            top_diameter=0.052,
            skirt=KnobSkirt(0.076, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=24, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "selector_knob",
    )
    for idx, x in enumerate((-0.125, 0.125)):
        knob = model.part(f"selector_knob_{idx}")
        knob.visual(
            knob_mesh,
            origin=Origin(),
            material=soft_black,
            name="knob_cap",
        )
        model.articulation(
            f"knob_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x, front_y - 0.016, 0.057), rpy=(pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("door_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")

    # The hinge pin is intentionally captured inside the rotating hinge barrel.
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The fixed hinge pin is intentionally nested through the rotating door hinge barrel.",
    )
    ctx.expect_overlap(
        cabinet,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.18,
        name="hinge pin spans the door barrel",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            min_gap=0.002,
            max_gap=0.008,
            positive_elem="control_panel",
            negative_elem="door_panel",
            name="closed door sits just proud of the front panel",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="viewing_glass",
            elem_b="cavity_back",
            min_overlap=0.12,
            name="door glass covers the fixed oven cavity",
        )
        closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 1.15}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward on the vertical side hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.25,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_gap(
        turntable,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.0015,
        positive_elem="glass_disk",
        negative_elem="turntable_spindle",
        name="glass turntable bears on the center spindle",
    )
    ctx.expect_within(
        turntable,
        cabinet,
        axes="xy",
        inner_elem="glass_disk",
        outer_elem="cavity_floor",
        margin=0.003,
        name="turntable remains inside the oven floor footprint",
    )
    with ctx.pose({turntable_spin: pi / 2.0}):
        ctx.expect_within(
            turntable,
            cabinet,
            axes="xy",
            inner_elem="glass_disk",
            outer_elem="cavity_floor",
            margin=0.003,
            name="rotated turntable remains seated in the cavity",
        )

    for idx in range(2):
        knob = object_model.get_part(f"selector_knob_{idx}")
        joint = object_model.get_articulation(f"knob_{idx}_spin")
        ctx.expect_gap(
            cabinet,
            knob,
            axis="y",
            min_gap=-0.0005,
            max_gap=0.0015,
            positive_elem="control_panel",
            negative_elem="knob_cap",
            name=f"selector knob {idx} is mounted flush to the lower panel",
        )
        with ctx.pose({joint: pi / 2.0}):
            ctx.expect_gap(
                cabinet,
                knob,
                axis="y",
                min_gap=-0.0005,
                max_gap=0.0015,
                positive_elem="control_panel",
                negative_elem="knob_cap",
                name=f"selector knob {idx} rotates on its front-panel axis",
            )

    return ctx.report()


object_model = build_object_model()
