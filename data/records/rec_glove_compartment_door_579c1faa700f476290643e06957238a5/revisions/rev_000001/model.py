from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_vehicle_glove_compartment")

    dash_plastic = model.material("charcoal_molded_plastic", color=(0.035, 0.038, 0.040, 1.0))
    bin_plastic = model.material("matte_black_bin", color=(0.010, 0.011, 0.012, 1.0))
    door_plastic = model.material("slightly_lighter_door", color=(0.070, 0.075, 0.073, 1.0))
    black_trim = model.material("black_rubber_trim", color=(0.006, 0.006, 0.005, 1.0))
    dark_steel = model.material("blackened_steel", color=(0.025, 0.024, 0.022, 1.0))
    latch_metal = model.material("satin_latch_metal", color=(0.42, 0.40, 0.36, 1.0))

    dashboard = model.part("dashboard_bin")

    # Dashboard front frame: four connected molded strips leave the glove-box opening clear.
    dashboard.visual(
        Box((0.72, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dash_plastic,
        name="front_top",
    )
    dashboard.visual(
        Box((0.72, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=dash_plastic,
        name="front_bottom",
    )
    dashboard.visual(
        Box((0.070, 0.030, 0.360)),
        origin=Origin(xyz=(-0.245, 0.0, 0.0)),
        material=dash_plastic,
        name="hinge_jamb",
    )
    dashboard.visual(
        Box((0.070, 0.030, 0.360)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=dash_plastic,
        name="latch_jamb",
    )

    # Thin black lip around the opening suggests a gasket and gives the molded front depth.
    dashboard.visual(
        Box((0.462, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.018, 0.124)),
        material=black_trim,
        name="upper_lip",
    )
    dashboard.visual(
        Box((0.462, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.018, -0.124)),
        material=black_trim,
        name="lower_lip",
    )
    dashboard.visual(
        Box((0.014, 0.010, 0.250)),
        origin=Origin(xyz=(-0.232, 0.018, 0.0)),
        material=black_trim,
        name="hinge_lip",
    )
    dashboard.visual(
        Box((0.014, 0.010, 0.250)),
        origin=Origin(xyz=(0.232, 0.018, 0.0)),
        material=black_trim,
        name="latch_lip",
    )

    # Fixed hollow bin extends rearward into the dash.  It is open at the front.
    dashboard.visual(
        Box((0.460, 0.020, 0.260)),
        origin=Origin(xyz=(0.0, -0.330, 0.0)),
        material=bin_plastic,
        name="back_wall",
    )
    dashboard.visual(
        Box((0.440, 0.320, 0.020)),
        origin=Origin(xyz=(0.0, -0.175, -0.120)),
        material=bin_plastic,
        name="bin_floor",
    )
    dashboard.visual(
        Box((0.440, 0.320, 0.020)),
        origin=Origin(xyz=(0.0, -0.175, 0.120)),
        material=bin_plastic,
        name="bin_roof",
    )
    dashboard.visual(
        Box((0.020, 0.320, 0.240)),
        origin=Origin(xyz=(-0.220, -0.175, 0.0)),
        material=bin_plastic,
        name="hinge_wall",
    )
    dashboard.visual(
        Box((0.020, 0.320, 0.240)),
        origin=Origin(xyz=(0.220, -0.175, 0.0)),
        material=bin_plastic,
        name="latch_wall",
    )

    # Exposed stationary halves of two barrel hinges fastened to the hinge-side jamb.
    for suffix, zc in (("upper", 0.070), ("lower", -0.070)):
        dashboard.visual(
            Box((0.036, 0.012, 0.078)),
            origin=Origin(xyz=(-0.242, 0.021, zc)),
            material=dark_steel,
            name=f"frame_leaf_{suffix}",
        )
        dashboard.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(-0.225, 0.034, zc + 0.029)),
            material=dark_steel,
            name=f"frame_knuckle_{suffix}_top",
        )
        dashboard.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(-0.225, 0.034, zc - 0.029)),
            material=dark_steel,
            name=f"frame_knuckle_{suffix}_bottom",
        )

    dashboard.visual(
        Box((0.012, 0.008, 0.070)),
        origin=Origin(xyz=(0.216, 0.010, 0.0)),
        material=latch_metal,
        name="striker_plate",
    )

    door = model.part("door")
    door.visual(
        Box((0.430, 0.024, 0.235)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=door_plastic,
        name="door_panel",
    )
    door.visual(
        Box((0.365, 0.008, 0.014)),
        origin=Origin(xyz=(0.236, 0.016, 0.092)),
        material=black_trim,
        name="upper_rib",
    )
    door.visual(
        Box((0.365, 0.008, 0.014)),
        origin=Origin(xyz=(0.236, 0.016, -0.092)),
        material=black_trim,
        name="lower_rib",
    )
    door.visual(
        Box((0.014, 0.008, 0.165)),
        origin=Origin(xyz=(0.070, 0.016, 0.0)),
        material=black_trim,
        name="hinge_rib",
    )
    door.visual(
        Box((0.014, 0.008, 0.165)),
        origin=Origin(xyz=(0.400, 0.016, 0.0)),
        material=black_trim,
        name="free_edge_rib",
    )
    for suffix, zc in (("upper", 0.070), ("lower", -0.070)):
        door.visual(
            Box((0.045, 0.007, 0.036)),
            origin=Origin(xyz=(0.026, -0.006, zc)),
            material=dark_steel,
            name=f"door_leaf_{suffix}",
        )
        door.visual(
            Cylinder(radius=0.008, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=dark_steel,
            name=f"door_knuckle_{suffix}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(-0.225, 0.034, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    latch = model.part("latch")
    latch_knob = KnobGeometry(
        0.038,
        0.016,
        body_style="faceted",
        grip=KnobGrip(style="ribbed", count=12, depth=0.0007),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    latch.visual(
        mesh_from_geometry(latch_knob, "latch_knob"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="rotary_knob",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="pivot_stem",
    )
    latch.visual(
        Box((0.065, 0.010, 0.016)),
        origin=Origin(xyz=(0.027, -0.031, 0.0)),
        material=latch_metal,
        name="cam_tongue",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.365, 0.012, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="door_panel",
        elem_b="pivot_stem",
        reason="The rotary latch stem intentionally passes through a molded hole in the door.",
    )
    ctx.expect_overlap(
        door,
        latch,
        axes="y",
        elem_a="door_panel",
        elem_b="pivot_stem",
        min_overlap=0.018,
        name="latch stem passes through door thickness",
    )

    ctx.expect_gap(
        door,
        dashboard,
        axis="y",
        positive_elem="door_panel",
        negative_elem="front_top",
        min_gap=0.004,
        max_gap=0.020,
        name="closed door stands just proud of dashboard",
    )
    ctx.expect_overlap(
        door,
        dashboard,
        axes="xz",
        elem_a="door_panel",
        elem_b="back_wall",
        min_overlap=0.180,
        name="door covers the storage bin opening",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    closed_cam_aabb = ctx.part_element_world_aabb(latch, elem="cam_tongue")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({latch_pivot: math.pi / 2.0}):
        turned_cam_aabb = ctx.part_element_world_aabb(latch, elem="cam_tongue")

    ctx.check(
        "door swings outward on vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "latch cam quarter-turns on front-back pivot",
        closed_cam_aabb is not None
        and turned_cam_aabb is not None
        and (closed_cam_aabb[1][0] - closed_cam_aabb[0][0]) > 0.055
        and (turned_cam_aabb[1][2] - turned_cam_aabb[0][2]) > 0.055,
        details=f"closed={closed_cam_aabb}, turned={turned_cam_aabb}",
    )
    ctx.check(
        "hinge and latch axes match requested directions",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 3) for v in latch_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"door_axis={door_hinge.axis}, latch_axis={latch_pivot.axis}",
    )

    return ctx.report()


object_model = build_object_model()
