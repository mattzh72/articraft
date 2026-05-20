from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_espresso_machine")

    shell = model.material("shell", rgba=(0.14, 0.15, 0.17, 1.0))
    trim = model.material("trim", rgba=(0.76, 0.78, 0.80, 1.0))
    plastic = model.material("plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    tray = model.material("tray", rgba=(0.24, 0.25, 0.27, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.280, 0.150, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=shell,
        name="base",
    )
    body.visual(
        Box((0.280, 0.008, 0.294)),
        origin=Origin(xyz=(0.000, -0.071, 0.153)),
        material=shell,
        name="left_wall",
    )
    body.visual(
        Box((0.280, 0.008, 0.294)),
        origin=Origin(xyz=(0.000, 0.071, 0.153)),
        material=shell,
        name="right_wall",
    )
    body.visual(
        Box((0.008, 0.150, 0.300)),
        origin=Origin(xyz=(-0.136, 0.000, 0.156)),
        material=shell,
        name="rear_wall",
    )
    body.visual(
        Box((0.008, 0.150, 0.170)),
        origin=Origin(xyz=(0.136, 0.000, 0.091)),
        material=shell,
        name="front_wall",
    )
    body.visual(
        Box((0.070, 0.152, 0.100)),
        origin=Origin(xyz=(0.105, 0.000, 0.256)),
        material=shell,
        name="upper_cowl",
    )
    body.visual(
        Box((0.164, 0.150, 0.008)),
        origin=Origin(xyz=(0.050, 0.000, 0.304)),
        material=trim,
        name="top_deck",
    )
    body.visual(
        Box((0.008, 0.144, 0.018)),
        origin=Origin(xyz=(-0.028, 0.000, 0.309)),
        material=trim,
        name="tank_rim_front",
    )
    body.visual(
        Box((0.104, 0.134, 0.008)),
        origin=Origin(xyz=(-0.080, 0.000, 0.250)),
        material=tray,
        name="tank_floor",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.108, 0.000, 0.216)),
        material=trim,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.108, 0.000, 0.201)),
        material=trim,
        name="group_head_face",
    )
    body.visual(
        Box((0.092, 0.120, 0.012)),
        origin=Origin(xyz=(0.090, 0.000, 0.124)),
        material=tray,
        name="front_panel",
    )
    body.visual(
        Box((0.008, 0.040, 0.012)),
        origin=Origin(xyz=(0.142, 0.000, 0.182)),
        material=trim,
        name="brew_spout_block",
    )
    body.visual(
        Box((0.026, 0.010, 0.050)),
        origin=Origin(xyz=(0.118, 0.076, 0.212)),
        material=trim,
        name="steam_mount",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(
            xyz=(-0.030, 0.079, 0.092),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="selector_boss",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(
            xyz=(0.136, 0.000, 0.055),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="shelf_hinge_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.280, 0.150, 0.320)),
        mass=7.0,
        origin=Origin(xyz=(0.000, 0.000, 0.160)),
    )

    tank_lid = model.part("tank_lid")
    tank_lid.visual(
        Box((0.108, 0.144, 0.006)),
        origin=Origin(xyz=(0.054, 0.000, 0.003)),
        material=trim,
        name="lid_panel",
    )
    tank_lid.visual(
        Box((0.016, 0.060, 0.010)),
        origin=Origin(xyz=(0.100, 0.000, 0.008)),
        material=plastic,
        name="lid_tab",
    )
    tank_lid.inertial = Inertial.from_geometry(
        Box((0.108, 0.144, 0.016)),
        mass=0.18,
        origin=Origin(xyz=(0.054, 0.000, 0.008)),
    )
    model.articulation(
        "body_to_tank_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tank_lid,
        origin=Origin(xyz=(-0.132, 0.000, 0.318)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=trim,
        name="filter_ring",
    )
    portafilter.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, -0.016)),
        material=trim,
        name="filter_basket",
    )
    portafilter.visual(
        Box((0.026, 0.018, 0.010)),
        origin=Origin(xyz=(0.004, 0.000, -0.034)),
        material=trim,
        name="spout_body",
    )
    portafilter.visual(
        Cylinder(radius=0.009, length=0.090),
        origin=Origin(
            xyz=(0.073, 0.000, -0.028),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic,
        name="handle_core",
    )
    portafilter.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(
            xyz=(0.112, 0.000, -0.036),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic,
        name="handle_grip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.155, 0.070, 0.050)),
        mass=0.52,
        origin=Origin(xyz=(0.055, 0.000, -0.025)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.108, 0.000, 0.194)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.15,
        ),
    )

    wand = model.part("steam_wand")
    wand.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.000, 0.010, 0.000)),
        material=trim,
        name="wand_collar",
    )
    wand_tube = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.014, 0.000),
                (0.004, 0.018, -0.010),
                (0.012, 0.024, -0.028),
                (0.022, 0.026, -0.050),
                (0.030, 0.022, -0.070),
            ],
            radius=0.0035,
            samples_per_segment=18,
            radial_segments=18,
        ),
        "steam_wand_tube",
    )
    wand.visual(
        wand_tube,
        material=trim,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.030, 0.022, -0.077)),
        material=trim,
        name="wand_tip",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.100)),
        mass=0.11,
        origin=Origin(xyz=(0.016, 0.016, -0.040)),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.118, 0.079, 0.212)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.20,
        ),
    )

    shelf = model.part("cup_shelf")
    shelf.visual(
        Box((0.008, 0.110, 0.080)),
        origin=Origin(xyz=(0.004, 0.000, 0.040)),
        material=tray,
        name="shelf_panel",
    )
    shelf.visual(
        Box((0.010, 0.110, 0.010)),
        origin=Origin(xyz=(0.005, 0.000, 0.080)),
        material=trim,
        name="shelf_lip",
    )
    for index, rib_z in enumerate((0.020, 0.040, 0.060)):
        shelf.visual(
            Box((0.006, 0.090, 0.006)),
            origin=Origin(xyz=(0.007, 0.000, rib_z)),
            material=trim,
            name=f"shelf_rib_{index}",
        )
    shelf.inertial = Inertial.from_geometry(
        Box((0.016, 0.110, 0.090)),
        mass=0.22,
        origin=Origin(xyz=(0.008, 0.000, 0.045)),
    )
    model.articulation(
        "body_to_cup_shelf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.140, 0.000, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.50,
        ),
    )

    selector = model.part("selector_knob")
    selector.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(
            xyz=(0.000, 0.006, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="selector_shaft",
    )
    selector.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(
            xyz=(0.000, 0.019, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=plastic,
        name="selector_dial",
    )
    selector.visual(
        Box((0.008, 0.006, 0.004)),
        origin=Origin(xyz=(0.014, 0.026, 0.000)),
        material=trim,
        name="selector_mark",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.044, 0.032, 0.044)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.020, 0.000)),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector,
        origin=Origin(xyz=(-0.030, 0.083, 0.092)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tank_lid = object_model.get_part("tank_lid")
    portafilter = object_model.get_part("portafilter")
    wand = object_model.get_part("steam_wand")
    shelf = object_model.get_part("cup_shelf")
    selector = object_model.get_part("selector_knob")

    lid_joint = object_model.get_articulation("body_to_tank_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    shelf_joint = object_model.get_articulation("body_to_cup_shelf")
    selector_joint = object_model.get_articulation("body_to_selector_knob")

    def elem_center(part_name, elem_name):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        tank_lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="tank_rim_front",
        max_gap=0.002,
        max_penetration=0.0,
        name="tank lid sits on the rear tank rim",
    )
    ctx.expect_overlap(
        tank_lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="tank_floor",
        min_overlap=0.100,
        name="tank lid covers the tank opening footprint",
    )
    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_head_face",
        negative_elem="filter_basket",
        max_gap=0.001,
        max_penetration=0.0,
        name="portafilter basket seats below the group head",
    )
    ctx.expect_overlap(
        body,
        portafilter,
        axes="xy",
        elem_a="group_head_face",
        elem_b="filter_basket",
        min_overlap=0.050,
        name="portafilter stays centered on the brew axis",
    )
    ctx.expect_gap(
        wand,
        body,
        axis="y",
        positive_elem="wand_collar",
        negative_elem="steam_mount",
        max_gap=0.002,
        max_penetration=1e-6,
        name="steam wand collar stays mounted to the side pivot",
    )
    ctx.expect_gap(
        selector,
        body,
        axis="y",
        positive_elem="selector_shaft",
        negative_elem="selector_boss",
        max_gap=0.002,
        max_penetration=0.0,
        name="selector knob sits on the side boss",
    )
    ctx.expect_gap(
        shelf,
        body,
        axis="x",
        positive_elem="shelf_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="cup shelf folds flat against the front body",
    )
    ctx.expect_overlap(
        shelf,
        body,
        axes="yz",
        elem_a="shelf_panel",
        elem_b="front_panel",
        min_overlap=0.010,
        name="cup shelf aligns with the lower front panel when closed",
    )

    lid_closed = elem_center(tank_lid, "lid_tab")
    with ctx.pose({lid_joint: 1.00}):
        lid_open = elem_center(tank_lid, "lid_tab")
    ctx.check(
        "tank lid opens upward",
        lid_closed is not None and lid_open is not None and lid_open[2] > lid_closed[2] + 0.060,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    handle_locked = elem_center(portafilter, "handle_grip")
    with ctx.pose({portafilter_joint: -0.45}):
        handle_unlocked = elem_center(portafilter, "handle_grip")
    ctx.check(
        "portafilter handle twists into the group head",
        handle_locked is not None
        and handle_unlocked is not None
        and handle_unlocked[1] < handle_locked[1] - 0.030,
        details=f"locked={handle_locked}, unlocked={handle_unlocked}",
    )

    tip_rest = elem_center(wand, "wand_tip")
    with ctx.pose({wand_joint: -1.00}):
        tip_swung = elem_center(wand, "wand_tip")
    ctx.check(
        "steam wand swings around the side pivot",
        tip_rest is not None and tip_swung is not None and tip_swung[1] < tip_rest[1] - 0.025,
        details=f"rest={tip_rest}, swung={tip_swung}",
    )

    shelf_closed = elem_center(shelf, "shelf_panel")
    with ctx.pose({shelf_joint: 1.35}):
        shelf_open = elem_center(shelf, "shelf_panel")
    ctx.check(
        "cup shelf folds down into a platform",
        shelf_closed is not None
        and shelf_open is not None
        and shelf_open[0] > shelf_closed[0] + 0.030
        and shelf_open[2] < shelf_closed[2] - 0.020,
        details=f"closed={shelf_closed}, open={shelf_open}",
    )

    mark_low = elem_center(selector, "selector_mark")
    with ctx.pose({selector_joint: 1.00}):
        mark_high = elem_center(selector, "selector_mark")
    ctx.check(
        "selector knob rotates about its side-facing shaft",
        mark_low is not None and mark_high is not None and abs(mark_high[2] - mark_low[2]) > 0.008,
        details=f"low={mark_low}, high={mark_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
