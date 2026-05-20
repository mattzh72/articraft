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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heat_exchange_espresso_machine")

    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    brushed = model.material("brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    chrome = model.material("chrome", rgba=(0.90, 0.91, 0.93, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark = model.material("dark", rgba=(0.18, 0.18, 0.19, 1.0))

    tray_grate = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.224, 0.248),
            0.004,
            slot_size=(0.040, 0.005),
            pitch=(0.054, 0.015),
            frame=0.010,
            corner_radius=0.004,
            slot_angle_deg=0.0,
            stagger=False,
        ),
        "tray_grate",
    )
    steam_tube = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.004),
                (0.012, 0.000, -0.004),
                (0.024, 0.000, -0.024),
                (0.040, 0.000, -0.082),
                (0.050, 0.000, -0.150),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "steam_tube",
    )
    water_tube = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.004),
                (0.010, 0.000, -0.004),
                (0.021, 0.000, -0.020),
                (0.032, 0.000, -0.060),
                (0.040, 0.000, -0.112),
            ],
            radius=0.0040,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "water_tube",
    )

    body = model.part("body")
    body.visual(
        Box((0.404, 0.320, 0.014)),
        origin=Origin(xyz=(-0.048, 0.0, 0.023)),
        material=stainless,
        name="base_plate",
    )
    body.visual(
        Box((0.390, 0.014, 0.344)),
        origin=Origin(xyz=(-0.041, 0.153, 0.186)),
        material=stainless,
        name="side_panel_0",
    )
    body.visual(
        Box((0.390, 0.014, 0.344)),
        origin=Origin(xyz=(-0.041, -0.153, 0.186)),
        material=stainless,
        name="side_panel_1",
    )
    body.visual(
        Box((0.014, 0.320, 0.330)),
        origin=Origin(xyz=(-0.243, 0.0, 0.185)),
        material=stainless,
        name="rear_panel",
    )
    body.visual(
        Box((0.390, 0.320, 0.012)),
        origin=Origin(xyz=(-0.041, 0.0, 0.360)),
        material=stainless,
        name="top_deck",
    )
    body.visual(
        Box((0.022, 0.292, 0.120)),
        origin=Origin(xyz=(0.143, 0.0, 0.300)),
        material=stainless,
        name="front_panel",
    )
    body.visual(
        Box((0.022, 0.012, 0.106)),
        origin=Origin(xyz=(0.143, 0.146, 0.082)),
        material=stainless,
        name="tray_cheek_0",
    )
    body.visual(
        Box((0.022, 0.012, 0.106)),
        origin=Origin(xyz=(0.143, -0.146, 0.082)),
        material=stainless,
        name="tray_cheek_1",
    )
    body.visual(
        Box((0.132, 0.292, 0.012)),
        origin=Origin(xyz=(0.066, 0.0, 0.136)),
        material=brushed,
        name="brew_deck",
    )
    body.visual(
        Box((0.022, 0.220, 0.044)),
        origin=Origin(xyz=(0.143, 0.0, 0.218)),
        material=stainless,
        name="group_backplate",
    )
    body.visual(
        Box((0.280, 0.016, 0.010)),
        origin=Origin(xyz=(-0.040, 0.138, 0.067)),
        material=brushed,
        name="tray_guide_0",
    )
    body.visual(
        Box((0.280, 0.016, 0.010)),
        origin=Origin(xyz=(-0.040, -0.138, 0.067)),
        material=brushed,
        name="tray_guide_1",
    )
    body.visual(
        Box((0.082, 0.095, 0.060)),
        origin=Origin(xyz=(0.098, 0.0, 0.214)),
        material=chrome,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.034),
        origin=Origin(xyz=(0.122, 0.0, 0.201)),
        material=chrome,
        name="group_collar",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.108),
        origin=Origin(xyz=(0.082, 0.0, 0.233), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="group_crossbar",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.034),
        origin=Origin(xyz=(0.140, 0.0, 0.214), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="group_nose",
    )
    body.visual(
        Box((0.020, 0.020, 0.028)),
        origin=Origin(xyz=(0.144, 0.132, 0.274)),
        material=chrome,
        name="wand_mount_0",
    )
    body.visual(
        Box((0.020, 0.020, 0.028)),
        origin=Origin(xyz=(0.144, -0.132, 0.274)),
        material=chrome,
        name="wand_mount_1",
    )

    rail_z = 0.400
    post_z = rail_z
    for index, (px, py) in enumerate(
        ((0.030, 0.135), (0.030, -0.135), (-0.140, 0.135), (-0.140, -0.135))
    ):
        body.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(px, py, 0.380)),
            material=chrome,
            name=f"rail_post_{index}",
        )
    body.visual(
        Cylinder(radius=0.005, length=0.270),
        origin=Origin(xyz=(0.030, 0.0, post_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rail_front",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.270),
        origin=Origin(xyz=(-0.140, 0.0, post_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rail_rear",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.170),
        origin=Origin(xyz=(-0.055, 0.135, post_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="rail_side_0",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.170),
        origin=Origin(xyz=(-0.055, -0.135, post_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="rail_side_1",
    )
    for index, (px, py) in enumerate(((-0.175, 0.112), (-0.175, -0.112), (0.090, 0.112), (0.090, -0.112))):
        body.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(px, py, 0.009)),
            material=dark,
            name=f"foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.430, 0.330, 0.420)),
        mass=18.0,
        origin=Origin(xyz=(-0.040, 0.0, 0.210)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=chrome,
        name="basket_rim",
    )
    portafilter.visual(
        Cylinder(radius=0.027, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=chrome,
        name="basket_body",
    )
    portafilter.visual(
        Box((0.020, 0.032, 0.008)),
        origin=Origin(xyz=(0.021, 0.0, -0.046)),
        material=chrome,
        name="spout_bridge",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.026, 0.010, -0.053), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="spout_0",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.026, -0.010, -0.053), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="spout_1",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.034, 0.0, -0.042), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_neck",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.150),
        origin=Origin(xyz=(0.120, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle",
    )
    portafilter.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.196, 0.0, -0.050)),
        material=black,
        name="handle_cap",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.230, 0.070, 0.070)),
        mass=0.7,
        origin=Origin(xyz=(0.095, 0.0, -0.024)),
    )

    hatch = model.part("reservoir_hatch")
    hatch.visual(
        Box((0.126, 0.154, 0.005)),
        origin=Origin(xyz=(0.063, 0.0, 0.0025)),
        material=brushed,
        name="lid_panel",
    )
    hatch.visual(
        Box((0.014, 0.090, 0.014)),
        origin=Origin(xyz=(0.119, 0.0, 0.010)),
        material=black,
        name="pull_lip",
    )
    hatch.inertial = Inertial.from_geometry(
        Box((0.130, 0.160, 0.022)),
        mass=0.3,
        origin=Origin(xyz=(0.064, 0.0, 0.010)),
    )

    tray = model.part("drip_tray")
    tray.visual(
        Box((0.012, 0.280, 0.070)),
        origin=Origin(xyz=(0.006, 0.0, 0.035)),
        material=brushed,
        name="front_face",
    )
    tray.visual(
        Box((0.230, 0.260, 0.004)),
        origin=Origin(xyz=(-0.115, 0.0, 0.002)),
        material=brushed,
        name="pan_floor",
    )
    tray.visual(
        Box((0.230, 0.006, 0.050)),
        origin=Origin(xyz=(-0.115, 0.127, 0.025)),
        material=brushed,
        name="wall_0",
    )
    tray.visual(
        Box((0.230, 0.006, 0.050)),
        origin=Origin(xyz=(-0.115, -0.127, 0.025)),
        material=brushed,
        name="wall_1",
    )
    tray.visual(
        Box((0.006, 0.248, 0.050)),
        origin=Origin(xyz=(-0.227, 0.0, 0.025)),
        material=brushed,
        name="rear_wall",
    )
    tray.visual(
        tray_grate,
        origin=Origin(xyz=(-0.112, 0.0, 0.048)),
        material=dark,
        name="grate",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.242, 0.282, 0.074)),
        mass=1.2,
        origin=Origin(xyz=(-0.110, 0.0, 0.036)),
    )

    steam = model.part("steam_wand")
    steam.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_arm",
    )
    steam.visual(steam_tube, material=chrome, name="tube")
    steam.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.046, 0.0, -0.095)),
        material=black,
        name="grip",
    )
    steam.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.051, 0.0, -0.157)),
        material=chrome,
        name="tip",
    )
    steam.inertial = Inertial.from_geometry(
        Box((0.070, 0.020, 0.180)),
        mass=0.3,
        origin=Origin(xyz=(0.036, 0.0, -0.080)),
    )

    water = model.part("water_wand")
    water.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_arm",
    )
    water.visual(water_tube, material=chrome, name="tube")
    water.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(xyz=(0.040, 0.0, -0.118)),
        material=chrome,
        name="tip",
    )
    water.inertial = Inertial.from_geometry(
        Box((0.060, 0.018, 0.140)),
        mass=0.2,
        origin=Origin(xyz=(0.028, 0.0, -0.062)),
    )

    model.articulation(
        "portafilter_lock",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.122, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.25,
        ),
    )
    model.articulation(
        "reservoir_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.185, 0.0, 0.366)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "drip_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.154, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.25,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "steam_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam,
        origin=Origin(xyz=(0.154, 0.132, 0.288)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "water_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=water,
        origin=Origin(xyz=(0.154, -0.132, 0.288)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-1.10,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    hatch = object_model.get_part("reservoir_hatch")
    tray = object_model.get_part("drip_tray")
    steam = object_model.get_part("steam_wand")
    water = object_model.get_part("water_wand")

    portafilter_lock = object_model.get_articulation("portafilter_lock")
    reservoir_hinge = object_model.get_articulation("reservoir_hinge")
    drip_tray_slide = object_model.get_articulation("drip_tray_slide")
    steam_pivot = object_model.get_articulation("steam_pivot")
    water_pivot = object_model.get_articulation("water_pivot")

    def element_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_contact(
        portafilter,
        body,
        elem_a="basket_rim",
        elem_b="group_collar",
        name="portafilter seats against brew collar",
    )

    with ctx.pose({reservoir_hinge: 0.0}):
        ctx.expect_gap(
            hatch,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_deck",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed hatch sits on the top deck",
        )

    closed_lip = element_center(hatch, "pull_lip")
    with ctx.pose({reservoir_hinge: 1.0}):
        open_lip = element_center(hatch, "pull_lip")
    ctx.check(
        "reservoir hatch opens upward",
        closed_lip is not None and open_lip is not None and open_lip[2] > closed_lip[2] + 0.05,
        details=f"closed_lip={closed_lip}, open_lip={open_lip}",
    )

    ctx.expect_within(
        tray,
        body,
        axes="yz",
        margin=0.006,
        name="drip tray stays centered in the front opening",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        min_overlap=0.070,
        name="resting drip tray remains inserted",
    )
    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({drip_tray_slide: 0.120}):
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.006,
            name="extended drip tray stays aligned with the guides",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.030,
            name="extended drip tray keeps retained insertion",
        )
        tray_extended = ctx.part_world_position(tray)
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[0] > tray_rest[0] + 0.08,
        details=f"tray_rest={tray_rest}, tray_extended={tray_extended}",
    )

    locked_handle = element_center(portafilter, "handle")
    with ctx.pose({portafilter_lock: -0.60}):
        turned_handle = element_center(portafilter, "handle")
    ctx.check(
        "portafilter rotates under the group head",
        locked_handle is not None
        and turned_handle is not None
        and abs(turned_handle[1] - locked_handle[1]) > 0.05,
        details=f"locked_handle={locked_handle}, turned_handle={turned_handle}",
    )

    steam_parked = element_center(steam, "tip")
    with ctx.pose({steam_pivot: 0.90}):
        steam_swung = element_center(steam, "tip")
    ctx.check(
        "steam wand swings outboard",
        steam_parked is not None and steam_swung is not None and steam_swung[1] > steam_parked[1] + 0.03,
        details=f"steam_parked={steam_parked}, steam_swung={steam_swung}",
    )

    water_parked = element_center(water, "tip")
    with ctx.pose({water_pivot: -0.90}):
        water_swung = element_center(water, "tip")
    ctx.check(
        "hot water wand swings outboard",
        water_parked is not None and water_swung is not None and water_swung[1] < water_parked[1] - 0.02,
        details=f"water_parked={water_parked}, water_swung={water_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
