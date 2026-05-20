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
    model = ArticulatedObject(name="dual_boiler_espresso_machine")

    steel = model.material("steel", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")

    depth = 0.44
    width = 0.34
    height = 0.39
    wall = 0.004
    front = depth / 2.0
    rear = -depth / 2.0
    side = width / 2.0

    opening_front = -0.04
    opening_rear = -0.18
    opening_width = 0.16
    opening_half = opening_width / 2.0
    opening_length = opening_front - opening_rear
    bay_depth = 0.066
    bay_floor_top = height - bay_depth

    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, side - wall / 2.0, height / 2.0)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -side + wall / 2.0, height / 2.0)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=steel,
        name="bottom",
    )
    body.visual(
        Box((wall, width - 2.0 * wall, height)),
        origin=Origin(xyz=(front - wall / 2.0, 0.0, height / 2.0)),
        material=steel,
        name="front_panel",
    )
    body.visual(
        Box((wall, width - 2.0 * wall, height)),
        origin=Origin(xyz=(rear + wall / 2.0, 0.0, height / 2.0)),
        material=steel,
        name="rear_panel",
    )

    body.visual(
        Box((0.26, width, wall)),
        origin=Origin(xyz=(0.09, 0.0, height - wall / 2.0)),
        material=steel,
        name="top_front_deck",
    )
    body.visual(
        Box((opening_length, 0.09, wall)),
        origin=Origin(
            xyz=((opening_front + opening_rear) / 2.0, opening_half + 0.045, height - wall / 2.0)
        ),
        material=steel,
        name="top_left_rail",
    )
    body.visual(
        Box((opening_length, 0.09, wall)),
        origin=Origin(
            xyz=((opening_front + opening_rear) / 2.0, -opening_half - 0.045, height - wall / 2.0)
        ),
        material=steel,
        name="top_right_rail",
    )
    body.visual(
        Box((0.04, width, wall)),
        origin=Origin(xyz=(-0.20, 0.0, height - wall / 2.0)),
        material=steel,
        name="rear_bridge",
    )

    body.visual(
        Box((0.136, 0.156, wall)),
        origin=Origin(xyz=(-0.11, 0.0, bay_floor_top - wall / 2.0)),
        material=dark_trim,
        name="bay_floor",
    )
    body.visual(
        Box((opening_length, wall, bay_depth)),
        origin=Origin(
            xyz=(
                (opening_front + opening_rear) / 2.0,
                opening_half - wall / 2.0,
                bay_floor_top + bay_depth / 2.0 - wall / 2.0,
            )
        ),
        material=steel,
        name="bay_left_wall",
    )
    body.visual(
        Box((opening_length, wall, bay_depth)),
        origin=Origin(
            xyz=(
                (opening_front + opening_rear) / 2.0,
                -opening_half + wall / 2.0,
                bay_floor_top + bay_depth / 2.0 - wall / 2.0,
            )
        ),
        material=steel,
        name="bay_right_wall",
    )
    body.visual(
        Box((wall, 0.156, bay_depth)),
        origin=Origin(
            xyz=(
                opening_front - wall / 2.0,
                0.0,
                bay_floor_top + bay_depth / 2.0 - wall / 2.0,
            )
        ),
        material=steel,
        name="bay_front_wall",
    )
    body.visual(
        Box((wall, 0.156, bay_depth)),
        origin=Origin(
            xyz=(
                opening_rear + wall / 2.0,
                0.0,
                bay_floor_top + bay_depth / 2.0 - wall / 2.0,
            )
        ),
        material=steel,
        name="bay_rear_wall",
    )

    body.visual(
        Box((0.10, 0.26, 0.055)),
        origin=Origin(xyz=(0.17, 0.0, 0.0475)),
        material=dark_trim,
        name="drip_tray",
    )
    body.visual(
        Box((0.075, 0.08, 0.075)),
        origin=Origin(xyz=(0.255, 0.0, 0.305)),
        material=steel,
        name="group_body",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.031),
        origin=Origin(xyz=(0.27, 0.0, 0.2525)),
        material=steel,
        name="brew_collar",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.185, 0.125, 0.345)),
        material=steel,
        name="steam_support",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.185, -0.125, 0.345)),
        material=steel,
        name="water_support",
    )
    body.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="portafilter_collar",
    )
    portafilter.visual(
        Cylinder(radius=0.029, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0345)),
        material=steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.018, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=steel,
        name="spout_block",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.09),
        origin=Origin(xyz=(0.048, 0.0, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_stem",
    )
    portafilter.visual(
        Cylinder(radius=0.014, length=0.09),
        origin=Origin(xyz=(0.11, 0.0, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_grip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.16, 0.07, 0.08)),
        mass=0.85,
        origin=Origin(xyz=(0.08, 0.0, -0.04)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.27, 0.0, 0.237)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.9,
            upper=0.0,
        ),
    )

    def add_wand(
        part_name: str,
        joint_name: str,
        pivot_y: float,
        tube_points: list[tuple[float, float, float]],
        tip_origin: tuple[float, float, float],
    ) -> None:
        wand = model.part(part_name)
        wand.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=steel,
            name="pivot_collar",
        )
        wand.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    tube_points,
                    radius=0.004,
                    samples_per_segment=18,
                    radial_segments=16,
                ),
                f"{part_name}_tube",
            ),
            material=steel,
            name="wand_tube",
        )
        wand.visual(
            Cylinder(radius=0.0035, length=0.018),
            origin=Origin(xyz=tip_origin, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="wand_tip",
        )
        wand.inertial = Inertial.from_geometry(
            Box((0.10, 0.03, 0.20)),
            mass=0.25,
            origin=Origin(xyz=(0.04, 0.0, -0.10)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=wand,
            origin=Origin(xyz=(0.185, pivot_y, 0.300)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=-1.2,
                upper=1.2,
            ),
        )

    add_wand(
        "steam_wand",
        "body_to_steam_wand",
        0.125,
        [
            (0.0, 0.0, -0.02),
            (0.0, 0.0, -0.07),
            (0.02, 0.0, -0.10),
            (0.055, 0.0, -0.165),
            (0.075, 0.0, -0.165),
        ],
        (0.084, 0.0, -0.165),
    )
    add_wand(
        "water_wand",
        "body_to_water_wand",
        -0.125,
        [
            (0.0, 0.0, -0.018),
            (0.0, 0.0, -0.055),
            (0.016, 0.0, -0.078),
            (0.040, 0.0, -0.135),
            (0.058, 0.0, -0.135),
        ],
        (0.067, 0.0, -0.135),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.005, length=0.15),
        origin=Origin(xyz=(0.005, 0.0, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((opening_length, 0.164, 0.006)),
        origin=Origin(xyz=(opening_length / 2.0, 0.0, 0.003)),
        material=steel,
        name="lid_panel",
    )
    lid.visual(
        Box((0.02, 0.08, 0.012)),
        origin=Origin(xyz=(opening_length - 0.01, 0.0, 0.009)),
        material=black,
        name="lid_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((opening_length, 0.164, 0.016)),
        mass=0.45,
        origin=Origin(xyz=(opening_length / 2.0, 0.0, 0.008)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(opening_rear, 0.0, height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    cup_platform = model.part("cup_platform")
    cup_platform.visual(
        Cylinder(radius=0.005, length=0.14),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="platform_hinge",
    )
    cup_platform.visual(
        Box((0.006, 0.16, 0.09)),
        origin=Origin(xyz=(0.003, 0.0, -0.045)),
        material=steel,
        name="platform_plate",
    )
    cup_platform.visual(
        Box((0.012, 0.16, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, -0.086)),
        material=dark_trim,
        name="platform_lip",
    )
    cup_platform.inertial = Inertial.from_geometry(
        Box((0.015, 0.16, 0.10)),
        mass=0.25,
        origin=Origin(xyz=(0.007, 0.0, -0.045)),
    )
    model.articulation(
        "body_to_cup_platform",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cup_platform,
        origin=Origin(xyz=(front, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.57,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    water_wand = object_model.get_part("water_wand")
    lid = object_model.get_part("lid")
    cup_platform = object_model.get_part("cup_platform")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    water_joint = object_model.get_articulation("body_to_water_wand")
    lid_joint = object_model.get_articulation("body_to_lid")
    platform_joint = object_model.get_articulation("body_to_cup_platform")

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="brew_collar",
        negative_elem="portafilter_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="portafilter seats beneath brew collar",
    )
    ctx.expect_overlap(
        body,
        portafilter,
        axes="xy",
        elem_a="brew_collar",
        elem_b="portafilter_collar",
        min_overlap=0.05,
        name="portafilter stays centered under the brew axis",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_front_deck",
        max_gap=0.008,
        max_penetration=0.0,
        name="reservoir lid sits flush with the top deck",
    )
    ctx.expect_gap(
        cup_platform,
        body,
        axis="x",
        positive_elem="platform_plate",
        negative_elem="front_panel",
        max_gap=0.008,
        max_penetration=0.0,
        name="cup platform stores close to the front panel",
    )

    locked_handle = elem_center("portafilter", "handle_grip")
    with ctx.pose({portafilter_joint: -0.75}):
        unlocked_handle = elem_center("portafilter", "handle_grip")
    ctx.check(
        "portafilter rotates into the group head",
        locked_handle is not None
        and unlocked_handle is not None
        and unlocked_handle[1] < locked_handle[1] - 0.06
        and abs(unlocked_handle[2] - locked_handle[2]) < 0.01,
        details=f"locked={locked_handle}, unlocked={unlocked_handle}",
    )

    steam_tip_rest = elem_center("steam_wand", "wand_tip")
    with ctx.pose({steam_joint: 0.95}):
        steam_tip_swung = elem_center("steam_wand", "wand_tip")
    ctx.check(
        "steam wand swings about its vertical support pivot",
        steam_tip_rest is not None
        and steam_tip_swung is not None
        and steam_tip_swung[1] > steam_tip_rest[1] + 0.05
        and abs(steam_tip_swung[2] - steam_tip_rest[2]) < 0.02,
        details=f"rest={steam_tip_rest}, swung={steam_tip_swung}",
    )

    water_tip_rest = elem_center("water_wand", "wand_tip")
    with ctx.pose({water_joint: 0.95}):
        water_tip_swung = elem_center("water_wand", "wand_tip")
    ctx.check(
        "hot water wand swings about its own vertical support pivot",
        water_tip_rest is not None
        and water_tip_swung is not None
        and water_tip_swung[1] > water_tip_rest[1] + 0.05
        and abs(water_tip_swung[2] - water_tip_rest[2]) < 0.02,
        details=f"rest={water_tip_rest}, swung={water_tip_swung}",
    )

    lid_pull_closed = elem_center("lid", "lid_pull")
    with ctx.pose({lid_joint: 1.15}):
        lid_pull_open = elem_center("lid", "lid_pull")
    ctx.check(
        "access lid opens upward from the rear hinge",
        lid_pull_closed is not None
        and lid_pull_open is not None
        and lid_pull_open[2] > lid_pull_closed[2] + 0.08
        and lid_pull_open[0] < lid_pull_closed[0] - 0.04,
        details=f"closed={lid_pull_closed}, open={lid_pull_open}",
    )

    platform_closed = elem_center("cup_platform", "platform_plate")
    with ctx.pose({platform_joint: 1.55}):
        platform_open = elem_center("cup_platform", "platform_plate")
        ctx.expect_overlap(
            cup_platform,
            body,
            axes="y",
            elem_a="platform_plate",
            elem_b="drip_tray",
            min_overlap=0.12,
            name="opened cup platform remains centered over the drip tray width",
        )
    ctx.check(
        "cup platform folds down into a forward shelf",
        platform_closed is not None
        and platform_open is not None
        and platform_open[0] > platform_closed[0] + 0.035
        and platform_open[2] > platform_closed[2] + 0.035,
        details=f"closed={platform_closed}, open={platform_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
