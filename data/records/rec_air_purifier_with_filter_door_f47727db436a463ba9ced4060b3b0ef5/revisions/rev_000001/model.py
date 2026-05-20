from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SIZE = 0.34
WALL = 0.024
SIDE_OPEN_Y = 0.282
SIDE_OPEN_Z = 0.262
SIDE_OPEN_CENTER_Z = 0.174
SIDE_FACE_X = SIZE / 2.0

DOOR_THICKNESS = 0.014
DOOR_WIDTH = 0.300
DOOR_HEIGHT = 0.292
DOOR_BOTTOM_Z = 0.030
HINGE_X = SIDE_FACE_X + 0.004
HINGE_Y = -DOOR_WIDTH / 2.0

FILTER_LEN = 0.210
FILTER_WIDTH = 0.214
FILTER_HEIGHT = 0.214
FILTER_ORIGIN_X = SIDE_FACE_X - WALL
FILTER_CENTER_Z = SIDE_OPEN_CENTER_Z
FILTER_TRAVEL = 0.160


def _housing_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(SIZE, SIZE, SIZE).translate((0.0, 0.0, SIZE / 2.0))
    inner = (
        cq.Workplane("XY")
        .box(SIZE - 2.0 * WALL, SIZE - 2.0 * WALL, SIZE - 2.0 * WALL)
        .translate((0.0, 0.0, SIZE / 2.0))
    )
    shell = outer.cut(inner)

    side_opening = (
        cq.Workplane("XY")
        .box(0.090, SIDE_OPEN_Y, SIDE_OPEN_Z)
        .translate((SIDE_FACE_X + 0.012, 0.0, SIDE_OPEN_CENTER_Z))
    )
    shell = shell.cut(side_opening)

    front_opening = (
        cq.Workplane("XY")
        .box(0.230, 0.090, 0.185)
        .translate((0.0, -SIZE / 2.0 - 0.012, 0.195))
    )
    shell = shell.cut(front_opening)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_cube_air_purifier")

    plastic = model.material("warm_white_plastic", color=(0.88, 0.89, 0.86, 1.0))
    dark = model.material("charcoal_shadow", color=(0.05, 0.055, 0.06, 1.0))
    paper = model.material("pleated_filter_media", color=(0.92, 0.86, 0.68, 1.0))
    gasket = model.material("soft_gray_gasket", color=(0.25, 0.27, 0.28, 1.0))
    rail_mat = model.material("matte_gray_rails", color=(0.36, 0.37, 0.36, 1.0))
    metal = model.material("brushed_hinge_pin", color=(0.62, 0.62, 0.58, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "hollow_cube_housing"),
        material=plastic,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.238, 0.192),
                frame=0.016,
                face_thickness=0.004,
                duct_depth=0.024,
                duct_wall=0.003,
                slat_pitch=0.020,
                slat_width=0.009,
                slat_angle_deg=32.0,
                corner_radius=0.006,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
            ),
            "front_louvered_grille",
        ),
        origin=Origin(xyz=(0.0, -SIZE / 2.0 + 0.006, 0.195), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="front_grille",
    )

    for i, y in enumerate((-0.146, 0.146)):
        housing.visual(
            Box((0.230, 0.048, 0.014)),
            origin=Origin(xyz=(0.040, y, 0.091)),
            material=rail_mat,
            name=f"guide_rail_{i}",
        )

    for i, z in enumerate((0.055, 0.174, 0.293)):
        housing.visual(
            Cylinder(radius=0.010, length=0.045),
            origin=Origin(xyz=(HINGE_X + 0.002, HINGE_Y - 0.010, z)),
            material=metal,
            name=f"hinge_knuckle_{i}",
        )
    housing.visual(
        Box((0.012, 0.020, DOOR_HEIGHT + 0.020)),
        origin=Origin(xyz=(HINGE_X - 0.006, HINGE_Y - 0.006, DOOR_BOTTOM_Z + DOOR_HEIGHT / 2.0)),
        material=plastic,
        name="hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=0.004, length=DOOR_HEIGHT + 0.026),
        origin=Origin(xyz=(HINGE_X + 0.002, HINGE_Y - 0.010, DOOR_BOTTOM_Z + DOOR_HEIGHT / 2.0)),
        material=metal,
        name="hinge_pin",
    )

    door = model.part("filter_door")
    door.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (DOOR_HEIGHT, DOOR_WIDTH),
                DOOR_THICKNESS,
                slot_size=(0.052, 0.007),
                pitch=(0.068, 0.021),
                frame=0.028,
                corner_radius=0.005,
                slot_angle_deg=0.0,
                stagger=True,
            ),
            "side_filter_door_panel",
        ),
        origin=Origin(
            xyz=(0.011, DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic,
        name="door_panel",
    )
    door.visual(
        Box((0.014, 0.055, 0.108)),
        origin=Origin(xyz=(0.024, DOOR_WIDTH - 0.030, DOOR_HEIGHT / 2.0)),
        material=dark,
        name="pull_recess",
    )
    for i, z in enumerate((0.084, 0.203)):
        door.visual(
            Cylinder(radius=0.009, length=0.050),
            origin=Origin(xyz=(0.012, -0.010, z)),
            material=metal,
            name=f"door_barrel_{i}",
        )
        door.visual(
            Box((0.012, 0.030, 0.046)),
            origin=Origin(xyz=(0.011, 0.006, z)),
            material=plastic,
            name=f"door_hinge_leaf_{i}",
        )

    filter_block = model.part("hepa_filter")
    filter_block.visual(
        Box((FILTER_LEN, FILTER_WIDTH, FILTER_HEIGHT)),
        origin=Origin(xyz=(-FILTER_LEN / 2.0, 0.0, 0.0)),
        material=paper,
        name="filter_media",
    )
    filter_block.visual(
        Box((0.018, FILTER_WIDTH + 0.014, FILTER_HEIGHT + 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=gasket,
        name="front_gasket",
    )
    for i, y in enumerate((-0.116, 0.116)):
        filter_block.visual(
            Box((FILTER_LEN, 0.012, 0.030)),
            origin=Origin(xyz=(-FILTER_LEN / 2.0, y, -0.074)),
            material=gasket,
            name=f"rail_shoe_{i}",
        )
    for i, z in enumerate([-0.078, -0.052, -0.026, 0.000, 0.026, 0.052, 0.078]):
        filter_block.visual(
            Box((0.004, FILTER_WIDTH - 0.028, 0.010)),
            origin=Origin(xyz=(0.004, 0.0, z)),
            material=dark,
            name=f"pleat_edge_{i}",
        )
    filter_block.visual(
        Box((0.014, 0.060, 0.034)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=dark,
        name="pull_tab",
    )

    model.articulation(
        "housing_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "housing_to_hepa_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_block,
        origin=Origin(xyz=(FILTER_ORIGIN_X, 0.0, FILTER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=FILTER_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("filter_door")
    hepa = object_model.get_part("hepa_filter")
    door_hinge = object_model.get_articulation("housing_to_filter_door")
    filter_slide = object_model.get_articulation("housing_to_hepa_filter")

    for barrel_name in ("door_barrel_0", "door_barrel_1"):
        ctx.allow_overlap(
            housing,
            door,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The side door barrels are intentionally captured around the fixed vertical hinge pin.",
        )
        ctx.expect_within(
            housing,
            door,
            axes="xy",
            margin=0.006,
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            name=f"{barrel_name} is centered around hinge pin",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="z",
            min_overlap=0.040,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            name=f"{barrel_name} wraps a useful pin length",
        )

    ctx.expect_gap(
        door,
        housing,
        axis="x",
        min_gap=0.002,
        max_gap=0.040,
        positive_elem="door_panel",
        negative_elem="housing_shell",
        name="closed door sits just outside side wall",
    )
    ctx.expect_within(
        hepa,
        housing,
        axes="yz",
        margin=0.002,
        inner_elem="filter_media",
        outer_elem="housing_shell",
        name="filter fits within the side opening height and width",
    )
    ctx.expect_overlap(
        hepa,
        housing,
        axes="x",
        min_overlap=0.080,
        elem_a="filter_media",
        elem_b="housing_shell",
        name="closed filter remains deeply inserted",
    )

    closed_filter_pos = ctx.part_world_position(hepa)
    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.45, filter_slide: FILTER_TRAVEL}):
        ctx.expect_overlap(
            hepa,
            housing,
            axes="x",
            min_overlap=0.035,
            elem_a="filter_media",
            elem_b="housing_shell",
            name="extended filter retains rail insertion",
        )
        ctx.expect_within(
            hepa,
            housing,
            axes="yz",
            margin=0.002,
            inner_elem="filter_media",
            outer_elem="housing_shell",
            name="extended filter stays aligned on rails",
        )
        extended_filter_pos = ctx.part_world_position(hepa)
        open_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "filter slides outward through side door",
        closed_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[0] > closed_filter_pos[0] + 0.12,
        details=f"closed={closed_filter_pos}, extended={extended_filter_pos}",
    )
    ctx.check(
        "door swings outward from vertical left edge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.12,
        details=f"closed_aabb={closed_door_aabb}, open_aabb={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
