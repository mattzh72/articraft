from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_electric_oven")

    wall_paint = model.material("warm_wall_paint", rgba=(0.82, 0.80, 0.74, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.62, 0.65, 0.65, 1.0))
    dark_liner = model.material("dark_enamel", rgba=(0.015, 0.016, 0.018, 1.0))
    black_trim = model.material("black_trim", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoked_glass", rgba=(0.05, 0.09, 0.11, 0.55))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    heat = model.material("warm_heating_element", rgba=(1.0, 0.27, 0.04, 1.0))

    # World frame: +X goes into the wall/cabinet, +Y is oven width, +Z is up.
    center_z = 1.35
    wall = model.part("wall_frame")
    # A simple wall/cabinet face made from four connected bars around the oven cutout.
    wall.visual(
        Box((0.08, 1.10, 0.16)),
        origin=Origin(xyz=(0.040, 0.0, center_z + 0.40)),
        material=wall_paint,
        name="wall_top",
    )
    wall.visual(
        Box((0.08, 1.10, 0.16)),
        origin=Origin(xyz=(0.040, 0.0, center_z - 0.40)),
        material=wall_paint,
        name="wall_bottom",
    )
    wall.visual(
        Box((0.08, 0.16, 0.90)),
        origin=Origin(xyz=(0.040, 0.47, center_z)),
        material=wall_paint,
        name="wall_side_0",
    )
    wall.visual(
        Box((0.08, 0.16, 0.90)),
        origin=Origin(xyz=(0.040, -0.47, center_z)),
        material=wall_paint,
        name="wall_side_1",
    )

    housing = model.part("housing")
    # Stainless trim/bezel around the oven mouth; it sits just in front of the wall.
    front_flange = BezelGeometry(
        opening_size=(0.58, 0.44),
        outer_size=(0.86, 0.72),
        depth=0.035,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.030,
        outer_corner_radius=0.035,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.004),
    )
    housing.visual(
        mesh_from_geometry(front_flange, "front_flange_mesh"),
        origin=Origin(xyz=(-0.0175, 0.0, center_z), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=stainless,
        name="front_flange",
    )

    # Hollow oven box: individual walls leave a real front cavity.
    housing.visual(
        Box((0.56, 0.68, 0.040)),
        origin=Origin(xyz=(0.270, 0.0, center_z + 0.280)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((0.56, 0.68, 0.040)),
        origin=Origin(xyz=(0.270, 0.0, center_z - 0.280)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.56, 0.040, 0.56)),
        origin=Origin(xyz=(0.270, 0.320, center_z)),
        material=stainless,
        name="side_shell_0",
    )
    housing.visual(
        Box((0.56, 0.040, 0.56)),
        origin=Origin(xyz=(0.270, -0.320, center_z)),
        material=stainless,
        name="side_shell_1",
    )
    housing.visual(
        Box((0.035, 0.68, 0.56)),
        origin=Origin(xyz=(0.5375, 0.0, center_z)),
        material=stainless,
        name="rear_shell",
    )
    # Dark enamel liner visible through the glass door.
    housing.visual(
        Box((0.505, 0.020, 0.48)),
        origin=Origin(xyz=(0.270, 0.290, center_z)),
        material=dark_liner,
        name="inner_side_0",
    )
    housing.visual(
        Box((0.505, 0.020, 0.48)),
        origin=Origin(xyz=(0.270, -0.290, center_z)),
        material=dark_liner,
        name="inner_side_1",
    )
    housing.visual(
        Box((0.020, 0.58, 0.48)),
        origin=Origin(xyz=(0.505, 0.0, center_z)),
        material=dark_liner,
        name="inner_back",
    )
    housing.visual(
        Box((0.505, 0.58, 0.018)),
        origin=Origin(xyz=(0.270, 0.0, center_z + 0.245)),
        material=dark_liner,
        name="inner_ceiling",
    )
    housing.visual(
        Box((0.505, 0.58, 0.018)),
        origin=Origin(xyz=(0.270, 0.0, center_z - 0.245)),
        material=dark_liner,
        name="inner_floor",
    )

    rod_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    for i, z in enumerate((center_z - 0.145, center_z + 0.035)):
        housing.visual(
            Cylinder(radius=0.0045, length=0.61),
            origin=Origin(xyz=(0.180, 0.0, z), rpy=rod_y.rpy),
            material=chrome,
            name=f"rack_front_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0045, length=0.61),
            origin=Origin(xyz=(0.390, 0.0, z), rpy=rod_y.rpy),
            material=chrome,
            name=f"rack_rear_{i}",
        )
    for i, (x, z) in enumerate(((0.230, center_z - 0.215), (0.355, center_z + 0.205))):
        housing.visual(
            Cylinder(radius=0.007, length=0.61),
            origin=Origin(xyz=(x, 0.0, z), rpy=rod_y.rpy),
            material=heat,
            name=f"heating_rod_{i}",
        )

    # Fixed center hinge knuckle and leaf mounted to the lower front flange.
    hinge_x = -0.070
    hinge_z = center_z - 0.255
    housing.visual(
        Box((0.070, 0.235, 0.050)),
        origin=Origin(xyz=(-0.040, 0.0, hinge_z)),
        material=stainless,
        name="hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.235),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=rod_y.rpy),
        material=stainless,
        name="hinge_barrel",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.67),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=rod_y.rpy),
        material=chrome,
        name="hinge_pin",
    )

    door = model.part("door")
    door_frame = BezelGeometry(
        opening_size=(0.50, 0.31),
        outer_size=(0.66, 0.50),
        depth=0.055,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.020,
        outer_corner_radius=0.030,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.003),
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame_mesh"),
        origin=Origin(xyz=(0.0, 0.285, 0.0)),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Box((0.54, 0.35, 0.007)),
        origin=Origin(xyz=(0.0, 0.285, -0.031)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.515, 0.325),
                outer_size=(0.565, 0.375),
                depth=0.008,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.023,
            ),
            "glass_gasket_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.285, -0.034)),
        material=black_trim,
        name="glass_gasket",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(0.0, 0.455, -0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="pull_handle",
    )
    for i, x in enumerate((-0.225, 0.225)):
        door.visual(
            Cylinder(radius=0.012, length=0.064),
            origin=Origin(xyz=(x, 0.455, -0.057)),
            material=stainless,
            name=f"handle_post_{i}",
        )
        door.visual(
            Box((0.050, 0.055, 0.013)),
            origin=Origin(xyz=(x, 0.455, -0.025)),
            material=stainless,
            name=f"handle_foot_{i}",
        )
    for i, x in enumerate((-0.220, 0.220)):
        door.visual(
            Box((0.175, 0.064, 0.012)),
            origin=Origin(xyz=(x, 0.032, 0.017)),
            material=stainless,
            name=f"hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.018, length=0.165),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"hinge_knuckle_{i}",
        )

    model.articulation(
        "wall_to_housing",
        ArticulationType.FIXED,
        parent=wall,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        # The joint frame maps local X to the real hinge line (world +Y),
        # local Y upward along the closed door, and local Z into the oven.
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=math.radians(92.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_frame")
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    for i in (0, 1):
        ctx.allow_overlap(
            housing,
            door,
            elem_a="hinge_pin",
            elem_b=f"hinge_knuckle_{i}",
            reason="The continuous hinge pin is intentionally captured inside the moving door knuckle.",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="xyz",
            elem_a="hinge_pin",
            elem_b=f"hinge_knuckle_{i}",
            min_overlap=0.008,
            name=f"hinge pin is captured by door knuckle {i}",
        )

    ctx.expect_contact(
        wall,
        housing,
        elem_a="wall_bottom",
        elem_b="front_flange",
        contact_tol=0.002,
        name="stainless mounting flange seats against the wall face",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="x",
        positive_elem="front_flange",
        negative_elem="door_frame",
        min_gap=0.004,
        max_gap=0.012,
        name="closed door stands just proud of stainless flange",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="yz",
        elem_a="door_frame",
        elem_b="front_flange",
        min_overlap=0.45,
        name="closed door covers the oven mouth",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
    with ctx.pose({hinge: math.radians(82.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
    ctx.check(
        "drop-down hinge moves handle outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] < closed_aabb[0][0] - 0.18
        and open_aabb[0][2] < closed_aabb[0][2] - 0.30,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
