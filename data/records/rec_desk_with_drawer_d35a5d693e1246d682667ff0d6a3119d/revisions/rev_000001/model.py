from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]):
    shape = _cq_box(boxes[0][0], boxes[0][1])
    for size, center in boxes[1:]:
        shape = shape.union(_cq_box(size, center))
    return shape


def _l_tabletop_shape():
    thickness = 0.045
    main = _cq_box((1.65, 0.70, thickness), (0.0, 0.0, thickness / 2.0))
    ret = _cq_box((0.70, 1.25, thickness), (-0.475, 0.975, thickness / 2.0))
    top = main.union(ret)
    # Small radiused vertical corners keep the desktop from reading as a raw block.
    try:
        return top.edges("|Z").fillet(0.025)
    except Exception:
        return top


def _lower_frame_shape():
    column_positions = [
        (-0.70, -0.25),
        (0.70, -0.25),
        (-0.70, 1.35),
    ]
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    # Low L-shaped stabilizing feet, like a commercial three-leg corner desk.
    boxes.append(((1.75, 0.085, 0.055), (0.0, -0.25, 0.035)))
    boxes.append(((0.085, 1.72, 0.055), (-0.70, 0.55, 0.035)))

    # Hollow square lower sleeves: four walls around an open telescoping bore.
    outer = 0.092
    wall = 0.012
    height = 0.62
    zc = 0.10 + height / 2.0
    for x, y in column_positions:
        boxes.extend(
            [
                ((wall, outer, height), (x - outer / 2.0 + wall / 2.0, y, zc)),
                ((wall, outer, height), (x + outer / 2.0 - wall / 2.0, y, zc)),
                ((outer, wall, height), (x, y - outer / 2.0 + wall / 2.0, zc)),
                ((outer, wall, height), (x, y + outer / 2.0 - wall / 2.0, zc)),
                ((0.135, 0.135, 0.020), (x, y, 0.095)),
            ]
        )

    # A low rear brace visually ties the return-leg sleeve back to the main foot.
    boxes.append(((0.060, 1.58, 0.040), (-0.70, 0.55, 0.17)))
    return _union_boxes(boxes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_standing_desk")

    wood = model.material("warm_bamboo", rgba=(0.72, 0.50, 0.30, 1.0))
    dark_metal = model.material("dark_powdercoat", rgba=(0.035, 0.040, 0.045, 1.0))
    rail_metal = model.material("black_slide_rail", rgba=(0.010, 0.012, 0.014, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.62, 0.64, 0.65, 1.0))
    black_plastic = model.material("matte_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))

    column_positions = [
        (-0.70, -0.25),
        (0.70, -0.25),
        (-0.70, 1.35),
    ]

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        Box((1.75, 0.085, 0.055)),
        origin=Origin(xyz=(0.0, -0.25, 0.035)),
        material=dark_metal,
        name="front_foot",
    )
    lower_frame.visual(
        Box((0.085, 1.72, 0.055)),
        origin=Origin(xyz=(-0.70, 0.55, 0.035)),
        material=dark_metal,
        name="return_foot",
    )
    lower_frame.visual(
        Box((0.060, 1.58, 0.040)),
        origin=Origin(xyz=(-0.70, 0.55, 0.08)),
        material=dark_metal,
        name="rear_brace",
    )
    lower_frame.visual(
        Box((1.42, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.25, 0.08)),
        material=dark_metal,
        name="front_brace",
    )
    lower_outer = 0.092
    lower_wall = 0.012
    lower_height = 0.63
    lower_z = 0.095 + lower_height / 2.0
    sleeve_wall_names = (
        ("sleeve_wall_0_0", "sleeve_wall_0_1", "sleeve_wall_0_2", "sleeve_wall_0_3"),
        ("sleeve_wall_1_0", "sleeve_wall_1_1", "sleeve_wall_1_2", "sleeve_wall_1_3"),
        ("sleeve_wall_2_0", "sleeve_wall_2_1", "sleeve_wall_2_2", "sleeve_wall_2_3"),
    )
    for index, (x, y) in enumerate(column_positions):
        lower_frame.visual(
            Box((0.135, 0.135, 0.050)),
            origin=Origin(xyz=(x, y, 0.075)),
            material=dark_metal,
            name=f"sleeve_base_{index}",
        )
        lower_frame.visual(
            Box((lower_wall, lower_outer, lower_height)),
            origin=Origin(xyz=(x - lower_outer / 2.0 + lower_wall / 2.0, y, lower_z)),
            material=dark_metal,
            name=sleeve_wall_names[index][0],
        )
        lower_frame.visual(
            Box((lower_wall, lower_outer, lower_height)),
            origin=Origin(xyz=(x + lower_outer / 2.0 - lower_wall / 2.0, y, lower_z)),
            material=dark_metal,
            name=sleeve_wall_names[index][1],
        )
        lower_frame.visual(
            Box((lower_outer, lower_wall, lower_height)),
            origin=Origin(xyz=(x, y - lower_outer / 2.0 + lower_wall / 2.0, lower_z)),
            material=dark_metal,
            name=sleeve_wall_names[index][2],
        )
        lower_frame.visual(
            Box((lower_outer, lower_wall, lower_height)),
            origin=Origin(xyz=(x, y + lower_outer / 2.0 - lower_wall / 2.0, lower_z)),
            material=dark_metal,
            name=sleeve_wall_names[index][3],
        )
    for index, (x, y) in enumerate(column_positions):
        lower_frame.visual(
            Box((0.16, 0.16, 0.012)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=black_plastic,
            name=f"leveling_pad_{index}",
        )

    desktop = model.part("desktop")
    desktop.visual(
        mesh_from_cadquery(_l_tabletop_shape(), "l_tabletop"),
        material=wood,
        name="l_tabletop",
    )

    # Upper telescoping stages and mounting plates move with the desktop.
    upper_column_names = ("upper_column_0", "upper_column_1", "upper_column_2")
    for index, (x, y) in enumerate(column_positions):
        desktop.visual(
            Box((0.068, 0.068, 0.66)),
            origin=Origin(xyz=(x, y, -0.33)),
            material=dark_metal,
            name=upper_column_names[index],
        )
        desktop.visual(
            Box((0.23, 0.16, 0.020)),
            origin=Origin(xyz=(x, y, -0.010)),
            material=dark_metal,
            name=f"mount_plate_{index}",
        )

    # Hidden but visible-from-below steel rails under the L-shaped top.
    desktop.visual(
        Box((1.42, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, -0.25, -0.030)),
        material=dark_metal,
        name="main_underbeam",
    )
    desktop.visual(
        Box((0.055, 1.42, 0.040)),
        origin=Origin(xyz=(-0.70, 0.55, -0.020)),
        material=dark_metal,
        name="return_underbeam",
    )
    desktop.visual(
        Box((0.54, 0.050, 0.035)),
        origin=Origin(xyz=(0.25, 0.045, -0.045)),
        material=dark_metal,
        name="drawer_rear_crossrail",
    )

    # Drawer guide rails live under the front edge of the main wing.
    for name, x in (("drawer_rail_0", 0.025), ("drawer_rail_1", 0.495)):
        desktop.visual(
            Box((0.030, 0.42, 0.035)),
            origin=Origin(xyz=(x, -0.16, -0.105)),
            material=rail_metal,
            name=name,
        )
        for hanger_index, y in enumerate((-0.33, 0.01)):
            desktop.visual(
                Box((0.040, 0.020, 0.110)),
                origin=Origin(xyz=(x, y, -0.050)),
                material=rail_metal,
                name=f"{name}_hanger_{hanger_index}",
            )
    desktop.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(-0.62, 1.14, 0.043)),
        material=black_plastic,
        name="cable_grommet",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.42, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=drawer_gray,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.34, 0.072)),
        origin=Origin(xyz=(-0.204, 0.0, 0.0)),
        material=drawer_gray,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.012, 0.34, 0.072)),
        origin=Origin(xyz=(0.204, 0.0, 0.0)),
        material=drawer_gray,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.42, 0.012, 0.072)),
        origin=Origin(xyz=(0.0, 0.164, 0.0)),
        material=drawer_gray,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.44, 0.025, 0.105)),
        origin=Origin(xyz=(0.0, -0.182, -0.080)),
        material=wood,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.22, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, -0.215, -0.080)),
        material=black_plastic,
        name="drawer_pull",
    )
    for index, x in enumerate((-0.075, 0.075)):
        drawer.visual(
            Box((0.024, 0.030, 0.028)),
            origin=Origin(xyz=(x, -0.199, -0.080)),
            material=black_plastic,
            name=f"pull_post_{index}",
        )

    height_joint = model.articulation(
        "height_lift",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.48),
        motion_properties=MotionProperties(damping=18.0, friction=3.0),
    )
    height_joint.meta["qc_samples"] = [0.0, 0.24, 0.48]

    drawer_joint = model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desktop,
        child=drawer,
        origin=Origin(xyz=(0.25, -0.17, -0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.30),
        motion_properties=MotionProperties(damping=4.0, friction=1.2),
    )
    drawer_joint.meta["qc_samples"] = [0.0, 0.15, 0.30]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    desktop = object_model.get_part("desktop")
    drawer = object_model.get_part("drawer")
    height_lift = object_model.get_articulation("height_lift")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.expect_overlap(
        desktop,
        lower_frame,
        axes="z",
        elem_a="upper_column_0",
        elem_b="sleeve_wall_0_0",
        min_overlap=0.55,
        name="collapsed column remains deeply sleeved",
    )

    base_pos = ctx.part_world_position(desktop)
    with ctx.pose({height_lift: 0.48}):
        raised_pos = ctx.part_world_position(desktop)
        ctx.expect_overlap(
            desktop,
            lower_frame,
            axes="z",
            elem_a="upper_column_0",
            elem_b="sleeve_wall_0_0",
            min_overlap=0.09,
            name="raised column keeps retained insertion",
        )
    ctx.check(
        "desktop lift raises the work surface",
        base_pos is not None and raised_pos is not None and raised_pos[2] > base_pos[2] + 0.45,
        details=f"base={base_pos}, raised={raised_pos}",
    )

    closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.30}):
        open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desktop,
            axes="y",
            elem_a="drawer_side_0",
            elem_b="drawer_rail_0",
            min_overlap=0.06,
            name="open drawer remains captured in the rail",
        )
    ctx.check(
        "drawer slide moves outward from the front edge",
        closed_pos is not None and open_pos is not None and open_pos[1] < closed_pos[1] - 0.25,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
