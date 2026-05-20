from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

DESK_WIDTH = 1.52
DESK_DEPTH = 0.76
DESK_HEIGHT = 0.76
TOP_THICKNESS = 0.03
UNDER_TOP = DESK_HEIGHT - TOP_THICKNESS

SIDE_THICKNESS = 0.022
PARTITION_THICKNESS = 0.018
PEDESTAL_INNER_WIDTH = 0.38
CENTER_OPENING_WIDTH = (
    DESK_WIDTH
    - 2.0 * SIDE_THICKNESS
    - 2.0 * PARTITION_THICKNESS
    - 2.0 * PEDESTAL_INNER_WIDTH
)

LEFT_BAY_CENTER_Y = -0.548
RIGHT_BAY_CENTER_Y = 0.548
DRAWER_FRONT_X = DESK_DEPTH / 2.0 - 0.012

DRAWER_SPECS = (
    {
        "name": "left_drawer_0",
        "side": "left",
        "bay_y": LEFT_BAY_CENTER_Y,
        "front_center_z": 0.6225,
        "front_height": 0.145,
        "body_depth": 0.50,
        "travel": 0.30,
    },
    {
        "name": "left_drawer_1",
        "side": "left",
        "bay_y": LEFT_BAY_CENTER_Y,
        "front_center_z": 0.4715,
        "front_height": 0.145,
        "body_depth": 0.50,
        "travel": 0.30,
    },
    {
        "name": "left_drawer_2",
        "side": "left",
        "bay_y": LEFT_BAY_CENTER_Y,
        "front_center_z": 0.2430,
        "front_height": 0.300,
        "body_depth": 0.52,
        "travel": 0.34,
    },
    {
        "name": "right_drawer_0",
        "side": "right",
        "bay_y": RIGHT_BAY_CENTER_Y,
        "front_center_z": 0.6075,
        "front_height": 0.175,
        "body_depth": 0.50,
        "travel": 0.30,
    },
    {
        "name": "right_drawer_1",
        "side": "right",
        "bay_y": RIGHT_BAY_CENTER_Y,
        "front_center_z": 0.3230,
        "front_height": 0.382,
        "body_depth": 0.52,
        "travel": 0.34,
    },
)


def _runner_z_offset(front_height: float) -> float:
    return -front_height / 2.0 + min(0.065, front_height * 0.42)


def _add_drawer(
    model: ArticulatedObject,
    carcass,
    *,
    name: str,
    bay_y: float,
    front_center_z: float,
    front_height: float,
    body_depth: float,
    travel: float,
    front_material,
    box_material,
    metal_material,
) -> None:
    front_width = 0.370
    front_thickness = 0.024
    body_width = 0.336
    wall_thickness = 0.012
    bottom_thickness = 0.008
    back_thickness = 0.012
    rail_thickness = 0.010
    rail_length = min(body_depth - 0.06, 0.44)
    rail_center_x = -0.02 - rail_length / 2.0
    rail_center_z = _runner_z_offset(front_height)
    side_height = max(front_height - 0.040, 0.095)
    body_center_x = -0.012 - body_depth / 2.0
    body_center_z = -0.010

    drawer = model.part(name)
    drawer.visual(
        Box((front_thickness, front_width, front_height)),
        origin=Origin(),
        material=front_material,
        name="front",
    )

    drawer.visual(
        Box((body_depth, wall_thickness, side_height)),
        origin=Origin(xyz=(body_center_x, -(body_width / 2.0 - wall_thickness / 2.0), body_center_z)),
        material=box_material,
        name="side_0",
    )
    drawer.visual(
        Box((body_depth, wall_thickness, side_height)),
        origin=Origin(xyz=(body_center_x, body_width / 2.0 - wall_thickness / 2.0, body_center_z)),
        material=box_material,
        name="side_1",
    )
    drawer.visual(
        Box((body_depth, body_width - 2.0 * wall_thickness, bottom_thickness)),
        origin=Origin(
            xyz=(body_center_x, 0.0, body_center_z - side_height / 2.0 + bottom_thickness / 2.0)
        ),
        material=box_material,
        name="bottom",
    )
    drawer.visual(
        Box((back_thickness, body_width - 2.0 * wall_thickness, side_height)),
        origin=Origin(xyz=(-0.012 - body_depth + back_thickness / 2.0, 0.0, body_center_z)),
        material=box_material,
        name="back",
    )

    drawer.visual(
        Box((rail_length, rail_thickness, 0.025)),
        origin=Origin(xyz=(rail_center_x, -0.173, rail_center_z)),
        material=metal_material,
        name="runner_0",
    )
    drawer.visual(
        Box((rail_length, rail_thickness, 0.025)),
        origin=Origin(xyz=(rail_center_x, 0.173, rail_center_z)),
        material=metal_material,
        name="runner_1",
    )

    handle_post_depth = 0.018
    handle_post_width = 0.014
    handle_post_height = 0.012
    handle_bar_depth = 0.014
    handle_bar_width = min(0.180, front_width * 0.42)
    handle_bar_height = 0.012
    handle_z = 0.0

    drawer.visual(
        Box((handle_post_depth, handle_post_width, handle_post_height)),
        origin=Origin(
            xyz=(front_thickness / 2.0 + handle_post_depth / 2.0 - 0.001, -handle_bar_width / 2.4, handle_z)
        ),
        material=metal_material,
        name="pull_post_0",
    )
    drawer.visual(
        Box((handle_post_depth, handle_post_width, handle_post_height)),
        origin=Origin(
            xyz=(front_thickness / 2.0 + handle_post_depth / 2.0 - 0.001, handle_bar_width / 2.4, handle_z)
        ),
        material=metal_material,
        name="pull_post_1",
    )
    drawer.visual(
        Box((handle_bar_depth, handle_bar_width, handle_bar_height)),
        origin=Origin(
            xyz=(front_thickness / 2.0 + handle_post_depth + handle_bar_depth / 2.0 - 0.002, 0.0, handle_z)
        ),
        material=metal_material,
        name="pull",
    )

    model.articulation(
        f"carcass_to_{name}",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(DRAWER_FRONT_X, bay_y, front_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_desk")

    top_material = model.material("top_laminate", rgba=(0.55, 0.43, 0.31, 1.0))
    carcass_material = model.material("carcass_laminate", rgba=(0.62, 0.50, 0.36, 1.0))
    drawer_box_material = model.material("drawer_box", rgba=(0.78, 0.77, 0.73, 1.0))
    metal_material = model.material("hardware", rgba=(0.22, 0.23, 0.25, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((DESK_DEPTH, DESK_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0)),
        material=top_material,
        name="top",
    )

    full_side_height = UNDER_TOP
    carcass.visual(
        Box((DESK_DEPTH, SIDE_THICKNESS, full_side_height)),
        origin=Origin(xyz=(0.0, -(DESK_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), full_side_height / 2.0)),
        material=carcass_material,
        name="side_0",
    )
    carcass.visual(
        Box((DESK_DEPTH, SIDE_THICKNESS, full_side_height)),
        origin=Origin(xyz=(0.0, DESK_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, full_side_height / 2.0)),
        material=carcass_material,
        name="side_1",
    )
    carcass.visual(
        Box((DESK_DEPTH, PARTITION_THICKNESS, full_side_height)),
        origin=Origin(xyz=(0.0, -0.349, full_side_height / 2.0)),
        material=carcass_material,
        name="partition_0",
    )
    carcass.visual(
        Box((DESK_DEPTH, PARTITION_THICKNESS, full_side_height)),
        origin=Origin(xyz=(0.0, 0.349, full_side_height / 2.0)),
        material=carcass_material,
        name="partition_1",
    )

    pedestal_width = SIDE_THICKNESS + PEDESTAL_INNER_WIDTH + PARTITION_THICKNESS
    carcass.visual(
        Box((0.72, pedestal_width, 0.022)),
        origin=Origin(xyz=(-0.020, LEFT_BAY_CENTER_Y, 0.011)),
        material=carcass_material,
        name="left_bottom",
    )
    carcass.visual(
        Box((0.72, pedestal_width, 0.022)),
        origin=Origin(xyz=(-0.020, RIGHT_BAY_CENTER_Y, 0.011)),
        material=carcass_material,
        name="right_bottom",
    )

    left_divider_zs = (0.547, 0.396)
    right_divider_zs = (0.516,)
    for index, z in enumerate(left_divider_zs):
        carcass.visual(
            Box((0.72, pedestal_width, 0.018)),
            origin=Origin(xyz=(-0.020, LEFT_BAY_CENTER_Y, z)),
            material=carcass_material,
            name=f"left_divider_{index}",
        )
    for index, z in enumerate(right_divider_zs):
        carcass.visual(
            Box((0.72, pedestal_width, 0.018)),
            origin=Origin(xyz=(-0.020, RIGHT_BAY_CENTER_Y, z)),
            material=carcass_material,
            name=f"right_divider_{index}",
        )

    carcass.visual(
        Box((0.016, pedestal_width, UNDER_TOP)),
        origin=Origin(xyz=(-DESK_DEPTH / 2.0 + 0.008, LEFT_BAY_CENTER_Y, UNDER_TOP / 2.0)),
        material=carcass_material,
        name="left_back",
    )
    carcass.visual(
        Box((0.016, pedestal_width, UNDER_TOP)),
        origin=Origin(xyz=(-DESK_DEPTH / 2.0 + 0.008, RIGHT_BAY_CENTER_Y, UNDER_TOP / 2.0)),
        material=carcass_material,
        name="right_back",
    )

    carcass.visual(
        Box((0.080, CENTER_OPENING_WIDTH, 0.070)),
        origin=Origin(xyz=(DESK_DEPTH / 2.0 - 0.040, 0.0, UNDER_TOP - 0.035)),
        material=carcass_material,
        name="front_apron",
    )
    carcass.visual(
        Box((0.016, CENTER_OPENING_WIDTH, 0.640)),
        origin=Origin(xyz=(-DESK_DEPTH / 2.0 + 0.008, 0.0, 0.320)),
        material=carcass_material,
        name="modesty_panel",
    )

    fixed_runner_center_x = 0.110
    fixed_runner_length = 0.460
    fixed_runner_thickness = 0.012
    for spec in DRAWER_SPECS:
        runner_z = spec["front_center_z"] + _runner_z_offset(spec["front_height"])
        carcass.visual(
            Box((fixed_runner_length, fixed_runner_thickness, 0.030)),
            origin=Origin(xyz=(fixed_runner_center_x, spec["bay_y"] - 0.184, runner_z)),
            material=metal_material,
            name=f"{spec['name']}_runner_0_fixed",
        )
        carcass.visual(
            Box((fixed_runner_length, fixed_runner_thickness, 0.030)),
            origin=Origin(xyz=(fixed_runner_center_x, spec["bay_y"] + 0.184, runner_z)),
            material=metal_material,
            name=f"{spec['name']}_runner_1_fixed",
        )

    for spec in DRAWER_SPECS:
        _add_drawer(
            model,
            carcass,
            name=spec["name"],
            bay_y=spec["bay_y"],
            front_center_z=spec["front_center_z"],
            front_height=spec["front_height"],
            body_depth=spec["body_depth"],
            travel=spec["travel"],
            front_material=carcass_material,
            box_material=drawer_box_material,
            metal_material=metal_material,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    carcass_aabb = ctx.part_world_aabb(carcass)

    if carcass_aabb is None:
        ctx.fail("carcass has measurable geometry", "The desk carcass AABB could not be resolved.")
        return ctx.report()

    carcass_dims = tuple(carcass_aabb[1][axis] - carcass_aabb[0][axis] for axis in range(3))
    front_plane_x = carcass_aabb[1][0]
    ctx.check(
        "desk reads as office workstation scale",
        0.72 <= carcass_dims[0] <= 0.80
        and 1.45 <= carcass_dims[1] <= 1.58
        and 0.74 <= carcass_dims[2] <= 0.78,
        details=f"dims={carcass_dims}",
    )

    for spec in DRAWER_SPECS:
        drawer = object_model.get_part(spec["name"])
        joint = object_model.get_articulation(f"carcass_to_{spec['name']}")
        closed_front_aabb = ctx.part_element_world_aabb(drawer, elem="front")
        closed_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{spec['name']} sits in the correct pedestal",
            closed_pos is not None
            and (
                closed_pos[1] < -0.30 if spec["side"] == "left" else closed_pos[1] > 0.30
            ),
            details=f"position={closed_pos}",
        )
        ctx.check(
            f"{spec['name']} closes flush with the desk front",
            closed_front_aabb is not None and abs(closed_front_aabb[1][0] - front_plane_x) <= 0.001,
            details=f"drawer_front={closed_front_aabb}, desk_front={front_plane_x}",
        )

        upper = 0.0 if joint.motion_limits is None or joint.motion_limits.upper is None else joint.motion_limits.upper
        with ctx.pose({joint: upper}):
            open_pos = ctx.part_world_position(drawer)
            open_front_aabb = ctx.part_element_world_aabb(drawer, elem="front")
            ctx.check(
                f"{spec['name']} extends straight forward",
                closed_pos is not None
                and open_pos is not None
                and open_pos[0] > closed_pos[0] + upper - 0.005
                and abs(open_pos[1] - closed_pos[1]) <= 0.002
                and abs(open_pos[2] - closed_pos[2]) <= 0.002,
                details=f"closed={closed_pos}, open={open_pos}, travel={upper}",
            )
            ctx.check(
                f"{spec['name']} opens visibly beyond the desk face",
                open_front_aabb is not None and open_front_aabb[1][0] >= front_plane_x + upper - 0.005,
                details=f"open_front={open_front_aabb}, desk_front={front_plane_x}, travel={upper}",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="runner_0",
                elem_b=f"{spec['name']}_runner_0_fixed",
                min_overlap=0.08,
                name=f"{spec['name']} retains runner engagement at full extension",
            )

    return ctx.report()


object_model = build_object_model()
