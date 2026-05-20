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


DESK_HEIGHT = 0.748
TOP_THICKNESS = 0.036
SUPPORT_HEIGHT = DESK_HEIGHT - TOP_THICKNESS
TOP_Z = SUPPORT_HEIGHT + TOP_THICKNESS / 2.0

CARCASS_FRONT_Y = 0.691
CARCASS_DEPTH = 0.580
CARCASS_Y = CARCASS_FRONT_Y + CARCASS_DEPTH / 2.0
CARCASS_X = 0.556


def _add_drawer(
    model: ArticulatedObject,
    wood: str,
    hardware: str,
    *,
    name: str,
    joint_name: str,
    z_center: float,
) -> None:
    drawer = model.part(name)

    drawer.visual(
        Box((0.512, 0.018, 0.270)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=wood,
        name="front",
    )
    drawer.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=hardware,
        name="pull",
    )
    drawer.visual(
        Box((0.466, 0.448, 0.012)),
        origin=Origin(xyz=(0.0, 0.224, -0.102)),
        material=wood,
        name="bottom",
    )
    drawer.visual(
        Box((0.012, 0.448, 0.180)),
        origin=Origin(xyz=(-0.239, 0.224, -0.018)),
        material=wood,
        name="side_left",
    )
    drawer.visual(
        Box((0.012, 0.448, 0.180)),
        origin=Origin(xyz=(0.239, 0.224, -0.018)),
        material=wood,
        name="side_right",
    )
    drawer.visual(
        Box((0.466, 0.012, 0.180)),
        origin=Origin(xyz=(0.0, 0.442, -0.018)),
        material=wood,
        name="back",
    )
    drawer.visual(
        Box((0.010, 0.400, 0.020)),
        origin=Origin(xyz=(-0.249, 0.220, -0.018)),
        material=hardware,
        name="runner_left",
    )
    drawer.visual(
        Box((0.010, 0.400, 0.020)),
        origin=Origin(xyz=(0.249, 0.220, -0.018)),
        material=hardware,
        name="runner_right",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="desk",
        child=drawer,
        origin=Origin(xyz=(CARCASS_X, CARCASS_FRONT_Y, z_center)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.320,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_desk")

    walnut = model.material("walnut", rgba=(0.43, 0.29, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))

    desk = model.part("desk")

    desk.visual(
        Box((1.650, 0.720, TOP_THICKNESS)),
        origin=Origin(xyz=(0.025, 0.360, TOP_Z)),
        material=walnut,
        name="main_top",
    )
    desk.visual(
        Box((0.560, 1.050, TOP_THICKNESS)),
        origin=Origin(xyz=(0.570, 1.245, TOP_Z)),
        material=walnut,
        name="return_top",
    )
    desk.visual(
        Box((0.050, 0.650, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(-0.720, 0.395, SUPPORT_HEIGHT / 2.0)),
        material=graphite,
        name="left_panel",
    )
    desk.visual(
        Box((0.055, 0.055, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.255, 0.080, SUPPORT_HEIGHT / 2.0)),
        material=steel,
        name="front_leg",
    )
    desk.visual(
        Box((0.940, 0.018, 0.450)),
        origin=Origin(xyz=(-0.180, 0.691, 0.487)),
        material=graphite,
        name="rear_modesty",
    )
    desk.visual(
        Box((0.018, 0.581, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.290, 0.9815, SUPPORT_HEIGHT / 2.0)),
        material=graphite,
        name="corner_panel",
    )
    desk.visual(
        Box((0.018, 0.580, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.831, CARCASS_Y, SUPPORT_HEIGHT / 2.0)),
        material=graphite,
        name="outer_side",
    )
    desk.visual(
        Box((0.550, 0.018, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(CARCASS_X, 1.271, SUPPORT_HEIGHT / 2.0)),
        material=graphite,
        name="carcass_back",
    )
    desk.visual(
        Box((0.550, 0.580, 0.018)),
        origin=Origin(xyz=(CARCASS_X, CARCASS_Y, 0.009)),
        material=graphite,
        name="carcass_bottom",
    )
    desk.visual(
        Box((0.514, 0.540, 0.018)),
        origin=Origin(xyz=(CARCASS_X, 0.981, 0.342)),
        material=graphite,
        name="mid_shelf",
    )
    desk.visual(
        Box((0.514, 0.018, 0.060)),
        origin=Origin(xyz=(CARCASS_X, CARCASS_FRONT_Y, 0.665)),
        material=graphite,
        name="top_rail",
    )

    runner_specs = (
        ("upper_runner_housing_left", (0.300, 0.951, 0.481)),
        ("upper_runner_housing_right", (0.812, 0.951, 0.481)),
        ("lower_runner_housing_left", (0.300, 0.951, 0.166)),
        ("lower_runner_housing_right", (0.812, 0.951, 0.166)),
    )
    for visual_name, xyz in runner_specs:
        desk.visual(
            Box((0.024, 0.480, 0.034)),
            origin=Origin(xyz=xyz),
            material=steel,
            name=visual_name,
        )

    _add_drawer(
        model,
        walnut,
        steel,
        name="drawer_0",
        joint_name="drawer_0_slide",
        z_center=0.499,
    )
    _add_drawer(
        model,
        walnut,
        steel,
        name="drawer_1",
        joint_name="drawer_1_slide",
        z_center=0.184,
    )

    return model


def _check_drawer_slide(
    ctx: TestContext,
    *,
    drawer_name: str,
    joint_name: str,
    left_housing_name: str,
    right_housing_name: str,
) -> None:
    drawer = object_model.get_part(drawer_name)
    joint = object_model.get_articulation(joint_name)
    upper = 0.0
    if joint.motion_limits is not None and joint.motion_limits.upper is not None:
        upper = joint.motion_limits.upper

    ctx.allow_overlap(
        drawer,
        "desk",
        elem_a="runner_left",
        elem_b=left_housing_name,
        reason="The drawer uses a simplified telescoping slide housing around the moving runner.",
    )
    ctx.allow_overlap(
        drawer,
        "desk",
        elem_a="runner_right",
        elem_b=right_housing_name,
        reason="The drawer uses a simplified telescoping slide housing around the moving runner.",
    )

    ctx.expect_within(
        drawer,
        "desk",
        axes="xz",
        inner_elem="runner_left",
        outer_elem=left_housing_name,
        margin=0.003,
        name=f"{drawer_name} left runner stays captured when closed",
    )
    ctx.expect_overlap(
        drawer,
        "desk",
        axes="y",
        elem_a="runner_left",
        elem_b=left_housing_name,
        min_overlap=0.300,
        name=f"{drawer_name} left runner has deep closed engagement",
    )

    rest_position = ctx.part_world_position(drawer)
    with ctx.pose({joint: upper}):
        ctx.expect_within(
            drawer,
            "desk",
            axes="xz",
            inner_elem="runner_left",
            outer_elem=left_housing_name,
            margin=0.003,
            name=f"{drawer_name} left runner stays captured when extended",
        )
        ctx.expect_overlap(
            drawer,
            "desk",
            axes="y",
            elem_a="runner_left",
            elem_b=left_housing_name,
            min_overlap=0.075,
            name=f"{drawer_name} left runner keeps retained insertion",
        )
        extended_position = ctx.part_world_position(drawer)

    ctx.check(
        f"{drawer_name} extends toward the seated side",
        rest_position is not None
        and extended_position is not None
        and extended_position[1] < rest_position[1] - 0.250,
        details=f"rest={rest_position}, extended={extended_position}",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    _check_drawer_slide(
        ctx,
        drawer_name="drawer_0",
        joint_name="drawer_0_slide",
        left_housing_name="upper_runner_housing_left",
        right_housing_name="upper_runner_housing_right",
    )
    _check_drawer_slide(
        ctx,
        drawer_name="drawer_1",
        joint_name="drawer_1_slide",
        left_housing_name="lower_runner_housing_left",
        right_housing_name="lower_runner_housing_right",
    )

    return ctx.report()


object_model = build_object_model()
