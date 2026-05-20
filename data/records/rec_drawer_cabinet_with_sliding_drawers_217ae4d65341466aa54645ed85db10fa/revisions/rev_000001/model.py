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


CABINET_WIDTH = 0.58
CABINET_DEPTH = 0.54
CABINET_HEIGHT = 1.55

SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.009
TOP_THICKNESS = 0.018
FRAME_THICKNESS = 0.018
DECK_THICKNESS = 0.012
BASE_FLOOR_THICKNESS = 0.012

TOE_KICK_HEIGHT = 0.125
BOTTOM_RAIL_HEIGHT = 0.030
INTER_RAIL_HEIGHT = 0.018
DRAWER_OPENING_HEIGHT = 0.244
DRAWER_FACE_HEIGHT = 0.240
TOP_HEADER_HEIGHT = (
    CABINET_HEIGHT
    - TOE_KICK_HEIGHT
    - BOTTOM_RAIL_HEIGHT
    - 4.0 * INTER_RAIL_HEIGHT
    - 5.0 * DRAWER_OPENING_HEIGHT
)

DRAWER_COUNT = 5
DRAWER_FACE_THICKNESS = 0.018
DRAWER_FACE_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS - 0.012
DRAWER_BIN_WIDTH = DRAWER_FACE_WIDTH - 0.034
DRAWER_BIN_HEIGHT = 0.198
DRAWER_BIN_DEPTH = 0.440
DRAWER_SIDE_THICKNESS = 0.008
DRAWER_BOTTOM_THICKNESS = 0.006
DRAWER_BACK_THICKNESS = 0.008

RUNNER_THICKNESS = 0.006
RUNNER_HEIGHT = 0.012
RUNNER_LENGTH = DRAWER_BIN_DEPTH - 0.030

GUIDE_THICKNESS = 0.017
GUIDE_HEIGHT = 0.018
GUIDE_LENGTH = 0.410
GUIDE_CENTER_X = 0.005

HANDLE_DEPTH = 0.020
HANDLE_HEIGHT = 0.012
HANDLE_LENGTH = 0.280
HANDLE_MOUNT_DEPTH = 0.012
HANDLE_MOUNT_WIDTH = 0.028
HANDLE_MOUNT_HEIGHT = 0.028

DRAWER_TRAVEL = 0.280

TOE_RECESS = 0.025
TOE_FLAP_WIDTH = 0.456
TOE_FLAP_HEIGHT = 0.104
TOE_FLAP_THICKNESS = 0.014
TOE_HINGE_Z = 0.012
TOE_FLAP_OPEN = 1.22

BODY_FRONT_X = CABINET_DEPTH / 2.0
BODY_FRAME_CENTER_X = BODY_FRONT_X - FRAME_THICKNESS / 2.0
TOE_FACE_X = BODY_FRONT_X - TOE_RECESS


def drawer_center_z(index: int) -> float:
    return (
        TOE_KICK_HEIGHT
        + BOTTOM_RAIL_HEIGHT
        + DRAWER_OPENING_HEIGHT / 2.0
        + index * (DRAWER_OPENING_HEIGHT + INTER_RAIL_HEIGHT)
    )


def add_drawer(
    model: ArticulatedObject,
    body,
    drawer_index: int,
    drawer_finish: str,
    rail_finish: str,
    handle_finish: str,
) -> None:
    center_z = drawer_center_z(drawer_index)
    drawer = model.part(f"drawer_{drawer_index}")

    drawer.visual(
        Box((DRAWER_FACE_THICKNESS, DRAWER_FACE_WIDTH, DRAWER_FACE_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_FACE_THICKNESS / 2.0, 0.0, 0.0)),
        material=drawer_finish,
        name="drawer_front",
    )

    bin_center_x = -DRAWER_FACE_THICKNESS - DRAWER_BIN_DEPTH / 2.0
    side_center_y = (DRAWER_BIN_WIDTH - DRAWER_SIDE_THICKNESS) / 2.0

    drawer.visual(
        Box((DRAWER_BIN_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BIN_HEIGHT)),
        origin=Origin(xyz=(bin_center_x, side_center_y, 0.0)),
        material=drawer_finish,
        name="side_0",
    )
    drawer.visual(
        Box((DRAWER_BIN_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BIN_HEIGHT)),
        origin=Origin(xyz=(bin_center_x, -side_center_y, 0.0)),
        material=drawer_finish,
        name="side_1",
    )
    drawer.visual(
        Box((DRAWER_BIN_DEPTH, DRAWER_BIN_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                bin_center_x,
                0.0,
                -DRAWER_BIN_HEIGHT / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0,
            )
        ),
        material=drawer_finish,
        name="bottom",
    )
    drawer.visual(
        Box((DRAWER_BACK_THICKNESS, DRAWER_BIN_WIDTH, DRAWER_BIN_HEIGHT)),
        origin=Origin(
            xyz=(
                -DRAWER_FACE_THICKNESS - DRAWER_BIN_DEPTH + DRAWER_BACK_THICKNESS / 2.0,
                0.0,
                0.0,
            )
        ),
        material=drawer_finish,
        name="back",
    )

    runner_center_x = -DRAWER_FACE_THICKNESS - DRAWER_BIN_DEPTH / 2.0 + 0.005
    runner_center_y = DRAWER_BIN_WIDTH / 2.0 + RUNNER_THICKNESS / 2.0
    drawer.visual(
        Box((RUNNER_LENGTH, RUNNER_THICKNESS, RUNNER_HEIGHT)),
        origin=Origin(xyz=(runner_center_x, runner_center_y, 0.0)),
        material=rail_finish,
        name="runner_l",
    )
    drawer.visual(
        Box((RUNNER_LENGTH, RUNNER_THICKNESS, RUNNER_HEIGHT)),
        origin=Origin(xyz=(runner_center_x, -runner_center_y, 0.0)),
        material=rail_finish,
        name="runner_r",
    )

    handle_z = DRAWER_FACE_HEIGHT / 2.0 - 0.050
    handle_mount_y = 0.110
    drawer.visual(
        Box((HANDLE_MOUNT_DEPTH, HANDLE_MOUNT_WIDTH, HANDLE_MOUNT_HEIGHT)),
        origin=Origin(xyz=(HANDLE_MOUNT_DEPTH / 2.0, handle_mount_y, handle_z)),
        material=handle_finish,
        name="handle_mount_0",
    )
    drawer.visual(
        Box((HANDLE_MOUNT_DEPTH, HANDLE_MOUNT_WIDTH, HANDLE_MOUNT_HEIGHT)),
        origin=Origin(xyz=(HANDLE_MOUNT_DEPTH / 2.0, -handle_mount_y, handle_z)),
        material=handle_finish,
        name="handle_mount_1",
    )
    drawer.visual(
        Box((HANDLE_DEPTH, HANDLE_LENGTH, HANDLE_HEIGHT)),
        origin=Origin(
            xyz=(
                HANDLE_MOUNT_DEPTH + HANDLE_DEPTH / 2.0 - 0.002,
                0.0,
                handle_z,
            )
        ),
        material=handle_finish,
        name="handle_bar",
    )

    model.articulation(
        f"body_to_drawer_{drawer_index}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=120.0,
            velocity=0.30,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_supply_drawer_cabinet")

    body_finish = model.material("body_finish", rgba=(0.93, 0.94, 0.95, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.95, 0.96, 0.97, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.65, 0.69, 0.73, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.34, 0.37, 0.41, 1.0))
    toe_finish = model.material("toe_finish", rgba=(0.30, 0.32, 0.35, 1.0))

    body = model.part("body")

    side_depth = CABINET_DEPTH - BACK_THICKNESS
    side_center_x = BACK_THICKNESS / 2.0
    body.visual(
        Box((side_depth, SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                side_center_x,
                (CABINET_WIDTH - SIDE_THICKNESS) / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="side_0",
    )
    body.visual(
        Box((side_depth, SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                side_center_x,
                -(CABINET_WIDTH - SIDE_THICKNESS) / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="side_1",
    )
    body.visual(
        Box((BACK_THICKNESS, CABINET_WIDTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                0.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="back",
    )
    body.visual(
        Box((CABINET_DEPTH - BACK_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                side_center_x,
                0.0,
                CABINET_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=body_finish,
        name="top",
    )
    body.visual(
        Box((CABINET_DEPTH - BACK_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                side_center_x,
                0.0,
                TOE_KICK_HEIGHT + DECK_THICKNESS / 2.0,
            )
        ),
        material=body_finish,
        name="deck",
    )
    body.visual(
        Box((TOE_FACE_X + CABINET_DEPTH / 2.0 - BACK_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BASE_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                (TOE_FACE_X - BACK_THICKNESS - 0.0) / 2.0,
                0.0,
                BASE_FLOOR_THICKNESS / 2.0,
            )
        ),
        material=toe_finish,
        name="base_floor",
    )

    body.visual(
        Box((FRAME_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, TOP_HEADER_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_FRAME_CENTER_X,
                0.0,
                CABINET_HEIGHT - TOP_HEADER_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="top_header",
    )
    body.visual(
        Box((FRAME_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_FRAME_CENTER_X,
                0.0,
                TOE_KICK_HEIGHT + BOTTOM_RAIL_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="bottom_rail",
    )

    for drawer_index in range(DRAWER_COUNT - 1):
        inter_rail_bottom = (
            TOE_KICK_HEIGHT
            + BOTTOM_RAIL_HEIGHT
            + (drawer_index + 1) * DRAWER_OPENING_HEIGHT
            + drawer_index * INTER_RAIL_HEIGHT
        )
        body.visual(
            Box((FRAME_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, INTER_RAIL_HEIGHT)),
            origin=Origin(
                xyz=(
                    BODY_FRAME_CENTER_X,
                    0.0,
                    inter_rail_bottom + INTER_RAIL_HEIGHT / 2.0,
                )
            ),
            material=body_finish,
            name=f"inter_rail_{drawer_index}",
        )

    guide_center_y = CABINET_WIDTH / 2.0 - SIDE_THICKNESS - GUIDE_THICKNESS / 2.0
    for drawer_index in range(DRAWER_COUNT):
        center_z = drawer_center_z(drawer_index)
        body.visual(
            Box((GUIDE_LENGTH, GUIDE_THICKNESS, GUIDE_HEIGHT)),
            origin=Origin(xyz=(GUIDE_CENTER_X, guide_center_y, center_z)),
            material=rail_finish,
            name=f"guide_{drawer_index}_l",
        )
        body.visual(
            Box((GUIDE_LENGTH, GUIDE_THICKNESS, GUIDE_HEIGHT)),
            origin=Origin(xyz=(GUIDE_CENTER_X, -guide_center_y, center_z)),
            material=rail_finish,
            name=f"guide_{drawer_index}_r",
        )

    for drawer_index in range(DRAWER_COUNT):
        add_drawer(
            model=model,
            body=body,
            drawer_index=drawer_index,
            drawer_finish=drawer_finish,
            rail_finish=rail_finish,
            handle_finish=handle_finish,
        )

    toe_flap = model.part("toe_flap")
    toe_flap.visual(
        Box((TOE_FLAP_THICKNESS, TOE_FLAP_WIDTH, TOE_FLAP_HEIGHT)),
        origin=Origin(xyz=(-TOE_FLAP_THICKNESS / 2.0, 0.0, TOE_FLAP_HEIGHT / 2.0)),
        material=toe_finish,
        name="toe_panel",
    )
    toe_flap.visual(
        Box((0.010, 0.026, 0.020)),
        origin=Origin(xyz=(0.005, 0.060, TOE_FLAP_HEIGHT - 0.026)),
        material=handle_finish,
        name="toe_pull_mount_0",
    )
    toe_flap.visual(
        Box((0.010, 0.026, 0.020)),
        origin=Origin(xyz=(0.005, -0.060, TOE_FLAP_HEIGHT - 0.026)),
        material=handle_finish,
        name="toe_pull_mount_1",
    )
    toe_flap.visual(
        Box((0.018, 0.160, 0.012)),
        origin=Origin(xyz=(0.013, 0.0, TOE_FLAP_HEIGHT - 0.026)),
        material=handle_finish,
        name="toe_pull_bar",
    )

    model.articulation(
        "body_to_toe_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=toe_flap,
        origin=Origin(xyz=(TOE_FACE_X, 0.0, TOE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOE_FLAP_OPEN,
            effort=20.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    toe_flap = object_model.get_part("toe_flap")
    toe_joint = object_model.get_articulation("body_to_toe_flap")

    def element_aabb(part_name: str, elem_name: str):
        return ctx.part_element_world_aabb(part_name, elem=elem_name)

    def element_max_x(part_name: str, elem_name: str):
        aabb = element_aabb(part_name, elem_name)
        return None if aabb is None else aabb[1][0]

    def element_max_z(part_name: str, elem_name: str):
        aabb = element_aabb(part_name, elem_name)
        return None if aabb is None else aabb[1][2]

    body_front_x = element_max_x("body", "top_header")

    for drawer_index in range(DRAWER_COUNT):
        drawer = object_model.get_part(f"drawer_{drawer_index}")
        joint = object_model.get_articulation(f"body_to_drawer_{drawer_index}")
        closed_front_x = element_max_x(drawer.name, "drawer_front")

        ctx.check(
            f"drawer_{drawer_index} front sits flush with cabinet face",
            body_front_x is not None
            and closed_front_x is not None
            and abs(closed_front_x - body_front_x) <= 0.0015,
            details=f"drawer_front_x={closed_front_x}, body_front_x={body_front_x}",
        )

        with ctx.pose({joint: DRAWER_TRAVEL}):
            open_front_x = element_max_x(drawer.name, "drawer_front")
            ctx.expect_contact(
                drawer,
                body,
                elem_a="runner_l",
                elem_b=f"guide_{drawer_index}_l",
                name=f"drawer_{drawer_index} left runner stays in contact with guide",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="runner_r",
                elem_b=f"guide_{drawer_index}_r",
                name=f"drawer_{drawer_index} right runner stays in contact with guide",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="runner_l",
                elem_b=f"guide_{drawer_index}_l",
                min_overlap=0.085,
                name=f"drawer_{drawer_index} left runner retains insertion",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="runner_r",
                elem_b=f"guide_{drawer_index}_r",
                min_overlap=0.085,
                name=f"drawer_{drawer_index} right runner retains insertion",
            )

        ctx.check(
            f"drawer_{drawer_index} extends outward",
            closed_front_x is not None
            and open_front_x is not None
            and open_front_x > closed_front_x + 0.20,
            details=f"closed_front_x={closed_front_x}, open_front_x={open_front_x}",
        )

    toe_closed_x = element_max_x(toe_flap.name, "toe_panel")
    toe_closed_z = element_max_z(toe_flap.name, "toe_panel")
    ctx.check(
        "toe flap remains recessed behind drawer fronts when closed",
        body_front_x is not None
        and toe_closed_x is not None
        and toe_closed_x <= body_front_x - 0.020,
        details=f"toe_closed_x={toe_closed_x}, body_front_x={body_front_x}",
    )

    with ctx.pose({toe_joint: TOE_FLAP_OPEN}):
        toe_open_x = element_max_x(toe_flap.name, "toe_panel")
        toe_open_z = element_max_z(toe_flap.name, "toe_panel")

    ctx.check(
        "toe flap rotates downward and outward",
        toe_closed_x is not None
        and toe_open_x is not None
        and toe_closed_z is not None
        and toe_open_z is not None
        and toe_open_x > toe_closed_x + 0.075
        and toe_open_z < toe_closed_z - 0.050,
        details=(
            f"toe_closed_x={toe_closed_x}, toe_open_x={toe_open_x}, "
            f"toe_closed_z={toe_closed_z}, toe_open_z={toe_open_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
