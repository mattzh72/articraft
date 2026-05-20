from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.52
CABINET_DEPTH = 0.42
CABINET_BODY_HEIGHT = 0.662
LID_THICKNESS = 0.018
LID_WIDTH = 0.516
LID_DEPTH = 0.416
SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.012
PANEL_THICKNESS = 0.018
SHELF_HEIGHT = 0.53
DRAWER_FRONT_THICKNESS = 0.018
DRAWER_FRONT_WIDTH = 0.512
DRAWER_FRONT_HEIGHT = 0.173
DRAWER_BOX_DEPTH = 0.33
DRAWER_BOX_WIDTH = 0.448
DRAWER_BOX_HEIGHT = 0.14
DRAWER_PANEL_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.01
DRAWER_TRAVEL = 0.24
DRAWER_CENTER_Z = (0.0905, 0.2675, 0.4445)


def _add_cabinet_body(model: ArticulatedObject):
    body = model.part("body")

    side_origin_z = CABINET_BODY_HEIGHT / 2.0
    body.visual(
        Box((CABINET_DEPTH, SIDE_THICKNESS, CABINET_BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, (CABINET_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0), side_origin_z)
        ),
        material="oak_case",
        name="side_0",
    )
    body.visual(
        Box((CABINET_DEPTH, SIDE_THICKNESS, CABINET_BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(CABINET_WIDTH / 2.0) + (SIDE_THICKNESS / 2.0), side_origin_z)
        ),
        material="oak_case",
        name="side_1",
    )
    body.visual(
        Box((BACK_THICKNESS, CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_DEPTH / 2.0) + (BACK_THICKNESS / 2.0),
                0.0,
                side_origin_z,
            )
        ),
        material="oak_case",
        name="back",
    )
    body.visual(
        Box((CABINET_DEPTH - BACK_THICKNESS, CABINET_WIDTH - (2.0 * SIDE_THICKNESS), PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS / 2.0,
                0.0,
                PANEL_THICKNESS / 2.0,
            )
        ),
        material="oak_case",
        name="bottom",
    )
    body.visual(
        Box((CABINET_DEPTH - BACK_THICKNESS, CABINET_WIDTH - (2.0 * SIDE_THICKNESS), PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS / 2.0,
                0.0,
                SHELF_HEIGHT,
            )
        ),
        material="oak_case",
        name="compartment_floor",
    )

    top_band_height = CABINET_BODY_HEIGHT - (SHELF_HEIGHT + (PANEL_THICKNESS / 2.0))
    body.visual(
        Box((PANEL_THICKNESS, CABINET_WIDTH - (2.0 * SIDE_THICKNESS), top_band_height)),
        origin=Origin(
            xyz=(
                (CABINET_DEPTH / 2.0) - (PANEL_THICKNESS / 2.0),
                0.0,
                SHELF_HEIGHT + (PANEL_THICKNESS / 2.0) + (top_band_height / 2.0),
            )
        ),
        material="oak_case",
        name="compartment_front",
    )

    return body


def _add_lid(model: ArticulatedObject, body):
    lid = model.part("lid")
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        origin=Origin(
            xyz=(LID_DEPTH / 2.0, 0.0, LID_THICKNESS / 2.0)
        ),
        material="oak_case",
        name="lid_panel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                -(CABINET_DEPTH / 2.0),
                0.0,
                CABINET_BODY_HEIGHT,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.25,
            effort=20.0,
            velocity=1.5,
        ),
    )

    return lid


def _add_drawer(model: ArticulatedObject, body, index: int, center_z: float):
    drawer = model.part(f"drawer_{index}")
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)),
        origin=Origin(
            xyz=(DRAWER_FRONT_THICKNESS / 2.0, 0.0, 0.0)
        ),
        material="drawer_front",
        name="front_panel",
    )
    for side_sign, side_name in ((1.0, "box_side_0"), (-1.0, "box_side_1")):
        drawer.visual(
            Box((DRAWER_BOX_DEPTH, DRAWER_PANEL_THICKNESS, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    -(DRAWER_BOX_DEPTH / 2.0),
                    side_sign
                    * ((DRAWER_BOX_WIDTH / 2.0) - (DRAWER_PANEL_THICKNESS / 2.0)),
                    0.0,
                )
            ),
            material="drawer_box",
            name=side_name,
        )
    drawer.visual(
        Box(
            (
                DRAWER_PANEL_THICKNESS,
                DRAWER_BOX_WIDTH - (2.0 * DRAWER_PANEL_THICKNESS),
                DRAWER_BOX_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                -DRAWER_BOX_DEPTH + (DRAWER_PANEL_THICKNESS / 2.0),
                0.0,
                0.0,
            )
        ),
        material="drawer_box",
        name="box_back",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_DEPTH,
                DRAWER_BOX_WIDTH - (2.0 * DRAWER_PANEL_THICKNESS),
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                -(DRAWER_BOX_DEPTH / 2.0),
                0.0,
                -(DRAWER_BOX_HEIGHT / 2.0) + (DRAWER_BOTTOM_THICKNESS / 2.0),
            )
        ),
        material="drawer_box",
        name="box_bottom",
    )
    drawer.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(DRAWER_FRONT_THICKNESS + 0.006, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="knob_metal",
        name="knob_stem",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(
            xyz=(DRAWER_FRONT_THICKNESS + 0.019, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="knob_metal",
        name="knob_cap",
    )

    model.articulation(
        f"body_to_drawer_{index}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=((CABINET_DEPTH / 2.0), 0.0, center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=80.0,
            velocity=0.22,
        ),
    )

    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_drawer_cabinet")
    model.material("oak_case", rgba=(0.58, 0.44, 0.28, 1.0))
    model.material("drawer_front", rgba=(0.52, 0.38, 0.24, 1.0))
    model.material("drawer_box", rgba=(0.74, 0.67, 0.53, 1.0))
    model.material("knob_metal", rgba=(0.22, 0.22, 0.23, 1.0))

    body = _add_cabinet_body(model)
    _add_lid(model, body)
    for index, center_z in enumerate(DRAWER_CENTER_Z):
        _add_drawer(model, body, index=index, center_z=center_z)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    drawers = [object_model.get_part(f"drawer_{index}") for index in range(3)]
    drawer_joints = [
        object_model.get_articulation(f"body_to_drawer_{index}") for index in range(3)
    ]

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid rests on cabinet top plane",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.30,
            name="lid covers the top opening",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.1}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for index, (drawer, joint) in enumerate(zip(drawers, drawer_joints)):
        with ctx.pose({joint: 0.0}):
            ctx.expect_gap(
                drawer,
                body,
                axis="x",
                positive_elem="front_panel",
                max_gap=0.001,
                max_penetration=0.0,
                name=f"drawer_{index} front closes flush",
            )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            ctx.expect_gap(
                drawer,
                body,
                axis="x",
                positive_elem="front_panel",
                min_gap=0.20,
                name=f"drawer_{index} slides forward",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.08,
                name=f"drawer_{index} remains retained in the cabinet",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{index} origin moves outward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
