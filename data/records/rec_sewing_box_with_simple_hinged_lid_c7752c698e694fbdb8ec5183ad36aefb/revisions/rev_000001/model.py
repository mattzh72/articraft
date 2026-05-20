from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_W = 0.28
OUTER_D = 0.18
BODY_H = 0.12
BASE_T = 0.008
WALL_T = 0.010
LID_T = 0.008
PANEL_CLEAR = 0.002
LEDGE_W = 0.008
LEDGE_T = 0.004

INNER_W = OUTER_W - 2.0 * WALL_T
INNER_D = OUTER_D - 2.0 * WALL_T
WALL_H = BODY_H - BASE_T
LID_W = INNER_W - 2.0 * PANEL_CLEAR
LID_D = INNER_D - 2.0 * PANEL_CLEAR


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    outer_finish = model.material("outer_finish", rgba=(0.71, 0.60, 0.46, 1.0))
    inner_finish = model.material("inner_finish", rgba=(0.61, 0.50, 0.38, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.74, 0.63, 0.49, 1.0))

    body = model.part("body")
    body.visual(
        Box((OUTER_W, OUTER_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=inner_finish,
        name="base_panel",
    )
    body.visual(
        Box((WALL_T, OUTER_D, WALL_H)),
        origin=Origin(
            xyz=(-(OUTER_W - WALL_T) / 2.0, 0.0, BASE_T + WALL_H / 2.0)
        ),
        material=outer_finish,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, OUTER_D, WALL_H)),
        origin=Origin(
            xyz=((OUTER_W - WALL_T) / 2.0, 0.0, BASE_T + WALL_H / 2.0)
        ),
        material=outer_finish,
        name="right_wall",
    )
    body.visual(
        Box((INNER_W, WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, (OUTER_D - WALL_T) / 2.0, BASE_T + WALL_H / 2.0)
        ),
        material=outer_finish,
        name="front_wall",
    )
    body.visual(
        Box((INNER_W, WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, -(OUTER_D - WALL_T) / 2.0, BASE_T + WALL_H / 2.0)
        ),
        material=outer_finish,
        name="rear_wall",
    )
    body.visual(
        Box((LEDGE_W, INNER_D, LEDGE_T)),
        origin=Origin(
            xyz=(-(INNER_W - LEDGE_W) / 2.0, 0.0, BODY_H - LEDGE_T / 2.0)
        ),
        material=inner_finish,
        name="left_ledge",
    )
    body.visual(
        Box((LEDGE_W, INNER_D, LEDGE_T)),
        origin=Origin(
            xyz=((INNER_W - LEDGE_W) / 2.0, 0.0, BODY_H - LEDGE_T / 2.0)
        ),
        material=inner_finish,
        name="right_ledge",
    )
    body.visual(
        Box((INNER_W - 2.0 * LEDGE_W, LEDGE_W, LEDGE_T)),
        origin=Origin(
            xyz=(0.0, (INNER_D - LEDGE_W) / 2.0, BODY_H - LEDGE_T / 2.0)
        ),
        material=inner_finish,
        name="front_ledge",
    )
    body.inertial = Inertial.from_geometry(
        Box((OUTER_W, OUTER_D, BODY_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
        material=lid_finish,
        name="lid_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=0.35,
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -LID_D / 2.0, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    def span(aabb):
        if aabb is None:
            return None
        return aabb[0], aabb[1]

    rest_panel = span(ctx.part_element_world_aabb(lid, elem="lid_panel"))
    left_wall = span(ctx.part_element_world_aabb(body, elem="left_wall"))
    right_wall = span(ctx.part_element_world_aabb(body, elem="right_wall"))
    front_wall = span(ctx.part_element_world_aabb(body, elem="front_wall"))
    rear_wall = span(ctx.part_element_world_aabb(body, elem="rear_wall"))

    symmetric = False
    even_front_back = False
    if (
        rest_panel is not None
        and left_wall is not None
        and right_wall is not None
        and front_wall is not None
        and rear_wall is not None
    ):
        left_gap = rest_panel[0][0] - left_wall[1][0]
        right_gap = right_wall[0][0] - rest_panel[1][0]
        rear_gap = rest_panel[0][1] - rear_wall[1][1]
        front_gap = front_wall[0][1] - rest_panel[1][1]
        symmetric = (
            0.001 <= left_gap <= 0.004
            and 0.001 <= right_gap <= 0.004
            and abs(left_gap - right_gap) <= 0.0005
        )
        even_front_back = (
            0.001 <= rear_gap <= 0.004
            and 0.001 <= front_gap <= 0.004
            and abs(rear_gap - front_gap) <= 0.0005
        )

    ctx.check(
        "lid panel is centered between the side walls",
        symmetric,
        details=(
            f"panel={rest_panel}, left_wall={left_wall}, right_wall={right_wall}"
        ),
    )
    ctx.check(
        "lid panel is centered between the front and rear frame edges",
        even_front_back,
        details=(
            f"panel={rest_panel}, rear_wall={rear_wall}, front_wall={front_wall}"
        ),
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid seats on the box rim without sinking into it",
    )

    opened_upward = False
    with ctx.pose({hinge: 1.2}):
        opened_panel = span(ctx.part_element_world_aabb(lid, elem="lid_panel"))
        if rest_panel is not None and opened_panel is not None:
            opened_upward = opened_panel[1][2] > rest_panel[1][2] + 0.08
        ctx.check(
            "lid opens upward from the rear hinge",
            opened_upward,
            details=f"rest={rest_panel}, opened={opened_panel}",
        )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
