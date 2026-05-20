from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_service_panel")

    body_color = model.material("body_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    frame_color = model.material("frame_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    panel_color = model.material("panel_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    handle_color = model.material("handle_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body_width = 1.10
    body_depth = 0.55
    body_height = 0.46
    shell_thickness = 0.03

    opening_width = 0.86
    opening_height = 0.26
    frame_depth = 0.04
    frame_side = (body_width - 2.0 * shell_thickness - opening_width) / 2.0
    frame_top_bottom = (body_height - 2.0 * shell_thickness - opening_height) / 2.0
    opening_center_z = body_height / 2.0

    door_gap = 0.004
    door_width = opening_width - 2.0 * door_gap
    door_height = opening_height - 2.0 * door_gap
    door_thickness = 0.022
    hinge_x = -opening_width / 2.0 + door_gap
    front_y = body_depth / 2.0

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness / 2.0)),
        material=body_color,
        name="base_plate",
    )
    body.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - shell_thickness / 2.0)),
        material=body_color,
        name="top_plate",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height - 2.0 * shell_thickness)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + shell_thickness / 2.0,
                0.0,
                body_height / 2.0,
            )
        ),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height - 2.0 * shell_thickness)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - shell_thickness / 2.0,
                0.0,
                body_height / 2.0,
            )
        ),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, body_height - 2.0 * shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -body_depth / 2.0 + shell_thickness / 2.0,
                body_height / 2.0,
            )
        ),
        material=body_color,
        name="back_wall",
    )

    body.visual(
        Box((frame_side, frame_depth, body_height - 2.0 * shell_thickness)),
        origin=Origin(
            xyz=(
                -opening_width / 2.0 - frame_side / 2.0,
                front_y - frame_depth / 2.0,
                body_height / 2.0,
            )
        ),
        material=frame_color,
        name="frame_left",
    )
    body.visual(
        Box((frame_side, frame_depth, body_height - 2.0 * shell_thickness)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + frame_side / 2.0,
                front_y - frame_depth / 2.0,
                body_height / 2.0,
            )
        ),
        material=frame_color,
        name="frame_right",
    )
    body.visual(
        Box((opening_width + 2.0 * frame_side, frame_depth, frame_top_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - frame_depth / 2.0,
                opening_center_z + opening_height / 2.0 + frame_top_bottom / 2.0,
            )
        ),
        material=frame_color,
        name="frame_top",
    )
    body.visual(
        Box((opening_width + 2.0 * frame_side, frame_depth, frame_top_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - frame_depth / 2.0,
                opening_center_z - opening_height / 2.0 - frame_top_bottom / 2.0,
            )
        ),
        material=frame_color,
        name="frame_bottom",
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, -door_thickness / 2.0, 0.0)),
        material=panel_color,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.016, 0.12)),
        origin=Origin(xyz=(door_width - 0.035, 0.008, 0.0)),
        material=handle_color,
        name="latch_pull",
    )
    door.visual(
        Box((0.028, 0.006, 0.16)),
        origin=Origin(xyz=(door_width - 0.008, 0.003, 0.0)),
        material=handle_color,
        name="latch_pad",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, front_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="frame_left",
        min_gap=0.003,
        max_gap=0.006,
        name="hinge side panel clearance",
    )
    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem="frame_right",
        negative_elem="door_panel",
        min_gap=0.003,
        max_gap=0.006,
        name="latch side panel clearance",
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        positive_elem="frame_top",
        negative_elem="door_panel",
        min_gap=0.003,
        max_gap=0.006,
        name="top panel clearance",
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        positive_elem="door_panel",
        negative_elem="frame_bottom",
        min_gap=0.003,
        max_gap=0.006,
        name="bottom panel clearance",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.35}):
        opened_panel = ctx.part_element_world_aabb(door, elem="door_panel")

    opens_outward = (
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][1] > closed_panel[1][1] + 0.18
    )
    ctx.check(
        "door swings outward from the equipment face",
        opens_outward,
        details=f"closed_panel={closed_panel}, opened_panel={opened_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
