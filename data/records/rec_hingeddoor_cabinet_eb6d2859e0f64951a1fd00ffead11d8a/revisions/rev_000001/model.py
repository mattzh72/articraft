from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_base_cabinet")

    painted = model.material("painted_cabinet", rgba=(0.90, 0.90, 0.87, 1.0))
    hamper_finish = model.material("hamper_finish", rgba=(0.84, 0.85, 0.83, 1.0))
    metal = model.material("handle_metal", rgba=(0.68, 0.69, 0.72, 1.0))

    width = 0.76
    depth = 0.58
    height = 0.86
    side_t = 0.018
    back_t = 0.006
    panel_t = 0.018
    toe_h = 0.10
    toe_recess = 0.075
    toe_skin_t = 0.015
    door_w = 0.728
    door_h = 0.744
    door_t = 0.022
    door_bottom = toe_h + 0.006
    door_handle_w = 0.020
    door_handle_d = 0.030
    door_handle_h = 0.180
    hamper_w = 0.658
    hamper_front_t = 0.018
    hamper_front_h = 0.560
    hamper_side_t = 0.012
    hamper_depth = 0.080
    hamper_side_h = 0.400
    hamper_bottom_t = 0.020
    hamper_back_t = 0.016
    hamper_back_h = 0.090

    body = model.part("cabinet_body")
    upper_side_h = height - toe_h
    lower_side_d = depth - toe_recess

    body.visual(
        Box((side_t, depth, upper_side_h)),
        origin=Origin(xyz=(-width / 2 + side_t / 2, 0.0, toe_h + upper_side_h / 2)),
        material=painted,
        name="left_side",
    )
    body.visual(
        Box((side_t, depth, upper_side_h)),
        origin=Origin(xyz=(width / 2 - side_t / 2, 0.0, toe_h + upper_side_h / 2)),
        material=painted,
        name="right_side",
    )
    body.visual(
        Box((side_t, lower_side_d, toe_h)),
        origin=Origin(
            xyz=(-width / 2 + side_t / 2, -toe_recess / 2, toe_h / 2),
        ),
        material=painted,
        name="left_kick_return",
    )
    body.visual(
        Box((side_t, lower_side_d, toe_h)),
        origin=Origin(
            xyz=(width / 2 - side_t / 2, -toe_recess / 2, toe_h / 2),
        ),
        material=painted,
        name="right_kick_return",
    )
    body.visual(
        Box((width - 2 * side_t, depth - back_t, panel_t)),
        origin=Origin(xyz=(0.0, back_t / 2, toe_h + panel_t / 2)),
        material=painted,
        name="bottom_panel",
    )
    body.visual(
        Box((width - 2 * side_t, depth - back_t, panel_t)),
        origin=Origin(xyz=(0.0, back_t / 2, height - panel_t / 2)),
        material=painted,
        name="top_panel",
    )
    body.visual(
        Box((width - 2 * side_t, back_t, height)),
        origin=Origin(xyz=(0.0, -depth / 2 + back_t / 2, height / 2)),
        material=painted,
        name="back_panel",
    )
    body.visual(
        Box((width - 2 * side_t, toe_skin_t, toe_h)),
        origin=Origin(
            xyz=(0.0, depth / 2 - toe_recess - toe_skin_t / 2, toe_h / 2),
        ),
        material=painted,
        name="toe_kick_panel",
    )
    body.visual(
        Box((0.010, 0.037, door_h)),
        origin=Origin(
            xyz=(-door_w / 2 - 0.005, depth / 2 + 0.0185, door_bottom + door_h / 2),
        ),
        material=metal,
        name="door_hinge_stile",
    )
    body.visual(
        Box((hamper_w - 2 * hamper_side_t, hamper_depth, 0.027)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2 - 0.035 - (hamper_front_t / 2 + hamper_depth / 2),
                toe_h + panel_t + 0.0135,
            ),
        ),
        material=painted,
        name="hamper_hinge_support",
    )

    door = model.part("main_door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2, 0.0, door_h / 2)),
        material=painted,
        name="door_shell",
    )
    door.visual(
        Box((door_handle_w, door_handle_d, door_handle_h)),
        origin=Origin(
            xyz=(door_w - 0.056, door_t / 2 + door_handle_d / 2, door_h * 0.54),
        ),
        material=metal,
        name="door_pull",
    )
    door.visual(
        Box((0.010, 0.012, door_h)),
        origin=Origin(xyz=(0.005, door_t / 2 + 0.006, door_h / 2)),
        material=metal,
        name="door_hinge_leaf",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(-door_w / 2, depth / 2 + 0.003 + door_t / 2, door_bottom),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )

    hamper = model.part("inner_hamper_panel")
    hamper.visual(
        Box((hamper_w, hamper_front_t, hamper_front_h)),
        origin=Origin(xyz=(0.0, 0.0, hamper_front_h / 2)),
        material=hamper_finish,
        name="hamper_front",
    )
    hamper.visual(
        Box((hamper_side_t, hamper_depth, hamper_side_h)),
        origin=Origin(
            xyz=(
                -(hamper_w - hamper_side_t) / 2,
                -(hamper_front_t / 2 + hamper_depth / 2),
                hamper_side_h / 2,
            ),
        ),
        material=hamper_finish,
        name="hamper_left_flange",
    )
    hamper.visual(
        Box((hamper_side_t, hamper_depth, hamper_side_h)),
        origin=Origin(
            xyz=(
                (hamper_w - hamper_side_t) / 2,
                -(hamper_front_t / 2 + hamper_depth / 2),
                hamper_side_h / 2,
            ),
        ),
        material=hamper_finish,
        name="hamper_right_flange",
    )
    hamper.visual(
        Box((hamper_w - 2 * hamper_side_t, hamper_depth, hamper_bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(hamper_front_t / 2 + hamper_depth / 2),
                hamper_bottom_t / 2,
            ),
        ),
        material=hamper_finish,
        name="hamper_bottom_rail",
    )
    hamper.visual(
        Box((hamper_w - 2 * hamper_side_t, hamper_back_t, hamper_back_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(hamper_front_t / 2 + hamper_depth - hamper_back_t / 2),
                hamper_side_h - hamper_back_h / 2,
            ),
        ),
        material=hamper_finish,
        name="hamper_back_rail",
    )

    model.articulation(
        "body_to_hamper",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hamper,
        origin=Origin(xyz=(0.0, depth / 2 - 0.035, toe_h + 0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=0.95,
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

    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("main_door")
    hamper = object_model.get_part("inner_hamper_panel")
    door_joint = object_model.get_articulation("body_to_door")
    hamper_joint = object_model.get_articulation("body_to_hamper")

    ctx.check(
        "primary parts are present",
        all(part is not None for part in (body, door, hamper)),
        details="cabinet_body, main_door, and inner_hamper_panel should all exist",
    )
    ctx.check(
        "door hinge is vertical",
        tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"door axis={door_joint.axis}",
    )
    ctx.check(
        "hamper hinge is lower horizontal",
        tuple(hamper_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"hamper axis={hamper_joint.axis}",
    )

    with ctx.pose({door_joint: 0.0, hamper_joint: 0.0}):
        ctx.expect_gap(
            door,
            hamper,
            axis="y",
            positive_elem="door_shell",
            negative_elem="hamper_front",
            min_gap=0.015,
            max_gap=0.045,
            name="closed door clears inner hamper front",
        )
        ctx.expect_within(
            hamper,
            body,
            axes="x",
            inner_elem="hamper_front",
            margin=0.0,
            name="closed hamper stays between cabinet sides",
        )
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
        closed_hamper_aabb = ctx.part_element_world_aabb(hamper, elem="hamper_front")

    with ctx.pose({door_joint: 1.35, hamper_joint: 0.0}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_shell")

    with ctx.pose({door_joint: 1.35, hamper_joint: 0.85}):
        tilted_hamper_aabb = ctx.part_element_world_aabb(hamper, elem="hamper_front")

    ctx.check(
        "door swings outward from the cabinet front",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "hamper panel tilts outward when opened",
        closed_hamper_aabb is not None
        and tilted_hamper_aabb is not None
        and tilted_hamper_aabb[1][1] > closed_hamper_aabb[1][1] + 0.18,
        details=f"closed={closed_hamper_aabb}, tilted={tilted_hamper_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
