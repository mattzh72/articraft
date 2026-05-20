from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_size_office_keyboard")

    case_mat = Material("warm_black_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    deck_mat = Material("slate_recessed_deck", rgba=(0.055, 0.060, 0.065, 1.0))
    key_side_mat = Material("black_key_sides", rgba=(0.015, 0.016, 0.017, 1.0))
    key_top_mat = Material("charcoal_key_tops", rgba=(0.14, 0.145, 0.15, 1.0))
    hinge_mat = Material("dark_hinge_plastic", rgba=(0.018, 0.019, 0.021, 1.0))
    foot_mat = Material("rubberized_feet", rgba=(0.030, 0.030, 0.028, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.460, 0.175, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=case_mat,
        name="case_shell",
    )
    housing.visual(
        Box((0.438, 0.140, 0.004)),
        origin=Origin(xyz=(-0.003, 0.004, 0.022)),
        material=deck_mat,
        name="top_plate",
    )
    housing.visual(
        Box((0.455, 0.018, 0.009)),
        origin=Origin(xyz=(0.0, 0.077, 0.0245)),
        material=case_mat,
        name="rear_raised_lip",
    )
    housing.visual(
        Box((0.455, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.079, 0.022)),
        material=case_mat,
        name="front_low_lip",
    )

    # Narrow stepped shelves under each key row; they make the top plate read as
    # a real office-keyboard stair step rather than a flat array of blocks.
    row_surfaces = {
        0: 0.0310,  # rear function row
        1: 0.0295,
        2: 0.0280,
        3: 0.0268,
        4: 0.0258,
        5: 0.0250,  # front space-bar row
    }
    row_y = {
        0: 0.060,
        1: 0.037,
        2: 0.014,
        3: -0.009,
        4: -0.032,
        5: -0.055,
    }
    for row, y in row_y.items():
        top_z = row_surfaces[row]
        thickness = top_z - 0.024
        housing.visual(
            Box((0.325, 0.020, thickness)),
            origin=Origin(xyz=(-0.070, y, 0.024 + thickness / 2.0)),
            material=deck_mat,
            name="main_row_shelf_3" if row == 3 else f"main_row_shelf_{row}",
        )
        if row >= 1:
            housing.visual(
                Box((0.086, 0.020, thickness)),
                origin=Origin(xyz=(0.178, y, 0.024 + thickness / 2.0)),
                material=deck_mat,
                name="num_row_shelf_3" if row == 3 else f"num_row_shelf_{row}",
            )

    # Underside recess frames and hinge cheeks for the two rear feet.  These are
    # fixed to the housing while the feet rotate between them.
    for idx, x in enumerate((-0.155, 0.155)):
        housing.visual(
            Box((0.086, 0.006, 0.002)),
            origin=Origin(xyz=(x, 0.032, -0.001)),
            material=deck_mat,
            name=f"foot_recess_front_{idx}",
        )
        housing.visual(
            Box((0.086, 0.006, 0.002)),
            origin=Origin(xyz=(x, 0.082, -0.001)),
            material=deck_mat,
            name=f"foot_recess_rear_{idx}",
        )
        housing.visual(
            Box((0.006, 0.050, 0.002)),
            origin=Origin(xyz=(x - 0.043, 0.057, -0.001)),
            material=deck_mat,
            name=f"foot_recess_side_{idx}_0",
        )
        housing.visual(
            Box((0.006, 0.050, 0.002)),
            origin=Origin(xyz=(x + 0.043, 0.057, -0.001)),
            material=deck_mat,
            name=f"foot_recess_side_{idx}_1",
        )
        for side, sx in enumerate((-0.039, 0.039)):
            housing.visual(
                Box((0.006, 0.012, 0.008)),
                origin=Origin(xyz=(x + sx, 0.076, -0.004)),
                material=hinge_mat,
                name=f"foot_hinge_cheek_{idx}_{side}",
            )

    key_pitch = 0.01905
    key_gap = 0.0022
    key_depth = 0.0166
    func_depth = 0.0138
    key_travel = 0.0042

    def add_key(
        name: str,
        x: float,
        y: float,
        width: float,
        depth: float,
        surface_z: float,
        *,
        cap_height: float = 0.0088,
    ) -> None:
        key = model.part(name)
        lower_h = cap_height * 0.55
        upper_h = cap_height * 0.52
        lower_w = max(width, 0.006)
        lower_d = max(depth, 0.006)
        upper_w = max(width - 0.0030, 0.004)
        upper_d = max(depth - 0.0030, 0.004)

        # The child frame lies on the local key well; geometry starts just above
        # it so the key has a visible operating gap before it plunges down.
        key.visual(
            Box((lower_w, lower_d, lower_h)),
            origin=Origin(xyz=(0.0, 0.0, 0.0010 + lower_h / 2.0)),
            material=key_side_mat,
            name="key_skirt",
        )
        key.visual(
            Box((upper_w, upper_d, upper_h)),
            origin=Origin(xyz=(0.0, 0.0, 0.0010 + lower_h * 0.74 + upper_h / 2.0)),
            material=key_top_mat,
            name="key_cap",
        )
        key.visual(
            Box((min(width * 0.36, 0.010), min(depth * 0.36, 0.010), 0.0030)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=hinge_mat,
            name="visible_plunger",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key,
            origin=Origin(xyz=(x, y, surface_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.35, lower=0.0, upper=key_travel),
        )

    def row_keys(prefix: str, row: int, unit_widths: list[float], x_start: float, y: float, depth: float) -> None:
        cursor = x_start
        for col, units in enumerate(unit_widths):
            width = units * key_pitch - key_gap
            x = cursor + units * key_pitch / 2.0
            add_key(f"{prefix}_{row}_{col}", x, y, width, depth, row_surfaces[row])
            cursor += units * key_pitch

    main_x = -0.215
    row_keys("key", 0, [1.0] * 15, main_x, row_y[0], func_depth)
    row_keys("key", 1, [1.0] * 15, main_x, row_y[1], key_depth)
    row_keys("key", 2, [1.50] + [1.0] * 12 + [1.50], main_x, row_y[2], key_depth)
    row_keys("key", 3, [1.75] + [1.0] * 11 + [2.25], main_x, row_y[3], key_depth)
    row_keys("key", 4, [2.25] + [1.0] * 10 + [2.75], main_x, row_y[4], key_depth)
    row_keys("key", 5, [1.25, 1.25, 1.25, 5.25, 1.25, 1.25, 1.25, 1.25, 1.25], main_x, row_y[5], key_depth)

    num_x = 0.140
    for nrow, row in enumerate((1, 2, 3, 4, 5)):
        y = row_y[row]
        for col in range(4):
            # The keypad zero key is wide on the front row; this is the most
            # recognizable numeric-keypad cue on a compact model.
            if row == 5 and col == 0:
                width_units = 2.0
                x = num_x + width_units * key_pitch / 2.0
                add_key("numkey_4_0", x, y, width_units * key_pitch - key_gap, key_depth, row_surfaces[row])
            elif row == 5 and col == 1:
                continue
            else:
                x = num_x + col * key_pitch + key_pitch / 2.0
                add_key(f"numkey_{nrow}_{col}", x, y, key_pitch - key_gap, key_depth, row_surfaces[row])

    for idx, x in enumerate((-0.155, 0.155)):
        foot = model.part(f"rear_foot_{idx}")
        foot.visual(
            Cylinder(radius=0.0030, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, -0.0044), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.066, 0.044, 0.005)),
            origin=Origin(xyz=(0.0, -0.022, -0.0040)),
            material=foot_mat,
            name="folding_pad",
        )
        foot.visual(
            Box((0.052, 0.010, 0.002)),
            origin=Origin(xyz=(0.0, -0.040, -0.0075)),
            material=case_mat,
            name="rubber_tip",
        )
        model.articulation(
            f"housing_to_rear_foot_{idx}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=foot,
            origin=Origin(xyz=(x, 0.076, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    sample_key = object_model.get_part("key_3_5")
    sample_key_joint = object_model.get_articulation("housing_to_key_3_5")
    sample_num = object_model.get_part("numkey_2_1")
    sample_num_joint = object_model.get_articulation("housing_to_numkey_2_1")
    foot = object_model.get_part("rear_foot_0")
    foot_joint = object_model.get_articulation("housing_to_rear_foot_0")

    ctx.check(
        "broad office-keyboard part count",
        len(object_model.parts) >= 90,
        details=f"expected articulated main block, numpad, and feet; got {len(object_model.parts)} parts",
    )
    ctx.expect_gap(
        sample_key,
        housing,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="key_skirt",
        negative_elem="main_row_shelf_3",
        name="typing key floats just above its row shelf",
    )
    ctx.expect_gap(
        sample_num,
        housing,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="key_skirt",
        negative_elem="num_row_shelf_3",
        name="numeric keypad key sits above a stepped shelf",
    )
    ctx.expect_gap(
        housing,
        foot,
        axis="z",
        min_gap=0.0005,
        max_gap=0.003,
        positive_elem="case_shell",
        negative_elem="folding_pad",
        name="folded rear foot is recessed just below case",
    )

    rest_key_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: 0.0042, sample_num_joint: 0.0042}):
        pressed_key_pos = ctx.part_world_position(sample_key)
        pressed_num_pos = ctx.part_world_position(sample_num)
    ctx.check(
        "sample typing key plunges downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.0035,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )
    ctx.check(
        "sample numeric key plunges downward",
        pressed_num_pos is not None
        and pressed_num_pos[2] < object_model.get_articulation("housing_to_numkey_2_1").origin.xyz[2] - 0.0035,
        details=f"pressed={pressed_num_pos}",
    )

    folded_aabb = ctx.part_element_world_aabb(foot, elem="rubber_tip")
    with ctx.pose({foot_joint: 1.15}):
        deployed_aabb = ctx.part_element_world_aabb(foot, elem="rubber_tip")
    ctx.check(
        "rear foot rotates downward when deployed",
        folded_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < folded_aabb[0][2] - 0.020,
        details=f"folded={folded_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
