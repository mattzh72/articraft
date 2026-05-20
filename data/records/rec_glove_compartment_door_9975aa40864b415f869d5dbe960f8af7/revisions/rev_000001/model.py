from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="passenger_car_glove_compartment")

    dash_mat = Material("soft_black_dashboard", rgba=(0.025, 0.026, 0.028, 1.0))
    bin_mat = Material("dark_gray_bin_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    door_mat = Material("grained_charcoal_door", rgba=(0.075, 0.077, 0.080, 1.0))
    hinge_mat = Material("black_oxide_hinge_pin", rgba=(0.015, 0.015, 0.016, 1.0))
    arm_mat = Material("matte_black_stay_arms", rgba=(0.05, 0.052, 0.055, 1.0))
    button_mat = Material("satin_latch_button", rgba=(0.19, 0.19, 0.18, 1.0))

    bin_part = model.part("bin")

    # Fixed dashboard surround and hollow storage pocket.  The bin is built from
    # connected walls, leaving the front opening visibly hollow.
    bin_part.visual(
        Box((0.34, 0.58, 0.025)),
        origin=Origin(xyz=(0.17, 0.0, 0.040)),
        material=bin_mat,
        name="floor_wall",
    )
    bin_part.visual(
        Box((0.34, 0.58, 0.025)),
        origin=Origin(xyz=(0.17, 0.0, 0.360)),
        material=bin_mat,
        name="top_wall",
    )
    bin_part.visual(
        Box((0.34, 0.025, 0.345)),
        origin=Origin(xyz=(0.17, 0.2875, 0.200)),
        material=bin_mat,
        name="side_wall_0",
    )
    bin_part.visual(
        Box((0.34, 0.025, 0.345)),
        origin=Origin(xyz=(0.17, -0.2875, 0.200)),
        material=bin_mat,
        name="side_wall_1",
    )
    bin_part.visual(
        Box((0.025, 0.58, 0.345)),
        origin=Origin(xyz=(0.3275, 0.0, 0.200)),
        material=bin_mat,
        name="back_wall",
    )

    # Dashboard face frame around the opening.
    bin_part.visual(
        Box((0.045, 0.68, 0.060)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.405)),
        material=dash_mat,
        name="upper_bezel",
    )
    bin_part.visual(
        Box((0.045, 0.68, 0.060)),
        origin=Origin(xyz=(-0.0125, 0.0, -0.020)),
        material=dash_mat,
        name="lower_bezel",
    )
    bin_part.visual(
        Box((0.045, 0.070, 0.425)),
        origin=Origin(xyz=(-0.0125, 0.325, 0.1925)),
        material=dash_mat,
        name="side_bezel_0",
    )
    bin_part.visual(
        Box((0.045, 0.070, 0.425)),
        origin=Origin(xyz=(-0.0125, -0.325, 0.1925)),
        material=dash_mat,
        name="side_bezel_1",
    )

    # Lower hinge hardware and the side stay support pivots are fixed to the bin.
    bin_part.visual(
        Cylinder(radius=0.006, length=0.60),
        origin=Origin(xyz=(-0.015, 0.0, 0.065), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )
    for idx, y in enumerate((0.285, -0.285)):
        bin_part.visual(
            Box((0.030, 0.030, 0.050)),
            origin=Origin(xyz=(-0.015, y, 0.037)),
            material=dash_mat,
            name=f"hinge_bracket_{idx}",
        )

    for idx, y_sign in enumerate((1.0, -1.0)):
        pin_y = y_sign * 0.264
        bin_part.visual(
            Cylinder(radius=0.010, length=0.032),
            origin=Origin(xyz=(0.120, pin_y, 0.220), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"wall_pivot_{idx}",
        )
        bin_part.visual(
            Box((0.050, 0.020, 0.045)),
            origin=Origin(xyz=(0.120, y_sign * 0.276, 0.220)),
            material=bin_mat,
            name=f"pivot_boss_{idx}",
        )

    door = model.part("door")

    # Door skin with a real rectangular latch-button opening rather than a
    # solid face.  Overlapping edges keep the four skin regions a single panel.
    door.visual(
        Box((0.035, 0.510, 0.160)),
        origin=Origin(xyz=(-0.018, 0.0, 0.085)),
        material=door_mat,
        name="lower_skin",
    )
    door.visual(
        Box((0.035, 0.510, 0.082)),
        origin=Origin(xyz=(-0.018, 0.0, 0.251)),
        material=door_mat,
        name="upper_skin",
    )
    door.visual(
        Box((0.035, 0.190, 0.053)),
        origin=Origin(xyz=(-0.018, 0.158, 0.191)),
        material=door_mat,
        name="side_skin_0",
    )
    door.visual(
        Box((0.035, 0.190, 0.053)),
        origin=Origin(xyz=(-0.018, -0.158, 0.191)),
        material=door_mat,
        name="side_skin_1",
    )
    door.visual(
        Box((0.020, 0.465, 0.030)),
        origin=Origin(xyz=(-0.001, 0.0, 0.020)),
        material=door_mat,
        name="inner_lower_rib",
    )
    door.visual(
        Box((0.018, 0.038, 0.180)),
        origin=Origin(xyz=(-0.002, 0.205, 0.150)),
        material=door_mat,
        name="inner_side_rib_0",
    )
    door.visual(
        Box((0.018, 0.038, 0.180)),
        origin=Origin(xyz=(-0.002, -0.205, 0.150)),
        material=door_mat,
        name="inner_side_rib_1",
    )

    # Latch housing is a small open sleeve behind the button slot.
    door.visual(
        Box((0.036, 0.028, 0.078)),
        origin=Origin(xyz=(0.016, 0.069, 0.191)),
        material=door_mat,
        name="latch_cheek_0",
    )
    door.visual(
        Box((0.036, 0.028, 0.078)),
        origin=Origin(xyz=(0.016, -0.069, 0.191)),
        material=door_mat,
        name="latch_cheek_1",
    )
    door.visual(
        Box((0.036, 0.112, 0.015)),
        origin=Origin(xyz=(0.016, 0.0, 0.230)),
        material=door_mat,
        name="latch_bridge",
    )

    # Door hinge knuckles grip the fixed lower hinge pin.
    for idx, y in enumerate((-0.175, 0.0, 0.175)):
        door.visual(
            Cylinder(radius=0.012, length=0.095),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=door_mat,
            name=f"hinge_barrel_{idx}",
        )

    # Door-side rollers sit in the open slots of the two side stay arms.
    for idx, y in enumerate((0.247, -0.247)):
        door.visual(
            Cylinder(radius=0.007, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.155), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"stay_roller_{idx}",
        )
        door.visual(
            Box((0.016, 0.028, 0.035)),
            origin=Origin(xyz=(-0.008, y * 0.965, 0.155)),
            material=door_mat,
            name=f"roller_boss_{idx}",
        )

    button = model.part("button")
    button.visual(
        Box((0.014, 0.126, 0.034)),
        origin=Origin(),
        material=button_mat,
        name="button_cap",
    )
    button.visual(
        Box((0.050, 0.034, 0.020)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=button_mat,
        name="button_stem",
    )

    for idx, y in enumerate((0.255, -0.255)):
        arm = model.part(f"stay_arm_{idx}")
        # Open slotted link: two rails with a clear center channel for the door
        # roller, plus a pivot eye around the wall support pin.
        arm.visual(
            Box((0.300, 0.006, 0.005)),
            origin=Origin(xyz=(-0.150, 0.0, 0.013)),
            material=arm_mat,
            name="upper_rail",
        )
        arm.visual(
            Box((0.300, 0.006, 0.005)),
            origin=Origin(xyz=(-0.150, 0.0, -0.013)),
            material=arm_mat,
            name="lower_rail",
        )
        arm.visual(
            Box((0.010, 0.006, 0.031)),
            origin=Origin(xyz=(-0.295, 0.0, 0.0)),
            material=arm_mat,
            name="end_bridge",
        )
        arm.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=arm_mat,
            name="pivot_eye",
        )

        model.articulation(
            f"bin_to_stay_{idx}",
            ArticulationType.REVOLUTE,
            parent=bin_part,
            child=arm,
            origin=Origin(xyz=(0.120, y, 0.220)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.35, upper=0.05),
        )

    model.articulation(
        "bin_to_door",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=door,
        origin=Origin(xyz=(-0.015, 0.0, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.95),
    )

    model.articulation(
        "door_to_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=button,
        origin=Origin(xyz=(-0.0425, 0.0, 0.191)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bin_part = object_model.get_part("bin")
    door = object_model.get_part("door")
    button = object_model.get_part("button")
    stay_0 = object_model.get_part("stay_arm_0")
    stay_1 = object_model.get_part("stay_arm_1")
    door_hinge = object_model.get_articulation("bin_to_door")
    button_slide = object_model.get_articulation("door_to_button")
    stay_hinge_0 = object_model.get_articulation("bin_to_stay_0")
    stay_hinge_1 = object_model.get_articulation("bin_to_stay_1")

    # Hinge barrels and pivot eyes intentionally surround their support pins.
    for barrel in ("hinge_barrel_0", "hinge_barrel_1", "hinge_barrel_2"):
        ctx.allow_overlap(
            "bin",
            "door",
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The lower hinge pin is intentionally captured inside the door hinge barrel.",
        )
        ctx.expect_overlap(
            bin_part,
            door,
            axes="y",
            elem_a="hinge_pin",
            elem_b=barrel,
            min_overlap=0.080,
            name=f"{barrel} captures the lower hinge pin",
        )

    for idx, stay in enumerate((stay_0, stay_1)):
        ctx.allow_overlap(
            "bin",
            stay.name,
            elem_a=f"wall_pivot_{idx}",
            elem_b="pivot_eye",
            reason="The wall support pin is intentionally captured by the rotating stay-arm eye.",
        )
        ctx.expect_overlap(
            bin_part,
            stay,
            axes="y",
            elem_a=f"wall_pivot_{idx}",
            elem_b="pivot_eye",
            min_overlap=0.010,
            name=f"stay arm {idx} is retained on its wall pivot",
        )

    # Door is clipped to the lower hinge, sits in the dashboard opening, and the
    # latch button sits in a clear opening at rest.
    ctx.expect_overlap(
        door,
        bin_part,
        axes="y",
        elem_a="lower_skin",
        elem_b="floor_wall",
        min_overlap=0.45,
        name="door spans the glove-box opening width",
    )
    ctx.expect_gap(
        bin_part,
        door,
        axis="x",
        positive_elem="floor_wall",
        negative_elem="lower_skin",
        min_gap=None,
        max_penetration=0.0,
        name="closed door remains in front of the storage bin",
    )
    cap_box = ctx.part_element_world_aabb(button, elem="button_cap")
    skin_box = ctx.part_element_world_aabb(door, elem="side_skin_0")
    ctx.check(
        "button cap starts proud of the front skin",
        cap_box is not None
        and skin_box is not None
        and cap_box[0][0] < skin_box[0][0] - 0.010
        and cap_box[1][0] <= skin_box[0][0] + 0.001,
        details=f"button_cap={cap_box}, side_skin={skin_box}",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 0.90, stay_hinge_0: -0.23, stay_hinge_1: -0.23}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops outward and downward on the lower hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][0] < closed_aabb[0][0] - 0.10
            and open_aabb[1][2] < closed_aabb[1][2] - 0.03,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        for idx, stay in enumerate((stay_0, stay_1)):
            roller_box = ctx.part_element_world_aabb(door, elem=f"stay_roller_{idx}")
            slot_box = ctx.part_world_aabb(stay)
            ctx.check(
                f"door roller {idx} stays in the slotted side stay envelope",
                roller_box is not None
                and slot_box is not None
                and slot_box[0][0] - 0.010 <= roller_box[0][0] <= slot_box[1][0] + 0.010
                and slot_box[0][2] - 0.025 <= roller_box[0][2] <= slot_box[1][2] + 0.025,
                details=f"roller={roller_box}, slot={slot_box}",
            )

    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.012}):
        pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            "push button translates inward into the latch housing",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[0] > rest_button_pos[0] + 0.010,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
