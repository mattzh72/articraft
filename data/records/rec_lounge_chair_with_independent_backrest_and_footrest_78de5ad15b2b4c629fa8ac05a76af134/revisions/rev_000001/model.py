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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recliner_lounge_chair")

    frame_finish = model.material("frame_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    frame_pad = model.material("frame_pad", rgba=(0.11, 0.11, 0.12, 1.0))
    fabric = model.material("fabric", rgba=(0.74, 0.70, 0.63, 1.0))
    fabric_dark = model.material("fabric_dark", rgba=(0.58, 0.53, 0.46, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.38, 0.39, 0.41, 1.0))

    seat_width = 0.64
    outer_width = 0.74
    seat_depth = 0.54
    seat_front_y = 0.27
    seat_rear_y = -0.27
    hinge_z = 0.39

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, 0.88, 0.66)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(0.35, -0.01, 0.27)),
        material=frame_finish,
        name="left_lower_rail",
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(-0.35, -0.01, 0.27)),
        material=frame_finish,
        name="right_lower_rail",
    )
    frame.visual(
        Box((0.04, 0.06, 0.60)),
        origin=Origin(xyz=(0.35, 0.34, 0.30)),
        material=frame_finish,
        name="left_front_leg",
    )
    frame.visual(
        Box((0.04, 0.06, 0.60)),
        origin=Origin(xyz=(-0.35, 0.34, 0.30)),
        material=frame_finish,
        name="right_front_leg",
    )
    frame.visual(
        Box((0.04, 0.06, 0.68)),
        origin=Origin(xyz=(0.35, -0.36, 0.34)),
        material=frame_finish,
        name="left_rear_leg",
    )
    frame.visual(
        Box((0.04, 0.06, 0.68)),
        origin=Origin(xyz=(-0.35, -0.36, 0.34)),
        material=frame_finish,
        name="right_rear_leg",
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(0.35, -0.01, 0.58)),
        material=frame_finish,
        name="left_arm_rail",
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(-0.35, -0.01, 0.58)),
        material=frame_finish,
        name="right_arm_rail",
    )
    frame.visual(
        Box((0.70, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.30, 0.27)),
        material=frame_finish,
        name="front_stretcher",
    )
    frame.visual(
        Box((0.70, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.34, 0.27)),
        material=frame_finish,
        name="rear_stretcher",
    )
    frame.visual(
        Box((0.66, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.02, 0.29)),
        material=frame_finish,
        name="seat_support_crossbar",
    )
    frame.visual(
        Box((0.68, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.30, 0.02)),
        material=frame_pad,
        name="front_feet_bar",
    )
    frame.visual(
        Box((0.68, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.34, 0.02)),
        material=frame_pad,
        name="rear_feet_bar",
    )

    seat_base = model.part("seat_base")
    seat_base.inertial = Inertial.from_geometry(
        Box((seat_width, seat_depth, 0.12)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )
    seat_base.visual(
        Box((seat_width, seat_depth, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=frame_finish,
        name="seat_deck",
    )
    seat_base.visual(
        Box((seat_width - 0.04, 0.48, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=fabric,
        name="seat_cushion",
    )
    for side, x in (("left", 0.28), ("right", -0.28)):
        seat_base.visual(
            Box((0.02, seat_depth, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.06)),
            material=frame_finish,
            name=f"{side}_seat_side_rail",
        )
        seat_base.visual(
            Box((0.02, 0.03, 0.14)),
            origin=Origin(xyz=(x, seat_rear_y + 0.015, 0.09)),
            material=hinge_finish,
            name=f"{side}_backrest_clevis",
        )
        seat_base.visual(
            Box((0.02, 0.03, 0.14)),
            origin=Origin(xyz=(x, seat_front_y - 0.015, 0.09)),
            material=hinge_finish,
            name=f"{side}_footrest_clevis",
        )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((seat_width, 0.16, 0.74)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.06, 0.37)),
    )
    backrest.visual(
        Box((seat_width - 0.16, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, -0.005, 0.08)),
        material=hinge_finish,
        name="backrest_lower_beam",
    )
    backrest.visual(
        Box((seat_width - 0.04, 0.03, 0.66)),
        origin=Origin(xyz=(0.0, -0.03, 0.33)),
        material=frame_finish,
        name="backrest_panel",
    )
    backrest.visual(
        Box((seat_width - 0.08, 0.08, 0.58)),
        origin=Origin(xyz=(0.0, -0.05, 0.35)),
        material=fabric_dark,
        name="backrest_cushion",
    )
    for side, x in (("left", 0.30), ("right", -0.30)):
        backrest.visual(
            Box((0.02, 0.03, 0.10)),
            origin=Origin(xyz=(x, -0.015, 0.03)),
            material=hinge_finish,
            name=f"{side}_backrest_hinge_lug",
        )

    footrest = model.part("footrest")
    footrest.inertial = Inertial.from_geometry(
        Box((seat_width, 0.44, 0.10)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.22, 0.04)),
    )
    footrest.visual(
        Box((seat_width - 0.16, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, 0.015, 0.04)),
        material=hinge_finish,
        name="footrest_hinge_beam",
    )
    footrest.visual(
        Box((seat_width - 0.04, 0.42, 0.025)),
        origin=Origin(xyz=(0.0, 0.21, -0.0125)),
        material=frame_finish,
        name="footrest_panel",
    )
    footrest.visual(
        Box((seat_width - 0.08, 0.34, 0.07)),
        origin=Origin(xyz=(0.0, 0.17, 0.035)),
        material=fabric,
        name="footrest_pad",
    )
    for side, x in (("left", 0.30), ("right", -0.30)):
        footrest.visual(
            Box((0.02, 0.03, 0.10)),
            origin=Origin(xyz=(x, -0.015, 0.03)),
            material=hinge_finish,
            name=f"{side}_footrest_hinge_lug",
        )

    model.articulation(
        "frame_to_seat",
        ArticulationType.FIXED,
        parent=frame,
        child=seat_base,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=backrest,
        origin=Origin(xyz=(0.0, seat_rear_y, hinge_z - 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=footrest,
        origin=Origin(xyz=(0.0, seat_front_y, hinge_z - 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.95, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_base = object_model.get_part("seat_base")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    foot_hinge = object_model.get_articulation("seat_to_footrest")

    ctx.check("frame exists", frame is not None)
    ctx.check("seat base exists", seat_base is not None)
    ctx.check("backrest exists", backrest is not None)
    ctx.check("footrest exists", footrest is not None)

    ctx.expect_contact(
        seat_base,
        frame,
        elem_a="seat_deck",
        elem_b="seat_support_crossbar",
        name="seat deck is mounted to the frame support bar",
    )
    ctx.expect_contact(
        backrest,
        seat_base,
        elem_a="left_backrest_hinge_lug",
        elem_b="left_backrest_clevis",
        name="backrest left hinge lug bears on the seat clevis",
    )
    ctx.expect_contact(
        backrest,
        seat_base,
        elem_a="right_backrest_hinge_lug",
        elem_b="right_backrest_clevis",
        name="backrest right hinge lug bears on the seat clevis",
    )
    ctx.expect_contact(
        footrest,
        seat_base,
        elem_a="left_footrest_hinge_lug",
        elem_b="left_footrest_clevis",
        name="footrest left hinge lug bears on the seat clevis",
    )
    ctx.expect_contact(
        footrest,
        seat_base,
        elem_a="right_footrest_hinge_lug",
        elem_b="right_footrest_clevis",
        name="footrest right hinge lug bears on the seat clevis",
    )

    with ctx.pose({back_hinge: 0.0, foot_hinge: 0.0}):
        ctx.expect_gap(
            seat_base,
            backrest,
            axis="y",
            positive_elem="seat_cushion",
            negative_elem="backrest_cushion",
            max_gap=0.05,
            max_penetration=0.0,
            name="backrest sits just behind the seat cushion at rest",
        )
        ctx.expect_gap(
            footrest,
            seat_base,
            axis="y",
            positive_elem="footrest_pad",
            negative_elem="seat_cushion",
            max_gap=0.05,
            max_penetration=0.0,
            name="footrest begins at the front edge of the seat",
        )
        ctx.expect_overlap(
            backrest,
            seat_base,
            axes="x",
            elem_a="backrest_panel",
            elem_b="seat_deck",
            min_overlap=0.50,
            name="backrest spans the seat width",
        )
        ctx.expect_overlap(
            footrest,
            seat_base,
            axes="x",
            elem_a="footrest_panel",
            elem_b="seat_deck",
            min_overlap=0.50,
            name="footrest spans the seat width",
        )

    backrest_rest = ctx.part_element_world_aabb(backrest, elem="backrest_panel")
    with ctx.pose({back_hinge: 0.8}):
        backrest_reclined = ctx.part_element_world_aabb(backrest, elem="backrest_panel")
    ctx.check(
        "backrest reclines backward",
        backrest_rest is not None
        and backrest_reclined is not None
        and backrest_reclined[0][1] < backrest_rest[0][1] - 0.12,
        details=f"rest={backrest_rest}, reclined={backrest_reclined}",
    )

    footrest_rest = ctx.part_element_world_aabb(footrest, elem="footrest_panel")
    with ctx.pose({foot_hinge: -0.75}):
        footrest_dropped = ctx.part_element_world_aabb(footrest, elem="footrest_panel")
    ctx.check(
        "footrest rotates down from the seat front",
        footrest_rest is not None
        and footrest_dropped is not None
        and footrest_dropped[0][2] < footrest_rest[0][2] - 0.12,
        details=f"rest={footrest_rest}, dropped={footrest_dropped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
