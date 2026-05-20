from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="location_slate")

    board_w = 0.280
    board_h = 0.220
    board_t = 0.0065

    stick_w = 0.268
    stick_h = 0.036
    stick_t = 0.012

    trunnion_r = 0.0045
    trunnion_len = 0.010

    cheek_t = 0.004
    cheek_d = 0.020
    cheek_h = 0.050

    pin_head_r = 0.004
    pin_head_len = 0.004

    board_top_z = board_h / 2.0
    hinge_z = board_top_z + trunnion_r

    paint_depth = 0.0007
    stripe_depth = 0.0011
    face_paint_y = board_t / 2.0 + paint_depth / 2.0 - 0.0002
    stick_paint_y = stick_t / 2.0 + stripe_depth / 2.0 - 0.0003

    board_black = model.material("board_black", rgba=(0.11, 0.12, 0.13, 1.0))
    chalk_white = model.material("chalk_white", rgba=(0.90, 0.90, 0.86, 1.0))
    worn_white = model.material("worn_white", rgba=(0.86, 0.86, 0.80, 1.0))
    dull_metal = model.material("dull_metal", rgba=(0.57, 0.60, 0.63, 1.0))

    slate = model.part("slate")
    slate.visual(
        Box((board_w, board_t, board_h)),
        material=board_black,
        name="board_panel",
    )

    border_inset = 0.012
    border_width = 0.004
    inner_w = board_w - 2.0 * border_inset
    inner_h = board_h - 2.0 * border_inset

    border_specs = (
        ("front_border_top", (inner_w, paint_depth, border_width), (0.0, face_paint_y, board_h / 2.0 - border_inset)),
        ("front_border_bottom", (inner_w, paint_depth, border_width), (0.0, face_paint_y, -board_h / 2.0 + border_inset)),
        ("front_border_left", (border_width, paint_depth, inner_h), (-board_w / 2.0 + border_inset, face_paint_y, 0.0)),
        ("front_border_right", (border_width, paint_depth, inner_h), (board_w / 2.0 - border_inset, face_paint_y, 0.0)),
    )
    for name, size, xyz in border_specs:
        slate.visual(Box(size), origin=Origin(xyz=xyz), material=chalk_white, name=name)

    divider_specs = (
        ("header_divider", (inner_w * 0.94, paint_depth, 0.0032), (0.0, face_paint_y, 0.040)),
        ("middle_divider", (inner_w * 0.94, paint_depth, 0.0032), (0.0, face_paint_y, -0.006)),
        ("footer_divider", (inner_w * 0.94, paint_depth, 0.0032), (0.0, face_paint_y, -0.056)),
        ("column_divider_0", (0.0032, paint_depth, 0.092), (-0.060, face_paint_y, -0.010)),
        ("column_divider_1", (0.0032, paint_depth, 0.092), (0.020, face_paint_y, -0.010)),
        ("column_divider_2", (0.0032, paint_depth, 0.092), (0.095, face_paint_y, -0.010)),
        ("title_strip", (0.130, paint_depth, 0.012), (-0.040, face_paint_y, 0.074)),
        ("scene_strip", (0.050, paint_depth, 0.012), (0.078, face_paint_y, 0.074)),
    )
    for name, size, xyz in divider_specs:
        slate.visual(Box(size), origin=Origin(xyz=xyz), material=chalk_white, name=name)

    scuff_specs = (
        ("scuff_0", (0.018, paint_depth, 0.004), (-0.088, face_paint_y, 0.018)),
        ("scuff_1", (0.026, paint_depth, 0.004), (0.064, face_paint_y, -0.042)),
        ("scuff_2", (0.012, paint_depth, 0.0035), (0.112, face_paint_y, 0.018)),
    )
    for name, size, xyz in scuff_specs:
        slate.visual(Box(size), origin=Origin(xyz=xyz), material=worn_white, name=name)

    cheek_z = board_top_z + cheek_h / 2.0 - 0.018
    cheek_x = board_w / 2.0 - cheek_t / 2.0
    slate.visual(
        Box((cheek_t, cheek_d, cheek_h)),
        origin=Origin(xyz=(-cheek_x, 0.0, cheek_z)),
        material=dull_metal,
        name="side_cheek_0",
    )
    slate.visual(
        Box((cheek_t, cheek_d, cheek_h)),
        origin=Origin(xyz=(cheek_x, 0.0, cheek_z)),
        material=dull_metal,
        name="side_cheek_1",
    )

    pin_x = board_w / 2.0 + pin_head_len / 2.0 - 0.0005
    for i, x in enumerate((-pin_x, pin_x)):
        slate.visual(
            Cylinder(radius=pin_head_r, length=pin_head_len),
            origin=Origin(xyz=(x, 0.0, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dull_metal,
            name=f"pin_head_{i}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((stick_w, stick_t, stick_h)),
        origin=Origin(xyz=(0.0, 0.0, stick_h / 2.0 - trunnion_r)),
        material=board_black,
        name="stick_body",
    )

    trunnion_x = stick_w / 2.0 - trunnion_len / 2.0 - 0.002
    for i, x in enumerate((-trunnion_x, trunnion_x)):
        clapstick.visual(
            Cylinder(radius=trunnion_r, length=trunnion_len),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dull_metal,
            name=f"trunnion_{i}",
        )

    stripe_angle = 0.56
    stripe_centers = (-0.102, -0.051, 0.0, 0.051, 0.102)
    for i, x in enumerate(stripe_centers):
        clapstick.visual(
            Box((0.054, stripe_depth, 0.008)),
            origin=Origin(xyz=(x, stick_paint_y, 0.015), rpy=(0.0, stripe_angle, 0.0)),
            material=worn_white,
            name=f"stripe_{i}",
        )

    edge_scuffs = (
        ("edge_wear_0", (-0.121, stick_paint_y, 0.0065)),
        ("edge_wear_1", (0.118, stick_paint_y, 0.024)),
    )
    for name, xyz in edge_scuffs:
        clapstick.visual(
            Box((0.012, stripe_depth, 0.004)),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.26, 0.0)),
            material=chalk_white,
            name=name,
        )

    model.articulation(
        "slate_hinge",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("slate_hinge")
    limits = hinge.motion_limits

    ctx.expect_gap(
        clapstick,
        slate,
        axis="z",
        positive_elem="stick_body",
        negative_elem="board_panel",
        min_gap=0.0,
        max_gap=0.0015,
        name="clapstick seats on the slate top edge when closed",
    )
    ctx.expect_overlap(
        clapstick,
        slate,
        axes="x",
        elem_a="stick_body",
        elem_b="board_panel",
        min_overlap=0.240,
        name="clapstick spans nearly the full slate width",
    )
    ctx.expect_within(
        clapstick,
        slate,
        axes="x",
        inner_elem="stick_body",
        outer_elem="board_panel",
        margin=0.008,
        name="clapstick stays captured between the side cheeks",
    )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")

        opens_forward = (
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.020
        )
        ctx.check(
            "clapstick opens forward from the slate face",
            opens_forward,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
