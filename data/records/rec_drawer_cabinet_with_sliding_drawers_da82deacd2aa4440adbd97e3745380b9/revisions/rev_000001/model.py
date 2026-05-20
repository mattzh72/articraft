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


WIDTH = 0.90
DEPTH = 0.56
HEIGHT = 0.72
SIDE_T = 0.018
BACK_T = 0.006
PANEL_T = 0.018
TOE_H = 0.10
TOE_RECESS = 0.075
FRONT_T = 0.019
REVEAL = 0.003

SLIDE_T = 0.012
SLIDE_H = 0.035
RUNNER_T = 0.010
SLIDE_LENGTH = 0.43
SLIDE_FRONT_SETBACK = 0.03

DRAWER_BODY_W = 0.82
DRAWER_BODY_LEN = 0.46
DRAWER_SIDE_T = 0.013
DRAWER_BOTTOM_T = 0.010
DRAWER_BOTTOM_OFFSET = 0.018
DRAWER_TRAVEL = 0.40

TOP_DRAWER_H = 0.144
MID_DRAWER_H = 0.144
POT_DRAWER_H = 0.320
TOP_DRAWER_BOX_H = 0.10
MID_DRAWER_BOX_H = 0.10
POT_DRAWER_BOX_H = 0.25
FRONT_W = WIDTH - (2.0 * REVEAL)


def _box_center_z(front_h: float, box_h: float) -> float:
    return (-front_h * 0.5) + DRAWER_BOTTOM_OFFSET + (box_h * 0.5)


def _slide_center_y() -> float:
    return -(SLIDE_FRONT_SETBACK + (SLIDE_LENGTH * 0.5))


def _add_slide_pair(model_part, prefix: str, z_center: float, material: str) -> None:
    interior_half = (WIDTH - (2.0 * SIDE_T)) * 0.5
    x_center = interior_half - (SLIDE_T * 0.5)
    y_center = _slide_center_y()

    model_part.visual(
        Box((SLIDE_T, SLIDE_LENGTH, SLIDE_H)),
        origin=Origin(xyz=(-x_center, y_center, z_center)),
        material=material,
        name=f"{prefix}_left_slide",
    )
    model_part.visual(
        Box((SLIDE_T, SLIDE_LENGTH, SLIDE_H)),
        origin=Origin(xyz=(x_center, y_center, z_center)),
        material=material,
        name=f"{prefix}_right_slide",
    )


def _add_drawer(
    model: ArticulatedObject,
    carcass,
    *,
    name: str,
    joint_name: str,
    front_center_z: float,
    front_h: float,
    box_h: float,
    handle_len: float,
    front_material: str,
    box_material: str,
    hardware_material: str,
    recessed_pull: bool = False,
) -> None:
    drawer = model.part(name)

    if recessed_pull:
        stile_w = 0.058
        top_rail_h = 0.040
        bottom_rail_h = 0.050
        pocket_h = front_h - top_rail_h - bottom_rail_h
        pocket_w = FRONT_W - (2.0 * stile_w)
        pocket_center_z = (bottom_rail_h - top_rail_h) * 0.5

        drawer.visual(
            Box((stile_w, FRONT_T, front_h)),
            origin=Origin(
                xyz=(-(FRONT_W * 0.5) + (stile_w * 0.5), FRONT_T * 0.5, 0.0)
            ),
            material=front_material,
            name="left_front_stile",
        )
        drawer.visual(
            Box((stile_w, FRONT_T, front_h)),
            origin=Origin(
                xyz=((FRONT_W * 0.5) - (stile_w * 0.5), FRONT_T * 0.5, 0.0)
            ),
            material=front_material,
            name="right_front_stile",
        )
        drawer.visual(
            Box((pocket_w, FRONT_T, top_rail_h)),
            origin=Origin(
                xyz=(0.0, FRONT_T * 0.5, (front_h * 0.5) - (top_rail_h * 0.5))
            ),
            material=front_material,
            name="top_front_rail",
        )
        drawer.visual(
            Box((pocket_w, FRONT_T, bottom_rail_h)),
            origin=Origin(
                xyz=(0.0, FRONT_T * 0.5, -(front_h * 0.5) + (bottom_rail_h * 0.5))
            ),
            material=front_material,
            name="bottom_front_rail",
        )
        drawer.visual(
            Box((pocket_w, 0.006, pocket_h + 0.010)),
            origin=Origin(xyz=(0.0, 0.003, pocket_center_z)),
            material=box_material,
            name="front_panel",
        )

        post_x = (handle_len * 0.5) - 0.040
        drawer.visual(
            Box((0.014, 0.008, 0.016)),
            origin=Origin(xyz=(-post_x, 0.004, pocket_center_z)),
            material=hardware_material,
            name="left_handle_post",
        )
        drawer.visual(
            Box((0.014, 0.008, 0.016)),
            origin=Origin(xyz=(post_x, 0.004, pocket_center_z)),
            material=hardware_material,
            name="right_handle_post",
        )
        drawer.visual(
            Box((handle_len, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.007, pocket_center_z)),
            material=hardware_material,
            name="handle_bar",
        )
    else:
        drawer.visual(
            Box((FRONT_W, FRONT_T, front_h)),
            origin=Origin(xyz=(0.0, FRONT_T * 0.5, 0.0)),
            material=front_material,
            name="front_panel",
        )

        handle_z = (front_h * 0.5) - (0.042 if front_h < 0.20 else 0.055)
        post_x = (handle_len * 0.5) - 0.050
        drawer.visual(
            Box((0.014, 0.010, 0.018)),
            origin=Origin(xyz=(-post_x, 0.005, handle_z)),
            material=hardware_material,
            name="left_handle_post",
        )
        drawer.visual(
            Box((0.014, 0.010, 0.018)),
            origin=Origin(xyz=(post_x, 0.005, handle_z)),
            material=hardware_material,
            name="right_handle_post",
        )
        drawer.visual(
            Box((handle_len, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, 0.014, handle_z)),
            material=hardware_material,
            name="handle_bar",
        )

    box_center_z = _box_center_z(front_h, box_h)
    side_depth = DRAWER_BODY_LEN - DRAWER_SIDE_T
    side_y = -(side_depth * 0.5)
    side_x = (DRAWER_BODY_W * 0.5) - (DRAWER_SIDE_T * 0.5)
    bottom_w = DRAWER_BODY_W - (2.0 * DRAWER_SIDE_T)
    bottom_z = (-front_h * 0.5) + DRAWER_BOTTOM_OFFSET + (DRAWER_BOTTOM_T * 0.5)

    drawer.visual(
        Box((DRAWER_SIDE_T, side_depth, box_h)),
        origin=Origin(xyz=(-side_x, side_y, box_center_z)),
        material=box_material,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_T, side_depth, box_h)),
        origin=Origin(xyz=(side_x, side_y, box_center_z)),
        material=box_material,
        name="right_side",
    )
    drawer.visual(
        Box((bottom_w, side_depth, DRAWER_BOTTOM_T)),
        origin=Origin(xyz=(0.0, side_y, bottom_z)),
        material=box_material,
        name="bottom_panel",
    )
    drawer.visual(
        Box((DRAWER_BODY_W, DRAWER_SIDE_T, box_h)),
        origin=Origin(
            xyz=(0.0, -DRAWER_BODY_LEN + (DRAWER_SIDE_T * 0.5), box_center_z)
        ),
        material=box_material,
        name="back_panel",
    )

    runner_x = (DRAWER_BODY_W * 0.5) + (RUNNER_T * 0.5)
    drawer.visual(
        Box((RUNNER_T, SLIDE_LENGTH, SLIDE_H)),
        origin=Origin(xyz=(-runner_x, _slide_center_y(), box_center_z)),
        material=hardware_material,
        name="left_runner",
    )
    drawer.visual(
        Box((RUNNER_T, SLIDE_LENGTH, SLIDE_H)),
        origin=Origin(xyz=(runner_x, _slide_center_y(), box_center_z)),
        material=hardware_material,
        name="right_runner",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, front_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.8,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_base_cabinet")

    carcass_mat = model.material("cabinet_shell", rgba=(0.95, 0.95, 0.93, 1.0))
    interior_mat = model.material("cabinet_interior", rgba=(0.92, 0.92, 0.89, 1.0))
    drawer_front_mat = model.material("drawer_front", rgba=(0.96, 0.96, 0.94, 1.0))
    drawer_box_mat = model.material("drawer_box", rgba=(0.80, 0.74, 0.64, 1.0))
    hardware_mat = model.material("hardware", rgba=(0.25, 0.27, 0.30, 1.0))
    slide_mat = model.material("slide_rail", rgba=(0.58, 0.60, 0.63, 1.0))

    model.meta["object_description"] = (
        "Kitchen base cabinet with two shallow drawers and one deep pot drawer."
    )

    carcass = model.part("carcass")

    upper_side_h = HEIGHT - TOE_H
    rear_foot_depth = DEPTH - TOE_RECESS
    outer_x = (WIDTH * 0.5) - (SIDE_T * 0.5)

    carcass.visual(
        Box((SIDE_T, DEPTH, upper_side_h)),
        origin=Origin(xyz=(-outer_x, -(DEPTH * 0.5), TOE_H + (upper_side_h * 0.5))),
        material=carcass_mat,
        name="left_side_upper",
    )
    carcass.visual(
        Box((SIDE_T, DEPTH, upper_side_h)),
        origin=Origin(xyz=(outer_x, -(DEPTH * 0.5), TOE_H + (upper_side_h * 0.5))),
        material=carcass_mat,
        name="right_side_upper",
    )
    carcass.visual(
        Box((SIDE_T, rear_foot_depth, TOE_H)),
        origin=Origin(
            xyz=(-outer_x, -((DEPTH + TOE_RECESS) * 0.5), TOE_H * 0.5)
        ),
        material=carcass_mat,
        name="left_side_rear_foot",
    )
    carcass.visual(
        Box((SIDE_T, rear_foot_depth, TOE_H)),
        origin=Origin(
            xyz=(outer_x, -((DEPTH + TOE_RECESS) * 0.5), TOE_H * 0.5)
        ),
        material=carcass_mat,
        name="right_side_rear_foot",
    )
    carcass.visual(
        Box((WIDTH, PANEL_T, TOE_H)),
        origin=Origin(xyz=(0.0, -(TOE_RECESS + (PANEL_T * 0.5)), TOE_H * 0.5)),
        material=carcass_mat,
        name="toe_kick",
    )
    carcass.visual(
        Box((WIDTH - (2.0 * SIDE_T), DEPTH - BACK_T, PANEL_T)),
        origin=Origin(
            xyz=(0.0, -((DEPTH - BACK_T) * 0.5), TOE_H + (PANEL_T * 0.5))
        ),
        material=interior_mat,
        name="bottom_panel",
    )
    carcass.visual(
        Box((WIDTH - (2.0 * SIDE_T), BACK_T, HEIGHT - TOE_H - PANEL_T)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH + (BACK_T * 0.5),
                TOE_H + PANEL_T + ((HEIGHT - TOE_H - PANEL_T) * 0.5),
            )
        ),
        material=interior_mat,
        name="back_panel",
    )
    carcass.visual(
        Box((WIDTH - (2.0 * SIDE_T), 0.080, PANEL_T)),
        origin=Origin(xyz=(0.0, -0.040, HEIGHT - (PANEL_T * 0.5))),
        material=interior_mat,
        name="top_front_stretcher",
    )
    carcass.visual(
        Box((WIDTH - (2.0 * SIDE_T), 0.100, PANEL_T)),
        origin=Origin(
            xyz=(0.0, -DEPTH + BACK_T + 0.050, HEIGHT - (PANEL_T * 0.5))
        ),
        material=interior_mat,
        name="top_back_stretcher",
    )

    lower_bottom_z = TOE_H + REVEAL
    lower_center_z = lower_bottom_z + (POT_DRAWER_H * 0.5)
    middle_bottom_z = lower_bottom_z + POT_DRAWER_H + REVEAL
    middle_center_z = middle_bottom_z + (MID_DRAWER_H * 0.5)
    top_bottom_z = middle_bottom_z + MID_DRAWER_H + REVEAL
    top_center_z = top_bottom_z + (TOP_DRAWER_H * 0.5)

    _add_slide_pair(
        carcass,
        "top",
        top_center_z + _box_center_z(TOP_DRAWER_H, TOP_DRAWER_BOX_H),
        slide_mat,
    )
    _add_slide_pair(
        carcass,
        "middle",
        middle_center_z + _box_center_z(MID_DRAWER_H, MID_DRAWER_BOX_H),
        slide_mat,
    )
    _add_slide_pair(
        carcass,
        "pot",
        lower_center_z + _box_center_z(POT_DRAWER_H, POT_DRAWER_BOX_H),
        slide_mat,
    )

    _add_drawer(
        model,
        carcass,
        name="top_drawer",
        joint_name="carcass_to_top_drawer",
        front_center_z=top_center_z,
        front_h=TOP_DRAWER_H,
        box_h=TOP_DRAWER_BOX_H,
        handle_len=0.42,
        front_material=drawer_front_mat,
        box_material=drawer_box_mat,
        hardware_material=hardware_mat,
        recessed_pull=True,
    )
    _add_drawer(
        model,
        carcass,
        name="middle_drawer",
        joint_name="carcass_to_middle_drawer",
        front_center_z=middle_center_z,
        front_h=MID_DRAWER_H,
        box_h=MID_DRAWER_BOX_H,
        handle_len=0.42,
        front_material=drawer_front_mat,
        box_material=drawer_box_mat,
        hardware_material=hardware_mat,
        recessed_pull=True,
    )
    _add_drawer(
        model,
        carcass,
        name="pot_drawer",
        joint_name="carcass_to_pot_drawer",
        front_center_z=lower_center_z,
        front_h=POT_DRAWER_H,
        box_h=POT_DRAWER_BOX_H,
        handle_len=0.58,
        front_material=drawer_front_mat,
        box_material=drawer_box_mat,
        hardware_material=hardware_mat,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    top_drawer = object_model.get_part("top_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    pot_drawer = object_model.get_part("pot_drawer")

    top_joint = object_model.get_articulation("carcass_to_top_drawer")
    middle_joint = object_model.get_articulation("carcass_to_middle_drawer")
    pot_joint = object_model.get_articulation("carcass_to_pot_drawer")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    joints_ok = all(
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 1.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper == DRAWER_TRAVEL
        for joint in (top_joint, middle_joint, pot_joint)
    )
    ctx.check(
        "drawer_joints_are_full_extension_prismatics",
        joints_ok,
        "Each drawer should translate forward on a prismatic slide with 0.40 m travel.",
    )

    ctx.expect_contact(
        top_drawer,
        carcass,
        elem_a="left_runner",
        elem_b="top_left_slide",
        name="top_drawer_contacts_left_slide",
    )
    ctx.expect_contact(
        middle_drawer,
        carcass,
        elem_a="left_runner",
        elem_b="middle_left_slide",
        name="middle_drawer_contacts_left_slide",
    )
    ctx.expect_contact(
        pot_drawer,
        carcass,
        elem_a="left_runner",
        elem_b="pot_left_slide",
        name="pot_drawer_contacts_left_slide",
    )

    ctx.expect_within(
        top_drawer,
        carcass,
        axes="xz",
        margin=0.0,
        name="top_drawer_within_cabinet_face_bounds",
    )
    ctx.expect_within(
        middle_drawer,
        carcass,
        axes="xz",
        margin=0.0,
        name="middle_drawer_within_cabinet_face_bounds",
    )
    ctx.expect_within(
        pot_drawer,
        carcass,
        axes="xz",
        margin=0.0,
        name="pot_drawer_within_cabinet_face_bounds",
    )

    ctx.expect_gap(
        top_drawer,
        middle_drawer,
        axis="z",
        min_gap=REVEAL - 1e-6,
        max_gap=REVEAL + 1e-6,
        name="top_and_middle_front_reveal",
    )
    ctx.expect_gap(
        middle_drawer,
        pot_drawer,
        axis="z",
        min_gap=REVEAL - 1e-6,
        max_gap=REVEAL + 1e-6,
        name="middle_and_pot_front_reveal",
    )

    with ctx.pose({top_joint: DRAWER_TRAVEL}):
        ctx.expect_origin_gap(
            top_drawer,
            carcass,
            axis="y",
            min_gap=DRAWER_TRAVEL - 1e-6,
            max_gap=DRAWER_TRAVEL + 1e-6,
            name="top_drawer_full_extension_travel",
        )
    with ctx.pose({middle_joint: DRAWER_TRAVEL}):
        ctx.expect_origin_gap(
            middle_drawer,
            carcass,
            axis="y",
            min_gap=DRAWER_TRAVEL - 1e-6,
            max_gap=DRAWER_TRAVEL + 1e-6,
            name="middle_drawer_full_extension_travel",
        )
    with ctx.pose({pot_joint: DRAWER_TRAVEL}):
        ctx.expect_origin_gap(
            pot_drawer,
            carcass,
            axis="y",
            min_gap=DRAWER_TRAVEL - 1e-6,
            max_gap=DRAWER_TRAVEL + 1e-6,
            name="pot_drawer_full_extension_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
