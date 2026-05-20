from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_DEPTH = 0.78
BODY_WIDTH = 1.46
BODY_HEIGHT = 0.89
WALL_THICKNESS = 0.05
FLOOR_THICKNESS = 0.055
DIVIDER_THICKNESS = 0.03

LID_THICKNESS = 0.055
LID_DEPTH = 0.762
SEAM_GAP = 0.01
SIDE_REVEAL = 0.008
HINGE_AXIS_ABOVE_BODY = 0.03
HINGE_RADIUS = 0.016
HINGE_BARREL_LENGTH = 0.12
HINGE_Y_OFFSET = 0.28
HINGE_BRACKET_DEPTH = 0.022
HINGE_BRACKET_WIDTH = 0.055
HINGE_BRACKET_HEIGHT = 0.05
LID_FRONT_LIP_HEIGHT = 0.03

LID_WIDTH = (BODY_WIDTH - 2.0 * SIDE_REVEAL - SEAM_GAP) / 2.0
LID_CENTER_OFFSET_Y = SEAM_GAP / 2.0 + LID_WIDTH / 2.0


def _add_lid_visuals(model: ArticulatedObject, part_name: str) -> None:
    lid = model.part(part_name)
    lid_material = "appliance_white"
    trim_material = "trim_gray"
    gasket_material = "gasket_gray"
    hinge_material = "hinge_dark"

    panel_center_z = (LID_THICKNESS / 2.0) - HINGE_AXIS_ABOVE_BODY

    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, panel_center_z)),
        material=lid_material,
        name=f"{part_name}_panel",
    )

    lid.visual(
        Box((0.04, LID_WIDTH, 0.032)),
        origin=Origin(xyz=(0.02, 0.0, -0.014)),
        material=trim_material,
        name=f"{part_name}_rear_header",
    )

    lid.visual(
        Box((0.03, LID_WIDTH - 0.08, LID_FRONT_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                LID_DEPTH - 0.055,
                0.0,
                -HINGE_AXIS_ABOVE_BODY - (LID_FRONT_LIP_HEIGHT / 2.0),
            )
        ),
        material=gasket_material,
        name=f"{part_name}_front_lip",
    )

    lid.visual(
        Box((0.03, LID_WIDTH * 0.56, 0.022)),
        origin=Origin(xyz=(LID_DEPTH - 0.022, 0.0, panel_center_z + (LID_THICKNESS / 2.0) + 0.011)),
        material=trim_material,
        name=f"{part_name}_handle",
    )

    for index, sign in enumerate((-1.0, 1.0), start=1):
        lid.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(
                xyz=(0.005, sign * (LID_WIDTH * HINGE_Y_OFFSET), 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_material,
            name=f"{part_name}_hinge_barrel_{index}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        mass=6.5,
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, panel_center_z)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_door_chest_freezer")

    model.material("appliance_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("liner_white", rgba=(0.86, 0.88, 0.89, 1.0))
    model.material("trim_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("gasket_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("hinge_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    body = model.part("body")

    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL_THICKNESS, BODY_WIDTH - 2.0 * WALL_THICKNESS, FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FLOOR_THICKNESS / 2.0,
            )
        ),
        material="liner_white",
        name="floor_pan",
    )

    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_DEPTH - WALL_THICKNESS) / 2.0,
                0.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material="appliance_white",
        name="front_wall",
    )

    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_DEPTH - WALL_THICKNESS) / 2.0,
                0.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material="appliance_white",
        name="rear_wall",
    )

    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_WIDTH - WALL_THICKNESS) / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material="appliance_white",
        name="left_wall",
    )

    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_WIDTH - WALL_THICKNESS) / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material="appliance_white",
        name="right_wall",
    )

    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL_THICKNESS, DIVIDER_THICKNESS, BODY_HEIGHT - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FLOOR_THICKNESS + (BODY_HEIGHT - FLOOR_THICKNESS) / 2.0,
            )
        ),
        material="liner_white",
        name="center_divider",
    )

    hinge_bracket_center_z = (
        BODY_HEIGHT + HINGE_AXIS_ABOVE_BODY - HINGE_RADIUS - 0.002 - (HINGE_BRACKET_HEIGHT / 2.0)
    )
    for lid_center_y in (-LID_CENTER_OFFSET_Y, LID_CENTER_OFFSET_Y):
        for barrel_sign in (-1.0, 1.0):
            body.visual(
                Box((HINGE_BRACKET_DEPTH, HINGE_BRACKET_WIDTH, HINGE_BRACKET_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -(BODY_DEPTH / 2.0) - (HINGE_BRACKET_DEPTH / 2.0),
                        lid_center_y + barrel_sign * (LID_WIDTH * HINGE_Y_OFFSET),
                        hinge_bracket_center_z,
                    )
                ),
                material="hinge_dark",
                name=(
                    f"hinge_bracket_"
                    f"{'left' if lid_center_y < 0.0 else 'right'}_"
                    f"{'inner' if barrel_sign > 0.0 else 'outer'}"
                ),
            )

    body.visual(
        Box((0.012, BODY_WIDTH * 0.42, 0.10)),
        origin=Origin(
            xyz=(
                (BODY_DEPTH / 2.0) + 0.006,
                0.0,
                0.14,
            )
        ),
        material="trim_gray",
        name="front_grille",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.06, 0.09, 0.018)),
                origin=Origin(
                    xyz=(
                        x_sign * (BODY_DEPTH / 2.0 - 0.11),
                        y_sign * (BODY_WIDTH / 2.0 - 0.13),
                        -0.009,
                    )
                ),
                material="trim_gray",
                name=f"foot_{'front' if x_sign > 0 else 'rear'}_{'right' if y_sign > 0 else 'left'}",
            )

    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    _add_lid_visuals(model, "left_lid")
    _add_lid_visuals(model, "right_lid")

    left_lid = model.get_part("left_lid")
    right_lid = model.get_part("right_lid")

    model.articulation(
        "body_to_left_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_lid,
        origin=Origin(
            xyz=(
                -(BODY_DEPTH / 2.0),
                -LID_CENTER_OFFSET_Y,
                BODY_HEIGHT + HINGE_AXIS_ABOVE_BODY,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.4,
        ),
    )

    model.articulation(
        "body_to_right_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_lid,
        origin=Origin(
            xyz=(
                -(BODY_DEPTH / 2.0),
                LID_CENTER_OFFSET_Y,
                BODY_HEIGHT + HINGE_AXIS_ABOVE_BODY,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_hinge = object_model.get_articulation("body_to_left_lid")
    right_hinge = object_model.get_articulation("body_to_right_lid")

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

    ctx.expect_gap(
        left_lid,
        body,
        axis="z",
        positive_elem="left_lid_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="left lid front edge seats on the body top line",
    )
    ctx.expect_gap(
        right_lid,
        body,
        axis="z",
        positive_elem="right_lid_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="right lid front edge seats on the body top line",
    )
    ctx.expect_overlap(
        left_lid,
        body,
        axes="xy",
        elem_a="left_lid_panel",
        min_overlap=0.60,
        name="left lid covers the left freezer opening",
    )
    ctx.expect_overlap(
        right_lid,
        body,
        axes="xy",
        elem_a="right_lid_panel",
        min_overlap=0.60,
        name="right lid covers the right freezer opening",
    )
    ctx.expect_gap(
        right_lid,
        left_lid,
        axis="y",
        positive_elem="right_lid_panel",
        negative_elem="left_lid_panel",
        min_gap=0.009,
        max_gap=0.011,
        name="the twin lids keep a narrow center seam",
    )

    left_closed_panel = ctx.part_element_world_aabb(left_lid, elem="left_lid_panel")
    right_closed_panel = ctx.part_element_world_aabb(right_lid, elem="right_lid_panel")

    same_width = (
        left_closed_panel is not None
        and right_closed_panel is not None
        and abs(
            (left_closed_panel[1][1] - left_closed_panel[0][1])
            - (right_closed_panel[1][1] - right_closed_panel[0][1])
        )
        <= 1e-6
    )
    ctx.check(
        "the two lid panels are equal width",
        same_width,
        details=f"left={left_closed_panel}, right={right_closed_panel}",
    )

    with ctx.pose({left_hinge: 1.15, right_hinge: 0.0}):
        left_open_panel = ctx.part_element_world_aabb(left_lid, elem="left_lid_panel")
        right_still_closed_panel = ctx.part_element_world_aabb(right_lid, elem="right_lid_panel")

    left_opens_independently = (
        left_closed_panel is not None
        and left_open_panel is not None
        and right_closed_panel is not None
        and right_still_closed_panel is not None
        and left_open_panel[1][2] > left_closed_panel[1][2] + 0.28
        and abs(right_still_closed_panel[1][2] - right_closed_panel[1][2]) <= 1e-6
    )
    ctx.check(
        "left lid opens upward while the right lid stays shut",
        left_opens_independently,
        details=(
            f"left_closed={left_closed_panel}, left_open={left_open_panel}, "
            f"right_closed={right_closed_panel}, right_while_left_open={right_still_closed_panel}"
        ),
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 1.15}):
        left_still_closed_panel = ctx.part_element_world_aabb(left_lid, elem="left_lid_panel")
        right_open_panel = ctx.part_element_world_aabb(right_lid, elem="right_lid_panel")

    right_opens_independently = (
        right_closed_panel is not None
        and right_open_panel is not None
        and left_closed_panel is not None
        and left_still_closed_panel is not None
        and right_open_panel[1][2] > right_closed_panel[1][2] + 0.28
        and abs(left_still_closed_panel[1][2] - left_closed_panel[1][2]) <= 1e-6
    )
    ctx.check(
        "right lid opens upward while the left lid stays shut",
        right_opens_independently,
        details=(
            f"right_closed={right_closed_panel}, right_open={right_open_panel}, "
            f"left_closed={left_closed_panel}, left_while_right_open={left_still_closed_panel}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
