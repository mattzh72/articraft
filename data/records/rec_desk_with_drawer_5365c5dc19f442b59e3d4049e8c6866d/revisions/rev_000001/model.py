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


BODY_WIDTH = 0.96
BODY_DEPTH = 0.44
BODY_BOTTOM_Z = 0.74
BODY_HEIGHT = 0.44
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT
PANEL_THICKNESS = 0.02
OPENING_HALF_WIDTH = 0.41
OPENING_TOP_Z = 1.12
LEAF_WIDTH = 0.81
LEAF_HEIGHT = 0.36
LEAF_THICKNESS = 0.02
INTERIOR_FRONT_Y = 0.40


def _build_writing_shelf(part, wood, lining, brass) -> None:
    part.visual(
        Box((0.50, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.12, 0.009)),
        material=wood,
        name="shelf_top",
    )
    part.visual(
        Box((0.50, 0.018, 0.03)),
        origin=Origin(xyz=(0.0, 0.231, 0.015)),
        material=wood,
        name="front_lip",
    )
    part.visual(
        Box((0.44, 0.17, 0.002)),
        origin=Origin(xyz=(0.0, 0.115, 0.019)),
        material=lining,
        name="writing_pad",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.248, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shelf_pull",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.50, 0.24, 0.03)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.12, 0.015)),
    )


def _build_drawer(part, wood, brass) -> None:
    part.visual(
        Box((0.24, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, 0.231, 0.05)),
        material=wood,
        name="drawer_front",
    )
    part.visual(
        Box((0.012, 0.228, 0.07)),
        origin=Origin(xyz=(-0.114, 0.114, 0.035)),
        material=wood,
        name="left_side",
    )
    part.visual(
        Box((0.012, 0.228, 0.07)),
        origin=Origin(xyz=(0.114, 0.114, 0.035)),
        material=wood,
        name="right_side",
    )
    part.visual(
        Box((0.216, 0.012, 0.07)),
        origin=Origin(xyz=(0.0, 0.006, 0.035)),
        material=wood,
        name="back_panel",
    )
    part.visual(
        Box((0.216, 0.228, 0.008)),
        origin=Origin(xyz=(0.0, 0.114, 0.004)),
        material=wood,
        name="bottom_panel",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.248, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="knob",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.10)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.12, 0.05)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_drop_front_desk")

    walnut = model.material("walnut", rgba=(0.36, 0.23, 0.14, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.26, 0.16, 0.09, 1.0))
    lining = model.material("lining", rgba=(0.32, 0.25, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.60, 0.26, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((1.02, 0.50, 1.21)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.22, 0.605)),
    )

    # Legs and apron base.
    leg_size = 0.055
    leg_centers = (
        (-0.40, 0.05),
        (0.40, 0.05),
        (-0.40, 0.39),
        (0.40, 0.39),
    )
    for index, (x_pos, y_pos) in enumerate(leg_centers):
        body.visual(
            Box((leg_size, leg_size, 0.66)),
            origin=Origin(xyz=(x_pos, y_pos, 0.33)),
            material=walnut_dark,
            name=f"leg_{index}",
        )

    body.visual(
        Box((0.80, 0.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.41, 0.70)),
        material=walnut_dark,
        name="front_apron",
    )
    body.visual(
        Box((0.80, 0.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.03, 0.70)),
        material=walnut_dark,
        name="rear_apron",
    )
    body.visual(
        Box((0.02, 0.36, 0.08)),
        origin=Origin(xyz=(-0.41, 0.22, 0.70)),
        material=walnut_dark,
        name="left_apron",
    )
    body.visual(
        Box((0.02, 0.36, 0.08)),
        origin=Origin(xyz=(0.41, 0.22, 0.70)),
        material=walnut_dark,
        name="right_apron",
    )

    # Main case.
    body.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-0.47, BODY_DEPTH / 2.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=walnut,
        name="left_case_side",
    )
    body.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.47, BODY_DEPTH / 2.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=walnut,
        name="right_case_side",
    )
    body.visual(
        Box((0.92, 0.40, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.21, BODY_BOTTOM_Z + PANEL_THICKNESS / 2.0)),
        material=walnut,
        name="case_floor",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0, BODY_TOP_Z - PANEL_THICKNESS / 2.0)),
        material=walnut,
        name="case_top",
    )
    body.visual(
        Box((0.92, 0.012, 0.40)),
        origin=Origin(xyz=(0.0, 0.006, 0.95)),
        material=walnut_dark,
        name="back_panel",
    )
    body.visual(
        Box((0.07, 0.02, 0.38)),
        origin=Origin(xyz=(-0.445, 0.43, 0.93)),
        material=walnut,
        name="left_stile",
    )
    body.visual(
        Box((0.07, 0.02, 0.38)),
        origin=Origin(xyz=(0.445, 0.43, 0.93)),
        material=walnut,
        name="right_stile",
    )
    body.visual(
        Box((0.82, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.43, 1.15)),
        material=walnut,
        name="top_rail",
    )
    body.visual(
        Box((1.02, 0.48, 0.03)),
        origin=Origin(xyz=(0.0, 0.24, 1.195)),
        material=walnut_dark,
        name="cornice",
    )

    # Interior desk fittings.
    body.visual(
        Box((0.016, 0.38, 0.34)),
        origin=Origin(xyz=(0.14, 0.22, 0.92)),
        material=walnut_dark,
        name="drawer_partition",
    )
    body.visual(
        Box((0.592, 0.35, 0.014)),
        origin=Origin(xyz=(-0.164, 0.187, 0.96)),
        material=walnut,
        name="pigeonhole_shelf",
    )
    body.visual(
        Box((0.014, 0.22, 0.13)),
        origin=Origin(xyz=(-0.29, 0.122, 1.032)),
        material=walnut,
        name="left_cubby_divider",
    )
    body.visual(
        Box((0.014, 0.22, 0.13)),
        origin=Origin(xyz=(-0.08, 0.122, 1.032)),
        material=walnut,
        name="center_cubby_divider",
    )

    body.visual(
        Box((0.04, 0.28, 0.02)),
        origin=Origin(xyz=(-0.44, 0.22, 0.82)),
        material=walnut_dark,
        name="shelf_runner_left",
    )
    body.visual(
        Box((0.04, 0.28, 0.02)),
        origin=Origin(xyz=(0.112, 0.22, 0.82)),
        material=walnut_dark,
        name="shelf_runner_right",
    )

    drawer_rail_specs = (
        ("lower_left_rail", 0.151, 0.22, 0.888),
        ("lower_right_rail", 0.407, 0.22, 0.888),
        ("upper_left_rail", 0.151, 0.22, 1.008),
        ("upper_right_rail", 0.407, 0.22, 1.008),
    )
    for rail_name, x_pos, y_pos, z_pos in drawer_rail_specs:
        body.visual(
            Box((0.016, 0.24, 0.016)),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=walnut_dark,
            name=rail_name,
        )
    body.visual(
        Box((0.045, 0.24, 0.136)),
        origin=Origin(xyz=(0.4375, 0.22, 0.948)),
        material=walnut_dark,
        name="right_drawer_web",
    )

    drop_leaf = model.part("drop_leaf")
    drop_leaf.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(xyz=(0.0, LEAF_THICKNESS / 2.0, LEAF_HEIGHT / 2.0)),
        material=walnut,
        name="writing_panel",
    )
    drop_leaf.visual(
        Box((0.71, 0.002, 0.28)),
        origin=Origin(xyz=(0.0, 0.001, 0.18)),
        material=lining,
        name="writing_inset",
    )
    drop_leaf.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(-0.18, 0.028, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="left_pull",
    )
    drop_leaf.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.18, 0.028, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="right_pull",
    )
    drop_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        mass=6.2,
        origin=Origin(xyz=(0.0, LEAF_THICKNESS / 2.0, LEAF_HEIGHT / 2.0)),
    )

    writing_shelf = model.part("writing_shelf")
    _build_writing_shelf(writing_shelf, walnut, lining, brass)

    lower_drawer = model.part("lower_drawer")
    _build_drawer(lower_drawer, walnut, brass)

    upper_drawer = model.part("upper_drawer")
    _build_drawer(upper_drawer, walnut, brass)

    model.articulation(
        "body_to_drop_leaf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drop_leaf,
        origin=Origin(xyz=(0.0, 0.42, BODY_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )
    model.articulation(
        "body_to_writing_shelf",
        ArticulationType.PRISMATIC,
        parent=body,
        child=writing_shelf,
        origin=Origin(xyz=(-0.14, 0.16, 0.83)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.16,
        ),
    )
    model.articulation(
        "body_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_drawer,
        origin=Origin(xyz=(0.279, 0.16, 0.86)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.14,
        ),
    )
    model.articulation(
        "body_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_drawer,
        origin=Origin(xyz=(0.279, 0.16, 0.98)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drop_leaf = object_model.get_part("drop_leaf")
    writing_shelf = object_model.get_part("writing_shelf")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_drawer = object_model.get_part("upper_drawer")

    leaf_joint = object_model.get_articulation("body_to_drop_leaf")
    shelf_joint = object_model.get_articulation("body_to_writing_shelf")
    lower_drawer_joint = object_model.get_articulation("body_to_lower_drawer")
    upper_drawer_joint = object_model.get_articulation("body_to_upper_drawer")

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

    ctx.check(
        "primary articulations use intended axes",
        leaf_joint.axis == (-1.0, 0.0, 0.0)
        and shelf_joint.axis == (0.0, 1.0, 0.0)
        and lower_drawer_joint.axis == (0.0, 1.0, 0.0)
        and upper_drawer_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"leaf={leaf_joint.axis}, shelf={shelf_joint.axis}, "
            f"lower={lower_drawer_joint.axis}, upper={upper_drawer_joint.axis}"
        ),
    )

    closed_leaf_aabb = ctx.part_element_world_aabb(drop_leaf, elem="writing_panel")
    ctx.check(
        "drop leaf closes as a vertical front panel",
        closed_leaf_aabb is not None
        and closed_leaf_aabb[1][2] - closed_leaf_aabb[0][2] > 0.34
        and closed_leaf_aabb[1][1] - closed_leaf_aabb[0][1] < 0.03
        and closed_leaf_aabb[1][1] <= BODY_DEPTH + 0.001,
        details=f"closed leaf aabb={closed_leaf_aabb}",
    )

    with ctx.pose({leaf_joint: pi / 2.0}):
        open_leaf_aabb = ctx.part_element_world_aabb(drop_leaf, elem="writing_panel")
        ctx.check(
            "drop leaf opens flat as a writing surface",
            open_leaf_aabb is not None
            and open_leaf_aabb[1][1] - open_leaf_aabb[0][1] > 0.34
            and open_leaf_aabb[1][2] - open_leaf_aabb[0][2] < 0.03
            and abs(open_leaf_aabb[1][2] - BODY_BOTTOM_Z) < 0.002,
            details=f"open leaf aabb={open_leaf_aabb}",
        )

    shelf_rest = ctx.part_element_world_aabb(writing_shelf, elem="shelf_top")
    with ctx.pose({shelf_joint: shelf_joint.motion_limits.upper}):
        shelf_extended = ctx.part_element_world_aabb(writing_shelf, elem="shelf_top")
        ctx.check(
            "writing shelf slides forward and remains inserted",
            shelf_rest is not None
            and shelf_extended is not None
            and shelf_extended[1][1] > shelf_rest[1][1] + 0.14
            and shelf_extended[0][1] < INTERIOR_FRONT_Y
            and shelf_extended[1][1] > BODY_DEPTH + 0.10,
            details=f"rest={shelf_rest}, extended={shelf_extended}",
        )

    lower_rest = ctx.part_element_world_aabb(lower_drawer, elem="drawer_front")
    upper_rest = ctx.part_element_world_aabb(upper_drawer, elem="drawer_front")
    with ctx.pose(
        {
            lower_drawer_joint: lower_drawer_joint.motion_limits.upper,
            upper_drawer_joint: upper_drawer_joint.motion_limits.upper,
        }
    ):
        lower_extended = ctx.part_element_world_aabb(lower_drawer, elem="drawer_front")
        upper_extended = ctx.part_element_world_aabb(upper_drawer, elem="drawer_front")
        ctx.check(
            "stacked drawers extend outward on their runners",
            lower_rest is not None
            and upper_rest is not None
            and lower_extended is not None
            and upper_extended is not None
            and lower_extended[1][1] > lower_rest[1][1] + 0.12
            and upper_extended[1][1] > upper_rest[1][1] + 0.12
            and lower_extended[0][2] >= BODY_BOTTOM_Z + 0.11
            and upper_extended[0][2] > lower_extended[1][2] + 0.01,
            details=(
                f"lower_rest={lower_rest}, lower_extended={lower_extended}, "
                f"upper_rest={upper_rest}, upper_extended={upper_extended}"
            ),
        )

    with ctx.pose(
        {
            leaf_joint: pi / 2.0,
            shelf_joint: shelf_joint.motion_limits.upper,
            lower_drawer_joint: lower_drawer_joint.motion_limits.upper,
            upper_drawer_joint: upper_drawer_joint.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="open desk mechanisms remain clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
