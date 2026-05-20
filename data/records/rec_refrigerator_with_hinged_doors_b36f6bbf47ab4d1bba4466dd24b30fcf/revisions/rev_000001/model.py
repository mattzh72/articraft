from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bottom_freezer_refrigerator")

    enamel_white = model.material("enamel_white", rgba=(0.92, 0.94, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.71, 0.74, 0.78, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.76, 0.80, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    toe_kick_black = model.material("toe_kick_black", rgba=(0.08, 0.08, 0.08, 1.0))

    width = 0.78
    depth = 0.72
    height = 1.82
    side_t = 0.035
    back_t = 0.025
    bottom_t = 0.040
    divider_t = 0.040
    trim_h = 0.070
    door_t = 0.055
    door_gap = 0.004

    front_y = depth / 2.0
    axis_y = front_y + 0.010
    right_axis_x = (width / 2.0) + 0.006
    left_axis_x = -right_axis_x
    panel_center_rel_y = front_y + door_gap + (door_t / 2.0) - axis_y

    freezer_open_bottom = bottom_t
    freezer_open_top = 0.570
    fresh_open_bottom = freezer_open_top + divider_t
    fresh_open_top = height - trim_h

    freezer_center_z = 0.5 * (freezer_open_bottom + freezer_open_top)
    fresh_center_z = 0.5 * (fresh_open_bottom + fresh_open_top)

    fresh_door_h = (fresh_open_top - fresh_open_bottom) - 0.006
    freezer_door_h = (freezer_open_top - freezer_open_bottom) - 0.006
    door_w = 0.756
    axis_to_panel_edge = 0.018

    hinge_outer_r = 0.011
    hinge_inner_r = 0.0075
    pin_r = 0.006
    sleeve_len = 0.024
    pin_len = 0.018

    def sleeve_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (outer_radius, -length / 2.0),
                    (outer_radius, length / 2.0),
                ],
                [
                    (inner_radius, -length / 2.0),
                    (inner_radius, length / 2.0),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-(width / 2.0) + (side_t / 2.0), 0.0, height / 2.0)),
        material=enamel_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=((width / 2.0) - (side_t / 2.0), 0.0, height / 2.0)),
        material=enamel_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((width - (2.0 * side_t), back_t, height)),
        origin=Origin(xyz=(0.0, -(depth / 2.0) + (back_t / 2.0), height / 2.0)),
        material=enamel_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((width - (2.0 * side_t), depth - back_t, bottom_t)),
        origin=Origin(xyz=(0.0, back_t / 2.0, bottom_t / 2.0)),
        material=enamel_white,
        name="bottom_floor",
    )
    cabinet.visual(
        Box((width - (2.0 * side_t), depth - back_t, divider_t)),
        origin=Origin(
            xyz=(0.0, back_t / 2.0, freezer_open_top + (divider_t / 2.0)),
        ),
        material=enamel_white,
        name="center_divider",
    )
    cabinet.visual(
        Box((width - (2.0 * side_t), depth - back_t, trim_h)),
        origin=Origin(xyz=(0.0, back_t / 2.0, height - (trim_h / 2.0))),
        material=enamel_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((width - (2.0 * side_t), 0.028, trim_h - 0.004)),
        origin=Origin(xyz=(0.0, front_y - 0.014, height - (trim_h / 2.0))),
        material=trim_gray,
        name="trim_face",
    )
    cabinet.visual(
        Box((width - 0.130, 0.026, 0.090)),
        origin=Origin(xyz=(0.0, front_y - 0.013, 0.045)),
        material=toe_kick_black,
        name="toe_kick",
    )

    fresh_pin_offset_z = (fresh_door_h / 2.0) - 0.014
    freezer_pin_offset_z = (freezer_door_h / 2.0) - 0.014

    cabinet.visual(
        sleeve_mesh(
            "fresh_top_sleeve",
            outer_radius=hinge_outer_r,
            inner_radius=hinge_inner_r,
            length=sleeve_len,
        ),
        origin=Origin(xyz=(right_axis_x, axis_y, fresh_center_z + fresh_pin_offset_z)),
        material=steel,
        name="fresh_top_sleeve",
    )
    cabinet.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(
            xyz=(right_axis_x - 0.013, front_y - 0.010, fresh_center_z + fresh_pin_offset_z),
        ),
        material=trim_gray,
        name="fresh_top_bracket",
    )
    cabinet.visual(
        sleeve_mesh(
            "fresh_bottom_sleeve",
            outer_radius=hinge_outer_r,
            inner_radius=hinge_inner_r,
            length=sleeve_len,
        ),
        origin=Origin(xyz=(right_axis_x, axis_y, fresh_center_z - fresh_pin_offset_z)),
        material=steel,
        name="fresh_bottom_sleeve",
    )
    cabinet.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(
            xyz=(right_axis_x - 0.013, front_y - 0.010, fresh_center_z - fresh_pin_offset_z),
        ),
        material=trim_gray,
        name="fresh_bottom_bracket",
    )

    cabinet.visual(
        sleeve_mesh(
            "freezer_top_sleeve",
            outer_radius=hinge_outer_r,
            inner_radius=hinge_inner_r,
            length=sleeve_len,
        ),
        origin=Origin(xyz=(left_axis_x, axis_y, freezer_center_z + freezer_pin_offset_z)),
        material=steel,
        name="freezer_top_sleeve",
    )
    cabinet.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(
            xyz=(left_axis_x + 0.013, front_y - 0.010, freezer_center_z + freezer_pin_offset_z),
        ),
        material=trim_gray,
        name="freezer_top_bracket",
    )
    cabinet.visual(
        sleeve_mesh(
            "freezer_bottom_sleeve",
            outer_radius=hinge_outer_r,
            inner_radius=hinge_inner_r,
            length=sleeve_len,
        ),
        origin=Origin(xyz=(left_axis_x, axis_y, freezer_center_z - freezer_pin_offset_z)),
        material=steel,
        name="freezer_bottom_sleeve",
    )
    cabinet.visual(
        Box((0.028, 0.020, 0.030)),
        origin=Origin(
            xyz=(left_axis_x + 0.013, front_y - 0.010, freezer_center_z - freezer_pin_offset_z),
        ),
        material=trim_gray,
        name="freezer_bottom_bracket",
    )

    cabinet.visual(
        sleeve_mesh(
            "knob_bezel",
            outer_radius=0.028,
            inner_radius=0.022,
            length=0.012,
        ),
        origin=Origin(
            xyz=(-0.220, front_y + 0.006, height - (trim_h / 2.0)),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="knob_bezel",
    )

    fresh_door = model.part("fresh_food_door")
    fresh_door.visual(
        Box((door_w, door_t, fresh_door_h)),
        origin=Origin(
            xyz=(
                -axis_to_panel_edge - (door_w / 2.0),
                panel_center_rel_y,
                0.0,
            )
        ),
        material=enamel_white,
        name="door_panel",
    )
    fresh_door.visual(
        Box((0.030, 0.028, 0.720)),
        origin=Origin(
            xyz=(-axis_to_panel_edge - door_w + 0.036, panel_center_rel_y + (door_t / 2.0) + 0.014, 0.020),
        ),
        material=steel,
        name="handle_bar",
    )
    fresh_door.visual(
        Box((door_w - 0.090, 0.003, fresh_door_h - 0.090)),
        origin=Origin(
            xyz=(
                -axis_to_panel_edge - (door_w / 2.0),
                panel_center_rel_y - (door_t / 2.0) + 0.0015,
                0.0,
            )
        ),
        material=gasket_dark,
        name="gasket",
    )
    fresh_door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(0.0, 0.0, fresh_pin_offset_z)),
        material=steel,
        name="top_pin",
    )
    fresh_door.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, fresh_pin_offset_z - 0.018)),
        material=steel,
        name="top_pin_bridge",
    )
    fresh_door.visual(
        Box((0.0022, 0.002, 0.024)),
        origin=Origin(xyz=(-0.0062, 0.0, fresh_pin_offset_z - 0.014)),
        material=steel,
        name="top_pin_stem",
    )
    fresh_door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(0.0, 0.0, -fresh_pin_offset_z)),
        material=steel,
        name="bottom_pin",
    )
    fresh_door.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, -fresh_pin_offset_z + 0.018)),
        material=steel,
        name="bottom_pin_bridge",
    )
    fresh_door.visual(
        Box((0.0022, 0.002, 0.024)),
        origin=Origin(xyz=(-0.0062, 0.0, -fresh_pin_offset_z + 0.014)),
        material=steel,
        name="bottom_pin_stem",
    )
    fresh_door.visual(
        Box((axis_to_panel_edge, 0.022, 0.030)),
        origin=Origin(xyz=(-0.024, 0.0, fresh_pin_offset_z)),
        material=trim_gray,
        name="top_hinge_leaf",
    )
    fresh_door.visual(
        Box((axis_to_panel_edge, 0.022, 0.030)),
        origin=Origin(xyz=(-0.024, 0.0, -fresh_pin_offset_z)),
        material=trim_gray,
        name="bottom_hinge_leaf",
    )

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        Box((door_w, door_t, freezer_door_h)),
        origin=Origin(
            xyz=(
                axis_to_panel_edge + (door_w / 2.0),
                panel_center_rel_y,
                0.0,
            )
        ),
        material=enamel_white,
        name="door_panel",
    )
    freezer_door.visual(
        Box((0.030, 0.028, 0.320)),
        origin=Origin(
            xyz=(axis_to_panel_edge + door_w - 0.036, panel_center_rel_y + (door_t / 2.0) + 0.014, 0.000),
        ),
        material=steel,
        name="handle_bar",
    )
    freezer_door.visual(
        Box((door_w - 0.090, 0.003, freezer_door_h - 0.090)),
        origin=Origin(
            xyz=(
                axis_to_panel_edge + (door_w / 2.0),
                panel_center_rel_y - (door_t / 2.0) + 0.0015,
                0.0,
            )
        ),
        material=gasket_dark,
        name="gasket",
    )
    freezer_door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(0.0, 0.0, freezer_pin_offset_z)),
        material=steel,
        name="top_pin",
    )
    freezer_door.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, freezer_pin_offset_z - 0.018)),
        material=steel,
        name="top_pin_bridge",
    )
    freezer_door.visual(
        Box((0.0022, 0.002, 0.024)),
        origin=Origin(xyz=(0.0062, 0.0, freezer_pin_offset_z - 0.014)),
        material=steel,
        name="top_pin_stem",
    )
    freezer_door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(0.0, 0.0, -freezer_pin_offset_z)),
        material=steel,
        name="bottom_pin",
    )
    freezer_door.visual(
        Box((0.012, 0.004, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, -freezer_pin_offset_z + 0.018)),
        material=steel,
        name="bottom_pin_bridge",
    )
    freezer_door.visual(
        Box((0.0022, 0.002, 0.024)),
        origin=Origin(xyz=(0.0062, 0.0, -freezer_pin_offset_z + 0.014)),
        material=steel,
        name="bottom_pin_stem",
    )
    freezer_door.visual(
        Box((axis_to_panel_edge, 0.022, 0.030)),
        origin=Origin(xyz=(0.024, 0.0, freezer_pin_offset_z)),
        material=trim_gray,
        name="top_hinge_leaf",
    )
    freezer_door.visual(
        Box((axis_to_panel_edge, 0.022, 0.030)),
        origin=Origin(xyz=(0.024, 0.0, -freezer_pin_offset_z)),
        material=trim_gray,
        name="bottom_hinge_leaf",
    )

    thermostat_knob = model.part("thermostat_knob")
    thermostat_knob.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_dark,
        name="dial",
    )
    thermostat_knob.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    thermostat_knob.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.014, 0.007, 0.0)),
        material=steel,
        name="pointer",
    )

    model.articulation(
        "fresh_food_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fresh_door,
        origin=Origin(xyz=(right_axis_x, axis_y, fresh_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=-2.05,
            upper=0.0,
        ),
    )
    model.articulation(
        "freezer_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(left_axis_x, axis_y, freezer_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "thermostat_control",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=thermostat_knob,
        origin=Origin(xyz=(-0.220, front_y + 0.014, height - (trim_h / 2.0))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    fresh_door = object_model.get_part("fresh_food_door")
    freezer_door = object_model.get_part("freezer_door")
    thermostat_knob = object_model.get_part("thermostat_knob")

    fresh_hinge = object_model.get_articulation("fresh_food_hinge")
    freezer_hinge = object_model.get_articulation("freezer_hinge")
    thermostat_control = object_model.get_articulation("thermostat_control")

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
        "door hinges are vertical",
        tuple(fresh_hinge.axis) == (0.0, 0.0, 1.0) and tuple(freezer_hinge.axis) == (0.0, 0.0, 1.0),
        f"fresh axis={fresh_hinge.axis}, freezer axis={freezer_hinge.axis}",
    )
    ctx.check(
        "thermostat knob axis faces out of trim",
        tuple(thermostat_control.axis) == (0.0, 1.0, 0.0),
        f"knob axis={thermostat_control.axis}",
    )
    ctx.check(
        "door hinges swing from opposite sides",
        fresh_hinge.motion_limits is not None
        and freezer_hinge.motion_limits is not None
        and fresh_hinge.motion_limits.upper == 0.0
        and fresh_hinge.motion_limits.lower is not None
        and fresh_hinge.motion_limits.lower < -1.0
        and freezer_hinge.motion_limits.lower == 0.0
        and freezer_hinge.motion_limits.upper is not None
        and freezer_hinge.motion_limits.upper > 1.0,
        (
            f"fresh limits={fresh_hinge.motion_limits}, "
            f"freezer limits={freezer_hinge.motion_limits}"
        ),
    )

    ctx.expect_gap(
        fresh_door,
        cabinet,
        axis="y",
        min_gap=0.003,
        max_gap=0.008,
        positive_elem="door_panel",
        negative_elem="left_wall",
        name="fresh door panel sits just ahead of cabinet face",
    )
    ctx.expect_gap(
        freezer_door,
        cabinet,
        axis="y",
        min_gap=0.003,
        max_gap=0.008,
        positive_elem="door_panel",
        negative_elem="left_wall",
        name="freezer door panel sits just ahead of cabinet face",
    )
    ctx.expect_gap(
        thermostat_knob,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="shaft",
        negative_elem="trim_face",
        name="thermostat shaft seats into trim face",
    )

    ctx.expect_overlap(
        fresh_door,
        cabinet,
        axes="xy",
        min_overlap=0.010,
        elem_a="top_pin",
        elem_b="fresh_top_sleeve",
        name="fresh top pin remains captured by upper sleeve",
    )
    ctx.expect_overlap(
        fresh_door,
        cabinet,
        axes="xy",
        min_overlap=0.010,
        elem_a="bottom_pin",
        elem_b="fresh_bottom_sleeve",
        name="fresh bottom pin remains captured by lower sleeve",
    )
    ctx.expect_overlap(
        freezer_door,
        cabinet,
        axes="xy",
        min_overlap=0.010,
        elem_a="top_pin",
        elem_b="freezer_top_sleeve",
        name="freezer top pin remains captured by upper sleeve",
    )
    ctx.expect_overlap(
        freezer_door,
        cabinet,
        axes="xy",
        min_overlap=0.010,
        elem_a="bottom_pin",
        elem_b="freezer_bottom_sleeve",
        name="freezer bottom pin remains captured by lower sleeve",
    )
    ctx.expect_overlap(
        thermostat_knob,
        cabinet,
        axes="xz",
        min_overlap=0.020,
        elem_a="dial",
        elem_b="knob_bezel",
        name="thermostat knob stays centered in bezel",
    )

    with ctx.pose({fresh_hinge: -1.20, freezer_hinge: 1.20, thermostat_control: 0.75}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_overlap_check")
        ctx.expect_overlap(
            fresh_door,
            cabinet,
            axes="xy",
            min_overlap=0.010,
            elem_a="top_pin",
            elem_b="fresh_top_sleeve",
            name="fresh top pin stays in sleeve while open",
        )
        ctx.expect_overlap(
            freezer_door,
            cabinet,
            axes="xy",
            min_overlap=0.010,
            elem_a="top_pin",
            elem_b="freezer_top_sleeve",
            name="freezer top pin stays in sleeve while open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
