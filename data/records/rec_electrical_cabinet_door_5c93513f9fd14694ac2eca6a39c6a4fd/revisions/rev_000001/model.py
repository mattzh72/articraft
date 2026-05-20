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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="relay_protection_panel")

    cabinet_gray = model.material("cabinet_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.60, 0.63, 0.67, 1.0))
    inner_panel_white = model.material("inner_panel_white", rgba=(0.92, 0.93, 0.91, 1.0))
    relay_black = model.material("relay_black", rgba=(0.16, 0.17, 0.18, 1.0))
    display_black = model.material("display_black", rgba=(0.08, 0.09, 0.10, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.12, 0.12, 1.0))
    signal_green = model.material("signal_green", rgba=(0.13, 0.58, 0.24, 1.0))
    signal_amber = model.material("signal_amber", rgba=(0.83, 0.56, 0.08, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.34, 0.36, 0.39, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.11, 0.12, 1.0))
    copper = model.material("copper", rgba=(0.72, 0.46, 0.20, 1.0))
    wireway_gray = model.material("wireway_gray", rgba=(0.68, 0.70, 0.72, 1.0))

    cabinet_width = 0.80
    cabinet_depth = 0.45
    cabinet_height = 2.10
    shell_thickness = 0.0025
    plinth_height = 0.10
    shell_height = cabinet_height - plinth_height
    front_frame_depth = 0.030
    hinge_radius = 0.011

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.68, 0.32, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=cabinet_shadow,
        name="plinth",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - shell_thickness * 0.5)),
        material=cabinet_gray,
        name="roof_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * shell_thickness, cabinet_depth - shell_thickness, shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -shell_thickness * 0.25,
                plinth_height + shell_thickness * 0.5,
            )
        ),
        material=cabinet_gray,
        name="bottom_floor",
    )
    cabinet.visual(
        Box((shell_thickness, cabinet_depth, shell_height)),
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + shell_thickness * 0.5,
                0.0,
                plinth_height + shell_height * 0.5,
            )
        ),
        material=cabinet_gray,
        name="left_side",
    )
    cabinet.visual(
        Box((shell_thickness, cabinet_depth, shell_height)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - shell_thickness * 0.5,
                0.0,
                plinth_height + shell_height * 0.5,
            )
        ),
        material=cabinet_gray,
        name="right_side",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * shell_thickness, shell_thickness, shell_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth * 0.5 + shell_thickness * 0.5,
                plinth_height + shell_height * 0.5,
            )
        ),
        material=cabinet_gray,
        name="back_panel",
    )

    jamb_width = 0.035
    lower_frame_height = 0.070
    upper_frame_height = 0.050
    frame_center_y = cabinet_depth * 0.5 - front_frame_depth * 0.5
    cabinet.visual(
        Box((jamb_width, front_frame_depth, shell_height)),
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + jamb_width * 0.5,
                frame_center_y,
                plinth_height + shell_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((jamb_width, front_frame_depth, shell_height)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - jamb_width * 0.5,
                frame_center_y,
                plinth_height + shell_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="front_right_jamb",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * jamb_width, front_frame_depth, upper_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                cabinet_height - upper_frame_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * jamb_width, front_frame_depth, lower_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                plinth_height + lower_frame_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((0.080, 0.080, 1.70)),
        origin=Origin(xyz=(0.305, -0.185, 0.95)),
        material=wireway_gray,
        name="rear_wireway",
    )
    cabinet.visual(
        Box((0.24, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.208, 0.145)),
        material=cabinet_shadow,
        name="gland_plate",
    )
    panel_hinge_x = -0.334
    panel_hinge_y = 0.075
    panel_hinge_radius = 0.009
    cabinet.visual(
        Box((0.018, 0.022, 1.74)),
        origin=Origin(xyz=(-0.355, panel_hinge_y, 0.97)),
        material=wireway_gray,
        name="instrument_hinge_post",
    )
    cabinet.visual(
        Box((0.18, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, -0.208, 2.0675)),
        material=cabinet_shadow,
        name="top_cable_trough",
    )

    door_hinge_x = -cabinet_width * 0.5 - 0.014
    door_hinge_y = cabinet_depth * 0.5
    for hinge_name, hinge_z in (("upper", 1.72), ("lower", 0.38)):
        pass
    for hinge_name, hinge_z in (("upper_panel", 1.45), ("lower_panel", 0.55)):
        cabinet.visual(
            Cylinder(radius=panel_hinge_radius, length=0.032),
            origin=Origin(xyz=(-0.352, 0.075, hinge_z)),
            material=hinge_dark,
            name=f"{hinge_name}_hinge_barrel",
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=168.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    door = model.part("front_door")
    door_width = 0.758
    door_height = 1.99
    door_depth = 0.028
    door_return = 0.044
    door.visual(
        Box((door_width, 0.002, door_height)),
        origin=Origin(xyz=(hinge_radius + door_width * 0.5, door_depth - 0.001, 0.0)),
        material=cabinet_gray,
        name="door_shell_front",
    )
    door.visual(
        Box((door_return, door_depth, door_height)),
        origin=Origin(xyz=(hinge_radius + door_return * 0.5, door_depth * 0.5, 0.0)),
        material=cabinet_gray,
        name="door_left_return",
    )
    door.visual(
        Box((door_return, door_depth, door_height)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width - door_return * 0.5,
                door_depth * 0.5,
                0.0,
            )
        ),
        material=cabinet_gray,
        name="door_right_return",
    )
    door.visual(
        Box((door_width - 2.0 * door_return, door_depth, door_return)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width * 0.5,
                door_depth * 0.5,
                door_height * 0.5 - door_return * 0.5,
            )
        ),
        material=cabinet_gray,
        name="door_top_return",
    )
    door.visual(
        Box((door_width - 2.0 * door_return, door_depth, door_return)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width * 0.5,
                door_depth * 0.5,
                -door_height * 0.5 + door_return * 0.5,
            )
        ),
        material=cabinet_gray,
        name="door_bottom_return",
    )
    door.visual(
        Box((door_width - 0.14, 0.016, door_height - 0.22)),
        origin=Origin(xyz=(hinge_radius + door_width * 0.5, 0.018, 0.0)),
        material=cabinet_shadow,
        name="door_inner_reinforcement",
    )
    door.visual(
        Box((0.060, 0.016, 0.260)),
        origin=Origin(xyz=(hinge_radius + door_width - 0.060, door_depth + 0.008, 0.0)),
        material=handle_black,
        name="latch_handle_body",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(
            xyz=(hinge_radius + door_width - 0.070, door_depth + 0.017, 0.0),
        ),
        material=handle_black,
        name="latch_pull",
    )
    door.visual(
        Box((0.16, 0.003, 0.05)),
        origin=Origin(
            xyz=(hinge_radius + door_width * 0.5, door_depth + 0.0015, door_height * 0.5 - 0.14)
        ),
        material=wireway_gray,
        name="door_nameplate",
    )
    for hinge_name, hinge_z in (("upper", 0.67), ("lower", -0.67)):
        door.visual(
            Box((0.012, 0.018, 0.120)),
            origin=Origin(xyz=(hinge_radius * 0.5, 0.020, hinge_z)),
            material=hinge_dark,
            name=f"{hinge_name}_hinge_knuckle_leaf",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=hinge_dark,
            name=f"{hinge_name}_hinge_knuckle",
        )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_depth, door_height)),
        mass=32.0,
        origin=Origin(xyz=(hinge_radius + door_width * 0.5, door_depth * 0.5, 0.0)),
    )

    panel = model.part("instrument_panel")
    panel_width = 0.650
    panel_height = 1.72
    panel_depth = 0.016
    panel_return = 0.034
    panel.visual(
        Box((panel_width, 0.002, panel_height)),
        origin=Origin(xyz=(panel_hinge_radius + panel_width * 0.5, panel_depth - 0.001, 0.0)),
        material=inner_panel_white,
        name="panel_face",
    )
    panel.visual(
        Box((panel_return, panel_depth + 0.010, panel_height)),
        origin=Origin(
            xyz=(
                panel_hinge_radius + panel_return * 0.5,
                (panel_depth + 0.010) * 0.5,
                0.0,
            )
        ),
        material=inner_panel_white,
        name="panel_left_return",
    )
    panel.visual(
        Box((panel_return, panel_depth + 0.010, panel_height)),
        origin=Origin(
            xyz=(
                panel_hinge_radius + panel_width - panel_return * 0.5,
                (panel_depth + 0.010) * 0.5,
                0.0,
            )
        ),
        material=inner_panel_white,
        name="panel_right_return",
    )
    panel.visual(
        Box((panel_width - 2.0 * panel_return, panel_depth + 0.010, panel_return)),
        origin=Origin(
            xyz=(
                panel_hinge_radius + panel_width * 0.5,
                (panel_depth + 0.010) * 0.5,
                panel_height * 0.5 - panel_return * 0.5,
            )
        ),
        material=inner_panel_white,
        name="panel_top_return",
    )
    panel.visual(
        Box((panel_width - 2.0 * panel_return, panel_depth + 0.010, panel_return)),
        origin=Origin(
            xyz=(
                panel_hinge_radius + panel_width * 0.5,
                (panel_depth + 0.010) * 0.5,
                -panel_height * 0.5 + panel_return * 0.5,
            )
        ),
        material=inner_panel_white,
        name="panel_bottom_return",
    )
    panel.visual(
        Box((0.220, 0.052, 0.230)),
        origin=Origin(xyz=(0.185, 0.040, 0.46)),
        material=relay_black,
        name="relay_block_left",
    )
    panel.visual(
        Box((0.220, 0.052, 0.230)),
        origin=Origin(xyz=(0.450, 0.040, 0.46)),
        material=relay_black,
        name="relay_block_right",
    )
    panel.visual(
        Box((0.160, 0.045, 0.160)),
        origin=Origin(xyz=(0.195, 0.0365, 0.10)),
        material=display_black,
        name="meter_left",
    )
    panel.visual(
        Box((0.160, 0.045, 0.160)),
        origin=Origin(xyz=(0.392, 0.0365, 0.10)),
        material=display_black,
        name="meter_right",
    )
    panel.visual(
        Box((0.340, 0.026, 0.090)),
        origin=Origin(xyz=(0.360, 0.026, -0.17)),
        material=display_black,
        name="annunciator_strip",
    )
    panel.visual(
        Box((0.420, 0.022, 0.090)),
        origin=Origin(xyz=(0.340, 0.024, -0.55)),
        material=copper,
        name="terminal_block",
    )
    for idx, (lamp_x, lamp_material, lamp_name) in enumerate(
        (
            (0.135, signal_red, "trip_lamp"),
            (0.185, signal_amber, "alarm_lamp"),
            (0.235, signal_green, "healthy_lamp"),
        )
    ):
        panel.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(
                xyz=(lamp_x, 0.023, -0.34),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=lamp_material,
            name=lamp_name,
        )
    for switch_index, switch_x in enumerate((0.360, 0.430, 0.500)):
        panel.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(
                xyz=(switch_x, 0.024, -0.34),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=handle_black,
            name=f"selector_switch_{switch_index + 1}",
        )
    for hinge_name, hinge_z in (("upper_panel", 0.45), ("lower_panel", -0.45)):
        panel.visual(
            Box((0.010, 0.018, 0.110)),
            origin=Origin(xyz=(panel_hinge_radius * 0.5, 0.009, hinge_z)),
            material=hinge_dark,
            name=f"{hinge_name}_hinge_leaf",
        )
        panel.visual(
            Cylinder(radius=panel_hinge_radius, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=hinge_dark,
            name=f"{hinge_name}_hinge_knuckle",
        )
    panel.inertial = Inertial.from_geometry(
        Box((panel_width, 0.060, panel_height)),
        mass=26.0,
        origin=Origin(xyz=(panel_hinge_radius + panel_width * 0.5, 0.030, 0.0)),
    )

    model.articulation(
        "cabinet_to_front_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, cabinet_height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "cabinet_to_instrument_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=panel,
        origin=Origin(xyz=(panel_hinge_x, panel_hinge_y, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("front_door")
    panel = object_model.get_part("instrument_panel")
    door_hinge = object_model.get_articulation("cabinet_to_front_door")
    panel_hinge = object_model.get_articulation("cabinet_to_instrument_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "panel_hinge_axis_is_vertical",
        tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={panel_hinge.axis}",
    )

    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.70,
        name="door_covers_cabinet_opening",
    )
    ctx.expect_overlap(
        panel,
        cabinet,
        axes="xz",
        min_overlap=0.60,
        name="instrument_panel_spans_internal_mounting_zone",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="front_left_jamb",
        elem_b="door_left_return",
        name="door_left_return_contacts_left_jamb",
    )
    ctx.expect_contact(
        cabinet,
        panel,
        elem_a="upper_panel_hinge_barrel",
        elem_b="upper_panel_hinge_knuckle",
        name="instrument_panel_hinge_barrel_contacts_upper_knuckle",
    )
    ctx.expect_gap(
        door,
        panel,
        axis="y",
        min_gap=0.05,
        name="closed_door_clears_instrument_panel",
    )
    ctx.expect_within(
        panel,
        cabinet,
        axes="xz",
        margin=0.0,
        name="instrument_panel_sits_inside_cabinet_opening",
    )

    with ctx.pose(
        {
            door_hinge: math.radians(100.0),
            panel_hinge: math.radians(95.0),
        }
    ):
        ctx.expect_gap(
            panel,
            door,
            axis="x",
            min_gap=0.02,
            positive_elem="panel_face",
            negative_elem="door_shell_front",
            name="open_panel_clears_open_door",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
