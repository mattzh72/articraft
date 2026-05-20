from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_WIDTH = 0.46
BODY_DEPTH = 0.38
BODY_HEIGHT = 1.58
WALL = 0.02
FRONT_Y = BODY_DEPTH * 0.5 - WALL * 0.5


def _coil_mesh(name: str, *, radius: float, length: float, wire_radius: float, turns: float):
    samples = max(72, int(turns * 30))
    points = []
    for i in range(samples + 1):
        t = i / samples
        angle = turns * 2.0 * pi * t
        points.append(
            (
                radius * cos(angle),
                -length * 0.5 + length * t,
                radius * sin(angle),
            )
        )

    coil_geom = tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=1,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    shaft_radius = wire_radius * 1.45
    shaft_geom = CylinderGeometry(shaft_radius, length + 0.03, radial_segments=18, closed=True).rotate_x(-pi / 2.0)
    rear_spoke = tube_from_spline_points(
        [(shaft_radius, -length * 0.5, 0.0), (radius * 0.98, -length * 0.5, 0.0)],
        radius=wire_radius * 0.72,
        samples_per_segment=1,
        radial_segments=12,
        cap_ends=True,
    )
    front_spoke = tube_from_spline_points(
        [(shaft_radius, length * 0.5, 0.0), (radius * 0.98, length * 0.5, 0.0)],
        radius=wire_radius * 0.72,
        samples_per_segment=1,
        radial_segments=12,
        cap_ends=True,
    )
    coil_geom.merge(shaft_geom)
    coil_geom.merge(rear_spoke)
    coil_geom.merge(front_spoke)
    return mesh_from_geometry(coil_geom, name)


def _add_spiral_stage(
    model: ArticulatedObject,
    *,
    name: str,
    mount_xyz: tuple[float, float, float],
    steel,
    tray_gray,
    candy_color,
) -> None:
    tray_width = 0.275
    tray_depth = 0.236
    tray_thickness = 0.006
    rail_thickness = 0.006
    rail_height = 0.020
    support_width = 0.030
    support_depth = 0.012
    support_height = 0.060
    coil_length = 0.206
    coil_radius = 0.044
    coil_wire = 0.0052

    stage = model.part(name)
    stage.visual(
        Box((tray_width, tray_depth, tray_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=tray_gray,
        name="tray_floor",
    )
    stage.visual(
        Box((tray_width, rail_thickness, 0.024)),
        origin=Origin(xyz=(0.0, tray_depth * 0.5 - rail_thickness * 0.5, -0.046)),
        material=tray_gray,
        name="front_lip",
    )
    stage.visual(
        Box((tray_width, rail_thickness, 0.028)),
        origin=Origin(xyz=(0.0, -tray_depth * 0.5 + rail_thickness * 0.5, -0.044)),
        material=tray_gray,
        name="rear_lip",
    )
    stage.visual(
        Box((rail_thickness, tray_depth, rail_height)),
        origin=Origin(xyz=(-tray_width * 0.5 + rail_thickness * 0.5, 0.0, -0.046)),
        material=tray_gray,
        name="left_rail",
    )
    stage.visual(
        Box((rail_thickness, tray_depth, rail_height)),
        origin=Origin(xyz=(tray_width * 0.5 - rail_thickness * 0.5, 0.0, -0.046)),
        material=tray_gray,
        name="right_rail",
    )
    stage.visual(
        Box((support_width, support_depth, support_height)),
        origin=Origin(xyz=(0.0, -coil_length * 0.5 - 0.010, -0.028)),
        material=tray_gray,
        name="rear_support",
    )
    stage.visual(
        Box((support_width, support_depth, support_height)),
        origin=Origin(xyz=(0.0, coil_length * 0.5 + 0.010, -0.028)),
        material=tray_gray,
        name="front_support",
    )
    stage.visual(
        _coil_mesh(
            f"{name}_coil",
            radius=coil_radius,
            length=coil_length,
            wire_radius=coil_wire,
            turns=3.7,
        ),
        material=steel,
        name="spiral_assembly",
    )
    stage.visual(
        Box((0.042, 0.028, 0.020)),
        origin=Origin(xyz=(-0.078, -0.070, -0.046)),
        material=candy_color,
        name="candy_box_left",
    )
    stage.visual(
        Box((0.040, 0.026, 0.018)),
        origin=Origin(xyz=(0.012, 0.000, -0.047)),
        material=candy_color,
        name="candy_box_mid",
    )
    stage.visual(
        Box((0.040, 0.028, 0.018)),
        origin=Origin(xyz=(0.086, 0.070, -0.046)),
        material=candy_color,
        name="candy_box_right",
    )
    stage.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth, 0.130)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        f"{name}_mount",
        ArticulationType.FIXED,
        parent="cabinet",
        child=stage,
        origin=Origin(xyz=mount_xyz),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_candy_vending_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.73, 0.12, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.38, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.44, 0.45, 0.48, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.60, 0.82, 0.93, 0.26))
    panel_black = model.material("panel_black", rgba=(0.12, 0.13, 0.14, 1.0))
    lcd_green = model.material("lcd_green", rgba=(0.24, 0.42, 0.26, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.16, 0.14, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.88, 0.71, 0.18, 1.0))
    candy_cyan = model.material("candy_cyan", rgba=(0.18, 0.72, 0.84, 1.0))
    candy_magenta = model.material("candy_magenta", rgba=(0.80, 0.31, 0.56, 1.0))
    candy_orange = model.material("candy_orange", rgba=(0.92, 0.49, 0.16, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((BODY_WIDTH, 0.090, 0.10)),
        origin=Origin(xyz=(0.0, -0.145, 0.05)),
        material=trim_black,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.030, 0.240, 0.10)),
        origin=Origin(xyz=(-0.215, -0.070, 0.05)),
        material=trim_black,
        name="left_base_leg",
    )
    cabinet.visual(
        Box((0.030, 0.240, 0.10)),
        origin=Origin(xyz=(0.215, -0.070, 0.05)),
        material=trim_black,
        name="right_base_leg",
    )
    cabinet.visual(
        Box((BODY_WIDTH, WALL, BODY_HEIGHT - 0.10)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + WALL * 0.5, 0.84)),
        material=cabinet_red,
        name="back_panel",
    )
    cabinet.visual(
        Box((WALL, BODY_DEPTH, BODY_HEIGHT - 0.10)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + WALL * 0.5, 0.0, 0.84)),
        material=cabinet_red,
        name="left_side",
    )
    cabinet.visual(
        Box((WALL, BODY_DEPTH, BODY_HEIGHT - 0.10)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - WALL * 0.5, 0.0, 0.84)),
        material=cabinet_red,
        name="right_side",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - WALL * 0.5)),
        material=cabinet_red,
        name="roof_panel",
    )
    cabinet.visual(
        Box((WALL, BODY_DEPTH - 0.04, 1.04)),
        origin=Origin(xyz=(0.085, -0.01, 1.05)),
        material=trim_black,
        name="bay_divider",
    )
    cabinet.visual(
        Box((0.025, 0.024, 0.94)),
        origin=Origin(xyz=(-0.207, FRONT_Y, 1.00)),
        material=trim_black,
        name="glass_left_frame",
    )
    cabinet.visual(
        Box((0.025, 0.024, 0.94)),
        origin=Origin(xyz=(0.072, FRONT_Y, 1.00)),
        material=trim_black,
        name="glass_right_frame",
    )
    cabinet.visual(
        Box((0.304, 0.024, 0.025)),
        origin=Origin(xyz=(-0.067, FRONT_Y, 1.457)),
        material=trim_black,
        name="glass_top_frame",
    )
    cabinet.visual(
        Box((0.304, 0.024, 0.025)),
        origin=Origin(xyz=(-0.067, FRONT_Y, 0.542)),
        material=trim_black,
        name="glass_bottom_frame",
    )
    cabinet.visual(
        Box((0.118, 0.024, 1.02)),
        origin=Origin(xyz=(0.151, FRONT_Y, 0.965)),
        material=trim_black,
        name="control_column_front",
    )
    cabinet.visual(
        Box((0.420, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, FRONT_Y, 0.172)),
        material=trim_black,
        name="drawer_header",
    )
    cabinet.visual(
        Box((0.252, 0.024, 0.026)),
        origin=Origin(xyz=(-0.070, FRONT_Y, 0.198)),
        material=trim_black,
        name="pickup_sill",
    )
    cabinet.visual(
        Box((0.252, 0.024, 0.026)),
        origin=Origin(xyz=(-0.070, FRONT_Y, 0.328)),
        material=trim_black,
        name="pickup_header",
    )
    cabinet.visual(
        Box((0.026, 0.024, 0.124)),
        origin=Origin(xyz=(-0.196, FRONT_Y, 0.266)),
        material=trim_black,
        name="pickup_left_jamb",
    )
    cabinet.visual(
        Box((0.026, 0.024, 0.124)),
        origin=Origin(xyz=(0.056, FRONT_Y, 0.266)),
        material=trim_black,
        name="pickup_right_jamb",
    )
    cabinet.visual(
        Box((0.252, 0.024, 0.190)),
        origin=Origin(xyz=(-0.070, FRONT_Y, 0.435)),
        material=cabinet_red,
        name="front_mid_panel",
    )
    cabinet.visual(
        Box((0.222, 0.190, 0.018)),
        origin=Origin(xyz=(-0.070, 0.085, 0.192)),
        material=tray_gray,
        name="pickup_chute_floor",
    )
    cabinet.visual(
        Box((0.220, 0.018, 0.108)),
        origin=Origin(xyz=(-0.070, 0.004, 0.255)),
        material=tray_gray,
        name="pickup_chute_back",
    )
    cabinet.visual(
        Box((0.018, 0.256, 0.010)),
        origin=Origin(xyz=(-0.160, -0.012, 0.102)),
        material=dark_steel,
        name="left_runner",
    )
    cabinet.visual(
        Box((0.018, 0.256, 0.010)),
        origin=Origin(xyz=(0.160, -0.012, 0.102)),
        material=dark_steel,
        name="right_runner",
    )
    cabinet.visual(
        Box((0.280, 0.060, 0.050)),
        origin=Origin(xyz=(-0.067, 0.105, 1.535)),
        material=steel,
        name="top_light_baffle",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    glass = model.part("glass_front")
    glass.visual(
        Box((0.276, 0.008, 0.882)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=smoked_glass,
        name="glass_panel",
    )
    glass.inertial = Inertial.from_geometry(Box((0.276, 0.008, 0.882)), mass=3.0)
    model.articulation(
        "cabinet_to_glass",
        ArticulationType.FIXED,
        parent=cabinet,
        child=glass,
        origin=Origin(xyz=(-0.067, 0.164, 0.995)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.110, 0.032, 0.292)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=panel_black,
        name="panel_housing",
    )
    control_panel.visual(
        Box((0.090, 0.006, 0.270)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=trim_black,
        name="panel_face",
    )
    control_panel.visual(
        Box((0.060, 0.004, 0.038)),
        origin=Origin(xyz=(0.0, 0.022, 0.094)),
        material=lcd_green,
        name="display_window",
    )
    control_panel.visual(
        Box((0.042, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.017, 0.146)),
        material=dark_steel,
        name="coin_slot",
    )
    control_panel.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.028, 0.022, -0.096), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=button_red,
        name="cancel_button",
    )
    control_panel.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.028, 0.022, -0.096), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=button_yellow,
        name="vend_button",
    )
    control_panel.visual(
        Box((0.050, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.022, -0.142)),
        material=dark_steel,
        name="return_tray",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.110, 0.032, 0.292)),
        mass=1.4,
        origin=Origin(),
    )
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(0.151, 0.208, 0.868)),
    )

    dial = model.part("selector_dial")
    dial.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="dial_face",
    )
    dial.visual(
        Box((0.008, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, 0.024, 0.024)),
        material=button_red,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.072, 0.032, 0.072)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
    )
    model.articulation(
        "control_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(0.0, 0.019, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=6.0),
    )

    drawer = model.part("cash_drawer")
    drawer.visual(
        Box((0.292, 0.224, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=tray_gray,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.010, 0.224, 0.050)),
        origin=Origin(xyz=(-0.141, 0.0, 0.053)),
        material=tray_gray,
        name="left_bin_wall",
    )
    drawer.visual(
        Box((0.010, 0.224, 0.050)),
        origin=Origin(xyz=(0.141, 0.0, 0.053)),
        material=tray_gray,
        name="right_bin_wall",
    )
    drawer.visual(
        Box((0.272, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.107, 0.053)),
        material=tray_gray,
        name="rear_bin_wall",
    )
    drawer.visual(
        Box((0.008, 0.140, 0.035)),
        origin=Origin(xyz=(0.0, -0.010, 0.045)),
        material=dark_steel,
        name="cash_divider",
    )
    drawer.visual(
        Box((0.394, 0.024, 0.124)),
        origin=Origin(xyz=(0.0, 0.122, 0.002)),
        material=dark_steel,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.112, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.142, 0.006)),
        material=steel,
        name="drawer_handle",
    )
    drawer.visual(
        Box((0.010, 0.178, 0.004)),
        origin=Origin(xyz=(-0.160, -0.012, 0.013)),
        material=dark_steel,
        name="left_tongue",
    )
    drawer.visual(
        Box((0.010, 0.178, 0.004)),
        origin=Origin(xyz=(0.160, -0.012, 0.013)),
        material=dark_steel,
        name="right_tongue",
    )
    drawer.visual(
        Box((0.020, 0.178, 0.020)),
        origin=Origin(xyz=(-0.150, -0.012, 0.023)),
        material=dark_steel,
        name="left_slide_bracket",
    )
    drawer.visual(
        Box((0.020, 0.178, 0.020)),
        origin=Origin(xyz=(0.150, -0.012, 0.023)),
        material=dark_steel,
        name="right_slide_bracket",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.404, 0.244, 0.124)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.012, 0.002)),
    )
    model.articulation(
        "cabinet_to_cash_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.038, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=0.130),
    )

    flap = model.part("retrieval_flap")
    flap.visual(
        Box((0.228, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, 0.006, -0.055)),
        material=panel_black,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.007, length=0.232),
        origin=Origin(xyz=(0.0, 0.007, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.232, 0.016, 0.114)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.008, -0.052)),
    )
    model.articulation(
        "cabinet_to_retrieval_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(-0.070, FRONT_Y + 0.012, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.20),
    )

    _add_spiral_stage(
        model,
        name="spiral_stage_bottom",
        mount_xyz=(-0.0625, -0.008, 0.640),
        steel=steel,
        tray_gray=tray_gray,
        candy_color=candy_cyan,
    )
    _add_spiral_stage(
        model,
        name="spiral_stage_lower_mid",
        mount_xyz=(-0.0625, -0.008, 0.825),
        steel=steel,
        tray_gray=tray_gray,
        candy_color=candy_magenta,
    )
    _add_spiral_stage(
        model,
        name="spiral_stage_upper_mid",
        mount_xyz=(-0.0625, -0.008, 1.010),
        steel=steel,
        tray_gray=tray_gray,
        candy_color=candy_orange,
    )
    _add_spiral_stage(
        model,
        name="spiral_stage_top",
        mount_xyz=(-0.0625, -0.008, 1.195),
        steel=steel,
        tray_gray=tray_gray,
        candy_color=candy_cyan,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    glass = object_model.get_part("glass_front")
    control_panel = object_model.get_part("control_panel")
    dial = object_model.get_part("selector_dial")
    drawer = object_model.get_part("cash_drawer")
    flap = object_model.get_part("retrieval_flap")
    stage_top = object_model.get_part("spiral_stage_top")

    drawer_joint = object_model.get_articulation("cabinet_to_cash_drawer")
    flap_joint = object_model.get_articulation("cabinet_to_retrieval_flap")
    dial_joint = object_model.get_articulation("control_panel_to_selector_dial")

    ctx.expect_gap(
        glass,
        stage_top,
        axis="y",
        min_gap=0.03,
        max_gap=0.24,
        positive_elem="glass_panel",
        negative_elem="spiral_assembly",
        name="top candy spiral sits behind the glass front",
    )

    ctx.expect_within(
        drawer,
        cabinet,
        axes="x",
        inner_elem="drawer_body",
        margin=0.03,
        name="cash drawer stays within cabinet width envelope",
    )
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="z",
        positive_elem="drawer_header",
        negative_elem="drawer_body",
        min_gap=0.008,
        max_gap=0.080,
        name="cash drawer fits below the pickup fascia",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        elem_a="left_tongue",
        elem_b="left_runner",
        min_overlap=0.030,
        name="left drawer tongue remains engaged on its runner at rest",
    )
    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.130}):
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="left_tongue",
            elem_b="left_runner",
            min_overlap=0.030,
            name="left drawer tongue retains insertion at full extension",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "cash drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.10,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "retrieval flap swings outward on the pickup hinge",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > rest_flap_aabb[1][1] + 0.07,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: pi / 2.0}):
        quarter_turn_pointer_aabb = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "selector dial rotates about its shaft",
        dial_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in dial_joint.axis) == (0.0, 1.0, 0.0)
        and rest_pointer_aabb is not None
        and quarter_turn_pointer_aabb is not None
        and abs(quarter_turn_pointer_aabb[1][0] - rest_pointer_aabb[1][0]) > 0.015,
        details=(
            f"joint_type={dial_joint.joint_type}, axis={dial_joint.axis}, "
            f"rest={rest_pointer_aabb}, rotated={quarter_turn_pointer_aabb}"
        ),
    )

    ctx.check(
        "stack includes four candy spiral stages",
        all(
            object_model.get_part(name) is not None
            for name in (
                "spiral_stage_bottom",
                "spiral_stage_lower_mid",
                "spiral_stage_upper_mid",
                "spiral_stage_top",
            )
        ),
        details="Expected four fixed dispenser stages behind the glass bay.",
    )
    ctx.check(
        "control panel mounted at mid height",
        0.70 < ctx.part_world_position(control_panel)[2] < 1.05,
        details=f"control_panel_position={ctx.part_world_position(control_panel)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
