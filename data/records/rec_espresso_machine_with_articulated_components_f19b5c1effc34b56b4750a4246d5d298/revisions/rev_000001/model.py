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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_espresso_machine")

    shell_dark = model.material("shell_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.10, 1.0))
    button_black = model.material("button_black", rgba=(0.05, 0.05, 0.06, 1.0))
    tray_black = model.material("tray_black", rgba=(0.08, 0.08, 0.09, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.65, 0.67, 0.70, 1.0))

    def yz_section(
        x_pos: float,
        width_y: float,
        height_z: float,
        corner_radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val + z_center)
            for y_val, z_val in rounded_rect_profile(width_y, height_z, corner_radius)
        ]

    body = model.part("body")

    upper_shell = section_loft(
        [
            yz_section(-0.135, 0.252, 0.240, 0.028, 0.255),
            yz_section(-0.015, 0.246, 0.224, 0.026, 0.255),
            yz_section(0.120, 0.210, 0.200, 0.024, 0.260),
        ]
    )
    body.visual(
        mesh_from_geometry(upper_shell, "upper_shell"),
        material=shell_dark,
        name="upper_shell",
    )
    body.visual(
        Box((0.340, 0.280, 0.018)),
        origin=Origin(xyz=(-0.010, 0.000, 0.009)),
        material=panel_dark,
        name="base_plinth",
    )
    body.visual(
        Box((0.300, 0.020, 0.158)),
        origin=Origin(xyz=(-0.020, -0.130, 0.079)),
        material=shell_dark,
        name="left_side_wall",
    )
    body.visual(
        Box((0.300, 0.020, 0.158)),
        origin=Origin(xyz=(-0.020, 0.130, 0.079)),
        material=shell_dark,
        name="right_side_wall",
    )
    body.visual(
        Box((0.020, 0.240, 0.158)),
        origin=Origin(xyz=(-0.170, 0.000, 0.079)),
        material=shell_dark,
        name="rear_wall",
    )
    body.visual(
        Box((0.132, 0.220, 0.018)),
        origin=Origin(xyz=(0.084, 0.000, 0.085)),
        material=shell_dark,
        name="tray_bay_roof",
    )
    body.visual(
        Box((0.115, 0.008, 0.012)),
        origin=Origin(xyz=(0.082, -0.096, 0.031)),
        material=panel_dark,
        name="left_tray_rail",
    )
    body.visual(
        Box((0.115, 0.008, 0.012)),
        origin=Origin(xyz=(0.082, 0.096, 0.031)),
        material=panel_dark,
        name="right_tray_rail",
    )
    body.visual(
        Box((0.035, 0.026, 0.122)),
        origin=Origin(xyz=(0.143, -0.111, 0.079)),
        material=shell_dark,
        name="front_column_0",
    )
    body.visual(
        Box((0.035, 0.026, 0.122)),
        origin=Origin(xyz=(0.143, 0.111, 0.079)),
        material=shell_dark,
        name="front_column_1",
    )
    body.visual(
        Box((0.020, 0.128, 0.104)),
        origin=Origin(xyz=(0.134, 0.000, 0.112)),
        material=shell_dark,
        name="front_spine",
    )
    body.visual(
        Box((0.024, 0.190, 0.060)),
        origin=Origin(xyz=(0.136, 0.000, 0.148)),
        material=panel_dark,
        name="button_panel",
    )
    body.visual(
        Box((0.008, 0.110, 0.004)),
        origin=Origin(xyz=(0.151, 0.000, 0.149)),
        material=panel_dark,
        name="button_bezel_bottom",
    )
    body.visual(
        Box((0.008, 0.110, 0.004)),
        origin=Origin(xyz=(0.151, 0.000, 0.161)),
        material=panel_dark,
        name="button_bezel_top",
    )
    body.visual(
        Box((0.008, 0.006, 0.016)),
        origin=Origin(xyz=(0.151, -0.053, 0.155)),
        material=panel_dark,
        name="button_bezel_end_0",
    )
    body.visual(
        Box((0.008, 0.006, 0.016)),
        origin=Origin(xyz=(0.151, -0.017, 0.155)),
        material=panel_dark,
        name="button_bezel_sep_0",
    )
    body.visual(
        Box((0.008, 0.006, 0.016)),
        origin=Origin(xyz=(0.151, 0.017, 0.155)),
        material=panel_dark,
        name="button_bezel_sep_1",
    )
    body.visual(
        Box((0.008, 0.006, 0.016)),
        origin=Origin(xyz=(0.151, 0.053, 0.155)),
        material=panel_dark,
        name="button_bezel_end_1",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.052),
        origin=Origin(xyz=(0.122, 0.000, 0.232), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=metal,
        name="group_head_body",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.146, 0.000, 0.208)),
        material=metal,
        name="group_head_collar",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.072, 0.137, 0.286)),
        material=metal,
        name="steam_pivot_boss",
    )
    body.visual(
        Box((0.048, 0.028, 0.024)),
        origin=Origin(xyz=(0.068, 0.118, 0.286)),
        material=shell_dark,
        name="steam_support_bracket",
    )
    body.visual(
        Box((0.014, 0.096, 0.012)),
        origin=Origin(xyz=(0.149, -0.050, 0.062)),
        material=metal,
        name="lower_platform_mount",
    )
    body.visual(
        Box((0.013, 0.074, 0.018)),
        origin=Origin(xyz=(0.1565, 0.090, 0.100)),
        material=metal,
        name="upper_platform_mount",
    )
    body.visual(
        Box((0.008, 0.014, 0.018)),
        origin=Origin(xyz=(0.159, 0.052, 0.100)),
        material=metal,
        name="upper_platform_cheek_0",
    )
    body.visual(
        Box((0.008, 0.014, 0.018)),
        origin=Origin(xyz=(0.159, 0.128, 0.100)),
        material=metal,
        name="upper_platform_cheek_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.280, 0.380)),
        mass=8.0,
        origin=Origin(xyz=(-0.010, 0.000, 0.190)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=metal,
        name="basket_ring",
    )
    portafilter.visual(
        Box((0.018, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.023)),
        material=metal,
        name="spout_block",
    )
    portafilter.visual(
        Box((0.030, 0.018, 0.016)),
        origin=Origin(xyz=(0.018, 0.000, -0.014)),
        material=metal,
        name="handle_neck",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.093, 0.000, -0.020), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=handle_black,
        name="handle",
    )
    portafilter.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(xyz=(0.000, -0.024, -0.004)),
        material=metal,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(xyz=(0.000, 0.024, -0.004)),
        material=metal,
        name="lug_1",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.150, 0.060, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.060, 0.000, -0.015)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.176, 0.000, 0.198)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.15,
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
        material=metal,
        name="pivot_sleeve",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.016, 0.010, -0.024), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=metal,
        name="arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.004, length=0.138),
        origin=Origin(xyz=(0.030, 0.013, -0.093)),
        material=metal,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0025, length=0.016),
        origin=Origin(xyz=(0.036, 0.013, -0.162), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=metal,
        name="tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.050, 0.030, 0.170)),
        mass=0.18,
        origin=Origin(xyz=(0.016, 0.008, -0.080)),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.072, 0.137, 0.286)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.5,
            lower=-1.10,
            upper=0.70,
        ),
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        Cylinder(radius=0.004, length=0.092),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=metal,
        name="hinge_barrel",
    )
    lower_platform.visual(
        Box((0.008, 0.092, 0.080)),
        origin=Origin(xyz=(0.004, 0.000, 0.040)),
        material=tray_metal,
        name="platform_plate",
    )
    lower_platform.visual(
        Box((0.014, 0.086, 0.010)),
        origin=Origin(xyz=(0.011, 0.000, 0.075)),
        material=metal,
        name="front_lip",
    )
    lower_platform.inertial = Inertial.from_geometry(
        Box((0.018, 0.092, 0.082)),
        mass=0.09,
        origin=Origin(xyz=(0.008, 0.000, 0.041)),
    )
    model.articulation(
        "body_to_lower_platform",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_platform,
        origin=Origin(xyz=(0.159, -0.050, 0.062)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=1.4,
            velocity=2.0,
            lower=0.00,
            upper=1.52,
        ),
    )

    upper_platform = model.part("upper_platform")
    upper_platform.visual(
        Cylinder(radius=0.004, length=0.070),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=metal,
        name="hinge_barrel",
    )
    upper_platform.visual(
        Box((0.008, 0.070, 0.060)),
        origin=Origin(xyz=(0.004, 0.000, 0.030)),
        material=tray_metal,
        name="platform_plate",
    )
    upper_platform.visual(
        Box((0.014, 0.064, 0.010)),
        origin=Origin(xyz=(0.011, 0.000, 0.055)),
        material=metal,
        name="front_lip",
    )
    upper_platform.inertial = Inertial.from_geometry(
        Box((0.018, 0.070, 0.062)),
        mass=0.07,
        origin=Origin(xyz=(0.008, 0.000, 0.031)),
    )
    model.articulation(
        "body_to_upper_platform",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_platform,
        origin=Origin(xyz=(0.167, 0.090, 0.100)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=0.00,
            upper=1.52,
        ),
    )

    button_y_positions = (-0.034, 0.000, 0.034)
    for index, y_pos in enumerate(button_y_positions):
        body.visual(
            Box((0.010, 0.003, 0.008)),
            origin=Origin(xyz=(0.152, y_pos - 0.0065, 0.155)),
            material=panel_dark,
            name=f"button_guide_{index}_0",
        )
        body.visual(
            Box((0.010, 0.003, 0.008)),
            origin=Origin(xyz=(0.152, y_pos + 0.0065, 0.155)),
            material=panel_dark,
            name=f"button_guide_{index}_1",
        )
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.006, 0.010, 0.005)),
            origin=Origin(xyz=(0.000, 0.000, 0.0025)),
            material=panel_dark,
            name="plunger",
        )
        button.visual(
            Box((0.010, 0.022, 0.007)),
            origin=Origin(xyz=(0.006, 0.000, 0.0035)),
            material=button_black,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.014, 0.022, 0.007)),
            mass=0.012,
            origin=Origin(xyz=(0.005, 0.000, 0.0035)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.1570, y_pos, 0.152)),
            axis=(-1.000, 0.000, 0.000),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.000,
                upper=0.003,
            ),
        )

    tray_grate = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.108, 0.170),
            0.003,
            slot_size=(0.020, 0.004),
            pitch=(0.028, 0.012),
            frame=0.010,
            corner_radius=0.004,
            slot_angle_deg=0.0,
            stagger=True,
        ),
        "drip_tray_grate",
    )
    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.126, 0.188, 0.018)),
        origin=Origin(xyz=(0.033, 0.000, 0.011)),
        material=tray_black,
        name="tray_pan",
    )
    drip_tray.visual(
        tray_grate,
        origin=Origin(xyz=(0.030, 0.000, 0.0205)),
        material=tray_metal,
        name="tray_grate",
    )
    drip_tray.visual(
        Box((0.008, 0.156, 0.016)),
        origin=Origin(xyz=(0.100, 0.000, 0.008)),
        material=tray_black,
        name="tray_handle",
    )
    drip_tray.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(-0.041, -0.074, 0.006)),
        material=tray_black,
        name="guide_0",
    )
    drip_tray.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(-0.041, 0.074, 0.006)),
        material=tray_black,
        name="guide_1",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.148, 0.188, 0.024)),
        mass=0.48,
        origin=Origin(xyz=(0.024, 0.000, 0.012)),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.060, 0.000, 0.020)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.000,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    lower_platform = object_model.get_part("lower_platform")
    upper_platform = object_model.get_part("upper_platform")
    drip_tray = object_model.get_part("drip_tray")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    lower_platform_joint = object_model.get_articulation("body_to_lower_platform")
    upper_platform_joint = object_model.get_articulation("body_to_upper_platform")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    def aabb_dims(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple(upper[index] - lower[index] for index in range(3))

    rest_handle = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: -0.60}):
        unlocked_handle = ctx.part_element_world_aabb(portafilter, elem="handle")
    rest_handle_center = aabb_center(rest_handle)
    unlocked_handle_center = aabb_center(unlocked_handle)
    ctx.check(
        "portafilter rotates sideways under the group head",
        rest_handle_center is not None
        and unlocked_handle_center is not None
        and unlocked_handle_center[1] < rest_handle_center[1] - 0.045,
        details=f"rest={rest_handle_center}, unlocked={unlocked_handle_center}",
    )

    rest_tip = ctx.part_element_world_aabb(steam_wand, elem="tip")
    with ctx.pose({steam_joint: -0.90}):
        swung_tip = ctx.part_element_world_aabb(steam_wand, elem="tip")
    rest_tip_center = aabb_center(rest_tip)
    swung_tip_center = aabb_center(swung_tip)
    ctx.check(
        "steam wand swings around its side pivot",
        rest_tip_center is not None
        and swung_tip_center is not None
        and abs(swung_tip_center[1] - rest_tip_center[1]) > 0.020,
        details=f"rest={rest_tip_center}, swung={swung_tip_center}",
    )

    lower_closed = ctx.part_element_world_aabb(lower_platform, elem="platform_plate")
    with ctx.pose({lower_platform_joint: 1.45}):
        lower_open = ctx.part_element_world_aabb(lower_platform, elem="platform_plate")
    lower_closed_dims = aabb_dims(lower_closed)
    lower_open_dims = aabb_dims(lower_open)
    ctx.check(
        "lower cup platform flips down into a shelf",
        lower_closed_dims is not None
        and lower_open_dims is not None
        and lower_closed_dims[2] > 0.070
        and lower_closed_dims[0] < 0.012
        and lower_open_dims[0] > 0.070
        and lower_open_dims[2] < 0.020,
        details=f"closed={lower_closed_dims}, open={lower_open_dims}",
    )

    upper_closed = ctx.part_element_world_aabb(upper_platform, elem="platform_plate")
    with ctx.pose({upper_platform_joint: 1.45}):
        upper_open = ctx.part_element_world_aabb(upper_platform, elem="platform_plate")
    upper_closed_dims = aabb_dims(upper_closed)
    upper_open_dims = aabb_dims(upper_open)
    ctx.check(
        "upper cup platform flips down into a shelf",
        upper_closed_dims is not None
        and upper_open_dims is not None
        and upper_closed_dims[2] > 0.050
        and upper_closed_dims[0] < 0.012
        and upper_open_dims[0] > 0.050
        and upper_open_dims[2] < 0.020,
        details=f"closed={upper_closed_dims}, open={upper_open_dims}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_cap = ctx.part_element_world_aabb(button, elem="cap")
        with ctx.pose({button_joint: 0.003}):
            pressed_cap = ctx.part_element_world_aabb(button, elem="cap")
        rest_cap_center = aabb_center(rest_cap)
        pressed_cap_center = aabb_center(pressed_cap)
        ctx.check(
            f"button {index} presses inward on a short plunger",
            rest_cap_center is not None
            and pressed_cap_center is not None
            and pressed_cap_center[0] < rest_cap_center[0] - 0.0025,
            details=f"rest={rest_cap_center}, pressed={pressed_cap_center}",
        )

    ctx.expect_within(
        drip_tray,
        body,
        axes="y",
        inner_elem="tray_pan",
        outer_elem="tray_bay_roof",
        margin=0.016,
        name="drip tray stays laterally within the bay",
    )
    tray_rest = ctx.part_element_world_aabb(drip_tray, elem="tray_handle")
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            elem_a="tray_pan",
            elem_b="tray_bay_roof",
            min_overlap=0.045,
            name="extended drip tray retains insertion in the machine",
        )
        tray_extended = ctx.part_element_world_aabb(drip_tray, elem="tray_handle")
    tray_rest_center = aabb_center(tray_rest)
    tray_extended_center = aabb_center(tray_extended)
    ctx.check(
        "drip tray slides forward on the base guide",
        tray_rest_center is not None
        and tray_extended_center is not None
        and tray_extended_center[0] > tray_rest_center[0] + 0.060,
        details=f"rest={tray_rest_center}, extended={tray_extended_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
