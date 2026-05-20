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
    model = ArticulatedObject(name="bar_mounted_casino_machine")

    cabinet_dark = model.material("cabinet_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    cabinet_trim = model.material("cabinet_trim", rgba=(0.52, 0.54, 0.58, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.18, 0.22, 0.72))
    button_red = model.material("button_red", rgba=(0.78, 0.18, 0.16, 1.0))
    button_amber = model.material("button_amber", rgba=(0.84, 0.58, 0.16, 1.0))
    button_blue = model.material("button_blue", rgba=(0.18, 0.42, 0.82, 1.0))
    button_green = model.material("button_green", rgba=(0.20, 0.66, 0.28, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))

    body = model.part("machine_body")

    # Bar mount and pedestal.
    body.visual(
        Box((0.180, 0.130, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=steel,
        name="mount_base",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.140),
        origin=Origin(xyz=(0.000, 0.000, 0.084)),
        material=steel,
        name="pedestal_column",
    )
    body.visual(
        Box((0.085, 0.080, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.161)),
        material=steel,
        name="pedestal_cap",
    )

    # Cabinet shell.
    body.visual(
        Box((0.180, 0.012, 0.560)),
        origin=Origin(xyz=(0.000, -0.174, 0.434)),
        material=cabinet_dark,
        name="left_side_wall",
    )
    body.visual(
        Box((0.180, 0.012, 0.560)),
        origin=Origin(xyz=(0.000, 0.174, 0.434)),
        material=cabinet_dark,
        name="right_side_wall",
    )
    body.visual(
        Box((0.170, 0.336, 0.016)),
        origin=Origin(xyz=(-0.005, 0.000, 0.162)),
        material=cabinet_dark,
        name="bottom_shell",
    )
    body.visual(
        Box((0.150, 0.336, 0.016)),
        origin=Origin(xyz=(-0.015, 0.000, 0.706)),
        material=cabinet_dark,
        name="top_shell",
    )
    body.visual(
        Box((0.086, 0.336, 0.050)),
        origin=Origin(xyz=(0.048, 0.000, 0.674), rpy=(0.0, -0.28, 0.0)),
        material=cabinet_dark,
        name="front_hood",
    )

    # Front face and screen bezel.
    body.visual(
        Box((0.014, 0.300, 0.083)),
        origin=Origin(xyz=(0.083, 0.000, 0.2555)),
        material=cabinet_dark,
        name="front_lower_panel",
    )
    body.visual(
        Box((0.014, 0.336, 0.050)),
        origin=Origin(xyz=(0.083, 0.000, 0.322)),
        material=cabinet_trim,
        name="screen_lower_bezel",
    )
    body.visual(
        Box((0.014, 0.336, 0.060)),
        origin=Origin(xyz=(0.081, 0.000, 0.618)),
        material=cabinet_trim,
        name="screen_upper_bezel",
    )
    body.visual(
        Box((0.014, 0.042, 0.302)),
        origin=Origin(xyz=(0.081, -0.131, 0.470)),
        material=cabinet_trim,
        name="screen_left_bezel",
    )
    body.visual(
        Box((0.014, 0.042, 0.302)),
        origin=Origin(xyz=(0.081, 0.131, 0.470)),
        material=cabinet_trim,
        name="screen_right_bezel",
    )
    body.visual(
        Box((0.010, 0.218, 0.280)),
        origin=Origin(xyz=(0.076, 0.000, 0.470)),
        material=screen_glass,
        name="screen_panel",
    )

    # Button shelf and coin tray housing.
    body.visual(
        Box((0.110, 0.280, 0.018)),
        origin=Origin(xyz=(0.118, 0.000, 0.245)),
        material=cabinet_trim,
        name="shelf_deck",
    )
    body.visual(
        Box((0.076, 0.012, 0.045)),
        origin=Origin(xyz=(0.132, -0.146, 0.2135)),
        material=cabinet_dark,
        name="shelf_left_cheek",
    )
    body.visual(
        Box((0.076, 0.012, 0.045)),
        origin=Origin(xyz=(0.132, 0.146, 0.2135)),
        material=cabinet_dark,
        name="shelf_right_cheek",
    )
    body.visual(
        Box((0.066, 0.160, 0.010)),
        origin=Origin(xyz=(0.129, 0.000, 0.186)),
        material=tray_dark,
        name="tray_floor",
    )
    body.visual(
        Box((0.070, 0.010, 0.055)),
        origin=Origin(xyz=(0.132, -0.085, 0.2085)),
        material=tray_dark,
        name="tray_left_wall",
    )
    body.visual(
        Box((0.070, 0.010, 0.055)),
        origin=Origin(xyz=(0.132, 0.085, 0.2085)),
        material=tray_dark,
        name="tray_right_wall",
    )
    body.visual(
        Box((0.008, 0.160, 0.050)),
        origin=Origin(xyz=(0.100, 0.000, 0.206)),
        material=tray_dark,
        name="tray_back_wall",
    )
    body.visual(
        Box((0.014, 0.280, 0.022)),
        origin=Origin(xyz=(0.166, 0.000, 0.225)),
        material=cabinet_dark,
        name="tray_hinge_lip",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.142, 0.149, 0.224), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cabinet_trim,
        name="selector_boss",
    )

    for index, (y_pos, material) in enumerate(
        (
            (-0.090, button_red),
            (-0.030, button_amber),
            (0.030, button_blue),
            (0.090, button_green),
        ),
        start=1,
    ):
        body.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.124, y_pos, 0.259)),
            material=material,
            name=f"shelf_button_{index}",
        )

    # Rear frame with a service opening and internal stops.
    body.visual(
        Box((0.012, 0.312, 0.050)),
        origin=Origin(xyz=(-0.084, 0.000, 0.192)),
        material=cabinet_dark,
        name="rear_bottom_rail",
    )
    body.visual(
        Box((0.012, 0.312, 0.070)),
        origin=Origin(xyz=(-0.084, 0.000, 0.679)),
        material=cabinet_dark,
        name="rear_top_rail",
    )
    body.visual(
        Box((0.012, 0.024, 0.487)),
        origin=Origin(xyz=(-0.084, -0.144, 0.4355)),
        material=cabinet_dark,
        name="rear_left_stile",
    )
    body.visual(
        Box((0.012, 0.024, 0.487)),
        origin=Origin(xyz=(-0.084, 0.144, 0.4355)),
        material=cabinet_dark,
        name="rear_right_stile",
    )
    body.visual(
        Box((0.004, 0.272, 0.010)),
        origin=Origin(xyz=(-0.076, 0.000, 0.223)),
        material=cabinet_trim,
        name="rear_stop_bottom",
    )
    body.visual(
        Box((0.004, 0.272, 0.010)),
        origin=Origin(xyz=(-0.076, 0.000, 0.638)),
        material=cabinet_trim,
        name="rear_stop_top",
    )
    body.visual(
        Box((0.004, 0.010, 0.415)),
        origin=Origin(xyz=(-0.076, -0.131, 0.4305)),
        material=cabinet_trim,
        name="rear_stop_left",
    )
    body.visual(
        Box((0.004, 0.010, 0.415)),
        origin=Origin(xyz=(-0.076, 0.131, 0.4305)),
        material=cabinet_trim,
        name="rear_stop_right",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.300, 0.360, 0.714)),
        mass=15.0,
        origin=Origin(xyz=(0.020, 0.000, 0.357)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.000, 0.019, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.000, 0.030, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cabinet_trim,
        name="knob_cap",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.038, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.000, 0.019, 0.000)),
    )

    tray_flap = model.part("coin_tray_flap")
    tray_flap.visual(
        Box((0.008, 0.168, 0.072)),
        origin=Origin(xyz=(-0.004, 0.000, -0.036)),
        material=cabinet_trim,
        name="flap_panel",
    )
    tray_flap.visual(
        Box((0.014, 0.100, 0.008)),
        origin=Origin(xyz=(-0.011, 0.000, -0.066)),
        material=steel,
        name="flap_pull",
    )
    tray_flap.inertial = Inertial.from_geometry(
        Box((0.018, 0.168, 0.076)),
        mass=0.30,
        origin=Origin(xyz=(-0.009, 0.000, -0.038)),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.010, 0.264, 0.422)),
        origin=Origin(xyz=(-0.005, 0.132, 0.211)),
        material=cabinet_trim,
        name="panel_slab",
    )
    service_panel.visual(
        Box((0.018, 0.026, 0.075)),
        origin=Origin(xyz=(-0.014, 0.244, 0.211)),
        material=steel,
        name="panel_pull",
    )
    service_panel.inertial = Inertial.from_geometry(
        Box((0.024, 0.264, 0.422)),
        mass=0.85,
        origin=Origin(xyz=(-0.009, 0.132, 0.211)),
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.142, 0.152, 0.224)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    model.articulation(
        "body_to_coin_tray_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray_flap,
        origin=Origin(xyz=(0.168, 0.000, 0.214)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(-0.078, -0.132, 0.219)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("machine_body")
    selector_knob = object_model.get_part("selector_knob")
    tray_flap = object_model.get_part("coin_tray_flap")
    service_panel = object_model.get_part("service_panel")

    knob_joint = object_model.get_articulation("body_to_selector_knob")
    tray_joint = object_model.get_articulation("body_to_coin_tray_flap")
    service_joint = object_model.get_articulation("body_to_service_panel")

    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.joint_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"joint_type={knob_joint.joint_type}, limits={knob_joint.motion_limits}",
    )
    ctx.check(
        "tray flap and service panel are hinged",
        tray_joint.joint_type == ArticulationType.REVOLUTE
        and service_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"tray={tray_joint.joint_type}, service={service_joint.joint_type}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        (min_x, min_y, min_z), (max_x, max_y, max_z) = body_aabb
        ctx.check(
            "machine body keeps compact bar-top proportions",
            0.24 <= (max_x - min_x) <= 0.30
            and 0.32 <= (max_y - min_y) <= 0.38
            and 0.68 <= (max_z - min_z) <= 0.74,
            details=f"aabb={body_aabb}",
        )
    else:
        ctx.fail("machine body aabb resolves", "machine_body has no world aabb")

    with ctx.pose():
        ctx.expect_contact(
            selector_knob,
            body,
            elem_a="knob_shaft",
            elem_b="selector_boss",
            name="selector knob is mounted to the shelf boss",
        )
        ctx.expect_gap(
            body,
            tray_flap,
            axis="z",
            positive_elem="tray_hinge_lip",
            negative_elem="flap_panel",
            min_gap=0.0,
            max_gap=0.002,
            name="coin tray flap closes just below the shelf lip",
        )
        ctx.expect_gap(
            body,
            service_panel,
            axis="x",
            positive_elem="rear_stop_top",
            negative_elem="panel_slab",
            min_gap=0.0,
            max_gap=0.004,
            name="service panel rests against the rear stops",
        )

    closed_flap_aabb = ctx.part_world_aabb(tray_flap)
    with ctx.pose({tray_joint: 1.05}):
        open_flap_aabb = ctx.part_world_aabb(tray_flap)
    ctx.check(
        "coin tray flap swings outward from the shelf lip",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.025
        and open_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.015,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    closed_panel_aabb = ctx.part_world_aabb(service_panel)
    with ctx.pose({service_joint: 1.2}):
        open_panel_aabb = ctx.part_world_aabb(service_panel)
    ctx.check(
        "service panel swings out from the rear opening",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][0] < closed_panel_aabb[0][0] - 0.040,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
