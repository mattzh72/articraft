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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_top_slot_machine")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.56, 0.06, 0.10, 1.0))
    black_trim = model.material("black_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.79, 0.83, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.14, 0.16, 0.19, 0.30))
    display_black = model.material("display_black", rgba=(0.03, 0.04, 0.05, 1.0))
    reel_white = model.material("reel_white", rgba=(0.95, 0.95, 0.93, 1.0))
    reel_gold = model.material("reel_gold", rgba=(0.86, 0.73, 0.22, 1.0))
    reel_blue = model.material("reel_blue", rgba=(0.19, 0.41, 0.78, 1.0))
    reel_red = model.material("reel_red", rgba=(0.78, 0.18, 0.16, 1.0))
    knob_red = model.material("knob_red", rgba=(0.76, 0.12, 0.14, 1.0))

    cabinet = model.part("cabinet")
    lower_width = 0.60
    lower_depth = 0.68
    upper_width = 0.76
    upper_depth = 0.74

    cabinet.visual(
        Box((0.62, 0.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=black_trim,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.05, lower_depth, 0.86)),
        origin=Origin(xyz=(-0.275, 0.0, 0.51)),
        material=cabinet_paint,
        name="lower_left_side",
    )
    cabinet.visual(
        Box((0.05, lower_depth, 0.86)),
        origin=Origin(xyz=(0.275, 0.0, 0.51)),
        material=cabinet_paint,
        name="lower_right_side",
    )
    cabinet.visual(
        Box((0.50, 0.04, 0.86)),
        origin=Origin(xyz=(0.0, 0.32, 0.51)),
        material=cabinet_paint,
        name="lower_back",
    )
    cabinet.visual(
        Box((0.32, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, -0.26, 0.10)),
        material=black_trim,
        name="door_bottom_rail",
    )
    cabinet.visual(
        Box((0.10, 0.16, 0.54)),
        origin=Origin(xyz=(-0.24, -0.26, 0.35)),
        material=black_trim,
        name="door_left_stile",
    )
    cabinet.visual(
        Box((0.10, 0.16, 0.54)),
        origin=Origin(xyz=(0.24, -0.26, 0.35)),
        material=black_trim,
        name="door_right_stile",
    )
    cabinet.visual(
        Box((0.32, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, -0.26, 0.60)),
        material=black_trim,
        name="door_top_rail",
    )
    cabinet.visual(
        Box((0.34, 0.10, 0.44)),
        origin=Origin(xyz=(0.0, -0.19, 0.34)),
        material=dark_metal,
        name="service_bay_back",
    )
    cabinet.visual(
        Box((0.44, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, -0.17, 0.74)),
        material=dark_metal,
        name="console_body",
    )
    cabinet.visual(
        Box((0.46, 0.20, 0.06)),
        origin=Origin(xyz=(0.0, -0.24, 0.77), rpy=(-0.58, 0.0, 0.0)),
        material=chrome,
        name="console_cap",
    )
    cabinet.visual(
        Box((0.08, upper_depth, 0.14)),
        origin=Origin(xyz=(-0.34, 0.0, 0.99)),
        material=cabinet_paint,
        name="left_shoulder",
    )
    cabinet.visual(
        Box((0.08, upper_depth, 0.14)),
        origin=Origin(xyz=(0.34, 0.0, 0.99)),
        material=cabinet_paint,
        name="right_shoulder",
    )
    cabinet.visual(
        Box((0.05, upper_depth, 0.82)),
        origin=Origin(xyz=(-0.355, 0.0, 1.37)),
        material=cabinet_paint,
        name="upper_left_side",
    )
    cabinet.visual(
        Box((0.05, upper_depth, 0.82)),
        origin=Origin(xyz=(0.355, 0.0, 1.37)),
        material=cabinet_paint,
        name="upper_right_side",
    )
    cabinet.visual(
        Box((0.66, 0.04, 0.82)),
        origin=Origin(xyz=(0.0, 0.35, 1.37)),
        material=cabinet_paint,
        name="upper_back",
    )
    cabinet.visual(
        Box((0.66, 0.70, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.76)),
        material=black_trim,
        name="top_roof",
    )
    cabinet.visual(
        Cylinder(radius=0.14, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 1.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cabinet_paint,
        name="marquee_cap",
    )
    cabinet.visual(
        Box((0.11, 0.14, 0.44)),
        origin=Origin(xyz=(-0.285, -0.29, 1.02)),
        material=chrome,
        name="reel_left_frame",
    )
    cabinet.visual(
        Box((0.11, 0.14, 0.44)),
        origin=Origin(xyz=(0.285, -0.29, 1.02)),
        material=chrome,
        name="reel_right_frame",
    )
    cabinet.visual(
        Box((0.46, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, -0.28, 0.79)),
        material=chrome,
        name="reel_bottom_frame",
    )
    cabinet.visual(
        Box((0.46, 0.14, 0.09)),
        origin=Origin(xyz=(0.0, -0.29, 1.25)),
        material=chrome,
        name="reel_top_frame",
    )
    cabinet.visual(
        Box((0.02, 0.24, 0.40)),
        origin=Origin(xyz=(-0.08, -0.15, 1.02)),
        material=chrome,
        name="reel_divider_left",
    )
    cabinet.visual(
        Box((0.02, 0.24, 0.40)),
        origin=Origin(xyz=(0.08, -0.15, 1.02)),
        material=chrome,
        name="reel_divider_right",
    )
    cabinet.visual(
        Box((0.47, 0.008, 0.37)),
        origin=Origin(xyz=(0.0, -0.336, 1.02)),
        material=smoked_glass,
        name="reel_glass",
    )
    cabinet.visual(
        Box((0.13, 0.12, 0.42)),
        origin=Origin(xyz=(-0.275, -0.31, 1.52)),
        material=chrome,
        name="topper_left_frame",
    )
    cabinet.visual(
        Box((0.13, 0.12, 0.42)),
        origin=Origin(xyz=(0.275, -0.31, 1.52)),
        material=chrome,
        name="topper_right_frame",
    )
    cabinet.visual(
        Box((0.44, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.31, 1.31)),
        material=chrome,
        name="topper_bottom_frame",
    )
    cabinet.visual(
        Box((0.44, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, -0.31, 1.73)),
        material=chrome,
        name="topper_top_frame",
    )
    cabinet.visual(
        Box((0.45, 0.01, 0.41)),
        origin=Origin(xyz=(0.0, -0.338, 1.52)),
        material=display_black,
        name="topper_display",
    )
    cabinet.visual(
        Box((0.30, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, -0.29, 1.74)),
        material=chrome,
        name="marquee_badge",
    )
    cabinet.visual(
        Box((0.03, 0.16, 0.22)),
        origin=Origin(xyz=(0.285, -0.10, 0.82)),
        material=black_trim,
        name="lever_mount_plate",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.76, 0.74, 1.92)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Cylinder(radius=0.006, length=0.38),
        origin=Origin(xyz=(0.010, -0.026, 0.19)),
        material=chrome,
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.020, 0.014, 0.38)),
        origin=Origin(xyz=(0.018, -0.022, 0.19)),
        material=chrome,
        name="hinge_leaf",
    )
    service_door.visual(
        Box((0.35, 0.028, 0.38)),
        origin=Origin(xyz=(0.195, -0.014, 0.19)),
        material=black_trim,
        name="door_panel",
    )
    service_door.visual(
        Box((0.31, 0.008, 0.31)),
        origin=Origin(xyz=(0.205, -0.032, 0.19)),
        material=dark_metal,
        name="door_inset",
    )
    service_door.visual(
        Box((0.04, 0.035, 0.14)),
        origin=Origin(xyz=(0.31, -0.046, 0.20)),
        material=chrome,
        name="door_pull",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.37, 0.06, 0.40)),
        mass=8.0,
        origin=Origin(xyz=(0.19, -0.02, 0.19)),
    )

    pull_lever = model.part("pull_lever")
    pull_lever.visual(
        Cylinder(radius=0.028, length=0.03),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    pull_lever.visual(
        Box((0.05, 0.06, 0.08)),
        origin=Origin(xyz=(0.045, 0.035, -0.02)),
        material=chrome,
        name="lever_yoke",
    )
    pull_lever.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(xyz=(0.07, 0.09, -0.11), rpy=(-2.46, 0.0, 0.0)),
        material=chrome,
        name="lever_rod",
    )
    pull_lever.visual(
        Sphere(radius=0.04),
        origin=Origin(xyz=(0.07, 0.18, -0.22)),
        material=knob_red,
        name="lever_knob",
    )
    pull_lever.inertial = Inertial.from_geometry(
        Box((0.12, 0.22, 0.30)),
        mass=1.2,
        origin=Origin(xyz=(0.06, 0.10, -0.12)),
    )

    def add_reel(
        name: str,
        x_pos: float,
        accent_material,
        marker_name: str,
        center_name: str,
        lower_name: str,
    ) -> None:
        reel = model.part(name)
        reel.visual(
            Cylinder(radius=0.105, length=0.14),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_white,
            name="reel_drum",
        )
        reel.visual(
            Cylinder(radius=0.118, length=0.006),
            origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="left_end_cap",
        )
        reel.visual(
            Cylinder(radius=0.118, length=0.006),
            origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="right_end_cap",
        )
        reel.visual(
            Box((0.118, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, -0.094, 0.048)),
            material=accent_material,
            name=marker_name,
        )
        reel.visual(
            Box((0.118, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, -0.097, 0.000)),
            material=reel_gold,
            name=center_name,
        )
        reel.visual(
            Box((0.118, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, -0.094, -0.048)),
            material=reel_blue,
            name=lower_name,
        )
        reel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.118, length=0.14),
            mass=1.4,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x_pos, -0.17, 1.02)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=24.0),
        )

    add_reel("left_reel", -0.16, reel_red, "marker_top", "marker_mid", "marker_low")
    add_reel("center_reel", 0.0, reel_blue, "marker_top", "marker_mid", "marker_low")
    add_reel("right_reel", 0.16, reel_red, "marker_top", "marker_mid", "marker_low")

    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(-0.19, -0.31, 0.16)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "cabinet_to_pull_lever",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=pull_lever,
        origin=Origin(xyz=(0.30, -0.10, 0.86)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    service_door = object_model.get_part("service_door")
    pull_lever = object_model.get_part("pull_lever")
    left_reel = object_model.get_part("left_reel")
    center_reel = object_model.get_part("center_reel")
    right_reel = object_model.get_part("right_reel")

    door_joint = object_model.get_articulation("cabinet_to_service_door")
    lever_joint = object_model.get_articulation("cabinet_to_pull_lever")
    reel_joints = [
        object_model.get_articulation("left_reel_spin"),
        object_model.get_articulation("center_reel_spin"),
        object_model.get_articulation("right_reel_spin"),
    ]

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.check(
        "service door hinge axis is vertical",
        abs(door_joint.axis[0]) < 1e-9 and abs(door_joint.axis[1]) < 1e-9 and abs(abs(door_joint.axis[2]) - 1.0) < 1e-9,
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "lever pivots on a horizontal flank axis",
        abs(abs(lever_joint.axis[0]) - 1.0) < 1e-9 and abs(lever_joint.axis[1]) < 1e-9 and abs(lever_joint.axis[2]) < 1e-9,
        details=f"axis={lever_joint.axis}",
    )
    ctx.check(
        "reels spin on parallel horizontal axes",
        all(
            abs(abs(joint.axis[0]) - 1.0) < 1e-9
            and abs(joint.axis[1]) < 1e-9
            and abs(joint.axis[2]) < 1e-9
            for joint in reel_joints
        ),
        details=", ".join(f"{joint.name}:{joint.axis}" for joint in reel_joints),
    )

    for reel_name, reel in (
        ("left", left_reel),
        ("center", center_reel),
        ("right", right_reel),
    ):
        ctx.expect_gap(
            reel,
            cabinet,
            axis="y",
            min_gap=0.030,
            max_gap=0.080,
            positive_elem="reel_drum",
            negative_elem="reel_glass",
            name=f"{reel_name} reel sits behind the display glass",
        )

    rest_door_panel = aabb_center(ctx.part_element_world_aabb(service_door, elem="door_panel"))
    with ctx.pose({door_joint: 1.0}):
        open_door_panel = aabb_center(ctx.part_element_world_aabb(service_door, elem="door_panel"))
    ctx.check(
        "service door swings out from the lower front opening",
        rest_door_panel is not None
        and open_door_panel is not None
        and open_door_panel[1] < rest_door_panel[1] - 0.12
        and open_door_panel[0] < rest_door_panel[0] - 0.04,
        details=f"rest={rest_door_panel}, open={open_door_panel}",
    )

    rest_knob = aabb_center(ctx.part_element_world_aabb(pull_lever, elem="lever_knob"))
    with ctx.pose({lever_joint: 0.70}):
        pulled_knob = aabb_center(ctx.part_element_world_aabb(pull_lever, elem="lever_knob"))
    ctx.check(
        "side pull lever moves downward and forward when pulled",
        rest_knob is not None
        and pulled_knob is not None
        and pulled_knob[2] < rest_knob[2] - 0.06
        and pulled_knob[1] < rest_knob[1] - 0.07,
        details=f"rest={rest_knob}, pulled={pulled_knob}",
    )

    rest_marker = aabb_center(ctx.part_element_world_aabb(center_reel, elem="marker_top"))
    with ctx.pose({reel_joints[1]: 0.95}):
        spun_marker = aabb_center(ctx.part_element_world_aabb(center_reel, elem="marker_top"))
    ctx.check(
        "center reel marker rotates around the reel axis",
        rest_marker is not None
        and spun_marker is not None
        and abs(spun_marker[0] - rest_marker[0]) < 0.005
        and (
            ((spun_marker[1] - rest_marker[1]) ** 2 + (spun_marker[2] - rest_marker[2]) ** 2)
            ** 0.5
        )
        > 0.06,
        details=f"rest={rest_marker}, spun={spun_marker}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
