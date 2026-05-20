from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _deck_shell():
    return (
        cq.Workplane("XY")
        .box(0.90, 0.24, 0.045)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.006)
    )


def _arm_rod_origin(side: float, y_out: float, z_drop: float) -> tuple[Origin, float]:
    length = math.hypot(y_out, z_drop)
    # Cylinder local +Z is rotated into the hinge-to-axle vector in the arm YZ plane.
    roll = math.atan2(-side * y_out, z_drop)
    return Origin(xyz=(0.0, side * y_out * 0.5, z_drop * 0.5), rpy=(roll, 0.0, 0.0)), length


def _add_wheel_visuals(part, prefix: str, *, radius: float, width: float, rim_radius: float) -> None:
    rim = WheelGeometry(
        rim_radius,
        width * 0.88,
        rim=WheelRim(
            inner_radius=rim_radius * 0.62,
            flange_height=0.0045,
            flange_thickness=0.0025,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=rim_radius * 0.36,
            width=width * 0.62,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=rim_radius * 0.48,
                hole_diameter=0.003,
            ),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0028, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.014),
    )
    tire = TireGeometry(
        radius,
        width,
        inner_radius=rim_radius * 0.94,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="chevron", depth=0.0035, count=18, angle_deg=22.0, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0015),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.0035, radius=0.0025),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire_mesh"), material="mat_tire", name="tire")
    part.visual(mesh_from_geometry(rim, f"{prefix}_rim_mesh"), material="mat_silver", name="rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stairclimber_scooter")

    model.material("mat_deck_blue", rgba=(0.04, 0.18, 0.55, 1.0))
    model.material("mat_grip_black", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("mat_tire", rgba=(0.006, 0.006, 0.005, 1.0))
    model.material("mat_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    model.material("mat_dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("mat_handle_grip", rgba=(0.015, 0.015, 0.018, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_deck_shell(), "rounded_deck_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material="mat_deck_blue",
        name="deck_shell",
    )
    deck.visual(
        Box((0.67, 0.165, 0.004)),
        origin=Origin(xyz=(-0.035, 0.0, 0.204)),
        material="mat_grip_black",
        name="grip_pad",
    )

    # Fixed scooter frame details: a simple handlebar, front/rear forks, and rear hinge yokes.
    deck.visual(
        Cylinder(radius=0.012, length=0.68),
        origin=Origin(xyz=(0.315, 0.0, 0.540)),
        material="mat_dark_metal",
        name="handle_stem",
    )
    deck.visual(
        Cylinder(radius=0.011, length=0.42),
        origin=Origin(xyz=(0.315, 0.0, 0.880), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="mat_dark_metal",
        name="handlebar",
    )
    for y in (-0.155, 0.155):
        deck.visual(
            Cylinder(radius=0.016, length=0.070),
            origin=Origin(xyz=(0.315, y, 0.880), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="mat_handle_grip",
            name=f"grip_{0 if y < 0.0 else 1}",
        )

    wheel_z = 0.075
    deck_under_z = 0.1575
    fork_height = deck_under_z - wheel_z + 0.006
    for label, x in (("front", 0.405), ("rear", -0.405)):
        for y in (-0.040, 0.040):
            deck.visual(
                Box((0.030, 0.012, fork_height)),
                origin=Origin(xyz=(x, y, wheel_z + fork_height * 0.5 - 0.003)),
                material="mat_dark_metal",
                name=f"{label}_fork_{0 if y < 0.0 else 1}",
            )
        deck.visual(
            Cylinder(radius=0.0052, length=0.102),
            origin=Origin(xyz=(x, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="mat_silver",
            name=f"{label}_axle_pin",
        )
        for y in (-0.022, 0.022):
            deck.visual(
                Cylinder(radius=0.021, length=0.004),
                origin=Origin(xyz=(x, y, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="mat_silver",
                name=f"{label}_axle_collar_{0 if y < 0.0 else 1}",
            )

    hinge_x = -0.325
    hinge_y = 0.124
    hinge_z = 0.150
    for side, side_name in ((1.0, "rear_left"), (-1.0, "rear_right")):
        for x_off in (-0.024, 0.024):
            deck.visual(
                Box((0.008, 0.033, 0.044)),
                origin=Origin(xyz=(hinge_x + x_off, side * hinge_y, deck_under_z - 0.022)),
                material="mat_dark_metal",
                name=f"{side_name}_hinge_yoke_{0 if x_off < 0.0 else 1}",
            )

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(front_wheel, "front_wheel", radius=0.075, width=0.045, rim_radius=0.056)
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(0.405, 0.0, wheel_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(rear_wheel, "rear_wheel", radius=0.075, width=0.045, rim_radius=0.056)
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.405, 0.0, wheel_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    assist_drop = -0.070
    assist_out = 0.055
    assist_rod_out = 0.032
    for side, side_name, lower, upper in (
        (1.0, "rear_left", 0.0, 1.35),
        (-1.0, "rear_right", -1.35, 0.0),
    ):
        arm = model.part(f"{side_name}_assist_arm")
        arm.visual(
            Cylinder(radius=0.011, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="mat_silver",
            name="hinge_barrel",
        )
        rod_origin, rod_length = _arm_rod_origin(side, assist_rod_out, assist_drop)
        arm.visual(
            Cylinder(radius=0.0065, length=rod_length),
            origin=rod_origin,
            material="mat_dark_metal",
            name="diagonal_arm",
        )
        arm.visual(
            Cylinder(radius=0.0050, length=0.060),
            origin=Origin(xyz=(0.0, side * assist_out, assist_drop), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="mat_silver",
            name="assist_axle_pin",
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(
                xyz=(0.0, side * (assist_out - 0.015), assist_drop),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="mat_silver",
            name="assist_axle_collar",
        )
        arm.visual(
            Box((0.026, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, side * assist_rod_out, assist_drop)),
            material="mat_dark_metal",
            name="axle_boss",
        )
        hinge = model.articulation(
            f"{side_name}_fold_hinge",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=arm,
            origin=Origin(xyz=(hinge_x, side * hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=lower, upper=upper),
        )

        wheel = model.part(f"{side_name}_assist_wheel")
        _add_wheel_visuals(wheel, f"{side_name}_assist_wheel", radius=0.055, width=0.034, rim_radius=0.040)
        model.articulation(
            f"{side_name}_assist_axle",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=wheel,
            origin=Origin(xyz=(0.0, side * assist_out, assist_drop), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=18.0),
        )

        # Keep the local variable semantically used for readability in the construction loop.
        hinge.meta["role"] = "folding stair-assist wheel bracket"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    left_wheel = object_model.get_part("rear_left_assist_wheel")
    right_wheel = object_model.get_part("rear_right_assist_wheel")
    left_hinge = object_model.get_articulation("rear_left_fold_hinge")
    right_hinge = object_model.get_articulation("rear_right_fold_hinge")

    for wheel, name in ((front_wheel, "front"), (rear_wheel, "rear")):
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.002,
            max_gap=0.018,
            positive_elem="deck_shell",
            name=f"{name} wheel clears flat deck underside",
        )
        ctx.expect_overlap(
            wheel,
            deck,
            axes="y",
            min_overlap=0.035,
            elem_a="tire",
            elem_b="deck_shell",
            name=f"{name} wheel sits under deck centerline",
        )

    rest_left = ctx.part_world_position(left_wheel)
    rest_right = ctx.part_world_position(right_wheel)
    with ctx.pose({left_hinge: 1.20, right_hinge: -1.20}):
        folded_left = ctx.part_world_position(left_wheel)
        folded_right = ctx.part_world_position(right_wheel)

    ctx.check(
        "assist wheels fold upward at rear corners",
        rest_left is not None
        and rest_right is not None
        and folded_left is not None
        and folded_right is not None
        and folded_left[2] > rest_left[2] + 0.07
        and folded_right[2] > rest_right[2] + 0.07,
        details=f"rest_left={rest_left}, folded_left={folded_left}, rest_right={rest_right}, folded_right={folded_right}",
    )

    return ctx.report()


object_model = build_object_model()
