from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=12),
            thickness,
            cap=True,
            center=True,
        ),
        name,
    )


def _wheel_meshes(prefix: str):
    wheel = WheelGeometry(
        0.078,
        0.044,
        rim=WheelRim(
            inner_radius=0.052,
            flange_height=0.004,
            flange_thickness=0.003,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.024,
            width=0.032,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=0.030,
                hole_diameter=0.0035,
            ),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0028, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.010),
    )
    tire = TireGeometry(
        0.096,
        0.050,
        inner_radius=0.074,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
        tread=TireTread(style="ribbed", depth=0.0025, count=28, land_ratio=0.68),
        grooves=(
            TireGroove(center_offset=-0.010, width=0.003, depth=0.0015),
            TireGroove(center_offset=0.010, width=0.003, depth=0.0015),
        ),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.004, radius=0.0025),
    )
    return (
        mesh_from_geometry(wheel, f"{prefix}_wheel"),
        mesh_from_geometry(tire, f"{prefix}_tire"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_folding_scooter")

    deck_mat = model.material("satin_charcoal_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    grip_mat = model.material("black_grip_tape", rgba=(0.015, 0.015, 0.013, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    hinge_mat = model.material("dark_hinge_hardware", rgba=(0.04, 0.045, 0.05, 1.0))
    tire_mat = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    urethane_mat = model.material("smoked_urethane_sidewall", rgba=(0.03, 0.032, 0.034, 1.0))

    deck = model.part("deck")
    deck.visual(
        _rounded_plate_mesh(0.86, 0.160, 0.045, 0.040, "rounded_deck_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=deck_mat,
        name="deck_shell",
    )
    deck.visual(
        _rounded_plate_mesh(0.74, 0.122, 0.004, 0.024, "grip_tape_pad"),
        origin=Origin(xyz=(-0.025, 0.0, 0.179)),
        material=grip_mat,
        name="grip_tape",
    )
    for y in (-0.086, 0.086):
        deck.visual(
            Box((0.78, 0.012, 0.022)),
            origin=Origin(xyz=(-0.015, 0.079 if y > 0 else -0.079, 0.143)),
            material=metal_mat,
            name=f"side_rail_{0 if y < 0 else 1}",
        )

    front_x = 0.540
    rear_x = -0.540
    wheel_z = 0.096
    fork_y = 0.056
    for label, x, bridge_x in (
        ("front", front_x, 0.475),
        ("rear", rear_x, -0.475),
    ):
        for y in (-fork_y, fork_y):
            deck.visual(
                Box((0.044, 0.014, 0.112)),
                origin=Origin(xyz=(x, y, 0.118)),
                material=metal_mat,
                name=f"{label}_fork_plate_{0 if y < 0 else 1}",
            )
            deck.visual(
                Box((0.142, 0.014, 0.026)),
                origin=Origin(xyz=(bridge_x, y, 0.165)),
                material=metal_mat,
                name=f"{label}_dropout_arm_{0 if y < 0 else 1}",
            )
            deck.visual(
                Cylinder(radius=0.017, length=0.010),
                origin=Origin(
                    xyz=(x, y + (0.010 if y > 0 else -0.010), wheel_z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=hinge_mat,
                name=f"{label}_axle_cap_{0 if y < 0 else 1}",
            )
        deck.visual(
            Cylinder(radius=0.007, length=0.126),
            origin=Origin(xyz=(x, 0.0, wheel_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"{label}_axle_pin",
        )

    deck.visual(
        Box((0.170, 0.120, 0.012)),
        origin=Origin(xyz=(rear_x, 0.0, 0.204)),
        material=hinge_mat,
        name="rear_brake_fender",
    )
    for y in (-0.050, 0.050):
        deck.visual(
            Box((0.016, 0.014, 0.032)),
            origin=Origin(xyz=(rear_x, y, 0.186)),
            material=hinge_mat,
            name=f"fender_stay_{0 if y < 0 else 1}",
        )

    hinge_x = 0.385
    hinge_z = 0.205
    for y in (-0.042, 0.042):
        deck.visual(
            Box((0.060, 0.012, 0.080)),
            origin=Origin(xyz=(hinge_x, y, hinge_z)),
            material=hinge_mat,
            name=f"stem_clevis_{0 if y < 0 else 1}",
        )
        deck.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(
                xyz=(hinge_x, y + (0.008 if y > 0 else -0.008), hinge_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_mat,
            name=f"stem_pivot_cap_{0 if y < 0 else 1}",
        )
    deck.visual(
        Cylinder(radius=0.007, length=0.106),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="stem_pivot_pin",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_barrel",
    )
    stem.visual(
        Cylinder(radius=0.023, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=metal_mat,
        name="lower_sleeve",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=metal_mat,
        name="upper_tube",
    )
    stem.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=hinge_mat,
        name="height_collar",
    )
    stem.visual(
        Box((0.052, 0.070, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=hinge_mat,
        name="bar_clamp",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.825), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="handlebar",
    )
    for y in (-0.255, 0.255):
        stem.visual(
            Cylinder(radius=0.018, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.825), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=tire_mat,
            name=f"rubber_grip_{0 if y < 0 else 1}",
        )

    front_wheel = model.part("front_wheel")
    front_wheel_mesh, front_tire_mesh = _wheel_meshes("front")
    front_wheel.visual(
        front_wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal_mat,
        name="spoked_hub",
    )
    front_wheel.visual(
        front_tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=urethane_mat,
        name="rubber_tire",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel_mesh, rear_tire_mesh = _wheel_meshes("rear")
    rear_wheel.visual(
        rear_wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal_mat,
        name="spoked_hub",
    )
    rear_wheel.visual(
        rear_tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=urethane_mat,
        name="rubber_tire",
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.38),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(front_x, 0.0, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(rear_x, 0.0, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    stem_hinge = object_model.get_articulation("deck_to_stem")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.allow_overlap(
        deck,
        stem,
        elem_a="stem_pivot_pin",
        elem_b="hinge_barrel",
        reason="The folding-stem pivot pin is intentionally captured inside the hinge barrel.",
    )
    ctx.allow_overlap(
        deck,
        front_wheel,
        elem_a="front_axle_pin",
        elem_b="spoked_hub",
        reason="The front axle pin intentionally passes through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle_pin",
        elem_b="spoked_hub",
        reason="The rear axle pin intentionally passes through the wheel hub bore.",
    )
    ctx.expect_overlap(
        deck,
        stem,
        axes="y",
        elem_a="stem_pivot_pin",
        elem_b="hinge_barrel",
        min_overlap=0.045,
        name="stem pivot pin spans hinge barrel",
    )
    ctx.expect_within(
        deck,
        stem,
        axes="xz",
        inner_elem="stem_pivot_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="stem pivot pin sits inside hinge barrel",
    )
    for wheel, pin_name, check_name in (
        (front_wheel, "front_axle_pin", "front axle captured in hub"),
        (rear_wheel, "rear_axle_pin", "rear axle captured in hub"),
    ):
        ctx.expect_overlap(
            deck,
            wheel,
            axes="y",
            elem_a=pin_name,
            elem_b="spoked_hub",
            min_overlap=0.030,
            name=f"{check_name} width overlap",
        )
        ctx.expect_within(
            deck,
            wheel,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="spoked_hub",
            margin=0.002,
            name=f"{check_name} radial fit",
        )

    deck_bounds = ctx.part_element_world_aabb(deck, elem="deck_shell")
    if deck_bounds is not None:
        deck_min, deck_max = deck_bounds
        ctx.check(
            "long narrow scooter deck",
            (deck_max[0] - deck_min[0]) > 0.82 and (deck_max[1] - deck_min[1]) < 0.22,
            details=f"deck aabb={deck_bounds}",
        )

    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=1.00,
        max_gap=1.12,
        name="wheelbase matches commuter scooter scale",
    )
    deck_shell_bounds = ctx.part_element_world_aabb(deck, elem="deck_shell")
    front_pos = ctx.part_world_position(front_wheel)
    rear_pos = ctx.part_world_position(rear_wheel)
    if deck_shell_bounds is not None and front_pos is not None and rear_pos is not None:
        deck_shell_min, _ = deck_shell_bounds
        ctx.check(
            "deck platform rides above both axles",
            deck_shell_min[2] > front_pos[2] + 0.025 and deck_shell_min[2] > rear_pos[2] + 0.025,
            details=f"deck_shell={deck_shell_bounds}, front={front_pos}, rear={rear_pos}",
        )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        max_penetration=0.0,
        positive_elem="rubber_tire",
        negative_elem="deck_shell",
        name="front tire clears deck nose",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        max_penetration=0.0,
        positive_elem="deck_shell",
        negative_elem="rubber_tire",
        name="rear tire clears deck tail",
    )

    ctx.check(
        "wheel axles rotate about scooter width",
        tuple(round(v, 3) for v in front_axle.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in rear_axle.axis) == (0.0, 1.0, 0.0),
        details=f"front_axis={front_axle.axis}, rear_axis={rear_axle.axis}",
    )

    upright_handlebar = ctx.part_element_world_aabb(stem, elem="handlebar")
    upright_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({stem_hinge: stem_hinge.motion_limits.upper}):
        folded_handlebar = ctx.part_element_world_aabb(stem, elem="handlebar")
        folded_aabb = ctx.part_world_aabb(stem)

    if upright_handlebar is not None and folded_handlebar is not None:
        upright_x = (upright_handlebar[0][0] + upright_handlebar[1][0]) * 0.5
        folded_x = (folded_handlebar[0][0] + folded_handlebar[1][0]) * 0.5
        ctx.check(
            "stem hinge collapses rearward",
            folded_x < upright_x - 0.45,
            details=f"upright_handlebar={upright_handlebar}, folded_handlebar={folded_handlebar}",
        )
    if upright_aabb is not None and folded_aabb is not None:
        ctx.check(
            "stem height drops when folded",
            folded_aabb[1][2] < upright_aabb[1][2] - 0.35,
            details=f"upright_aabb={upright_aabb}, folded_aabb={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
