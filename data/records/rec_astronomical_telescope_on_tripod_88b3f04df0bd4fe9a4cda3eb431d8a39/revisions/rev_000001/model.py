from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maksutov_cassegrain_altaz_tripod")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    tube_blue = model.material("deep_blue_enamel", rgba=(0.02, 0.05, 0.12, 1.0))
    anodized = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_grey = model.material("dark_grey_casting", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("pale_coated_glass", rgba=(0.45, 0.70, 0.88, 0.38))

    # Root: compact tripod crown, centre column, azimuth bearing base and leg
    # hinge barrels.  The frame origin is on the floor below the azimuth axis.
    tripod_hub = model.part("tripod_hub")
    tripod_hub.visual(
        Cylinder(radius=0.026, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=anodized,
        name="centre_column",
    )
    tripod_hub.visual(
        Cylinder(radius=0.12, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_grey,
        name="accessory_tray",
    )
    tripod_hub.visual(
        Cylinder(radius=0.092, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=dark_grey,
        name="tripod_crown",
    )
    tripod_hub.visual(
        Cylinder(radius=0.074, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=dark_grey,
        name="azimuth_base",
    )

    leg_hinge_radius = 0.112
    leg_hinge_z = 0.720
    leg_run = 0.450
    leg_drop = 0.690
    leg_length = math.hypot(leg_run, leg_drop)
    leg_pitch = math.atan2(leg_drop, leg_run)
    rail_start = 0.055

    legs = []
    for index, theta in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        hinge_x = leg_hinge_radius * math.cos(theta)
        hinge_y = leg_hinge_radius * math.sin(theta)

        tripod_hub.visual(
            Cylinder(radius=0.010, length=0.100),
            origin=Origin(
                xyz=(hinge_x, hinge_y, leg_hinge_z),
                rpy=(-math.pi / 2.0, 0.0, theta),
            ),
            material=dark_grey,
            name=f"leg_hinge_{index}",
        )
        for cheek, cheek_y in enumerate((-0.044, 0.044)):
            support_x = leg_hinge_radius - 0.022
            tripod_hub.visual(
                Box((0.055, 0.016, 0.030)),
                origin=Origin(
                    xyz=(
                        support_x * math.cos(theta) - cheek_y * math.sin(theta),
                        support_x * math.sin(theta) + cheek_y * math.cos(theta),
                        leg_hinge_z,
                    ),
                    rpy=(0.0, 0.0, theta),
                ),
                material=dark_grey,
                name=f"hinge_cheek_{index}_{cheek}",
            )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.016, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.080, 0.100, 0.045)),
            origin=Origin(xyz=(0.045, 0.0, -0.035)),
            material=dark_grey,
            name="top_block",
        )
        for side, y_offset in enumerate((-0.034, 0.034)):
            leg.visual(
                Box((leg_length - rail_start, 0.018, 0.018)),
                origin=Origin(
                    xyz=(
                        leg_run * (rail_start + leg_length) / (2.0 * leg_length),
                        y_offset,
                        -leg_drop * (rail_start + leg_length) / (2.0 * leg_length),
                    ),
                    rpy=(0.0, leg_pitch, 0.0),
                ),
                material=anodized,
                name=f"aluminium_rail_{side}",
            )
        leg.visual(
            Box((0.115, 0.105, 0.030)),
            origin=Origin(xyz=(leg_run - 0.020, 0.0, -leg_drop + 0.025)),
            material=dark_grey,
            name="lower_bridge",
        )
        leg.visual(
            Box((0.170, 0.075, 0.024)),
            origin=Origin(xyz=(leg_run + 0.020, 0.0, -leg_drop - 0.010)),
            material=rubber,
            name="foot_pad",
        )
        legs.append((leg, theta, hinge_x, hinge_y))

        model.articulation(
            f"hub_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=tripod_hub,
            child=leg,
            origin=Origin(xyz=(hinge_x, hinge_y, leg_hinge_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=0.65),
        )

    # Azimuth head: a compact rotating casting with a dual fork for the altitude
    # bearings.  Its local +X direction is the telescope's forward direction.
    az_mount = model.part("az_mount")
    az_mount.visual(
        Cylinder(radius=0.070, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_grey,
        name="azimuth_turntable",
    )
    az_mount.visual(
        Cylinder(radius=0.045, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_grey,
        name="mount_neck",
    )
    az_mount.visual(
        Box((0.110, 0.400, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_grey,
        name="fork_bridge",
    )
    for side, y_offset in enumerate((-0.172, 0.172)):
        az_mount.visual(
            Box((0.075, 0.045, 0.400)),
            origin=Origin(xyz=(0.0, y_offset, 0.320)),
            material=dark_grey,
            name=f"fork_arm_{side}",
        )
        az_mount.visual(
            Cylinder(radius=0.058, length=0.034),
            origin=Origin(
                xyz=(0.0, y_offset * 0.90, 0.360),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=anodized,
            name=f"altitude_bearing_{side}",
        )
    az_mount.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(-0.070, -0.210, 0.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="az_lock_knob",
    )

    model.articulation(
        "tripod_to_azimuth",
        ArticulationType.REVOLUTE,
        parent=tripod_hub,
        child=az_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )

    # Maksutov-Cassegrain optical tube: short, broad tube with a meniscus
    # corrector, secondary spot, rear cell, finder and side trunnions.
    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Cylinder(radius=0.105, length=0.500),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_blue,
        name="main_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.113, length=0.050),
        origin=Origin(xyz=(0.295, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.098, length=0.010),
        origin=Origin(xyz=(0.325, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_corrector",
    )
    optical_tube.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.332, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="secondary_spot",
    )
    optical_tube.visual(
        Cylinder(radius=0.110, length=0.052),
        origin=Origin(xyz=(-0.252, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.036, length=0.075),
        origin=Origin(xyz=(-0.315, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="visual_back",
    )
    for side, y_offset in enumerate((-0.123, 0.123)):
        optical_tube.visual(
            Cylinder(radius=0.044, length=0.042),
            origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name=f"side_trunnion_{side}",
        )
    optical_tube.visual(
        Box((0.245, 0.032, 0.020)),
        origin=Origin(xyz=(0.025, 0.0, 0.113)),
        material=matte_black,
        name="finder_rail",
    )
    for x_offset in (-0.075, 0.125):
        optical_tube.visual(
            Box((0.026, 0.055, 0.070)),
            origin=Origin(xyz=(x_offset, 0.0, 0.142)),
            material=matte_black,
            name=f"finder_stand_{x_offset:+.3f}",
        )
    optical_tube.visual(
        Cylinder(radius=0.023, length=0.310),
        origin=Origin(xyz=(0.025, 0.0, 0.178), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="finder_scope",
    )
    optical_tube.visual(
        Box((0.052, 0.018, 0.042)),
        origin=Origin(xyz=(-0.200, -0.107, -0.025)),
        material=matte_black,
        name="focus_boss",
    )

    model.articulation(
        "azimuth_to_tube",
        ArticulationType.REVOLUTE,
        parent=az_mount,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8, lower=-0.25, upper=1.35),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="knob_stem",
    )
    focus_knob.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="grip_drum",
    )

    model.articulation(
        "tube_to_focus",
        ArticulationType.REVOLUTE,
        parent=optical_tube,
        child=focus_knob,
        origin=Origin(xyz=(-0.200, -0.116, -0.025)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_hub = object_model.get_part("tripod_hub")
    az_mount = object_model.get_part("az_mount")
    optical_tube = object_model.get_part("optical_tube")

    az_joint = object_model.get_articulation("tripod_to_azimuth")
    alt_joint = object_model.get_articulation("azimuth_to_tube")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        ctx.allow_overlap(
            tripod_hub,
            leg,
            elem_a=f"leg_hinge_{index}",
            elem_b="hinge_knuckle",
            reason="Tripod leg knuckle is intentionally captured around the crown hinge pin.",
        )
        ctx.expect_overlap(
            tripod_hub,
            leg,
            axes="xyz",
            elem_a=f"leg_hinge_{index}",
            elem_b="hinge_knuckle",
            min_overlap=0.010,
            name=f"leg {index} hinge pin is captured",
        )

    for side in (0, 1):
        ctx.allow_overlap(
            az_mount,
            optical_tube,
            elem_a=f"altitude_bearing_{side}",
            elem_b=f"side_trunnion_{side}",
            reason="The telescope trunnion is seated in the altitude bearing socket.",
        )
        ctx.expect_overlap(
            az_mount,
            optical_tube,
            axes="xyz",
            elem_a=f"altitude_bearing_{side}",
            elem_b=f"side_trunnion_{side}",
            min_overlap=0.003,
            name=f"altitude trunnion {side} is seated",
        )

    ctx.expect_gap(
        az_mount,
        tripod_hub,
        axis="z",
        max_gap=0.002,
        max_penetration=1e-6,
        positive_elem="azimuth_turntable",
        negative_elem="azimuth_base",
        name="azimuth turntable rests on base",
    )
    ctx.expect_overlap(
        az_mount,
        tripod_hub,
        axes="xy",
        elem_a="azimuth_turntable",
        elem_b="azimuth_base",
        min_overlap=0.05,
        name="azimuth bearing is centred on tripod",
    )

    front_rest = ctx.part_element_world_aabb(optical_tube, elem="front_corrector")
    with ctx.pose({alt_joint: 1.0}):
        front_raised = ctx.part_element_world_aabb(optical_tube, elem="front_corrector")
    ctx.check(
        "altitude axis raises telescope front",
        front_rest is not None
        and front_raised is not None
        and (front_raised[0][2] + front_raised[1][2]) / 2.0 > (front_rest[0][2] + front_rest[1][2]) / 2.0 + 0.12,
        details=f"rest={front_rest}, raised={front_raised}",
    )

    with ctx.pose({az_joint: math.pi / 2.0}):
        front_turned = ctx.part_element_world_aabb(optical_tube, elem="front_corrector")
    ctx.check(
        "azimuth axis swings telescope around vertical",
        front_rest is not None
        and front_turned is not None
        and (front_turned[0][1] + front_turned[1][1]) / 2.0 > 0.20,
        details=f"rest={front_rest}, turned={front_turned}",
    )

    leg_0 = object_model.get_part("leg_0")
    leg_fold = object_model.get_articulation("hub_to_leg_0")
    foot_rest = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
    with ctx.pose({leg_fold: 0.50}):
        foot_folded = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
    rest_radius = None
    folded_radius = None
    if foot_rest is not None:
        rest_x = (foot_rest[0][0] + foot_rest[1][0]) / 2.0
        rest_y = (foot_rest[0][1] + foot_rest[1][1]) / 2.0
        rest_radius = math.hypot(rest_x, rest_y)
    if foot_folded is not None:
        folded_x = (foot_folded[0][0] + foot_folded[1][0]) / 2.0
        folded_y = (foot_folded[0][1] + foot_folded[1][1]) / 2.0
        folded_radius = math.hypot(folded_x, folded_y)
    ctx.check(
        "tripod leg folds toward centre column",
        rest_radius is not None and folded_radius is not None and folded_radius < rest_radius - 0.15,
        details=f"rest_radius={rest_radius}, folded_radius={folded_radius}, rest={foot_rest}, folded={foot_folded}",
    )

    return ctx.report()


object_model = build_object_model()
