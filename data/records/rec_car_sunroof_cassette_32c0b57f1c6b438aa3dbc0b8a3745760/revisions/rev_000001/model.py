from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lamella_louvre_sunroof_cassette")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.012, 1.0))
    matte_black = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    graphite = model.material("graphite_anodized", rgba=(0.10, 0.105, 0.11, 1.0))
    bearing_metal = model.material("dark_bearing_metal", rgba=(0.33, 0.34, 0.34, 1.0))
    tinted_glass = Material("smoked_blue_glass", rgba=(0.08, 0.18, 0.24, 0.48))

    frame = model.part("cassette_frame")

    # Realistic automobile sunroof cassette scale: a shallow rectangular cassette
    # with deep lateral rails and cross members around the louvre opening.
    outer_x = 1.10
    outer_y = 0.74
    rail_w = 0.075
    cross_w = 0.070
    rail_h = 0.055
    rail_z = 0.012
    open_x = outer_x - 2.0 * rail_w

    frame.visual(
        Box((rail_w, outer_y, rail_h)),
        origin=Origin(xyz=(-outer_x / 2.0 + rail_w / 2.0, 0.0, rail_z)),
        material=satin_black,
        name="side_rail_0",
    )
    frame.visual(
        Box((rail_w, outer_y, rail_h)),
        origin=Origin(xyz=(outer_x / 2.0 - rail_w / 2.0, 0.0, rail_z)),
        material=satin_black,
        name="side_rail_1",
    )
    frame.visual(
        Box((outer_x, cross_w, rail_h)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + cross_w / 2.0, rail_z)),
        material=satin_black,
        name="end_rail_0",
    )
    frame.visual(
        Box((outer_x, cross_w, rail_h)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - cross_w / 2.0, rail_z)),
        material=satin_black,
        name="end_rail_1",
    )

    # Inset ledges, gutters, and a thin lower pan make the frame read as a cassette
    # rather than a simple rectangular outline.
    for idx, x in enumerate((-open_x / 2.0 - 0.012, open_x / 2.0 + 0.012)):
        frame.visual(
            Box((0.026, 0.600, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.048)),
            material=graphite,
            name=f"inner_lip_{idx}",
        )
        frame.visual(
            Box((0.018, 0.610, 0.012)),
            origin=Origin(xyz=(x * 0.985, 0.0, -0.021)),
            material=matte_black,
            name=f"drain_channel_{idx}",
        )

    frame.visual(
        Box((0.96, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.300, -0.018)),
        material=matte_black,
        name="front_drain_bridge",
    )
    frame.visual(
        Box((0.96, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.300, -0.018)),
        material=matte_black,
        name="rear_drain_bridge",
    )

    slat_y = [-0.224, -0.112, 0.0, 0.112, 0.224]
    pivot_z = 0.036
    # The bearing barrels sit just proud of the side-rail inner faces.  The
    # moving pins engage the protruding bushing noses rather than passing
    # through the simplified solid rail boxes.
    bushing_x = open_x / 2.0 - 0.0145
    pin_x = open_x / 2.0 - 0.025
    bushing_len = 0.029
    bushing_radius = 0.014

    # Ten bearing bosses are visible in the two side rails; each glass lamella
    # carries a pin into a left/right bushing pair.
    for i, y in enumerate(slat_y):
        for side, sx in (("left", -bushing_x), ("right", bushing_x)):
            frame.visual(
                Cylinder(radius=bushing_radius, length=bushing_len),
                origin=Origin(
                    xyz=(sx, y, pivot_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=bearing_metal,
                name=f"{side}_bushing_{i}",
            )

    source_joint = None
    for i, y in enumerate(slat_y):
        slat = model.part(f"glass_slat_{i}")

        # The part frame sits on the horizontal pin axis.  The glass, rubber
        # seals, and aluminum spine overlap locally so each lamella is a single
        # supported blade assembly.
        slat.visual(
            Box((0.866, 0.096, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=tinted_glass,
            name="glass_blade",
        )
        slat.visual(
            Box((0.866, 0.011, 0.012)),
            origin=Origin(xyz=(0.0, -0.052, 0.005)),
            material=matte_black,
            name="edge_seal_0",
        )
        slat.visual(
            Box((0.866, 0.011, 0.012)),
            origin=Origin(xyz=(0.0, 0.052, 0.005)),
            material=matte_black,
            name="edge_seal_1",
        )
        slat.visual(
            Box((0.866, 0.012, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=graphite,
            name="pivot_spine",
        )
        for side, sx in (("left", -pin_x), ("right", pin_x)):
            slat.visual(
                Cylinder(radius=0.0065, length=0.044),
                origin=Origin(
                    xyz=(sx, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=bearing_metal,
                name=f"{side}_pin",
            )

        joint_name = f"frame_to_slat_{i}"
        joint = model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, y, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=0.92),
            mimic=Mimic(source_joint.name) if source_joint is not None else None,
        )
        if source_joint is None:
            source_joint = joint

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    source_joint = object_model.get_articulation("frame_to_slat_0")

    for i in range(5):
        slat = object_model.get_part(f"glass_slat_{i}")
        joint = object_model.get_articulation(f"frame_to_slat_{i}")

        ctx.check(
            f"slat {i} has horizontal pivot axis",
            tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

        for side in ("left", "right"):
            ctx.allow_overlap(
                frame,
                slat,
                elem_a=f"{side}_bushing_{i}",
                elem_b=f"{side}_pin",
                reason="The slat pin is intentionally captured inside the side-rail bearing bushing.",
            )
            ctx.expect_within(
                slat,
                frame,
                axes="yz",
                inner_elem=f"{side}_pin",
                outer_elem=f"{side}_bushing_{i}",
                margin=0.002,
                name=f"{side} pin {i} is concentric in bushing",
            )
            ctx.expect_overlap(
                slat,
                frame,
                axes="x",
                elem_a=f"{side}_pin",
                elem_b=f"{side}_bushing_{i}",
                min_overlap=0.025,
                name=f"{side} pin {i} remains inserted",
            )

    with ctx.pose({source_joint: 0.92}):
        zmaxes = []
        for i in range(5):
            opened_aabb = ctx.part_world_aabb(object_model.get_part(f"glass_slat_{i}"))
            zmaxes.append(opened_aabb[1][2] if opened_aabb is not None else None)
        ctx.check(
            "slats open together for ventilation",
            all(z is not None and z > 0.080 for z in zmaxes)
            and (max(zmaxes) - min(zmaxes) < 0.002),
            details=f"opened_zmaxes={zmaxes}",
        )

    return ctx.report()


object_model = build_object_model()
