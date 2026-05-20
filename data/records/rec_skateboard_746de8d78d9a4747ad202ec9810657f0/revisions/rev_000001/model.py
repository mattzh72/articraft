from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
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


def _capsule_deck(length: float, width: float, thickness: float) -> cq.Workplane:
    """Rounded, thick utility deck blank centered on the local origin."""
    straight = length - width
    body = cq.Workplane("XY").box(straight, width, thickness)
    end_a = (
        cq.Workplane("XY")
        .circle(width / 2.0)
        .extrude(thickness)
        .translate((straight / 2.0, 0.0, -thickness / 2.0))
    )
    end_b = (
        cq.Workplane("XY")
        .circle(width / 2.0)
        .extrude(thickness)
        .translate((-straight / 2.0, 0.0, -thickness / 2.0))
    )
    return body.union(end_a).union(end_b)


def _make_wheel_meshes(prefix: str):
    tire = TireGeometry(
        0.043,
        0.034,
        inner_radius=0.029,
        tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.54),
        grooves=(
            TireGroove(center_offset=-0.006, width=0.003, depth=0.0015),
            TireGroove(center_offset=0.006, width=0.003, depth=0.0015),
        ),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    wheel = WheelGeometry(
        0.031,
        0.032,
        rim=WheelRim(
            inner_radius=0.019,
            flange_height=0.003,
            flange_thickness=0.0025,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.014,
            width=0.038,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=6,
                circle_diameter=0.020,
                hole_diameter=0.0025,
            ),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.014),
    )
    return (
        mesh_from_geometry(tire, f"{prefix}_tire"),
        mesh_from_geometry(wheel, f"{prefix}_hub"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_skateboard")

    deck_core = model.material("molded_olive_deck", rgba=(0.13, 0.16, 0.11, 1.0))
    grip_mat = model.material("coarse_black_grip", rgba=(0.018, 0.018, 0.016, 1.0))
    armor_mat = model.material("scuffed_edge_armor", rgba=(0.20, 0.24, 0.18, 1.0))
    truck_mat = model.material("matte_gunmetal_truck", rgba=(0.28, 0.29, 0.27, 1.0))
    hardware_mat = model.material("black_zinc_hardware", rgba=(0.035, 0.036, 0.038, 1.0))
    bushing_mat = model.material("orange_urethane_bushings", rgba=(0.90, 0.36, 0.08, 1.0))
    tire_mat = model.material("black_block_rubber", rgba=(0.012, 0.011, 0.010, 1.0))
    hub_mat = model.material("reinforced_orange_hubs", rgba=(0.86, 0.24, 0.05, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_capsule_deck(0.86, 0.25, 0.032), "deck_shell"),
        material=deck_core,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_cadquery(_capsule_deck(0.66, 0.185, 0.002), "grip_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.0168)),
        material=grip_mat,
        name="grip_pad",
    )

    # Molded sacrificial rails and under-truck reinforcement pads make the deck
    # read as a heavy-service utility board rather than a light street deck.
    for y, name in ((0.116, "edge_rail_0"), (-0.116, "edge_rail_1")):
        deck.visual(
            Box((0.54, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.022)),
            material=armor_mat,
            name=name,
        )
    for x, name in ((0.285, "front_underpad"), (-0.285, "rear_underpad")):
        deck.visual(
            Box((0.150, 0.106, 0.008)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=armor_mat,
            name=name,
        )
        for i, dx in enumerate((-0.034, 0.034)):
            for j, dy in enumerate((-0.030, 0.030)):
                deck.visual(
                    Cylinder(radius=0.0065, length=0.004),
                    origin=Origin(xyz=(x + dx, dy, 0.0188), rpy=(0.0, 0.0, 0.0)),
                    material=hardware_mat,
                    name=f"{name}_bolt_{i}_{j}",
                )

    def make_truck(prefix: str, x: float):
        base = model.part(f"{prefix}_baseplate")
        base.visual(
            Box((0.118, 0.086, 0.012)),
            origin=Origin(),
            material=truck_mat,
            name="plate",
        )
        base.visual(
            Box((0.075, 0.055, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
            material=truck_mat,
            name="raised_boss",
        )
        base.visual(
            Cylinder(radius=0.019, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.021)),
            material=truck_mat,
            name="pivot_cup",
        )
        base.visual(
            Cylinder(radius=0.0155, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=bushing_mat,
            name="upper_bushing",
        )
        base.visual(
            Cylinder(radius=0.0135, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.049)),
            material=bushing_mat,
            name="lower_bushing",
        )
        base.visual(
            Cylinder(radius=0.0048, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, -0.034)),
            material=hardware_mat,
            name="kingpin",
        )
        for i, dx in enumerate((-0.034, 0.034)):
            for j, dy in enumerate((-0.030, 0.030)):
                base.visual(
                    Cylinder(radius=0.0055, length=0.004),
                    origin=Origin(xyz=(dx, dy, 0.008)),
                    material=hardware_mat,
                    name=f"mount_nut_{i}_{j}",
                )

        parent_pad = "front_underpad" if prefix == "front" else "rear_underpad"
        model.articulation(
            f"deck_to_{prefix}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=base,
            origin=Origin(xyz=(x, 0.0, -0.030)),
        )

        hanger = model.part(f"{prefix}_hanger")
        hanger.visual(
            Box((0.068, 0.145, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=truck_mat,
            name="hanger_body",
        )
        hanger.visual(
            Cylinder(radius=0.0070, length=0.372),
            origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware_mat,
            name="axle",
        )
        hanger.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=truck_mat,
            name="kingpin_seat",
        )
        for y, name in ((0.055, "side_rib_0"), (-0.055, "side_rib_1")):
            hanger.visual(
                Box((0.020, 0.050, 0.038)),
                origin=Origin(xyz=(0.0, y, -0.011), rpy=(0.0, 0.0, 0.0)),
                material=truck_mat,
                name=name,
            )

        model.articulation(
            f"{prefix}_steer",
            ArticulationType.REVOLUTE,
            parent=base,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, -0.064)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.28, upper=0.28),
            motion_properties=MotionProperties(damping=0.35, friction=0.18),
        )

        tire_mesh, hub_mesh = _make_wheel_meshes(prefix)
        for idx, y in enumerate((-0.182, 0.182)):
            wheel = model.part(f"{prefix}_wheel_{idx}")
            wheel.visual(tire_mesh, material=tire_mat, name="block_tire")
            wheel.visual(hub_mesh, material=hub_mat, name="spoked_hub")
            cap_x = -0.019 if y < 0.0 else 0.019
            wheel.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(cap_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hardware_mat,
                name="bearing_cap",
            )
            model.articulation(
                f"{prefix}_wheel_{idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, y, -0.006), rpy=(0.0, 0.0, math.pi / 2.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=6.0, velocity=35.0),
                motion_properties=MotionProperties(damping=0.02, friction=0.015),
            )

        return base, hanger, parent_pad

    make_truck("front", 0.285)
    make_truck("rear", -0.285)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    for prefix, pad_name in (("front", "front_underpad"), ("rear", "rear_underpad")):
        base = object_model.get_part(f"{prefix}_baseplate")
        hanger = object_model.get_part(f"{prefix}_hanger")
        steer = object_model.get_articulation(f"{prefix}_steer")

        ctx.expect_contact(
            deck,
            base,
            elem_a=pad_name,
            elem_b="plate",
            contact_tol=0.001,
            name=f"{prefix} baseplate clamps underpad",
        )
        ctx.expect_overlap(
            deck,
            base,
            axes="xy",
            elem_a=pad_name,
            elem_b="plate",
            min_overlap=0.075,
            name=f"{prefix} baseplate footprint is inside reinforced pad",
        )
        ctx.expect_gap(
            deck,
            hanger,
            axis="z",
            min_gap=0.020,
            name=f"{prefix} hanger clears the rugged deck underside",
        )

        rest_wheel = object_model.get_part(f"{prefix}_wheel_1")
        rest_pos = ctx.part_world_position(rest_wheel)
        with ctx.pose({steer: 0.22}):
            turned_pos = ctx.part_world_position(rest_wheel)
        ctx.check(
            f"{prefix} truck steer yaws the axle",
            rest_pos is not None
            and turned_pos is not None
            and abs(turned_pos[0] - rest_pos[0]) > 0.025,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

        for idx in (0, 1):
            wheel = object_model.get_part(f"{prefix}_wheel_{idx}")
            ctx.allow_overlap(
                hanger,
                wheel,
                elem_a="axle",
                elem_b="spoked_hub",
                reason="The through-axle is intentionally seated through the wheel bearing bore.",
            )
            ctx.expect_overlap(
                hanger,
                wheel,
                axes="xz",
                elem_a="axle",
                elem_b="spoked_hub",
                min_overlap=0.010,
                name=f"{prefix} wheel {idx} hub is centered on axle",
            )
            ctx.expect_overlap(
                hanger,
                wheel,
                axes="y",
                elem_a="axle",
                elem_b="spoked_hub",
                min_overlap=0.018,
                name=f"{prefix} wheel {idx} axle remains inserted through hub",
            )
            ctx.expect_gap(
                deck,
                wheel,
                axis="z",
                min_gap=0.010,
                name=f"{prefix} wheel {idx} clears deck at rest",
            )

    return ctx.report()


object_model = build_object_model()
