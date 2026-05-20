from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
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
    ExtrudeGeometry,
)


BOARD_LENGTH = 0.86
BOARD_WIDTH = 0.255
DECK_THICKNESS = 0.026
TRUCK_X = 0.265
STEER_Z = -0.060
AXLE_Z = -0.040
WHEEL_Y = 0.168


def _deck_mesh(name: str, width: float, length: float, thickness: float):
    """Rounded, capsule-like skateboard deck/plate in the local XY plane."""
    profile = rounded_rect_profile(length, width, radius=width * 0.48, corner_segments=14)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _add_cylinder(part, name, radius, length, xyz, rpy, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_skateboard")

    deck_mat = Material("sealed_maple", color=(0.54, 0.34, 0.16, 1.0))
    grip_mat = Material("coarse_black_grip", color=(0.015, 0.016, 0.014, 1.0))
    rail_mat = Material("replaceable_black_hdpe", color=(0.035, 0.035, 0.033, 1.0))
    wear_mat = Material("brushed_wear_plate", color=(0.62, 0.63, 0.59, 1.0))
    steel_mat = Material("zinc_plated_steel", color=(0.72, 0.74, 0.70, 1.0))
    dark_steel = Material("dark_oxide_steel", color=(0.18, 0.19, 0.18, 1.0))
    alloy_mat = Material("bead_blasted_aluminum", color=(0.55, 0.57, 0.54, 1.0))
    rubber_mat = Material("serviceable_rubber", color=(0.05, 0.045, 0.04, 1.0))
    urethane_mat = Material("amber_urethane", color=(0.95, 0.58, 0.16, 1.0))
    warning_mat = Material("painted_yellow_witness", color=(1.0, 0.72, 0.06, 1.0))

    deck = model.part("deck")
    deck.visual(
        _deck_mesh("deck_shell", BOARD_WIDTH, BOARD_LENGTH, DECK_THICKNESS),
        material=deck_mat,
        name="deck_shell",
    )
    # Three replaceable grip pads leave the bolt fields open for a driver.
    for i, (x, sx) in enumerate(((-0.365, 0.12), (0.0, 0.28), (0.365, 0.12))):
        deck.visual(
            Box((sx, 0.205, 0.002)),
            origin=Origin(xyz=(x, 0.0, DECK_THICKNESS / 2 + 0.0008)),
            material=grip_mat,
            name=f"grip_pad_{i}",
        )
    for side, y in enumerate((-BOARD_WIDTH / 2 - 0.006, BOARD_WIDTH / 2 + 0.006)):
        deck.visual(
            Box((0.66, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, y, -DECK_THICKNESS / 2 - 0.004)),
            material=rail_mat,
            name=f"side_rail_{side}",
        )
    for i, x in enumerate((-0.38, 0.38)):
        deck.visual(
            Box((0.14, 0.20, 0.004)),
            origin=Origin(xyz=(x, 0.0, -DECK_THICKNESS / 2 - 0.0018)),
            material=wear_mat,
            name=f"skid_plate_{i}",
        )
    for truck_index, x0 in enumerate((-TRUCK_X, TRUCK_X)):
        # Service bolt heads and large washers are visible on top of the deck.
        for ix, dx in enumerate((-0.035, 0.035)):
            for iy, y in enumerate((-0.032, 0.032)):
                deck.visual(
                    Cylinder(radius=0.008, length=0.004),
                    origin=Origin(xyz=(x0 + dx, y, DECK_THICKNESS / 2 + 0.003)),
                    material=steel_mat,
                    name=f"bolt_head_{truck_index}_{ix}_{iy}",
                )
        deck.visual(
            Box((0.105, 0.082, 0.0016)),
            origin=Origin(xyz=(x0, 0.0, DECK_THICKNESS / 2 + 0.0009)),
            material=warning_mat,
            name=f"torque_window_{truck_index}",
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.027,
            0.034,
            rim=WheelRim(inner_radius=0.017, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.011,
                width=0.032,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.018, hole_diameter=0.0022),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0022, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "service_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.038,
            0.048,
            inner_radius=0.027,
            carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.0035, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0018),),
            sidewall=TireSidewall(style="square", bulge=0.015),
            shoulder=TireShoulder(width=0.005, radius=0.002),
        ),
        "field_urethane_tire",
    )

    truck_parts = []
    for label, x0, lean_sign in (("rear", -TRUCK_X, 1.0), ("front", TRUCK_X, -1.0)):
        base = model.part(f"{label}_baseplate")
        # Riser pad and baseplate stack contact the underside of the deck.
        base.visual(
            Box((0.122, 0.096, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.0160)),
            material=rubber_mat,
            name="riser_pad",
        )
        base.visual(
            Box((0.108, 0.084, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.0260)),
            material=alloy_mat,
            name="baseplate",
        )
        for ix, dx in enumerate((-0.035, 0.035)):
            for iy, y in enumerate((-0.032, 0.032)):
                _add_cylinder(
                    base,
                    f"locknut_{ix}_{iy}",
                    0.007,
                    0.006,
                    (dx, y, -0.0360),
                    (0.0, 0.0, 0.0),
                    steel_mat,
                )

        # A through-kingpin ties both bushing stacks to the baseplate; the steering
        # joint uses the same slightly-inclined axis.
        lean = math.radians(12.0) * lean_sign
        _add_cylinder(
            base,
            "kingpin_shaft",
            0.0052,
            0.066,
            (0.0, 0.0, -0.061),
            (0.0, lean, 0.0),
            dark_steel,
        )
        _add_cylinder(
            base,
            "upper_bushing",
            0.021,
            0.014,
            (0.0, 0.0, -0.043),
            (0.0, lean, 0.0),
            rubber_mat,
        )
        _add_cylinder(
            base,
            "lower_bushing",
            0.020,
            0.013,
            (0.0, 0.0, -0.077),
            (0.0, lean, 0.0),
            rubber_mat,
        )
        _add_cylinder(
            base,
            "kingpin_nut",
            0.012,
            0.006,
            (0.0, 0.0, -0.087),
            (0.0, lean, 0.0),
            steel_mat,
        )
        base.visual(
            Box((0.040, 0.046, 0.012)),
            origin=Origin(xyz=(0.020 * lean_sign, 0.0, -0.041)),
            material=alloy_mat,
            name="pivot_cup",
        )

        model.articulation(
            f"deck_to_{label}_base",
            ArticulationType.FIXED,
            parent=deck,
            child=base,
            origin=Origin(xyz=(x0, 0.0, 0.0)),
        )

        hanger = model.part(f"{label}_hanger")
        # Central sleeve is squeezed between the two bushings and carries the axle.
        hanger.visual(
            Box((0.068, 0.060, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=alloy_mat,
            name="bushing_yoke",
        )
        hanger.visual(
            Box((0.112, 0.094, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, AXLE_Z - 0.009)),
            material=alloy_mat,
            name="hanger_body",
        )
        _add_cylinder(
            hanger,
            "axle",
            0.0065,
            0.322,
            (0.0, 0.0, AXLE_Z),
            (math.pi / 2, 0.0, 0.0),
            dark_steel,
        )
        for side_index, y in enumerate((-0.130, 0.130)):
            _add_cylinder(
                hanger,
                f"bearing_collar_{side_index}",
                0.012,
                0.010,
                (0.0, y, AXLE_Z),
                (math.pi / 2, 0.0, 0.0),
                steel_mat,
            )
        for rib_index, y in enumerate((-0.033, 0.033)):
            hanger.visual(
                Box((0.026, 0.014, 0.040)),
                origin=Origin(xyz=(0.0, y, -0.026)),
                material=alloy_mat,
                name=f"cheek_rib_{rib_index}",
            )

        steer_axis = (math.sin(lean), 0.0, math.cos(lean))
        model.articulation(
            f"{label}_steer",
            ArticulationType.REVOLUTE,
            parent=base,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, STEER_Z)),
            axis=steer_axis,
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.34, upper=0.34),
            motion_properties=MotionProperties(damping=0.45, friction=0.08),
        )

        truck_parts.append((label, x0, base, hanger))

        for side_index, y in enumerate((-WHEEL_Y, WHEEL_Y)):
            wheel = model.part(f"{label}_wheel_{side_index}")
            wheel.visual(wheel_mesh, material=alloy_mat, name="wheel_core")
            wheel.visual(tire_mesh, material=urethane_mat, name="tire")
            _add_cylinder(
                wheel,
                "bearing_sleeve",
                0.0095,
                0.052,
                (0.0, 0.0, 0.0),
                (0.0, math.pi / 2, 0.0),
                dark_steel,
            )
            # Small outside service nut reads as a replaceable retainer.
            _add_cylinder(
                wheel,
                "axle_nut",
                0.010,
                0.008,
                ((-0.028 if y < 0 else 0.028), 0.0, 0.0),
                (0.0, math.pi / 2, 0.0),
                steel_mat,
            )
            model.articulation(
                f"{label}_wheel_spin_{side_index}",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(0.0, 0.0, math.pi / 2)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=3.0, velocity=60.0),
                motion_properties=MotionProperties(damping=0.02, friction=0.015),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    for label in ("rear", "front"):
        base = object_model.get_part(f"{label}_baseplate")
        hanger = object_model.get_part(f"{label}_hanger")
        steer = object_model.get_articulation(f"{label}_steer")

        ctx.allow_overlap(
            base,
            hanger,
            elem_a="kingpin_shaft",
            elem_b="bushing_yoke",
            reason="The serviceable truck hanger is captured around the through-kingpin between the rubber bushings.",
        )
        ctx.expect_overlap(
            base,
            hanger,
            axes="xyz",
            elem_a="kingpin_shaft",
            elem_b="bushing_yoke",
            min_overlap=0.004,
            name=f"{label} kingpin passes through hanger yoke",
        )
        ctx.expect_gap(
            deck,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem="deck_shell",
            negative_elem="riser_pad",
            name=f"{label} riser pad seats against deck",
        )

        with ctx.pose({steer: 0.28}):
            ctx.expect_overlap(
                base,
                hanger,
                axes="z",
                elem_a="kingpin_shaft",
                elem_b="bushing_yoke",
                min_overlap=0.010,
                name=f"{label} steered hanger remains kingpin captured",
            )

        for side_index in (0, 1):
            wheel = object_model.get_part(f"{label}_wheel_{side_index}")
            spin = object_model.get_articulation(f"{label}_wheel_spin_{side_index}")
            ctx.allow_overlap(
                hanger,
                wheel,
                elem_a="axle",
                elem_b="bearing_sleeve",
                reason="The wheel bearing sleeve intentionally surrounds the steel axle so the wheel is retained while spinning.",
            )
            ctx.expect_overlap(
                hanger,
                wheel,
                axes="xyz",
                elem_a="axle",
                elem_b="bearing_sleeve",
                min_overlap=0.010,
                name=f"{label} wheel {side_index} bearing surrounds axle",
            )
            rest_pos = ctx.part_world_position(wheel)
            with ctx.pose({spin: math.pi}):
                spun_pos = ctx.part_world_position(wheel)
                ctx.expect_overlap(
                    hanger,
                    wheel,
                    axes="xyz",
                    elem_a="axle",
                    elem_b="bearing_sleeve",
                    min_overlap=0.010,
                    name=f"{label} wheel {side_index} spin stays on axle",
                )
            ctx.check(
                f"{label} wheel {side_index} spin is pure rotation",
                rest_pos is not None
                and spun_pos is not None
                and max(abs(rest_pos[i] - spun_pos[i]) for i in range(3)) < 1e-6,
                details=f"rest={rest_pos}, spun={spun_pos}",
            )

    return ctx.report()


object_model = build_object_model()
