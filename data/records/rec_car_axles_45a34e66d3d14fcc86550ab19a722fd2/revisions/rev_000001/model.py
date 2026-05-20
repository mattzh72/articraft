from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_x(length: float, outer_radius: float, inner_radius: float, center_x: float = 0.0):
    """CadQuery annular cylinder whose axis is the model X axis."""

    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def _origin_x_cylinder(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Origin for an SDK cylinder rotated so its local Z axis lies along X."""

    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_floating_rear_axle")

    cast_iron = Material("dark cast iron", rgba=(0.08, 0.085, 0.08, 1.0))
    black_oxide = Material("black brake drum", rgba=(0.015, 0.014, 0.012, 1.0))
    machined_steel = Material("machined steel", rgba=(0.72, 0.70, 0.66, 1.0))
    dull_steel = Material("dull bearing steel", rgba=(0.48, 0.50, 0.50, 1.0))
    rust_tint = Material("worn drum edge", rgba=(0.32, 0.16, 0.07, 1.0))
    red_paint = Material("red index paint", rgba=(0.8, 0.04, 0.02, 1.0))

    housing = model.part("axle_housing")

    # Cast central differential carrier ("pumpkin") with cover, pinion nose,
    # welded sockets, hollow axle tubes, and fixed bearing races at the tube ends.
    housing.visual(Sphere(0.30), material=cast_iron, name="differential_bowl")
    housing.visual(
        Cylinder(radius=0.225, length=0.052),
        origin=Origin(xyz=(0.0, -0.298, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="rear_cover",
    )
    housing.visual(
        Cylinder(radius=0.135, length=0.18),
        origin=Origin(xyz=(0.0, 0.350, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pinion_nose",
    )
    housing.visual(
        Cylinder(radius=0.085, length=0.060),
        origin=Origin(xyz=(0.0, 0.470, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pinion_flange",
    )

    for index, sign in enumerate((-1.0, 1.0)):
        # Hollow sockets and tubes keep the hidden rotating axle shafts clear,
        # instead of treating the tubes as solid proxy cylinders.
        housing.visual(
            mesh_from_cadquery(
                _annular_x(0.20, 0.118, 0.052, sign * 0.385),
                f"tube_socket_{index}",
            ),
            material=cast_iron,
            name=f"tube_socket_{index}",
        )
        housing.visual(
            mesh_from_cadquery(
                _annular_x(0.66, 0.074, 0.047, sign * 0.735),
                f"tube_{index}",
            ),
            material=cast_iron,
            name=f"tube_{index}",
        )
        housing.visual(
            mesh_from_cadquery(
                _annular_x(0.115, 0.106, 0.055, sign * 1.085),
                f"bearing_race_{index}",
            ),
            material=dull_steel,
            name=f"bearing_race_{index}",
        )
        housing.visual(
            mesh_from_cadquery(
                _annular_x(0.050, 0.103, 0.058, sign * 1.020),
                f"tube_end_flange_{index}",
            ),
            material=cast_iron,
            name=f"tube_end_flange_{index}",
        )

    for bolt in range(10):
        angle = 2.0 * math.pi * bolt / 10.0
        housing.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(
                xyz=(0.175 * math.cos(angle), -0.328, 0.175 * math.sin(angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name=f"cover_bolt_{bolt}",
        )

    for index, sign in enumerate((-1.0, 1.0)):
        shaft = model.part(f"shaft_{index}")
        shaft.visual(
            Cylinder(radius=0.027, length=0.66),
            origin=_origin_x_cylinder(0.0),
            material=machined_steel,
            name="shaft_core",
        )
        shaft.visual(
            Box((0.54, 0.007, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            material=dull_steel,
            name="shaft_key",
        )
        shaft.visual(
            Cylinder(radius=0.056, length=0.070),
            origin=_origin_x_cylinder(sign * 0.365),
            material=machined_steel,
            name="outer_spline",
        )
        model.articulation(
            f"housing_to_shaft_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=shaft,
            origin=Origin(xyz=(sign * 0.735, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=160.0, velocity=45.0),
        )

        hub = model.part(f"hub_drum_{index}")
        hub.visual(
            mesh_from_cadquery(_annular_x(0.180, 0.132, 0.104, sign * 0.020), f"hub_shell_{index}"),
            material=machined_steel,
            name="hub_shell",
        )
        hub.visual(
            mesh_from_cadquery(_annular_x(0.200, 0.232, 0.182, sign * 0.145), f"drum_band_{index}"),
            material=black_oxide,
            name="drum_band",
        )
        hub.visual(
            mesh_from_cadquery(_annular_x(0.038, 0.188, 0.060, sign * 0.095), f"web_flange_{index}"),
            material=black_oxide,
            name="web_flange",
        )
        hub.visual(
            mesh_from_cadquery(_annular_x(0.205, 0.095, 0.050, sign * 0.170), f"hub_nose_{index}"),
            material=machined_steel,
            name="hub_nose",
        )
        hub.visual(
            mesh_from_cadquery(_annular_x(0.036, 0.150, 0.050, sign * 0.255), f"wheel_flange_{index}"),
            material=machined_steel,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.075, length=0.032),
            origin=_origin_x_cylinder(sign * 0.288),
            material=machined_steel,
            name="dust_cap",
        )
        hub.visual(
            Box((0.055, 0.030, 0.014)),
            origin=Origin(xyz=(sign * 0.145, 0.0, 0.234)),
            material=red_paint,
            name="index_mark",
        )
        hub.visual(
            Cylinder(radius=0.234, length=0.012),
            origin=_origin_x_cylinder(sign * 0.045),
            material=rust_tint,
            name="inboard_drum_lip",
        )

        for stud in range(6):
            angle = 2.0 * math.pi * stud / 6.0 + math.pi / 6.0
            hub.visual(
                Cylinder(radius=0.012, length=0.052),
                origin=_origin_x_cylinder(
                    sign * 0.289,
                    0.115 * math.cos(angle),
                    0.115 * math.sin(angle),
                ),
                material=machined_steel,
                name=f"wheel_stud_{stud}",
            )

        model.articulation(
            f"housing_to_hub_drum_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=hub,
            origin=Origin(xyz=(sign * 1.105, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=240.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("axle_housing")

    def center_of(bounds):
        if bounds is None:
            return None
        low, high = bounds
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    for index in (0, 1):
        shaft = object_model.get_part(f"shaft_{index}")
        hub = object_model.get_part(f"hub_drum_{index}")
        shaft_joint = object_model.get_articulation(f"housing_to_shaft_{index}")
        hub_joint = object_model.get_articulation(f"housing_to_hub_drum_{index}")

        ctx.allow_overlap(
            housing,
            shaft,
            elem_a=f"bearing_race_{index}",
            elem_b="outer_spline",
            reason=(
                "The splined shaft journal is intentionally captured in the "
                "stationary bearing-race proxy so the rotating shaft is visibly supported."
            ),
        )
        ctx.allow_overlap(
            housing,
            hub,
            elem_a=f"bearing_race_{index}",
            elem_b="hub_shell",
            reason=(
                "The hub shell is intentionally represented as a rotating bearing "
                "member seated around the fixed race at the tube end."
            ),
        )
        ctx.expect_overlap(
            shaft,
            housing,
            axes="x",
            elem_a="outer_spline",
            elem_b=f"bearing_race_{index}",
            min_overlap=0.050,
            name=f"shaft_{index} journal remains captured by bearing race",
        )
        ctx.expect_within(
            shaft,
            housing,
            axes="yz",
            inner_elem="shaft_core",
            outer_elem=f"tube_{index}",
            margin=0.0,
            name=f"shaft_{index} sits inside tube bore envelope",
        )
        ctx.expect_overlap(
            shaft,
            housing,
            axes="x",
            elem_a="shaft_core",
            elem_b=f"tube_{index}",
            min_overlap=0.56,
            name=f"shaft_{index} runs through axle tube",
        )
        ctx.expect_overlap(
            hub,
            housing,
            axes="x",
            elem_a="hub_shell",
            elem_b=f"bearing_race_{index}",
            min_overlap=0.070,
            name=f"hub_drum_{index} surrounds bearing race axially",
        )
        ctx.expect_within(
            housing,
            hub,
            axes="yz",
            inner_elem=f"bearing_race_{index}",
            outer_elem="hub_shell",
            margin=0.0,
            name=f"bearing_race_{index} is coaxial with hub shell",
        )

        shaft_rest = center_of(ctx.part_element_world_aabb(shaft, elem="shaft_key"))
        hub_rest = center_of(ctx.part_element_world_aabb(hub, elem="index_mark"))

        with ctx.pose({shaft_joint: math.pi / 2.0, hub_joint: math.pi / 2.0}):
            shaft_rotated = center_of(ctx.part_element_world_aabb(shaft, elem="shaft_key"))
            hub_rotated = center_of(ctx.part_element_world_aabb(hub, elem="index_mark"))

        ctx.check(
            f"shaft_{index} key orbits inside tube",
            shaft_rest is not None
            and shaft_rotated is not None
            and abs(shaft_rotated[1] - shaft_rest[1]) > 0.020
            and shaft_rotated[2] < shaft_rest[2] - 0.015,
            details=f"rest={shaft_rest}, rotated={shaft_rotated}",
        )
        ctx.check(
            f"hub_drum_{index} index mark rotates on bearing",
            hub_rest is not None
            and hub_rotated is not None
            and abs(hub_rotated[1] - hub_rest[1]) > 0.18
            and hub_rotated[2] < hub_rest[2] - 0.16,
            details=f"rest={hub_rest}, rotated={hub_rotated}",
        )

    return ctx.report()


object_model = build_object_model()
