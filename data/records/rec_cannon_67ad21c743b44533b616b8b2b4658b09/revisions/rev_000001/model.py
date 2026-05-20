from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireGeometry,
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


BARREL_PIVOT_X = 0.0
BARREL_PIVOT_Z = 0.82
BARREL_REST_ELEVATION = math.radians(5.0)
WHEEL_CENTER_X = -0.24
WHEEL_CENTER_Y = 0.60
WHEEL_RADIUS = 0.43
WHEEL_WIDTH = 0.12


def _cylinder_along_x(radius: float, length: float, center_x: float):
    return (
        cq.Workplane("YZ", origin=(center_x - length / 2.0, 0.0, 0.0))
        .circle(radius)
        .extrude(length)
    )


def _frustum_along_x(radius_a: float, radius_b: float, length: float, start_x: float):
    return (
        cq.Workplane("YZ", origin=(start_x, 0.0, 0.0))
        .circle(radius_a)
        .workplane(offset=length)
        .circle(radius_b)
        .loft(combine=True)
    )


def _build_barrel_shell():
    """A thick early iron tube with reinforcing rings and an open muzzle bore."""
    barrel = _frustum_along_x(0.215, 0.145, 1.55, -0.55)
    barrel = barrel.union(_cylinder_along_x(0.230, 0.34, -0.58))
    barrel = barrel.union(_cylinder_along_x(0.185, 0.16, -0.20))
    barrel = barrel.union(_cylinder_along_x(0.170, 0.14, 0.50))
    barrel = barrel.union(_cylinder_along_x(0.190, 0.24, 1.12))
    barrel = barrel.union(_frustum_along_x(0.170, 0.205, 0.16, 1.22))

    # Cut a real axial bore from just ahead of the closed breech through the muzzle.
    bore = _cylinder_along_x(0.078, 1.88, 0.52)
    return barrel.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_iron_cannon")

    old_iron = model.material("blackened_iron", rgba=(0.05, 0.052, 0.05, 1.0))
    soot = model.material("sooty_bore", rgba=(0.005, 0.004, 0.003, 1.0))
    dark_iron = model.material("iron_tire", rgba=(0.025, 0.026, 0.026, 1.0))
    aged_oak = model.material("aged_oak", rgba=(0.48, 0.28, 0.12, 1.0))
    end_grain = model.material("worn_end_grain", rgba=(0.58, 0.39, 0.20, 1.0))

    carriage = model.part("carriage")

    # Sloped timber side cheeks: two heavy rails rising from a rear skid to the trunnions.
    rail_pitch = -math.atan2(0.46, 1.65)
    for idx, side, rail_name, cheek_name, saddle_name in (
        (0, -1.0, "side_rail_0", "cheek_block_0", "trunnion_saddle_0"),
        (1, 1.0, "side_rail_1", "cheek_block_1", "trunnion_saddle_1"),
    ):
        y = side * 0.31
        carriage.visual(
            Box((1.72, 0.14, 0.17)),
            origin=Origin(xyz=(-0.175, y, 0.45), rpy=(0.0, rail_pitch, 0.0)),
            material=aged_oak,
            name=rail_name,
        )
        carriage.visual(
            Box((0.38, 0.15, 0.16)),
            origin=Origin(xyz=(0.02, y, 0.555), rpy=(0.0, rail_pitch, 0.0)),
            material=aged_oak,
            name=cheek_name,
        )
        # Flat-topped bearing saddles sit directly under the barrel trunnion pin.
        carriage.visual(
            Box((0.24, 0.17, 0.16)),
            origin=Origin(xyz=(BARREL_PIVOT_X, y, BARREL_PIVOT_Z - 0.155)),
            material=aged_oak,
            name=saddle_name,
        )

    # Cross timbers and rear ground shoe tie the two side rails into one carriage.
    carriage.visual(
        Box((0.22, 0.86, 0.13)),
        origin=Origin(xyz=(0.48, 0.0, 0.50), rpy=(0.0, rail_pitch, 0.0)),
        material=aged_oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.20, 0.82, 0.13)),
        origin=Origin(xyz=(-0.72, 0.0, 0.30), rpy=(0.0, rail_pitch, 0.0)),
        material=aged_oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.36, 0.74, 0.14)),
        origin=Origin(xyz=(-1.02, 0.0, 0.07)),
        material=end_grain,
        name="rear_skid",
    )
    carriage.visual(
        Box((0.52, 0.18, 0.12)),
        origin=Origin(xyz=(-0.70, 0.0, 0.23), rpy=(0.0, rail_pitch, 0.0)),
        material=aged_oak,
        name="trail_tongue",
    )

    # A fixed axle passes through the wheel hubs.
    carriage.visual(
        Cylinder(radius=0.0575, length=1.42),
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, WHEEL_RADIUS), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_oak,
        name="wheel_axle",
    )
    for side in (-1.0, 1.0):
        carriage.visual(
            Cylinder(radius=0.080, length=0.05),
            origin=Origin(
                xyz=(WHEEL_CENTER_X, side * (WHEEL_CENTER_Y + WHEEL_WIDTH / 2.0 + 0.045), WHEEL_RADIUS),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"axle_cap_{0 if side < 0 else 1}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel_shell(), "barrel_shell", tolerance=0.0015, angular_tolerance=0.08),
        material=old_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.075, length=0.92),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=old_iron,
        name="trunnion_pin",
    )
    barrel.visual(
        Cylinder(radius=0.085, length=0.16),
        origin=Origin(xyz=(-0.80, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=old_iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.105),
        origin=Origin(xyz=(-0.91, 0.0, 0.0)),
        material=old_iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.080, length=0.11),
        origin=Origin(xyz=(1.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soot,
        name="dark_muzzle_bore",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.385,
            WHEEL_WIDTH * 0.82,
            rim=WheelRim(inner_radius=0.285, flange_height=0.018, flange_thickness=0.012),
            hub=WheelHub(radius=0.092, width=WHEEL_WIDTH * 0.85, cap_style="domed"),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.030, window_radius=0.070),
            bore=WheelBore(style="round", diameter=0.115),
        ),
        "wooden_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.386,
            tread=TireTread(style="circumferential", depth=0.004, count=1),
            sidewall=TireSidewall(style="square", bulge=0.01),
        ),
        "iron_wheel_hoop",
    )
    for idx, side in enumerate((-1.0, 1.0)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=aged_oak,
            name="spoked_wheel",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dark_iron,
            name="iron_hoop",
        )
        model.articulation(
            f"carriage_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(WHEEL_CENTER_X, side * WHEEL_CENTER_Y, WHEEL_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=6.0),
        )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(
            xyz=(BARREL_PIVOT_X, 0.0, BARREL_PIVOT_Z),
            rpy=(0.0, -BARREL_REST_ELEVATION, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.6, lower=-0.18, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    barrel_joint = object_model.get_articulation("carriage_to_barrel")

    ctx.check("cannon_has_main_parts", all(p is not None for p in (carriage, barrel, wheel_0, wheel_1)))
    ctx.check("barrel_has_trunnion_joint", barrel_joint is not None)

    if carriage is not None and barrel is not None:
        ctx.expect_gap(
            barrel,
            carriage,
            axis="z",
            positive_elem="trunnion_pin",
            negative_elem="trunnion_saddle_0",
            max_gap=0.008,
            max_penetration=0.002,
            name="trunnion rests on saddle",
        )

    if wheel_0 is not None and wheel_1 is not None:
        for idx, wheel in enumerate((wheel_0, wheel_1)):
            ctx.allow_overlap(
                carriage,
                wheel,
                elem_a="wheel_axle",
                elem_b="spoked_wheel",
                reason="The wooden axle is intentionally captured through the wheel hub bore proxy.",
            )
            ctx.expect_within(
                carriage,
                wheel,
                axes="xz",
                inner_elem="wheel_axle",
                outer_elem="spoked_wheel",
                margin=0.002,
                name=f"wheel_{idx}_axle_centered_in_hub",
            )
            ctx.expect_overlap(
                carriage,
                wheel,
                axes="y",
                elem_a="wheel_axle",
                elem_b="spoked_wheel",
                min_overlap=0.09,
                name=f"wheel_{idx}_hub_retained_on_axle",
            )
            aabb = ctx.part_world_aabb(wheel)
            if aabb is not None:
                mins, maxs = aabb
                ctx.check(
                    f"wheel_{idx}_touches_ground",
                    abs(float(mins[2])) < 0.015,
                    details=f"wheel AABB min z={float(mins[2]):.4f}",
                )
                ctx.check(
                    f"wheel_{idx}_large_diameter",
                    0.82 <= float(maxs[2] - mins[2]) <= 0.90,
                    details=f"wheel diameter={float(maxs[2] - mins[2]):.4f}",
                )

    if barrel_joint is not None and barrel is not None:
        rest_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
        with ctx.pose({barrel_joint: 0.32}):
            elevated_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
        if rest_aabb is not None and elevated_aabb is not None:
            rest_muzzle_height = float(rest_aabb[1][2])
            elevated_muzzle_height = float(elevated_aabb[1][2])
            ctx.check(
                "barrel_elevates_on_trunnions",
                elevated_muzzle_height > rest_muzzle_height + 0.10,
                details=f"rest top={rest_muzzle_height:.3f}, elevated top={elevated_muzzle_height:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
