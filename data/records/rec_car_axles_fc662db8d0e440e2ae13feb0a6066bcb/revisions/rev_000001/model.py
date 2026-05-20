from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


AXLE_Z = 0.20


def _circle_profile(radius: float, *, segments: int = 48, center=(0.0, 0.0)):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _sprocket_plate_geometry() -> ExtrudeWithHolesGeometry:
    tooth_count = 36
    outer_radius = 0.132
    root_radius = 0.116
    profile = []
    for tooth in range(tooth_count):
        base = 2.0 * math.pi * tooth / tooth_count
        for frac, radius in (
            (0.00, root_radius),
            (0.22, outer_radius),
            (0.50, outer_radius),
            (0.78, root_radius),
        ):
            a = base + frac * 2.0 * math.pi / tooth_count
            profile.append((radius * math.cos(a), radius * math.sin(a)))

    holes = [_circle_profile(0.020, segments=48)]
    for i in range(6):
        a = 2.0 * math.pi * i / 6.0
        holes.append(
            _circle_profile(
                0.014,
                segments=24,
                center=(0.066 * math.cos(a), 0.066 * math.sin(a)),
            )
        )

    return ExtrudeWithHolesGeometry(profile, holes, 0.012, center=True)


def _pillow_housing_cq():
    width_x = 0.110
    base = cq.Workplane("XY").box(0.210, 0.145, 0.022).translate((0.0, 0.0, -0.139))
    pedestal = cq.Workplane("XY").box(width_x, 0.098, 0.095).translate((0.0, 0.0, -0.090))
    boss = cq.Workplane("YZ").circle(0.078).extrude(width_x, both=True)
    housing = base.union(pedestal).union(boss)
    bearing_seat = cq.Workplane("YZ").circle(0.049).extrude(width_x + 0.030, both=True)
    return housing.cut(bearing_seat)


def _bearing_outer_cq():
    outer = cq.Workplane("YZ").circle(0.050).extrude(0.070, both=True)
    inner = cq.Workplane("YZ").circle(0.029).extrude(0.090, both=True)
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="go_kart_live_axle")

    black = model.material("powder_coated_black", rgba=(0.02, 0.018, 0.016, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    zinc = model.material("zinc_plated_fasteners", rgba=(0.78, 0.76, 0.68, 1.0))
    red = model.material("anodized_red_hubs", rgba=(0.78, 0.07, 0.035, 1.0))
    sprocket_mat = model.material("oiled_sprocket_steel", rgba=(0.23, 0.22, 0.19, 1.0))

    carrier = model.part("rear_carrier")
    carrier.visual(
        Box((0.920, 0.100, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=black,
        name="rear_rail",
    )

    housing_mesh = mesh_from_cadquery(_pillow_housing_cq(), "pillow_housing", tolerance=0.0008)
    bearing_mesh = mesh_from_cadquery(_bearing_outer_cq(), "bearing_outer", tolerance=0.0008)
    for idx, x in enumerate((-0.325, 0.325)):
        carrier.visual(
            housing_mesh,
            origin=Origin(xyz=(x, 0.0, AXLE_Z)),
            material=black,
            name=f"pillow_block_{idx}",
        )
        carrier.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, AXLE_Z)),
            material=dark,
            name=f"bearing_outer_{idx}",
        )
        for bx in (-0.070, 0.070):
            for by in (-0.045, 0.045):
                carrier.visual(
                    Cylinder(radius=0.008, length=0.026),
                    origin=Origin(xyz=(x + bx, by, 0.059)),
                    material=zinc,
                    name=f"base_bolt_{idx}_{0 if bx < 0 else 1}_{0 if by < 0 else 1}",
                )

    live_axle = model.part("live_axle")
    live_axle.visual(
        Cylinder(radius=0.015, length=1.300),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_tube",
    )
    live_axle.visual(
        Box((0.300, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="axle_keyway",
    )

    for idx, x in enumerate((-0.325, 0.325)):
        live_axle.visual(
            Cylinder(radius=0.024, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"bearing_inner_{idx}",
        )
        collar_x = x + (-0.070 if x < 0.0 else 0.070)
        live_axle.visual(
            Cylinder(radius=0.027, length=0.024),
            origin=Origin(xyz=(collar_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"lock_collar_{idx}",
        )
        live_axle.visual(
            Box((0.010, 0.010, 0.006)),
            origin=Origin(xyz=(collar_x, -0.026, 0.0)),
            material=zinc,
            name=f"collar_screw_{idx}",
        )

    sprocket_mesh = mesh_from_geometry(_sprocket_plate_geometry(), "chain_sprocket")
    live_axle.visual(
        sprocket_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sprocket_mat,
        name="sprocket_plate",
    )
    live_axle.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="sprocket_hub",
    )
    for i in range(6):
        a = 2.0 * math.pi * i / 6.0
        live_axle.visual(
            Cylinder(radius=0.0045, length=0.018),
            origin=Origin(
                xyz=(0.012, 0.048 * math.cos(a), 0.048 * math.sin(a)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"sprocket_bolt_{i}",
        )

    for idx, side in enumerate((-1.0, 1.0)):
        hub_x = side * 0.535
        flange_x = side * 0.595
        stud_x = side * 0.612
        live_axle.visual(
            Cylinder(radius=0.036, length=0.115),
            origin=Origin(xyz=(hub_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=red,
            name=f"hub_barrel_{idx}",
        )
        live_axle.visual(
            Cylinder(radius=0.076, length=0.018),
            origin=Origin(xyz=(flange_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=red,
            name=f"hub_flange_{idx}",
        )
        for j in range(4):
            a = math.pi / 4.0 + j * math.pi / 2.0
            live_axle.visual(
                Cylinder(radius=0.005, length=0.040),
                origin=Origin(
                    xyz=(stud_x, 0.055 * math.cos(a), 0.055 * math.sin(a)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=zinc,
                name=f"hub_stud_{idx}_{j}",
            )

    model.articulation(
        "carrier_to_live_axle",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=live_axle,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    carrier = object_model.get_part("rear_carrier")
    live_axle = object_model.get_part("live_axle")
    spin = object_model.get_articulation("carrier_to_live_axle")

    ctx.check(
        "axle has continuous bearing rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "rotation axis follows axle tube",
        abs(spin.axis[0] - 1.0) < 1e-6 and abs(spin.axis[1]) < 1e-6 and abs(spin.axis[2]) < 1e-6,
        details=f"axis={spin.axis}",
    )

    rotating_visuals = {visual.name for visual in live_axle.visuals}
    ctx.check(
        "hubs and sprocket are carried by live axle",
        {"hub_barrel_0", "hub_barrel_1", "sprocket_plate", "axle_tube"}.issubset(rotating_visuals),
        details=f"rotating_visuals={sorted(rotating_visuals)}",
    )
    ctx.expect_overlap(
        live_axle,
        carrier,
        axes="x",
        elem_a="axle_tube",
        elem_b="bearing_outer_0",
        min_overlap=0.055,
        name="axle passes through first pillow block",
    )
    ctx.expect_overlap(
        live_axle,
        carrier,
        axes="x",
        elem_a="axle_tube",
        elem_b="bearing_outer_1",
        min_overlap=0.055,
        name="axle passes through second pillow block",
    )

    with ctx.pose({spin: 0.0}):
        sprocket_aabb = ctx.part_element_world_aabb(live_axle, elem="sprocket_plate")
        keyway_rest = ctx.part_element_world_aabb(live_axle, elem="axle_keyway")
    with ctx.pose({spin: math.pi / 2.0}):
        keyway_quarter = ctx.part_element_world_aabb(live_axle, elem="axle_keyway")

    if sprocket_aabb is not None:
        sprocket_center_x = 0.5 * (sprocket_aabb[0][0] + sprocket_aabb[1][0])
        ctx.check(
            "sprocket is mounted near mid-span",
            abs(sprocket_center_x) < 0.015,
            details=f"sprocket_center_x={sprocket_center_x:.4f}",
        )
    else:
        ctx.fail("sprocket is mounted near mid-span", "missing sprocket_plate AABB")

    if keyway_rest is not None and keyway_quarter is not None:
        rest_center = (
            0.5 * (keyway_rest[0][1] + keyway_rest[1][1]),
            0.5 * (keyway_rest[0][2] + keyway_rest[1][2]),
        )
        quarter_center = (
            0.5 * (keyway_quarter[0][1] + keyway_quarter[1][1]),
            0.5 * (keyway_quarter[0][2] + keyway_quarter[1][2]),
        )
        ctx.check(
            "visible keyway rotates with live axle",
            abs(rest_center[1] - AXLE_Z) > 0.010
            and abs(quarter_center[0]) > 0.010
            and abs(quarter_center[1] - AXLE_Z) < 0.006,
            details=f"rest_yz={rest_center}, quarter_yz={quarter_center}",
        )
    else:
        ctx.fail("visible keyway rotates with live axle", "missing axle_keyway AABB")

    return ctx.report()


object_model = build_object_model()
