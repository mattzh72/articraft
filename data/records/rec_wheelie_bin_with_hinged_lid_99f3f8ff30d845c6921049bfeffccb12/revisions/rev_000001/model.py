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
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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


GREEN = Material("molded_deep_green", rgba=(0.02, 0.26, 0.10, 1.0))
DARK_GREEN = Material("dark_reinforced_green", rgba=(0.01, 0.18, 0.07, 1.0))
BLACK = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
RIM_GRAY = Material("dark_gray_rim", rgba=(0.12, 0.13, 0.12, 1.0))
METAL = Material("dull_steel", rgba=(0.48, 0.47, 0.43, 1.0))


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _wheelie_bin_body_shape() -> cq.Workplane:
    """One connected molded-plastic body: hollow tapered tub, rim, ribs, brackets, feet."""

    bottom_depth = 0.56
    bottom_width = 0.43
    top_depth = 0.72
    top_width = 0.58
    z0 = 0.14
    height = 0.80
    wall = 0.032

    outer = (
        cq.Workplane("XY")
        .rect(bottom_depth, bottom_width)
        .workplane(offset=height)
        .rect(top_depth, top_width)
        .loft(combine=True)
    )
    shell = outer.faces(">Z").shell(-wall).translate((0.0, 0.0, z0))

    # Heavy rolled top frame and molded reinforcement.  These intentionally
    # overlap the shell a little so the exported body is one manufactured part.
    top_z = z0 + height
    parts = [
        shell,
        _box((top_depth / 2.0, 0.0, top_z + 0.014), (0.065, top_width + 0.070, 0.070)),
        _box((-top_depth / 2.0, 0.0, top_z + 0.014), (0.065, top_width + 0.070, 0.070)),
        _box((0.0, top_width / 2.0, top_z + 0.014), (top_depth + 0.070, 0.065, 0.070)),
        _box((0.0, -top_width / 2.0, top_z + 0.014), (top_depth + 0.070, 0.065, 0.070)),
        # Front vertical strengthening ribs.
        _box((top_depth / 2.0 - 0.015, 0.0, z0 + 0.43), (0.055, 0.050, 0.610)),
        _box((top_depth / 2.0 - 0.018, 0.185, z0 + 0.42), (0.050, 0.040, 0.560)),
        _box((top_depth / 2.0 - 0.018, -0.185, z0 + 0.42), (0.050, 0.040, 0.560)),
        _box((top_depth / 2.0 - 0.040, 0.0, z0 + 0.18), (0.080, top_width * 0.72, 0.040)),
        # Side vertical ribs.
        _box((0.06, top_width / 2.0 - 0.014, z0 + 0.42), (0.045, 0.055, 0.570)),
        _box((-0.18, top_width / 2.0 - 0.014, z0 + 0.42), (0.045, 0.055, 0.570)),
        _box((0.06, -top_width / 2.0 + 0.014, z0 + 0.42), (0.045, 0.055, 0.570)),
        _box((-0.18, -top_width / 2.0 + 0.014, z0 + 0.42), (0.045, 0.055, 0.570)),
        # Rear hinge/handle bosses molded into the rear rim.
        _box((-top_depth / 2.0 - 0.025, 0.0, top_z + 0.014), (0.070, 0.080, 0.045)),
        _box((-top_depth / 2.0 - 0.025, 0.215, top_z + 0.014), (0.070, 0.080, 0.045)),
        _box((-top_depth / 2.0 - 0.025, -0.215, top_z + 0.014), (0.070, 0.080, 0.045)),
        _box((-top_depth / 2.0 - 0.025, 0.0, top_z + 0.055), (0.044, 0.060, 0.038)),
        _box((-top_depth / 2.0 - 0.025, 0.215, top_z + 0.055), (0.044, 0.060, 0.038)),
        _box((-top_depth / 2.0 - 0.025, -0.215, top_z + 0.055), (0.044, 0.060, 0.038)),
        _box((-top_depth / 2.0 - 0.045, 0.0, top_z - 0.025), (0.055, top_width + 0.060, 0.085)),
        # A lower molded tray ties the feet and axle saddles back into the bin floor.
        _box((-0.020, 0.0, 0.125), (0.620, 0.480, 0.050)),
        # Front feet keep the bin canted and supported opposite the wheel axle.
        _box((0.245, 0.165, 0.080), (0.105, 0.085, 0.140)),
        _box((0.245, -0.165, 0.080), (0.105, 0.085, 0.140)),
        # Axle saddle blocks tie the rear axle into the molded bin.
        _box((-0.235, 0.245, 0.145), (0.110, 0.110, 0.105)),
        _box((-0.235, -0.245, 0.145), (0.110, 0.110, 0.105)),
    ]

    combined = parts[0]
    for feature in parts[1:]:
        combined = combined.union(feature)
    return combined


def _lid_shape() -> cq.Workplane:
    """Closed lid in a hinge-line local frame; it extends in +X from the rear hinge."""

    rear_clearance = 0.020
    depth = 0.735
    width = 0.655
    front_x = rear_clearance + depth
    slab = _box((rear_clearance + depth / 2.0, 0.0, 0.020), (depth, width, 0.036))
    features = [
        slab,
        # Raised, heavy perimeter frame around the moving panel.
        _box((rear_clearance + depth / 2.0, width / 2.0 - 0.030, 0.046), (depth - 0.020, 0.060, 0.040)),
        _box((rear_clearance + depth / 2.0, -width / 2.0 + 0.030, 0.046), (depth - 0.020, 0.060, 0.040)),
        _box((front_x - 0.038, 0.0, 0.050), (0.075, width, 0.045)),
        _box((rear_clearance + 0.060, 0.0, 0.050), (0.120, width, 0.045)),
        # A shallow center stiffening rib and a front grab lip.
        _box((rear_clearance + depth / 2.0, 0.0, 0.047), (depth - 0.180, 0.050, 0.030)),
        _box((front_x - 0.010, 0.0, 0.014), (0.035, width * 0.78, 0.050)),
    ]
    combined = features[0]
    for feature in features[1:]:
        combined = combined.union(feature)
    return combined


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")
    model.materials.extend([GREEN, DARK_GREEN, BLACK, RIM_GRAY, METAL])

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_wheelie_bin_body_shape(), "body_shell", tolerance=0.0015),
        material=GREEN,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.870),
        origin=Origin(xyz=(-0.235, 0.0, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=METAL,
        name="rear_axle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid_panel", tolerance=0.0012),
        material=DARK_GREEN,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.024, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=DARK_GREEN,
        name="hinge_barrel",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.125,
            0.078,
            inner_radius=0.076,
            tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.008, radius=0.003),
        ),
        "wheel_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.084,
            0.080,
            rim=WheelRim(inner_radius=0.055, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.034,
                width=0.052,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.045, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.038),
        ),
        "wheel_rim",
    )

    wheel_origins = {"wheel_0": (0.0, 0.370, 0.0), "wheel_1": (0.0, -0.370, 0.0)}
    for wheel_name in wheel_origins:
        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=BLACK,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=RIM_GRAY,
            name="rim",
        )

    hinge_x = -0.360
    hinge_z = 1.005
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.75),
    )

    axle_x = -0.235
    axle_z = 0.125
    for wheel_name, (_, y, _) in wheel_origins.items():
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_name,
            origin=Origin(xyz=(axle_x, y, axle_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="body_shell",
        elem_b="hinge_barrel",
        reason="The molded rear hinge saddles intentionally capture the lid hinge barrel at the pivot.",
    )
    for wheel_name in ("wheel_0", "wheel_1"):
        ctx.allow_overlap(
            body,
            wheel_name,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The fixed rear axle intentionally passes through the simplified wheel hub bore.",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.040,
            max_penetration=0.025,
            positive_elem="lid_panel",
            negative_elem="body_shell",
            name="closed lid seats on the rear hinge and top rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.50,
            elem_a="lid_panel",
            elem_b="body_shell",
            name="closed lid covers the bin opening full width",
        )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            positive_elem="lid_panel",
            negative_elem="body_shell",
            name="raised lid clears the bin body",
        )
    ctx.check(
        "positive lid angle lifts the lid panel",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.20,
        details=f"rest_aabb={rest_lid_aabb}, open_aabb={open_lid_aabb}",
    )

    ctx.expect_origin_gap(wheel_0, wheel_1, axis="y", min_gap=0.70, name="wheels sit on opposite axle ends")
    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.expect_overlap(
            wheel,
            body,
            axes="xz",
            min_overlap=0.02,
            elem_a="rim",
            elem_b="rear_axle",
            name=f"{wheel_name} hub is centered on the rear axle",
        )

    return ctx.report()


object_model = build_object_model()
