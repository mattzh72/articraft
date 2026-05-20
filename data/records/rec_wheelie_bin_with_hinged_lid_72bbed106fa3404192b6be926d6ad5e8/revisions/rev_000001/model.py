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


BIN_BOTTOM_Z = 0.16
BIN_HEIGHT = 0.86
BIN_TOP_Z = BIN_BOTTOM_Z + BIN_HEIGHT
BIN_TOP_WIDTH = 0.62
BIN_TOP_DEPTH = 0.68
BIN_BOTTOM_WIDTH = 0.48
BIN_BOTTOM_DEPTH = 0.45
WHEEL_RADIUS = 0.145
WHEEL_WIDTH = 0.076
WHEEL_X = 0.385
AXLE_Y = 0.405
AXLE_Z = 0.160
HINGE_Y = 0.370
HINGE_Z = 1.085


def _bin_shell_shape() -> cq.Workplane:
    """Tapered open-top HDPE bin shell, authored in the root/bin frame."""
    shell = (
        cq.Workplane("XY")
        .workplane(offset=BIN_BOTTOM_Z)
        .rect(BIN_BOTTOM_WIDTH, BIN_BOTTOM_DEPTH)
        .workplane(offset=BIN_HEIGHT)
        .rect(BIN_TOP_WIDTH, BIN_TOP_DEPTH)
        .loft(combine=True)
        .faces(">Z")
        .shell(-0.024)
    )
    return shell


def _lid_panel_shape() -> cq.Workplane:
    """One-piece rounded full-width lid with its local origin on the hinge axis."""
    lid_width = 0.700
    lid_depth = 0.735
    lid_thickness = 0.040
    # The rear edge sits just ahead of the hinge barrel; the panel extends toward
    # local -Y so the revolute joint frame can be exactly the hinge line.
    return (
        cq.Workplane("XY")
        .box(lid_width, lid_depth, lid_thickness)
        .edges("|Z")
        .fillet(0.045)
        .translate((0.0, -lid_depth / 2.0 + 0.020, 0.028))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wheelie_bin")

    green = model.material("weathered_green_hdpe", rgba=(0.05, 0.25, 0.13, 1.0))
    dark_green = model.material("dark_reinforced_hdpe", rgba=(0.03, 0.16, 0.09, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.55, 1.0))
    bolt = model.material("dull_bolt_heads", rgba=(0.30, 0.31, 0.29, 1.0))

    body = model.part("bin")
    body.visual(
        mesh_from_cadquery(_bin_shell_shape(), "tapered_hollow_bin_shell", tolerance=0.002),
        material=green,
        name="bin_shell",
    )

    # Heavy top rim and old-school raised ribs keep the shell visibly connected
    # and make the thin hollow bin read as a reinforced manufactured part.
    body.visual(
        Box((0.710, 0.060, 0.045)),
        origin=Origin(xyz=(0.0, -0.355, BIN_TOP_Z + 0.018)),
        material=dark_green,
        name="top_rim_front",
    )
    body.visual(
        Box((0.710, 0.060, 0.045)),
        origin=Origin(xyz=(0.0, 0.355, BIN_TOP_Z + 0.018)),
        material=dark_green,
        name="top_rim_rear",
    )
    for x, name in ((-0.340, "top_rim_side_0"), (0.340, "top_rim_side_1")):
        body.visual(
            Box((0.055, 0.710, 0.045)),
            origin=Origin(xyz=(x, 0.0, BIN_TOP_Z + 0.018)),
            material=dark_green,
            name=name,
        )

    for x, name in ((-0.205, "front_rib_0"), (0.0, "front_rib_1"), (0.205, "front_rib_2")):
        body.visual(
            Box((0.042, 0.080, 0.650)),
            origin=Origin(xyz=(x, -0.286, 0.575)),
            material=dark_green,
            name=name,
        )
    for x, name in ((-0.322, "side_rib_0"), (0.322, "side_rib_1")):
        body.visual(
            Box((0.070, 0.042, 0.600)),
            origin=Origin(xyz=(x, -0.030, 0.560)),
            material=dark_green,
            name=name,
        )

    # Retrofit service hatch and bolted adapter plates are static hardware fixed
    # to the shell, not separate floating doors.
    body.visual(
        Box((0.315, 0.052, 0.225)),
        origin=Origin(xyz=(0.0, -0.305, 0.610)),
        material=dark_green,
        name="service_hatch",
    )
    body.visual(
        Box((0.350, 0.036, 0.265)),
        origin=Origin(xyz=(0.0, -0.344, 0.610)),
        material=steel,
        name="hatch_adapter",
    )
    for i, (x, z) in enumerate(
        ((-0.145, 0.500), (0.145, 0.500), (-0.145, 0.720), (0.145, 0.720))
    ):
        body.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(x, -0.363, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"hatch_bolt_{i}",
        )

    # Rear grab rail and hinge fixtures deliberately look like legacy hardware
    # retained by modern reinforced plastic pads.
    body.visual(
        Cylinder(radius=0.018, length=0.680),
        origin=Origin(xyz=(0.0, 0.392, 0.965), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_green,
        name="rear_handle",
    )
    for x, name in ((-0.250, "handle_post_0"), (0.250, "handle_post_1")):
        body.visual(
            Box((0.045, 0.060, 0.170)),
            origin=Origin(xyz=(x, 0.360, 0.895)),
            material=dark_green,
            name=name,
        )

    body.visual(
        Cylinder(radius=0.007, length=0.720),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    for x, name in ((-0.265, "hinge_saddle_0"), (0.0, "hinge_saddle_1"), (0.265, "hinge_saddle_2")):
        body.visual(
            Box((0.060, 0.058, 0.086)),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z - 0.068)),
            material=dark_green,
            name=name,
        )
    for x, name in ((-0.345, "pin_end_block_0"), (0.345, "pin_end_block_1")):
        body.visual(
            Box((0.035, 0.060, 0.080)),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z - 0.047)),
            material=dark_green,
            name=name,
        )

    # Rear axle, bearing blocks, and bolted side adapters support both rolling
    # wheels. The axle is intentionally visible, like a retrofit service part.
    body.visual(
        Cylinder(radius=0.016, length=0.840),
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    body.visual(
        Box((0.660, 0.085, 0.075)),
        origin=Origin(xyz=(0.0, 0.245, 0.225)),
        material=dark_green,
        name="axle_cradle",
    )
    for x, name in ((-0.292, "axle_bearing_0"), (0.292, "axle_bearing_1")):
        body.visual(
            Box((0.060, 0.210, 0.125)),
            origin=Origin(xyz=(x, 0.330, 0.220)),
            material=dark_green,
            name=name,
        )
    for side, x in enumerate((-0.285, 0.285)):
        outer_x = x - 0.049 if x < 0.0 else x + 0.049
        body.visual(
            Box((0.090, 0.160, 0.140)),
            origin=Origin(xyz=(x, 0.355, 0.270)),
            material=steel,
            name=f"axle_adapter_{side}",
        )
        for j, (y, z) in enumerate(((0.305, 0.225), (0.405, 0.225), (0.305, 0.315), (0.405, 0.315))):
            body.visual(
                Cylinder(radius=0.009, length=0.010),
                origin=Origin(xyz=(outer_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt,
                name=f"axle_bolt_{side}_{j}",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel_shape(), "rounded_full_width_lid", tolerance=0.002),
        material=green,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.640),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_green,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.630, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, -0.685, 0.061)),
        material=dark_green,
        name="front_lid_rib",
    )
    for x, name in ((-0.185, "lid_rib_0"), (0.185, "lid_rib_1")):
        lid.visual(
            Box((0.055, 0.510, 0.026)),
            origin=Origin(xyz=(x, -0.355, 0.061)),
            material=dark_green,
            name=name,
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    wheel_rim = WheelGeometry(
        0.112,
        WHEEL_WIDTH * 0.82,
        rim=WheelRim(inner_radius=0.076, flange_height=0.010, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.038,
            width=0.050,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.052, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.007, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.031),
    )
    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.104,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=0.007, count=22, land_ratio=0.55),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.008, radius=0.004),
    )
    for side, x in enumerate((-WHEEL_X, WHEEL_X)):
        wheel = model.part(f"wheel_{side}")
        wheel.visual(mesh_from_geometry(wheel_rim, f"wheel_{side}_rim"), material=steel, name="wheel_rim")
        wheel.visual(mesh_from_geometry(tire, f"wheel_{side}_tire"), material=black, name="tire")
        model.articulation(
            f"wheel_{side}_roll",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, AXLE_Y, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_0_roll = object_model.get_articulation("wheel_0_roll")
    wheel_1_roll = object_model.get_articulation("wheel_1_roll")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The full-width lid barrel is intentionally captured around the fixed hinge pin.",
    )
    for wheel, rim_name in ((wheel_0, "wheel_0"), (wheel_1, "wheel_1")):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="axle_shaft",
            elem_b="wheel_rim",
            reason=f"The rear axle intentionally runs through the supported bore of {rim_name}.",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="bin_shell",
        min_overlap=0.55,
        name="lid covers the open bin shell",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_rim_front",
        min_gap=0.000,
        max_gap=0.060,
        max_penetration=0.0,
        name="closed lid sits above reinforced front rim",
    )
    ctx.expect_within(
        body,
        lid,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin is centered in lid barrel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.60,
        name="hinge support spans almost full lid width",
    )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid swings upward from rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.25,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    for wheel, joint, name in ((wheel_0, wheel_0_roll, "wheel_0"), (wheel_1, wheel_1_roll, "wheel_1")):
        ctx.expect_within(
            body,
            wheel,
            axes="yz",
            inner_elem="axle_shaft",
            outer_elem="wheel_rim",
            margin=0.003,
            name=f"{name} rim surrounds rear axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="x",
            elem_a="axle_shaft",
            elem_b="wheel_rim",
            min_overlap=0.045,
            name=f"{name} stays retained on axle width",
        )
        rest_pos = ctx.part_world_position(wheel)
        with ctx.pose({joint: math.pi / 2.0}):
            rolled_pos = ctx.part_world_position(wheel)
        ctx.check(
            f"{name} rolls about fixed axle center",
            rest_pos is not None
            and rolled_pos is not None
            and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, rolled_pos)),
            details=f"rest={rest_pos}, rolled={rolled_pos}",
        )

    return ctx.report()


object_model = build_object_model()
