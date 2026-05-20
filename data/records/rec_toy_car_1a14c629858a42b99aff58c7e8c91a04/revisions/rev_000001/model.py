from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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


def _cabin_mesh():
    """A small one-piece sloped cabin for the toy car shell."""
    cabin = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.095, 0.000),
                (0.055, 0.000),
                (0.044, 0.034),
                (0.006, 0.062),
                (-0.064, 0.056),
                (-0.092, 0.026),
            ]
        )
        .close()
        .extrude(0.104, both=True)
    )
    return mesh_from_cadquery(cabin, "cabin_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_car_hood")

    red = model.material("molded_red_plastic", rgba=(0.86, 0.05, 0.03, 1.0))
    black = model.material("soft_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    dark = model.material("matte_black_chassis", rgba=(0.02, 0.022, 0.025, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    yellow = model.material("warm_yellow_hubcap", rgba=(1.0, 0.74, 0.08, 1.0))
    glass = model.material("smoky_blue_window", rgba=(0.10, 0.22, 0.34, 0.72))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.036,
            0.024,
            inner_radius=0.026,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0025, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0012),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.0035, radius=0.002),
        ),
        "toy_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.026,
            0.020,
            rim=WheelRim(inner_radius=0.017, flange_height=0.0025, flange_thickness=0.0015),
            hub=WheelHub(
                radius=0.010,
                width=0.014,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.014, hole_diameter=0.002),
            ),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "toy_wheel_rim",
    )
    cabin_mesh = _cabin_mesh()

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.360, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark,
        name="deck_plate",
    )
    chassis.visual(
        Box((0.300, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="center_spine",
    )
    chassis.visual(
        Box((0.030, 0.118, 0.018)),
        origin=Origin(xyz=(0.178, 0.0, 0.045)),
        material=dark,
        name="front_bumper_pad",
    )
    chassis.visual(
        Box((0.030, 0.118, 0.018)),
        origin=Origin(xyz=(-0.178, 0.0, 0.045)),
        material=dark,
        name="rear_bumper_pad",
    )

    body = model.part("body_shell")
    body.visual(
        Box((0.340, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.064, 0.079)),
        material=red,
        name="side_rail_left",
    )
    body.visual(
        Box((0.340, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.064, 0.079)),
        material=red,
        name="side_rail_right",
    )
    body.visual(
        Box((0.028, 0.132, 0.042)),
        origin=Origin(xyz=(0.170, 0.0, 0.075)),
        material=red,
        name="front_nose",
    )
    body.visual(
        Box((0.110, 0.132, 0.060)),
        origin=Origin(xyz=(-0.125, 0.0, 0.084)),
        material=red,
        name="rear_deck",
    )
    body.visual(
        Box((0.018, 0.132, 0.016)),
        origin=Origin(xyz=(0.019, 0.0, 0.099)),
        material=red,
        name="cowl_bar",
    )
    body.visual(
        cabin_mesh,
        origin=Origin(xyz=(-0.045, 0.0, 0.096)),
        material=red,
        name="cabin_shell",
    )
    for x in (0.115, -0.115):
        for side, suffix in ((1.0, "left"), (-1.0, "right")):
            body.visual(
                Box((0.078, 0.022, 0.012)),
                origin=Origin(xyz=(x, side * 0.068, 0.083)),
                material=red,
                name=f"{'front' if x > 0 else 'rear'}_{suffix}_fender",
            )
    body.visual(
        Box((0.006, 0.086, 0.034)),
        origin=Origin(xyz=(0.012, 0.0, 0.122)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.064, 0.004, 0.028)),
        origin=Origin(xyz=(-0.057, 0.054, 0.128), rpy=(0.0, 0.0, 0.0)),
        material=glass,
        name="side_window_left",
    )
    body.visual(
        Box((0.064, 0.004, 0.028)),
        origin=Origin(xyz=(-0.057, -0.054, 0.128), rpy=(0.0, 0.0, 0.0)),
        material=glass,
        name="side_window_right",
    )
    for y, suffix in ((0.040, "left"), (-0.040, "right")):
        body.visual(
            Box((0.014, 0.020, 0.014)),
            origin=Origin(xyz=(0.028, y, 0.111)),
            material=metal,
            name=f"hinge_block_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(xyz=(0.028, y, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_knuckle_{suffix}",
        )

    model.articulation(
        "chassis_to_body_shell",
        ArticulationType.FIXED,
        parent=chassis,
        child=body,
        origin=Origin(),
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.114, 0.098, 0.008)),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=red,
        name="hood_panel",
    )
    hood.visual(
        Box((0.082, 0.022, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, 0.006)),
        material=red,
        name="raised_hood_rib",
    )
    hood.visual(
        Cylinder(radius=0.003, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    hood.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="hinge_sleeve",
    )
    hood.visual(
        Box((0.014, 0.020, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=red,
        name="hinge_leaf",
    )
    model.articulation(
        "body_shell_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.028, 0.0, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    def add_mount_and_wheel(x: float, side: float, axle_name: str, wheel_name: str) -> None:
        y = side * 0.082
        mount = model.part(axle_name)
        mount.visual(
            Box((0.050, 0.006, 0.026)),
            origin=Origin(xyz=(0.0, -side * 0.024, 0.006)),
            material=metal,
            name="mount_plate",
        )
        mount.visual(
            Cylinder(radius=0.005, length=0.015),
            origin=Origin(
                xyz=(0.0, -side * 0.0195, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name="axle_stub",
        )
        model.articulation(
            f"chassis_to_{axle_name}",
            ArticulationType.FIXED,
            parent=chassis,
            child=mount,
            origin=Origin(xyz=(x, y, 0.040)),
        )

        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=metal,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, side * 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=yellow,
            name="outer_hubcap",
        )
        model.articulation(
            f"{axle_name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=mount,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=25.0),
        )

    add_mount_and_wheel(0.115, 1.0, "front_left_mount", "front_left_wheel")
    add_mount_and_wheel(0.115, -1.0, "front_right_mount", "front_right_wheel")
    add_mount_and_wheel(-0.115, 1.0, "rear_left_mount", "rear_left_wheel")
    add_mount_and_wheel(-0.115, -1.0, "rear_right_mount", "rear_right_wheel")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    body = object_model.get_part("body_shell")
    hood = object_model.get_part("hood")
    hood_hinge = object_model.get_articulation("body_shell_to_hood")

    for suffix in ("left", "right"):
        ctx.allow_overlap(
            body,
            hood,
            elem_a=f"hinge_knuckle_{suffix}",
            elem_b="hinge_pin",
            reason="The hood hinge pin is intentionally captured inside the molded hinge knuckle proxy.",
        )
        ctx.expect_within(
            hood,
            body,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=f"hinge_knuckle_{suffix}",
            margin=0.001,
            name=f"hinge pin centered in {suffix} knuckle",
        )
        ctx.expect_overlap(
            hood,
            body,
            axes="y",
            elem_a="hinge_pin",
            elem_b=f"hinge_knuckle_{suffix}",
            min_overlap=0.012,
            name=f"hinge pin passes through {suffix} knuckle",
        )

    ctx.expect_contact(
        body,
        chassis,
        elem_a="rear_deck",
        elem_b="deck_plate",
        name="body shell seats on chassis plate",
    )
    ctx.expect_gap(
        hood,
        body,
        axis="z",
        positive_elem="hood_panel",
        negative_elem="cowl_bar",
        min_gap=0.006,
        name="closed hood clears cowl bar except hinge hardware",
    )

    wheel_specs = (
        ("front_left_mount", "front_left_wheel", "y", "wheel_positive"),
        ("rear_left_mount", "rear_left_wheel", "y", "wheel_positive"),
        ("front_right_mount", "front_right_wheel", "y", "mount_positive"),
        ("rear_right_mount", "rear_right_wheel", "y", "mount_positive"),
    )
    for mount_name, wheel_name, axis, side_rule in wheel_specs:
        mount = object_model.get_part(mount_name)
        wheel = object_model.get_part(wheel_name)
        ctx.expect_contact(
            mount,
            chassis,
            elem_a="mount_plate",
            elem_b="deck_plate",
            name=f"{mount_name} plate touches chassis side",
        )
        if side_rule == "wheel_positive":
            ctx.expect_gap(
                wheel,
                mount,
                axis=axis,
                positive_elem="tire",
                negative_elem="axle_stub",
                max_gap=0.003,
                max_penetration=0.0,
                name=f"{wheel_name} seated against mount stub",
            )
        else:
            ctx.expect_gap(
                mount,
                wheel,
                axis=axis,
                positive_elem="axle_stub",
                negative_elem="tire",
                max_gap=0.003,
                max_penetration=0.0,
                name=f"{wheel_name} seated against mount stub",
            )

    closed_aabb = ctx.part_world_aabb(hood)
    with ctx.pose({hood_hinge: 0.90}):
        raised_aabb = ctx.part_world_aabb(hood)
    ctx.check(
        "front hood opens upward",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, open={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
