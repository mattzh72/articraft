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
    TestContext,
    TestReport,
    TireGeometry,
    TireTread,
    TireSidewall,
    TireShoulder,
    WheelGeometry,
    WheelRim,
    WheelHub,
    WheelFace,
    WheelSpokes,
    WheelBore,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tapered_bin_body() -> cq.Workplane:
    """One-piece hollow tapered wheelie-bin body with an open top and rim."""

    bottom_z = 0.12
    height = 0.83
    floor = 0.045
    wall = 0.035
    bottom_depth = 0.47
    bottom_width = 0.42
    top_depth = 0.62
    top_width = 0.55

    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .rect(bottom_depth, bottom_width)
        .workplane(offset=height)
        .rect(top_depth, top_width)
        .loft(combine=True)
    )
    inner_cutter = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z + floor)
        .rect(bottom_depth - 2.0 * wall, bottom_width - 2.0 * wall)
        .workplane(offset=height + 0.12)
        .rect(top_depth - 2.0 * wall, top_width - 2.0 * wall)
        .loft(combine=True)
    )
    shell = outer.cut(inner_cutter)

    rim_center_z = bottom_z + height + 0.006
    rim_outer = cq.Workplane("XY").box(top_depth + 0.075, top_width + 0.075, 0.055).translate(
        (0.0, 0.0, rim_center_z)
    )
    rim_inner = cq.Workplane("XY").box(top_depth - 0.030, top_width - 0.030, 0.080).translate(
        (0.0, 0.0, rim_center_z + 0.004)
    )
    rim = rim_outer.cut(rim_inner)
    body = shell.union(rim)

    # Softened outside corners make the large uninterrupted plastic panels read
    # as molded rather than a sharp metal box.  Keep the fallback because fillet
    # robustness can depend on tessellation/boolean details.
    try:
        body = body.edges("|Z").fillet(0.018)
    except Exception:
        pass

    return body


def _lid_panel() -> cq.Workplane:
    """Full-width single-piece lid panel; separate lip visual mounts to it."""

    depth = 0.705
    width = 0.635
    thickness = 0.040

    main = (
        cq.Workplane("XY")
        .box(depth, width, thickness)
        .edges("|Z")
        .fillet(0.020)
        .translate((depth / 2.0 + 0.035, 0.0, -0.004))
    )
    return main


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    bin_green = model.material("molded_green_plastic", rgba=(0.035, 0.26, 0.12, 1.0))
    dark_green = model.material("dark_green_shadow", rgba=(0.018, 0.13, 0.060, 1.0))
    axle_metal = model.material("blackened_steel", rgba=(0.015, 0.015, 0.014, 1.0))
    tire_black = model.material("matte_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    hub_gray = model.material("charcoal_plastic", rgba=(0.075, 0.080, 0.075, 1.0))

    hinge_x = -0.347
    hinge_z = 1.007
    axle_x = -0.310
    axle_z = 0.160
    wheel_y = 0.370

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tapered_bin_body(), "body_shell", tolerance=0.0015),
        material=bin_green,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.840),
        origin=Origin(xyz=(axle_x, 0.0, axle_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="rear_axle",
    )
    for i, y in enumerate((-0.225, 0.225)):
        body.visual(
            Box((0.115, 0.045, 0.215)),
            origin=Origin(xyz=(axle_x + 0.015, y, axle_z + 0.075)),
            material=bin_green,
            name=f"axle_bracket_{i}",
        )
    for i, y in enumerate((-0.238, 0.238)):
        body.visual(
            Cylinder(radius=0.025, length=0.118),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"body_hinge_{i}",
        )
    body.visual(
        Box((0.060, 0.575, 0.030)),
        origin=Origin(xyz=(hinge_x + 0.020, 0.0, hinge_z - 0.050)),
        material=dark_green,
        name="rear_hinge_land",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel(), "lid_panel", tolerance=0.0015),
        material=bin_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.040, 0.610, 0.078)),
        origin=Origin(xyz=(0.758, 0.0, -0.043)),
        material=dark_green,
        name="front_lip",
    )
    for i, y in enumerate((-0.075, 0.075)):
        lid.visual(
            Box((0.070, 0.105, 0.030)),
            origin=Origin(xyz=(0.020, y, -0.005)),
            material=dark_green,
            name=f"lid_hinge_tongue_{i}",
        )
        lid.visual(
            Cylinder(radius=0.023, length=0.110),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"lid_hinge_{i}",
        )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.130,
            0.075,
            inner_radius=0.090,
            tread=TireTread(style="block", depth=0.007, count=20, land_ratio=0.56),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "utility_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.091,
            0.066,
            rim=WheelRim(inner_radius=0.058, flange_height=0.007, flange_thickness=0.004),
            hub=WheelHub(radius=0.034, width=0.070, cap_style="domed"),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.043),
        ),
        "wheel_disc",
    )

    for i, y in enumerate((-wheel_y, wheel_y)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=tire_black,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=hub_gray,
            name="hub",
        )
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(axle_x, y, axle_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=15.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")

    for wheel_name in ("wheel_0", "wheel_1"):
        ctx.allow_overlap(
            body,
            wheel_name,
            elem_a="rear_axle",
            elem_b="hub",
            reason="The steel axle is intentionally captured through the wheel hub bore with a tiny bearing-seat interference.",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.50,
        name="closed full-width lid covers the open bin body",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        max_penetration=0.001,
        max_gap=0.020,
        name="closed lid sits just above the top rim",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens upward from the rear edge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for i, joint in enumerate((wheel_joint_0, wheel_joint_1)):
        ctx.check(
            f"wheel_{i} is continuous on the rear axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(joint.axis[1] - 1.0) < 1e-6,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
    ctx.expect_origin_gap(
        wheel_1,
        wheel_0,
        axis="y",
        min_gap=0.70,
        max_gap=0.76,
        name="two wheels sit at opposite ends of the rear axle",
    )
    ctx.expect_overlap(
        wheel_0,
        body,
        axes="xz",
        elem_a="hub",
        elem_b="rear_axle",
        min_overlap=0.03,
        name="wheel_0 hub is centered on the axle",
    )
    ctx.expect_overlap(
        wheel_1,
        body,
        axes="xz",
        elem_a="hub",
        elem_b="rear_axle",
        min_overlap=0.03,
        name="wheel_1 hub is centered on the axle",
    )

    return ctx.report()


object_model = build_object_model()
