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
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges().fillet(radius)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_rolling_toolbox")

    charcoal = model.material("charcoal_molded_plastic", rgba=(0.045, 0.048, 0.052, 1.0))
    black = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    red = model.material("red_lid_plastic", rgba=(0.72, 0.055, 0.035, 1.0))
    dark_gray = model.material("dark_gray_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    clear = model.material("smoky_clear_polycarbonate", rgba=(0.62, 0.86, 1.0, 0.36))
    metal = model.material("brushed_steel", rgba=(0.72, 0.72, 0.70, 1.0))

    body = model.part("body")

    lower_shell = (
        _rounded_box((0.80, 0.44, 0.09), 0.016).translate((0.0, 0.0, 0.075))
        .union(_rounded_box((0.72, 0.38, 0.30), 0.024).translate((0.0, 0.0, 0.215)))
    )
    lid_cap = _rounded_box((0.74, 0.40, 0.075), 0.018).translate((0.0, 0.0, 0.4025))

    body.visual(
        mesh_from_cadquery(lower_shell, "lower_shell", tolerance=0.0015),
        material=charcoal,
        name="lower_shell",
    )
    body.visual(
        mesh_from_cadquery(lid_cap, "lid_cap", tolerance=0.0015),
        material=red,
        name="lid_cap",
    )

    # Molded front latch, side pads, and rear telescoping-handle channel details.
    body.visual(Box((0.035, 0.13, 0.050)), origin=Origin(xyz=(0.386, 0.0, 0.355)), material=dark_gray, name="front_latch")
    body.visual(Box((0.24, 0.012, 0.075)), origin=Origin(xyz=(0.05, 0.194, 0.235)), material=dark_gray, name="side_grip_0")
    body.visual(Box((0.24, 0.012, 0.075)), origin=Origin(xyz=(0.05, -0.194, 0.235)), material=dark_gray, name="side_grip_1")
    body.visual(Box((0.012, 0.040, 0.365)), origin=Origin(xyz=(-0.404, 0.115, 0.245)), material=black, name="channel_0")
    body.visual(Box((0.012, 0.040, 0.365)), origin=Origin(xyz=(-0.404, -0.115, 0.245)), material=black, name="channel_1")
    body.visual(Box((0.035, 0.030, 0.340)), origin=Origin(xyz=(-0.397, 0.115, 0.240)), material=charcoal, name="channel_lip_0")
    body.visual(Box((0.035, 0.030, 0.340)), origin=Origin(xyz=(-0.397, -0.115, 0.240)), material=charcoal, name="channel_lip_1")
    body.visual(Box((0.048, 0.310, 0.016)), origin=Origin(xyz=(-0.387, 0.0, 0.444)), material=black, name="handle_recess")

    # Axle bosses stop just short of the freely rotating wheel hubs.
    body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(-0.280, 0.220, 0.087), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="axle_boss_0",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(-0.280, -0.220, 0.087), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="axle_boss_1",
    )

    # Twin organizer trays under the transparent covers.
    organizer_ys = (0.095, -0.095)
    for idx, y in enumerate(organizer_ys):
        body.visual(Box((0.285, 0.165, 0.008)), origin=Origin(xyz=(-0.012, y, 0.444)), material=dark_gray, name=f"tray_{idx}")
        body.visual(Box((0.012, 0.150, 0.012)), origin=Origin(xyz=(-0.060, y, 0.453)), material=charcoal, name=f"divider_{idx}_0")
        body.visual(Box((0.012, 0.150, 0.012)), origin=Origin(xyz=(0.035, y, 0.453)), material=charcoal, name=f"divider_{idx}_1")
        body.visual(Box((0.014, 0.060, 0.014)), origin=Origin(xyz=(0.127, y, 0.455)), material=black, name=f"cover_latch_{idx}")
        body.visual(Box((0.020, 0.160, 0.020)), origin=Origin(xyz=(-0.155, y, 0.450)), material=dark_gray, name=f"hinge_sill_{idx}")

    # Sliding U-handle: two parallel metal tubes joined by a rubberized grip.
    handle = model.part("handle")
    for idx, y in enumerate((0.115, -0.115)):
        handle.visual(
            Cylinder(radius=0.009, length=0.440),
            origin=Origin(xyz=(0.0, y, -0.040)),
            material=metal,
            name=f"rod_{idx}",
        )
    handle.visual(
        Cylinder(radius=0.014, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.190), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=0.023, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.190), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_sleeve",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, -0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lower_tie",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.4235, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.28, lower=0.0, upper=0.36),
    )

    # Two separate clear organizer lids, each hinged from its rear edge.
    cover_length = 0.280
    cover_width = 0.155
    cover_thickness = 0.012
    for idx, y in enumerate(organizer_ys):
        cover = model.part(f"cover_{idx}")
        cover.visual(
            Box((cover_length, cover_width, cover_thickness)),
            origin=Origin(xyz=(cover_length / 2.0, 0.0, 0.012)),
            material=clear,
            name="clear_panel",
        )
        cover.visual(Box((0.014, cover_width + 0.010, 0.014)), origin=Origin(xyz=(0.004, 0.0, 0.011)), material=dark_gray, name="rear_lip")
        cover.visual(Box((0.014, cover_width + 0.010, 0.014)), origin=Origin(xyz=(cover_length - 0.004, 0.0, 0.011)), material=dark_gray, name="front_lip")
        cover.visual(Box((cover_length, 0.010, 0.014)), origin=Origin(xyz=(cover_length / 2.0, cover_width / 2.0, 0.011)), material=dark_gray, name="side_lip_0")
        cover.visual(Box((cover_length, 0.010, 0.014)), origin=Origin(xyz=(cover_length / 2.0, -cover_width / 2.0, 0.011)), material=dark_gray, name="side_lip_1")
        cover.visual(
            Cylinder(radius=0.008, length=cover_width + 0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="hinge_barrel",
        )
        model.articulation(
            f"body_to_cover_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=cover,
            origin=Origin(xyz=(-0.155, y, 0.462)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.22),
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.060,
            0.046,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.0025, bead_seat_depth=0.002),
            hub=WheelHub(radius=0.022, width=0.038, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.001),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.006),
        ),
        "wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.085,
            0.055,
            inner_radius=0.059,
            tread=TireTread(style="block", depth=0.005, count=20, land_ratio=0.56),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.005, radius=0.002),
        ),
        "wheel_tire",
    )

    for idx, y in enumerate((0.251, -0.251)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=black, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=dark_gray, name="rim")
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.280, y, 0.087)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_overlap(handle, body, axes="z", elem_a="rod_0", elem_b="channel_0", min_overlap=0.30, name="retracted handle remains in channel")
        ctx.expect_within(handle, body, axes="y", inner_elem="rod_0", outer_elem="channel_0", margin=0.004, name="rod 0 centered in channel")
        rest_handle = ctx.part_world_position(handle)

    with ctx.pose({handle_joint: 0.36}):
        ctx.expect_overlap(handle, body, axes="z", elem_a="rod_0", elem_b="channel_0", min_overlap=0.020, name="extended handle retains insertion")
        extended_handle = ctx.part_world_position(handle)

    ctx.check(
        "handle slides upward",
        rest_handle is not None and extended_handle is not None and extended_handle[2] > rest_handle[2] + 0.30,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    for idx in (0, 1):
        cover = object_model.get_part(f"cover_{idx}")
        hinge = object_model.get_articulation(f"body_to_cover_{idx}")
        with ctx.pose({hinge: 0.0}):
            ctx.expect_overlap(cover, body, axes="xy", elem_a="clear_panel", elem_b=f"tray_{idx}", min_overlap=0.13, name=f"cover {idx} sits over tray")
            ctx.expect_gap(cover, body, axis="z", min_gap=0.002, max_gap=0.030, positive_elem="clear_panel", negative_elem=f"tray_{idx}", name=f"cover {idx} seated above organizer")
            closed_aabb = ctx.part_world_aabb(cover)
        with ctx.pose({hinge: 1.05}):
            open_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            f"cover {idx} opens upward",
            closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")
    for idx, wheel in ((0, wheel_0), (1, wheel_1)):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=f"axle_boss_{idx}",
            elem_b="rim",
            reason="The molded axle boss is intentionally represented as captured inside the rotating wheel hub.",
        )
        ctx.expect_within(
            body,
            wheel,
            axes="xz",
            inner_elem=f"axle_boss_{idx}",
            outer_elem="rim",
            margin=0.003,
            name=f"wheel {idx} is coaxial with axle boss",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=f"axle_boss_{idx}",
            elem_b="rim",
            min_overlap=0.0005,
            name=f"wheel {idx} hub remains captured on axle",
        )
    wheel_pos = ctx.part_world_position(wheel_0)
    with ctx.pose({wheel_joint_0: math.pi, wheel_joint_1: math.pi}):
        spun_pos = ctx.part_world_position(wheel_0)
    ctx.check("wheel rotation keeps axle center fixed", wheel_pos == spun_pos, details=f"rest={wheel_pos}, spun={spun_pos}")

    return ctx.report()


object_model = build_object_model()
