from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _lofted_open_bin_body() -> cq.Workplane:
    """A tapered hollow wheelie-bin tub with a closed floor and open top."""

    bottom_w, bottom_d = 0.42, 0.38
    top_w, top_d = 0.60, 0.62
    z0, z1 = 0.22, 0.92
    wall = 0.026
    floor = 0.055

    outer = (
        cq.Workplane("XY")
        .rect(bottom_w, bottom_d)
        .workplane(offset=z1 - z0)
        .rect(top_w, top_d)
        .loft(combine=True)
        .translate((0.0, 0.0, z0))
    )
    inner = (
        cq.Workplane("XY")
        .rect(bottom_w - 2.0 * wall, bottom_d - 2.0 * wall)
        .workplane(offset=z1 - z0 + 0.040)
        .rect(top_w - 2.0 * wall, top_d - 2.0 * wall)
        .loft(combine=True)
        .translate((0.0, 0.0, z0 + floor))
    )
    body = outer.cut(inner)

    # A thick molded top rim reinforces the open tub and gives the lid a
    # believable seat while keeping the cavity visible if the lid is raised.
    rim_z = z1 + 0.006
    rim_h = 0.028
    rim_t = 0.040
    rim_front = cq.Workplane("XY").box(top_w + 0.050, rim_t, rim_h).translate((0.0, top_d / 2.0, rim_z))
    rim_back = cq.Workplane("XY").box(top_w + 0.050, rim_t, rim_h).translate((0.0, -top_d / 2.0, rim_z))
    rim_0 = cq.Workplane("XY").box(rim_t, top_d + 0.040, rim_h).translate((top_w / 2.0, 0.0, rim_z))
    rim_1 = cq.Workplane("XY").box(rim_t, top_d + 0.040, rim_h).translate((-top_w / 2.0, 0.0, rim_z))

    # Subtle side ribs and a rear handle boss are molded into the same shell.
    rib_h = 0.46
    rib_z = 0.54
    rib_y = -0.265
    rear_rib_0 = cq.Workplane("XY").box(0.035, 0.030, rib_h).translate((-0.205, rib_y, rib_z))
    rear_rib_1 = cq.Workplane("XY").box(0.035, 0.030, rib_h).translate((0.205, rib_y, rib_z))
    rear_cross = cq.Workplane("XY").box(0.46, 0.030, 0.052).translate((0.0, rib_y, 0.78))

    return body.union(rim_front).union(rim_back).union(rim_0).union(rim_1).union(rear_rib_0).union(rear_rib_1).union(rear_cross)


def _lid_panel() -> cq.Workplane:
    """Full-width molded plastic lid in a hinge-line local frame."""

    width = 0.65
    depth = 0.655
    panel_t = 0.032

    panel = cq.Workplane("XY").box(width, depth, panel_t).translate((0.0, depth / 2.0 + 0.010, 0.044))
    rear_strip = cq.Workplane("XY").box(width + 0.020, 0.060, 0.042).translate((0.0, 0.030, 0.041))
    front_lip = cq.Workplane("XY").box(width + 0.018, 0.046, 0.045).translate((0.0, depth + 0.008, 0.040))
    side_lip_0 = cq.Workplane("XY").box(0.038, depth + 0.030, 0.038).translate((width / 2.0 - 0.012, depth / 2.0 + 0.018, 0.047))
    side_lip_1 = cq.Workplane("XY").box(0.038, depth + 0.030, 0.038).translate((-width / 2.0 + 0.012, depth / 2.0 + 0.018, 0.047))
    shallow_center = cq.Workplane("XY").box(width - 0.160, depth - 0.175, 0.010).translate((0.0, depth / 2.0 + 0.040, 0.064))

    return panel.union(rear_strip).union(front_lip).union(side_lip_0).union(side_lip_1).union(shallow_center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    green = model.material("molded_green", rgba=(0.03, 0.32, 0.15, 1.0))
    dark_green = model.material("dark_molded_green", rgba=(0.015, 0.20, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.035, 0.038, 0.038, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))

    bin_body = model.part("bin")
    bin_body.visual(
        mesh_from_cadquery(_lofted_open_bin_body(), "tapered_hollow_body", tolerance=0.0015),
        material=green,
        name="body_shell",
    )

    # Rear axle and molded support yokes.  The yokes are part of the bin body,
    # so the body visibly rests on the axle rather than leaving the wheels
    # floating below a featureless tub.
    bin_body.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(0.0, -0.285, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    for i, x in enumerate((-0.235, 0.235)):
        bin_body.visual(
            Box((0.060, 0.085, 0.185)),
            origin=Origin(xyz=(x, -0.275, 0.205)),
            material=dark_green,
            name=f"axle_yoke_{i}",
        )
        bin_body.visual(
            Box((0.076, 0.100, 0.070)),
            origin=Origin(xyz=(x, -0.235, 0.315)),
            material=dark_green,
            name=f"axle_mount_{i}",
        )

    # Paired cheeks/brackets at the hinge line frame the full-width moving lid.
    for i, x in enumerate((-0.365, 0.365)):
        bin_body.visual(
            Box((0.052, 0.130, 0.155)),
            origin=Origin(xyz=(x, -0.335, 0.900)),
            material=dark_green,
            name=f"side_cheek_{i}",
        )
        bin_body.visual(
            Box((0.150, 0.055, 0.120)),
            origin=Origin(xyz=(x, -0.300, 0.830)),
            material=dark_green,
            name=f"cheek_root_{i}",
        )

    bin_body.visual(
        Cylinder(radius=0.010, length=0.780),
        origin=Origin(xyz=(0.0, -0.370, 0.925), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    bin_body.visual(
        Cylinder(radius=0.022, length=0.600),
        origin=Origin(xyz=(0.0, -0.356, 0.805), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_green,
        name="rear_handle",
    )
    for i, x in enumerate((-0.205, 0.205)):
        bin_body.visual(
            Box((0.045, 0.074, 0.052)),
            origin=Origin(xyz=(x, -0.322, 0.805)),
            material=dark_green,
            name=f"handle_standoff_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel(), "full_width_lid", tolerance=0.0012),
        material=green,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green,
        name="hinge_knuckle_0",
    )
    lid.visual(
        Cylinder(radius=0.022, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green,
        name="hinge_knuckle_1",
    )
    lid.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green,
        name="hinge_knuckle_2",
    )

    wheel_core = mesh_from_geometry(
        WheelGeometry(
            0.088,
            0.070,
            rim=WheelRim(inner_radius=0.058, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.032,
                width=0.086,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.042, hole_diameter=0.005),
            ),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "bin_wheel_core",
    )
    tire = mesh_from_geometry(
        TireGeometry(
            0.115,
            0.078,
            inner_radius=0.086,
            tread=TireTread(style="block", depth=0.005, count=20, land_ratio=0.58),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.007, radius=0.003),
        ),
        "bin_wheel_tire",
    )

    wheel_positions = ((-0.365, -0.285, 0.130), (0.365, -0.285, 0.130))
    wheels = []
    for i, _pos in enumerate(wheel_positions):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire, material=rubber, name="tire")
        wheel.visual(wheel_core, material=charcoal, name="rim")
        wheels.append(wheel)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=bin_body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.370, 0.925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=0.0, upper=1.95),
    )

    for i, (wheel, pos) in enumerate(zip(wheels, wheel_positions)):
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=bin_body,
            child=wheel,
            origin=Origin(xyz=pos),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bin_body = object_model.get_part("bin")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_spin_0 = object_model.get_articulation("wheel_spin_0")
    wheel_spin_1 = object_model.get_articulation("wheel_spin_1")

    for knuckle in ("hinge_knuckle_0", "hinge_knuckle_1", "hinge_knuckle_2"):
        ctx.allow_overlap(
            bin_body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The steel hinge pin is intentionally captured inside the lid hinge knuckle proxy.",
        )
    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            bin_body,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The rear axle is intentionally represented as passing through the rotating wheel hub.",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            bin_body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.50,
            name="closed lid covers the open bin mouth",
        )
        ctx.expect_within(
            bin_body,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem="hinge_knuckle_1",
            margin=0.002,
            name="hinge pin is captured inside the center lid knuckle",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid hinge lifts the front edge upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.25,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for wheel, spin in ((wheel_0, wheel_spin_0), (wheel_1, wheel_spin_1)):
        ctx.expect_within(
            bin_body,
            wheel,
            axes="yz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.004,
            name=f"{wheel.name} rim is centered on rear axle",
        )
        rest_pos = ctx.part_world_position(wheel)
        with ctx.pose({spin: math.pi}):
            spun_pos = ctx.part_world_position(wheel)
        ctx.check(
            f"{wheel.name} spins about a fixed axle center",
            rest_pos is not None and spun_pos is not None and all(abs(a - b) < 1e-7 for a, b in zip(rest_pos, spun_pos)),
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


object_model = build_object_model()
