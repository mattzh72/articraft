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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    LatheGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PI = math.pi


def _daisy_wheel_mesh():
    """Perforated rotating top vent plate in local coordinates, z=0 at its seat."""
    height = 0.012
    body = cq.Workplane("XY").circle(0.075).extrude(height)
    for i in range(6):
        cutter = (
            cq.Workplane("XY")
            .rect(0.052, 0.016)
            .extrude(0.050)
            .translate((0.032, 0.0, -0.018))
            .rotate((0, 0, 0), (0, 0, 1), i * 60.0)
        )
        body = body.cut(cutter)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kamado_grill_wheeled_nest")

    ceramic = model.material("deep_green_ceramic", rgba=(0.04, 0.23, 0.14, 1.0))
    ceramic_dark = model.material("shadowed_green_ceramic", rgba=(0.025, 0.14, 0.09, 1.0))
    black_iron = model.material("black_powder_coated_steel", rgba=(0.005, 0.005, 0.005, 1.0))
    satin_steel = model.material("satin_hinge_steel", rgba=(0.47, 0.48, 0.46, 1.0))
    gasket = model.material("black_felt_gasket", rgba=(0.015, 0.013, 0.011, 1.0))
    tire_rubber = model.material("matte_black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    rim_metal = model.material("dark_silver_wheel_rim", rgba=(0.35, 0.34, 0.31, 1.0))

    # Root nest stand: a black tubular cradle, lower rails, axle, and front feet.
    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.355, 0.016, radial_segments=24, tubular_segments=96), "cradle_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        material=black_iron,
        name="cradle_ring",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.175, 0.012, radial_segments=20, tubular_segments=72), "lower_support_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=black_iron,
        name="lower_support_ring",
    )
    stand.visual(
        Cylinder(radius=0.175, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.3725)),
        material=black_iron,
        name="support_pan",
    )
    for i, angle in enumerate((0.0, PI / 2.0, PI, 3.0 * PI / 2.0)):
        stand.visual(
            Cylinder(radius=0.008, length=0.048),
            origin=Origin(xyz=(0.170 * math.cos(angle), 0.170 * math.sin(angle), 0.398)),
            material=black_iron,
            name=f"pan_post_{i}",
        )

    for x in (-0.235, 0.235):
        for y in (-0.235, 0.235):
            stand.visual(
                Cylinder(radius=0.015, length=0.620),
                origin=Origin(xyz=(x, y, 0.365)),
                material=black_iron,
                name=f"leg_{'rear' if x < 0 else 'front'}_{'side0' if y < 0 else 'side1'}",
            )
            angle = math.atan2(y, x)
            stand.visual(
                Cylinder(radius=0.010, length=0.172),
                origin=Origin(
                    xyz=(0.254 * math.cos(angle), 0.254 * math.sin(angle), 0.415),
                    rpy=(0.0, PI / 2.0, angle),
                ),
                material=black_iron,
                name=f"support_spoke_{'rear' if x < 0 else 'front'}_{'side0' if y < 0 else 'side1'}",
            )
    for y in (-0.235, 0.235):
        stand.visual(
            Cylinder(radius=0.013, length=0.560),
            origin=Origin(xyz=(0.0, y, 0.180), rpy=(0.0, PI / 2.0, 0.0)),
            material=black_iron,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.235, 0.235):
        stand.visual(
            Cylinder(radius=0.013, length=0.500),
            origin=Origin(xyz=(x, 0.0, 0.180), rpy=(-PI / 2.0, 0.0, 0.0)),
            material=black_iron,
            name=f"cross_rail_{0 if x < 0 else 1}",
        )
    stand.visual(
        Cylinder(radius=0.012, length=0.960),
        origin=Origin(xyz=(-0.365, 0.0, 0.150), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="rear_axle",
    )
    for y in (-0.245, 0.245):
        stand.visual(
            Box((0.150, 0.036, 0.034)),
            origin=Origin(xyz=(-0.305, y, 0.160)),
            material=black_iron,
            name=f"axle_mount_{0 if y < 0 else 1}",
        )
    for y in (-0.235, 0.235):
        stand.visual(
            Cylinder(radius=0.045, length=0.026),
            origin=Origin(xyz=(0.270, y, 0.043)),
            material=tire_rubber,
            name=f"front_foot_{0 if y < 0 else 1}",
        )

    # The fixed thick ceramic lower bowl is a real hollow lathe shell.
    lower_bowl = model.part("lower_bowl")
    lower_shell = LatheGeometry.from_shell_profiles(
        [
            (0.130, 0.420),
            (0.215, 0.480),
            (0.300, 0.610),
            (0.340, 0.790),
            (0.326, 0.930),
        ],
        [
            (0.078, 0.470),
            (0.165, 0.530),
            (0.250, 0.665),
            (0.286, 0.880),
            (0.260, 0.910),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    lower_bowl.visual(
        mesh_from_geometry(lower_shell, "lower_bowl_shell"),
        material=ceramic,
        name="ceramic_bowl",
    )
    lower_bowl.visual(
        mesh_from_geometry(TorusGeometry(0.326, 0.010, radial_segments=20, tubular_segments=96), "lower_rim_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=gasket,
        name="rim_gasket",
    )
    lower_bowl.visual(
        mesh_from_geometry(TorusGeometry(0.335, 0.009, radial_segments=18, tubular_segments=96), "lower_steel_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=black_iron,
        name="lower_band",
    )
    lower_bowl.visual(
        Cylinder(radius=0.136, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.407)),
        material=ceramic_dark,
        name="ceramic_foot",
    )
    # Rear hinge hardware fixed to the lower bowl: side knuckles and brackets.
    lower_bowl.visual(
        Cylinder(radius=0.007, length=0.350),
        origin=Origin(xyz=(-0.372, 0.0, 0.965), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_pin",
    )
    for y in (-0.132, 0.132):
        lower_bowl.visual(
            Cylinder(radius=0.026, length=0.072),
            origin=Origin(xyz=(-0.372, y, 0.965), rpy=(-PI / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"hinge_knuckle_{0 if y < 0 else 1}",
        )
        lower_bowl.visual(
            Box((0.075, 0.030, 0.090)),
            origin=Origin(xyz=(-0.335, y, 0.925)),
            material=satin_steel,
            name=f"hinge_leaf_{0 if y < 0 else 1}",
        )

    model.articulation(
        "stand_to_bowl",
        ArticulationType.FIXED,
        parent=stand,
        child=lower_bowl,
        origin=Origin(),
    )

    # Lid frame is exactly on the rear hinge axis.  The dome extends forward (+X)
    # from that axis in the closed pose, so -Y is the positive-opening axis.
    lid = model.part("lid")
    lid_shell = LatheGeometry.from_shell_profiles(
        [
            (0.337, 0.000),
            (0.330, 0.050),
            (0.294, 0.175),
            (0.212, 0.315),
            (0.110, 0.405),
            (0.054, 0.438),
        ],
        [
            (0.284, 0.035),
            (0.274, 0.080),
            (0.232, 0.196),
            (0.162, 0.306),
            (0.078, 0.376),
            (0.038, 0.395),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    lid.visual(
        mesh_from_geometry(lid_shell, "lid_dome_shell"),
        origin=Origin(xyz=(0.372, 0.0, 0.006)),
        material=ceramic,
        name="dome_shell",
    )
    lid.visual(
        mesh_from_geometry(TorusGeometry(0.337, 0.010, radial_segments=18, tubular_segments=96), "lid_steel_band"),
        origin=Origin(xyz=(0.372, 0.0, 0.028)),
        material=black_iron,
        name="lid_band",
    )
    lid.visual(
        Cylinder(radius=0.056, length=0.040),
        origin=Origin(xyz=(0.372, 0.0, 0.456)),
        material=black_iron,
        name="vent_collar",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.112, 0.055, 0.030)),
        origin=Origin(xyz=(0.052, 0.0, 0.038)),
        material=satin_steel,
        name="rear_clip",
    )
    lid.visual(
        Box((0.100, 0.200, 0.018)),
        origin=Origin(xyz=(0.105, 0.0, 0.022)),
        material=satin_steel,
        name="lid_hinge_strap",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_bowl,
        child=lid,
        origin=Origin(xyz=(-0.372, 0.0, 0.965)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.18),
    )

    # Daisy-wheel cap with separate rotation about the lid crown fastener.
    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        mesh_from_cadquery(_daisy_wheel_mesh(), "daisy_wheel_plate", tolerance=0.0008, angular_tolerance=0.08),
        material=black_iron,
        name="daisy_plate",
    )
    vent_cap.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_steel,
        name="central_fastener",
    )
    vent_cap.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=satin_steel,
        name="fastener_stem",
    )
    for i in range(3):
        angle = i * 2.0 * PI / 3.0
        vent_cap.visual(
            Box((0.050, 0.014, 0.010)),
            origin=Origin(
                xyz=(0.045 * math.cos(angle), 0.045 * math.sin(angle), 0.017),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_steel,
            name=f"vent_tab_{i}",
        )
    model.articulation(
        "vent_spin",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.372, 0.0, 0.476)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # Two transport wheels spin on the rear axle.  Each wheel is a tire plus
    # visible hub/spokes, not a plain cylinder.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.090,
            0.050,
            rim=WheelRim(inner_radius=0.055, flange_height=0.006, flange_thickness=0.004, bead_seat_depth=0.003),
            hub=WheelHub(
                radius=0.027,
                width=0.038,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.034, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.118,
            0.060,
            inner_radius=0.088,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "wheel_tire",
    )
    for y, suffix in ((-0.455, "0"), (0.455, "1")):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, PI / 2.0)),
            material=tire_rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, PI / 2.0)),
            material=rim_metal,
            name="wheel_core",
        )
        model.articulation(
            f"wheel_spin_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=stand,
            child=wheel,
            origin=Origin(xyz=(-0.365, y, 0.150)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    # Record intended samples for the main hinge so QC sees both closed and open poses.
    lid_hinge.meta["qc_samples"] = [0.0, 0.75, 1.18]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    lower_bowl = object_model.get_part("lower_bowl")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # Local, mechanically meaningful intersections: a hinge pin captured inside
    # the lid barrel, a vent fastener stem in its crown collar, and the rear axle
    # through wheel hubs.
    ctx.allow_overlap(
        lower_bowl,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured inside the lid-side hinge barrel.",
    )
    ctx.expect_overlap(
        lower_bowl,
        lid,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.12,
        name="hinge pin runs through lid barrel",
    )
    ctx.expect_within(
        lid,
        lower_bowl,
        axes="xz",
        inner_elem="hinge_barrel",
        outer_elem="hinge_pin",
        margin=0.026,
        name="lid barrel is centered on hinge pin",
    )

    ctx.allow_overlap(
        vent_cap,
        lid,
        elem_a="fastener_stem",
        elem_b="vent_collar",
        reason="The daisy-wheel cap rotates on a central fastener seated inside the crown collar.",
    )
    ctx.expect_within(
        vent_cap,
        lid,
        axes="xy",
        inner_elem="fastener_stem",
        outer_elem="vent_collar",
        margin=0.002,
        name="vent stem is centered in crown collar",
    )
    ctx.expect_overlap(
        vent_cap,
        lid,
        axes="z",
        elem_a="fastener_stem",
        elem_b="vent_collar",
        min_overlap=0.018,
        name="vent fastener remains inserted in collar",
    )

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            stand,
            wheel,
            elem_a="rear_axle",
            elem_b="wheel_core",
            reason="The transport wheel hub is intentionally captured around the fixed axle.",
        )
        ctx.expect_overlap(
            stand,
            wheel,
            axes="y",
            elem_a="rear_axle",
            elem_b="wheel_core",
            min_overlap=0.030,
            name=f"{wheel.name} hub is on the axle",
        )

    ctx.allow_overlap(
        stand,
        lower_bowl,
        elem_a="support_pan",
        elem_b="ceramic_foot",
        reason="The heavy ceramic foot is intentionally seated with slight compression on the nest support pan.",
    )
    ctx.expect_gap(
        lower_bowl,
        stand,
        axis="z",
        positive_elem="ceramic_foot",
        negative_elem="support_pan",
        max_penetration=0.003,
        max_gap=0.001,
        name="ceramic foot rests on support pan",
    )

    # The lower ceramic body reads as seated in the nest: its foot is inside the
    # lower support ring and the wide belly is retained by the cradle ring.
    ctx.expect_within(
        lower_bowl,
        stand,
        axes="xy",
        inner_elem="ceramic_foot",
        outer_elem="lower_support_ring",
        margin=0.020,
        name="ceramic foot sits inside lower support ring",
    )
    ctx.expect_within(
        lower_bowl,
        stand,
        axes="xy",
        inner_elem="ceramic_bowl",
        outer_elem="cradle_ring",
        margin=0.030,
        name="bowl belly is retained by tubular cradle",
    )

    with ctx.pose({lid_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            lower_bowl,
            axis="z",
            positive_elem="dome_shell",
            negative_elem="rim_gasket",
            min_gap=-0.002,
            max_gap=0.025,
            name="closed lid seats above lower gasket",
        )
    with ctx.pose({lid_hinge: 1.18}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            lower_bowl,
            axis="z",
            positive_elem="dome_shell",
            negative_elem="rim_gasket",
            min_gap=0.030,
            name="opened lid clears lower gasket",
        )
    ctx.check(
        "lid hinge raises heavy dome",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
