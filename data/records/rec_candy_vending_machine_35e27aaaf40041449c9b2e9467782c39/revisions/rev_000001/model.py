from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_center: float):
    """A simple vertical ring, authored in world coordinates for the root part."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )


def _bowl_shell():
    """Thin transparent sphere with vertical top and bottom throat openings."""
    outer = cq.Workplane("XY").sphere(0.285)
    inner = cq.Workplane("XY").sphere(0.270)
    throat = cq.Workplane("XY").circle(0.090).extrude(0.700, both=True)
    return outer.cut(inner).cut(throat).translate((0.0, 0.0, 1.130))


def _cast_housing():
    """Rounded cast mechanism housing with a real through-bore for the knob shaft."""
    body = cq.Workplane("XY").box(0.44, 0.34, 0.30)
    body = body.edges("|Z").fillet(0.040)
    bore = cq.Workplane("XZ").center(0.0, 0.020).circle(0.046).extrude(0.62, both=True)
    return body.cut(bore).translate((0.0, 0.0, 0.720))


def _faceplate():
    """Chrome front coin-mechanism plate, with a matching clearance hole."""
    plate = cq.Workplane("XY").box(0.270, 0.014, 0.205)
    hole = cq.Workplane("XZ").center(0.0, 0.0).circle(0.060).extrude(0.060, both=True)
    plate = plate.cut(hole).edges("|Y").fillet(0.006)
    return plate.translate((0.0, -0.177, 0.740))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_gumball_machine")

    red = model.material("candy_red_enamel", rgba=(0.78, 0.05, 0.035, 1.0))
    red_dark = model.material("shadowed_red_cast", rgba=(0.48, 0.025, 0.020, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    dark = model.material("black_chute_shadow", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("clear_bowl_plastic", rgba=(0.72, 0.90, 1.00, 0.28))
    amber = model.material("warm_clear_flap", rgba=(1.00, 0.72, 0.36, 0.42))
    white = model.material("white_gumball", rgba=(0.96, 0.94, 0.82, 1.0))
    yellow = model.material("yellow_gumball", rgba=(1.00, 0.80, 0.06, 1.0))
    blue = model.material("blue_gumball", rgba=(0.04, 0.30, 0.92, 1.0))
    green = model.material("green_gumball", rgba=(0.05, 0.62, 0.18, 1.0))
    pink = model.material("pink_gumball", rgba=(0.96, 0.15, 0.42, 1.0))

    machine = model.part("machine")

    # Floor pedestal, column, and cast lower mechanism.
    machine.visual(
        Cylinder(radius=0.255, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=red,
        name="round_pedestal_foot",
    )
    machine.visual(
        Cylinder(radius=0.060, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=chrome,
        name="pedestal_column",
    )
    machine.visual(
        Cylinder(radius=0.125, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=red,
        name="pedestal_cap",
    )
    machine.visual(
        mesh_from_cadquery(_cast_housing(), "cast_mechanism_housing", tolerance=0.0015),
        material=red,
        name="cast_housing",
    )
    machine.visual(
        mesh_from_cadquery(_faceplate(), "front_coin_plate", tolerance=0.001),
        material=chrome,
        name="coin_plate",
    )
    machine.visual(
        Box((0.095, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.186, 0.835)),
        material=dark,
        name="coin_slot",
    )
    machine.visual(
        Box((0.145, 0.006, 0.085)),
        origin=Origin(xyz=(0.0, -0.187, 0.585)),
        material=dark,
        name="chute_dark",
    )
    machine.visual(
        Box((0.210, 0.030, 0.065)),
        origin=Origin(xyz=(0.0, -0.171, 0.575)),
        material=red_dark,
        name="chute_cast_lip",
    )

    # Bowl collars and transparent spherical bowl.
    machine.visual(
        mesh_from_cadquery(_annular_cylinder(0.175, 0.092, 0.085, 0.885), "lower_bowl_ring"),
        material=red,
        name="lower_bowl_ring",
    )
    machine.visual(
        mesh_from_cadquery(_bowl_shell(), "clear_bowl_shell", tolerance=0.0015),
        material=glass,
        name="clear_spherical_bowl",
    )
    machine.visual(
        mesh_from_cadquery(_annular_cylinder(0.142, 0.082, 0.035, 1.415), "top_crown_ring"),
        material=red,
        name="top_crown",
    )
    machine.visual(
        Cylinder(radius=0.105, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.397)),
        material=red_dark,
        name="crown_inner_shadow",
    )

    # Rear refill hinge hardware fixed to the crown.
    for i, x in enumerate((-0.100, 0.100)):
        machine.visual(
            Box((0.030, 0.060, 0.060)),
            origin=Origin(xyz=(x, 0.135, 1.435)),
            material=red,
            name=f"lid_hinge_lug_{i}",
        )
        machine.visual(
            Cylinder(radius=0.009, length=0.040),
            origin=Origin(xyz=(x, 0.150, 1.470), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"lid_hinge_knuckle_{i}",
        )

    # Lower chute hinge hardware fixed to the cast housing.
    for i, x in enumerate((-0.100, 0.100)):
        machine.visual(
            Box((0.030, 0.030, 0.035)),
            origin=Origin(xyz=(x, -0.190, 0.525)),
            material=red,
            name=f"flap_hinge_ear_{i}",
        )
        machine.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(xyz=(x, -0.212, 0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"flap_hinge_knuckle_{i}",
        )

    # A connected mound of visible gumballs inside the clear bowl.
    gumballs = [
        ((-0.072, -0.030, 0.910), yellow),
        ((-0.022, -0.048, 0.908), pink),
        ((0.030, -0.020, 0.912), blue),
        ((0.076, 0.018, 0.910), green),
        ((-0.044, 0.024, 0.950), white),
        ((0.006, 0.030, 0.952), yellow),
        ((0.056, -0.030, 0.954), pink),
        ((-0.085, 0.040, 0.990), blue),
        ((-0.030, -0.008, 0.995), green),
        ((0.025, 0.014, 0.996), white),
        ((0.080, 0.040, 0.992), yellow),
        ((-0.055, 0.000, 1.038), pink),
        ((0.000, 0.048, 1.040), blue),
        ((0.055, -0.002, 1.038), green),
        ((0.000, 0.010, 1.084), white),
    ]
    for i, (xyz, material) in enumerate(gumballs):
        machine.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=xyz),
            material=material,
            name=f"gumball_{i}",
        )

    # Articulated rear-hinged refill lid.
    lid = model.part("refill_lid")
    lid.visual(
        Cylinder(radius=0.130, length=0.018),
        origin=Origin(xyz=(0.0, -0.150, -0.025)),
        material=red,
        name="lid_cap",
    )
    lid.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, -0.150, -0.006)),
        material=chrome,
        name="lid_pull",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.075, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, -0.010)),
        material=red,
        name="lid_hinge_bridge",
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=machine,
        child=lid,
        origin=Origin(xyz=(0.0, 0.150, 1.470)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    # Continuously rotating dispensing knob and shaft.
    knob = model.part("dispensing_knob")
    knob.visual(
        Cylinder(radius=0.017, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.100,
                0.058,
                body_style="lobed",
                base_diameter=0.070,
                top_diameter=0.092,
                crown_radius=0.004,
                grip=KnobGrip(style="ribbed", count=12, depth=0.003),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "lobed_dispensing_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_collar",
    )
    model.articulation(
        "knob_shaft",
        ArticulationType.CONTINUOUS,
        parent=machine,
        child=knob,
        origin=Origin(xyz=(0.0, -0.184, 0.740)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    # Lower-hinged candy door flap at the chute.
    flap = model.part("candy_flap")
    flap.visual(
        Box((0.165, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=amber,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.174),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.085, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.090)),
        material=chrome,
        name="flap_pull_lip",
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=machine,
        child=flap,
        origin=Origin(xyz=(0.0, -0.212, 0.515)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    machine = object_model.get_part("machine")
    lid = object_model.get_part("refill_lid")
    knob = object_model.get_part("dispensing_knob")
    flap = object_model.get_part("candy_flap")
    lid_hinge = object_model.get_articulation("lid_hinge")
    knob_shaft = object_model.get_articulation("knob_shaft")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.check(
        "dispensing knob is continuous",
        knob_shaft.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_shaft.articulation_type}",
    )
    ctx.check(
        "knob shaft axis is horizontal through housing",
        abs(knob_shaft.axis[1]) > 0.99 and abs(knob_shaft.axis[2]) < 0.01,
        details=f"axis={knob_shaft.axis}",
    )

    ctx.expect_gap(
        lid,
        machine,
        axis="z",
        positive_elem="lid_cap",
        negative_elem="top_crown",
        min_gap=0.0,
        max_gap=0.010,
        name="refill lid rests just above crown ring",
    )
    ctx.expect_overlap(
        lid,
        machine,
        axes="xy",
        elem_a="lid_cap",
        elem_b="top_crown",
        min_overlap=0.090,
        name="refill lid covers crown opening",
    )
    ctx.expect_contact(
        knob,
        machine,
        elem_a="knob_collar",
        elem_b="coin_plate",
        contact_tol=0.001,
        name="knob collar bears on faceplate",
    )
    ctx.expect_gap(
        machine,
        flap,
        axis="y",
        positive_elem="chute_dark",
        negative_elem="flap_panel",
        min_gap=0.004,
        max_gap=0.025,
        name="flap sits in front of chute",
    )

    lid_rest = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 0.90}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "refill lid opens upward on rear hinge",
        lid_rest is not None
        and lid_open is not None
        and lid_open[1][2] > lid_rest[1][2] + 0.060,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    flap_rest = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.90}):
        flap_open = ctx.part_world_aabb(flap)
    ctx.check(
        "candy flap swings outward and downward",
        flap_rest is not None
        and flap_open is not None
        and flap_open[0][1] < flap_rest[0][1] - 0.030
        and flap_open[1][2] < flap_rest[1][2] - 0.015,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    return ctx.report()


object_model = build_object_model()
