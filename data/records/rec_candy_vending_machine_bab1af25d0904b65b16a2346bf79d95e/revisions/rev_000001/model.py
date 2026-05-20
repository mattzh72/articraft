from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


BODY_WIDTH = 0.42
BODY_DEPTH = 0.32
BODY_HEIGHT = 0.28
JAR_RADIUS = 0.175
JAR_HEIGHT = 0.44


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _ring_mesh(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z_min), (outer_radius, z_max)],
            [(inner_radius, z_min), (inner_radius, z_max)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=3,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_peanut_vendor")

    red_enamel = _mat("deep red enamel", (0.55, 0.03, 0.02, 1.0))
    chrome = _mat("polished chrome", (0.82, 0.80, 0.72, 1.0))
    dark_chrome = _mat("dark polished pedestal", (0.08, 0.075, 0.07, 1.0))
    glass = _mat("slightly blue transparent glass", (0.62, 0.88, 1.0, 0.28))
    peanut = _mat("visible peanuts", (0.88, 0.58, 0.22, 1.0))
    black = _mat("black rubber", (0.015, 0.012, 0.010, 1.0))
    brass = _mat("warm brass", (0.94, 0.70, 0.24, 1.0))
    label = _mat("cream painted label", (0.95, 0.84, 0.58, 1.0))

    # Root: a slim lobby-height pedestal made from touching circular members.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.19, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_chrome,
        name="floor_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=chrome,
        name="base_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=chrome,
        name="slim_post",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=chrome,
        name="top_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.188, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black,
        name="rubber_foot",
    )

    # The coin-mechanism body is a distinct stacked red casting.
    body = model.part("mechanism_body")
    body_shell = superellipse_side_loft(
        [
            (-BODY_DEPTH / 2, 0.0, BODY_HEIGHT, BODY_WIDTH),
            (BODY_DEPTH / 2, 0.0, BODY_HEIGHT, BODY_WIDTH),
        ],
        exponents=4.0,
        segments=64,
        cap=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "rounded_mechanism_body"),
        material=red_enamel,
        name="rounded_shell",
    )
    body.visual(
        Box((0.210, 0.012, 0.175)),
        origin=Origin(xyz=(0.0, -0.166, 0.145)),
        material=chrome,
        name="front_coin_plate",
    )
    body.visual(
        Box((0.090, 0.008, 0.018)),
        origin=Origin(xyz=(-0.070, -0.173, 0.215)),
        material=black,
        name="coin_slot",
    )
    body.visual(
        Box((0.105, 0.006, 0.040)),
        origin=Origin(xyz=(-0.070, -0.174, 0.250)),
        material=label,
        name="price_label",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.038),
        origin=Origin(xyz=(0.0, -0.179, 0.145), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="knob_bearing",
    )
    body.visual(
        Box((0.145, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.185, 0.045)),
        material=chrome,
        name="dispense_tray",
    )
    body.visual(
        Box((0.120, 0.006, 0.055)),
        origin=Origin(xyz=(0.0, -0.214, 0.078)),
        material=chrome,
        name="tray_front_lip",
    )
    body.visual(
        Box((0.012, 0.054, 0.045)),
        origin=Origin(xyz=(-0.072, -0.185, 0.068)),
        material=chrome,
        name="tray_side_0",
    )
    body.visual(
        Box((0.012, 0.054, 0.045)),
        origin=Origin(xyz=(0.072, -0.185, 0.068)),
        material=chrome,
        name="tray_side_1",
    )
    body.visual(
        Box((0.095, 0.010, 0.022)),
        origin=Origin(xyz=(0.117, -0.169, 0.112)),
        material=chrome,
        name="return_hinge_leaf",
    )

    model.articulation(
        "pedestal_to_body",
        ArticulationType.FIXED,
        parent=pedestal,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
    )

    # A separate cylindrical glass jar sits on the body rather than being merged
    # into the casting.  The peanut fill is one connected mound inside it.
    jar = model.part("glass_jar")
    jar.visual(
        _ring_mesh(0.188, 0.142, 0.000, 0.050, "jar_bottom_ring"),
        material=chrome,
        name="bottom_ring",
    )
    jar.visual(
        Cylinder(radius=0.162, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=glass,
        name="glass_floor",
    )
    jar.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(JAR_RADIUS, 0.040), (JAR_RADIUS, JAR_HEIGHT - 0.010)],
                [(JAR_RADIUS - 0.012, 0.047), (JAR_RADIUS - 0.012, JAR_HEIGHT - 0.017)],
                segments=96,
                start_cap="round",
                end_cap="round",
                lip_samples=8,
            ),
            "hollow_glass_cylinder",
        ),
        material=glass,
        name="glass_wall",
    )
    jar.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, 0.040),
                    (0.140, 0.040),
                    (0.151, 0.180),
                    (0.138, 0.250),
                    (0.070, 0.278),
                    (0.0, 0.282),
                ],
                segments=72,
                closed=True,
            ),
            "single_peanut_mound",
        ),
        material=peanut,
        name="peanut_mound",
    )
    jar.visual(
        _ring_mesh(0.185, 0.146, JAR_HEIGHT - 0.038, JAR_HEIGHT + 0.020, "jar_top_ring"),
        material=chrome,
        name="top_ring",
    )
    # Rear hinge support knuckles for the refill lid.
    jar.visual(
        Box((0.034, 0.056, 0.020)),
        origin=Origin(xyz=(-0.070, 0.174, JAR_HEIGHT + 0.006)),
        material=chrome,
        name="rear_hinge_strap_0",
    )
    jar.visual(
        Box((0.034, 0.056, 0.020)),
        origin=Origin(xyz=(0.070, 0.174, JAR_HEIGHT + 0.006)),
        material=chrome,
        name="rear_hinge_strap_1",
    )
    jar.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(-0.070, 0.198, JAR_HEIGHT + 0.020), rpy=(0.0, pi / 2, 0.0)),
        material=chrome,
        name="rear_hinge_knuckle_0",
    )
    jar.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.070, 0.198, JAR_HEIGHT + 0.020), rpy=(0.0, pi / 2, 0.0)),
        material=chrome,
        name="rear_hinge_knuckle_1",
    )

    model.articulation(
        "body_to_jar",
        ArticulationType.FIXED,
        parent=body,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
    )

    # Refill lid: a shallow domed cap whose local origin lies on its rear hinge.
    lid = model.part("refill_lid")
    lid.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, 0.000),
                    (0.170, 0.000),
                    (0.184, 0.018),
                    (0.164, 0.055),
                    (0.092, 0.086),
                    (0.0, 0.092),
                ],
                segments=96,
                closed=True,
            ).translate(0.0, -0.198, 0.0),
            "domed_refill_lid",
        ),
        material=chrome,
        name="domed_cap",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=chrome,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.030, 0.176, 0.008)),
        origin=Origin(xyz=(-0.026, -0.088, 0.004)),
        material=chrome,
        name="lid_hinge_strap_0",
    )
    lid.visual(
        Box((0.030, 0.176, 0.008)),
        origin=Origin(xyz=(0.026, -0.088, 0.004)),
        material=chrome,
        name="lid_hinge_strap_1",
    )

    model.articulation(
        "jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.198, JAR_HEIGHT + 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    # Front dispense knob: continuous rotation around the front shaft.
    knob = model.part("dispense_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=brass,
        name="front_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.095,
                0.040,
                body_style="faceted",
                top_diameter=0.082,
                edge_radius=0.003,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0025),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=True,
            ).rotate_x(pi / 2).translate(0.0, -0.050, 0.0),
            "ribbed_dispense_knob",
        ),
        material=brass,
        name="ribbed_dial",
    )
    knob.visual(
        Box((0.082, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.074, 0.000)),
        material=brass,
        name="turning_bar",
    )

    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, -0.198, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    # A separate coin-return cover beside the knob, hinged on its short outer edge.
    cover = model.part("coin_return_cover")
    cover.visual(
        Box((0.072, 0.012, 0.066)),
        origin=Origin(xyz=(-0.036, -0.006, 0.0)),
        material=chrome,
        name="return_flap",
    )
    cover.visual(
        Cylinder(radius=0.007, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="side_hinge_barrel",
    )
    cover.visual(
        Box((0.034, 0.004, 0.010)),
        origin=Origin(xyz=(-0.036, -0.014, -0.016)),
        material=black,
        name="finger_pull",
    )

    model.articulation(
        "body_to_coin_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.153, -0.181, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    body = object_model.get_part("mechanism_body")
    jar = object_model.get_part("glass_jar")
    lid = object_model.get_part("refill_lid")
    knob = object_model.get_part("dispense_knob")
    cover = object_model.get_part("coin_return_cover")
    lid_hinge = object_model.get_articulation("jar_to_lid")
    knob_joint = object_model.get_articulation("body_to_knob")
    cover_hinge = object_model.get_articulation("body_to_coin_cover")

    ctx.check(
        "distinct stacked pedestal body and jar",
        {p.name for p in object_model.parts}
        == {
            "pedestal",
            "mechanism_body",
            "glass_jar",
            "refill_lid",
            "dispense_knob",
            "coin_return_cover",
        },
        details="Pedestal, coin body, jar, lid, knob, and return cover should remain separate links.",
    )
    ctx.expect_gap(
        body,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="coin body sits on pedestal flange",
    )
    ctx.expect_gap(
        jar,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="glass jar stacks on mechanism body",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="front_shaft",
        elem_b="knob_bearing",
        contact_tol=0.002,
        name="knob shaft is seated at bearing",
    )
    ctx.check(
        "dispense knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {knob_joint.articulation_type}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 0.95}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "refill lid opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="return_flap")
    with ctx.pose({cover_hinge: 0.85}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="return_flap")
    ctx.check(
        "coin-return cover swings outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.010,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
