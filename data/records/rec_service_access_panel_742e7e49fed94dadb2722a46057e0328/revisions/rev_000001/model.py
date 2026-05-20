from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    BezelRecess,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_service_access_panel")

    matte_graphite = model.material("matte_graphite", rgba=(0.075, 0.082, 0.088, 1.0))
    soft_black = model.material("soft_black", rgba=(0.006, 0.007, 0.008, 1.0))
    satin_titanium = model.material("satin_titanium", rgba=(0.58, 0.60, 0.60, 1.0))
    satin_edge = model.material("satin_edge", rgba=(0.42, 0.44, 0.44, 1.0))
    shadow_rubber = model.material("shadow_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    warm_fastener = model.material("brushed_fastener", rgba=(0.70, 0.69, 0.65, 1.0))

    enclosure = model.part("enclosure")
    panel = model.part("panel")
    latch = model.part("latch")

    # Fixed enclosure face: a single refined framed aperture rather than a
    # solid placeholder.  Local bezel XY maps to world XZ; depth maps along Y.
    enclosure_bezel = BezelGeometry(
        opening_size=(0.540, 0.360),
        outer_size=(0.740, 0.540),
        depth=0.050,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.030,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
        recess=BezelRecess(depth=0.006, inset=0.006, floor_radius=0.002),
    )
    enclosure.visual(
        mesh_from_geometry(enclosure_bezel, "enclosure_bezel"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="main_bezel",
    )

    # Dark, supported service opening liner and rear shadow wall.  These pieces
    # tuck under the inner lip so the aperture reads as a real framed opening.
    enclosure.visual(
        Box((0.012, 0.052, 0.364)),
        origin=Origin(xyz=(-0.276, 0.002, 0.0)),
        material=soft_black,
        name="left_liner",
    )
    enclosure.visual(
        Box((0.012, 0.052, 0.364)),
        origin=Origin(xyz=(0.276, 0.002, 0.0)),
        material=soft_black,
        name="right_liner",
    )
    enclosure.visual(
        Box((0.552, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, 0.186)),
        material=soft_black,
        name="top_liner",
    )
    enclosure.visual(
        Box((0.552, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, -0.186)),
        material=soft_black,
        name="bottom_liner",
    )
    enclosure.visual(
        Box((0.552, 0.008, 0.364)),
        origin=Origin(xyz=(0.0, 0.029, 0.0)),
        material=soft_black,
        name="shadow_back",
    )

    # Hairline reveal around the removable panel face.
    for name, xyz, size in (
        ("top_reveal", (0.0, -0.027, 0.199), (0.610, 0.003, 0.006)),
        ("bottom_reveal", (0.0, -0.027, -0.199), (0.610, 0.003, 0.006)),
        ("hinge_reveal", (-0.300, -0.027, 0.0), (0.006, 0.003, 0.392)),
        ("latch_reveal", (0.270, -0.027, 0.0), (0.006, 0.003, 0.392)),
    ):
        enclosure.visual(Box(size), origin=Origin(xyz=xyz), material=shadow_rubber, name=name)

    # Latch strike plate and restrained fasteners on the fixed frame.
    enclosure.visual(
        Box((0.020, 0.006, 0.122)),
        origin=Origin(xyz=(0.300, -0.029, 0.0)),
        material=satin_edge,
        name="strike_plate",
    )
    enclosure.visual(
        Box((0.004, 0.002, 0.050)),
        origin=Origin(xyz=(0.290, -0.033, 0.0)),
        material=soft_black,
        name="strike_slot",
    )

    screw_disk = Cylinder(radius=0.0085, length=0.004)
    screw_pose = (math.pi / 2.0, 0.0, 0.0)
    for index, (x, z) in enumerate(
        (
            (-0.315, 0.220),
            (0.315, 0.220),
            (-0.315, -0.220),
            (0.315, -0.220),
            (-0.205, 0.238),
            (0.205, 0.238),
            (-0.205, -0.238),
            (0.205, -0.238),
        )
    ):
        enclosure.visual(
            screw_disk,
            origin=Origin(xyz=(x, -0.027, z), rpy=screw_pose),
            material=warm_fastener,
            name=f"frame_screw_{index}",
        )

    # Fixed half of the exposed hinge: leaf attached to enclosure and alternate
    # satin knuckles around the real pivot line.
    enclosure.visual(
        Box((0.044, 0.005, 0.338)),
        origin=Origin(xyz=(-0.319, -0.028, 0.0)),
        material=satin_edge,
        name="frame_hinge_leaf",
    )
    for index, z in enumerate((-0.066, 0.066)):
        enclosure.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(-0.295, -0.037, z)),
            material=satin_titanium,
            name=f"frame_knuckle_{index}",
        )

    for index, z in enumerate((-0.122, 0.122)):
        enclosure.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(xyz=(-0.319, -0.0295, z), rpy=screw_pose),
            material=warm_fastener,
            name=f"hinge_leaf_screw_{index}",
        )

    for index, z in enumerate((-0.040, 0.040)):
        enclosure.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(xyz=(0.300, -0.033, z), rpy=screw_pose),
            material=warm_fastener,
            name=f"strike_screw_{index}",
        )

    # Hinged service panel.  The part frame is on the vertical pivot line; the
    # panel geometry extends in +X so the revolute joint reads mechanically.
    panel.visual(
        Box((0.550, 0.018, 0.370)),
        origin=Origin(xyz=(0.286, 0.0, 0.0)),
        material=matte_graphite,
        name="door_slab",
    )
    panel.visual(
        Box((0.468, 0.004, 0.278)),
        origin=Origin(xyz=(0.288, -0.011, 0.0)),
        material=Material("slightly_satin_face", rgba=(0.115, 0.123, 0.128, 1.0)),
        name="satin_inset",
    )
    panel.visual(
        Box((0.018, 0.006, 0.336)),
        origin=Origin(xyz=(0.524, -0.012, 0.0)),
        material=satin_edge,
        name="latch_edge_rail",
    )
    panel.visual(
        Box((0.018, 0.006, 0.336)),
        origin=Origin(xyz=(0.034, -0.012, 0.0)),
        material=satin_edge,
        name="hinge_edge_rail",
    )
    for name, xyz, size in (
        ("door_top_seam", (0.278, -0.014, 0.164), (0.500, 0.003, 0.005)),
        ("door_bottom_seam", (0.278, -0.014, -0.164), (0.500, 0.003, 0.005)),
        ("door_latch_seam", (0.512, -0.014, 0.0), (0.005, 0.003, 0.315)),
        ("door_hinge_seam", (0.044, -0.014, 0.0), (0.005, 0.003, 0.315)),
    ):
        panel.visual(Box(size), origin=Origin(xyz=xyz), material=shadow_rubber, name=name)

    # Moving half of the hinge, interleaved with the frame knuckles.
    panel.visual(
        Box((0.046, 0.006, 0.338)),
        origin=Origin(xyz=(0.024, -0.0100, 0.0)),
        material=satin_edge,
        name="panel_hinge_leaf",
    )
    for index, z in enumerate((-0.132, 0.0, 0.132)):
        panel.visual(
            Cylinder(radius=0.008, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_titanium,
            name=f"panel_knuckle_{index}",
        )
    for index, z in enumerate((-0.126, 0.126)):
        panel.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(xyz=(0.024, -0.0145, z), rpy=screw_pose),
            material=warm_fastener,
            name=f"panel_hinge_screw_{index}",
        )

    for index, (x, z) in enumerate(
        (
            (0.078, 0.145),
            (0.470, 0.145),
            (0.078, -0.145),
            (0.470, -0.145),
            (0.525, 0.096),
            (0.525, -0.096),
        )
    ):
        panel.visual(
            Cylinder(radius=0.0058, length=0.003),
            origin=Origin(xyz=(x, -0.0105, z), rpy=screw_pose),
            material=warm_fastener,
            name=f"door_screw_{index}",
        )

    # Quarter-turn latch control mounted on the service panel face.  The collar
    # is a contact surface; the cap and indicator turn about the panel normal.
    latch.visual(
        Cylinder(radius=0.033, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=screw_pose),
        material=satin_edge,
        name="latch_collar",
    )
    latch_knob = KnobGeometry(
        0.052,
        0.014,
        body_style="faceted",
        base_diameter=0.056,
        top_diameter=0.046,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=16, depth=0.0006, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    latch.visual(
        mesh_from_geometry(latch_knob, "latch_knob"),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="latch_cap",
    )
    latch.visual(
        Box((0.050, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.0195, 0.0)),
        material=soft_black,
        name="latch_indicator",
    )

    panel_hinge = model.articulation(
        "enclosure_to_panel",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=panel,
        origin=Origin(xyz=(-0.295, -0.037, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.70),
    )
    panel_hinge.meta["description"] = "Vertical service-panel hinge; positive rotation swings the panel outward."

    latch_joint = model.articulation(
        "panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(0.490, -0.009, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=math.pi / 2.0),
    )
    latch_joint.meta["description"] = "Flush quarter-turn latch control on the panel face."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("enclosure_to_panel")
    latch_joint = object_model.get_articulation("panel_to_latch")

    with ctx.pose({hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            enclosure,
            panel,
            axis="y",
            positive_elem="main_bezel",
            negative_elem="door_slab",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed panel keeps a tight face reveal",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="xz",
            elem_a="door_slab",
            elem_b="main_bezel",
            min_overlap=0.32,
            name="closed panel covers the framed service opening",
        )
        ctx.expect_contact(
            latch,
            panel,
            elem_a="latch_collar",
            elem_b="door_slab",
            contact_tol=0.0008,
            name="latch collar seats on panel face",
        )

    closed_panel = ctx.part_element_world_aabb(panel, elem="door_slab")
    with ctx.pose({hinge: 1.25}):
        opened_panel = ctx.part_element_world_aabb(panel, elem="door_slab")
    ctx.check(
        "hinged panel swings outward",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[0][1] < closed_panel[0][1] - 0.15,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    with ctx.pose({latch_joint: 0.0}):
        latch_zero = ctx.part_element_world_aabb(latch, elem="latch_indicator")
    with ctx.pose({latch_joint: math.pi / 2.0}):
        latch_turn = ctx.part_element_world_aabb(latch, elem="latch_indicator")
    if latch_zero is not None and latch_turn is not None:
        zero_width = latch_zero[1][0] - latch_zero[0][0]
        zero_height = latch_zero[1][2] - latch_zero[0][2]
        turn_width = latch_turn[1][0] - latch_turn[0][0]
        turn_height = latch_turn[1][2] - latch_turn[0][2]
        latch_motion_ok = zero_width > zero_height * 4.0 and turn_height > turn_width * 4.0
    else:
        latch_motion_ok = False
    ctx.check(
        "quarter turn latch visibly rotates",
        latch_motion_ok,
        details=f"zero={latch_zero}, turned={latch_turn}",
    )

    return ctx.report()


object_model = build_object_model()
