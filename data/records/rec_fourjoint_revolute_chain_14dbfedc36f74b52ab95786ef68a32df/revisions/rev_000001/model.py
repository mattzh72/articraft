from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_THICKNESS = 0.014
RAIL_WIDTH = 0.025
RAIL_SPACING = 0.045
LINK_LAYER_GAP = 0.011
LOW_LAYER_Z = 0.0
HIGH_LAYER_Z = PLATE_THICKNESS + LINK_LAYER_GAP
CHAIN_HEIGHT = 0.220


def _add_bolt_heads(part, material: Material) -> None:
    for ix, x in enumerate((-0.150, 0.150)):
        for iy, y in enumerate((-0.085, 0.085)):
            part.visual(
                Cylinder(radius=0.012, length=0.0012),
                origin=Origin(xyz=(x, y, 0.0256)),
                material=material,
                name=f"base_bolt_{ix}_{iy}",
            )


def _add_ladder_link(
    part,
    *,
    length: float,
    layer_z: float,
    material: Material,
    dark_material: Material,
    previous_layer_z: float | None = None,
) -> None:
    """Two long side plates with cross-rungs and round pivot bosses."""
    rail_length = length + 0.030
    for side_name, y in (("side_plate_0", -RAIL_SPACING), ("side_plate_1", RAIL_SPACING)):
        part.visual(
            Box((rail_length, RAIL_WIDTH, PLATE_THICKNESS)),
            origin=Origin(xyz=(length / 2.0, y, layer_z)),
            material=material,
            name=side_name,
        )

    for rung_name, x, width in (
        ("proximal_rung", 0.0, 0.058),
        ("middle_rung", length / 2.0, 0.040),
        ("distal_rung", length, 0.058),
    ):
        part.visual(
            Box((width, 2.0 * RAIL_SPACING + RAIL_WIDTH, PLATE_THICKNESS)),
            origin=Origin(xyz=(x, 0.0, layer_z)),
            material=material,
            name=rung_name,
        )

    for boss_name, x in (("proximal_boss", 0.0), ("distal_boss", length)):
        part.visual(
            Cylinder(radius=0.052, length=PLATE_THICKNESS),
            origin=Origin(xyz=(x, 0.0, layer_z)),
            material=material,
            name=boss_name,
        )
        part.visual(
            Cylinder(radius=0.018, length=0.0008),
            origin=Origin(xyz=(x, 0.0, layer_z + PLATE_THICKNESS / 2.0 + 0.0001)),
            material=dark_material,
            name=f"{boss_name}_bore_shadow",
        )

    if previous_layer_z is not None:
        child_lower = layer_z - PLATE_THICKNESS / 2.0
        child_upper = layer_z + PLATE_THICKNESS / 2.0
        parent_lower = previous_layer_z - PLATE_THICKNESS / 2.0
        parent_upper = previous_layer_z + PLATE_THICKNESS / 2.0
        if layer_z > previous_layer_z:
            spacer_length = child_lower - parent_upper
            spacer_center = (child_lower + parent_upper) / 2.0
        else:
            spacer_length = parent_lower - child_upper
            spacer_center = (parent_lower + child_upper) / 2.0
        if spacer_length > 0.0:
            part.visual(
                Cylinder(radius=0.015, length=spacer_length),
                origin=Origin(xyz=(0.0, 0.0, spacer_center)),
                material=dark_material,
                name="proximal_spacer",
            )


def _add_end_tab(
    part,
    *,
    length: float,
    layer_z: float,
    material: Material,
    dark_material: Material,
    previous_layer_z: float,
) -> None:
    """Compact final output tab with a short boss and a small mounting hole mark."""
    part.visual(
        Box((length, 0.076, PLATE_THICKNESS)),
        origin=Origin(xyz=(length / 2.0, 0.0, layer_z)),
        material=material,
        name="tab_plate",
    )
    for boss_name, x, radius in (
        ("proximal_boss", 0.0, 0.050),
        ("tip_boss", length, 0.035),
    ):
        part.visual(
            Cylinder(radius=radius, length=PLATE_THICKNESS),
            origin=Origin(xyz=(x, 0.0, layer_z)),
            material=material,
            name=boss_name,
        )
        part.visual(
            Cylinder(radius=0.014 if boss_name == "proximal_boss" else 0.010, length=0.0008),
            origin=Origin(xyz=(x, 0.0, layer_z + PLATE_THICKNESS / 2.0 + 0.0001)),
            material=dark_material,
            name=f"{boss_name}_bore_shadow",
        )

    child_upper = layer_z + PLATE_THICKNESS / 2.0
    parent_lower = previous_layer_z - PLATE_THICKNESS / 2.0
    spacer_length = parent_lower - child_upper
    if spacer_length > 0.0:
        part.visual(
            Cylinder(radius=0.015, length=spacer_length),
            origin=Origin(xyz=(0.0, 0.0, (parent_lower + child_upper) / 2.0)),
            material=dark_material,
            name="proximal_spacer",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_four_joint_chain")

    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    blue = model.material("blue_anodized_plates", rgba=(0.10, 0.24, 0.58, 1.0))
    graphite = model.material("graphite_pin_faces", rgba=(0.02, 0.022, 0.025, 1.0))
    dark = model.material("black_oxide_hardware", rgba=(0.01, 0.01, 0.012, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        Box((0.420, 0.260, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=steel,
        name="base_plate",
    )
    root_bracket.visual(
        Box((0.120, 0.135, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=steel,
        name="pedestal",
    )
    root_bracket.visual(
        Box((0.112, 0.200, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, CHAIN_HEIGHT - PLATE_THICKNESS / 2.0 - 0.008)),
        material=steel,
        name="top_saddle",
    )
    for side_name, y in (("clevis_cheek_0", -0.085), ("clevis_cheek_1", 0.085)):
        root_bracket.visual(
            Box((0.085, 0.014, 0.105)),
            origin=Origin(xyz=(0.0, y, CHAIN_HEIGHT)),
            material=steel,
            name=side_name,
        )
    for pin_name, y in (("root_pin_0", -0.085), ("root_pin_1", 0.085)):
        root_bracket.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(0.0, y, CHAIN_HEIGHT), rpy=(1.57079632679, 0.0, 0.0)),
            material=dark,
            name=pin_name,
        )
    _add_bolt_heads(root_bracket, dark)

    lengths = (0.300, 0.280, 0.260, 0.240)
    layer_zs = (LOW_LAYER_Z, HIGH_LAYER_Z, LOW_LAYER_Z, HIGH_LAYER_Z)
    links = [model.part(f"link_{i}") for i in range(4)]
    _add_ladder_link(
        links[0],
        length=lengths[0],
        layer_z=layer_zs[0],
        material=blue,
        dark_material=graphite,
    )
    for i in range(1, 4):
        _add_ladder_link(
            links[i],
            length=lengths[i],
            layer_z=layer_zs[i],
            material=blue,
            dark_material=graphite,
            previous_layer_z=layer_zs[i - 1],
        )

    end_tab = model.part("end_tab")
    _add_end_tab(
        end_tab,
        length=0.125,
        layer_z=LOW_LAYER_Z,
        material=blue,
        dark_material=graphite,
        previous_layer_z=layer_zs[-1],
    )

    model.articulation(
        "bracket_to_link_0",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, CHAIN_HEIGHT)),
    )

    limits = MotionLimits(effort=18.0, velocity=2.2, lower=-1.75, upper=1.75)
    serial_children = [links[1], links[2], links[3], end_tab]
    for i, child in enumerate(serial_children):
        model.articulation(
            f"link_{i}_to_{'end_tab' if i == 3 else f'link_{i + 1}'}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=child,
            origin=Origin(xyz=(lengths[i], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    link_names = ["link_0", "link_1", "link_2", "link_3", "end_tab"]
    revolute_names = [
        "link_0_to_link_1",
        "link_1_to_link_2",
        "link_2_to_link_3",
        "link_3_to_end_tab",
    ]
    links = [object_model.get_part(name) for name in link_names]
    revolute_joints = [object_model.get_articulation(name) for name in revolute_names]

    ctx.check(
        "five rigid links in series",
        all(link is not None for link in links),
        details=f"links={links}",
    )
    ctx.check(
        "four serial revolute joints",
        len(revolute_joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in revolute_joints),
        details=f"joints={revolute_joints}",
    )
    ctx.check(
        "parallel revolute axes",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0) for joint in revolute_joints),
        details=[joint.axis for joint in revolute_joints],
    )

    ctx.expect_contact(
        "link_0",
        "root_bracket",
        elem_a="proximal_boss",
        elem_b="top_saddle",
        contact_tol=0.0005,
        name="root bracket saddle supports first link",
    )
    ctx.expect_overlap(
        "link_0",
        "root_bracket",
        axes="xy",
        elem_a="proximal_boss",
        elem_b="top_saddle",
        min_overlap=0.070,
        name="root bracket centered under first pivot",
    )

    for parent_name, child_name, child_above in (
        ("link_0", "link_1", True),
        ("link_1", "link_2", False),
        ("link_2", "link_3", True),
        ("link_3", "end_tab", False),
    ):
        positive = child_name if child_above else parent_name
        negative = parent_name if child_above else child_name
        positive_elem = "proximal_boss" if child_above else "distal_boss"
        negative_elem = "distal_boss" if child_above else "proximal_boss"
        ctx.expect_gap(
            positive,
            negative,
            axis="z",
            min_gap=0.009,
            max_gap=0.013,
            positive_elem=positive_elem,
            negative_elem=negative_elem,
            name=f"{parent_name} to {child_name} has plate clearance",
        )
        ctx.expect_contact(
            child_name,
            parent_name,
            elem_a="proximal_spacer",
            elem_b="distal_boss",
            contact_tol=0.0005,
            name=f"{child_name} spacer bears on {parent_name} pivot",
        )

    rest_tip = ctx.part_world_position("end_tab")
    with ctx.pose({"link_0_to_link_1": 0.65}):
        swept_tip = ctx.part_world_position("end_tab")
    ctx.check(
        "positive joint motion sweeps chain in plane",
        rest_tip is not None and swept_tip is not None and swept_tip[1] > rest_tip[1] + 0.20,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    return ctx.report()


object_model = build_object_model()
