from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LINK_LENGTHS = (0.28, 0.27, 0.26, 0.25, 0.18)
HINGE_AXIS = (0.0, 1.0, 0.0)
AXIS_CYLINDER = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _cyl_y(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose native Z axis is turned into the chain Y axis."""

    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_root_bracket(model: ArticulatedObject):
    bracket = model.part("root_bracket")
    bracket.visual(
        Box((0.24, 0.22, 0.035)),
        origin=Origin(xyz=(-0.050, 0.0, -0.142)),
        material="painted_steel",
        name="floor_plate",
    )
    bracket.visual(
        Box((0.070, 0.120, 0.150)),
        origin=Origin(xyz=(-0.075, 0.0, -0.066)),
        material="painted_steel",
        name="boxed_riser",
    )
    bracket.visual(
        Box((0.085, 0.026, 0.135)),
        origin=Origin(xyz=(-0.012, 0.060, -0.026)),
        material="painted_steel",
        name="cheek_plate_0",
    )
    bracket.visual(
        Box((0.085, 0.026, 0.135)),
        origin=Origin(xyz=(-0.012, -0.060, -0.026)),
        material="painted_steel",
        name="cheek_plate_1",
    )
    bracket.visual(
        Box((0.042, 0.145, 0.038)),
        origin=Origin(xyz=(-0.066, 0.0, 0.012)),
        material="painted_steel",
        name="rear_tie_bar",
    )
    for index, y in enumerate((0.060, -0.060)):
        bracket.visual(
            Cylinder(radius=0.044, length=0.026),
            origin=_cyl_y(0.0, y, 0.0),
            material="dark_bushing",
            name=f"root_boss_{index}",
        )
    bracket.visual(
        Cylinder(radius=0.010, length=0.154),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material="black_pin",
        name="root_pin",
    )
    return bracket


def _add_chain_link(
    model: ArticulatedObject,
    name: str,
    *,
    length: float,
    has_distal_joint: bool,
    is_tip: bool = False,
):
    link = model.part(name)

    side_y = 0.052
    side_thickness_y = 0.024
    plate_height = 0.032
    plate_start = 0.075
    plate_end = length + (0.008 if has_distal_joint else 0.020)
    plate_length = plate_end - plate_start

    link.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material="dark_bushing",
        name="proximal_barrel",
    )
    link.visual(
        Box((0.068, 0.032, 0.026)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material="brushed_steel",
        name="proximal_neck",
    )
    link.visual(
        Box((0.034, 0.126, 0.026)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material="brushed_steel",
        name="proximal_spacer",
    )
    for index, y in enumerate((side_y, -side_y)):
        link.visual(
            Box((plate_length, side_thickness_y, plate_height)),
            origin=Origin(xyz=((plate_start + plate_end) / 2.0, y, 0.0)),
            material="brushed_steel",
            name=f"side_plate_{index}",
        )
    link.visual(
        Box((0.040, 0.126, 0.026)),
        origin=Origin(xyz=(max(length - 0.054, 0.120), 0.0, 0.0)),
        material="brushed_steel",
        name="distal_spacer",
    )

    if has_distal_joint:
        for index, y in enumerate((side_y, -side_y)):
            link.visual(
                Cylinder(radius=0.038, length=0.026),
                origin=_cyl_y(length, y, 0.0),
                material="dark_bushing",
                name=f"distal_boss_{index}",
            )
        link.visual(
            Cylinder(radius=0.010, length=0.154),
            origin=_cyl_y(length, 0.0, 0.0),
            material="black_pin",
            name="distal_pin",
        )
    if is_tip:
        link.visual(
            Box((0.060, 0.112, 0.030)),
            origin=Origin(xyz=(length + 0.030, 0.0, 0.0)),
            material="brushed_steel",
            name="end_tab_web",
        )
        link.visual(
            Cylinder(radius=0.032, length=0.056),
            origin=_cyl_y(length + 0.064, 0.0, 0.0),
            material="brushed_steel",
            name="end_tab_round",
        )
        link.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=_cyl_y(length + 0.064, 0.0, 0.0),
            material="black_pin",
            name="tab_hole_liner",
        )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_four_joint_revolute_chain")
    model.material("painted_steel", rgba=(0.93, 0.58, 0.16, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.68, 0.67, 1.0))
    model.material("dark_bushing", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("black_pin", rgba=(0.015, 0.015, 0.018, 1.0))

    bracket = _add_root_bracket(model)
    links = [
        _add_chain_link(
            model,
            f"link_{index}",
            length=length,
            has_distal_joint=index < 4,
            is_tip=index == 4,
        )
        for index, length in enumerate(LINK_LENGTHS)
    ]

    model.articulation(
        "root_bracket_to_link_0",
        ArticulationType.FIXED,
        parent=bracket,
        child=links[0],
        origin=Origin(),
        axis=HINGE_AXIS,
    )

    for index in range(4):
        model.articulation(
            f"link_{index}_to_link_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(LINK_LENGTHS[index], 0.0, 0.0)),
            axis=HINGE_AXIS,
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=2.2,
                lower=-1.35,
                upper=1.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    links = [object_model.get_part(f"link_{index}") for index in range(5)]
    bracket = object_model.get_part("root_bracket")

    ctx.check(
        "four serial revolute joints",
        sum(
            1
            for joint in object_model.articulations
            if joint.articulation_type == ArticulationType.REVOLUTE
        )
        == 4,
        details="The requested planar chain should have exactly four revolute joints.",
    )
    for index in range(4):
        joint = object_model.get_articulation(f"link_{index}_to_link_{index + 1}")
        ctx.check(
            f"joint_{index}_axis_parallel",
            tuple(round(component, 6) for component in joint.axis) == HINGE_AXIS,
            details=f"{joint.name} axis={joint.axis}; expected parallel Y hinge axes.",
        )

    ctx.allow_overlap(
        bracket,
        links[0],
        elem_a="root_pin",
        elem_b="proximal_barrel",
        reason="The grounded bracket pin is intentionally captured inside the first link barrel.",
    )
    ctx.expect_within(
        bracket,
        links[0],
        axes="xz",
        inner_elem="root_pin",
        outer_elem="proximal_barrel",
        name="root pin passes through first barrel",
    )
    ctx.expect_overlap(
        bracket,
        links[0],
        axes="y",
        elem_a="root_pin",
        elem_b="proximal_barrel",
        min_overlap=0.045,
        name="root pin spans first barrel width",
    )

    for index in range(4):
        parent = links[index]
        child = links[index + 1]
        ctx.allow_overlap(
            parent,
            child,
            elem_a="distal_pin",
            elem_b="proximal_barrel",
            reason="Each hinge pin is intentionally captured inside the next link's bearing barrel.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem="distal_pin",
            outer_elem="proximal_barrel",
            name=f"joint_{index}_pin_centered_in_barrel",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a="distal_pin",
            elem_b="proximal_barrel",
            min_overlap=0.045,
            name=f"joint_{index}_pin_spans_barrel_width",
        )

    for index in range(4):
        ctx.expect_origin_gap(
            links[index + 1],
            links[index],
            axis="x",
            min_gap=LINK_LENGTHS[index] - 0.001,
            max_gap=LINK_LENGTHS[index] + 0.001,
            name=f"rest_origin_spacing_{index}",
        )

    driven_joint = object_model.get_articulation("link_1_to_link_2")
    with ctx.pose({driven_joint: 0.65}):
        plane_positions = [ctx.part_world_position(link) for link in links]
        ctx.check(
            "articulation_stays_in_xz_plane",
            all(pos is not None and abs(pos[1]) < 1.0e-6 for pos in plane_positions),
            details=f"link origins under pose: {plane_positions}",
        )
        link_3_pos = ctx.part_world_position(links[3])
        link_3_rest_x = sum(LINK_LENGTHS[:3])
        ctx.check(
            "revolute_pose_moves_downstream_links",
            link_3_pos is not None and abs(link_3_pos[0] - link_3_rest_x) > 0.010,
            details=f"link_3 posed position={link_3_pos}, rest_x={link_3_rest_x}",
        )

    return ctx.report()


object_model = build_object_model()
