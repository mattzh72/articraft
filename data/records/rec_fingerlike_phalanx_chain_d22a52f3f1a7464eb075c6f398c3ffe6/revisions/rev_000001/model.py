from __future__ import annotations

import math

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
    mesh_from_geometry,
    superellipse_side_loft,
)


HINGE_Z = 0.125
PROXIMAL_LEN = 0.165
MIDDLE_LEN = 0.132
PIN_CAP_NAMES = ("distal_pin_cap_0", "distal_pin_cap_1")
BASE_PIN_CAP_NAMES = ("base_pin_cap_0", "base_pin_cap_1")


def _cylinder_y(part, *, name: str, radius: float, length: float, xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, *, name: str, radius: float, length: float, xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _make_tapered_body_mesh(
    *,
    length: float,
    width_0: float,
    width_1: float,
    height_0: float,
    height_1: float,
    start_x: float,
    end_x: float,
    mesh_name: str,
):
    """Rounded rectangular tapered beam, authored along +X in the part frame."""
    mid_x = (start_x + end_x) * 0.5
    sections = [
        (start_x, -height_0 * 0.50, height_0 * 0.50, width_0),
        (mid_x, -((height_0 + height_1) * 0.25), (height_0 + height_1) * 0.25, (width_0 + width_1) * 0.5),
        (end_x, -height_1 * 0.50, height_1 * 0.50, width_1),
    ]
    geom = superellipse_side_loft(sections, exponents=3.8, segments=40, cap=True)
    # superellipse_side_loft runs along local +Y; rotate it so the segment runs along +X.
    geom.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _add_fasteners_on_side_cover(
    part,
    *,
    side_index: int,
    y: float,
    x0: float,
    x1: float,
    material: Material,
) -> None:
    for screw_index, x in enumerate((x0, x1)):
        _cylinder_y(
            part,
            name=f"cover_screw_{side_index}_{screw_index}",
            radius=0.0022,
            length=0.0028,
            xyz=(x, y, 0.0),
            material=material,
        )


def _add_phalanx_segment(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width_0: float,
    width_1: float,
    height_0: float,
    height_1: float,
    bearing_radius: float,
    bearing_length: float,
    parent_gap: float,
    distal_gap: float | None,
    materials: dict[str, Material],
):
    segment = model.part(name)
    body_start = bearing_radius + 0.019
    body_end = length - (0.047 if distal_gap is not None else 0.026)
    body_mesh = _make_tapered_body_mesh(
        length=length,
        width_0=width_0,
        width_1=width_1,
        height_0=height_0,
        height_1=height_1,
        start_x=body_start,
        end_x=body_end,
        mesh_name=f"{name}_tapered_body",
    )
    segment.visual(body_mesh, material=materials["aluminum"], name="tapered_body")

    # The proximal bearing is the moving lug that sits between the parent cheeks.
    _cylinder_y(
        segment,
        name="proximal_bearing",
        radius=bearing_radius,
        length=bearing_length,
        xyz=(0.0, 0.0, 0.0),
        material=materials["bronze"],
    )
    neck_len = body_start - bearing_radius + 0.010
    segment.visual(
        Box((neck_len, bearing_length * 0.78, height_0 * 0.62)),
        origin=Origin(xyz=(bearing_radius + neck_len * 0.5 - 0.004, 0.0, 0.0)),
        material=materials["aluminum"],
        name="bearing_neck",
    )
    spacer_engagement = 0.0003
    spacer_len = max(0.0025, parent_gap * 0.5 - bearing_length * 0.5 + spacer_engagement)
    for idx, side in enumerate((-1.0, 1.0)):
        y = side * (bearing_length * 0.5 + spacer_len * 0.5 - spacer_engagement)
        _cylinder_y(
            segment,
            name=f"joint_spacer_{idx}",
            radius=bearing_radius * 1.08,
            length=spacer_len,
            xyz=(0.0, y, 0.0),
            material=materials["dark_steel"],
        )

    # Removable side access covers with visible screw heads.
    cover_len = max(0.040, (body_end - body_start) * 0.56)
    cover_x = (body_start + body_end) * 0.5
    cover_y = (width_0 + width_1) * 0.25 + 0.0010
    cover_height = min(height_0, height_1) * 0.56
    for side_index, side in enumerate((-1.0, 1.0)):
        segment.visual(
            Box((cover_len, 0.0030, cover_height)),
            origin=Origin(xyz=(cover_x, side * cover_y, 0.0)),
            material=materials["cover"],
            name=f"access_cover_{side_index}",
        )
        _add_fasteners_on_side_cover(
            segment,
            side_index=side_index,
            y=side * (cover_y + 0.0020),
            x0=cover_x - cover_len * 0.36,
            x1=cover_x + cover_len * 0.36,
            material=materials["fastener"],
        )

    # Dorsal tendon-guide stand-in: a small rail, feet, and a transverse eyelet.
    guide_z = max(height_0, height_1) * 0.5 + 0.010
    guide_len = (body_end - body_start) * 0.45
    guide_x = cover_x
    segment.visual(
        Box((guide_len, 0.0070, 0.0050)),
        origin=Origin(xyz=(guide_x, 0.0, guide_z)),
        material=materials["black"],
        name="guide_rail",
    )
    for foot_index, x in enumerate((guide_x - guide_len * 0.42, guide_x + guide_len * 0.42)):
        segment.visual(
            Box((0.0080, 0.0130, 0.0140)),
            origin=Origin(xyz=(x, 0.0, guide_z - 0.0065)),
            material=materials["dark_steel"],
            name=f"guide_foot_{foot_index}",
        )
    _cylinder_x(
        segment,
        name="guide_eyelet",
        radius=0.0048,
        length=0.0190,
        xyz=(guide_x, 0.0, guide_z + 0.0065),
        material=materials["black"],
    )

    # Stop pads around the proximal lug make the allowed flexion range legible.
    segment.visual(
        Box((0.0140, bearing_length * 0.58, 0.0060)),
        origin=Origin(xyz=(bearing_radius * 0.65, 0.0, bearing_radius)),
        material=materials["stop"],
        name="proximal_stop_pad",
    )

    if distal_gap is not None:
        cheek_thickness = 0.0090
        cheek_height = bearing_radius * 2.85
        cheek_len = 0.046
        cheek_center_x = length - 0.005
        cheek_y = distal_gap * 0.5 + cheek_thickness * 0.5
        web_x = length - 0.049
        segment.visual(
            Box((0.042, distal_gap + 2.0 * cheek_thickness, height_1 * 0.70)),
            origin=Origin(xyz=(web_x, 0.0, 0.0)),
            material=materials["aluminum"],
            name="distal_bridge",
        )
        for cheek_index, side in enumerate((-1.0, 1.0)):
            segment.visual(
                Box((cheek_len, cheek_thickness, cheek_height)),
                origin=Origin(xyz=(cheek_center_x, side * cheek_y, 0.0)),
                material=materials["aluminum"],
                name=f"distal_cheek_{cheek_index}",
            )
            cap_y = side * (distal_gap * 0.5 + cheek_thickness + 0.0030)
            _cylinder_y(
                segment,
                name=PIN_CAP_NAMES[cheek_index],
                radius=bearing_radius * 0.66,
                length=0.0060,
                xyz=(length, cap_y, 0.0),
                material=materials["fastener"],
            )
        segment.visual(
            Box((0.0240, distal_gap + 0.0020, 0.0070)),
            origin=Origin(xyz=(length - 0.027, 0.0, bearing_radius + 0.0060)),
            material=materials["stop"],
            name="flexion_stop_block",
        )
        segment.visual(
            Box((0.0240, distal_gap + 0.0020, 0.0050)),
            origin=Origin(xyz=(length - 0.027, 0.0, -bearing_radius - 0.0040)),
            material=materials["dark_steel"],
            name="extension_stop_block",
        )
    else:
        # Terminal protective cap and small measuring tongue keep the last link mechanical.
        _cylinder_x(
            segment,
            name="terminal_cap",
            radius=max(height_1 * 0.58, 0.010),
            length=0.026,
            xyz=(length - 0.006, 0.0, 0.0),
            material=materials["black"],
        )
        segment.visual(
            Box((0.035, width_1 * 0.42, 0.0045)),
            origin=Origin(xyz=(length - 0.030, 0.0, -height_1 * 0.5 - 0.003)),
            material=materials["dark_steel"],
            name="tip_stop_tongue",
        )
    return segment


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_phalanx_chain_study")

    materials = {
        "aluminum": model.material("bead_blasted_aluminum", rgba=(0.62, 0.64, 0.62, 1.0)),
        "dark_steel": model.material("phosphate_steel", rgba=(0.08, 0.085, 0.09, 1.0)),
        "fastener": model.material("black_oxide_fasteners", rgba=(0.015, 0.014, 0.013, 1.0)),
        "bronze": model.material("oilite_bronze_bushings", rgba=(0.73, 0.52, 0.24, 1.0)),
        "cover": model.material("dark_access_covers", rgba=(0.12, 0.13, 0.14, 1.0)),
        "black": model.material("matte_black_guides", rgba=(0.02, 0.022, 0.024, 1.0)),
        "stop": model.material("muted_urethane_stops", rgba=(0.45, 0.07, 0.04, 1.0)),
    }

    base = model.part("base")
    base.visual(
        Box((0.545, 0.165, 0.018)),
        origin=Origin(xyz=(0.205, 0.0, 0.009)),
        material=materials["dark_steel"],
        name="base_plate",
    )
    base.visual(
        Box((0.040, 0.120, 0.012)),
        origin=Origin(xyz=(-0.035, 0.0, 0.024)),
        material=materials["aluminum"],
        name="clevis_foot",
    )
    base_gap = 0.054
    base_cheek_thickness = 0.010
    base_cheek_y = base_gap * 0.5 + base_cheek_thickness * 0.5
    for cheek_index, side in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.060, base_cheek_thickness, 0.146)),
            origin=Origin(xyz=(0.0, side * base_cheek_y, 0.018 + 0.146 * 0.5)),
            material=materials["aluminum"],
            name=f"base_cheek_{cheek_index}",
        )
        _cylinder_y(
            base,
            name=BASE_PIN_CAP_NAMES[cheek_index],
            radius=0.0125,
            length=0.0060,
            xyz=(0.0, side * (base_gap * 0.5 + base_cheek_thickness + 0.0030), HINGE_Z),
            material=materials["fastener"],
        )
    base.visual(
        Box((0.035, base_gap + 2.0 * base_cheek_thickness, 0.088)),
        origin=Origin(xyz=(-0.032, 0.0, 0.018 + 0.088 * 0.5)),
        material=materials["aluminum"],
        name="clevis_backbone",
    )
    base.visual(
        Box((0.020, base_gap + 0.002, 0.008)),
        origin=Origin(xyz=(-0.014, 0.0, HINGE_Z + 0.030)),
        material=materials["stop"],
        name="base_stop_block",
    )
    # Standalone test-fixture details: an anchor rail and calibration bosses on the plate.
    for rail_index, y in enumerate((-0.060, 0.060)):
        base.visual(
            Box((0.390, 0.007, 0.008)),
            origin=Origin(xyz=(0.235, y, 0.022)),
            material=materials["black"],
            name=f"fixture_rail_{rail_index}",
        )
    for boss_index, x in enumerate((0.105, 0.305)):
        _cylinder_y(
            base,
            name=f"tendon_anchor_boss_{boss_index}",
            radius=0.012,
            length=0.030,
            xyz=(x, -0.060, 0.036),
            material=materials["bronze"],
        )

    proximal = _add_phalanx_segment(
        model,
        name="proximal",
        length=PROXIMAL_LEN,
        width_0=0.058,
        width_1=0.048,
        height_0=0.031,
        height_1=0.026,
        bearing_radius=0.019,
        bearing_length=0.044,
        parent_gap=base_gap,
        distal_gap=0.046,
        materials=materials,
    )
    middle = _add_phalanx_segment(
        model,
        name="middle",
        length=MIDDLE_LEN,
        width_0=0.048,
        width_1=0.039,
        height_0=0.026,
        height_1=0.022,
        bearing_radius=0.017,
        bearing_length=0.036,
        parent_gap=0.046,
        distal_gap=0.039,
        materials=materials,
    )
    distal = _add_phalanx_segment(
        model,
        name="distal",
        length=0.106,
        width_0=0.039,
        width_1=0.030,
        height_0=0.022,
        height_1=0.018,
        bearing_radius=0.0145,
        bearing_length=0.031,
        parent_gap=0.039,
        distal_gap=None,
        materials=materials,
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=0.0, upper=1.15),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.6, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")
    base_joint = object_model.get_articulation("base_to_proximal")
    prox_joint = object_model.get_articulation("proximal_to_middle")
    mid_joint = object_model.get_articulation("middle_to_distal")

    ctx.expect_origin_gap(
        middle,
        proximal,
        axis="x",
        min_gap=0.155,
        max_gap=0.175,
        name="middle hinge is at proximal distal knuckle",
    )
    ctx.expect_origin_gap(
        distal,
        middle,
        axis="x",
        min_gap=0.122,
        max_gap=0.142,
        name="distal hinge is at middle distal knuckle",
    )
    ctx.expect_overlap(
        proximal,
        base,
        axes="xz",
        elem_a="proximal_bearing",
        elem_b="base_pin_cap_0",
        min_overlap=0.010,
        name="base pin cap aligns with proximal bearing",
    )
    ctx.expect_overlap(
        middle,
        proximal,
        axes="xz",
        elem_a="proximal_bearing",
        elem_b="distal_pin_cap_0",
        min_overlap=0.009,
        name="proximal distal pin aligns with middle bearing",
    )
    ctx.expect_overlap(
        distal,
        middle,
        axes="xz",
        elem_a="proximal_bearing",
        elem_b="distal_pin_cap_0",
        min_overlap=0.008,
        name="middle distal pin aligns with distal bearing",
    )

    rest_tip = ctx.part_element_world_aabb(distal, elem="terminal_cap")
    with ctx.pose({base_joint: 0.45, prox_joint: 0.80, mid_joint: 0.60}):
        curled_tip = ctx.part_element_world_aabb(distal, elem="terminal_cap")
    ctx.check(
        "hinge chain curls downward from visible pin axes",
        rest_tip is not None
        and curled_tip is not None
        and curled_tip[0][2] < rest_tip[0][2] - 0.045
        and curled_tip[1][0] < rest_tip[1][0] - 0.025,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
    )

    return ctx.report()


object_model = build_object_model()
