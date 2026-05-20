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
)


HINGE_AXIS = (0.0, 1.0, 0.0)


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """A cylinder descriptor/origin pair aligned to the local/world Y axis."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _angle_for_vector(dx: float, dz: float) -> float:
    # Box local +X is mapped into the XZ plane by a pitch about Y.
    return -math.atan2(dz, dx)


def _box_along(
    part,
    *,
    center: tuple[float, float, float],
    length: float,
    width_y: float,
    thickness_z: float,
    angle: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Box((length, width_y, thickness_z)),
        origin=Origin(xyz=center, rpy=(0.0, angle, 0.0)),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    cyl, orient = _cyl_y(radius, length)
    part.visual(
        cyl,
        origin=Origin(xyz=xyz, rpy=orient.rpy),
        material=material,
        name=name,
    )


def _add_central_hub(
    part,
    *,
    radius: float,
    width: float,
    material: Material,
    name: str,
) -> None:
    _add_y_cylinder(
        part,
        radius=radius,
        length=width,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name=name,
    )


def _add_yoke(
    part,
    *,
    pivot: tuple[float, float, float],
    back_direction: tuple[float, float, float],
    cheek_radius: float,
    cheek_width: float,
    cheek_y: float,
    bridge_offset: float,
    bridge_length: float,
    bridge_thickness: float,
    material: Material,
    cheek_prefix: str,
    bridge_name: str,
) -> None:
    px, _, pz = pivot
    bx, _, bz = back_direction
    back_angle = _angle_for_vector(bx, bz)
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        _add_y_cylinder(
            part,
            radius=cheek_radius,
            length=cheek_width,
            xyz=(px, sign * cheek_y, pz),
            material=material,
            name=f"{cheek_prefix}_{label}",
        )

    bridge_center = (px + bx * bridge_offset, 0.0, pz + bz * bridge_offset)
    _box_along(
        part,
        center=bridge_center,
        length=bridge_length,
        width_y=2.0 * cheek_y + cheek_width + 0.012,
        thickness_z=bridge_thickness,
        angle=back_angle,
        material=material,
        name=bridge_name,
    )


def _add_link_with_distal_yoke(
    part,
    *,
    distal: tuple[float, float, float],
    hub_radius: float,
    hub_width: float,
    beam_width: float,
    beam_thickness: float,
    cheek_radius: float,
    cheek_width: float,
    cheek_y: float,
    link_material: Material,
    rib_material: Material,
    hub_name: str,
    beam_name: str,
    rib_prefix: str,
    cheek_prefix: str,
    bridge_name: str,
) -> float:
    dx, _, dz = distal
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    angle = _angle_for_vector(dx, dz)

    _add_central_hub(part, radius=hub_radius, width=hub_width, material=link_material, name=hub_name)

    beam_start = 0.025
    beam_end = length - 0.085
    beam_len = beam_end - beam_start
    beam_center_dist = (beam_start + beam_end) / 2.0
    _box_along(
        part,
        center=(ux * beam_center_dist, 0.0, uz * beam_center_dist),
        length=beam_len,
        width_y=beam_width,
        thickness_z=beam_thickness,
        angle=angle,
        material=link_material,
        name=beam_name,
    )

    rib_len = beam_len * 0.72
    rib_center_dist = beam_center_dist - 0.004
    for sign in (1.0, -1.0):
        _box_along(
            part,
            center=(ux * rib_center_dist, sign * (beam_width / 2.0 + 0.002), uz * rib_center_dist),
            length=rib_len,
            width_y=0.010,
            thickness_z=beam_thickness * 0.46,
            angle=angle,
            material=rib_material,
            name=f"{rib_prefix}_{'pos' if sign > 0 else 'neg'}",
        )

    _add_yoke(
        part,
        pivot=distal,
        back_direction=(-ux, 0.0, -uz),
        cheek_radius=cheek_radius,
        cheek_width=cheek_width,
        cheek_y=cheek_y,
        bridge_offset=0.068,
        bridge_length=0.050,
        bridge_thickness=0.058,
        material=link_material,
        cheek_prefix=cheek_prefix,
        bridge_name=bridge_name,
    )
    return length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_inspection_arm")

    anodized = model.material("warm_anodized_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = model.material("dark_machined_edges", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("brushed_steel", rgba=(0.76, 0.76, 0.72, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("wall_plate")
    # Shoulder frame sits on the hinge axis.  The wall plate and standoffs are
    # behind it, leaving the central lug clear for rotation.
    base.visual(
        Box((0.030, 0.360, 0.340)),
        origin=Origin(xyz=(-0.120, 0.0, 0.0)),
        material=dark,
        name="back_plate",
    )
    base.visual(
        Box((0.042, 0.188, 0.104)),
        origin=Origin(xyz=(-0.084, 0.0, 0.0)),
        material=anodized,
        name="rear_bridge",
    )
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        base.visual(
            Box((0.125, 0.042, 0.074)),
            origin=Origin(xyz=(-0.033, sign * 0.064, 0.0)),
            material=anodized,
            name=f"shoulder_rail_{label}",
        )
    _add_yoke(
        base,
        pivot=(0.0, 0.0, 0.0),
        back_direction=(-1.0, 0.0, 0.0),
        cheek_radius=0.058,
        cheek_width=0.036,
        cheek_y=0.066,
        bridge_offset=0.084,
        bridge_length=0.038,
        bridge_thickness=0.096,
        material=anodized,
        cheek_prefix="shoulder_cheek",
        bridge_name="shoulder_back_boss",
    )
    _add_y_cylinder(
        base,
        radius=0.014,
        length=0.160,
        xyz=(0.0, 0.0, 0.0),
        material=steel,
        name="shoulder_pin",
    )
    for y in (-0.125, 0.125):
        for z in (-0.115, 0.115):
            _add_y_cylinder(
                base,
                radius=0.014,
                length=0.010,
                xyz=(-0.100, y, z),
                material=steel,
                name=f"bolt_{'pos' if y > 0 else 'neg'}_{'top' if z > 0 else 'bottom'}",
            )

    shoulder_link = model.part("shoulder_link")
    l1_distal = (0.455, 0.0, 0.155)
    l1_len = _add_link_with_distal_yoke(
        shoulder_link,
        distal=l1_distal,
        hub_radius=0.044,
        hub_width=0.066,
        beam_width=0.046,
        beam_thickness=0.036,
        cheek_radius=0.052,
        cheek_width=0.032,
        cheek_y=0.062,
        link_material=anodized,
        rib_material=dark,
        hub_name="shoulder_hub",
        beam_name="long_web",
        rib_prefix="long_rib",
        cheek_prefix="elbow_cheek",
        bridge_name="elbow_yoke_bridge",
    )
    _add_y_cylinder(
        shoulder_link,
        radius=0.0135,
        length=0.150,
        xyz=l1_distal,
        material=steel,
        name="elbow_pin",
    )

    elbow_link = model.part("elbow_link")
    l2_distal = (0.315, 0.0, -0.130)
    l2_len = _add_link_with_distal_yoke(
        elbow_link,
        distal=l2_distal,
        hub_radius=0.040,
        hub_width=0.060,
        beam_width=0.042,
        beam_thickness=0.032,
        cheek_radius=0.048,
        cheek_width=0.030,
        cheek_y=0.058,
        link_material=anodized,
        rib_material=dark,
        hub_name="elbow_hub",
        beam_name="middle_web",
        rib_prefix="middle_rib",
        cheek_prefix="wrist_cheek",
        bridge_name="wrist_yoke_bridge",
    )
    _add_y_cylinder(
        elbow_link,
        radius=0.013,
        length=0.142,
        xyz=l2_distal,
        material=steel,
        name="wrist_pin",
    )

    wrist_link = model.part("wrist_link")
    l3_distal = (0.235, 0.0, 0.050)
    dx3, _, dz3 = l3_distal
    l3_len = math.hypot(dx3, dz3)
    ux3, uz3 = dx3 / l3_len, dz3 / l3_len
    angle3 = _angle_for_vector(dx3, dz3)
    _add_central_hub(
        wrist_link,
        radius=0.038,
        width=0.056,
        material=anodized,
        name="wrist_hub",
    )
    beam_start = 0.024
    beam_end = l3_len - 0.040
    beam_center = (beam_start + beam_end) / 2.0
    _box_along(
        wrist_link,
        center=(ux3 * beam_center, 0.0, uz3 * beam_center),
        length=beam_end - beam_start,
        width_y=0.038,
        thickness_z=0.030,
        angle=angle3,
        material=anodized,
        name="short_web",
    )
    _box_along(
        wrist_link,
        center=(ux3 * (l3_len - 0.026), 0.0, uz3 * (l3_len - 0.026)),
        length=0.034,
        width_y=0.102,
        thickness_z=0.098,
        angle=angle3,
        material=steel,
        name="pad_backer",
    )
    _box_along(
        wrist_link,
        center=(ux3 * (l3_len - 0.006), 0.0, uz3 * (l3_len - 0.006)),
        length=0.026,
        width_y=0.118,
        thickness_z=0.118,
        angle=angle3,
        material=rubber,
        name="square_pad",
    )
    for sign in (1.0, -1.0):
        _box_along(
            wrist_link,
            center=(ux3 * 0.118, sign * 0.021, uz3 * 0.118),
            length=0.135,
            width_y=0.008,
            thickness_z=0.012,
            angle=angle3,
            material=dark,
            name=f"short_rib_{'pos' if sign > 0 else 'neg'}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.45, upper=0.70),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=l1_distal),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=36.0, velocity=1.3, lower=-0.62, upper=0.62),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_link,
        origin=Origin(xyz=l2_distal),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.58, upper=0.58),
    )
    model.meta["link_lengths_m"] = (round(l1_len, 3), round(l2_len, 3), round(l3_len, 3))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("wall_plate")
    shoulder_link = object_model.get_part("shoulder_link")
    elbow_link = object_model.get_part("elbow_link")
    wrist_link = object_model.get_part("wrist_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.allow_overlap(
        base,
        shoulder_link,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The steel shoulder pin is intentionally captured through the bearing hub.",
    )
    ctx.allow_overlap(
        shoulder_link,
        elbow_link,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The steel elbow pin is intentionally captured through the bearing hub.",
    )
    ctx.allow_overlap(
        elbow_link,
        wrist_link,
        elem_a="wrist_pin",
        elem_b="wrist_hub",
        reason="The steel wrist pin is intentionally captured through the bearing hub.",
    )

    ctx.expect_within(
        base,
        shoulder_link,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.0,
        name="shoulder pin is inside hub bore",
    )
    ctx.expect_overlap(
        base,
        shoulder_link,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.060,
        name="shoulder pin spans the hub",
    )
    ctx.expect_within(
        shoulder_link,
        elbow_link,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.0,
        name="elbow pin is inside hub bore",
    )
    ctx.expect_overlap(
        shoulder_link,
        elbow_link,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.055,
        name="elbow pin spans the hub",
    )
    ctx.expect_within(
        elbow_link,
        wrist_link,
        axes="xz",
        inner_elem="wrist_pin",
        outer_elem="wrist_hub",
        margin=0.0,
        name="wrist pin is inside hub bore",
    )
    ctx.expect_overlap(
        elbow_link,
        wrist_link,
        axes="y",
        elem_a="wrist_pin",
        elem_b="wrist_hub",
        min_overlap=0.050,
        name="wrist pin spans the hub",
    )

    for positive, negative, pos_elem, neg_elem, check_name in (
        (base, shoulder_link, "shoulder_cheek_pos", "shoulder_hub", "shoulder upper fork has side clearance"),
        (shoulder_link, elbow_link, "elbow_cheek_pos", "elbow_hub", "elbow upper fork has side clearance"),
        (elbow_link, wrist_link, "wrist_cheek_pos", "wrist_hub", "wrist upper fork has side clearance"),
    ):
        ctx.expect_gap(
            positive,
            negative,
            axis="y",
            positive_elem=pos_elem,
            negative_elem=neg_elem,
            min_gap=0.006,
            max_gap=0.025,
            name=check_name,
        )
    for positive, negative, pos_elem, neg_elem, check_name in (
        (shoulder_link, base, "shoulder_hub", "shoulder_cheek_neg", "shoulder lower fork has side clearance"),
        (elbow_link, shoulder_link, "elbow_hub", "elbow_cheek_neg", "elbow lower fork has side clearance"),
        (wrist_link, elbow_link, "wrist_hub", "wrist_cheek_neg", "wrist lower fork has side clearance"),
    ):
        ctx.expect_gap(
            positive,
            negative,
            axis="y",
            positive_elem=pos_elem,
            negative_elem=neg_elem,
            min_gap=0.006,
            max_gap=0.025,
            name=check_name,
        )

    lengths = object_model.meta["link_lengths_m"]
    ctx.check(
        "three link lengths are visibly varied",
        max(lengths) - min(lengths) > 0.18 and lengths[0] > lengths[1] > lengths[2],
        details=f"link_lengths_m={lengths}",
    )
    ctx.check(
        "hinge axes are parallel",
        tuple(shoulder.axis) == HINGE_AXIS and tuple(elbow.axis) == HINGE_AXIS and tuple(wrist.axis) == HINGE_AXIS,
        details=f"axes={shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    rest_pad = ctx.part_element_world_aabb(wrist_link, elem="square_pad")
    with ctx.pose({shoulder: 0.55, elbow: -0.35, wrist: 0.35}):
        posed_pad = ctx.part_element_world_aabb(wrist_link, elem="square_pad")
    ctx.check(
        "articulations move pad through the inspection plane",
        rest_pad is not None
        and posed_pad is not None
        and posed_pad[0][0] > -0.010
        and abs(posed_pad[0][2] - rest_pad[0][2]) > 0.050,
        details=f"rest_pad={rest_pad}, posed_pad={posed_pad}",
    )

    return ctx.report()


object_model = build_object_model()
