from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(profile, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(radius, cx=0.0, cy=0.0, segments=36):
    return [
        (
            cx + radius * cos(2.0 * pi * index / segments),
            cy + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _washer_mesh(name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0125, segments=48),
            [_circle_profile(0.0062, segments=32)],
            0.0016,
            center=True,
        ),
        name,
    )


def _add_flat_link(
    part,
    *,
    mesh_prefix: str,
    length: float,
    direction: float,
    width: float,
    plate_thickness: float,
    plate_z: float,
    metal,
    rib_metal,
):
    """Add a stamped flat bar link with real pivot holes and formed ribs."""
    center_x = direction * length * 0.5
    distal_x = direction * length
    outer = _translate_profile(
        rounded_rect_profile(length + width, width, width * 0.5, corner_segments=14),
        dx=center_x,
    )
    holes = [
        _circle_profile(0.0072, 0.0, 0.0, segments=36),
        _circle_profile(0.0072, distal_x, 0.0, segments=36),
    ]
    plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, plate_thickness, center=True),
        f"{mesh_prefix}_plate",
    )
    part.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_z)),
        material=metal,
        name="link_plate",
    )

    rib_profile = _translate_profile(
        rounded_rect_profile(length * 0.52, width * 0.20, width * 0.08, corner_segments=8),
        dx=center_x,
    )
    rib_mesh = mesh_from_geometry(
        ExtrudeGeometry(rib_profile, 0.0022, center=True),
        f"{mesh_prefix}_pressed_rib",
    )
    part.visual(
        rib_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_z + plate_thickness * 0.5 + 0.0009)),
        material=rib_metal,
        name="pressed_rib",
    )

    flange_profile = rounded_rect_profile(length * 0.58, width * 0.09, width * 0.035, corner_segments=5)
    for side, y_offset in (("upper", width * 0.39), ("lower", -width * 0.39)):
        flange_mesh = mesh_from_geometry(
            ExtrudeGeometry(_translate_profile(flange_profile, dx=center_x, dy=y_offset), 0.0020, center=True),
            f"{mesh_prefix}_{side}_formed_edge",
        )
        part.visual(
            flange_mesh,
            origin=Origin(xyz=(0.0, 0.0, plate_z + plate_thickness * 0.5 + 0.0007)),
            material=rib_metal,
            name=f"{side}_edge",
        )

    washer_mesh = _washer_mesh(f"{mesh_prefix}_washer")
    washer_z = plate_z + plate_thickness * 0.5 + 0.0006
    for name, x in (("prox_washer", 0.0), ("dist_washer", distal_x)):
        part.visual(
            washer_mesh,
            origin=Origin(xyz=(x, 0.0, washer_z)),
            material=rib_metal,
            name=name,
        )


def _add_pin_stack(
    part,
    *,
    x: float,
    shaft_z: float,
    shaft_length: float,
    cap_z: float,
    metal,
    name: str,
):
    part.visual(
        Cylinder(radius=0.0044, length=shaft_length),
        origin=Origin(xyz=(x, 0.0, shaft_z)),
        material=metal,
        name=f"{name}_shaft",
    )
    part.visual(
        Cylinder(radius=0.0090, length=0.0024),
        origin=Origin(xyz=(x, 0.0, cap_z)),
        material=metal,
        name=f"{name}_head",
    )


def _add_triangular_shoe(part, *, metal, rubber):
    shoe_outline = [
        (-0.018, -0.027),
        (0.086, -0.052),
        (0.112, 0.000),
        (0.086, 0.052),
        (-0.018, 0.027),
    ]
    shoe_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            shoe_outline,
            [_circle_profile(0.0073, 0.0, 0.0, segments=36)],
            0.0050,
            center=True,
        ),
        "shoe_triangular_plate",
    )
    part.visual(
        shoe_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0290)),
        material=metal,
        name="shoe_plate",
    )
    pad_outline = [
        (0.024, -0.041),
        (0.084, -0.050),
        (0.106, 0.000),
        (0.084, 0.050),
        (0.024, 0.041),
    ]
    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(pad_outline, 0.0045, center=True),
        "shoe_rubber_pad",
    )
    part.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0244)),
        material=rubber,
        name="rubber_pad",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=metal,
        name="shoe_shoulder_pin",
    )
    part.visual(
        Cylinder(radius=0.0105, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0324)),
        material=metal,
        name="end_washer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_stay")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    burnished = model.material("burnished_edges", rgba=(0.78, 0.77, 0.70, 1.0))
    dark_fastener = model.material("dark_fasteners", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.032, 1.0))

    link_width = 0.032
    plate_t = 0.0040
    link_a_len = 0.280
    link_b_len = 0.210
    link_c_len = 0.140

    root = model.part("root_bracket")
    root.visual(
        Box((0.130, 0.082, 0.008)),
        origin=Origin(xyz=(-0.018, 0.0, -0.026)),
        material=galvanized,
        name="mounting_plate",
    )
    root.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=galvanized,
        name="raised_boss",
    )
    root.visual(
        Box((0.046, 0.006, 0.024)),
        origin=Origin(xyz=(0.000, 0.027, -0.013)),
        material=galvanized,
        name="side_gusset_0",
    )
    root.visual(
        Box((0.046, 0.006, 0.024)),
        origin=Origin(xyz=(0.000, -0.027, -0.013)),
        material=galvanized,
        name="side_gusset_1",
    )
    root.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark_fastener,
        name="root_shoulder_pin",
    )
    root.visual(
        Cylinder(radius=0.010, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=dark_fastener,
        name="root_bolt_head",
    )
    for index, (x, y) in enumerate(((-0.050, -0.024), (-0.050, 0.024), (0.030, -0.024), (0.030, 0.024))):
        root.visual(
            Cylinder(radius=0.0055, length=0.0018),
            origin=Origin(xyz=(x, y, -0.0214)),
            material=dark_fastener,
            name=f"mount_screw_{index}",
        )

    link_a = model.part("long_link")
    _add_flat_link(
        link_a,
        mesh_prefix="long_link",
        length=link_a_len,
        direction=1.0,
        width=link_width,
        plate_thickness=plate_t,
        plate_z=0.000,
        metal=galvanized,
        rib_metal=burnished,
    )
    _add_pin_stack(
        link_a,
        x=link_a_len,
        shaft_z=0.0060,
        shaft_length=0.0140,
        cap_z=0.0138,
        metal=dark_fastener,
        name="middle_bolt",
    )

    link_b = model.part("middle_link")
    _add_flat_link(
        link_b,
        mesh_prefix="middle_link",
        length=link_b_len,
        direction=-1.0,
        width=link_width * 0.92,
        plate_thickness=plate_t,
        plate_z=0.0090,
        metal=galvanized,
        rib_metal=burnished,
    )
    _add_pin_stack(
        link_b,
        x=-link_b_len,
        shaft_z=0.0145,
        shaft_length=0.0140,
        cap_z=0.02265,
        metal=dark_fastener,
        name="outer_bolt",
    )

    link_c = model.part("short_link")
    _add_flat_link(
        link_c,
        mesh_prefix="short_link",
        length=link_c_len,
        direction=1.0,
        width=link_width * 0.84,
        plate_thickness=plate_t,
        plate_z=0.0180,
        metal=galvanized,
        rib_metal=burnished,
    )

    shoe = model.part("end_shoe")
    _add_triangular_shoe(shoe, metal=galvanized, rubber=rubber)

    model.articulation(
        "root_to_long_link",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_a,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "long_to_middle_link",
        ArticulationType.REVOLUTE,
        parent=link_a,
        child=link_b,
        origin=Origin(xyz=(link_a_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=38.0, velocity=2.0, lower=0.0, upper=pi),
    )
    model.articulation(
        "middle_to_short_link",
        ArticulationType.REVOLUTE,
        parent=link_b,
        child=link_c,
        origin=Origin(xyz=(-link_b_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=pi),
    )
    model.articulation(
        "short_to_end_shoe",
        ArticulationType.FIXED,
        parent=link_c,
        child=shoe,
        origin=Origin(xyz=(link_c_len, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    long_link = object_model.get_part("long_link")
    middle_link = object_model.get_part("middle_link")
    short_link = object_model.get_part("short_link")
    shoe = object_model.get_part("end_shoe")
    root_joint = object_model.get_articulation("root_to_long_link")
    middle_joint = object_model.get_articulation("long_to_middle_link")
    outer_joint = object_model.get_articulation("middle_to_short_link")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three planar revolute link joints",
        len(revolute_joints) == 3 and all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in revolute_joints),
        details=f"revolutes={[(joint.name, joint.axis) for joint in revolute_joints]}",
    )

    ctx.expect_contact(
        root,
        long_link,
        elem_a="root_shoulder_pin",
        elem_b="link_plate",
        name="root shoulder bolt sits in the long link eye",
    )
    ctx.expect_overlap(
        long_link,
        middle_link,
        axes="xy",
        min_overlap=0.020,
        elem_a="link_plate",
        elem_b="link_plate",
        name="folded long and middle links overlap in plan",
    )
    ctx.expect_overlap(
        middle_link,
        short_link,
        axes="xy",
        min_overlap=0.018,
        elem_a="link_plate",
        elem_b="link_plate",
        name="folded middle and short links overlap in plan",
    )
    ctx.expect_gap(
        middle_link,
        long_link,
        axis="z",
        min_gap=0.003,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="middle link is shimmed above long link",
    )
    ctx.expect_gap(
        short_link,
        middle_link,
        axis="z",
        min_gap=0.003,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="short link is shimmed above middle link",
    )

    folded_pos = ctx.part_world_position(shoe)
    with ctx.pose({root_joint: 0.25, middle_joint: pi, outer_joint: pi}):
        extended_pos = ctx.part_world_position(shoe)
        ctx.expect_gap(
            middle_link,
            long_link,
            axis="z",
            min_gap=0.003,
            positive_elem="link_plate",
            negative_elem="link_plate",
            name="extended middle link clears long link",
        )
        ctx.expect_gap(
            short_link,
            middle_link,
            axis="z",
            min_gap=0.003,
            positive_elem="link_plate",
            negative_elem="link_plate",
            name="extended short link clears middle link",
        )
    ctx.check(
        "end shoe extends outward from folded stack",
        folded_pos is not None and extended_pos is not None and extended_pos[0] > folded_pos[0] + 0.34,
        details=f"folded={folded_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
