from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from math import isclose, pi

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


BRACKET_WIDTH = 0.016
CHEEK_WIDTH = 0.0035
JOINT_GAP = BRACKET_WIDTH - 2.0 * CHEEK_WIDTH
CENTER_BARREL_WIDTH = JOINT_GAP
CHEEK_Y = JOINT_GAP / 2.0 + CHEEK_WIDTH / 2.0
BARREL_RADIUS = 0.005
WEB_WIDTH = 0.0068

LINK_1 = 0.043
LINK_2 = 0.033
LINK_3 = 0.028

THICK_1 = 0.0085
THICK_2 = 0.0078
THICK_3 = 0.0070
TIP_PAD_LENGTH = 0.010


def add_box_visual(
    part,
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def add_y_cylinder_visual(
    part,
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def centered_box(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((x, y, z))


def y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, y, z))
    )


def z_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, y, z))
    )


def extruded_side_profile(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)


def fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def make_root_bracket() -> cq.Workplane:
    return fuse_all(
        [
            centered_box(0.018, BRACKET_WIDTH, 0.024, x=-0.014),
            centered_box(0.008, WEB_WIDTH, 0.012, x=-0.010),
            centered_box(0.008, CHEEK_WIDTH, 0.011, x=-0.003, y=CHEEK_Y),
            centered_box(0.008, CHEEK_WIDTH, 0.011, x=-0.003, y=-CHEEK_Y),
            y_cylinder(BARREL_RADIUS, CHEEK_WIDTH, x=0.0, y=CHEEK_Y),
            y_cylinder(BARREL_RADIUS, CHEEK_WIDTH, x=0.0, y=-CHEEK_Y),
        ]
    )


def make_intermediate_link(link_length: float, thickness: float) -> cq.Workplane:
    body_end = link_length - 0.015
    body = extruded_side_profile(
        [
            (0.002, -thickness / 2.0),
            (body_end * 0.35, -(thickness * 0.46)),
            (body_end, -(thickness * 0.38)),
            (body_end, thickness * 0.38),
            (body_end * 0.35, thickness * 0.46),
            (0.002, thickness / 2.0),
        ],
        WEB_WIDTH,
    )
    return fuse_all(
        [
            centered_box(0.006, JOINT_GAP, thickness * 0.84, x=0.001),
            body,
            y_cylinder(BARREL_RADIUS, CENTER_BARREL_WIDTH),
            centered_box(0.007, BRACKET_WIDTH, thickness * 0.56, x=link_length - 0.012),
            centered_box(0.011, CHEEK_WIDTH, thickness * 0.92, x=link_length - 0.005, y=CHEEK_Y),
            centered_box(0.011, CHEEK_WIDTH, thickness * 0.92, x=link_length - 0.005, y=-CHEEK_Y),
            y_cylinder(BARREL_RADIUS, CHEEK_WIDTH, x=link_length, y=CHEEK_Y),
            y_cylinder(BARREL_RADIUS, CHEEK_WIDTH, x=link_length, y=-CHEEK_Y),
        ]
    )


def make_distal_link() -> tuple[cq.Workplane, cq.Workplane]:
    body_end = LINK_3 - 0.006
    metal = fuse_all(
        [
            centered_box(0.006, JOINT_GAP, THICK_3 * 0.84, x=0.001),
            extruded_side_profile(
                [
                    (0.002, -THICK_3 / 2.0),
                    (body_end * 0.42, -(THICK_3 * 0.45)),
                    (body_end, -(THICK_3 * 0.34)),
                    (body_end, THICK_3 * 0.34),
                    (body_end * 0.42, THICK_3 * 0.45),
                    (0.002, THICK_3 / 2.0),
                ],
                WEB_WIDTH,
            ),
            y_cylinder(BARREL_RADIUS, CENTER_BARREL_WIDTH),
            centered_box(0.012, 0.0105, THICK_3 * 0.60, x=LINK_3 - 0.003),
        ]
    )
    tip_pad = centered_box(
        TIP_PAD_LENGTH,
        0.0115,
        0.0045,
        x=LINK_3 + TIP_PAD_LENGTH / 2.0 - 0.003,
        z=-0.0014,
    )
    return metal, tip_pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_finger_module")

    bracket_metal = model.material("bracket_metal", rgba=(0.20, 0.20, 0.22, 1.0))
    link_metal = model.material("link_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    tip_rubber = model.material("tip_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    root_bracket = model.part("root_bracket")
    add_box_visual(
        root_bracket,
        (0.022, BRACKET_WIDTH, 0.024),
        xyz=(-0.016, 0.0, 0.0),
        material=bracket_metal,
        name="bracket_body",
    )
    add_box_visual(
        root_bracket,
        (0.006, WEB_WIDTH, 0.012),
        xyz=(-0.008, 0.0, 0.0),
        material=bracket_metal,
        name="bracket_neck",
    )
    add_box_visual(
        root_bracket,
        (0.008, CHEEK_WIDTH, 0.011),
        xyz=(-0.003, CHEEK_Y, 0.0),
        material=bracket_metal,
        name="root_cheek_upper",
    )
    add_box_visual(
        root_bracket,
        (0.008, CHEEK_WIDTH, 0.011),
        xyz=(-0.003, -CHEEK_Y, 0.0),
        material=bracket_metal,
        name="root_cheek_lower",
    )
    add_y_cylinder_visual(
        root_bracket,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(0.0, CHEEK_Y, 0.0),
        material=bracket_metal,
        name="root_barrel_upper",
    )
    add_y_cylinder_visual(
        root_bracket,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(0.0, -CHEEK_Y, 0.0),
        material=bracket_metal,
        name="root_barrel_lower",
    )

    proximal_link = model.part("proximal_link")
    add_y_cylinder_visual(
        proximal_link,
        BARREL_RADIUS,
        JOINT_GAP,
        xyz=(0.0, 0.0, 0.0),
        material=link_metal,
        name="proximal_root_barrel",
    )
    add_box_visual(
        proximal_link,
        (0.006, JOINT_GAP, THICK_1 * 0.80),
        xyz=(0.002, 0.0, 0.0),
        material=link_metal,
        name="proximal_root_web",
    )
    add_box_visual(
        proximal_link,
        (0.028, WEB_WIDTH, THICK_1),
        xyz=(0.019, 0.0, 0.0),
        material=link_metal,
        name="proximal_body",
    )
    add_box_visual(
        proximal_link,
        (0.006, JOINT_GAP, THICK_1 * 0.72),
        xyz=(LINK_1 - 0.008, 0.0, 0.0),
        material=link_metal,
        name="proximal_tip_bridge",
    )
    add_box_visual(
        proximal_link,
        (0.008, CHEEK_WIDTH, THICK_1 * 0.92),
        xyz=(LINK_1 - 0.004, CHEEK_Y, 0.0),
        material=link_metal,
        name="proximal_cheek_upper",
    )
    add_box_visual(
        proximal_link,
        (0.008, CHEEK_WIDTH, THICK_1 * 0.92),
        xyz=(LINK_1 - 0.004, -CHEEK_Y, 0.0),
        material=link_metal,
        name="proximal_cheek_lower",
    )
    add_y_cylinder_visual(
        proximal_link,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(LINK_1, CHEEK_Y, 0.0),
        material=link_metal,
        name="proximal_barrel_upper",
    )
    add_y_cylinder_visual(
        proximal_link,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(LINK_1, -CHEEK_Y, 0.0),
        material=link_metal,
        name="proximal_barrel_lower",
    )

    middle_link = model.part("middle_link")
    add_y_cylinder_visual(
        middle_link,
        BARREL_RADIUS,
        JOINT_GAP,
        xyz=(0.0, 0.0, 0.0),
        material=link_metal,
        name="middle_root_barrel",
    )
    add_box_visual(
        middle_link,
        (0.006, JOINT_GAP, THICK_2 * 0.80),
        xyz=(0.002, 0.0, 0.0),
        material=link_metal,
        name="middle_root_web",
    )
    add_box_visual(
        middle_link,
        (0.018, WEB_WIDTH, THICK_2),
        xyz=(0.014, 0.0, 0.0),
        material=link_metal,
        name="middle_body",
    )
    add_box_visual(
        middle_link,
        (0.006, JOINT_GAP, THICK_2 * 0.72),
        xyz=(LINK_2 - 0.008, 0.0, 0.0),
        material=link_metal,
        name="middle_tip_bridge",
    )
    add_box_visual(
        middle_link,
        (0.008, CHEEK_WIDTH, THICK_2 * 0.92),
        xyz=(LINK_2 - 0.004, CHEEK_Y, 0.0),
        material=link_metal,
        name="middle_cheek_upper",
    )
    add_box_visual(
        middle_link,
        (0.008, CHEEK_WIDTH, THICK_2 * 0.92),
        xyz=(LINK_2 - 0.004, -CHEEK_Y, 0.0),
        material=link_metal,
        name="middle_cheek_lower",
    )
    add_y_cylinder_visual(
        middle_link,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(LINK_2, CHEEK_Y, 0.0),
        material=link_metal,
        name="middle_barrel_upper",
    )
    add_y_cylinder_visual(
        middle_link,
        BARREL_RADIUS,
        CHEEK_WIDTH,
        xyz=(LINK_2, -CHEEK_Y, 0.0),
        material=link_metal,
        name="middle_barrel_lower",
    )

    distal_link = model.part("distal_link")
    add_y_cylinder_visual(
        distal_link,
        BARREL_RADIUS,
        JOINT_GAP,
        xyz=(0.0, 0.0, 0.0),
        material=link_metal,
        name="distal_root_barrel",
    )
    add_box_visual(
        distal_link,
        (0.006, JOINT_GAP, THICK_3 * 0.80),
        xyz=(0.002, 0.0, 0.0),
        material=link_metal,
        name="distal_root_web",
    )
    add_box_visual(
        distal_link,
        (0.018, WEB_WIDTH, THICK_3),
        xyz=(0.014, 0.0, 0.0),
        material=link_metal,
        name="distal_body",
    )
    add_box_visual(
        distal_link,
        (0.006, 0.009, THICK_3 * 0.64),
        xyz=(LINK_3 - 0.005, 0.0, 0.0),
        material=link_metal,
        name="distal_tip_mount",
    )
    add_box_visual(
        distal_link,
        (TIP_PAD_LENGTH, 0.0115, 0.0045),
        xyz=(LINK_3 + TIP_PAD_LENGTH / 2.0 - 0.003, 0.0, -0.0014),
        material=tip_rubber,
        name="tip_pad",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=middle_link,
        origin=Origin(xyz=(LINK_1, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(LINK_2, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    proximal_link = object_model.get_part("proximal_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")
    tip_pad = distal_link.get_visual("tip_pad")

    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    joints_are_revolute = all(
        joint.articulation_type == ArticulationType.REVOLUTE
        for joint in (root_to_proximal, proximal_to_middle, middle_to_distal)
    )
    axes_are_parallel = all(
        joint.axis == (0.0, 1.0, 0.0)
        for joint in (root_to_proximal, proximal_to_middle, middle_to_distal)
    )
    ctx.check(
        "serial_revolute_chain_configured",
        joints_are_revolute and axes_are_parallel,
        "Expected three serial revolute joints sharing the local +Y hinge axis.",
    )
    ctx.check(
        "joint_center_offsets_match_link_lengths",
        isclose(proximal_to_middle.origin.xyz[0], LINK_1, abs_tol=1e-6)
        and isclose(middle_to_distal.origin.xyz[0], LINK_2, abs_tol=1e-6),
        "Successive hinge centers should sit at the distal knuckles of the preceding phalanx.",
    )
    ctx.check(
        "flat_tip_visual_present",
        tip_pad.name == "tip_pad",
        "Distal link should carry a distinct flat fingertip pad visual.",
    )

    with ctx.pose({root_to_proximal: 0.0, proximal_to_middle: 0.0, middle_to_distal: 0.0}):
        ctx.expect_contact(
            root_bracket,
            proximal_link,
            name="root_bracket_supports_proximal_link_at_rest",
        )
        ctx.expect_contact(
            proximal_link,
            middle_link,
            name="proximal_link_supports_middle_link_at_rest",
        )
        ctx.expect_contact(
            middle_link,
            distal_link,
            name="middle_link_supports_distal_link_at_rest",
        )
        ctx.expect_origin_gap(
            middle_link,
            proximal_link,
            axis="x",
            min_gap=LINK_1 - 1e-4,
            max_gap=LINK_1 + 1e-4,
            name="rest_pose_proximal_to_middle_spacing",
        )
        ctx.expect_origin_gap(
            distal_link,
            middle_link,
            axis="x",
            min_gap=LINK_2 - 1e-4,
            max_gap=LINK_2 + 1e-4,
            name="rest_pose_middle_to_distal_spacing",
        )
        ctx.expect_origin_gap(
            distal_link,
            root_bracket,
            axis="z",
            min_gap=-1e-4,
            max_gap=1e-4,
            name="rest_pose_stays_in_single_bending_plane",
        )

    with ctx.pose({root_to_proximal: 0.55, proximal_to_middle: 0.70, middle_to_distal: 0.55}):
        ctx.expect_origin_gap(
            root_bracket,
            distal_link,
            axis="z",
            min_gap=0.018,
            name="curl_pose_drops_distal_link_below_root",
        )
        ctx.expect_origin_gap(
            distal_link,
            root_bracket,
            axis="x",
            min_gap=0.020,
            name="curl_pose_keeps_chain_reaching_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
