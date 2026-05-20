from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.160
FOOT_WIDTH = 0.090
FOOT_THICKNESS = 0.016

PEDESTAL_LENGTH = 0.050
PEDESTAL_WIDTH = 0.032
PEDESTAL_HEIGHT = 0.018

LINK_THICKNESS = 0.012
STRAP_WIDTH = 0.010
PLATE_THICKNESS = 0.004
CLEVIS_OUTER_WIDTH = LINK_THICKNESS + 2.0 * PLATE_THICKNESS
PLATE_OFFSET_Y = LINK_THICKNESS * 0.5 + PLATE_THICKNESS * 0.5

PIN_RADIUS = 0.0035
EYE_OUTER_RADIUS = 0.008
HINGE_PLATE_X = 0.016
HINGE_BRIDGE_X = 0.020
HINGE_BRIDGE_Z = 0.008
TONGUE_LENGTH = 0.016
CLEVIS_DEPTH = 0.020
PLATE_TOP = 0.008

LINK_BODY_X = 0.014

BASE_JOINT_Z = 0.056
LINK_1_LENGTH = 0.140
LINK_2_LENGTH = 0.120
LINK_3_LENGTH = 0.110

PROX_TONGUE_HEIGHT = 0.018
STRAP_OVERLAP = 0.003
CLEVIS_LOWER = 0.018
CLEVIS_UPPER = 0.010
HINGE_BLOCK_HEIGHT = 0.012
HINGE_BLOCK_TOP_BELOW_AXIS = 0.006


def _add_box_visual(part, size: tuple[float, float, float], center: tuple[float, float, float], *, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y + length * 0.5, z))


def _joint_pin(axis_z: float) -> cq.Workplane:
    return _cylinder_y(PIN_RADIUS, CLEVIS_OUTER_WIDTH, (0.0, 0.0, axis_z))


def _eye_at(axis_z: float) -> cq.Workplane:
    return _cylinder_y(EYE_OUTER_RADIUS, LINK_THICKNESS, (0.0, 0.0, axis_z)).cut(
        _cylinder_y(PIN_RADIUS, LINK_THICKNESS + 0.002, (0.0, 0.0, axis_z))
    )


def _side_plate(axis_z: float, y_center: float) -> cq.Workplane:
    return _box(
        (HINGE_PLATE_X, PLATE_THICKNESS, CLEVIS_DEPTH + PLATE_TOP),
        (0.0, y_center, axis_z - CLEVIS_DEPTH * 0.5 + PLATE_TOP * 0.5),
    )


def _clevis_at(axis_z: float) -> cq.Workplane:
    bridge = _box(
        (HINGE_BRIDGE_X, CLEVIS_OUTER_WIDTH, HINGE_BRIDGE_Z),
        (0.0, 0.0, axis_z - CLEVIS_DEPTH + HINGE_BRIDGE_Z * 0.5),
    )
    left_plate = _side_plate(axis_z, -PLATE_OFFSET_Y)
    right_plate = _side_plate(axis_z, PLATE_OFFSET_Y)
    return bridge.union(left_plate).union(right_plate).union(_joint_pin(axis_z))


def _base_shape() -> cq.Workplane:
    foot = (
        _box(
            (FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS),
            (0.0, 0.0, FOOT_THICKNESS * 0.5),
        )
        .edges("|Z")
        .fillet(0.004)
    )
    pedestal = (
        _box(
            (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
            (0.0, 0.0, FOOT_THICKNESS + PEDESTAL_HEIGHT * 0.5),
        )
        .edges("|Z")
        .fillet(0.003)
    )
    support_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.024, FOOT_THICKNESS),
                (0.024, FOOT_THICKNESS),
                (0.012, BASE_JOINT_Z - CLEVIS_DEPTH + 0.004),
                (-0.012, BASE_JOINT_Z - CLEVIS_DEPTH + 0.004),
            ]
        )
        .close()
        .extrude(STRAP_WIDTH)
        .translate((0.0, -STRAP_WIDTH * 0.5, 0.0))
    )
    return foot.union(pedestal).union(support_web).union(_clevis_at(BASE_JOINT_Z))


def _link_shape(length: float, *, distal_clevis: bool, end_tab: bool) -> cq.Workplane:
    body_top = length - CLEVIS_DEPTH + 0.002 if distal_clevis else length
    body_bottom = EYE_OUTER_RADIUS
    body_height = max(body_top - body_bottom, 0.030)
    body = _box((LINK_BODY_X, STRAP_WIDTH, body_height), (0.0, 0.0, body_bottom + body_height * 0.5))
    shape = body.union(_eye_at(0.0))

    if distal_clevis:
        shape = shape.union(_clevis_at(length))
    else:
        top_cap = _box((0.012, STRAP_WIDTH, 0.010), (0.0, 0.0, length - 0.005))
        shape = shape.union(top_cap)

    if end_tab:
        tab = _box((0.022, STRAP_WIDTH, 0.006), (0.007, 0.0, length + 0.003))
        shape = shape.union(tab)

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_lever_chain")

    base_color = model.material("base_coat", rgba=(0.18, 0.18, 0.20, 1.0))
    link_color = model.material("link_steel", rgba=(0.66, 0.68, 0.72, 1.0))

    base = model.part("base")
    _add_box_visual(
        base,
        (FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS),
        (0.0, 0.0, FOOT_THICKNESS * 0.5),
        material=base_color,
        name="base_shell",
    )
    _add_box_visual(
        base,
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (0.0, 0.0, FOOT_THICKNESS + PEDESTAL_HEIGHT * 0.5),
        material=base_color,
        name="pedestal",
    )
    base_plate_center_z = BASE_JOINT_Z + 0.5 * (CLEVIS_UPPER - CLEVIS_LOWER)
    base_plate_height = CLEVIS_LOWER + CLEVIS_UPPER
    base_column_bottom = FOOT_THICKNESS + PEDESTAL_HEIGHT
    base_column_top = BASE_JOINT_Z - CLEVIS_LOWER + 0.004
    base_column_height = base_column_top - base_column_bottom
    base_column_center_z = 0.5 * (base_column_bottom + base_column_top)
    for side, y_center in (("left", -PLATE_OFFSET_Y), ("right", PLATE_OFFSET_Y)):
        _add_box_visual(
            base,
            (HINGE_PLATE_X, PLATE_THICKNESS, base_column_height),
            (0.0, y_center, base_column_center_z),
            material=base_color,
            name=f"{side}_column",
        )
        _add_box_visual(
            base,
            (HINGE_PLATE_X, PLATE_THICKNESS, base_plate_height),
            (0.0, y_center, base_plate_center_z),
            material=base_color,
            name=f"{side}_joint_plate",
        )
    hinge_block_bottom = BASE_JOINT_Z - CLEVIS_LOWER
    hinge_block_center_z = hinge_block_bottom + HINGE_BLOCK_HEIGHT * 0.5
    _add_box_visual(
        base,
        (HINGE_BRIDGE_X, LINK_THICKNESS, HINGE_BLOCK_HEIGHT),
        (0.0, 0.0, hinge_block_center_z),
        material=base_color,
        name="joint_block",
    )

    def add_link_geometry(part, length: float, *, distal_clevis: bool, end_tab: bool) -> None:
        body_top = length - 0.014 if distal_clevis else length
        body_bottom = PROX_TONGUE_HEIGHT - STRAP_OVERLAP
        body_height = body_top - body_bottom

        _add_box_visual(
            part,
            (LINK_BODY_X, LINK_THICKNESS, PROX_TONGUE_HEIGHT),
            (0.0, 0.0, PROX_TONGUE_HEIGHT * 0.5),
            material=link_color,
            name="proximal_tongue",
        )
        _add_box_visual(
            part,
            (LINK_BODY_X, LINK_THICKNESS, body_height),
            (0.0, 0.0, body_bottom + body_height * 0.5),
            material=link_color,
            name="link_spine",
        )

        if distal_clevis:
            plate_height = CLEVIS_LOWER + CLEVIS_UPPER
            plate_center_z = length + 0.5 * (CLEVIS_UPPER - CLEVIS_LOWER)
            for side, y_center in (("left", -PLATE_OFFSET_Y), ("right", PLATE_OFFSET_Y)):
                _add_box_visual(
                    part,
                    (HINGE_PLATE_X, PLATE_THICKNESS, plate_height),
                    (0.0, y_center, plate_center_z),
                    material=link_color,
                    name=f"{side}_joint_plate",
                )
            link_block_center_z = (length - CLEVIS_LOWER) + HINGE_BLOCK_HEIGHT * 0.5
            _add_box_visual(
                part,
                (HINGE_BRIDGE_X, STRAP_WIDTH, HINGE_BLOCK_HEIGHT),
                (0.0, 0.0, link_block_center_z),
                material=link_color,
                name="joint_block",
            )
        else:
            _add_box_visual(
                part,
                (0.012, LINK_THICKNESS, 0.010),
                (0.0, 0.0, length - 0.005),
                material=link_color,
                name="tip_cap",
            )

        if end_tab:
            _add_box_visual(
                part,
                (0.022, LINK_THICKNESS, 0.006),
                (0.007, 0.0, length + 0.003),
                material=link_color,
                name="end_tab",
            )

    link1 = model.part("link_1")
    add_link_geometry(link1, LINK_1_LENGTH, distal_clevis=True, end_tab=False)

    link2 = model.part("link_2")
    add_link_geometry(link2, LINK_2_LENGTH, distal_clevis=True, end_tab=False)

    link3 = model.part("link_3")
    add_link_geometry(link3, LINK_3_LENGTH, distal_clevis=False, end_tab=True)

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-0.80, upper=1.20),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.0, 0.0, LINK_1_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.6, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(0.0, 0.0, LINK_2_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.8, lower=-1.30, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    link3 = object_model.get_part("link_3")

    joint1 = object_model.get_articulation("base_to_link_1")
    joint2 = object_model.get_articulation("link_1_to_link_2")
    joint3 = object_model.get_articulation("link_2_to_link_3")

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

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base, link1, link2, link3)),
        "One or more required chain parts are missing.",
    )
    ctx.check(
        "all_joint_axes_share_motion_plane",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0) for joint in (joint1, joint2, joint3)),
        (
            "Expected all revolute axes to align with +Y for a shared XZ motion plane; "
            f"got {[joint.axis for joint in (joint1, joint2, joint3)]}."
        ),
    )

    ctx.expect_contact(base, link1, contact_tol=5e-4, name="base_to_link_1_supported")
    ctx.expect_contact(link1, link2, contact_tol=5e-4, name="link_1_to_link_2_supported")
    ctx.expect_contact(link2, link3, contact_tol=5e-4, name="link_2_to_link_3_supported")

    ctx.expect_origin_gap(link1, base, axis="z", min_gap=0.045, max_gap=0.057, name="first_joint_stands_above_foot")

    with ctx.pose({joint1: 0.55, joint2: 0.35, joint3: 0.15}):
        link3_pos = ctx.part_world_position(link3)
        ctx.check(
            "distal_link_swings_forward_in_plane",
            link3_pos is not None
            and link3_pos[0] > 0.12
            and abs(link3_pos[1]) < 1e-5
            and link3_pos[2] > 0.22,
            f"Unexpected distal-link posed position: {link3_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
