from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.0045
PIN_CLEARANCE = 0.001
PIN_HOLE_RADIUS = PIN_RADIUS + PIN_CLEARANCE
UPPER_LUG_THICKNESS = 0.010
CHEEK_THICKNESS = 0.005
CLEVIS_GAP = UPPER_LUG_THICKNESS
CLEVIS_OUTER_WIDTH = CLEVIS_GAP + 2.0 * CHEEK_THICKNESS
CHEEK_CENTER_Y = CLEVIS_GAP / 2.0 + CHEEK_THICKNESS / 2.0
COLLAR_THICKNESS = 0.0025
COLLAR_RADIUS = 0.0075
JOINT_BARREL_RADIUS = 0.012
LINK_LUG_WIDTH = 0.022
LINK_WEB_WIDTH = 0.014
LINK_SPAN = 0.165
LOWER_CLEVIS_HEIGHT = 0.028
UPPER_LUG_HEIGHT = 0.028
WEB_FUSE = 0.002


def _y_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate((x, y, z))


def _box(size: tuple[float, float, float], *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").box(*size).translate((x, y, z))


def _hinge_pin(*, z: float, span_y: float):
    collar_center_y = span_y / 2.0 + COLLAR_THICKNESS / 2.0
    pin = _y_cylinder(PIN_RADIUS, span_y, z=z)
    pin = pin.union(_y_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, y=collar_center_y, z=z))
    pin = pin.union(_y_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, y=-collar_center_y, z=z))
    return pin


def _upper_lug():
    lug_plate = _box(
        (LINK_LUG_WIDTH, UPPER_LUG_THICKNESS, UPPER_LUG_HEIGHT),
        z=-UPPER_LUG_HEIGHT / 2.0,
    )
    lug_barrel = _y_cylinder(JOINT_BARREL_RADIUS, UPPER_LUG_THICKNESS, z=0.0)
    lug = lug_plate.union(lug_barrel)
    return lug.cut(_y_cylinder(PIN_HOLE_RADIUS, UPPER_LUG_THICKNESS + 0.004, z=0.0))


def _lower_clevis(*, z: float):
    cheek_plate = (0.018, CHEEK_THICKNESS, LOWER_CLEVIS_HEIGHT)
    left_cheek = _box(cheek_plate, y=CHEEK_CENTER_Y, z=z + LOWER_CLEVIS_HEIGHT / 2.0)
    right_cheek = _box(cheek_plate, y=-CHEEK_CENTER_Y, z=z + LOWER_CLEVIS_HEIGHT / 2.0)
    left_barrel = _y_cylinder(JOINT_BARREL_RADIUS, CHEEK_THICKNESS, y=CHEEK_CENTER_Y, z=z)
    right_barrel = _y_cylinder(JOINT_BARREL_RADIUS, CHEEK_THICKNESS, y=-CHEEK_CENTER_Y, z=z)
    clevis = left_cheek.union(right_cheek).union(left_barrel).union(right_barrel)
    return clevis.union(_hinge_pin(z=z, span_y=CLEVIS_OUTER_WIDTH))


def _support_bracket_shape():
    plate_size = (0.090, 0.042, 0.008)
    plate_z = 0.040
    rib_size = (0.020, CLEVIS_GAP, 0.020)

    plate = _box(plate_size, z=plate_z)
    left_cheek = _box((0.022, CHEEK_THICKNESS, 0.036), y=CHEEK_CENTER_Y, z=0.018)
    right_cheek = _box((0.022, CHEEK_THICKNESS, 0.036), y=-CHEEK_CENTER_Y, z=0.018)
    rib = _box(rib_size, z=0.026)

    bracket = plate.union(left_cheek).union(right_cheek).union(rib)
    bracket = bracket.union(_hinge_pin(z=0.0, span_y=CLEVIS_OUTER_WIDTH))
    bracket = bracket.union(_y_cylinder(JOINT_BARREL_RADIUS, CHEEK_THICKNESS, y=CHEEK_CENTER_Y, z=0.0))
    bracket = bracket.union(_y_cylinder(JOINT_BARREL_RADIUS, CHEEK_THICKNESS, y=-CHEEK_CENTER_Y, z=0.0))

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.024, 0.0), (0.024, 0.0)])
        .circle(0.004)
        .extrude(plate_size[2] + 0.004)
        .translate((0.0, 0.0, plate_z - plate_size[2] / 2.0 - 0.002))
    )
    return bracket.cut(mount_holes)


def _link_shape(span: float):
    web_length = span - UPPER_LUG_HEIGHT - LOWER_CLEVIS_HEIGHT + 2.0 * WEB_FUSE
    web_center_z = -(UPPER_LUG_HEIGHT + web_length / 2.0 - WEB_FUSE)
    web = _box((LINK_WEB_WIDTH, UPPER_LUG_THICKNESS, web_length), z=web_center_z)
    return _upper_lug().union(web).union(_lower_clevis(z=-span))


def _end_link_shape():
    bridge_len = 0.105
    neck_len = 0.030
    neck_width = 0.016
    pad_size = (0.050, 0.030, 0.010)
    pad_center_z = -(bridge_len + neck_len + pad_size[2] / 2.0)

    bridge = _box((LINK_WEB_WIDTH, UPPER_LUG_THICKNESS, bridge_len), z=-(bridge_len / 2.0 + 0.010))
    neck = _box((neck_width, 0.020, neck_len), z=-(bridge_len + neck_len / 2.0 + 0.010))
    pad = _box(pad_size, z=pad_center_z - 0.010)

    return _upper_lug().union(bridge).union(neck).union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_folding_arm_chain")

    bracket_finish = model.material("bracket_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    link_finish = model.material("link_finish", rgba=(0.73, 0.75, 0.78, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(_support_bracket_shape(), "support_bracket"),
        material=bracket_finish,
        name="support_bracket_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_shape(LINK_SPAN), "link_1"),
        material=link_finish,
        name="link_1_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_link_shape(LINK_SPAN), "link_2"),
        material=link_finish,
        name="link_2_body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_end_link_shape(), "link_3"),
        material=link_finish,
        name="link_3_body",
    )

    hinge_limits = MotionLimits(effort=24.0, velocity=2.0, lower=-1.6, upper=1.6)

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_SPAN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.4, lower=-1.7, upper=1.7),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.0, 0.0, -LINK_SPAN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.6, lower=-1.7, upper=1.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    support_bracket = object_model.get_part("support_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")

    ctx.allow_overlap(
        support_bracket,
        link_1,
        reason="Jointed hinge knuckle contact around the first axle is intentional for the support bracket mount.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Adjacent folding links share nested hinge hardware at their revolute joint.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        reason="The end link is carried by the second link through an intentional hinge-pin knuckle interface.",
    )

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

    hinge_1 = object_model.get_articulation("support_to_link_1")
    hinge_2 = object_model.get_articulation("link_1_to_link_2")
    hinge_3 = object_model.get_articulation("link_2_to_link_3")

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (support_bracket, link_1, link_2, link_3)),
        "Expected support bracket and all three hanging links.",
    )
    ctx.check(
        "parallel_hinge_axes",
        hinge_1.axis == hinge_2.axis == hinge_3.axis == (0.0, -1.0, 0.0),
        f"Expected all hinge axes parallel about -Y, got {hinge_1.axis}, {hinge_2.axis}, {hinge_3.axis}.",
    )

    ctx.expect_contact(support_bracket, link_1, name="support_bracket_carries_link_1")
    ctx.expect_contact(link_1, link_2, name="link_1_carries_link_2")
    ctx.expect_contact(link_2, link_3, name="link_2_carries_link_3")

    link_1_pos = ctx.part_world_position(link_1)
    link_2_pos = ctx.part_world_position(link_2)
    link_3_pos = ctx.part_world_position(link_3)
    hanging_ok = (
        link_1_pos is not None
        and link_2_pos is not None
        and link_3_pos is not None
        and abs(link_1_pos[0]) < 1e-6
        and abs(link_2_pos[0]) < 1e-6
        and abs(link_3_pos[0]) < 1e-6
        and link_2_pos[2] < link_1_pos[2] - 0.12
        and link_3_pos[2] < link_2_pos[2] - 0.12
    )
    ctx.check(
        "rest_pose_hangs_in_sequence",
        hanging_ok,
        f"Expected a vertical under-slung chain, got positions {link_1_pos}, {link_2_pos}, {link_3_pos}.",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    rest_link_3_center = _aabb_center(ctx.part_world_aabb(link_3))
    with ctx.pose({"support_to_link_1": 0.85, "link_1_to_link_2": 0.55, "link_2_to_link_3": 0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")
        folded_link_3_center = _aabb_center(ctx.part_world_aabb(link_3))
        folded_ok = (
            rest_link_3_center is not None
            and folded_link_3_center is not None
            and folded_link_3_center[0] > rest_link_3_center[0] + 0.05
            and folded_link_3_center[2] > rest_link_3_center[2] + 0.04
        )
        ctx.check(
            "positive_joint_motion_folds_chain_forward",
            folded_ok,
            f"Expected the end link to swing forward and upward, got rest={rest_link_3_center}, folded={folded_link_3_center}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
