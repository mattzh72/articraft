from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BACKPLATE_SIZE = (0.024, 0.240, 0.420)
SHOULDER_ORIGIN = (0.205, 0.0, 0.086)
ELBOW_ORIGIN = (0.540, 0.0, -0.032)
WRIST_ORIGIN = (0.745, 0.0, 0.026)


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def xz_prism(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width).translate((0.0, width / 2.0, 0.0))


def cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y + length / 2.0, z))


def add_all(base: cq.Workplane, *others: cq.Workplane) -> cq.Workplane:
    result = base
    for other in others:
        result = result.union(other)
    return result


def make_backplate() -> cq.Workplane:
    plate = box_at(BACKPLATE_SIZE, (0.0, 0.0, 0.0))
    center_spine = box_at((0.032, 0.100, 0.300), (0.024, 0.0, 0.0))
    upper_pad = box_at((0.032, 0.160, 0.085), (0.024, 0.0, 0.145))
    lower_pad = box_at((0.032, 0.160, 0.085), (0.024, 0.0, -0.145))
    return add_all(plate, center_spine, upper_pad, lower_pad)


def make_shoulder_bracket() -> cq.Workplane:
    web_profile = [
        (0.072, -0.082),
        (0.072, 0.098),
        (0.108, 0.098),
        (0.146, 0.086),
        (0.176, 0.068),
        (0.176, 0.024),
        (0.154, 0.016),
        (0.122, -0.010),
        (0.102, -0.082),
    ]
    rear_core = box_at((0.072, 0.140, 0.240), (0.072, 0.0, 0.0))
    web = xz_prism(web_profile, 0.060)
    left_cheek = box_at((0.050, 0.018, 0.112), (SHOULDER_ORIGIN[0] - 0.025, 0.029, SHOULDER_ORIGIN[2]))
    right_cheek = box_at((0.050, 0.018, 0.112), (SHOULDER_ORIGIN[0] - 0.025, -0.029, SHOULDER_ORIGIN[2]))
    lower_bridge = box_at((0.036, 0.076, 0.036), (SHOULDER_ORIGIN[0] - 0.043, 0.0, SHOULDER_ORIGIN[2] - 0.046))
    upper_bridge = box_at((0.024, 0.064, 0.024), (SHOULDER_ORIGIN[0] - 0.055, 0.0, SHOULDER_ORIGIN[2] + 0.046))
    return add_all(rear_core, web, left_cheek, right_cheek, lower_bridge, upper_bridge)


def make_first_link() -> cq.Workplane:
    shoulder_tongue = box_at((0.040, 0.040, 0.072), (0.020, 0.0, 0.0))
    beam = xz_prism(
        [
            (0.036, -0.046),
            (0.036, 0.046),
            (0.152, 0.052),
            (0.336, 0.044),
            (0.486, 0.026),
            (0.486, -0.024),
            (0.334, -0.040),
            (0.152, -0.052),
        ],
        0.042,
    )
    top_rib = box_at((0.190, 0.016, 0.018), (0.238, 0.0, 0.030))
    elbow_left = box_at((0.092, 0.018, 0.112), (ELBOW_ORIGIN[0] - 0.046, 0.029, ELBOW_ORIGIN[2] - 0.002))
    elbow_right = box_at((0.092, 0.018, 0.112), (ELBOW_ORIGIN[0] - 0.046, -0.029, ELBOW_ORIGIN[2] - 0.002))
    elbow_bridge = box_at((0.030, 0.070, 0.034), (ELBOW_ORIGIN[0] - 0.072, 0.0, ELBOW_ORIGIN[2] - 0.042))
    return add_all(shoulder_tongue, beam, top_rib, elbow_left, elbow_right, elbow_bridge)


def make_forearm_link() -> cq.Workplane:
    elbow_knuckle = box_at((0.050, 0.040, 0.120), (0.025, 0.0, 0.0))
    elbow_pack = box_at((0.120, 0.070, 0.156), (0.090, 0.0, -0.028))
    beam = xz_prism(
        [
            (0.122, -0.018),
            (0.122, 0.074),
            (0.264, 0.072),
            (0.560, 0.056),
            (0.706, 0.040),
            (0.706, 0.008),
            (0.564, -0.010),
            (0.258, 0.000),
        ],
        0.042,
    )
    top_rib = box_at((0.240, 0.016, 0.018), (0.366, 0.0, 0.046))
    wrist_left = box_at((0.080, 0.018, 0.100), (WRIST_ORIGIN[0] - 0.040, 0.029, WRIST_ORIGIN[2]))
    wrist_right = box_at((0.080, 0.018, 0.100), (WRIST_ORIGIN[0] - 0.040, -0.029, WRIST_ORIGIN[2]))
    wrist_bridge = box_at((0.028, 0.070, 0.032), (WRIST_ORIGIN[0] - 0.070, 0.0, WRIST_ORIGIN[2] - 0.038))
    return add_all(elbow_knuckle, elbow_pack, beam, top_rib, wrist_left, wrist_right, wrist_bridge)


def make_end_plate() -> cq.Workplane:
    wrist_tongue = box_at((0.040, 0.040, 0.092), (0.020, 0.0, 0.0))
    adapter = box_at((0.100, 0.080, 0.100), (0.068, 0.0, 0.0))
    plate = box_at((0.018, 0.180, 0.180), (0.118, 0.0, 0.0))
    hole_pattern = (
        cq.Workplane("YZ")
        .pushPoints([(-0.055, -0.055), (-0.055, 0.055), (0.055, -0.055), (0.055, 0.055)])
        .circle(0.010)
        .extrude(0.040)
        .translate((0.096, 0.0, 0.0))
    )
    return add_all(wrist_tongue, adapter, plate).cut(hole_pattern)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_transfer_arm")

    graphite = model.material("graphite", color=(0.18, 0.19, 0.21))
    bracket_gray = model.material("bracket_gray", color=(0.30, 0.32, 0.35))
    arm_orange = model.material("arm_orange", color=(0.76, 0.50, 0.16))
    endplate_dark = model.material("endplate_dark", color=(0.24, 0.25, 0.27))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(make_backplate(), "backplate"),
        material=graphite,
        name="backplate_shell",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((0.040, 0.240, 0.420)),
        mass=18.0,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    shoulder_bracket = model.part("shoulder_bracket")
    shoulder_bracket.visual(
        mesh_from_cadquery(make_shoulder_bracket(), "shoulder_bracket"),
        material=bracket_gray,
        name="shoulder_bracket_shell",
    )
    shoulder_bracket.inertial = Inertial.from_geometry(
        Box((0.235, 0.140, 0.235)),
        mass=9.0,
        origin=Origin(xyz=(0.135, 0.0, 0.020)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(make_first_link(), "first_link"),
        material=arm_orange,
        name="first_link_shell",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((0.600, 0.120, 0.125)),
        mass=10.0,
        origin=Origin(xyz=(0.300, 0.0, -0.006)),
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        mesh_from_cadquery(make_forearm_link(), "forearm_link"),
        material=arm_orange,
        name="forearm_link_shell",
    )
    forearm_link.inertial = Inertial.from_geometry(
        Box((0.820, 0.120, 0.170)),
        mass=13.0,
        origin=Origin(xyz=(0.360, 0.0, -0.005)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        mesh_from_cadquery(make_end_plate(), "end_plate"),
        material=endplate_dark,
        name="end_plate_shell",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((0.130, 0.180, 0.180)),
        mass=4.0,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    model.articulation(
        "backplate_to_shoulder_bracket",
        ArticulationType.FIXED,
        parent=backplate,
        child=shoulder_bracket,
        origin=Origin(),
    )
    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder_bracket,
        child=first_link,
        origin=Origin(xyz=SHOULDER_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-1.0, upper=0.75),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=forearm_link,
        origin=Origin(xyz=ELBOW_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.4, lower=-1.35, upper=1.1),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=end_plate,
        origin=Origin(xyz=WRIST_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8, lower=-1.3, upper=1.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    shoulder_bracket = object_model.get_part("shoulder_bracket")
    first_link = object_model.get_part("first_link")
    forearm_link = object_model.get_part("forearm_link")
    end_plate = object_model.get_part("end_plate")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

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
        "parallel_horizontal_pivots",
        shoulder_joint.axis == (0.0, 1.0, 0.0)
        and elbow_joint.axis == (0.0, 1.0, 0.0)
        and wrist_joint.axis == (0.0, 1.0, 0.0),
        details="shoulder, elbow, and wrist should all rotate about the same horizontal Y axis",
    )

    ctx.expect_contact(shoulder_bracket, backplate, name="bracket_mounted_to_backplate")
    ctx.expect_contact(first_link, shoulder_bracket, name="shoulder_knuckle_supported")
    ctx.expect_contact(forearm_link, first_link, name="elbow_knuckle_supported")
    ctx.expect_contact(end_plate, forearm_link, name="wrist_supported")

    ctx.expect_origin_gap(first_link, backplate, axis="x", min_gap=0.18, name="shoulder_reaches_off_wall")
    ctx.expect_origin_gap(forearm_link, backplate, axis="x", min_gap=0.70, name="forearm_extends_farther_than_first_link")
    ctx.expect_origin_gap(end_plate, forearm_link, axis="x", min_gap=0.70, max_gap=0.80, name="end_plate_sits_at_forearm_tip")

    with ctx.pose({"shoulder_joint": -0.35, "elbow_joint": 0.75, "wrist_joint": 0.20}):
        ctx.expect_gap(end_plate, backplate, axis="x", min_gap=0.15, name="bent_pose_end_plate_clears_wall")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
