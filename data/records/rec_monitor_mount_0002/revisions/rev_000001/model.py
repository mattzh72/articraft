from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

WALL_PLATE_W = 0.090
WALL_PLATE_T = 0.008
WALL_PLATE_H = 0.160

BASE_JOINT_Y = 0.055
PRIMARY_LEN = 0.180
SECONDARY_LEN = 0.154
TILT_AXIS_Y = 0.040


def _cyl_x(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _cyl_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _cyl_z(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True)


def _wall_bracket_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(WALL_PLATE_W, WALL_PLATE_T, WALL_PLATE_H).translate((0.0, WALL_PLATE_T / 2.0, 0.0))
    rib = cq.Workplane("XY").box(0.038, 0.044, 0.110).translate((0.0, 0.028, 0.0))
    fork_bridge = cq.Workplane("XY").box(0.026, 0.008, 0.040).translate((0.0, BASE_JOINT_Y - 0.018, 0.0))
    left_ear = cq.Workplane("XY").box(0.008, 0.014, 0.048).translate((-0.019, BASE_JOINT_Y, 0.0))
    right_ear = cq.Workplane("XY").box(0.008, 0.014, 0.048).translate((0.019, BASE_JOINT_Y, 0.0))
    left_collar = _cyl_z(0.009, 0.004).translate((-0.025, BASE_JOINT_Y, 0.0))
    right_collar = _cyl_z(0.009, 0.004).translate((0.025, BASE_JOINT_Y, 0.0))
    service_cover = cq.Workplane("XY").box(0.026, 0.004, 0.064).translate((0.0, 0.046, 0.0))

    slot_upper = cq.Workplane("XY").box(0.010, 0.012, 0.030).translate((0.0, WALL_PLATE_T / 2.0, 0.048))
    slot_lower = cq.Workplane("XY").box(0.010, 0.012, 0.030).translate((0.0, WALL_PLATE_T / 2.0, -0.048))
    lightening = cq.Workplane("XY").box(0.018, 0.020, 0.052).translate((0.0, 0.026, 0.0))

    return (
        plate.union(rib)
        .union(fork_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(left_collar)
        .union(right_collar)
        .cut(slot_upper)
        .cut(slot_lower)
        .cut(lightening)
        .union(service_cover)
    )


def _primary_arm_shape() -> cq.Workplane:
    hub = _cyl_z(0.008, 0.028)
    root_pad_left = cq.Workplane("XY").box(0.006, 0.012, 0.040).translate((-0.012, 0.0, 0.0))
    root_pad_right = cq.Workplane("XY").box(0.006, 0.012, 0.040).translate((0.012, 0.0, 0.0))

    beam = cq.Workplane("XY").box(0.030, 0.112, 0.022).translate((0.0, 0.116, 0.0))
    beam_void = cq.Workplane("XY").box(0.017, 0.072, 0.012).translate((0.0, 0.116, 0.0))
    side_window_a = cq.Workplane("XY").box(0.010, 0.024, 0.016).translate((0.0, 0.090, 0.0))
    side_window_b = cq.Workplane("XY").box(0.010, 0.024, 0.016).translate((0.0, 0.142, 0.0))
    cable_slot = cq.Workplane("XY").box(0.008, 0.046, 0.007).translate((0.0, 0.118, -0.006))

    spring_tube = _cyl_y(0.0085, 0.082).translate((0.0, 0.120, 0.018))
    spring_tube = spring_tube.union(cq.Workplane("XY").box(0.012, 0.014, 0.018).translate((0.0, 0.084, 0.010)))
    spring_tube = spring_tube.union(cq.Workplane("XY").box(0.012, 0.014, 0.018).translate((0.0, 0.156, 0.010)))

    fork_bridge = cq.Workplane("XY").box(0.026, 0.008, 0.040).translate((0.0, PRIMARY_LEN - 0.018, 0.0))
    left_ear = cq.Workplane("XY").box(0.008, 0.014, 0.048).translate((-0.019, PRIMARY_LEN, 0.0))
    right_ear = cq.Workplane("XY").box(0.008, 0.014, 0.048).translate((0.019, PRIMARY_LEN, 0.0))
    left_collar = _cyl_z(0.009, 0.004).translate((-0.025, PRIMARY_LEN, 0.0))
    right_collar = _cyl_z(0.009, 0.004).translate((0.025, PRIMARY_LEN, 0.0))

    return (
        hub.union(root_pad_left)
        .union(root_pad_right)
        .union(beam.cut(beam_void).cut(side_window_a).cut(side_window_b).cut(cable_slot))
        .union(spring_tube)
        .union(fork_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(left_collar)
        .union(right_collar)
    )


def _secondary_arm_shape() -> cq.Workplane:
    hub = _cyl_z(0.008, 0.024)
    root_pad_left = cq.Workplane("XY").box(0.006, 0.012, 0.038).translate((-0.012, 0.0, 0.0))
    root_pad_right = cq.Workplane("XY").box(0.006, 0.012, 0.038).translate((0.012, 0.0, 0.0))

    beam = cq.Workplane("XY").box(0.028, 0.092, 0.020).translate((0.0, 0.096, 0.0))
    beam_void = cq.Workplane("XY").box(0.016, 0.056, 0.011).translate((0.0, 0.096, 0.0))
    mid_window = cq.Workplane("XY").box(0.009, 0.024, 0.014).translate((0.0, 0.098, 0.0))
    cable_passage = cq.Workplane("XY").box(0.007, 0.038, 0.006).translate((0.0, 0.098, -0.005))
    housing = _cyl_y(0.007, 0.060).translate((0.0, 0.104, 0.017))
    housing = housing.union(cq.Workplane("XY").box(0.010, 0.012, 0.014).translate((0.0, 0.076, 0.010)))
    housing = housing.union(cq.Workplane("XY").box(0.010, 0.012, 0.014).translate((0.0, 0.132, 0.010)))

    fork_bridge = cq.Workplane("XY").box(0.022, 0.008, 0.034).translate((0.0, SECONDARY_LEN - 0.018, 0.0))
    left_ear = cq.Workplane("XY").box(0.008, 0.014, 0.042).translate((-0.019, SECONDARY_LEN, 0.0))
    right_ear = cq.Workplane("XY").box(0.008, 0.014, 0.042).translate((0.019, SECONDARY_LEN, 0.0))
    left_collar = _cyl_z(0.008, 0.004).translate((-0.025, SECONDARY_LEN, 0.0))
    right_collar = _cyl_z(0.008, 0.004).translate((0.025, SECONDARY_LEN, 0.0))

    return (
        hub.union(root_pad_left)
        .union(root_pad_right)
        .union(beam.cut(beam_void).cut(mid_window).cut(cable_passage))
        .union(housing)
        .union(fork_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(left_collar)
        .union(right_collar)
    )


def _pan_head_shape() -> cq.Workplane:
    root_hub = _cyl_z(0.007, 0.020)
    root_pad_left = cq.Workplane("XY").box(0.006, 0.010, 0.034).translate((-0.012, 0.0, 0.0))
    root_pad_right = cq.Workplane("XY").box(0.006, 0.010, 0.034).translate((0.012, 0.0, 0.0))
    collar = _cyl_z(0.008, 0.004)
    body = cq.Workplane("XY").box(0.020, 0.020, 0.016).translate((0.0, 0.018, 0.0))
    cover = cq.Workplane("XY").box(0.014, 0.004, 0.020).translate((0.0, 0.018, -0.009))

    upper_clevis = cq.Workplane("XY").box(0.032, 0.012, 0.006).translate((0.0, TILT_AXIS_Y, 0.017))
    lower_clevis = cq.Workplane("XY").box(0.032, 0.012, 0.006).translate((0.0, TILT_AXIS_Y, -0.017))
    backplate = cq.Workplane("XY").box(0.016, 0.008, 0.030).translate((0.0, TILT_AXIS_Y - 0.010, 0.0))

    return (
        root_hub.union(root_pad_left)
        .union(root_pad_right)
        .union(collar)
        .union(body)
        .union(cover)
        .union(upper_clevis)
        .union(lower_clevis)
        .union(backplate)
    )


def _tilt_trunnion_shape() -> cq.Workplane:
    trunnion = _cyl_x(0.007, 0.022)
    pad_upper = cq.Workplane("XY").box(0.028, 0.010, 0.006).translate((0.0, 0.0, 0.011))
    pad_lower = cq.Workplane("XY").box(0.028, 0.010, 0.006).translate((0.0, 0.0, -0.011))
    carrier = cq.Workplane("XY").box(0.022, 0.012, 0.020).translate((0.0, 0.014, 0.0))
    spine = cq.Workplane("XY").box(0.014, 0.034, 0.018).translate((0.0, 0.038, 0.0))
    gusset_upper = cq.Workplane("XZ").polyline([(-0.009, 0.0), (-0.009, 0.012), (0.0, 0.022), (0.009, 0.012), (0.009, 0.0)]).close().extrude(0.004).translate((0.0, 0.024, 0.0))
    return trunnion.union(pad_upper).union(pad_lower).union(carrier).union(spine).union(gusset_upper)


def _vesa_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XZ").box(0.110, 0.110, 0.003).translate((0.0, 0.0015, 0.0))
    center_land = cq.Workplane("XZ").box(0.030, 0.030, 0.007).translate((0.0, 0.005, 0.0))
    vertical_slot_a = cq.Workplane("XZ").box(0.008, 0.020, 0.006).translate((-0.0375, 0.0015, 0.0375))
    vertical_slot_b = cq.Workplane("XZ").box(0.008, 0.020, 0.006).translate((0.0375, 0.0015, 0.0375))
    vertical_slot_c = cq.Workplane("XZ").box(0.008, 0.020, 0.006).translate((-0.0375, 0.0015, -0.0375))
    vertical_slot_d = cq.Workplane("XZ").box(0.008, 0.020, 0.006).translate((0.0375, 0.0015, -0.0375))
    cable_window = cq.Workplane("XZ").box(0.022, 0.038, 0.006).translate((0.0, 0.0015, 0.0))
    return plate.union(center_land).cut(vertical_slot_a).cut(vertical_slot_b).cut(vertical_slot_c).cut(vertical_slot_d).cut(cable_window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_mount_mechanical_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.45, 0.47, 0.50, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.60, 0.62, 0.65, 1.0))
    service_gray = model.material("service_gray", rgba=(0.32, 0.34, 0.36, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(_wall_bracket_shape(), "wall_bracket.obj", assets=ASSETS),
        name="bracket_structure",
        material=steel_dark,
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((WALL_PLATE_W, 0.070, WALL_PLATE_H)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        mesh_from_cadquery(_primary_arm_shape(), "primary_arm.obj", assets=ASSETS),
        name="arm_body",
        material=steel_mid,
    )
    primary_arm.visual(
        Box((0.022, 0.064, 0.004)),
        origin=Origin(xyz=(0.0, 0.094, -0.010)),
        name="access_cover",
        material=service_gray,
    )
    primary_arm.inertial = Inertial.from_geometry(
        Box((0.040, 0.190, 0.055)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.095, 0.006)),
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        mesh_from_cadquery(_secondary_arm_shape(), "secondary_arm.obj", assets=ASSETS),
        name="arm_body",
        material=steel_mid,
    )
    secondary_arm.visual(
        Box((0.020, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.080, -0.009)),
        name="access_cover",
        material=service_gray,
    )
    secondary_arm.inertial = Inertial.from_geometry(
        Box((0.036, 0.162, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.081, 0.004)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        mesh_from_cadquery(_pan_head_shape(), "pan_head.obj", assets=ASSETS),
        name="pan_body",
        material=steel_dark,
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.040, 0.060, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        mesh_from_cadquery(_tilt_trunnion_shape(), "tilt_carrier.obj", assets=ASSETS),
        name="trunnion_carrier",
        material=plate_gray,
    )
    tilt_head.visual(
        mesh_from_cadquery(_vesa_plate_shape(), "vesa_plate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        name="vesa_plate",
        material=plate_gray,
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.060, 0.120)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
    )

    model.articulation(
        "wall_bracket_to_primary_arm",
        ArticulationType.CONTINUOUS,
        parent=wall_bracket,
        child=primary_arm,
        origin=Origin(xyz=(0.0, BASE_JOINT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2),
    )
    model.articulation(
        "primary_arm_to_secondary_arm",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(0.0, PRIMARY_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-2.3, upper=2.3),
    )
    model.articulation(
        "secondary_arm_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=secondary_arm,
        child=pan_head,
        origin=Origin(xyz=(0.0, SECONDARY_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "pan_head_to_tilt_head",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_head,
        origin=Origin(xyz=(0.0, TILT_AXIS_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-0.95, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_bracket = object_model.get_part("wall_bracket")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    pan_head = object_model.get_part("pan_head")
    tilt_head = object_model.get_part("tilt_head")

    vesa_plate = tilt_head.get_visual("vesa_plate")
    primary_cover = primary_arm.get_visual("access_cover")
    secondary_cover = secondary_arm.get_visual("access_cover")

    base_pan = object_model.get_articulation("wall_bracket_to_primary_arm")
    fold_joint = object_model.get_articulation("primary_arm_to_secondary_arm")
    head_pan = object_model.get_articulation("secondary_arm_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_tilt_head")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "expected mechanical study parts exist",
        len(object_model.parts) == 5,
        f"expected 5 parts, found {len(object_model.parts)}",
    )

    ctx.expect_contact(primary_arm, wall_bracket, name="base pan knuckle contacts bracket")
    ctx.expect_contact(secondary_arm, primary_arm, name="elbow fold knuckle contacts primary arm")
    ctx.expect_contact(pan_head, secondary_arm, name="head pan knuckle contacts secondary arm")
    ctx.expect_contact(tilt_head, pan_head, name="tilt trunnion contacts clevis")

    ctx.expect_contact(primary_arm, primary_arm, elem_a=primary_cover, elem_b="arm_body", name="primary access cover is seated")
    ctx.expect_contact(secondary_arm, secondary_arm, elem_a=secondary_cover, elem_b="arm_body", name="secondary access cover is seated")

    ctx.expect_origin_gap(tilt_head, wall_bracket, axis="y", min_gap=0.32, name="display plate sits well forward of wall bracket")
    ctx.expect_overlap(tilt_head, secondary_arm, axes="x", min_overlap=0.02, name="head remains centered on secondary arm width")
    ctx.expect_overlap(tilt_head, wall_bracket, axes="z", min_overlap=0.06, elem_a=vesa_plate, elem_b="bracket_structure", name="plate remains vertically aligned with support stack")
    ctx.expect_gap(tilt_head, pan_head, axis="y", min_gap=0.018, max_gap=0.050, positive_elem=vesa_plate, negative_elem="pan_body", name="vesa plate stands off from pan body")

    with ctx.pose({base_pan: 1.10}):
        ctx.expect_contact(primary_arm, wall_bracket, name="base pan stays seated when rotated")
        ctx.expect_origin_gap(tilt_head, wall_bracket, axis="y", min_gap=0.10, name="rotated assembly still projects away from wall")

    with ctx.pose({fold_joint: 1.35}):
        ctx.expect_contact(secondary_arm, primary_arm, name="fold joint stays seated when folded")
        ctx.expect_origin_gap(tilt_head, primary_arm, axis="y", min_gap=0.05, name="folded head remains outboard of primary arm root")

    with ctx.pose({head_pan: -1.20}):
        ctx.expect_contact(pan_head, secondary_arm, name="head pan stays seated through rotation")

    with ctx.pose({tilt_joint: 0.55}):
        ctx.expect_contact(tilt_head, pan_head, name="tilt trunnion stays seated at upward tilt")
        ctx.expect_overlap(tilt_head, pan_head, axes="x", min_overlap=0.02, name="tilt axis remains centered in clevis at operating pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
