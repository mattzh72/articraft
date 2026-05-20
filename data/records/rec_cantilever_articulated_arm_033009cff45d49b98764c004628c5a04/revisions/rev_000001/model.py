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


UPPER_LINK_LENGTH = 0.46
FORELINK_LENGTH = 0.36


def _box_span(x_min: float, x_max: float, width_y: float, height_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x_max - x_min, width_y, height_z, centered=(False, True, True))
        .translate((x_min, 0.0, 0.0))
    )


def _centered_profile(points: list[tuple[float, float]], width_y: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width_y / 2.0, both=True)


def _center_bar(x_min: float, x_max: float, width_y: float, height_z: float) -> cq.Workplane:
    return _box_span(x_min, x_max, width_y, height_z)


def _clevis_plates(
    x_min: float,
    x_max: float,
    *,
    outer_width_y: float,
    gap_width_y: float,
    height_z: float,
) -> cq.Workplane:
    plate_width = (outer_width_y - gap_width_y) / 2.0
    y_center = gap_width_y / 2.0 + plate_width / 2.0
    left = _box_span(x_min, x_max, plate_width, height_z).translate((0.0, y_center, 0.0))
    right = _box_span(x_min, x_max, plate_width, height_z).translate((0.0, -y_center, 0.0))
    return left.union(right)


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _root_block_shape() -> cq.Workplane:
    back_flange = _box_span(-0.165, -0.145, 0.170, 0.220)
    main_block = _box_span(-0.145, -0.020, 0.145, 0.185)
    fork_block = _box_span(-0.102, 0.000, 0.110, 0.122)
    center_spine = _center_bar(-0.058, -0.014, 0.058, 0.102)
    upper_brace = _box_span(-0.126, -0.040, 0.098, 0.026).translate((0.0, 0.0, 0.070))
    lower_brace = _box_span(-0.126, -0.040, 0.098, 0.026).translate((0.0, 0.0, -0.070))
    side_ribs = _box_span(-0.128, -0.022, 0.016, 0.148).translate((0.0, 0.034, 0.0)).union(
        _box_span(-0.128, -0.022, 0.016, 0.148).translate((0.0, -0.034, 0.0))
    )
    clevis_slot = _box_span(-0.032, 0.000, 0.032, 0.088)

    mount_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.052, -0.072), (-0.052, 0.072), (0.052, -0.072), (0.052, 0.072)])
        .circle(0.009)
        .extrude(0.050)
        .translate((-0.175, 0.0, 0.0))
    )

    return (
        back_flange.union(main_block)
        .union(fork_block)
        .union(center_spine)
        .union(upper_brace)
        .union(lower_brace)
        .union(side_ribs)
        .cut(clevis_slot)
        .cut(mount_holes)
    )


def _upper_link_shape() -> cq.Workplane:
    proximal_lug = _center_bar(0.000, 0.062, 0.030, 0.066)
    body = _centered_profile(
        [
            (0.048, -0.040),
            (0.048, 0.040),
            (0.155, 0.054),
            (0.295, 0.052),
            (UPPER_LINK_LENGTH - 0.110, 0.042),
            (UPPER_LINK_LENGTH - 0.110, -0.042),
            (0.295, -0.052),
            (0.155, -0.054),
        ],
        0.060,
    )
    distal_fork = _box_span(UPPER_LINK_LENGTH - 0.092, UPPER_LINK_LENGTH, 0.100, 0.094)
    upper_cap = _box_span(0.110, UPPER_LINK_LENGTH - 0.126, 0.046, 0.020).translate((0.0, 0.0, 0.040))
    lower_cap = _box_span(0.110, UPPER_LINK_LENGTH - 0.126, 0.046, 0.020).translate((0.0, 0.0, -0.040))
    clevis_slot = _box_span(UPPER_LINK_LENGTH - 0.028, UPPER_LINK_LENGTH, 0.032, 0.068)

    return (
        proximal_lug.union(body)
        .union(distal_fork)
        .union(upper_cap)
        .union(lower_cap)
        .cut(clevis_slot)
    )


def _forelink_shape() -> cq.Workplane:
    proximal_lug = _center_bar(0.000, 0.056, 0.028, 0.056)
    body = _centered_profile(
        [
            (0.044, -0.034),
            (0.044, 0.034),
            (0.128, 0.046),
            (0.228, 0.042),
            (FORELINK_LENGTH - 0.104, 0.034),
            (FORELINK_LENGTH - 0.104, -0.034),
            (0.228, -0.042),
            (0.128, -0.046),
        ],
        0.052,
    )
    distal_fork = _box_span(FORELINK_LENGTH - 0.082, FORELINK_LENGTH, 0.088, 0.082)
    upper_cap = _box_span(0.086, FORELINK_LENGTH - 0.114, 0.040, 0.016).translate((0.0, 0.0, 0.032))
    lower_cap = _box_span(0.086, FORELINK_LENGTH - 0.114, 0.040, 0.016).translate((0.0, 0.0, -0.032))
    clevis_slot = _box_span(FORELINK_LENGTH - 0.024, FORELINK_LENGTH, 0.028, 0.058)

    return (
        proximal_lug.union(body)
        .union(distal_fork)
        .union(upper_cap)
        .union(lower_cap)
        .cut(clevis_slot)
    )


def _end_plate_shape() -> cq.Workplane:
    proximal_lug = _center_bar(0.000, 0.052, 0.026, 0.050)
    neck = _centered_profile(
        [
            (0.042, -0.026),
            (0.042, 0.026),
            (0.070, 0.036),
            (0.094, 0.036),
            (0.094, -0.036),
            (0.070, -0.036),
        ],
        0.040,
    )
    plate = _box_span(0.094, 0.118, 0.140, 0.140)
    boss = (
        cq.Workplane("YZ")
        .circle(0.028)
        .extrude(0.028)
        .translate((0.092, 0.0, 0.0))
    )
    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.040, -0.040), (-0.040, 0.040), (0.040, -0.040), (0.040, 0.040)])
        .circle(0.007)
        .extrude(0.034)
        .translate((0.092, 0.0, 0.0))
    )

    return proximal_lug.union(neck).union(plate).union(boss).cut(bolt_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_cantilever_arm")

    model.material("root_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("link_steel", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("forelink_steel", rgba=(0.54, 0.58, 0.63, 1.0))
    model.material("tool_plate", rgba=(0.18, 0.20, 0.22, 1.0))

    root_block = model.part("root_block")
    _add_box_visual(root_block, (0.115, 0.145, 0.185), (-0.0875, 0.0, 0.0), material="root_steel", name="root_body")
    _add_box_visual(root_block, (0.020, 0.170, 0.220), (-0.155, 0.0, 0.0), material="root_steel", name="root_flange")
    _add_box_visual(root_block, (0.030, 0.050, 0.100), (-0.015, 0.0, 0.0), material="root_steel", name="root_hinge_block")
    _add_box_visual(root_block, (0.060, 0.030, 0.120), (-0.030, 0.035, 0.0), material="root_steel", name="root_left_cheek")
    _add_box_visual(root_block, (0.060, 0.030, 0.120), (-0.030, -0.035, 0.0), material="root_steel", name="root_right_cheek")
    _add_box_visual(root_block, (0.095, 0.094, 0.026), (-0.0675, 0.0, 0.070), material="root_steel", name="root_upper_brace")
    _add_box_visual(root_block, (0.095, 0.094, 0.026), (-0.0675, 0.0, -0.070), material="root_steel", name="root_lower_brace")
    _add_box_visual(root_block, (0.095, 0.016, 0.148), (-0.0675, 0.034, 0.0), material="root_steel", name="root_left_rib")
    _add_box_visual(root_block, (0.095, 0.016, 0.148), (-0.0675, -0.034, 0.0), material="root_steel", name="root_right_rib")
    root_block.inertial = Inertial.from_geometry(
        Box((0.165, 0.170, 0.220)),
        mass=18.0,
        origin=Origin(xyz=(-0.0825, 0.0, 0.0)),
    )

    upper_link = model.part("upper_link")
    _add_box_visual(upper_link, (0.055, 0.028, 0.062), (0.0275, 0.0, 0.0), material="link_steel", name="upper_lug")
    _add_box_visual(upper_link, (0.310, 0.055, 0.075), (0.200, 0.0, 0.0), material="link_steel", name="upper_beam")
    _add_box_visual(upper_link, (0.070, 0.088, 0.048), (0.383, 0.0, 0.0), material="link_steel", name="upper_fork_bridge")
    _add_box_visual(upper_link, (0.045, 0.028, 0.050), (0.438, 0.0, 0.0), material="link_steel", name="upper_hinge_nose")
    _add_box_visual(upper_link, (0.056, 0.030, 0.092), (0.432, 0.033, 0.0), material="link_steel", name="upper_left_fork")
    _add_box_visual(upper_link, (0.056, 0.030, 0.092), (0.432, -0.033, 0.0), material="link_steel", name="upper_right_fork")
    _add_box_visual(upper_link, (0.260, 0.042, 0.016), (0.205, 0.0, 0.041), material="link_steel", name="upper_top_stiffener")
    _add_box_visual(upper_link, (0.260, 0.042, 0.016), (0.205, 0.0, -0.041), material="link_steel", name="upper_bottom_stiffener")
    upper_link.inertial = Inertial.from_geometry(
        Box((UPPER_LINK_LENGTH + 0.060, 0.110, 0.120)),
        mass=9.0,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
    )

    forelink = model.part("forelink")
    _add_box_visual(forelink, (0.050, 0.028, 0.054), (0.025, 0.0, 0.0), material="forelink_steel", name="fore_lug")
    _add_box_visual(forelink, (0.235, 0.048, 0.064), (0.1575, 0.0, 0.0), material="forelink_steel", name="fore_beam")
    _add_box_visual(forelink, (0.052, 0.080, 0.042), (0.296, 0.0, 0.0), material="forelink_steel", name="fore_fork_bridge")
    _add_box_visual(forelink, (0.042, 0.026, 0.046), (0.339, 0.0, 0.0), material="forelink_steel", name="fore_hinge_nose")
    _add_box_visual(forelink, (0.050, 0.028, 0.082), (0.335, 0.029, 0.0), material="forelink_steel", name="fore_left_fork")
    _add_box_visual(forelink, (0.050, 0.028, 0.082), (0.335, -0.029, 0.0), material="forelink_steel", name="fore_right_fork")
    _add_box_visual(forelink, (0.180, 0.038, 0.014), (0.170, 0.0, 0.033), material="forelink_steel", name="fore_top_stiffener")
    _add_box_visual(forelink, (0.180, 0.038, 0.014), (0.170, 0.0, -0.033), material="forelink_steel", name="fore_bottom_stiffener")
    forelink.inertial = Inertial.from_geometry(
        Box((FORELINK_LENGTH + 0.055, 0.090, 0.095)),
        mass=6.2,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    _add_box_visual(end_plate, (0.050, 0.026, 0.048), (0.025, 0.0, 0.0), material="tool_plate", name="plate_lug")
    _add_box_visual(end_plate, (0.043, 0.040, 0.056), (0.0665, 0.0, 0.0), material="tool_plate", name="plate_neck")
    _add_box_visual(end_plate, (0.024, 0.140, 0.140), (0.100, 0.0, 0.0), material="tool_plate", name="plate_face")
    _add_box_visual(end_plate, (0.018, 0.056, 0.056), (0.089, 0.0, 0.0), material="tool_plate", name="plate_boss")
    end_plate.inertial = Inertial.from_geometry(
        Box((0.118, 0.140, 0.140)),
        mass=2.2,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_upper",
        ArticulationType.REVOLUTE,
        parent=root_block,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-1.10, upper=1.20),
    )
    model.articulation(
        "upper_to_fore",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-1.80, upper=1.35),
    )
    model.articulation(
        "fore_to_plate",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=end_plate,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_block = object_model.get_part("root_block")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    end_plate = object_model.get_part("end_plate")

    shoulder = object_model.get_articulation("root_to_upper")
    elbow = object_model.get_articulation("upper_to_fore")
    wrist = object_model.get_articulation("fore_to_plate")

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
        "required_parts_present",
        {"root_block", "upper_link", "forelink", "end_plate"}.issubset({part.name for part in object_model.parts}),
        "Model is missing one or more required arm parts.",
    )
    ctx.check(
        "three_serial_revolute_joints",
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, wrist)
        )
        and shoulder.parent == "root_block"
        and shoulder.child == "upper_link"
        and elbow.parent == "upper_link"
        and elbow.child == "forelink"
        and wrist.parent == "forelink"
        and wrist.child == "end_plate",
        "Expected a root-carried shoulder joint, upper-link elbow joint, and forelink wrist joint.",
    )
    ctx.check(
        "pitch_axes_face_forward",
        all(tuple(round(value, 6) for value in joint.axis) == (0.0, -1.0, 0.0) for joint in (shoulder, elbow, wrist)),
        "All three serial joints should pitch about the local Y hinge axis.",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        ctx.expect_gap(upper_link, root_block, axis="x", max_gap=0.001, max_penetration=0.001, name="shoulder_hinge_seating")
        ctx.expect_gap(forelink, upper_link, axis="x", max_gap=0.001, max_penetration=0.001, name="elbow_hinge_seating")
        ctx.expect_gap(end_plate, forelink, axis="x", max_gap=0.001, max_penetration=0.001, name="wrist_hinge_seating")
        ctx.expect_contact(root_block, upper_link, name="shoulder_contact")
        ctx.expect_contact(upper_link, forelink, name="elbow_contact")
        ctx.expect_contact(forelink, end_plate, name="wrist_contact")

    with ctx.pose({shoulder: 0.65, elbow: 0.35, wrist: 0.20}):
        root_pos = ctx.part_world_position(root_block)
        plate_pos = ctx.part_world_position(end_plate)
        pose_ok = (
            root_pos is not None
            and plate_pos is not None
            and plate_pos[0] > root_pos[0] + 0.50
            and plate_pos[2] > root_pos[2] + 0.45
        )
        ctx.check(
            "positive_pose_lifts_arm_forward",
            pose_ok,
            f"Expected the serial revolute chain to cantilever upward and forward; root={root_pos}, end_plate={plate_pos}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
