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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy, cz - (length * 0.5)))
    )


def _tube_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((cx, cy, cz - (length * 0.5)))
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy - (length * 0.5), cz))
    )


def _tube_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((cx, cy - (length * 0.5), cz))
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - (length * 0.5), cy, cz))
    )


def _base_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.38, 0.24, 0.02, centered=(True, True, False))
    relief = _box((0.18, 0.10, 0.008), (0.0, 0.0, 0.014))
    return plate.cut(relief)


def _base_feet_shape() -> cq.Workplane:
    foot = _cyl_z(0.018, 0.012, (0.14, 0.085, -0.006))
    for x in (-0.14, 0.14):
        for y in (-0.085, 0.085):
            if x == 0.14 and y == 0.085:
                continue
            foot = foot.union(_cyl_z(0.018, 0.012, (x, y, -0.006)))
    return foot


def _pedestal_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.12, 0.14, 0.12, centered=(True, True, False)).translate((0, 0, 0.02))
    side_window_x = _box((0.018, 0.080, 0.060), (0.051, 0.0, 0.085))
    side_window_x_m = _box((0.018, 0.080, 0.060), (-0.051, 0.0, 0.085))
    access_pocket = _box((0.076, 0.012, 0.072), (0.0, 0.064, 0.085))
    return body.cut(side_window_x).cut(side_window_x_m).cut(access_pocket)


def _base_mount_shape() -> cq.Workplane:
    mount = _cyl_z(0.055, 0.030, (0.0, 0.0, 0.155))
    bolt_relief = (
        cq.Workplane("XY")
        .pushPoints([(0.03, 0.03), (0.03, -0.03), (-0.03, 0.03), (-0.03, -0.03)])
        .circle(0.006)
        .extrude(0.008)
        .translate((0, 0, 0.162))
    )
    return mount.cut(bolt_relief)


def _base_access_cover_shape() -> cq.Workplane:
    cover = _box((0.078, 0.004, 0.074), (0.0, 0.072, 0.085))
    bosses = _cyl_y(0.0045, 0.004, (-0.026, 0.072, 0.109))
    bosses = bosses.union(_cyl_y(0.0045, 0.004, (0.026, 0.072, 0.109)))
    bosses = bosses.union(_cyl_y(0.0045, 0.004, (-0.026, 0.072, 0.061)))
    bosses = bosses.union(_cyl_y(0.0045, 0.004, (0.026, 0.072, 0.061)))
    return cover.union(bosses)


def _yaw_hub_shape() -> cq.Workplane:
    hub = _cyl_z(0.046, 0.070, (0.0, 0.0, 0.035))
    lower_flange = _cyl_z(0.070, 0.012, (0.0, 0.0, 0.006))
    upper_flange = _tube_z(0.067, 0.048, 0.012, (0.0, 0.0, 0.064))
    return hub.union(lower_flange).union(upper_flange)


def _yaw_beam_shape() -> cq.Workplane:
    backbone = _box((0.230, 0.055, 0.050), (0.160, 0.0, 0.110))
    tower = _box((0.090, 0.095, 0.135), (0.255, 0.0, 0.133))
    core_window = _box((0.140, 0.030, 0.060), (0.145, 0.0, 0.107))
    tower_window = _box((0.055, 0.050, 0.070), (0.255, 0.0, 0.133))
    return backbone.union(tower).cut(core_window).cut(tower_window)


def _yaw_access_cover_shape() -> cq.Workplane:
    cover = _box((0.140, 0.004, 0.085), (0.180, 0.0475, 0.130))
    pads = _cyl_y(0.004, 0.004, (0.132, 0.0475, 0.160))
    pads = pads.union(_cyl_y(0.004, 0.004, (0.228, 0.0475, 0.160)))
    pads = pads.union(_cyl_y(0.004, 0.004, (0.132, 0.0475, 0.100)))
    pads = pads.union(_cyl_y(0.004, 0.004, (0.228, 0.0475, 0.100)))
    return cover.union(pads)


def _yaw_clevis_left_shape() -> cq.Workplane:
    ear = _box((0.070, 0.015, 0.100), (0.300, 0.047, 0.215))
    gusset = (
        cq.Workplane("XZ")
        .polyline([(0.240, 0.125), (0.240, 0.175), (0.285, 0.215), (0.285, 0.125)])
        .close()
        .extrude(0.015)
        .translate((0.0, 0.0395, 0.0))
    )
    return ear.union(gusset)


def _yaw_clevis_right_shape() -> cq.Workplane:
    ear = _box((0.070, 0.015, 0.100), (0.300, -0.047, 0.215))
    gusset = (
        cq.Workplane("XZ")
        .polyline([(0.240, 0.125), (0.240, 0.175), (0.285, 0.215), (0.285, 0.125)])
        .close()
        .extrude(0.015)
        .translate((0.0, -0.0545, 0.0))
    )
    return ear.union(gusset)


def _elbow_hub_shape() -> cq.Workplane:
    hub = _cyl_y(0.030, 0.055, (0.0, 0.0, 0.0))
    left_spacer = _cyl_y(0.036, 0.012, (0.0, 0.0335, 0.0))
    right_spacer = _cyl_y(0.036, 0.012, (0.0, -0.0335, 0.0))
    return hub.union(left_spacer).union(right_spacer)


def _elbow_rear_block_shape() -> cq.Workplane:
    block = _box((0.070, 0.100, 0.090), (0.045, 0.0, 0.0))
    pocket = _box((0.032, 0.052, 0.050), (0.040, 0.0, 0.006))
    top_slot = _box((0.036, 0.060, 0.020), (0.050, 0.0, 0.027))
    return block.cut(pocket).cut(top_slot)


def _elbow_rail_left_shape() -> cq.Workplane:
    return _cyl_x(0.011, 0.280, (0.200, 0.050, 0.0))


def _elbow_rail_right_shape() -> cq.Workplane:
    return _cyl_x(0.011, 0.280, (0.200, -0.050, 0.0))


def _elbow_front_block_shape() -> cq.Workplane:
    block = _box((0.025, 0.130, 0.100), (0.3425, 0.0, 0.0))
    throat = _box((0.016, 0.074, 0.050), (0.3425, 0.0, 0.0))
    return block.cut(throat)


def _elbow_bridge_shape() -> cq.Workplane:
    return _box((0.290, 0.045, 0.012), (0.205, 0.0, -0.042))


def _elbow_access_cover_shape() -> cq.Workplane:
    cover = _box((0.070, 0.080, 0.005), (0.045, 0.0, 0.049))
    pads = _cyl_z(0.0035, 0.005, (0.020, 0.028, 0.049))
    pads = pads.union(_cyl_z(0.0035, 0.005, (0.020, -0.028, 0.049)))
    pads = pads.union(_cyl_z(0.0035, 0.005, (0.070, 0.028, 0.049)))
    pads = pads.union(_cyl_z(0.0035, 0.005, (0.070, -0.028, 0.049)))
    posts = _cyl_z(0.0025, 0.004, (0.020, 0.028, 0.047))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.020, -0.028, 0.047)))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.070, 0.028, 0.047)))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.070, -0.028, 0.047)))
    return cover.union(pads).union(posts)


def _slider_sleeve(center_y: float) -> cq.Workplane:
    sleeve = _box((0.060, 0.032, 0.032), (0.0, center_y, 0.0))
    bore = _cyl_x(0.011, 0.064, (0.0, center_y, 0.0))
    face_ring = _tube_x(0.015, 0.011, 0.004, (0.030, center_y, 0.0))
    rear_ring = _tube_x(0.015, 0.011, 0.004, (-0.030, center_y, 0.0))
    return sleeve.cut(bore).union(face_ring).union(rear_ring)


def _tube_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((cx - (length * 0.5), cy, cz))
    )


def _slider_bridge_shape() -> cq.Workplane:
    return _box((0.070, 0.120, 0.012), (-0.005, 0.0, 0.022))


def _slider_body_shape() -> cq.Workplane:
    body = _box((0.110, 0.080, 0.040), (0.045, 0.0, 0.048))
    channel = _box((0.050, 0.030, 0.018), (0.020, 0.0, 0.048))
    return body.cut(channel)


def _slider_output_head_shape() -> cq.Workplane:
    head = _box((0.030, 0.100, 0.080), (0.110, 0.0, 0.050))
    slot = _box((0.034, 0.028, 0.030), (0.110, 0.0, 0.050))
    return head.cut(slot)


def _slider_access_cover_shape() -> cq.Workplane:
    cover = _box((0.080, 0.060, 0.004), (0.030, 0.0, 0.071))
    posts = _cyl_z(0.0025, 0.004, (0.000, 0.020, 0.069))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.000, -0.020, 0.069)))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.060, 0.020, 0.069)))
    posts = posts.union(_cyl_z(0.0025, 0.004, (0.060, -0.020, 0.069)))
    return cover.union(posts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rrp_mechanical_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.42, 0.45, 0.49, 1.0))
    steel_light = model.material("steel_light", rgba=(0.68, 0.71, 0.75, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.54, 0.56, 0.60, 1.0))
    hardware = model.material("hardware", rgba=(0.78, 0.80, 0.82, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate.obj", assets=ASSETS),
        material=steel_dark,
        name="base_plate",
    )
    base_frame.visual(
        mesh_from_cadquery(_base_feet_shape(), "base_feet.obj", assets=ASSETS),
        material=steel_mid,
        name="base_feet",
    )
    base_frame.visual(
        mesh_from_cadquery(_pedestal_body_shape(), "pedestal_body.obj", assets=ASSETS),
        material=steel_mid,
        name="pedestal_body",
    )
    base_frame.visual(
        mesh_from_cadquery(_base_mount_shape(), "base_mount.obj", assets=ASSETS),
        material=steel_light,
        name="base_mount",
    )
    base_frame.visual(
        mesh_from_cadquery(_base_access_cover_shape(), "base_access_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="base_access_cover",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.24, 0.19)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    yaw_link = model.part("yaw_link")
    yaw_link.visual(
        mesh_from_cadquery(_yaw_hub_shape(), "yaw_hub.obj", assets=ASSETS),
        material=steel_light,
        name="yaw_hub",
    )
    yaw_link.visual(
        mesh_from_cadquery(_yaw_beam_shape(), "yaw_beam.obj", assets=ASSETS),
        material=steel_dark,
        name="yaw_beam",
    )
    yaw_link.visual(
        mesh_from_cadquery(_yaw_access_cover_shape(), "yaw_access_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="yaw_access_cover",
    )
    yaw_link.visual(
        mesh_from_cadquery(_yaw_clevis_left_shape(), "yaw_clevis_left.obj", assets=ASSETS),
        material=steel_mid,
        name="yaw_clevis_left",
    )
    yaw_link.visual(
        mesh_from_cadquery(_yaw_clevis_right_shape(), "yaw_clevis_right.obj", assets=ASSETS),
        material=steel_mid,
        name="yaw_clevis_right",
    )
    yaw_link.inertial = Inertial.from_geometry(
        Box((0.38, 0.13, 0.11)),
        mass=4.5,
        origin=Origin(xyz=(0.16, 0.0, 0.215)),
    )

    elbow_frame = model.part("elbow_frame")
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_hub_shape(), "elbow_hub.obj", assets=ASSETS),
        material=steel_light,
        name="elbow_hub",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_rear_block_shape(), "elbow_rear_block.obj", assets=ASSETS),
        material=steel_dark,
        name="elbow_rear_block",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_rail_left_shape(), "elbow_rail_left.obj", assets=ASSETS),
        material=hardware,
        name="elbow_rail_left",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_rail_right_shape(), "elbow_rail_right.obj", assets=ASSETS),
        material=hardware,
        name="elbow_rail_right",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_front_block_shape(), "elbow_front_block.obj", assets=ASSETS),
        material=steel_mid,
        name="elbow_front_block",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_bridge_shape(), "elbow_bridge.obj", assets=ASSETS),
        material=steel_mid,
        name="elbow_bridge",
    )
    elbow_frame.visual(
        mesh_from_cadquery(_elbow_access_cover_shape(), "elbow_access_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="elbow_access_cover",
    )
    elbow_frame.inertial = Inertial.from_geometry(
        Box((0.37, 0.14, 0.12)),
        mass=3.6,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    slider_carriage = model.part("slider_carriage")
    slider_carriage.visual(
        mesh_from_cadquery(_slider_sleeve(0.050), "slider_sleeve_left.obj", assets=ASSETS),
        material=steel_light,
        name="slider_sleeve_left",
    )
    slider_carriage.visual(
        mesh_from_cadquery(_slider_sleeve(-0.050), "slider_sleeve_right.obj", assets=ASSETS),
        material=steel_light,
        name="slider_sleeve_right",
    )
    slider_carriage.visual(
        mesh_from_cadquery(_slider_bridge_shape(), "slider_bridge.obj", assets=ASSETS),
        material=steel_mid,
        name="slider_bridge",
    )
    slider_carriage.visual(
        mesh_from_cadquery(_slider_body_shape(), "slider_body.obj", assets=ASSETS),
        material=steel_dark,
        name="slider_body",
    )
    slider_carriage.visual(
        mesh_from_cadquery(_slider_output_head_shape(), "slider_output_head.obj", assets=ASSETS),
        material=steel_mid,
        name="slider_output_head",
    )
    slider_carriage.visual(
        mesh_from_cadquery(_slider_access_cover_shape(), "slider_access_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="slider_access_cover",
    )
    slider_carriage.inertial = Inertial.from_geometry(
        Box((0.16, 0.13, 0.10)),
        mass=1.8,
        origin=Origin(xyz=(0.04, 0.0, 0.040)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=yaw_link,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "yaw_to_elbow",
        ArticulationType.REVOLUTE,
        parent=yaw_link,
        child=elbow_frame,
        origin=Origin(xyz=(0.32, 0.0, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-0.6, upper=1.0),
    )
    model.articulation(
        "elbow_to_slider",
        ArticulationType.PRISMATIC,
        parent=elbow_frame,
        child=slider_carriage,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.4, lower=0.0, upper=0.13),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    yaw_link = object_model.get_part("yaw_link")
    elbow_frame = object_model.get_part("elbow_frame")
    slider_carriage = object_model.get_part("slider_carriage")

    base_to_yaw = object_model.get_articulation("base_to_yaw")
    yaw_to_elbow = object_model.get_articulation("yaw_to_elbow")
    elbow_to_slider = object_model.get_articulation("elbow_to_slider")

    base_mount = base_frame.get_visual("base_mount")
    yaw_hub = yaw_link.get_visual("yaw_hub")
    yaw_clevis_left = yaw_link.get_visual("yaw_clevis_left")
    yaw_clevis_right = yaw_link.get_visual("yaw_clevis_right")
    elbow_hub = elbow_frame.get_visual("elbow_hub")
    elbow_rail_left = elbow_frame.get_visual("elbow_rail_left")
    elbow_rail_right = elbow_frame.get_visual("elbow_rail_right")
    slider_sleeve_left = slider_carriage.get_visual("slider_sleeve_left")
    slider_sleeve_right = slider_carriage.get_visual("slider_sleeve_right")
    slider_output_head = slider_carriage.get_visual("slider_output_head")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=True, ignore_fixed=True)
    ctx.allow_overlap(
        elbow_frame,
        slider_carriage,
        elem_a=elbow_rail_left,
        elem_b=slider_sleeve_left,
        reason="Left linear guide rail intentionally passes through the carriage sleeve.",
    )
    ctx.allow_overlap(
        elbow_frame,
        slider_carriage,
        elem_a=elbow_rail_right,
        elem_b=slider_sleeve_right,
        reason="Right linear guide rail intentionally passes through the carriage sleeve.",
    )
    ctx.allow_overlap(
        yaw_link,
        elbow_frame,
        elem_a=yaw_clevis_left,
        elem_b=elbow_hub,
        reason="Left clevis ear intentionally nests around the elbow hub as a revolute bearing seat.",
    )
    ctx.allow_overlap(
        yaw_link,
        elbow_frame,
        elem_a=yaw_clevis_right,
        elem_b=elbow_hub,
        reason="Right clevis ear intentionally nests around the elbow hub as a revolute bearing seat.",
    )

    ctx.check("base_frame_present", base_frame is not None, "base_frame part missing")
    ctx.check("yaw_link_present", yaw_link is not None, "yaw_link part missing")
    ctx.check("elbow_frame_present", elbow_frame is not None, "elbow_frame part missing")
    ctx.check("slider_carriage_present", slider_carriage is not None, "slider_carriage part missing")

    ctx.expect_contact(yaw_link, base_frame, elem_a=yaw_hub, elem_b=base_mount, name="yaw_hub_seated_on_base_mount")
    ctx.expect_overlap(yaw_link, base_frame, axes="xy", min_overlap=0.08, elem_a=yaw_hub, elem_b=base_mount, name="yaw_hub_covers_base_mount")

    ctx.expect_contact(elbow_frame, yaw_link, elem_a=elbow_hub, elem_b=yaw_clevis_left, name="elbow_hub_contacts_left_clevis")
    ctx.expect_contact(elbow_frame, yaw_link, elem_a=elbow_hub, elem_b=yaw_clevis_right, name="elbow_hub_contacts_right_clevis")
    ctx.expect_overlap(elbow_frame, yaw_link, axes="xz", min_overlap=0.05, elem_a=elbow_hub, elem_b=yaw_clevis_left, name="elbow_hub_nested_in_clevis_window")

    ctx.expect_contact(slider_carriage, elbow_frame, elem_a=slider_sleeve_left, elem_b=elbow_rail_left, name="left_slide_sleeve_on_left_rail")
    ctx.expect_contact(slider_carriage, elbow_frame, elem_a=slider_sleeve_right, elem_b=elbow_rail_right, name="right_slide_sleeve_on_right_rail")
    ctx.expect_overlap(slider_carriage, elbow_frame, axes="yz", min_overlap=0.02, elem_a=slider_sleeve_left, elem_b=elbow_rail_left, name="left_sleeve_tracks_left_rail")
    ctx.expect_overlap(slider_carriage, elbow_frame, axes="yz", min_overlap=0.02, elem_a=slider_sleeve_right, elem_b=elbow_rail_right, name="right_sleeve_tracks_right_rail")

    with ctx.pose({base_to_yaw: 0.0, yaw_to_elbow: 0.0, elbow_to_slider: 0.0}):
        ctx.expect_origin_gap(slider_carriage, elbow_frame, axis="x", min_gap=0.195, max_gap=0.205, name="retracted_slider_origin_position")
        ctx.expect_gap(slider_carriage, base_frame, axis="z", min_gap=0.11, name="retracted_slider_clears_base")
        ctx.expect_gap(elbow_frame, base_frame, axis="z", min_gap=0.015, name="elbow_frame_clears_base_at_rest")
    with ctx.pose({base_to_yaw: 0.0, yaw_to_elbow: 0.0, elbow_to_slider: 0.13}):
        ctx.expect_origin_gap(slider_carriage, elbow_frame, axis="x", min_gap=0.325, max_gap=0.335, name="extended_slider_origin_position")
        ctx.expect_gap(slider_carriage, base_frame, axis="z", min_gap=0.11, name="extended_slider_clears_base_in_neutral_axis")

    with ctx.pose({base_to_yaw: 0.9, yaw_to_elbow: 0.55, elbow_to_slider: 0.09}):
        ctx.expect_contact(slider_carriage, elbow_frame, elem_a=slider_sleeve_left, elem_b=elbow_rail_left, name="left_sleeve_contact_in_articulated_pose")
        ctx.expect_contact(slider_carriage, elbow_frame, elem_a=slider_sleeve_right, elem_b=elbow_rail_right, name="right_sleeve_contact_in_articulated_pose")
        ctx.expect_gap(slider_carriage, base_frame, axis="z", min_gap=0.005, name="extended_slider_clears_base")

    with ctx.pose({base_to_yaw: -0.8, yaw_to_elbow: -0.35, elbow_to_slider: 0.13}):
        ctx.expect_gap(slider_carriage, base_frame, axis="z", min_gap=0.015, name="low_pitch_slider_still_clears_base")
        ctx.expect_gap(slider_carriage, elbow_frame, axis="x", min_gap=0.095, positive_elem=slider_output_head, negative_elem=elbow_hub, name="output_head_projects_beyond_elbow_hub")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
