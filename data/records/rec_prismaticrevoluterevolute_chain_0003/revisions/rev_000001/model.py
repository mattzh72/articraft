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

BASE_LENGTH = 0.54
BASE_WIDTH = 0.26
BASE_PLATE_THICKNESS = 0.020
RAIL_CENTER_Y = 0.075
RAIL_LENGTH = 0.44
RAIL_PEDESTAL_WIDTH = 0.042
RAIL_PEDESTAL_HEIGHT = 0.024
GUIDE_RAIL_WIDTH = 0.024
GUIDE_RAIL_HEIGHT = 0.010
RAIL_TOP_Z = BASE_PLATE_THICKNESS + RAIL_PEDESTAL_HEIGHT + GUIDE_RAIL_HEIGHT

SLIDE_TRAVEL = 0.13


def _combine(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _z_cylinder(radius: float, height: float, base_xy: tuple[float, float], z0: float) -> cq.Workplane:
    return cq.Workplane("XY").center(*base_xy).circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _y_cylinder(
    radius: float,
    length: float,
    center_xz: tuple[float, float],
    center_y: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(*center_xz)
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y - 0.5 * length, 0.0))
    )


def _make_base_core() -> cq.Workplane:
    base_plate = _box((BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS), (0.0, 0.0, 0.5 * BASE_PLATE_THICKNESS))

    rail_pedestals = [
        _box(
            (RAIL_LENGTH, RAIL_PEDESTAL_WIDTH, RAIL_PEDESTAL_HEIGHT),
            (0.0, sign * RAIL_CENTER_Y, BASE_PLATE_THICKNESS + 0.5 * RAIL_PEDESTAL_HEIGHT),
        )
        for sign in (-1.0, 1.0)
    ]
    guide_rails = [
        _box(
            (RAIL_LENGTH - 0.01, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT),
            (
                0.0,
                sign * RAIL_CENTER_Y,
                BASE_PLATE_THICKNESS + RAIL_PEDESTAL_HEIGHT + 0.5 * GUIDE_RAIL_HEIGHT,
            ),
        )
        for sign in (-1.0, 1.0)
    ]

    end_blocks = [
        _box((0.060, 0.110, 0.040), (sign * 0.220, 0.0, BASE_PLATE_THICKNESS + 0.020))
        for sign in (-1.0, 1.0)
    ]
    center_tunnel = _box((0.360, 0.058, 0.026), (0.0, 0.0, BASE_PLATE_THICKNESS + 0.013))
    side_stringers = [
        _box((0.330, 0.020, 0.026), (0.0, sign * 0.105, BASE_PLATE_THICKNESS + 0.013))
        for sign in (-1.0, 1.0)
    ]

    core = _combine([base_plate, center_tunnel, *rail_pedestals, *end_blocks, *side_stringers])
    relief = _box((0.200, 0.090, 0.012), (0.0, 0.0, 0.010))
    center_slot = _box((0.280, 0.030, 0.020), (0.0, 0.0, BASE_PLATE_THICKNESS + 0.018))
    return core.cut(relief).cut(center_slot)


def _make_base_hardware() -> cq.Workplane:
    cap_heads: list[cq.Workplane] = []
    for rail_y in (-(RAIL_CENTER_Y + 0.030), RAIL_CENTER_Y + 0.030):
        for x in (-0.145, -0.115, 0.115, 0.145):
            cap_heads.append(_z_cylinder(0.007, 0.004, (x, rail_y), RAIL_TOP_Z))
    for x in (-0.220, 0.220):
        for y in (-0.030, 0.030):
            cap_heads.append(_z_cylinder(0.008, 0.005, (x, y), BASE_PLATE_THICKNESS + 0.040))
    return _combine(cap_heads)


def _make_guide_rails_visual() -> cq.Workplane:
    rails = [
        _box(
            (RAIL_LENGTH - 0.01, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT),
            (
                0.0,
                sign * RAIL_CENTER_Y,
                BASE_PLATE_THICKNESS + RAIL_PEDESTAL_HEIGHT + 0.5 * GUIDE_RAIL_HEIGHT,
            ),
        )
        for sign in (-1.0, 1.0)
    ]
    return _combine(rails)


def _make_carriage_core() -> cq.Workplane:
    deck = _box((0.170, 0.160, 0.022), (0.0, 0.0, 0.023))
    bridge_ribs = [
        _box((0.110, 0.020, 0.020), (0.0, sign * 0.045, 0.022))
        for sign in (-1.0, 1.0)
    ]
    turret_drum = _z_cylinder(0.038, 0.040, (0.0, 0.0), 0.034)
    upper_boss = _z_cylinder(0.025, 0.012, (0.0, 0.0), 0.062)
    web_front = _box((0.032, 0.072, 0.024), (0.030, 0.0, 0.046))
    web_rear = _box((0.028, 0.064, 0.022), (-0.030, 0.0, 0.045))
    return _combine([deck, *bridge_ribs, turret_drum, upper_boss, web_front, web_rear])


def _make_carriage_guide_blocks() -> cq.Workplane:
    blocks = []
    for sign in (-1.0, 1.0):
        for x in (-0.075, 0.075):
            blocks.append(_box((0.060, 0.030, 0.012), (x, sign * RAIL_CENTER_Y, 0.006)))
    return _combine(blocks)


def _make_turret_cap() -> cq.Workplane:
    cap_ring = _z_cylinder(0.046, 0.008, (0.0, 0.0), 0.074)
    cap_land = _box((0.078, 0.052, 0.006), (0.0, 0.0, 0.077))
    return _combine([cap_ring, cap_land])


def _make_carriage_covers() -> cq.Workplane:
    covers = [
        _box((0.056, 0.004, 0.032), (0.0, sign * 0.032, 0.053))
        for sign in (-1.0, 1.0)
    ]
    cover_bolts = []
    for sign in (-1.0, 1.0):
        for x in (-0.018, 0.018):
            cover_bolts.append(_z_cylinder(0.004, 0.004, (x, sign * 0.032), 0.064))
    return _combine([*covers, *cover_bolts])


def _make_shoulder_core() -> cq.Workplane:
    upright = _box((0.036, 0.060, 0.052), (0.0, 0.0, 0.036))
    rear_collar = _box((0.024, 0.050, 0.018), (-0.012, 0.0, 0.021))
    hub_flange = _z_cylinder(0.022, 0.012, (0.0, 0.0), 0.010)
    return _combine([upright, rear_collar, hub_flange])


def _make_turntable_disc() -> cq.Workplane:
    return _z_cylinder(0.044, 0.010, (0.0, 0.0), 0.0)


def _make_fork_stage() -> cq.Workplane:
    fork_plates = [
        _box((0.082, 0.012, 0.052), (0.041, sign * 0.026, 0.050))
        for sign in (-1.0, 1.0)
    ]
    pivot_bosses = [
        _y_cylinder(0.016, 0.014, (0.074, 0.055), sign * 0.026)
        for sign in (-1.0, 1.0)
    ]
    upper_bridge = _box((0.020, 0.040, 0.018), (0.012, 0.0, 0.072))
    return _combine([*fork_plates, *pivot_bosses, upper_bridge])


def _make_shoulder_hardware() -> cq.Workplane:
    heads = []
    for x, y in ((0.026, 0.026), (0.026, -0.026), (-0.026, 0.026), (-0.026, -0.026)):
        heads.append(_z_cylinder(0.004, 0.004, (x, y), 0.010))
    for y in (-0.033, 0.033):
        heads.append(_y_cylinder(0.006, 0.006, (0.074, 0.055), y))
    return _combine(heads)


def _make_link_core() -> cq.Workplane:
    root_cheek = _box((0.026, 0.028, 0.038), (0.031, 0.0, 0.0))
    beam = _box((0.108, 0.028, 0.038), (0.094, 0.0, 0.0))
    window = _box((0.054, 0.016, 0.018), (0.094, 0.0, 0.0))
    end_block = _box((0.030, 0.042, 0.052), (0.154, 0.0, 0.0))
    lower_rib = _box((0.048, 0.022, 0.014), (0.118, 0.0, -0.021))
    top_rib = _box((0.034, 0.018, 0.012), (0.068, 0.0, 0.025))
    body = _combine([root_cheek, beam, end_block, lower_rib, top_rib])
    return body.cut(window)


def _make_pivot_barrel() -> cq.Workplane:
    return _y_cylinder(0.014, 0.038, (0.0, 0.0), 0.0)


def _make_link_hardware() -> cq.Workplane:
    end_covers = [
        _box((0.020, 0.004, 0.034), (0.154, sign * 0.023, 0.0))
        for sign in (-1.0, 1.0)
    ]
    end_bolts = []
    for sign in (-1.0, 1.0):
        for z in (-0.014, 0.014):
            end_bolts.append(_box((0.008, 0.004, 0.008), (0.154, sign * 0.023, z)))
    return _combine([*end_covers, *end_bolts])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_prr_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.44, 0.47, 0.50, 1.0))
    steel_light = model.material("steel_light", rgba=(0.62, 0.65, 0.68, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_make_base_core(), "base_frame_core.obj", assets=ASSETS),
        material=steel_mid,
        name="base_core",
    )
    base.visual(
        mesh_from_cadquery(_make_guide_rails_visual(), "base_frame_rails.obj", assets=ASSETS),
        material=steel_light,
        name="guide_rails",
    )
    base.visual(
        mesh_from_cadquery(_make_base_hardware(), "base_frame_hardware.obj", assets=ASSETS),
        material=fastener_black,
        name="base_hardware",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.084)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    carriage = model.part("slide_carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_core(), "slide_carriage_core.obj", assets=ASSETS),
        material=steel_mid,
        name="carriage_core",
    )
    carriage.visual(
        mesh_from_cadquery(_make_carriage_guide_blocks(), "slide_carriage_guides.obj", assets=ASSETS),
        material=steel_light,
        name="guide_blocks",
    )
    carriage.visual(
        mesh_from_cadquery(_make_turret_cap(), "slide_carriage_turret_cap.obj", assets=ASSETS),
        material=steel_dark,
        name="turret_cap",
    )
    carriage.visual(
        mesh_from_cadquery(_make_carriage_covers(), "slide_carriage_covers.obj", assets=ASSETS),
        material=cover_gray,
        name="access_covers",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.170, 0.160, 0.082)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    shoulder = model.part("turret_stage")
    shoulder.visual(
        mesh_from_cadquery(_make_shoulder_core(), "turret_stage_core.obj", assets=ASSETS),
        material=steel_dark,
        name="shoulder_core",
    )
    shoulder.visual(
        mesh_from_cadquery(_make_turntable_disc(), "turret_stage_turntable.obj", assets=ASSETS),
        material=steel_light,
        name="turntable_disc",
    )
    shoulder.visual(
        mesh_from_cadquery(_make_fork_stage(), "turret_stage_fork.obj", assets=ASSETS),
        material=steel_mid,
        name="fork_stage",
    )
    shoulder.visual(
        mesh_from_cadquery(_make_shoulder_hardware(), "turret_stage_hardware.obj", assets=ASSETS),
        material=fastener_black,
        name="turret_hardware",
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.082)),
        mass=2.4,
        origin=Origin(xyz=(0.030, 0.0, 0.041)),
    )

    link = model.part("link_stage")
    link.visual(
        mesh_from_cadquery(_make_link_core(), "link_stage_core.obj", assets=ASSETS),
        material=steel_mid,
        name="link_core",
    )
    link.visual(
        mesh_from_cadquery(_make_pivot_barrel(), "link_stage_pivot.obj", assets=ASSETS),
        material=steel_light,
        name="pivot_barrel",
    )
    link.visual(
        mesh_from_cadquery(_make_link_hardware(), "link_stage_hardware.obj", assets=ASSETS),
        material=cover_gray,
        name="link_covers",
    )
    link.inertial = Inertial.from_geometry(
        Box((0.180, 0.060, 0.060)),
        mass=1.9,
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=-SLIDE_TRAVEL, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "turret_yaw",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-1.5, upper=1.5),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=link,
        origin=Origin(xyz=(0.074, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.6, lower=-0.35, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    carriage = object_model.get_part("slide_carriage")
    shoulder = object_model.get_part("turret_stage")
    link = object_model.get_part("link_stage")

    base_slide = object_model.get_articulation("base_slide")
    turret_yaw = object_model.get_articulation("turret_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")

    guide_rails = base.get_visual("guide_rails")
    guide_blocks = carriage.get_visual("guide_blocks")
    turret_cap = carriage.get_visual("turret_cap")
    turntable_disc = shoulder.get_visual("turntable_disc")
    fork_stage = shoulder.get_visual("fork_stage")
    pivot_barrel = link.get_visual("pivot_barrel")
    link_core = link.get_visual("link_core")

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
        "all_parts_present",
        all(part is not None for part in (base, carriage, shoulder, link)),
        "expected base, slide, turret, and link parts to exist",
    )
    ctx.check(
        "all_articulations_present",
        all(joint is not None for joint in (base_slide, turret_yaw, elbow_pitch)),
        "expected prismatic and two serial revolute articulations to exist",
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a=guide_blocks,
        elem_b=guide_rails,
        name="carriage_guide_blocks_seat_on_rails",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        min_overlap=0.11,
        elem_a=guide_blocks,
        elem_b=guide_rails,
        name="carriage_retains_long_rail_engagement",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="xy",
        margin=0.03,
        name="carriage_stays_within_base_envelope",
    )

    ctx.expect_contact(
        shoulder,
        carriage,
        elem_a=turntable_disc,
        elem_b=turret_cap,
        name="turntable_disc_is_seated_on_turret_cap",
    )
    ctx.expect_origin_distance(
        shoulder,
        carriage,
        axes="xy",
        max_dist=0.001,
        name="turret_axis_is_centered_over_carriage",
    )

    ctx.expect_contact(
        link,
        shoulder,
        elem_a=pivot_barrel,
        elem_b=fork_stage,
        name="pitch_stage_barrel_is_supported_by_fork_bosses",
    )
    ctx.expect_overlap(
        link,
        shoulder,
        axes="z",
        min_overlap=0.020,
        elem_a=pivot_barrel,
        elem_b=fork_stage,
        name="pitch_stage_has_vertical_boss_engagement",
    )
    ctx.expect_gap(
        link,
        carriage,
        axis="z",
        min_gap=0.020,
        positive_elem=link_core,
        negative_elem=turret_cap,
        name="link_clears_turret_cap_in_rest_pose",
    )

    with ctx.pose({base_slide: -0.11}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a=guide_blocks,
            elem_b=guide_rails,
            name="carriage_keeps_rail_contact_at_negative_slide",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.11,
            elem_a=guide_blocks,
            elem_b=guide_rails,
            name="rail_engagement_persists_at_negative_slide",
        )

    with ctx.pose({base_slide: 0.11}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a=guide_blocks,
            elem_b=guide_rails,
            name="carriage_keeps_rail_contact_at_positive_slide",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.11,
            elem_a=guide_blocks,
            elem_b=guide_rails,
            name="rail_engagement_persists_at_positive_slide",
        )

    with ctx.pose({turret_yaw: 1.1}):
        ctx.expect_contact(
            shoulder,
            carriage,
            elem_a=turntable_disc,
            elem_b=turret_cap,
            name="yaw_stage_remains_seated_at_positive_rotation",
        )

    with ctx.pose({turret_yaw: -1.1}):
        ctx.expect_contact(
            shoulder,
            carriage,
            elem_a=turntable_disc,
            elem_b=turret_cap,
            name="yaw_stage_remains_seated_at_negative_rotation",
        )

    with ctx.pose({turret_yaw: 0.7, elbow_pitch: 0.9, base_slide: 0.08}):
        ctx.expect_contact(
            link,
            shoulder,
            elem_a=pivot_barrel,
            elem_b=fork_stage,
            name="pitch_barrel_stays_supported_in_raised_pose",
        )

    with ctx.pose({turret_yaw: -0.6, elbow_pitch: -0.25, base_slide: -0.08}):
        ctx.expect_contact(
            link,
            shoulder,
            elem_a=pivot_barrel,
            elem_b=fork_stage,
            name="pitch_barrel_stays_supported_in_lowered_pose",
        )
        ctx.expect_gap(
            link,
            carriage,
            axis="z",
            min_gap=0.008,
            positive_elem=link_core,
            negative_elem=turret_cap,
            name="link_still_clears_turret_when_lowered",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=16, ignore_adjacent=False, ignore_fixed=True)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
