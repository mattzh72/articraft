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

RAIL_LENGTH = 0.92
RAIL_SPACING = 0.36
SUPPORT_WIDTH = 0.072
SUPPORT_HEIGHT = 0.048
SUPPORT_CLEARANCE = 0.006
SUPPORT_TOP_Z = SUPPORT_CLEARANCE + SUPPORT_HEIGHT
RAIL_PAD_WIDTH = 0.036
RAIL_PAD_HEIGHT = 0.010
RAIL_WIDTH = 0.024
RAIL_HEIGHT = 0.018
RAIL_CENTER_Z = SUPPORT_TOP_Z + RAIL_PAD_HEIGHT + RAIL_HEIGHT / 2.0
RAIL_TOP_Z = SUPPORT_TOP_Z + RAIL_PAD_HEIGHT + RAIL_HEIGHT

BRIDGE_BLOCK_X = 0.052
BRIDGE_BLOCK_Y = 0.036
BRIDGE_BLOCK_Z = 0.030
BRIDGE_BLOCK_X_OFF = 0.110

GUIDE_LENGTH = 0.280
GUIDE_X = 0.040
GUIDE_Z = 0.018
GUIDE_CENTER_Z = 0.136
GUIDE_TOP_Z = GUIDE_CENTER_Z + GUIDE_Z / 2.0

TRUCK_BEARING_X = 0.056
TRUCK_BEARING_Y = 0.036
TRUCK_BEARING_Z = 0.026
TRUCK_BEARING_Y_OFF = 0.045

BRIDGE_TRAVEL = 0.290
TRUCK_TRAVEL = 0.075


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _base_frame_shape() -> cq.Workplane:
    beam_y = RAIL_SPACING / 2.0
    beam_z = SUPPORT_CLEARANCE + SUPPORT_HEIGHT / 2.0
    pad_z = SUPPORT_TOP_Z + RAIL_PAD_HEIGHT / 2.0

    shape = _box((0.98, SUPPORT_WIDTH, SUPPORT_HEIGHT), (0.0, beam_y, beam_z))
    shape = shape.union(_box((0.98, SUPPORT_WIDTH, SUPPORT_HEIGHT), (0.0, -beam_y, beam_z)))
    shape = shape.union(_box((0.090, RAIL_SPACING + 0.100, 0.042), (0.0, 0.0, SUPPORT_CLEARANCE + 0.021)))
    shape = shape.union(_box((0.085, RAIL_SPACING + 0.150, 0.040), (0.420, 0.0, SUPPORT_CLEARANCE + 0.020)))
    shape = shape.union(_box((0.085, RAIL_SPACING + 0.150, 0.040), (-0.420, 0.0, SUPPORT_CLEARANCE + 0.020)))
    shape = shape.union(_box((RAIL_LENGTH + 0.050, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT), (0.0, beam_y, pad_z)))
    shape = shape.union(_box((RAIL_LENGTH + 0.050, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT), (0.0, -beam_y, pad_z)))

    for x in (-0.390, 0.390):
        for y in (-beam_y, beam_y):
            shape = shape.union(_box((0.090, 0.058, SUPPORT_CLEARANCE), (x, y, SUPPORT_CLEARANCE / 2.0)))

    window = _box((0.038, RAIL_SPACING + 0.050, 0.018), (0.0, 0.0, SUPPORT_CLEARANCE + 0.024))
    shape = shape.cut(window)

    end_window = _box((0.036, RAIL_SPACING + 0.080, 0.016), (0.420, 0.0, SUPPORT_CLEARANCE + 0.022))
    shape = shape.cut(end_window)
    shape = shape.cut(end_window.translate((-0.840, 0.0, 0.0)))
    return shape


def _rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
    hole_points = [(x, 0.0) for x in (-0.360, -0.180, 0.0, 0.180, 0.360)]
    rail = (
        rail.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(hole_points)
        .cboreHole(0.006, 0.010, 0.006)
    )
    underside_relief = _box((RAIL_LENGTH - 0.080, RAIL_WIDTH - 0.010, 0.006), (0.0, 0.0, -0.006))
    rail = rail.cut(underside_relief)
    return rail


def _bridge_core_shape() -> cq.Workplane:
    left_plate = _box((0.300, 0.016, 0.105), (0.0, RAIL_SPACING / 2.0, 0.082))
    left_plate = left_plate.cut(_box((0.070, 0.020, 0.040), (-0.085, RAIL_SPACING / 2.0, 0.073)))
    left_plate = left_plate.cut(_box((0.070, 0.020, 0.040), (0.085, RAIL_SPACING / 2.0, 0.073)))

    right_plate = _box((0.300, 0.016, 0.105), (0.0, -RAIL_SPACING / 2.0, 0.082))
    right_plate = right_plate.cut(_box((0.070, 0.020, 0.040), (-0.085, -RAIL_SPACING / 2.0, 0.073)))
    right_plate = right_plate.cut(_box((0.070, 0.020, 0.040), (0.085, -RAIL_SPACING / 2.0, 0.073)))

    cross_beam = _box((0.170, RAIL_SPACING - 0.060, 0.046), (0.0, 0.0, 0.105))
    top_cap = _box((0.105, RAIL_SPACING - 0.120, 0.028), (0.0, 0.0, 0.146))
    guide_saddle = _box((0.090, GUIDE_LENGTH - 0.040, 0.014), (0.0, 0.0, GUIDE_CENTER_Z - 0.017))

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline([(-0.058, 0.030), (0.058, 0.030), (0.018, 0.090), (-0.018, 0.090)])
        .close()
        .extrude(0.014)
    )
    left_gusset = gusset_profile.translate((0.0, 0.100, 0.0))
    right_gusset = gusset_profile.translate((0.0, -0.114, 0.0))

    bridge = left_plate.union(right_plate)
    bridge = bridge.union(cross_beam)
    bridge = bridge.union(top_cap)
    bridge = bridge.union(guide_saddle)
    bridge = bridge.union(left_gusset)
    bridge = bridge.union(right_gusset)

    cover_pocket = _box((0.075, 0.090, 0.010), (0.0, 0.0, 0.150))
    bridge = bridge.cut(cover_pocket)
    bridge = bridge.cut(_box((0.090, 0.132, 0.036), (0.0, -0.075, 0.154)))
    bridge = bridge.cut(_box((0.090, 0.132, 0.036), (0.0, 0.075, 0.154)))
    return bridge


def _truck_body_shape() -> cq.Workplane:
    plate = _box((0.160, 0.110, 0.022), (0.0, 0.0, 0.037))
    saddle = _box((0.100, 0.135, 0.030), (0.0, 0.0, 0.031))
    tower = _box((0.072, 0.072, 0.050), (0.0, 0.0, 0.073))
    ear_left = _box((0.055, 0.016, 0.034), (0.0, 0.055, 0.054))
    ear_right = _box((0.055, 0.016, 0.034), (0.0, -0.055, 0.054))

    truck = plate.union(saddle).union(tower).union(ear_left).union(ear_right)
    truck = truck.cut(_box((0.090, 0.050, 0.012), (0.0, 0.0, 0.042)))
    truck = truck.cut(_box((0.050, 0.040, 0.018), (0.0, 0.0, 0.072)))
    return truck


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry_axis", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.33, 0.37, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    polymer = model.material("polymer", rgba=(0.12, 0.13, 0.14, 1.0))
    seal_green = model.material("seal_green", rgba=(0.16, 0.31, 0.19, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame.obj", assets=ASSETS),
        material=painted_steel,
        name="frame_structure",
    )
    base.visual(
        Box((RAIL_LENGTH + 0.050, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_SPACING / 2.0, SUPPORT_TOP_Z + RAIL_PAD_HEIGHT / 2.0)),
        material=machined_aluminum,
        name="left_rail_pad",
    )
    base.visual(
        Box((RAIL_LENGTH + 0.050, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_SPACING / 2.0, SUPPORT_TOP_Z + RAIL_PAD_HEIGHT / 2.0)),
        material=machined_aluminum,
        name="right_rail_pad",
    )
    for name, x, y in (
        ("left_bridge_stop_neg", -0.470, RAIL_SPACING / 2.0),
        ("left_bridge_stop_pos", 0.470, RAIL_SPACING / 2.0),
        ("right_bridge_stop_neg", -0.470, -RAIL_SPACING / 2.0),
        ("right_bridge_stop_pos", 0.470, -RAIL_SPACING / 2.0),
    ):
        base.visual(
            Box((0.016, 0.032, 0.018)),
            origin=Origin(xyz=(x, y, SUPPORT_TOP_Z + RAIL_PAD_HEIGHT + 0.009)),
            material=black_oxide,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.98, RAIL_SPACING + 0.150, SUPPORT_TOP_Z)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_Z / 2.0)),
    )

    left_rail = model.part("left_rail")
    left_rail.visual(
        mesh_from_cadquery(_rail_shape(), "left_rail.obj", assets=ASSETS),
        material=rail_steel,
        name="rail_body",
    )
    left_rail.inertial = Inertial.from_geometry(Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)), mass=2.8)

    right_rail = model.part("right_rail")
    right_rail.visual(
        mesh_from_cadquery(_rail_shape(), "right_rail.obj", assets=ASSETS),
        material=rail_steel,
        name="rail_body",
    )
    right_rail.inertial = Inertial.from_geometry(Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)), mass=2.8)

    bridge = model.part("bridge_carriage")
    bridge.visual(
        mesh_from_cadquery(_bridge_core_shape(), "bridge_carriage.obj", assets=ASSETS),
        material=machined_aluminum,
        name="bridge_core",
    )
    for name, x, y in (
        ("left_front_bearing_block", -BRIDGE_BLOCK_X_OFF, RAIL_SPACING / 2.0),
        ("left_rear_bearing_block", BRIDGE_BLOCK_X_OFF, RAIL_SPACING / 2.0),
        ("right_front_bearing_block", -BRIDGE_BLOCK_X_OFF, -RAIL_SPACING / 2.0),
        ("right_rear_bearing_block", BRIDGE_BLOCK_X_OFF, -RAIL_SPACING / 2.0),
    ):
        bridge.visual(
            Box((BRIDGE_BLOCK_X, BRIDGE_BLOCK_Y, BRIDGE_BLOCK_Z)),
            origin=Origin(xyz=(x, y, BRIDGE_BLOCK_Z / 2.0)),
            material=black_oxide,
            name=name,
        )
    for name, x, y in (
        ("left_front_wiper", -BRIDGE_BLOCK_X_OFF - 0.024, RAIL_SPACING / 2.0),
        ("left_rear_wiper", BRIDGE_BLOCK_X_OFF + 0.024, RAIL_SPACING / 2.0),
        ("right_front_wiper", -BRIDGE_BLOCK_X_OFF - 0.024, -RAIL_SPACING / 2.0),
        ("right_rear_wiper", BRIDGE_BLOCK_X_OFF + 0.024, -RAIL_SPACING / 2.0),
    ):
        bridge.visual(
            Box((0.004, 0.034, 0.012)),
            origin=Origin(xyz=(x, y, 0.010)),
            material=seal_green,
            name=name,
        )
    bridge.visual(
        Box((GUIDE_X, GUIDE_LENGTH, GUIDE_Z)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        material=rail_steel,
        name="center_guide",
    )
    bridge.visual(
        Box((0.140, 0.126, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.161)),
        material=painted_steel,
        name="top_access_cover",
    )
    for idx, x in enumerate((-0.045, -0.015, 0.015, 0.045), start=1):
        bridge.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.165)),
            material=black_oxide,
            name=f"cover_screw_{idx}",
        )
    bridge.visual(
        Box((0.200, 0.003, 0.072)),
        origin=Origin(xyz=(0.0, RAIL_SPACING / 2.0 + 0.0095, 0.082)),
        material=painted_steel,
        name="left_access_cover",
    )
    bridge.visual(
        Box((0.200, 0.003, 0.072)),
        origin=Origin(xyz=(0.0, -RAIL_SPACING / 2.0 - 0.0095, 0.082)),
        material=painted_steel,
        name="right_access_cover",
    )
    for name, x in (("tray_bracket_front", -0.090), ("tray_bracket_rear", 0.090)):
        bridge.visual(
            Box((0.034, 0.070, 0.008)),
            origin=Origin(xyz=(x, RAIL_SPACING / 2.0 + 0.038, 0.134)),
            material=painted_steel,
            name=f"{name}_arm",
        )
        bridge.visual(
            Box((0.034, 0.008, 0.050)),
            origin=Origin(xyz=(x, RAIL_SPACING / 2.0 + 0.072, 0.109)),
            material=painted_steel,
            name=f"{name}_leg",
        )
    bridge.visual(
        Box((0.030, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.182, 0.145)),
        material=black_oxide,
        name="guide_stop_pos",
    )
    bridge.visual(
        Box((0.030, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.182, 0.145)),
        material=black_oxide,
        name="guide_stop_neg",
    )
    bridge.visual(
        Box((0.026, 0.022, 0.018)),
        origin=Origin(xyz=(0.163, RAIL_SPACING / 2.0, 0.086)),
        material=polymer,
        name="left_bumper_pad",
    )
    bridge.visual(
        Box((0.026, 0.022, 0.018)),
        origin=Origin(xyz=(-0.163, RAIL_SPACING / 2.0, 0.086)),
        material=polymer,
        name="right_bumper_pad",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.340, RAIL_SPACING + 0.120, 0.170)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    truck = model.part("center_truck")
    truck.visual(
        mesh_from_cadquery(_truck_body_shape(), "center_truck.obj", assets=ASSETS),
        material=machined_aluminum,
        name="truck_body",
    )
    truck.visual(
        Box((TRUCK_BEARING_X, TRUCK_BEARING_Y, TRUCK_BEARING_Z)),
        origin=Origin(xyz=(0.0, -TRUCK_BEARING_Y_OFF, TRUCK_BEARING_Z / 2.0)),
        material=black_oxide,
        name="truck_bearing_neg",
    )
    truck.visual(
        Box((TRUCK_BEARING_X, TRUCK_BEARING_Y, TRUCK_BEARING_Z)),
        origin=Origin(xyz=(0.0, TRUCK_BEARING_Y_OFF, TRUCK_BEARING_Z / 2.0)),
        material=black_oxide,
        name="truck_bearing_pos",
    )
    truck.visual(
        Box((0.052, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -TRUCK_BEARING_Y_OFF - 0.020, 0.010)),
        material=seal_green,
        name="truck_wiper_neg",
    )
    truck.visual(
        Box((0.052, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, TRUCK_BEARING_Y_OFF + 0.020, 0.010)),
        material=seal_green,
        name="truck_wiper_pos",
    )
    truck.visual(
        Box((0.120, 0.082, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=painted_steel,
        name="truck_cover",
    )
    for idx, x in enumerate((-0.035, -0.012, 0.012, 0.035), start=1):
        truck.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.093)),
            material=black_oxide,
            name=f"truck_cover_screw_{idx}",
        )
    truck.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0775, 0.044)),
        material=polymer,
        name="truck_stop_pad_pos",
    )
    truck.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.0775, 0.044)),
        material=polymer,
        name="truck_stop_pad_neg",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.160, 0.135, 0.094)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    model.articulation(
        "base_to_left_rail",
        ArticulationType.FIXED,
        parent=base,
        child=left_rail,
        origin=Origin(xyz=(0.0, RAIL_SPACING / 2.0, RAIL_CENTER_Z)),
    )
    model.articulation(
        "base_to_right_rail",
        ArticulationType.FIXED,
        parent=base,
        child=right_rail,
        origin=Origin(xyz=(0.0, -RAIL_SPACING / 2.0, RAIL_CENTER_Z)),
    )
    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.8, lower=-BRIDGE_TRAVEL, upper=BRIDGE_TRAVEL),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.6, lower=-TRUCK_TRAVEL, upper=TRUCK_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    left_rail = object_model.get_part("left_rail")
    right_rail = object_model.get_part("right_rail")
    bridge = object_model.get_part("bridge_carriage")
    truck = object_model.get_part("center_truck")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    left_rail_body = left_rail.get_visual("rail_body")
    right_rail_body = right_rail.get_visual("rail_body")
    left_pad = base.get_visual("left_rail_pad")
    right_pad = base.get_visual("right_rail_pad")
    left_front_block = bridge.get_visual("left_front_bearing_block")
    left_rear_block = bridge.get_visual("left_rear_bearing_block")
    right_front_block = bridge.get_visual("right_front_bearing_block")
    right_rear_block = bridge.get_visual("right_rear_bearing_block")
    center_guide = bridge.get_visual("center_guide")
    truck_bearing_neg = truck.get_visual("truck_bearing_neg")
    truck_bearing_pos = truck.get_visual("truck_bearing_pos")
    guide_stop_neg = bridge.get_visual("guide_stop_neg")
    guide_stop_pos = bridge.get_visual("guide_stop_pos")
    truck_stop_pad_neg = truck.get_visual("truck_stop_pad_neg")
    truck_stop_pad_pos = truck.get_visual("truck_stop_pad_pos")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="articulation_clearance_sweep")

    ctx.check(
        "part_inventory",
        all(part is not None for part in (base, left_rail, right_rail, bridge, truck)),
        "expected base frame, two rails, bridge carriage, and center truck",
    )

    ctx.expect_gap(
        left_rail,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=left_rail_body,
        negative_elem=left_pad,
        name="left_rail_seats_on_left_pad",
    )
    ctx.expect_gap(
        right_rail,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_rail_body,
        negative_elem=right_pad,
        name="right_rail_seats_on_right_pad",
    )
    ctx.expect_overlap(
        left_rail,
        base,
        axes="x",
        min_overlap=0.90,
        elem_a=left_rail_body,
        elem_b=left_pad,
        name="left_rail_spans_support_pad",
    )
    ctx.expect_overlap(
        right_rail,
        base,
        axes="x",
        min_overlap=0.90,
        elem_a=right_rail_body,
        elem_b=right_pad,
        name="right_rail_spans_support_pad",
    )

    for name, block, rail in (
        ("left_front_block_seats_on_left_rail", left_front_block, left_rail_body),
        ("left_rear_block_seats_on_left_rail", left_rear_block, left_rail_body),
        ("right_front_block_seats_on_right_rail", right_front_block, right_rail_body),
        ("right_rear_block_seats_on_right_rail", right_rear_block, right_rail_body),
    ):
        rail_owner = left_rail if "left" in name else right_rail
        ctx.expect_gap(
            bridge,
            rail_owner,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=block,
            negative_elem=rail,
            name=name,
        )
        ctx.expect_overlap(
            bridge,
            rail_owner,
            axes="x",
            min_overlap=0.040,
            elem_a=block,
            elem_b=rail,
            name=f"{name}_footprint",
        )

    ctx.expect_gap(
        truck,
        bridge,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=truck_bearing_neg,
        negative_elem=center_guide,
        name="truck_negative_bearing_seats_on_center_guide",
    )
    ctx.expect_gap(
        truck,
        bridge,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=truck_bearing_pos,
        negative_elem=center_guide,
        name="truck_positive_bearing_seats_on_center_guide",
    )
    ctx.expect_overlap(
        truck,
        bridge,
        axes="y",
        min_overlap=0.030,
        elem_a=truck_bearing_neg,
        elem_b=center_guide,
        name="truck_negative_bearing_has_guide_engagement",
    )
    ctx.expect_overlap(
        truck,
        bridge,
        axes="y",
        min_overlap=0.030,
        elem_a=truck_bearing_pos,
        elem_b=center_guide,
        name="truck_positive_bearing_has_guide_engagement",
    )

    with ctx.pose({bridge_slide: -BRIDGE_TRAVEL}):
        ctx.expect_within(
            bridge,
            left_rail,
            axes="x",
            margin=0.0005,
            inner_elem=left_front_block,
            outer_elem=left_rail_body,
            name="left_front_block_stays_within_left_rail_at_negative_travel",
        )
        ctx.expect_within(
            bridge,
            right_rail,
            axes="x",
            margin=0.0005,
            inner_elem=right_front_block,
            outer_elem=right_rail_body,
            name="right_front_block_stays_within_right_rail_at_negative_travel",
        )

    with ctx.pose({bridge_slide: BRIDGE_TRAVEL}):
        ctx.expect_within(
            bridge,
            left_rail,
            axes="x",
            margin=0.0005,
            inner_elem=left_rear_block,
            outer_elem=left_rail_body,
            name="left_rear_block_stays_within_left_rail_at_positive_travel",
        )
        ctx.expect_within(
            bridge,
            right_rail,
            axes="x",
            margin=0.0005,
            inner_elem=right_rear_block,
            outer_elem=right_rail_body,
            name="right_rear_block_stays_within_right_rail_at_positive_travel",
        )

    with ctx.pose({truck_slide: -TRUCK_TRAVEL}):
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            margin=0.0005,
            inner_elem=truck_bearing_neg,
            outer_elem=center_guide,
            name="negative_truck_bearing_remains_on_guide_at_negative_travel",
        )
        ctx.expect_gap(
            truck,
            bridge,
            axis="y",
            min_gap=0.010,
            max_gap=0.040,
            positive_elem=truck_stop_pad_neg,
            negative_elem=guide_stop_neg,
            name="negative_truck_stop_pad_clears_negative_guide_stop",
        )

    with ctx.pose({truck_slide: TRUCK_TRAVEL}):
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            margin=0.0005,
            inner_elem=truck_bearing_pos,
            outer_elem=center_guide,
            name="positive_truck_bearing_remains_on_guide_at_positive_travel",
        )
        ctx.expect_gap(
            bridge,
            truck,
            axis="y",
            min_gap=0.010,
            max_gap=0.040,
            positive_elem=guide_stop_pos,
            negative_elem=truck_stop_pad_pos,
            name="positive_truck_stop_pad_clears_positive_guide_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
