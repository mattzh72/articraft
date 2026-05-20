from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_RADIUS = 0.210
BASE_BODY_RADIUS = 0.176
BASE_TOP_RADIUS = 0.126
BASE_FOOT_THICKNESS = 0.018
BASE_BODY_HEIGHT = 0.052
BASE_TOP_HEIGHT = 0.020
BASE_HEIGHT = BASE_FOOT_THICKNESS + BASE_BODY_HEIGHT + BASE_TOP_HEIGHT

TURN_PLATE_RADIUS = 0.152
TURN_PLATE_THICKNESS = 0.022
TURN_PLATE_LIP_THICKNESS = 0.006
TURN_PLATE_GAP = 0.0025
TURN_PLATE_STACK_HEIGHT = TURN_PLATE_GAP + TURN_PLATE_THICKNESS + TURN_PLATE_LIP_THICKNESS
ROTARY_HUB_RADIUS = 0.054
ROTARY_HUB_HEIGHT = 0.050

MAST_WIDTH = 0.160
MAST_HEIGHT = 0.560
MAST_BACK_DEPTH = 0.018
MAST_SIDE_DEPTH = 0.060
MAST_SIDE_WALL = 0.016
MAST_FRONT_LIP_DEPTH = 0.012
MAST_FRONT_LIP_WIDTH = 0.032
MAST_TOP_CAP_HEIGHT = 0.024
MAST_BASE_BLOCK_DEPTH = 0.082
MAST_BASE_BLOCK_WIDTH = 0.110
MAST_BASE_BLOCK_HEIGHT = 0.050

RAIL_Y = 0.043
RAIL_X = 0.040
RAIL_RADIUS = 0.010
LOWER_BLOCK_Z = 0.115
UPPER_BLOCK_Z = 0.572
RAIL_BLOCK_DEPTH = 0.044
RAIL_BLOCK_WIDTH = 0.134
RAIL_BLOCK_HEIGHT = 0.030
RAIL_LENGTH = UPPER_BLOCK_Z - LOWER_BLOCK_Z + RAIL_BLOCK_HEIGHT
RAIL_CENTER_Z = (LOWER_BLOCK_Z + UPPER_BLOCK_Z) / 2.0

CARRIAGE_WIDTH = 0.182
CARRIAGE_HEIGHT = 0.160
CARRIAGE_FRONT_DEPTH = 0.018
CARRIAGE_BLOCK_DEPTH = 0.056
CARRIAGE_BLOCK_WIDTH = 0.036
CARRIAGE_BLOCK_HEIGHT = 0.118
CARRIAGE_CAP_HEIGHT = 0.018
CARRIAGE_TOP_PAD_HEIGHT = 0.012
CARRIAGE_JOINT_X = RAIL_X
CARRIAGE_JOINT_Z = 0.215
CARRIAGE_TRAVEL = 0.245
RAIL_CLEARANCE_RADIUS = 0.0125

YAW_LIMIT = 3.02


def _z_cylinder(radius: float, height: float, z_bottom: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z_bottom))


def _center_box(dx: float, dy: float, dz: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate(center)


def _base_shape() -> cq.Workplane:
    base = _z_cylinder(BASE_FOOT_RADIUS, BASE_FOOT_THICKNESS, 0.0)
    base = base.union(_z_cylinder(BASE_BODY_RADIUS, BASE_BODY_HEIGHT, BASE_FOOT_THICKNESS))
    base = base.union(_z_cylinder(BASE_TOP_RADIUS, BASE_TOP_HEIGHT, BASE_FOOT_THICKNESS + BASE_BODY_HEIGHT))

    base = base.cut(
        _z_cylinder(0.086, 0.008, BASE_HEIGHT - 0.008)
    )
    base = base.cut(
        _z_cylinder(0.052, BASE_HEIGHT - 0.022, 0.022)
    )

    top_holes = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT - 0.012)
        .polarArray(0.098, 0.0, 360.0, 8)
        .circle(0.005)
        .extrude(0.012)
    ).union(
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT - 0.004)
        .polarArray(0.098, 0.0, 360.0, 8)
        .circle(0.010)
        .extrude(0.004)
    )
    foot_holes = (
        cq.Workplane("XY")
        .polarArray(0.158, 0.0, 360.0, 6)
        .circle(0.006)
        .extrude(BASE_FOOT_THICKNESS)
    ).union(
        cq.Workplane("XY")
        .workplane(offset=BASE_FOOT_THICKNESS - 0.005)
        .polarArray(0.158, 0.0, 360.0, 6)
        .circle(0.012)
        .extrude(0.005)
    )
    base = base.cut(top_holes).cut(foot_holes)

    return base


def _turntable_shape() -> cq.Workplane:
    turntable = _z_cylinder(TURN_PLATE_RADIUS, TURN_PLATE_THICKNESS, TURN_PLATE_GAP)
    turntable = turntable.union(
        _z_cylinder(TURN_PLATE_RADIUS * 0.92, TURN_PLATE_LIP_THICKNESS, TURN_PLATE_GAP + TURN_PLATE_THICKNESS)
    )
    turntable = turntable.union(
        _z_cylinder(ROTARY_HUB_RADIUS, ROTARY_HUB_HEIGHT, TURN_PLATE_STACK_HEIGHT - 0.002)
    )

    pocket = cq.Workplane("XY").workplane(offset=TURN_PLATE_STACK_HEIGHT - 0.010).circle(0.036).extrude(0.010)
    bolt_circle = (
        cq.Workplane("XY")
        .workplane(offset=TURN_PLATE_STACK_HEIGHT - 0.010)
        .polarArray(0.104, 0.0, 360.0, 8)
        .circle(0.004)
        .extrude(0.010)
    ).union(
        cq.Workplane("XY")
        .workplane(offset=TURN_PLATE_STACK_HEIGHT - 0.003)
        .polarArray(0.104, 0.0, 360.0, 8)
        .circle(0.008)
        .extrude(0.003)
    )
    turntable = turntable.cut(pocket).cut(bolt_circle)
    return turntable


def _mast_frame_shape() -> cq.Workplane:
    mast_bottom = TURN_PLATE_STACK_HEIGHT - 0.002
    mast_center_z = mast_bottom + MAST_HEIGHT / 2.0

    base_block = _center_box(
        MAST_BASE_BLOCK_DEPTH,
        MAST_BASE_BLOCK_WIDTH,
        MAST_BASE_BLOCK_HEIGHT,
        (-0.010, 0.0, mast_bottom + MAST_BASE_BLOCK_HEIGHT / 2.0),
    )
    back_plate = _center_box(
        MAST_BACK_DEPTH,
        MAST_WIDTH,
        MAST_HEIGHT,
        (-0.030, 0.0, mast_center_z),
    )
    side_left = _center_box(
        MAST_SIDE_DEPTH,
        MAST_SIDE_WALL,
        MAST_HEIGHT,
        (-0.008, RAIL_Y + 0.030, mast_center_z),
    )
    side_right = _center_box(
        MAST_SIDE_DEPTH,
        MAST_SIDE_WALL,
        MAST_HEIGHT,
        (-0.008, -RAIL_Y - 0.030, mast_center_z),
    )
    front_lip_left = _center_box(
        MAST_FRONT_LIP_DEPTH,
        MAST_FRONT_LIP_WIDTH,
        MAST_HEIGHT - 0.040,
        (0.015, RAIL_Y + 0.010, mast_bottom + (MAST_HEIGHT - 0.040) / 2.0 + 0.020),
    )
    front_lip_right = _center_box(
        MAST_FRONT_LIP_DEPTH,
        MAST_FRONT_LIP_WIDTH,
        MAST_HEIGHT - 0.040,
        (0.015, -RAIL_Y - 0.010, mast_bottom + (MAST_HEIGHT - 0.040) / 2.0 + 0.020),
    )
    top_cap = _center_box(
        MAST_SIDE_DEPTH,
        MAST_WIDTH,
        MAST_TOP_CAP_HEIGHT,
        (-0.008, 0.0, mast_bottom + MAST_HEIGHT - MAST_TOP_CAP_HEIGHT / 2.0),
    )

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.050, TURN_PLATE_STACK_HEIGHT),
                (-0.050, TURN_PLATE_STACK_HEIGHT + 0.155),
                (0.012, TURN_PLATE_STACK_HEIGHT + 0.030),
                (0.012, TURN_PLATE_STACK_HEIGHT),
            ]
        )
        .close()
        .extrude(0.014, both=True)
    )
    gusset_left = gusset_profile.translate((0.0, 0.050, 0.0))
    gusset_right = gusset_profile.translate((0.0, -0.050, 0.0))

    frame = (
        base_block.union(back_plate)
        .union(side_left)
        .union(side_right)
        .union(front_lip_left)
        .union(front_lip_right)
        .union(top_cap)
        .union(gusset_left)
        .union(gusset_right)
    )

    frame = frame.faces(">X").workplane(centerOption="CenterOfMass").rect(0.044, 0.340).cutBlind(-0.018)
    return frame.edges("|Z").fillet(0.0025)


def _rail_block_shape(center_z: float) -> cq.Workplane:
    block = _center_box(
        RAIL_BLOCK_DEPTH,
        RAIL_BLOCK_WIDTH,
        RAIL_BLOCK_HEIGHT,
        (0.036, 0.0, center_z),
    )
    relief = _center_box(0.014, 0.048, 0.012, (0.050, 0.0, center_z))
    return block.cut(relief).edges(">X and |Z").fillet(0.0015)


def _carriage_shape() -> cq.Workplane:
    left_housing = _center_box(0.034, 0.044, 0.124, (0.0, RAIL_Y, 0.0))
    right_housing = _center_box(0.034, 0.044, 0.124, (0.0, -RAIL_Y, 0.0))
    rear_bridge = _center_box(0.014, 0.120, 0.108, (-0.009, 0.0, 0.0))
    front_bridge = _center_box(0.020, CARRIAGE_WIDTH, 0.142, (0.018, 0.0, 0.009))
    top_cap = _center_box(0.028, CARRIAGE_WIDTH, 0.018, (0.012, 0.0, 0.071))

    body = left_housing.union(right_housing).union(rear_bridge).union(front_bridge).union(top_cap)

    rail_bores = (
        cq.Workplane("XY")
        .pushPoints([(0.0, RAIL_Y), (0.0, -RAIL_Y)])
        .circle(RAIL_RADIUS)
        .extrude(0.260, both=True)
    )
    body = body.cut(rail_bores)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_indexer")

    model.material("powder_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("anodized_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("rail_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("oxide_black", rgba=(0.14, 0.15, 0.16, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS / 2.0)),
        material="powder_charcoal",
        name="base_shell",
    )
    pedestal_base.visual(
        Cylinder(radius=BASE_BODY_RADIUS, length=BASE_BODY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS + BASE_BODY_HEIGHT / 2.0)),
        material="powder_charcoal",
        name="base_body",
    )
    pedestal_base.visual(
        Cylinder(radius=BASE_TOP_RADIUS, length=BASE_TOP_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_FOOT_THICKNESS + BASE_BODY_HEIGHT + BASE_TOP_HEIGHT / 2.0)
        ),
        material="anodized_gray",
        name="base_top",
    )
    pedestal_base.visual(
        Cylinder(radius=0.094, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT - 0.004)),
        material="machined_steel",
        name="bearing_face",
    )
    pedestal_base.visual(
        Cylinder(radius=0.110, length=TURN_PLATE_GAP),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + TURN_PLATE_GAP / 2.0)),
        material="machined_steel",
        name="thrust_ring",
    )
    for angle_deg in range(0, 360, 60):
        angle = math.radians(angle_deg)
        pedestal_base.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(
                xyz=(0.158 * math.cos(angle), 0.158 * math.sin(angle), 0.016)
            ),
            material="oxide_black",
            name=f"foot_bolt_{angle_deg}",
        )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_BODY_RADIUS, length=BASE_HEIGHT),
        mass=20.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    rotary_mast = model.part("rotary_mast")
    rotary_mast.visual(
        Cylinder(radius=TURN_PLATE_RADIUS, length=TURN_PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, TURN_PLATE_GAP + TURN_PLATE_THICKNESS / 2.0)),
        material="machined_steel",
        name="turntable",
    )
    rotary_mast.visual(
        Cylinder(radius=TURN_PLATE_RADIUS * 0.92, length=TURN_PLATE_LIP_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, TURN_PLATE_GAP + TURN_PLATE_THICKNESS + TURN_PLATE_LIP_THICKNESS / 2.0)
        ),
        material="machined_steel",
        name="turntable_lip",
    )
    rotary_mast.visual(
        Cylinder(radius=ROTARY_HUB_RADIUS, length=ROTARY_HUB_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, TURN_PLATE_STACK_HEIGHT - 0.002 + ROTARY_HUB_HEIGHT / 2.0)
        ),
        material="anodized_gray",
        name="mast_hub",
    )
    rotary_mast.visual(
        Box((MAST_BASE_BLOCK_DEPTH, MAST_BASE_BLOCK_WIDTH, MAST_BASE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(-0.010, 0.0, TURN_PLATE_STACK_HEIGHT - 0.002 + MAST_BASE_BLOCK_HEIGHT / 2.0)
        ),
        material="anodized_gray",
        name="mast_foot",
    )
    rotary_mast.visual(
        Box((MAST_BACK_DEPTH, MAST_WIDTH, MAST_HEIGHT)),
        origin=Origin(xyz=(-0.030, 0.0, TURN_PLATE_STACK_HEIGHT - 0.002 + MAST_HEIGHT / 2.0)),
        material="anodized_gray",
        name="mast_frame",
    )
    rotary_mast.visual(
        Box((MAST_SIDE_DEPTH, MAST_SIDE_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(-0.008, RAIL_Y + 0.030, TURN_PLATE_STACK_HEIGHT - 0.002 + MAST_HEIGHT / 2.0)),
        material="anodized_gray",
        name="mast_side_left",
    )
    rotary_mast.visual(
        Box((MAST_SIDE_DEPTH, MAST_SIDE_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(-0.008, -RAIL_Y - 0.030, TURN_PLATE_STACK_HEIGHT - 0.002 + MAST_HEIGHT / 2.0)),
        material="anodized_gray",
        name="mast_side_right",
    )
    rotary_mast.visual(
        Box((MAST_FRONT_LIP_DEPTH, MAST_FRONT_LIP_WIDTH, MAST_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(0.014, RAIL_Y + 0.010, TURN_PLATE_STACK_HEIGHT + 0.028 + (MAST_HEIGHT - 0.040) / 2.0)
        ),
        material="anodized_gray",
        name="mast_lip_left",
    )
    rotary_mast.visual(
        Box((MAST_FRONT_LIP_DEPTH, MAST_FRONT_LIP_WIDTH, MAST_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(0.014, -RAIL_Y - 0.010, TURN_PLATE_STACK_HEIGHT + 0.028 + (MAST_HEIGHT - 0.040) / 2.0)
        ),
        material="anodized_gray",
        name="mast_lip_right",
    )
    rotary_mast.visual(
        Box((MAST_SIDE_DEPTH, MAST_WIDTH, MAST_TOP_CAP_HEIGHT)),
        origin=Origin(
            xyz=(-0.008, 0.0, TURN_PLATE_STACK_HEIGHT - 0.002 + MAST_HEIGHT - MAST_TOP_CAP_HEIGHT / 2.0)
        ),
        material="anodized_gray",
        name="mast_cap",
    )
    rotary_mast.visual(
        Box((0.012, 0.022, 0.180)),
        origin=Origin(xyz=(-0.018, 0.052, 0.118)),
        material="anodized_gray",
        name="stiffener_left",
    )
    rotary_mast.visual(
        Box((0.012, 0.022, 0.180)),
        origin=Origin(xyz=(-0.018, -0.052, 0.118)),
        material="anodized_gray",
        name="stiffener_right",
    )
    rotary_mast.visual(
        Box((RAIL_BLOCK_DEPTH, RAIL_BLOCK_WIDTH, RAIL_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.030, 0.0, LOWER_BLOCK_Z)),
        material="machined_steel",
        name="lower_block",
    )
    rotary_mast.visual(
        Box((RAIL_BLOCK_DEPTH, RAIL_BLOCK_WIDTH, RAIL_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.030, 0.0, UPPER_BLOCK_Z)),
        material="machined_steel",
        name="upper_block",
    )
    rotary_mast.visual(
        Cylinder(radius=RAIL_RADIUS, length=RAIL_LENGTH),
        origin=Origin(xyz=(RAIL_X, RAIL_Y, RAIL_CENTER_Z)),
        material="rail_steel",
        name="rail_left",
    )
    rotary_mast.visual(
        Cylinder(radius=RAIL_RADIUS, length=RAIL_LENGTH),
        origin=Origin(xyz=(RAIL_X, -RAIL_Y, RAIL_CENTER_Z)),
        material="rail_steel",
        name="rail_right",
    )
    for angle_deg in range(0, 360, 45):
        angle = math.radians(angle_deg)
        rotary_mast.visual(
            Cylinder(radius=0.007, length=0.003),
            origin=Origin(
                xyz=(0.104 * math.cos(angle), 0.104 * math.sin(angle), TURN_PLATE_STACK_HEIGHT + 0.0015)
            ),
            material="oxide_black",
            name=f"turntable_bolt_{angle_deg}",
        )
    rotary_mast.inertial = Inertial.from_geometry(
        Box((0.160, 0.160, 0.650)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.020, 0.136, 0.160)),
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
        material="machined_steel",
        name="carriage_body",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.130)),
        origin=Origin(xyz=(-0.015, RAIL_Y + 0.010, 0.0)),
        material="anodized_gray",
        name="left_shoe",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.130)),
        origin=Origin(xyz=(-0.015, -RAIL_Y - 0.010, 0.0)),
        material="anodized_gray",
        name="right_shoe",
    )
    carriage.visual(
        Box((0.060, 0.012, 0.060)),
        origin=Origin(xyz=(0.010, RAIL_Y + 0.023, 0.0)),
        material="anodized_gray",
        name="left_arm",
    )
    carriage.visual(
        Box((0.060, 0.012, 0.060)),
        origin=Origin(xyz=(0.010, -RAIL_Y - 0.023, 0.0)),
        material="anodized_gray",
        name="right_arm",
    )
    carriage.visual(
        Box((0.018, 0.136, 0.020)),
        origin=Origin(xyz=(0.028, 0.0, -0.070)),
        material="anodized_gray",
        name="carriage_cap_bottom",
    )
    carriage.visual(
        Box((0.024, 0.100, CARRIAGE_TOP_PAD_HEIGHT)),
        origin=Origin(xyz=(0.032, 0.0, 0.084)),
        material="machined_steel",
        name="tooling_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.060, CARRIAGE_WIDTH, CARRIAGE_HEIGHT + CARRIAGE_TOP_PAD_HEIGHT)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_PAD_HEIGHT / 2.0)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=rotary_mast,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=rotary_mast,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_JOINT_X, 0.0, CARRIAGE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.22,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    rotary_mast = object_model.get_part("rotary_mast")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("pedestal_yaw")
    lift = object_model.get_articulation("carriage_lift")
    thrust_ring = pedestal_base.get_visual("thrust_ring")
    turntable = rotary_mast.get_visual("turntable")
    lower_block = rotary_mast.get_visual("lower_block")
    upper_block = rotary_mast.get_visual("upper_block")
    mast_lip_left = rotary_mast.get_visual("mast_lip_left")
    mast_lip_right = rotary_mast.get_visual("mast_lip_right")
    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")
    tooling_pad = carriage.get_visual("tooling_pad")
    carriage_bottom = carriage.get_visual("carriage_cap_bottom")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "yaw_axis_is_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "lift_axis_is_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical lift axis, got {lift.axis}",
    )
    ctx.check(
        "lift_travel_is_believable",
        lift.motion_limits is not None and abs((lift.motion_limits.upper or 0.0) - CARRIAGE_TRAVEL) < 1e-9,
        details=f"unexpected lift travel: {lift.motion_limits}",
    )

    with ctx.pose({yaw: 0.0, lift: 0.0}):
        ctx.expect_contact(
            rotary_mast,
            pedestal_base,
            elem_a=turntable,
            elem_b=thrust_ring,
            name="turntable_is_grounded_on_thrust_ring",
        )
        ctx.expect_overlap(
            rotary_mast,
            pedestal_base,
            axes="xy",
            elem_a=turntable,
            elem_b=thrust_ring,
            min_overlap=0.180,
            name="turntable_is_centered_over_bearing_face",
        )
        ctx.expect_contact(
            carriage,
            rotary_mast,
            elem_a=left_shoe,
            elem_b=mast_lip_left,
            name="left_carriage_shoe_slides_on_left_lip",
        )
        ctx.expect_contact(
            carriage,
            rotary_mast,
            elem_a=right_shoe,
            elem_b=mast_lip_right,
            name="right_carriage_shoe_slides_on_right_lip",
        )
        ctx.expect_gap(
            carriage,
            rotary_mast,
            axis="z",
            positive_elem=carriage_bottom,
            negative_elem=lower_block,
            min_gap=0.004,
            max_gap=0.015,
            name="carriage_clears_lower_stop_block",
        )

    with ctx.pose({yaw: 1.3, lift: CARRIAGE_TRAVEL}):
        ctx.expect_gap(
            rotary_mast,
            carriage,
            axis="z",
            positive_elem=upper_block,
            negative_elem=tooling_pad,
            min_gap=0.004,
            max_gap=0.015,
            name="carriage_clears_upper_stop_block",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
