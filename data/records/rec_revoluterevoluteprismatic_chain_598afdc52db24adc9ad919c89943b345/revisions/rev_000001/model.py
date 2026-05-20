from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PLATE_T = 0.006
PIN_R = 0.008
HINGE_CLEARANCE_R = 0.0008

PROX_GAP = 0.028
PROX_OUTER_W = PROX_GAP + 2.0 * PLATE_T
PROX_PLATE_H = 0.050
PROX_LEN = 0.220

ROOT_EAR_T = 0.007
ROOT_INNER_GAP = PROX_OUTER_W
ROOT_TOTAL_W = ROOT_INNER_GAP + 2.0 * ROOT_EAR_T
ROOT_EAR_LEN = 0.040
ROOT_EAR_H = 0.060
WALL_PLATE_T = 0.012
WALL_PLATE_W = 0.090
WALL_PLATE_H = 0.180

DIST_LUG_W = PROX_GAP
DIST_OUTER_W = 0.056
DIST_OUTER_H = 0.052
DIST_SIDE_Y = DIST_OUTER_W / 2.0 - PLATE_T / 2.0
DIST_INNER_W = DIST_OUTER_W - 2.0 * PLATE_T
DIST_RAIL_T = 0.005
DIST_INNER_H = DIST_OUTER_H - 2.0 * DIST_RAIL_T
DIST_SIDE_LEN = 0.190
DIST_SIDE_START = 0.055
DIST_TOTAL_LEN = DIST_SIDE_START + DIST_SIDE_LEN

CARRIAGE_GUIDE_REAR = -0.075
CARRIAGE_GUIDE_LEN = 0.075
CARRIAGE_GUIDE_W = 0.014
CARRIAGE_GUIDE_CORE_H = 0.014
CARRIAGE_STALK_LEN = 0.035
CARRIAGE_STALK_W = 0.016
CARRIAGE_STALK_H = 0.018
CARRIAGE_FRONT_LEN = 0.070
CARRIAGE_FRONT_W = 0.034
CARRIAGE_FRONT_H = 0.030
CARRIAGE_FRONT_START = CARRIAGE_STALK_LEN
CARRIAGE_ROLLER_R = 0.006
CARRIAGE_ROLLER_LEN = 0.014
CARRIAGE_ROLLER_Z = DIST_INNER_H / 2.0 - CARRIAGE_ROLLER_R
CARRIAGE_ROLLER_POST_X = 0.008
CARRIAGE_ROLLER_POST_Y = 0.010
CARRIAGE_ROLLER_POST_H = CARRIAGE_ROLLER_Z - CARRIAGE_GUIDE_CORE_H / 2.0
CARRIAGE_ROLLER_XS = (-0.056, -0.022)
CARRIAGE_REAR_AT_ZERO = 0.215
CARRIAGE_TRAVEL = 0.090


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_x(
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
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def _cylinder_y(
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
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y, z))
    )


def _root_bracket_shape() -> cq.Workplane:
    wall_plate = _box(
        (WALL_PLATE_T, WALL_PLATE_W, WALL_PLATE_H),
        (-(ROOT_EAR_LEN + WALL_PLATE_T) / 2.0, 0.0, 0.0),
    )
    left_ear = _box(
        (ROOT_EAR_LEN, ROOT_EAR_T, ROOT_EAR_H),
        (-ROOT_EAR_LEN / 2.0, ROOT_INNER_GAP / 2.0 + ROOT_EAR_T / 2.0, 0.0),
    )
    right_ear = _box(
        (ROOT_EAR_LEN, ROOT_EAR_T, ROOT_EAR_H),
        (-ROOT_EAR_LEN / 2.0, -(ROOT_INNER_GAP / 2.0 + ROOT_EAR_T / 2.0), 0.0),
    )
    back_web = _box((0.016, ROOT_INNER_GAP, 0.036), (-0.032, 0.0, 0.0))
    bracket = wall_plate.union(left_ear).union(right_ear).union(back_web)

    hole_points = [(-0.010, -0.050), (-0.010, 0.050), (0.010, -0.050), (0.010, 0.050)]
    for y, z in hole_points:
        bracket = bracket.cut(
            _cylinder_x(
                0.0035,
                WALL_PLATE_T + 0.004,
                x=-(ROOT_EAR_LEN + WALL_PLATE_T) / 2.0,
                y=y,
                z=z,
            )
        )

    bracket = bracket.cut(
        _cylinder_y(
            PIN_R + HINGE_CLEARANCE_R,
            ROOT_TOTAL_W + 0.004,
            x=0.0,
            y=0.0,
            z=0.0,
        )
    )

    return bracket


def _plate_link_shape(
    *,
    length: float,
    plate_height: float,
    outer_width: float,
    inner_gap: float,
    proximal_hole_radius: float | None = None,
    distal_hole_radius: float | None = None,
) -> cq.Workplane:
    plate_y = inner_gap / 2.0 + PLATE_T / 2.0
    slot_len = length - 0.100
    slot_center_x = length / 2.0 + 0.006

    def side_plate(y_center: float) -> cq.Workplane:
        plate = _box((length, PLATE_T, plate_height), (length / 2.0, y_center, 0.0))
        if proximal_hole_radius is not None:
            plate = plate.cut(
                _cylinder_y(proximal_hole_radius, PLATE_T + 0.004, x=0.0, y=y_center, z=0.0)
            )
        window = _box((slot_len, PLATE_T + 0.002, 0.020), (slot_center_x, y_center, 0.0))
        return plate.cut(window)

    left_plate = side_plate(plate_y)
    right_plate = side_plate(-plate_y)
    spacer_a = _box((0.022, inner_gap, 0.022), (0.070, 0.0, 0.0))
    spacer_b = _box((0.022, inner_gap, 0.022), (0.155, 0.0, 0.0))

    link = left_plate.union(right_plate).union(spacer_a).union(spacer_b)
    if distal_hole_radius is not None:
        link = link.cut(
            _cylinder_y(
                distal_hole_radius,
                outer_width + 0.004,
                x=length,
                y=0.0,
                z=0.0,
            )
        )
    return link


def _distal_link_shape() -> cq.Workplane:
    lug = _box((0.032, DIST_LUG_W, 0.040), (0.016, 0.0, 0.0))
    lug = lug.cut(
        _cylinder_y(
            PIN_R + HINGE_CLEARANCE_R,
            DIST_LUG_W + 0.004,
            x=0.0,
            y=0.0,
            z=0.0,
        )
    )
    neck = _box((0.022, DIST_LUG_W, 0.030), (0.043, 0.0, 0.0))
    bridge = _box((0.020, DIST_INNER_W, 0.020), (0.055, 0.0, 0.0))

    def side_plate(y_center: float) -> cq.Workplane:
        plate = _box(
            (DIST_SIDE_LEN, PLATE_T, DIST_OUTER_H),
            (DIST_SIDE_START + DIST_SIDE_LEN / 2.0, y_center, 0.0),
        )
        window = _box((0.105, PLATE_T + 0.002, 0.022), (0.155, y_center, 0.0))
        return plate.cut(window)

    left_plate = side_plate(DIST_SIDE_Y)
    right_plate = side_plate(-DIST_SIDE_Y)

    top_rail = _box((0.165, DIST_INNER_W, DIST_RAIL_T), (0.145, 0.0, DIST_OUTER_H / 2.0 - DIST_RAIL_T / 2.0))
    bottom_rail = _box(
        (0.165, DIST_INNER_W, DIST_RAIL_T),
        (0.145, 0.0, -(DIST_OUTER_H / 2.0 - DIST_RAIL_T / 2.0)),
    )

    return (
        lug.union(neck)
        .union(bridge)
        .union(left_plate)
        .union(right_plate)
        .union(top_rail)
        .union(bottom_rail)
    )


def _nose_carriage_guide_shape() -> cq.Workplane:
    guide_center_x = CARRIAGE_GUIDE_REAR + CARRIAGE_GUIDE_LEN / 2.0
    core = _box(
        (CARRIAGE_GUIDE_LEN, CARRIAGE_GUIDE_W, CARRIAGE_GUIDE_CORE_H),
        (guide_center_x, 0.0, 0.0),
    )
    stalk = _box(
        (CARRIAGE_STALK_LEN, CARRIAGE_STALK_W, CARRIAGE_STALK_H),
        (CARRIAGE_STALK_LEN / 2.0, 0.0, 0.0),
    )
    guide = core.union(stalk)
    top_post_z = CARRIAGE_GUIDE_CORE_H / 2.0 + CARRIAGE_ROLLER_POST_H / 2.0
    bottom_post_z = -top_post_z
    for roller_x in CARRIAGE_ROLLER_XS:
        guide = guide.union(
            _box(
                (CARRIAGE_ROLLER_POST_X, CARRIAGE_ROLLER_POST_Y, CARRIAGE_ROLLER_POST_H),
                (roller_x, 0.0, top_post_z),
            )
        )
        guide = guide.union(
            _box(
                (CARRIAGE_ROLLER_POST_X, CARRIAGE_ROLLER_POST_Y, CARRIAGE_ROLLER_POST_H),
                (roller_x, 0.0, bottom_post_z),
            )
        )
    return guide


def _nose_carriage_head_shape() -> cq.Workplane:
    front_box = (
        _box(
            (CARRIAGE_FRONT_LEN, CARRIAGE_FRONT_W, CARRIAGE_FRONT_H),
            (CARRIAGE_FRONT_START + CARRIAGE_FRONT_LEN / 2.0, 0.0, 0.0),
        )
        .edges("|Z")
        .fillet(0.004)
    )
    nose_cap = _box(
        (0.018, 0.026, 0.022),
        (CARRIAGE_FRONT_START + CARRIAGE_FRONT_LEN + 0.009, 0.0, 0.0),
    )
    return front_box.union(nose_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_folding_arm")

    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    link_steel = model.material("link_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.18, 0.20, 0.22, 1.0))

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        mesh_from_cadquery(_root_bracket_shape(), "wall_mount_bracket"),
        material=dark_steel,
        name="bracket",
    )
    wall_mount.inertial = Inertial.from_geometry(
        Box((ROOT_EAR_LEN + WALL_PLATE_T, ROOT_TOTAL_W, WALL_PLATE_H)),
        mass=2.4,
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(
            _plate_link_shape(
                length=PROX_LEN,
                plate_height=PROX_PLATE_H,
                outer_width=PROX_OUTER_W,
                inner_gap=PROX_GAP,
                proximal_hole_radius=PIN_R + HINGE_CLEARANCE_R,
                distal_hole_radius=PIN_R + HINGE_CLEARANCE_R,
            ),
            "proximal_link",
        ),
        material=link_steel,
        name="beam",
    )
    proximal_link.inertial = Inertial.from_geometry(
        Box((PROX_LEN, PROX_OUTER_W, PROX_PLATE_H)),
        mass=1.3,
        origin=Origin(xyz=(PROX_LEN / 2.0, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material=link_steel,
        name="sleeve",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((DIST_TOTAL_LEN, DIST_OUTER_W, DIST_OUTER_H)),
        mass=1.15,
        origin=Origin(xyz=(DIST_TOTAL_LEN / 2.0, 0.0, 0.0)),
    )

    nose_carriage = model.part("nose_carriage")
    nose_carriage.visual(
        mesh_from_cadquery(_nose_carriage_guide_shape(), "nose_carriage_guide"),
        material=carriage_gray,
        name="guide_stem",
    )
    for i, roller_x in enumerate(CARRIAGE_ROLLER_XS, start=1):
        nose_carriage.visual(
            Cylinder(radius=CARRIAGE_ROLLER_R, length=CARRIAGE_ROLLER_LEN),
            origin=Origin(
                xyz=(roller_x, 0.0, CARRIAGE_ROLLER_Z),
                rpy=(-1.5707963267948966, 0.0, 0.0),
            ),
            material=link_steel,
            name=f"top_roller_{i}",
        )
        nose_carriage.visual(
            Cylinder(radius=CARRIAGE_ROLLER_R, length=CARRIAGE_ROLLER_LEN),
            origin=Origin(
                xyz=(roller_x, 0.0, -CARRIAGE_ROLLER_Z),
                rpy=(-1.5707963267948966, 0.0, 0.0),
            ),
            material=link_steel,
            name=f"bottom_roller_{i}",
        )
    nose_carriage.visual(
        mesh_from_cadquery(_nose_carriage_head_shape(), "nose_carriage_head"),
        material=carriage_gray,
        name="nose_head",
    )
    nose_carriage.inertial = Inertial.from_geometry(
        Box((0.197, CARRIAGE_FRONT_W, DIST_INNER_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0235, 0.0, 0.0)),
    )

    model.articulation(
        "wall_mount_to_proximal",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=proximal_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-2.50,
            upper=1.25,
        ),
    )
    model.articulation(
        "proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(PROX_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.0,
            lower=-2.75,
            upper=0.10,
        ),
    )
    model.articulation(
        "distal_to_nose",
        ArticulationType.PRISMATIC,
        parent=distal_link,
        child=nose_carriage,
        origin=Origin(xyz=(CARRIAGE_REAR_AT_ZERO, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    nose_carriage = object_model.get_part("nose_carriage")
    guide_stem = nose_carriage.get_visual("guide_stem")

    root_joint = object_model.get_articulation("wall_mount_to_proximal")
    elbow_joint = object_model.get_articulation("proximal_to_distal")
    slide_joint = object_model.get_articulation("distal_to_nose")

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
        "expected_parts_present",
        all(
            part is not None
            for part in (wall_mount, proximal_link, distal_link, nose_carriage)
        ),
        "All four authored parts should be present.",
    )
    ctx.check(
        "root_axis_is_lateral_hinge",
        tuple(root_joint.axis) == (0.0, 1.0, 0.0),
        f"root axis was {root_joint.axis}, expected (0, 1, 0)",
    )
    ctx.check(
        "elbow_axis_is_lateral_hinge",
        tuple(elbow_joint.axis) == (0.0, 1.0, 0.0),
        f"elbow axis was {elbow_joint.axis}, expected (0, 1, 0)",
    )
    ctx.check(
        "nose_axis_is_distal_slide",
        tuple(slide_joint.axis) == (1.0, 0.0, 0.0),
        f"nose slide axis was {slide_joint.axis}, expected (1, 0, 0)",
    )

    ctx.expect_contact(wall_mount, proximal_link, name="root_joint_connected")
    ctx.expect_contact(proximal_link, distal_link, name="elbow_joint_connected")
    ctx.expect_within(
        nose_carriage,
        distal_link,
        axes="yz",
        margin=0.002,
        inner_elem=guide_stem,
        name="nose_stays_within_distal_cross_section_at_rest",
    )
    ctx.expect_overlap(
        nose_carriage,
        distal_link,
        axes="x",
        min_overlap=0.095,
        elem_a=guide_stem,
        name="nose_has_real_insertion_at_rest",
    )

    rest_nose_x = ctx.part_world_position(nose_carriage)[0]
    with ctx.pose({slide_joint: CARRIAGE_TRAVEL}):
        ctx.expect_within(
            nose_carriage,
            distal_link,
            axes="yz",
            margin=0.002,
            inner_elem=guide_stem,
            name="nose_stays_within_distal_cross_section_extended",
        )
        ctx.expect_overlap(
            nose_carriage,
            distal_link,
            axes="x",
            min_overlap=0.012,
            elem_a=guide_stem,
            name="nose_remains_inserted_when_extended",
        )
        extended_nose_x = ctx.part_world_position(nose_carriage)[0]
    ctx.check(
        "nose_extends_forward",
        extended_nose_x > rest_nose_x + 0.050,
        f"expected carriage extension > 0.050 m, got {extended_nose_x - rest_nose_x:.4f} m",
    )

    with ctx.pose({root_joint: -1.35, elbow_joint: -2.20, slide_joint: 0.0}):
        folded_nose_x = ctx.part_world_position(nose_carriage)[0]
    ctx.check(
        "folded_pose_compacts_toward_wall",
        folded_nose_x < rest_nose_x - 0.120,
        f"folded nose x={folded_nose_x:.4f} m did not move meaningfully back from rest x={rest_nose_x:.4f} m",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
