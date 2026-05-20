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


PLATE_T = 0.022
PLATE_W = 0.620
PLATE_H = 1.040

Y_SUPPORT_T = 0.018
Y_SUPPORT_W = 0.520
Y_SUPPORT_H = 0.400
Y_SUPPORT_Z = 0.180

Y_RAIL_T = 0.014
Y_RAIL_H = 0.022
Y_RAIL_L = 0.480
Y_RAIL_TOP_Z = 0.250
Y_RAIL_BOTTOM_Z = 0.110

Y_STOP_X = 0.028
Y_STOP_Y = 0.014
Y_STOP_Z = 0.092
Y_STOP_CENTER_Y = 0.300

SADDLE_BRIDGE_X = 0.048
SADDLE_BRIDGE_Y = 0.320
SADDLE_BRIDGE_Z = 0.105

Y_BLOCK_X = 0.018
Y_BLOCK_Y = 0.064
Y_BLOCK_Z = 0.044
Y_BLOCK_CENTER_X = -(SADDLE_BRIDGE_X / 2.0) - (Y_BLOCK_X / 2.0) + 0.001

SADDLE_TOWER_X = 0.040
SADDLE_TOWER_Y = 0.118
SADDLE_TOWER_Z = 0.360
SADDLE_TOWER_CENTER_X = 0.004
SADDLE_TOWER_CENTER_Z = -0.222

SADDLE_STOP_BAR_X = 0.014
SADDLE_STOP_BAR_Y = 0.030
SADDLE_STOP_BAR_Z = 0.590
SADDLE_STOP_BAR_CENTER_X = -0.018
SADDLE_STOP_BAR_CENTER_Z = -0.306

SADDLE_NOSE_X = 0.050
SADDLE_NOSE_Y = 0.160
SADDLE_NOSE_Z = 0.024
SADDLE_NOSE_CENTER_X = 0.010
SADDLE_NOSE_CENTER_Z = -0.395

Z_RAIL_T = 0.015
Z_RAIL_Y = 0.020
Z_RAIL_Z = 0.385
Z_RAIL_CENTER_Y = 0.034
Z_RAIL_CENTER_Z = -0.220

Y_RAIL_FRONT_X = (PLATE_T / 2.0) + Y_SUPPORT_T + Y_RAIL_T
SADDLE_REAR_CONTACT_X = Y_BLOCK_CENTER_X - (Y_BLOCK_X / 2.0)
SADDLE_ORIGIN_X = Y_RAIL_FRONT_X - SADDLE_REAR_CONTACT_X
SADDLE_ORIGIN_Z = (Y_RAIL_TOP_Z + Y_RAIL_BOTTOM_Z) / 2.0

Z_RAIL_CENTER_X = SADDLE_TOWER_CENTER_X + (SADDLE_TOWER_X / 2.0) + (Z_RAIL_T / 2.0)

CAR_HEAD_X = 0.045
CAR_HEAD_Y = 0.108
CAR_HEAD_Z = 0.092

CAR_BLOCK_X = 0.018
CAR_BLOCK_Y = 0.040
CAR_BLOCK_Z = 0.080
CAR_BLOCK_CENTER_X = -(CAR_HEAD_X / 2.0) - (CAR_BLOCK_X / 2.0) + 0.001

CAR_PLATE_X = 0.035
CAR_PLATE_Y = 0.090
CAR_PLATE_Z = 0.250
CAR_PLATE_CENTER_X = 0.005
CAR_PLATE_CENTER_Z = -0.170

CAR_FRONT_COVER_X = 0.016
CAR_FRONT_COVER_Y = 0.078
CAR_FRONT_COVER_Z = 0.220
CAR_FRONT_COVER_CENTER_X = 0.020
CAR_FRONT_COVER_CENTER_Z = -0.175

CAR_PAD_X = 0.054
CAR_PAD_Y = 0.145
CAR_PAD_Z = 0.028
CAR_PAD_CENTER_X = 0.008
CAR_PAD_CENTER_Z = -0.306

TOP_DOG_X = 0.018
TOP_DOG_Y = 0.034
TOP_DOG_Z = 0.012
TOP_DOG_CENTER_X = -0.010
TOP_DOG_CENTER_Z = 0.049

BOTTOM_DOG_X = 0.020
BOTTOM_DOG_Y = 0.040
BOTTOM_DOG_Z = 0.012
BOTTOM_DOG_CENTER_X = -0.006
BOTTOM_DOG_CENTER_Z = -0.308

Z_JOINT_X = Z_RAIL_CENTER_X + (Z_RAIL_T / 2.0) - (CAR_BLOCK_CENTER_X - (CAR_BLOCK_X / 2.0))
Z_JOINT_Z = -0.125

Y_TRAVEL = 0.130
Z_TRAVEL = 0.150


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(Y_RAIL_T, Y_RAIL_L, Y_RAIL_H).edges("|Y").fillet(0.0025)
    slot = cq.Workplane("XY").box(Y_RAIL_T * 0.45, Y_RAIL_L * 0.82, Y_RAIL_H * 0.36).translate(
        (Y_RAIL_T * 0.12, 0.0, 0.0)
    )
    return rail.cut(slot)


def _z_rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(Z_RAIL_T, Z_RAIL_Y, Z_RAIL_Z).edges("|Z").fillet(0.002)
    slot = cq.Workplane("XY").box(Z_RAIL_T * 0.42, Z_RAIL_Y * 0.46, Z_RAIL_Z * 0.78).translate(
        (Z_RAIL_T * 0.10, 0.0, 0.0)
    )
    return rail.cut(slot)


def _gusset(y_center: float) -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (PLATE_T / 2.0 - 0.001, -0.090),
                (PLATE_T / 2.0 + 0.105, -0.090),
                (PLATE_T / 2.0 - 0.001, 0.070),
            ]
        )
        .close()
    )
    return profile.extrude(0.068).translate((0.0, y_center - 0.034, 0.0))


def _fixed_plate_body() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H)
    side_lip_pos = _box((0.032, 0.026, PLATE_H * 0.88), (0.021, (PLATE_W / 2.0) - 0.013, -0.010))
    side_lip_neg = _box((0.032, 0.026, PLATE_H * 0.88), (0.021, -(PLATE_W / 2.0) + 0.013, -0.010))
    top_cap = _box((0.032, PLATE_W * 0.76, 0.054), (0.020, 0.0, (PLATE_H / 2.0) - 0.050))
    support_pad = _box(
        (Y_SUPPORT_T, Y_SUPPORT_W, Y_SUPPORT_H),
        ((PLATE_T / 2.0) + (Y_SUPPORT_T / 2.0), 0.0, Y_SUPPORT_Z),
    )
    lower_shelf = _box((0.050, 0.200, 0.046), (0.031, 0.0, -0.005))
    return (
        plate.union(side_lip_pos)
        .union(side_lip_neg)
        .union(top_cap)
        .union(support_pad)
        .union(lower_shelf)
        .union(_gusset(0.180))
        .union(_gusset(-0.180))
    )


def _fixed_plate_rails() -> cq.Workplane:
    rail_x = (PLATE_T / 2.0) + Y_SUPPORT_T + (Y_RAIL_T / 2.0)
    top = _y_rail_shape().translate((rail_x, 0.0, Y_RAIL_TOP_Z))
    bottom = _y_rail_shape().translate((rail_x, 0.0, Y_RAIL_BOTTOM_Z))
    return top.union(bottom)


def _fixed_plate_stop(center_y: float) -> cq.Workplane:
    return _box(
        (Y_STOP_X, Y_STOP_Y, Y_STOP_Z),
        ((PLATE_T / 2.0) + Y_SUPPORT_T + (Y_STOP_X / 2.0) - 0.001, center_y, SADDLE_ORIGIN_Z),
    )


def _saddle_body() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(SADDLE_BRIDGE_X, SADDLE_BRIDGE_Y, SADDLE_BRIDGE_Z).edges("|Y").fillet(0.004)
    tower = _box(
        (SADDLE_TOWER_X, SADDLE_TOWER_Y, SADDLE_TOWER_Z),
        (SADDLE_TOWER_CENTER_X, 0.0, SADDLE_TOWER_CENTER_Z),
    )
    front_cover = _box((0.018, 0.100, 0.245), (0.020, 0.0, -0.195))
    lower_nose = _box(
        (SADDLE_NOSE_X, SADDLE_NOSE_Y, SADDLE_NOSE_Z),
        (SADDLE_NOSE_CENTER_X, 0.0, SADDLE_NOSE_CENTER_Z),
    )
    stop_bar = _box(
        (SADDLE_STOP_BAR_X, SADDLE_STOP_BAR_Y, SADDLE_STOP_BAR_Z),
        (SADDLE_STOP_BAR_CENTER_X, 0.0, SADDLE_STOP_BAR_CENTER_Z),
    )
    return bridge.union(tower).union(front_cover).union(lower_nose).union(stop_bar).union(
        _saddle_z_stop(-0.059)
    )


def _saddle_z_rails() -> cq.Workplane:
    right = _z_rail_shape().translate((Z_RAIL_CENTER_X, Z_RAIL_CENTER_Y, Z_RAIL_CENTER_Z))
    left = _z_rail_shape().translate((Z_RAIL_CENTER_X, -Z_RAIL_CENTER_Y, Z_RAIL_CENTER_Z))
    return right.union(left)


def _saddle_y_blocks() -> cq.Workplane:
    blocks = []
    for y_center in (-0.108, 0.108):
        for z_center in (Y_RAIL_TOP_Z - SADDLE_ORIGIN_Z, Y_RAIL_BOTTOM_Z - SADDLE_ORIGIN_Z):
            blocks.append(_box((Y_BLOCK_X, Y_BLOCK_Y, Y_BLOCK_Z), (Y_BLOCK_CENTER_X, y_center, z_center)))
    body = blocks[0]
    for block in blocks[1:]:
        body = body.union(block)
    return body


def _saddle_y_dog(center_y: float) -> cq.Workplane:
    return _box((0.018, 0.020, 0.032), (Y_BLOCK_CENTER_X + 0.006, center_y, 0.030))


def _saddle_z_stop(center_z: float) -> cq.Workplane:
    block = _box((0.014, 0.028, 0.014), (-0.014, 0.0, center_z))
    if center_z > -0.200:
        saddle_pad = _box((0.026, 0.028, 0.014), (-0.002, 0.0, center_z + 0.001))
        support = _box((0.014, 0.024, 0.038), (-0.002, 0.0, center_z + 0.022))
        return block.union(saddle_pad).union(support)
    return block


def _carriage_body() -> cq.Workplane:
    head = cq.Workplane("XY").box(CAR_HEAD_X, CAR_HEAD_Y, CAR_HEAD_Z).edges("|Y").fillet(0.0035)
    plate = _box((CAR_PLATE_X, CAR_PLATE_Y, CAR_PLATE_Z), (CAR_PLATE_CENTER_X, 0.0, CAR_PLATE_CENTER_Z))
    front_cover = _box(
        (CAR_FRONT_COVER_X, CAR_FRONT_COVER_Y, CAR_FRONT_COVER_Z),
        (CAR_FRONT_COVER_CENTER_X, 0.0, CAR_FRONT_COVER_CENTER_Z),
    )
    pad = _box((CAR_PAD_X, CAR_PAD_Y, CAR_PAD_Z), (CAR_PAD_CENTER_X, 0.0, CAR_PAD_CENTER_Z))
    side_cheek_pos = _box((0.014, 0.020, 0.210), (0.013, 0.049, -0.175))
    side_cheek_neg = _box((0.014, 0.020, 0.210), (0.013, -0.049, -0.175))
    return (
        head.union(plate)
        .union(front_cover)
        .union(pad)
        .union(side_cheek_pos)
        .union(side_cheek_neg)
        .union(_carriage_dog(BOTTOM_DOG_CENTER_X, BOTTOM_DOG_CENTER_Z, (BOTTOM_DOG_X, BOTTOM_DOG_Y, BOTTOM_DOG_Z)))
    )


def _carriage_z_blocks() -> cq.Workplane:
    block_pos = _box((CAR_BLOCK_X, CAR_BLOCK_Y, CAR_BLOCK_Z), (CAR_BLOCK_CENTER_X, Z_RAIL_CENTER_Y, -0.002))
    block_neg = _box((CAR_BLOCK_X, CAR_BLOCK_Y, CAR_BLOCK_Z), (CAR_BLOCK_CENTER_X, -Z_RAIL_CENTER_Y, -0.002))
    return block_pos.union(block_neg)


def _carriage_dog(center_x: float, center_z: float, size: tuple[float, float, float]) -> cq.Workplane:
    dog = _box(size, (center_x, 0.0, center_z))
    if center_z < -0.200:
        shoe = _box((0.028, 0.024, 0.016), (center_x + 0.010, 0.0, center_z + 0.018))
        hanger = _box((0.016, 0.024, 0.072), (center_x + 0.012, 0.0, center_z + 0.042))
        return dog.union(shoe).union(hanger)
    return dog


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_positioning_stage")

    model.material("painted_plate", rgba=(0.30, 0.34, 0.39, 1.0))
    model.material("machined_carriage", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("guide_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("cover_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("stop_black", rgba=(0.09, 0.09, 0.10, 1.0))

    fixed_plate = model.part("fixed_plate")
    fixed_plate.visual(
        mesh_from_cadquery(_fixed_plate_body(), "fixed_plate_body"),
        material="painted_plate",
        name="body",
    )
    fixed_plate.visual(
        mesh_from_cadquery(_fixed_plate_rails(), "fixed_plate_y_rails"),
        material="guide_steel",
        name="y_rails",
    )
    fixed_plate.visual(
        mesh_from_cadquery(_fixed_plate_stop(-Y_STOP_CENTER_Y), "fixed_plate_y_stop_left"),
        material="stop_black",
        name="y_stop_left",
    )
    fixed_plate.visual(
        mesh_from_cadquery(_fixed_plate_stop(Y_STOP_CENTER_Y), "fixed_plate_y_stop_right"),
        material="stop_black",
        name="y_stop_right",
    )
    fixed_plate.inertial = Inertial.from_geometry(
        Box((0.085, PLATE_W, PLATE_H)),
        mass=28.0,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_saddle_body(), "saddle_body"),
        material="machined_carriage",
        name="body",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_z_rails(), "saddle_z_rails"),
        material="guide_steel",
        name="z_rails",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_y_blocks(), "saddle_y_blocks"),
        material="machined_carriage",
        name="y_blocks",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_y_dog(-0.148), "saddle_y_dog_left"),
        material="stop_black",
        name="y_dog_left",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_y_dog(0.148), "saddle_y_dog_right"),
        material="stop_black",
        name="y_dog_right",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_z_stop(-0.059), "saddle_z_stop_top"),
        material="stop_black",
        name="z_stop_top",
    )
    saddle.visual(
        mesh_from_cadquery(_saddle_z_stop(-0.601), "saddle_z_stop_bottom"),
        material="stop_black",
        name="z_stop_bottom",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.125, SADDLE_BRIDGE_Y, 0.620)),
        mass=9.5,
        origin=Origin(xyz=(0.010, 0.0, -0.210)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body(), "carriage_body"),
        material="cover_dark",
        name="body",
    )
    carriage.visual(
        mesh_from_cadquery(_carriage_z_blocks(), "carriage_z_blocks"),
        material="machined_carriage",
        name="z_blocks",
    )
    carriage.visual(
        mesh_from_cadquery(
            _carriage_dog(TOP_DOG_CENTER_X, TOP_DOG_CENTER_Z, (TOP_DOG_X, TOP_DOG_Y, TOP_DOG_Z)),
            "carriage_z_dog_top",
        ),
        material="stop_black",
        name="z_dog_top",
    )
    carriage.visual(
        mesh_from_cadquery(
            _carriage_dog(
                BOTTOM_DOG_CENTER_X,
                BOTTOM_DOG_CENTER_Z,
                (BOTTOM_DOG_X, BOTTOM_DOG_Y, BOTTOM_DOG_Z),
            ),
            "carriage_z_dog_bottom",
        ),
        material="stop_black",
        name="z_dog_bottom",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.062, CAR_PAD_Y, 0.382)),
        mass=4.8,
        origin=Origin(xyz=(0.004, 0.0, -0.138)),
    )

    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_plate,
        child=saddle,
        origin=Origin(xyz=(SADDLE_ORIGIN_X, 0.0, SADDLE_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=1600.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(Z_JOINT_X, 0.0, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-Z_TRAVEL,
            upper=0.0,
            effort=900.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_plate = object_model.get_part("fixed_plate")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")

    y_stop_left = fixed_plate.get_visual("y_stop_left")
    y_stop_right = fixed_plate.get_visual("y_stop_right")
    y_dog_left = saddle.get_visual("y_dog_left")
    y_dog_right = saddle.get_visual("y_dog_right")
    z_stop_top = saddle.get_visual("z_stop_top")
    z_stop_bottom = saddle.get_visual("z_stop_bottom")
    z_dog_top = carriage.get_visual("z_dog_top")
    z_dog_bottom = carriage.get_visual("z_dog_bottom")
    y_blocks = saddle.get_visual("y_blocks")
    z_blocks = carriage.get_visual("z_blocks")
    y_rails = fixed_plate.get_visual("y_rails")
    z_rails = saddle.get_visual("z_rails")

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
        "y_axis_is_prismatic_left_right",
        y_slide.joint_type == ArticulationType.PRISMATIC and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected prismatic +Y axis, got {y_slide.joint_type} axis={y_slide.axis}",
    )
    ctx.check(
        "z_axis_is_prismatic_vertical",
        z_slide.joint_type == ArticulationType.PRISMATIC and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected prismatic +Z axis, got {z_slide.joint_type} axis={z_slide.axis}",
    )

    ctx.expect_contact(
        saddle,
        fixed_plate,
        elem_a=y_blocks,
        elem_b=y_rails,
        name="saddle_guides_seat_on_fixed_plate",
    )
    ctx.expect_contact(
        carriage,
        saddle,
        elem_a=z_blocks,
        elem_b=z_rails,
        name="carriage_guides_seat_on_hanging_saddle",
    )

    with ctx.pose({y_slide: y_slide.motion_limits.lower}):
        ctx.expect_within(
            saddle,
            fixed_plate,
            axes="y",
            margin=0.004,
            name="saddle_stays_within_plate_at_left_limit",
        )
        ctx.expect_gap(
            saddle,
            fixed_plate,
            axis="y",
            min_gap=0.003,
            max_gap=0.007,
            positive_elem=y_dog_left,
            negative_elem=y_stop_left,
            name="left_y_stop_has_hard_stop_clearance",
        )

    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        ctx.expect_within(
            saddle,
            fixed_plate,
            axes="y",
            margin=0.004,
            name="saddle_stays_within_plate_at_right_limit",
        )
        ctx.expect_gap(
            fixed_plate,
            saddle,
            axis="y",
            min_gap=0.003,
            max_gap=0.007,
            positive_elem=y_stop_right,
            negative_elem=y_dog_right,
            name="right_y_stop_has_hard_stop_clearance",
        )

    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        ctx.expect_gap(
            saddle,
            carriage,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem=z_stop_top,
            negative_elem=z_dog_top,
            name="top_z_stop_has_hard_stop_clearance",
        )

    with ctx.pose({z_slide: z_slide.motion_limits.lower}):
        ctx.expect_within(
            carriage,
            fixed_plate,
            axes="z",
            margin=0.006,
            name="carriage_stays_within_plate_height_at_lower_limit",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem=z_dog_bottom,
            negative_elem=z_stop_bottom,
            name="bottom_z_stop_has_hard_stop_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
