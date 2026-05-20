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


COLUMN_CENTER_X = 0.33
COLUMN_W = 0.11
COLUMN_D = 0.16
COLUMN_H = 1.50
FRAME_WALL = 0.008

FOOT_W = 0.20
FOOT_D = 0.22
FOOT_T = 0.02

BOTTOM_MEMBER_L = 0.60
BOTTOM_MEMBER_D = 0.12
BOTTOM_MEMBER_H = 0.10
BOTTOM_MEMBER_Z = 0.16

TOP_MEMBER_L = 0.60
TOP_MEMBER_D = 0.12
TOP_MEMBER_H = 0.08
TOP_MEMBER_Z = 1.50

GUIDE_ROD_R = 0.014
GUIDE_ROD_X = 0.21
GUIDE_ROD_Y = 0.00
GUIDE_ROD_Z0 = 0.24
GUIDE_ROD_Z1 = 1.42
GUIDE_ROD_LEN = GUIDE_ROD_Z1 - GUIDE_ROD_Z0
GUIDE_ROD_CENTER_Z = 0.5 * (GUIDE_ROD_Z0 + GUIDE_ROD_Z1)

LIFT_SCREW_R = 0.012
LIFT_SCREW_Y = -0.03
LIFT_SCREW_Z0 = 0.22
LIFT_SCREW_Z1 = 1.44
LIFT_SCREW_LEN = LIFT_SCREW_Z1 - LIFT_SCREW_Z0
LIFT_SCREW_CENTER_Z = 0.5 * (LIFT_SCREW_Z0 + LIFT_SCREW_Z1)

SYNC_SHAFT_R = 0.011
SYNC_SHAFT_LEN = 0.54
SYNC_SHAFT_Y = -0.075
TOP_SYNC_SHAFT_Z = 1.54
BOTTOM_SYNC_SHAFT_Z = 0.16

CARRIAGE_W = 0.46
CARRIAGE_D = 0.14
CARRIAGE_H = 0.54
CARRIAGE_LOWER_Z = 0.58
CARRIAGE_TRAVEL = 0.48


def _hollow_box(size_x: float, size_y: float, size_z: float, wall: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).cut(
        cq.Workplane("XY").box(size_x - 2 * wall, size_y - 2 * wall, size_z + 0.004)
    )


def _x_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((-length / 2, 0.0, 0.0))


def _y_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((0.0, -length / 2, 0.0))


def _z_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2))


def _z_tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2))
    )


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _make_frame() -> cq.Workplane:
    left_column = _hollow_box(COLUMN_W, COLUMN_D, COLUMN_H, FRAME_WALL).translate(
        (-COLUMN_CENTER_X, 0.0, COLUMN_H / 2)
    )
    right_column = _hollow_box(COLUMN_W, COLUMN_D, COLUMN_H, FRAME_WALL).translate(
        (COLUMN_CENTER_X, 0.0, COLUMN_H / 2)
    )

    left_foot = cq.Workplane("XY").box(FOOT_W, FOOT_D, FOOT_T).translate(
        (-COLUMN_CENTER_X, 0.0, FOOT_T / 2)
    )
    right_foot = cq.Workplane("XY").box(FOOT_W, FOOT_D, FOOT_T).translate(
        (COLUMN_CENTER_X, 0.0, FOOT_T / 2)
    )

    bottom_member = _hollow_box(
        BOTTOM_MEMBER_L, BOTTOM_MEMBER_D, BOTTOM_MEMBER_H, FRAME_WALL
    ).translate((0.0, 0.0, BOTTOM_MEMBER_Z))
    top_member = _hollow_box(TOP_MEMBER_L, TOP_MEMBER_D, TOP_MEMBER_H, FRAME_WALL).translate(
        (0.0, 0.0, TOP_MEMBER_Z)
    )

    frame = (
        left_column.union(right_column)
        .union(left_foot)
        .union(right_foot)
        .union(bottom_member)
        .union(top_member)
    )

    rod_bracket_w = 0.052
    rod_bracket_d = 0.074
    rod_bracket_h = 0.092
    rod_bracket_x = 0.5 * ((COLUMN_CENTER_X - COLUMN_W / 2) + GUIDE_ROD_X)

    for x_sign in (-1.0, 1.0):
        for zc in (GUIDE_ROD_Z0, GUIDE_ROD_Z1):
            bracket = cq.Workplane("XY").box(
                rod_bracket_w, rod_bracket_d, rod_bracket_h
            ).translate((x_sign * rod_bracket_x, GUIDE_ROD_Y, zc))
            rod_bore = _z_cylinder(rod_bracket_h + 0.006, GUIDE_ROD_R).translate(
                (x_sign * GUIDE_ROD_X, GUIDE_ROD_Y, zc)
            )
            gusset = cq.Workplane("XY").box(0.018, 0.074, 0.16).translate(
                (
                    x_sign * (COLUMN_CENTER_X - COLUMN_W / 2 + 0.010),
                    GUIDE_ROD_Y,
                    zc - 0.020 * (1.0 if zc > 0.8 else -1.0),
                )
            )
            frame = frame.union(bracket.cut(rod_bore)).union(gusset)

    top_gearbox = cq.Workplane("XY").box(0.14, 0.10, 0.10).translate(
        (0.0, -0.055, TOP_MEMBER_Z + 0.01)
    )
    bottom_gearbox = cq.Workplane("XY").box(0.14, 0.10, 0.10).translate(
        (0.0, -0.055, BOTTOM_MEMBER_Z)
    )
    top_screw_bore = _z_cylinder(0.12, LIFT_SCREW_R).translate(
        (0.0, LIFT_SCREW_Y, TOP_MEMBER_Z + 0.01)
    )
    bottom_screw_bore = _z_cylinder(0.12, LIFT_SCREW_R).translate(
        (0.0, LIFT_SCREW_Y, BOTTOM_MEMBER_Z)
    )
    frame = frame.union(top_gearbox.cut(top_screw_bore)).union(
        bottom_gearbox.cut(bottom_screw_bore)
    )

    for shaft_z in (TOP_SYNC_SHAFT_Z, BOTTOM_SYNC_SHAFT_Z):
        for xc in (-0.25, 0.0, 0.25):
            housing = cq.Workplane("XY").box(0.09, 0.10, 0.08).translate(
                (xc, SYNC_SHAFT_Y, shaft_z)
            )
            shaft_bore = _x_cylinder(0.11, SYNC_SHAFT_R).translate((xc, SYNC_SHAFT_Y, shaft_z))
            frame = frame.union(housing.cut(shaft_bore))

    left_motor_pad = cq.Workplane("XY").box(0.09, 0.08, 0.14).translate(
        (-COLUMN_CENTER_X + 0.015, -0.045, 0.30)
    )
    right_motor_pad = cq.Workplane("XY").box(0.09, 0.08, 0.14).translate(
        (COLUMN_CENTER_X - 0.015, -0.045, 0.30)
    )
    frame = frame.union(left_motor_pad).union(right_motor_pad)

    return frame


def _make_guide_rod() -> cq.Workplane:
    body = _z_cylinder(GUIDE_ROD_LEN, GUIDE_ROD_R)
    lower_land = _z_cylinder(0.030, 0.019).translate((0.0, 0.0, -GUIDE_ROD_LEN / 2 + 0.015))
    upper_land = _z_cylinder(0.030, 0.019).translate((0.0, 0.0, GUIDE_ROD_LEN / 2 - 0.015))
    return body.union(lower_land).union(upper_land)


def _make_lift_screw() -> cq.Workplane:
    main = _z_cylinder(LIFT_SCREW_LEN - 0.10, LIFT_SCREW_R)
    top_journal = _z_cylinder(0.050, 0.008).translate((0.0, 0.0, LIFT_SCREW_LEN / 2 - 0.025))
    bottom_journal = _z_cylinder(0.050, 0.008).translate(
        (0.0, 0.0, -LIFT_SCREW_LEN / 2 + 0.025)
    )
    upper_hub = _z_cylinder(0.020, 0.026).translate((0.0, 0.0, LIFT_SCREW_LEN / 2 - 0.090))
    lower_hub = _z_cylinder(0.020, 0.026).translate((0.0, 0.0, -LIFT_SCREW_LEN / 2 + 0.090))
    return main.union(top_journal).union(bottom_journal).union(upper_hub).union(lower_hub)


def _make_sync_shaft() -> cq.Workplane:
    shaft = _x_cylinder(SYNC_SHAFT_LEN, SYNC_SHAFT_R)
    center_hub = _x_cylinder(0.070, 0.025)
    left_hub = _x_cylinder(0.030, 0.020).translate((-0.25, 0.0, 0.0))
    right_hub = _x_cylinder(0.030, 0.020).translate((0.25, 0.0, 0.0))
    return shaft.union(center_hub).union(left_hub).union(right_hub)


def _make_carriage() -> cq.Workplane:
    front_plate = cq.Workplane("XY").box(0.28, 0.020, 0.38).translate((0.0, 0.055, 0.0))
    rear_plate = cq.Workplane("XY").box(0.20, 0.016, 0.34).translate((0.0, -0.045, 0.0))
    top_plate = cq.Workplane("XY").box(0.38, 0.090, 0.020).translate((0.0, 0.005, 0.230))
    bottom_plate = cq.Workplane("XY").box(0.38, 0.090, 0.020).translate((0.0, 0.005, -0.230))
    center_spine = cq.Workplane("XY").box(0.09, 0.090, 0.40).translate((0.0, 0.000, 0.0))

    carriage = (
        front_plate.union(rear_plate)
        .union(top_plate)
        .union(bottom_plate)
        .union(center_spine)
    )

    for x_sign in (-1.0, 1.0):
        side_web = cq.Workplane("XY").box(0.055, 0.105, 0.33).translate((x_sign * GUIDE_ROD_X, 0.0, 0.0))
        upper_bushing = _z_tube(0.075, 0.032, GUIDE_ROD_R).translate(
            (x_sign * GUIDE_ROD_X, 0.0, 0.180)
        )
        lower_bushing = _z_tube(0.075, 0.032, GUIDE_ROD_R).translate(
            (x_sign * GUIDE_ROD_X, 0.0, -0.180)
        )
        upper_pad = cq.Workplane("XY").box(0.085, 0.080, 0.080).translate(
            (x_sign * GUIDE_ROD_X, 0.0, 0.180)
        )
        lower_pad = cq.Workplane("XY").box(0.085, 0.080, 0.080).translate(
            (x_sign * GUIDE_ROD_X, 0.0, -0.180)
        )
        slot = cq.Workplane("XY").box(0.024, 0.120, 0.140).translate((x_sign * GUIDE_ROD_X, 0.0, 0.0))
        guide_bore = _z_cylinder(0.44, GUIDE_ROD_R).translate((x_sign * GUIDE_ROD_X, 0.0, 0.0))
        carriage = (
            carriage.union(side_web)
            .union(upper_bushing)
            .union(lower_bushing)
            .union(upper_pad)
            .union(lower_pad)
            .cut(slot)
            .cut(guide_bore)
        )

    front_window = cq.Workplane("XY").box(0.14, 0.040, 0.21).translate((0.0, 0.055, 0.0))
    screw_bore = _z_cylinder(CARRIAGE_H + 0.020, LIFT_SCREW_R).translate((0.0, LIFT_SCREW_Y, 0.0))
    center_window = cq.Workplane("XY").box(0.06, 0.120, 0.20).translate((0.0, 0.000, 0.0))
    return carriage.cut(front_window).cut(screw_bore).cut(center_window)


def _make_front_cover() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.21, 0.010, 0.29)
    opening = cq.Workplane("XY").box(0.07, 0.020, 0.13).translate((0.0, 0.0, 0.0))
    cover = plate.cut(opening)
    for x in (-0.085, 0.085):
        for z in (-0.115, -0.040, 0.040, 0.115):
            cover = cover.union(
                _y_cylinder(0.008, 0.006).translate((x, 0.009, z))
            )
    return cover


def _make_service_cover() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.18, 0.010, 0.18)
    rib_top = cq.Workplane("XY").box(0.14, 0.012, 0.020).translate((0.0, 0.002, 0.050))
    rib_bottom = cq.Workplane("XY").box(0.14, 0.012, 0.020).translate((0.0, 0.002, -0.050))
    cover = plate.union(rib_top).union(rib_bottom)
    for x in (-0.070, 0.070):
        for z in (-0.060, 0.060):
            cover = cover.union(_y_cylinder(0.008, 0.006).translate((x, 0.009, z)))
    return cover


def _make_stop(hand: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(0.060, 0.040, 0.020)
    tab = cq.Workplane("XY").box(0.030, 0.040, 0.040).translate((hand * 0.040, 0.0, 0.0))
    bolt = _y_cylinder(0.008, 0.005).translate((0.0, 0.009, 0.0))
    return block.union(tab).union(bolt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage", assets=ASSETS)

    frame_mat = model.material("frame_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    carriage_mat = model.material("carriage_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    rod_mat = model.material("rod_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    shaft_mat = model.material("shaft_oxide", rgba=(0.18, 0.18, 0.20, 1.0))
    cover_mat = model.material("cover_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    stop_mat = model.material("stop_gray", rgba=(0.36, 0.38, 0.40, 1.0))

    frame = model.part("frame")
    frame.visual(_mesh(_make_frame(), "frame.obj"), material=frame_mat, name="frame_body")
    frame.inertial = Inertial.from_geometry(
        Box((0.90, 0.24, 1.56)),
        mass=88.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    left_guide_rod = model.part("left_guide_rod")
    left_guide_rod.visual(_mesh(_make_guide_rod(), "left_guide_rod.obj"), material=rod_mat, name="rod_body")
    left_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=GUIDE_ROD_LEN),
        mass=3.0,
    )

    right_guide_rod = model.part("right_guide_rod")
    right_guide_rod.visual(
        _mesh(_make_guide_rod(), "right_guide_rod.obj"),
        material=rod_mat,
        name="rod_body",
    )
    right_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=GUIDE_ROD_LEN),
        mass=3.0,
    )

    lift_screw = model.part("lift_screw")
    lift_screw.visual(_mesh(_make_lift_screw(), "lift_screw.obj"), material=shaft_mat, name="screw_body")
    lift_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=LIFT_SCREW_LEN),
        mass=4.2,
    )

    top_sync_shaft = model.part("top_sync_shaft")
    top_sync_shaft.visual(
        _mesh(_make_sync_shaft(), "top_sync_shaft.obj"),
        material=shaft_mat,
        name="shaft_body",
    )
    top_sync_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=SYNC_SHAFT_LEN),
        mass=3.2,
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
    )

    bottom_sync_shaft = model.part("bottom_sync_shaft")
    bottom_sync_shaft.visual(
        _mesh(_make_sync_shaft(), "bottom_sync_shaft.obj"),
        material=shaft_mat,
        name="shaft_body",
    )
    bottom_sync_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=SYNC_SHAFT_LEN),
        mass=3.2,
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(_mesh(_make_carriage(), "carriage.obj"), material=carriage_mat, name="carriage_body")
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)),
        mass=22.0,
    )

    carriage_front_cover = model.part("carriage_front_cover")
    carriage_front_cover.visual(
        _mesh(_make_front_cover(), "carriage_front_cover.obj"),
        material=cover_mat,
        name="cover_body",
    )
    carriage_front_cover.inertial = Inertial.from_geometry(Box((0.21, 0.018, 0.29)), mass=1.8)

    lower_service_cover = model.part("lower_service_cover")
    lower_service_cover.visual(
        _mesh(_make_service_cover(), "lower_service_cover.obj"),
        material=cover_mat,
        name="cover_body",
    )
    lower_service_cover.inertial = Inertial.from_geometry(Box((0.18, 0.018, 0.18)), mass=1.4)

    top_left_stop = model.part("top_left_stop")
    top_left_stop.visual(_mesh(_make_stop(-1.0), "top_left_stop.obj"), material=stop_mat, name="stop_body")
    top_left_stop.inertial = Inertial.from_geometry(Box((0.10, 0.04, 0.04)), mass=0.8)

    top_right_stop = model.part("top_right_stop")
    top_right_stop.visual(
        _mesh(_make_stop(1.0), "top_right_stop.obj"),
        material=stop_mat,
        name="stop_body",
    )
    top_right_stop.inertial = Inertial.from_geometry(Box((0.10, 0.04, 0.04)), mass=0.8)

    bottom_left_stop = model.part("bottom_left_stop")
    bottom_left_stop.visual(
        _mesh(_make_stop(-1.0), "bottom_left_stop.obj"),
        material=stop_mat,
        name="stop_body",
    )
    bottom_left_stop.inertial = Inertial.from_geometry(Box((0.10, 0.04, 0.04)), mass=0.8)

    bottom_right_stop = model.part("bottom_right_stop")
    bottom_right_stop.visual(
        _mesh(_make_stop(1.0), "bottom_right_stop.obj"),
        material=stop_mat,
        name="stop_body",
    )
    bottom_right_stop.inertial = Inertial.from_geometry(Box((0.10, 0.04, 0.04)), mass=0.8)

    model.articulation(
        "frame_to_left_guide_rod",
        ArticulationType.FIXED,
        parent=frame,
        child=left_guide_rod,
        origin=Origin(xyz=(-GUIDE_ROD_X, GUIDE_ROD_Y, GUIDE_ROD_CENTER_Z)),
    )
    model.articulation(
        "frame_to_right_guide_rod",
        ArticulationType.FIXED,
        parent=frame,
        child=right_guide_rod,
        origin=Origin(xyz=(GUIDE_ROD_X, GUIDE_ROD_Y, GUIDE_ROD_CENTER_Z)),
    )
    model.articulation(
        "lift_screw_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=lift_screw,
        origin=Origin(xyz=(0.0, LIFT_SCREW_Y, LIFT_SCREW_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )
    model.articulation(
        "top_sync_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=top_sync_shaft,
        origin=Origin(xyz=(0.0, SYNC_SHAFT_Y, TOP_SYNC_SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=10.0),
    )
    model.articulation(
        "bottom_sync_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=bottom_sync_shaft,
        origin=Origin(xyz=(0.0, SYNC_SHAFT_Y, BOTTOM_SYNC_SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=10.0),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_front_cover",
        ArticulationType.FIXED,
        parent=carriage,
        child=carriage_front_cover,
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
    )
    model.articulation(
        "frame_to_lower_service_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_service_cover,
        origin=Origin(xyz=(0.0, 0.065, BOTTOM_MEMBER_Z)),
    )
    model.articulation(
        "frame_to_top_left_stop",
        ArticulationType.FIXED,
        parent=frame,
        child=top_left_stop,
        origin=Origin(xyz=(-GUIDE_ROD_X, 0.055, 1.360)),
    )
    model.articulation(
        "frame_to_top_right_stop",
        ArticulationType.FIXED,
        parent=frame,
        child=top_right_stop,
        origin=Origin(xyz=(GUIDE_ROD_X, 0.055, 1.360)),
    )
    model.articulation(
        "frame_to_bottom_left_stop",
        ArticulationType.FIXED,
        parent=frame,
        child=bottom_left_stop,
        origin=Origin(xyz=(-GUIDE_ROD_X, 0.055, 0.280)),
    )
    model.articulation(
        "frame_to_bottom_right_stop",
        ArticulationType.FIXED,
        parent=frame,
        child=bottom_right_stop,
        origin=Origin(xyz=(GUIDE_ROD_X, 0.055, 0.280)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_guide_rod = object_model.get_part("left_guide_rod")
    right_guide_rod = object_model.get_part("right_guide_rod")
    lift_screw = object_model.get_part("lift_screw")
    top_sync_shaft = object_model.get_part("top_sync_shaft")
    bottom_sync_shaft = object_model.get_part("bottom_sync_shaft")
    carriage = object_model.get_part("carriage")
    carriage_front_cover = object_model.get_part("carriage_front_cover")
    lower_service_cover = object_model.get_part("lower_service_cover")
    top_left_stop = object_model.get_part("top_left_stop")
    top_right_stop = object_model.get_part("top_right_stop")
    bottom_left_stop = object_model.get_part("bottom_left_stop")
    bottom_right_stop = object_model.get_part("bottom_right_stop")

    carriage_slide = object_model.get_articulation("carriage_slide")
    lift_screw_spin = object_model.get_articulation("lift_screw_spin")
    top_sync_spin = object_model.get_articulation("top_sync_spin")
    bottom_sync_spin = object_model.get_articulation("bottom_sync_spin")

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

    ctx.expect_contact(left_guide_rod, frame, name="left guide rod captured in frame brackets")
    ctx.expect_contact(right_guide_rod, frame, name="right guide rod captured in frame brackets")
    ctx.expect_contact(lift_screw, frame, name="lift screw supported by frame gearbox blocks")
    ctx.expect_contact(top_sync_shaft, frame, name="top synchronizing shaft seated in housings")
    ctx.expect_contact(bottom_sync_shaft, frame, name="bottom synchronizing shaft seated in housings")
    ctx.expect_contact(carriage, left_guide_rod, name="carriage left bushings ride on left guide rod")
    ctx.expect_contact(carriage, right_guide_rod, name="carriage right bushings ride on right guide rod")
    ctx.expect_contact(carriage, lift_screw, name="carriage nut path stays on lift screw")
    ctx.expect_contact(
        carriage_front_cover,
        carriage,
        name="carriage front access cover mounts flush to carriage",
    )
    ctx.expect_contact(
        lower_service_cover,
        frame,
        name="lower service cover mounts flush to bottom crossmember",
    )
    ctx.expect_contact(top_left_stop, frame, name="top left hard stop mounted to frame")
    ctx.expect_contact(top_right_stop, frame, name="top right hard stop mounted to frame")
    ctx.expect_contact(bottom_left_stop, frame, name="bottom left hard stop mounted to frame")
    ctx.expect_contact(bottom_right_stop, frame, name="bottom right hard stop mounted to frame")
    ctx.expect_origin_distance(
        carriage,
        lift_screw,
        axes="xy",
        max_dist=0.035,
        name="carriage stays aligned to central lift screw axis",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes=("x", "y"),
        margin=0.0,
        name="carriage stays laterally inside support frame envelope",
    )

    with ctx.pose({carriage_slide: 0.0}):
        ctx.expect_gap(
            carriage,
            bottom_left_stop,
            axis="z",
            min_gap=0.015,
            max_gap=0.040,
            name="left lower stop leaves controlled rest clearance",
        )
        ctx.expect_gap(
            carriage,
            bottom_right_stop,
            axis="z",
            min_gap=0.015,
            max_gap=0.040,
            name="right lower stop leaves controlled rest clearance",
        )

    with ctx.pose({carriage_slide: CARRIAGE_TRAVEL}):
        ctx.expect_gap(
            top_left_stop,
            carriage,
            axis="z",
            min_gap=0.015,
            max_gap=0.040,
            name="left upper stop brackets top out carriage without collision",
        )
        ctx.expect_gap(
            top_right_stop,
            carriage,
            axis="z",
            min_gap=0.015,
            max_gap=0.040,
            name="right upper stop brackets top out carriage without collision",
        )
        ctx.expect_contact(
            carriage,
            left_guide_rod,
            name="left guide contact remains through full carriage travel",
        )
        ctx.expect_contact(
            carriage,
            right_guide_rod,
            name="right guide contact remains through full carriage travel",
        )

    with ctx.pose(
        {
            carriage_slide: 0.26,
            lift_screw_spin: 2.1,
            top_sync_spin: 1.2,
            bottom_sync_spin: -0.9,
        }
    ):
        ctx.expect_contact(
            carriage,
            lift_screw,
            name="central drive path remains engaged at operating pose",
        )
        ctx.expect_contact(
            top_sync_shaft,
            frame,
            name="top synchronizing shaft remains supported while rotating",
        )
        ctx.expect_contact(
            bottom_sync_shaft,
            frame,
            name="bottom synchronizing shaft remains supported while rotating",
        )
        ctx.expect_origin_distance(
            carriage,
            lift_screw,
            axes="xy",
            max_dist=0.035,
            name="carriage remains centered on screw during motion",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
