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

BASE_L = 0.34
BASE_W = 0.14
BASE_CORE_W = 0.092
BASE_T = 0.018
EAR_HOLE_D = 0.011

RAIL_L = 0.27
RAIL_CAP_W = 0.018
RAIL_FOOT_W = 0.024
RAIL_H = 0.024
RAIL_FOOT_H = 0.006
RAIL_Y = 0.037

CARR_L = 0.118
CARR_W = 0.112
CARR_TOP_T = 0.014
CARR_POD_W = 0.028
CARR_POD_H = 0.040
CARR_Z = 0.045
CARR_SLOT_W = 0.020
CARR_SLOT_H = 0.028
CARR_OPEN_L = 0.060
CARR_OPEN_W = 0.050
TRAVEL = 0.075

COVER_L = 0.066
COVER_W = 0.056
COVER_T = 0.004

STOP_L = 0.012
STOP_W = 0.056
STOP_H = 0.026
STOP_FOOT_L = STOP_L + 0.010

BELLOWS_H = 0.018
BELLOWS_W = 0.052
STOP_X = 0.153
BELLOWS_L = STOP_X - (STOP_FOOT_L / 2.0) - (CARR_L / 2.0)
BELLOWS_CENTER_X = (STOP_X - (STOP_FOOT_L / 2.0) + (CARR_L / 2.0)) / 2.0


def _box_xy(length: float, width: float, height: float, *, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(True, True, False),
    ).translate((0.0, 0.0, z0))


def _make_base_shape() -> cq.Workplane:
    notch_w = (BASE_W - BASE_CORE_W) / 2.0 + 0.002
    notch_y = BASE_W / 2.0 - notch_w / 2.0

    shape = _box_xy(BASE_L, BASE_W, BASE_T)
    side_notch = _box_xy(BASE_L * 0.52, notch_w, BASE_T + 0.004, z0=-0.002)
    for y in (-notch_y, notch_y):
        shape = shape.cut(side_notch.translate((0.0, y, 0.0)))

    shape = shape.cut(_box_xy(0.204, 0.050, 0.006, z0=BASE_T - 0.006))
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(
        [
            (-0.126, -0.055),
            (-0.126, 0.055),
            (0.126, -0.055),
            (0.126, 0.055),
        ]
    ).hole(EAR_HOLE_D)
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(
        [(-0.082, 0.0), (0.082, 0.0)]
    ).slot2D(0.024, 0.009, angle=0.0).cutBlind(-0.006)

    corner_cutter = (
        _box_xy(0.018, 0.018, BASE_T + 0.004, z0=-0.002)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 45.0)
    )
    for x in (-BASE_L / 2.0 + 0.010, BASE_L / 2.0 - 0.010):
        for y in (-BASE_W / 2.0 + 0.010, BASE_W / 2.0 - 0.010):
            shape = shape.cut(corner_cutter.translate((x, y, 0.0)))
    return shape


def _make_rail_shape() -> cq.Workplane:
    shape = _box_xy(RAIL_L, RAIL_FOOT_W, RAIL_FOOT_H)
    shape = shape.union(_box_xy(RAIL_L, RAIL_CAP_W, RAIL_H - RAIL_FOOT_H, z0=RAIL_FOOT_H))
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(
        [(-0.090, 0.0), (-0.030, 0.0), (0.030, 0.0), (0.090, 0.0)]
    ).cskHole(0.0052, 0.0105, 90)
    return shape


def _make_carriage_shape() -> cq.Workplane:
    top_frame = cq.Workplane("XY").box(CARR_L, CARR_W, CARR_TOP_T, centered=True).translate((0.0, 0.0, 0.013))
    shape = top_frame

    for rail_y in (-RAIL_Y, RAIL_Y):
        pod = cq.Workplane("XY").box(CARR_L - 0.008, CARR_POD_W, CARR_POD_H, centered=True).translate((0.0, rail_y, 0.0))
        slot = cq.Workplane("XY").box(CARR_L + 0.004, CARR_SLOT_W, CARR_SLOT_H, centered=True).translate((0.0, rail_y, -0.011))
        shape = shape.union(pod).cut(slot)

    shape = shape.cut(
        cq.Workplane("XY").box(CARR_OPEN_L, CARR_OPEN_W, CARR_TOP_T + 0.008, centered=True).translate((0.0, 0.0, 0.013))
    )
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(
        [(-0.022, -0.018), (-0.022, 0.018), (0.022, -0.018), (0.022, 0.018)]
    ).cskHole(0.0042, 0.0085, 90, depth=CARR_TOP_T)

    for x in (-0.026, 0.026):
        shape = shape.cut(
            cq.Workplane("XY").box(0.026, 0.012, 0.020, centered=True).translate((x, RAIL_Y + 0.008, -0.002))
        )
        shape = shape.cut(
            cq.Workplane("XY").box(0.026, 0.012, 0.020, centered=True).translate((x, -RAIL_Y - 0.008, -0.002))
        )

    return shape


def _make_cover_shape() -> cq.Workplane:
    shape = _box_xy(COVER_L, COVER_W, COVER_T)
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").rect(COVER_L - 0.014, COVER_W - 0.014).cutBlind(-0.0012)
    shape = shape.faces(">Z").workplane(centerOption="CenterOfBoundBox").slot2D(0.020, 0.006, angle=0.0).cutBlind(-0.0012)
    bolt_points = [
        (-0.022, -0.018),
        (-0.022, 0.018),
        (0.022, -0.018),
        (0.022, 0.018),
    ]
    bolts = (
        cq.Workplane("XY")
        .pushPoints(bolt_points)
        .circle(0.0035)
        .extrude(0.003)
        .translate((0.0, 0.0, COVER_T))
    )
    return shape.union(bolts)


def _make_stop_shape() -> cq.Workplane:
    foot = _box_xy(STOP_FOOT_L, STOP_W + 0.014, 0.006)
    block = _box_xy(STOP_L, STOP_W, STOP_H, z0=0.006)
    wedge = (
        _box_xy(0.016, STOP_W + 0.004, 0.018, z0=0.014)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 22.0)
        .translate((0.004, 0.0, 0.0))
    )
    bolts = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.016), (0.0, 0.016)])
        .circle(0.0035)
        .extrude(0.003)
        .translate((0.0, 0.0, 0.006 + STOP_H))
    )
    return foot.union(block).cut(wedge).union(bolts)


def _make_bellows_shape(length: float) -> cq.Workplane:
    folds = 6
    pitch = length / folds
    x = -length / 2.0
    points: list[tuple[float, float]] = [(x, 0.0), (x, 0.004)]
    for _ in range(folds):
        points.append((x + pitch * 0.5, BELLOWS_H))
        x += pitch
        points.append((x, 0.004))
    points.append((length / 2.0, 0.0))
    shape = cq.Workplane("XZ").polyline(points).close().extrude(BELLOWS_W / 2.0, both=True)
    left_flange = cq.Workplane("XY").box(0.003, BELLOWS_W, BELLOWS_H, centered=(True, True, False)).translate(
        (-length / 2.0 + 0.0015, 0.0, 0.0)
    )
    right_flange = cq.Workplane("XY").box(0.003, BELLOWS_W, BELLOWS_H, centered=(True, True, False)).translate(
        (length / 2.0 - 0.0015, 0.0, 0.0)
    )
    return shape.union(left_flange).union(right_flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_slider_study", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    coated_plate = model.material("coated_plate", rgba=(0.18, 0.20, 0.22, 1.0))
    zinc = model.material("zinc", rgba=(0.78, 0.79, 0.80, 1.0))
    bellows_black = model.material("bellows_black", rgba=(0.16, 0.16, 0.15, 1.0))
    bronze = model.material("bronze", rgba=(0.55, 0.46, 0.27, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base.obj", assets=ASSETS),
        material=coated_plate,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    left_rail = model.part("left_rail")
    left_rail.visual(
        mesh_from_cadquery(_make_rail_shape(), "left_rail.obj", assets=ASSETS),
        material=ground_steel,
        name="rail_body",
    )
    left_rail.inertial = Inertial.from_geometry(
        Box((RAIL_L, RAIL_FOOT_W, RAIL_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, RAIL_H / 2.0)),
    )

    right_rail = model.part("right_rail")
    right_rail.visual(
        mesh_from_cadquery(_make_rail_shape(), "right_rail.obj", assets=ASSETS),
        material=ground_steel,
        name="rail_body",
    )
    right_rail.inertial = Inertial.from_geometry(
        Box((RAIL_L, RAIL_FOOT_W, RAIL_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, RAIL_H / 2.0)),
    )

    left_stop = model.part("left_stop")
    left_stop.visual(
        mesh_from_cadquery(_make_stop_shape(), "left_stop.obj", assets=ASSETS),
        material=dark_steel,
        name="stop_body",
    )
    left_stop.inertial = Inertial.from_geometry(
        Box((STOP_L + 0.010, STOP_W + 0.014, STOP_H + 0.006)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (STOP_H + 0.006) / 2.0)),
    )

    right_stop = model.part("right_stop")
    right_stop.visual(
        mesh_from_cadquery(_make_stop_shape(), "right_stop.obj", assets=ASSETS),
        material=dark_steel,
        name="stop_body",
    )
    right_stop.inertial = Inertial.from_geometry(
        Box((STOP_L + 0.010, STOP_W + 0.014, STOP_H + 0.006)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (STOP_H + 0.006) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage.obj", assets=ASSETS),
        material=dark_steel,
        name="carriage_body",
    )
    carriage.visual(
        Box((CARR_L - 0.012, CARR_SLOT_W - 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -RAIL_Y, 0.0015)),
        material=bronze,
        name="left_bearing_roof",
    )
    carriage.visual(
        Box((CARR_L - 0.012, CARR_SLOT_W - 0.002, 0.004)),
        origin=Origin(xyz=(0.0, RAIL_Y, 0.0015)),
        material=bronze,
        name="right_bearing_roof",
    )
    carriage.visual(
        Box((0.072, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=zinc,
        name="tooling_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_L, CARR_W, CARR_POD_H)),
        mass=2.1,
        origin=Origin(),
    )

    access_cover = model.part("access_cover")
    access_cover.visual(
        mesh_from_cadquery(_make_cover_shape(), "access_cover.obj", assets=ASSETS),
        material=zinc,
        name="cover_plate",
    )
    access_cover.inertial = Inertial.from_geometry(
        Box((COVER_L, COVER_W, COVER_T + 0.003)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, (COVER_T + 0.003) / 2.0)),
    )

    left_bellows = model.part("left_bellows")
    left_bellows.visual(
        mesh_from_cadquery(_make_bellows_shape(BELLOWS_L), "left_bellows.obj", assets=ASSETS),
        material=bellows_black,
        name="bellows_shell",
    )
    left_bellows.inertial = Inertial.from_geometry(
        Box((BELLOWS_L, BELLOWS_W, BELLOWS_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BELLOWS_H / 2.0)),
    )

    right_bellows = model.part("right_bellows")
    right_bellows.visual(
        mesh_from_cadquery(_make_bellows_shape(BELLOWS_L), "right_bellows.obj", assets=ASSETS),
        material=bellows_black,
        name="bellows_shell",
    )
    right_bellows.inertial = Inertial.from_geometry(
        Box((BELLOWS_L, BELLOWS_W, BELLOWS_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BELLOWS_H / 2.0)),
    )

    model.articulation(
        "base_to_left_rail",
        ArticulationType.FIXED,
        parent=base,
        child=left_rail,
        origin=Origin(xyz=(0.0, -RAIL_Y, BASE_T)),
    )
    model.articulation(
        "base_to_right_rail",
        ArticulationType.FIXED,
        parent=base,
        child=right_rail,
        origin=Origin(xyz=(0.0, RAIL_Y, BASE_T)),
    )
    model.articulation(
        "base_to_left_stop",
        ArticulationType.FIXED,
        parent=base,
        child=left_stop,
        origin=Origin(xyz=(-STOP_X, 0.0, BASE_T)),
    )
    model.articulation(
        "base_to_right_stop",
        ArticulationType.FIXED,
        parent=base,
        child=right_stop,
        origin=Origin(xyz=(STOP_X, 0.0, BASE_T)),
    )
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=-TRAVEL,
            upper=TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_access_cover",
        ArticulationType.FIXED,
        parent=carriage,
        child=access_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    model.articulation(
        "base_to_left_bellows",
        ArticulationType.FIXED,
        parent=base,
        child=left_bellows,
        origin=Origin(xyz=(-BELLOWS_CENTER_X, 0.0, BASE_T)),
    )
    model.articulation(
        "base_to_right_bellows",
        ArticulationType.FIXED,
        parent=base,
        child=right_bellows,
        origin=Origin(xyz=(BELLOWS_CENTER_X, 0.0, BASE_T)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    left_rail = object_model.get_part("left_rail")
    right_rail = object_model.get_part("right_rail")
    left_stop = object_model.get_part("left_stop")
    right_stop = object_model.get_part("right_stop")
    carriage = object_model.get_part("carriage")
    access_cover = object_model.get_part("access_cover")
    left_bellows = object_model.get_part("left_bellows")
    right_bellows = object_model.get_part("right_bellows")

    slide = object_model.get_articulation("base_to_carriage")

    base_body = base.get_visual("base_body")
    left_rail_body = left_rail.get_visual("rail_body")
    right_rail_body = right_rail.get_visual("rail_body")
    left_bearing_roof = carriage.get_visual("left_bearing_roof")
    right_bearing_roof = carriage.get_visual("right_bearing_roof")
    cover_plate = access_cover.get_visual("cover_plate")

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

    ctx.expect_gap(left_rail, base, axis="z", min_gap=0.0, max_gap=0.001, negative_elem=base_body, positive_elem=left_rail_body)
    ctx.expect_gap(right_rail, base, axis="z", min_gap=0.0, max_gap=0.001, negative_elem=base_body, positive_elem=right_rail_body)
    ctx.expect_gap(left_stop, base, axis="z", min_gap=0.0, max_gap=0.001)
    ctx.expect_gap(right_stop, base, axis="z", min_gap=0.0, max_gap=0.001)

    ctx.expect_overlap(carriage, left_rail, axes="x", min_overlap=0.11)
    ctx.expect_overlap(carriage, right_rail, axes="x", min_overlap=0.11)
    ctx.expect_origin_distance(carriage, base, axes="y", max_dist=0.001)
    ctx.expect_gap(
        carriage,
        left_rail,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        positive_elem=left_bearing_roof,
        negative_elem=left_rail_body,
    )
    ctx.expect_gap(
        carriage,
        right_rail,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        positive_elem=right_bearing_roof,
        negative_elem=right_rail_body,
    )

    ctx.expect_gap(access_cover, carriage, axis="z", min_gap=0.0, max_gap=0.001, positive_elem=cover_plate)
    ctx.expect_origin_distance(access_cover, carriage, axes="xy", max_dist=0.001)

    ctx.expect_gap(left_bellows, left_stop, axis="x", min_gap=0.0, max_gap=0.001)
    ctx.expect_gap(carriage, left_bellows, axis="x", max_gap=0.001, max_penetration=1e-6)
    ctx.expect_gap(right_bellows, carriage, axis="x", max_gap=0.001, max_penetration=1e-6)
    ctx.expect_gap(right_stop, right_bellows, axis="x", min_gap=0.0, max_gap=0.001)

    with ctx.pose({slide: -TRAVEL}):
        ctx.expect_gap(carriage, left_stop, axis="x", min_gap=0.006, max_gap=0.012)
        ctx.expect_overlap(carriage, left_rail, axes="x", min_overlap=0.11)
        ctx.expect_overlap(carriage, right_rail, axes="x", min_overlap=0.11)

    with ctx.pose({slide: TRAVEL}):
        ctx.expect_gap(right_stop, carriage, axis="x", min_gap=0.006, max_gap=0.012)
        ctx.expect_overlap(carriage, left_rail, axes="x", min_overlap=0.11)
        ctx.expect_overlap(carriage, right_rail, axes="x", min_overlap=0.11)
        ctx.expect_gap(access_cover, carriage, axis="z", min_gap=0.0, max_gap=0.001, positive_elem=cover_plate)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
