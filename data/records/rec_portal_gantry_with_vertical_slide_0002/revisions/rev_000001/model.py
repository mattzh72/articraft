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

BRIDGE_L = 0.62
BRIDGE_D = 0.16
BRIDGE_H = 0.10
BRIDGE_WALL = 0.008

LEG_CENTER_X = 0.255
LEG_W = 0.09
LEG_D = 0.11
LEG_H = 0.58
LEG_WALL = 0.008

FOOT_L = 0.18
FOOT_D = 0.20
FOOT_H = 0.022

COVER_L = 0.16
COVER_D = 0.09
COVER_T = 0.008

CROSS_PLATE_W = 0.19
CROSS_PLATE_H = 0.14
CROSS_PLATE_T = 0.012

HANGER_T = 0.014
HANGER_TOP_W = 0.22
HANGER_BODY_W = 0.18
HANGER_H = 0.40

RAIL_X = 0.055
RAIL_W = 0.016
RAIL_T = 0.010
RAIL_L = 0.38
RAIL_TOP_OFFSET = 0.055

GUARD_X = 0.079
GUARD_W = 0.010
GUARD_T = 0.006
GUARD_L = 0.40
GUARD_TOP_OFFSET = 0.045

CARRIAGE_W = 0.18
CARRIAGE_T = 0.014
CARRIAGE_H = 0.26
CARRIAGE_Y = 0.055

BEARING_W = 0.034
BEARING_D = 0.036
BEARING_H = 0.060
BEARING_TOP_Z = -0.055
BEARING_LOW_Z = -0.185

TOOL_W = 0.16
TOOL_T = 0.016
TOOL_H = 0.072
TOOL_Y = 0.060

Z_ORIGIN_Z = -0.065
Z_TRAVEL = 0.16


def _cq_mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def _set_box_inertial(
    part,
    size: tuple[float, float, float],
    mass: float,
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.inertial = Inertial.from_geometry(Box(size), mass=mass, origin=Origin(xyz=origin_xyz))


def _bridge_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BRIDGE_L, BRIDGE_D, BRIDGE_H)
    inner = cq.Workplane("XY").box(
        BRIDGE_L - 2.0 * BRIDGE_WALL,
        BRIDGE_D - 2.0 * BRIDGE_WALL,
        BRIDGE_H - 2.0 * BRIDGE_WALL,
    )
    beam = outer.cut(inner)

    recess_positions = (-0.17, 0.17)
    for x in recess_positions:
        recess = cq.Workplane("XY").box(COVER_L - 0.012, COVER_D - 0.012, 0.010).translate(
            (x, 0.0, BRIDGE_H / 2.0 - 0.005)
        )
        beam = beam.cut(recess)

    left_pad = cq.Workplane("XY").box(0.12, 0.12, 0.014).translate((-LEG_CENTER_X, 0.0, -0.043))
    right_pad = cq.Workplane("XY").box(0.12, 0.12, 0.014).translate((LEG_CENTER_X, 0.0, -0.043))
    center_pad = cq.Workplane("XY").box(0.20, 0.09, 0.018).translate((0.0, 0.0, -0.041))
    return beam.union(left_pad).union(right_pad).union(center_pad)


def _bridge_bracket(sign: float) -> cq.Workplane:
    gusset = (
        cq.Workplane("YZ")
        .polyline([(-0.050, 0.0), (0.050, 0.0), (0.0, -0.085)])
        .close()
        .extrude(0.010)
    )
    x_center = sign * (LEG_CENTER_X + LEG_W / 2.0 + 0.004)
    x_shift = x_center - 0.005
    return gusset.translate((x_shift, 0.0, -BRIDGE_H / 2.0))


def _leg_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(LEG_W, LEG_D, LEG_H).translate((0.0, 0.0, -LEG_H / 2.0))
    inner = (
        cq.Workplane("XY")
        .box(LEG_W - 2.0 * LEG_WALL, LEG_D - 2.0 * LEG_WALL, LEG_H - 0.020)
        .translate((0.0, 0.0, -(LEG_H - 0.020) / 2.0 - 0.010))
    )
    top_shoe = cq.Workplane("XY").box(LEG_W + 0.028, LEG_D + 0.018, 0.016).translate((0.0, 0.0, -0.008))
    lower_pad = cq.Workplane("XY").box(LEG_W + 0.014, LEG_D + 0.008, 0.014).translate((0.0, 0.0, -LEG_H + 0.007))
    return outer.cut(inner).union(top_shoe).union(lower_pad)


def _leg_face_plate(front: bool) -> cq.Workplane:
    y = (LEG_D / 2.0 + 0.003) * (1.0 if front else -1.0)
    return cq.Workplane("XY").box(LEG_W - 0.014, 0.006, LEG_H - 0.18).translate((0.0, y, -LEG_H / 2.0 - 0.03))


def _foot_body() -> cq.Workplane:
    foot = cq.Workplane("XY").box(FOOT_L, FOOT_D, FOOT_H).translate((0.0, 0.0, -FOOT_H / 2.0))
    hole_points = [(-0.060, -0.068), (-0.060, 0.068), (0.060, -0.068), (0.060, 0.068)]
    holes = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(0.007)
        .extrude(FOOT_H + 0.010)
        .translate((0.0, 0.0, -FOOT_H - 0.005))
    )
    center_relief = cq.Workplane("XY").box(0.090, 0.100, 0.008).translate((0.0, 0.0, -FOOT_H / 2.0))
    return foot.cut(holes).cut(center_relief)


def _crossbeam_plate() -> cq.Workplane:
    plate = cq.Workplane("XZ").rect(CROSS_PLATE_W, CROSS_PLATE_H).extrude(CROSS_PLATE_T / 2.0, both=True)
    window = cq.Workplane("XZ").rect(0.090, 0.085).extrude((CROSS_PLATE_T + 0.004) / 2.0, both=True)
    hole_points = [(-0.072, -0.045), (-0.072, 0.045), (0.072, -0.045), (0.072, 0.045)]
    holes = (
        cq.Workplane("XZ")
        .pushPoints(hole_points)
        .circle(0.0055)
        .extrude((CROSS_PLATE_T + 0.004) / 2.0, both=True)
    )
    return plate.cut(window).cut(holes)


def _hanger_body() -> cq.Workplane:
    profile = [
        (-HANGER_TOP_W / 2.0, 0.0),
        (HANGER_TOP_W / 2.0, 0.0),
        (HANGER_TOP_W / 2.0, -0.040),
        (HANGER_BODY_W / 2.0, -0.040),
        (HANGER_BODY_W / 2.0, -HANGER_H),
        (-HANGER_BODY_W / 2.0, -HANGER_H),
        (-HANGER_BODY_W / 2.0, -0.040),
        (-HANGER_TOP_W / 2.0, -0.040),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(HANGER_T / 2.0, both=True)
    upper_window = cq.Workplane("XZ").rect(0.060, 0.080).extrude((HANGER_T + 0.004) / 2.0, both=True).translate((0.0, 0.0, -0.11))
    lower_window = cq.Workplane("XZ").rect(0.070, 0.110).extrude((HANGER_T + 0.004) / 2.0, both=True).translate((0.0, 0.0, -0.26))
    return body.cut(upper_window).cut(lower_window)


def _guide_rail_body() -> cq.Workplane:
    rail = cq.Workplane("XY").box(RAIL_W, RAIL_T, RAIL_L)
    recesses = cq.Workplane("XZ").pushPoints(
        [(0.0, z) for z in (-0.130, -0.065, 0.0, 0.065, 0.130)]
    ).circle(0.0045).extrude(0.004)
    recesses = recesses.translate((0.0, RAIL_T / 2.0 - 0.002, 0.0))
    return rail.cut(recesses)


def _guard_strip_body() -> cq.Workplane:
    return cq.Workplane("XY").box(GUARD_W, GUARD_T, GUARD_L)


def _carriage_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(CARRIAGE_W, CARRIAGE_T, CARRIAGE_H).translate((0.0, CARRIAGE_Y, -CARRIAGE_H / 2.0))
    upper_window = cq.Workplane("XY").box(0.070, CARRIAGE_T + 0.004, 0.060).translate((0.0, CARRIAGE_Y, -0.080))
    lower_window = cq.Workplane("XY").box(0.060, CARRIAGE_T + 0.004, 0.055).translate((0.0, CARRIAGE_Y, -0.175))
    lower_nose = cq.Workplane("XY").box(0.090, 0.032, 0.028).translate((0.0, 0.040, -CARRIAGE_H - 0.014))
    nut_block = cq.Workplane("XY").box(0.052, 0.038, 0.082).translate((0.0, 0.034, -0.125))
    return plate.cut(upper_window).cut(lower_window).union(lower_nose).union(nut_block)


def _bearing_block(x_pos: float, z_pos: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(BEARING_W, BEARING_D, BEARING_H).translate((x_pos, 0.030, z_pos))
    rail_clear = cq.Workplane("XY").box(RAIL_W + 0.004, 0.026, BEARING_H + 0.010).translate((x_pos, 0.019, z_pos))
    fastener_relief = cq.Workplane("YZ").pushPoints([(-0.014, -0.018), (-0.014, 0.018)]).circle(0.0032).extrude(0.008)
    fastener_relief = fastener_relief.translate((x_pos + BEARING_W / 2.0 - 0.004, 0.0, z_pos))
    return body.cut(rail_clear).cut(fastener_relief)


def _tool_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TOOL_W, TOOL_T, TOOL_H).translate((0.0, TOOL_Y, -TOOL_H / 2.0))
    slot_points = [(-0.045, 0.0), (0.0, 0.0), (0.045, 0.0)]
    slots = (
        cq.Workplane("XZ")
        .pushPoints(slot_points)
        .rect(0.016, 0.040)
        .extrude((TOOL_T + 0.004) / 2.0, both=True)
        .translate((0.0, TOOL_Y, -TOOL_H / 2.0))
    )
    side_cheeks = (
        cq.Workplane("XY").box(0.020, 0.036, TOOL_H - 0.010).translate((-0.070, 0.045, -TOOL_H / 2.0))
        .union(cq.Workplane("XY").box(0.020, 0.036, TOOL_H - 0.010).translate((0.070, 0.045, -TOOL_H / 2.0)))
    )
    return plate.cut(slots).union(side_cheeks)


def _access_cover() -> cq.Workplane:
    plate = cq.Workplane("XY").box(COVER_L, COVER_D, COVER_T)
    holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.058, -0.030), (-0.058, 0.030), (0.058, -0.030), (0.058, 0.030)])
        .circle(0.0035)
        .extrude(COVER_T + 0.004)
        .translate((0.0, 0.0, -COVER_T / 2.0 - 0.002))
    )
    return plate.cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_vertical_slide", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.26, 0.29, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    zinc_plate = model.material("zinc_plate", rgba=(0.71, 0.73, 0.76, 1.0))

    bridge = model.part("bridge")
    bridge.visual(_cq_mesh(_bridge_shell(), "bridge_shell.obj"), material=painted_steel, name="bridge_shell")
    bridge.visual(_cq_mesh(_bridge_bracket(-1.0), "bridge_left_bracket.obj"), material=machined_steel, name="left_bridge_bracket")
    bridge.visual(_cq_mesh(_bridge_bracket(1.0), "bridge_right_bracket.obj"), material=machined_steel, name="right_bridge_bracket")
    _set_box_inertial(bridge, (BRIDGE_L, BRIDGE_D, BRIDGE_H), mass=18.0)

    left_leg = model.part("left_leg")
    left_leg.visual(_cq_mesh(_leg_shell(), "left_leg_shell.obj"), material=painted_steel, name="leg_shell")
    left_leg.visual(_cq_mesh(_leg_face_plate(True), "left_leg_front_plate.obj"), material=machined_steel, name="front_face_plate")
    left_leg.visual(_cq_mesh(_leg_face_plate(False), "left_leg_rear_plate.obj"), material=machined_steel, name="rear_face_plate")
    _set_box_inertial(left_leg, (LEG_W + 0.028, LEG_D + 0.018, LEG_H), mass=9.5, origin_xyz=(0.0, 0.0, -LEG_H / 2.0))

    right_leg = model.part("right_leg")
    right_leg.visual(_cq_mesh(_leg_shell(), "right_leg_shell.obj"), material=painted_steel, name="leg_shell")
    right_leg.visual(_cq_mesh(_leg_face_plate(True), "right_leg_front_plate.obj"), material=machined_steel, name="front_face_plate")
    right_leg.visual(_cq_mesh(_leg_face_plate(False), "right_leg_rear_plate.obj"), material=machined_steel, name="rear_face_plate")
    _set_box_inertial(right_leg, (LEG_W + 0.028, LEG_D + 0.018, LEG_H), mass=9.5, origin_xyz=(0.0, 0.0, -LEG_H / 2.0))

    left_foot = model.part("left_foot")
    left_foot.visual(_cq_mesh(_foot_body(), "left_foot.obj"), material=machined_steel, name="foot_plate")
    _set_box_inertial(left_foot, (FOOT_L, FOOT_D, FOOT_H), mass=3.2, origin_xyz=(0.0, 0.0, -FOOT_H / 2.0))

    right_foot = model.part("right_foot")
    right_foot.visual(_cq_mesh(_foot_body(), "right_foot.obj"), material=machined_steel, name="foot_plate")
    _set_box_inertial(right_foot, (FOOT_L, FOOT_D, FOOT_H), mass=3.2, origin_xyz=(0.0, 0.0, -FOOT_H / 2.0))

    front_crossbeam_plate = model.part("front_crossbeam_plate")
    front_crossbeam_plate.visual(
        _cq_mesh(_crossbeam_plate(), "front_crossbeam_plate.obj"),
        material=black_oxide,
        name="plate_body",
    )
    _set_box_inertial(front_crossbeam_plate, (CROSS_PLATE_W, CROSS_PLATE_T, CROSS_PLATE_H), mass=1.1)

    rear_crossbeam_plate = model.part("rear_crossbeam_plate")
    rear_crossbeam_plate.visual(
        _cq_mesh(_crossbeam_plate(), "rear_crossbeam_plate.obj"),
        material=black_oxide,
        name="plate_body",
    )
    _set_box_inertial(rear_crossbeam_plate, (CROSS_PLATE_W, CROSS_PLATE_T, CROSS_PLATE_H), mass=1.1)

    rail_backplate = model.part("rail_backplate")
    rail_backplate.visual(_cq_mesh(_hanger_body(), "rail_backplate.obj"), material=machined_steel, name="hanger_body")
    _set_box_inertial(
        rail_backplate,
        (HANGER_TOP_W, 0.090, HANGER_H),
        mass=7.0,
        origin_xyz=(0.0, 0.0, -HANGER_H / 2.0),
    )

    left_guide_rail = model.part("left_guide_rail")
    left_guide_rail.visual(_cq_mesh(_guide_rail_body(), "left_guide_rail.obj"), material=zinc_plate, name="rail_body")
    _set_box_inertial(left_guide_rail, (RAIL_W, RAIL_T, RAIL_L), mass=0.9)

    right_guide_rail = model.part("right_guide_rail")
    right_guide_rail.visual(_cq_mesh(_guide_rail_body(), "right_guide_rail.obj"), material=zinc_plate, name="rail_body")
    _set_box_inertial(right_guide_rail, (RAIL_W, RAIL_T, RAIL_L), mass=0.9)

    left_guard_strip = model.part("left_guard_strip")
    left_guard_strip.visual(_cq_mesh(_guard_strip_body(), "left_guard_strip.obj"), material=black_oxide, name="guard_strip")
    _set_box_inertial(left_guard_strip, (GUARD_W, GUARD_T, GUARD_L), mass=0.35)

    right_guard_strip = model.part("right_guard_strip")
    right_guard_strip.visual(_cq_mesh(_guard_strip_body(), "right_guard_strip.obj"), material=black_oxide, name="guard_strip")
    _set_box_inertial(right_guard_strip, (GUARD_W, GUARD_T, GUARD_L), mass=0.35)

    z_carriage = model.part("z_carriage")
    z_carriage.visual(_cq_mesh(_carriage_plate(), "z_carriage_plate.obj"), material=painted_steel, name="carriage_plate")
    z_carriage.visual(
        _cq_mesh(_bearing_block(-RAIL_X, BEARING_TOP_Z), "z_carriage_upper_left_bearing.obj"),
        material=black_oxide,
        name="upper_left_bearing",
    )
    z_carriage.visual(
        _cq_mesh(_bearing_block(-RAIL_X, BEARING_LOW_Z), "z_carriage_lower_left_bearing.obj"),
        material=black_oxide,
        name="lower_left_bearing",
    )
    z_carriage.visual(
        _cq_mesh(_bearing_block(RAIL_X, BEARING_TOP_Z), "z_carriage_upper_right_bearing.obj"),
        material=black_oxide,
        name="upper_right_bearing",
    )
    z_carriage.visual(
        _cq_mesh(_bearing_block(RAIL_X, BEARING_LOW_Z), "z_carriage_lower_right_bearing.obj"),
        material=black_oxide,
        name="lower_right_bearing",
    )
    _set_box_inertial(
        z_carriage,
        (CARRIAGE_W, 0.080, CARRIAGE_H + 0.028),
        mass=6.2,
        origin_xyz=(0.0, 0.040, -0.145),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(_cq_mesh(_tool_plate(), "tool_plate.obj"), material=machined_steel, name="tool_plate")
    _set_box_inertial(tool_plate, (TOOL_W, 0.036, TOOL_H), mass=1.6, origin_xyz=(0.0, 0.050, -TOOL_H / 2.0))

    left_access_cover = model.part("left_access_cover")
    left_access_cover.visual(_cq_mesh(_access_cover(), "left_access_cover.obj"), material=zinc_plate, name="cover_plate")
    _set_box_inertial(left_access_cover, (COVER_L, COVER_D, COVER_T), mass=0.45)

    right_access_cover = model.part("right_access_cover")
    right_access_cover.visual(_cq_mesh(_access_cover(), "right_access_cover.obj"), material=zinc_plate, name="cover_plate")
    _set_box_inertial(right_access_cover, (COVER_L, COVER_D, COVER_T), mass=0.45)

    model.articulation(
        "bridge_to_left_leg",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_leg,
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, -BRIDGE_H / 2.0)),
    )
    model.articulation(
        "bridge_to_right_leg",
        ArticulationType.FIXED,
        parent=bridge,
        child=right_leg,
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, -BRIDGE_H / 2.0)),
    )
    model.articulation(
        "left_leg_to_left_foot",
        ArticulationType.FIXED,
        parent=left_leg,
        child=left_foot,
        origin=Origin(xyz=(0.0, 0.0, -LEG_H)),
    )
    model.articulation(
        "right_leg_to_right_foot",
        ArticulationType.FIXED,
        parent=right_leg,
        child=right_foot,
        origin=Origin(xyz=(0.0, 0.0, -LEG_H)),
    )
    model.articulation(
        "bridge_to_front_crossbeam_plate",
        ArticulationType.FIXED,
        parent=bridge,
        child=front_crossbeam_plate,
        origin=Origin(xyz=(0.0, BRIDGE_D / 2.0 + CROSS_PLATE_T / 2.0, -0.005)),
    )
    model.articulation(
        "bridge_to_rear_crossbeam_plate",
        ArticulationType.FIXED,
        parent=bridge,
        child=rear_crossbeam_plate,
        origin=Origin(xyz=(0.0, -BRIDGE_D / 2.0 - CROSS_PLATE_T / 2.0, -0.005)),
    )
    model.articulation(
        "bridge_to_rail_backplate",
        ArticulationType.FIXED,
        parent=bridge,
        child=rail_backplate,
        origin=Origin(xyz=(0.0, 0.0, -BRIDGE_H / 2.0)),
    )
    model.articulation(
        "rail_backplate_to_left_guide_rail",
        ArticulationType.FIXED,
        parent=rail_backplate,
        child=left_guide_rail,
        origin=Origin(
            xyz=(
                -RAIL_X,
                HANGER_T / 2.0 + RAIL_T / 2.0,
                -RAIL_TOP_OFFSET - RAIL_L / 2.0,
            )
        ),
    )
    model.articulation(
        "rail_backplate_to_right_guide_rail",
        ArticulationType.FIXED,
        parent=rail_backplate,
        child=right_guide_rail,
        origin=Origin(
            xyz=(
                RAIL_X,
                HANGER_T / 2.0 + RAIL_T / 2.0,
                -RAIL_TOP_OFFSET - RAIL_L / 2.0,
            )
        ),
    )
    model.articulation(
        "rail_backplate_to_left_guard_strip",
        ArticulationType.FIXED,
        parent=rail_backplate,
        child=left_guard_strip,
        origin=Origin(
            xyz=(
                -GUARD_X,
                HANGER_T / 2.0 + GUARD_T / 2.0,
                -GUARD_TOP_OFFSET - GUARD_L / 2.0,
            )
        ),
    )
    model.articulation(
        "rail_backplate_to_right_guard_strip",
        ArticulationType.FIXED,
        parent=rail_backplate,
        child=right_guard_strip,
        origin=Origin(
            xyz=(
                GUARD_X,
                HANGER_T / 2.0 + GUARD_T / 2.0,
                -GUARD_TOP_OFFSET - GUARD_L / 2.0,
            )
        ),
    )
    model.articulation(
        "bridge_to_left_access_cover",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_access_cover,
        origin=Origin(xyz=(-0.17, 0.0, BRIDGE_H / 2.0 - COVER_T / 2.0)),
    )
    model.articulation(
        "bridge_to_right_access_cover",
        ArticulationType.FIXED,
        parent=bridge,
        child=right_access_cover,
        origin=Origin(xyz=(0.17, 0.0, BRIDGE_H / 2.0 - COVER_T / 2.0)),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=rail_backplate,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.0, Z_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.25, lower=0.0, upper=Z_TRAVEL),
    )
    model.articulation(
        "z_carriage_to_tool_plate",
        ArticulationType.FIXED,
        parent=z_carriage,
        child=tool_plate,
        origin=Origin(xyz=(0.0, 0.0, -CARRIAGE_H - 0.028)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bridge = object_model.get_part("bridge")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    front_crossbeam_plate = object_model.get_part("front_crossbeam_plate")
    rear_crossbeam_plate = object_model.get_part("rear_crossbeam_plate")
    rail_backplate = object_model.get_part("rail_backplate")
    left_guide_rail = object_model.get_part("left_guide_rail")
    right_guide_rail = object_model.get_part("right_guide_rail")
    left_guard_strip = object_model.get_part("left_guard_strip")
    right_guard_strip = object_model.get_part("right_guard_strip")
    z_carriage = object_model.get_part("z_carriage")
    tool_plate = object_model.get_part("tool_plate")
    left_access_cover = object_model.get_part("left_access_cover")
    right_access_cover = object_model.get_part("right_access_cover")
    z_slide = object_model.get_articulation("z_slide")

    bridge_shell = bridge.get_visual("bridge_shell")
    left_bridge_bracket = bridge.get_visual("left_bridge_bracket")
    right_bridge_bracket = bridge.get_visual("right_bridge_bracket")
    hanger_body = rail_backplate.get_visual("hanger_body")
    carriage_plate = z_carriage.get_visual("carriage_plate")
    upper_left_bearing = z_carriage.get_visual("upper_left_bearing")
    lower_left_bearing = z_carriage.get_visual("lower_left_bearing")
    upper_right_bearing = z_carriage.get_visual("upper_right_bearing")
    lower_right_bearing = z_carriage.get_visual("lower_right_bearing")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=True, ignore_fixed=True)

    for name, part in (
        ("bridge", bridge),
        ("left_leg", left_leg),
        ("right_leg", right_leg),
        ("left_foot", left_foot),
        ("right_foot", right_foot),
        ("front_crossbeam_plate", front_crossbeam_plate),
        ("rear_crossbeam_plate", rear_crossbeam_plate),
        ("rail_backplate", rail_backplate),
        ("left_guide_rail", left_guide_rail),
        ("right_guide_rail", right_guide_rail),
        ("left_guard_strip", left_guard_strip),
        ("right_guard_strip", right_guard_strip),
        ("z_carriage", z_carriage),
        ("tool_plate", tool_plate),
        ("left_access_cover", left_access_cover),
        ("right_access_cover", right_access_cover),
    ):
        ctx.check(f"{name}_present", part is not None, f"Missing required part: {name}")

    for label, visual in (
        ("bridge_shell_present", bridge_shell),
        ("left_bridge_bracket_present", left_bridge_bracket),
        ("right_bridge_bracket_present", right_bridge_bracket),
        ("hanger_body_present", hanger_body),
        ("carriage_plate_present", carriage_plate),
        ("upper_left_bearing_present", upper_left_bearing),
        ("lower_left_bearing_present", lower_left_bearing),
        ("upper_right_bearing_present", upper_right_bearing),
        ("lower_right_bearing_present", lower_right_bearing),
    ):
        ctx.check(label, visual is not None, f"Missing named detail visual for {label}")

    ctx.expect_contact(left_leg, bridge, name="left_leg_contacts_bridge")
    ctx.expect_contact(right_leg, bridge, name="right_leg_contacts_bridge")
    ctx.expect_contact(left_foot, left_leg, name="left_foot_contacts_left_leg")
    ctx.expect_contact(right_foot, right_leg, name="right_foot_contacts_right_leg")
    ctx.expect_contact(rail_backplate, bridge, name="rail_backplate_contacts_bridge")
    ctx.expect_contact(front_crossbeam_plate, bridge, name="front_crossbeam_plate_contacts_bridge")
    ctx.expect_contact(rear_crossbeam_plate, bridge, name="rear_crossbeam_plate_contacts_bridge")
    ctx.expect_contact(left_guide_rail, rail_backplate, name="left_guide_rail_contacts_backplate")
    ctx.expect_contact(right_guide_rail, rail_backplate, name="right_guide_rail_contacts_backplate")
    ctx.expect_contact(left_guard_strip, rail_backplate, name="left_guard_strip_contacts_backplate")
    ctx.expect_contact(right_guard_strip, rail_backplate, name="right_guard_strip_contacts_backplate")
    ctx.expect_contact(left_access_cover, bridge, name="left_access_cover_seated")
    ctx.expect_contact(right_access_cover, bridge, name="right_access_cover_seated")
    ctx.expect_contact(tool_plate, z_carriage, name="tool_plate_contacts_carriage")

    ctx.expect_origin_gap(right_leg, left_leg, axis="x", min_gap=0.50, max_gap=0.53, name="portal_span_between_legs")
    ctx.expect_origin_distance(rail_backplate, bridge, axes="xy", max_dist=0.001, name="hanger_centered_under_bridge")
    ctx.expect_origin_distance(z_carriage, rail_backplate, axes="x", max_dist=0.001, name="z_carriage_centered_on_guides")
    ctx.expect_overlap(front_crossbeam_plate, bridge, axes="xz", min_overlap=0.12, name="front_plate_overlaps_bridge_face")
    ctx.expect_overlap(rear_crossbeam_plate, bridge, axes="xz", min_overlap=0.12, name="rear_plate_overlaps_bridge_face")
    ctx.expect_within(left_access_cover, bridge, axes="xy", margin=0.03, name="left_access_cover_within_bridge_plan")
    ctx.expect_within(right_access_cover, bridge, axes="xy", margin=0.03, name="right_access_cover_within_bridge_plan")

    with ctx.pose({z_slide: 0.0}):
        ctx.expect_gap(bridge, z_carriage, axis="z", min_gap=0.010, max_gap=0.030, name="carriage_hangs_below_bridge_at_home")
        ctx.expect_gap(z_carriage, left_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="left_guide_clearance_at_home")
        ctx.expect_gap(z_carriage, right_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="right_guide_clearance_at_home")
        ctx.expect_overlap(z_carriage, left_guide_rail, axes="xz", min_overlap=0.015, name="left_rail_tracks_carriage_at_home")
        ctx.expect_overlap(z_carriage, right_guide_rail, axes="xz", min_overlap=0.015, name="right_rail_tracks_carriage_at_home")

    with ctx.pose({z_slide: Z_TRAVEL * 0.5}):
        ctx.expect_gap(z_carriage, left_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="left_guide_clearance_midstroke")
        ctx.expect_gap(z_carriage, right_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="right_guide_clearance_midstroke")
        ctx.expect_overlap(z_carriage, left_guide_rail, axes="xz", min_overlap=0.015, name="left_rail_tracks_carriage_midstroke")
        ctx.expect_overlap(z_carriage, right_guide_rail, axes="xz", min_overlap=0.015, name="right_rail_tracks_carriage_midstroke")

    with ctx.pose({z_slide: Z_TRAVEL}):
        ctx.expect_gap(bridge, z_carriage, axis="z", min_gap=0.15, name="carriage_retracts_downward_at_full_stroke")
        ctx.expect_gap(z_carriage, left_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="left_guide_clearance_fullstroke")
        ctx.expect_gap(z_carriage, right_guide_rail, axis="y", min_gap=0.010, max_gap=0.030, name="right_guide_clearance_fullstroke")
        ctx.expect_overlap(z_carriage, left_guide_rail, axes="xz", min_overlap=0.015, name="left_rail_tracks_carriage_fullstroke")
        ctx.expect_overlap(z_carriage, right_guide_rail, axes="xz", min_overlap=0.015, name="right_rail_tracks_carriage_fullstroke")

    home_pos = None
    fullstroke_pos = None
    with ctx.pose({z_slide: 0.0}):
        home_pos = ctx.part_world_position(z_carriage)
    with ctx.pose({z_slide: Z_TRAVEL}):
        fullstroke_pos = ctx.part_world_position(z_carriage)
    travel_ok = (
        home_pos is not None
        and fullstroke_pos is not None
        and abs((home_pos[2] - fullstroke_pos[2]) - Z_TRAVEL) <= 1e-6
        and abs(home_pos[0] - fullstroke_pos[0]) <= 1e-6
        and abs(home_pos[1] - fullstroke_pos[1]) <= 1e-6
    )
    ctx.check(
        "z_slide_moves_only_vertically",
        travel_ok,
        f"Expected pure vertical prismatic travel of {Z_TRAVEL:.3f} m, got home={home_pos}, fullstroke={fullstroke_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
