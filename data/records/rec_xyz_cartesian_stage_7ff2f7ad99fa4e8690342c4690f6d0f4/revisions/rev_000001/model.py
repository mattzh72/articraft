from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.78
BASE_Y = 0.42
BASE_T = 0.028
BASE_WINDOW_X = 0.50
BASE_WINDOW_Y = 0.22
PEDESTAL_W = 0.050
PEDESTAL_H = 0.030
RAIL_SPAN = 0.290
RAIL_LEN = 0.660
RAIL_W = 0.022
RAIL_H = 0.015

TRUCK_X = 0.105
TRUCK_Y = 0.066
TRUCK_Z = 0.038
BRIDGE_ORIGIN_Z = BASE_T + PEDESTAL_H + RAIL_H + (TRUCK_Z / 2.0)
BRIDGE_TRAVEL = 0.220

BRIDGE_BEAM_X = 0.128
BRIDGE_BEAM_Y = 0.314
BRIDGE_BEAM_Z = 0.080
BRIDGE_BEAM_CENTER_Z = 0.205
UPRIGHT_X = 0.088
UPRIGHT_Y = 0.058
UPRIGHT_Z = 0.150
UPRIGHT_CENTER_Z = 0.075
Y_RAIL_X = 0.018
Y_RAIL_Y = 0.260
Y_RAIL_Z = 0.030
Y_RAIL_CENTER_X = (BRIDGE_BEAM_X / 2.0) + (Y_RAIL_X / 2.0)
Y_RAIL_CENTER_Z = BRIDGE_BEAM_CENTER_Z
Y_TRAVEL = 0.080

CARRIAGE_MAIN_X = 0.028
CARRIAGE_MAIN_Y = 0.118
CARRIAGE_MAIN_Z = 0.072
CARRIAGE_MAIN_CENTER_X = 0.023
CARRIAGE_GUIDE_X = 0.050
CARRIAGE_GUIDE_Y = 0.094
CARRIAGE_GUIDE_Z = 0.190
CARRIAGE_GUIDE_CENTER_X = 0.063
CARRIAGE_GUIDE_CENTER_Z = 0.012

RAM_X = 0.044
RAM_Y = 0.060
RAM_Z = 0.235
FLANGE_R = 0.032
FLANGE_T = 0.014
FLANGE_PILOT_R = 0.010
FLANGE_PILOT_T = 0.012
Z_JOINT_X = CARRIAGE_GUIDE_CENTER_X + (CARRIAGE_GUIDE_X / 2.0)
Z_JOINT_Z = 0.108
Z_TRAVEL = 0.080


def _rect_points(span_x: float, span_y: float) -> list[tuple[float, float]]:
    hx = span_x / 2.0
    hy = span_y / 2.0
    return [(-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)]


def _make_base_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T).translate((0.0, 0.0, BASE_T / 2.0))
    central_window = (
        cq.Workplane("XY")
        .box(BASE_WINDOW_X, BASE_WINDOW_Y, BASE_T + 0.004)
        .translate((0.0, 0.0, BASE_T / 2.0))
    )
    top_relief = (
        cq.Workplane("XY")
        .box(BASE_X - 0.16, BASE_Y - 0.10, 0.010)
        .translate((0.0, 0.0, BASE_T - 0.005))
    )
    pedestal = cq.Workplane("XY").box(RAIL_LEN + 0.040, PEDESTAL_W, PEDESTAL_H)
    frame = frame.cut(central_window).cut(top_relief)
    frame = frame.union(pedestal.translate((0.0, RAIL_SPAN / 2.0, BASE_T + (PEDESTAL_H / 2.0))))
    frame = frame.union(pedestal.translate((0.0, -RAIL_SPAN / 2.0, BASE_T + (PEDESTAL_H / 2.0))))
    return frame


def _make_linear_rail() -> cq.Workplane:
    hole_x = 0.115
    rail = cq.Workplane("XY").box(RAIL_LEN, RAIL_W, RAIL_H)
    return (
        rail.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-2.0 * hole_x, 0.0), (-hole_x, 0.0), (0.0, 0.0), (hole_x, 0.0), (2.0 * hole_x, 0.0)])
        .circle(0.0035)
        .cutBlind(-0.006)
    )


def _make_truck() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TRUCK_X, TRUCK_Y, TRUCK_Z)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_rect_points(TRUCK_X * 0.46, TRUCK_Y * 0.48))
        .circle(0.004)
        .cutBlind(-0.006)
    )


def _make_bridge_beam() -> cq.Workplane:
    main_beam = (
        cq.Workplane("XY")
        .box(BRIDGE_BEAM_X, BRIDGE_BEAM_Y, BRIDGE_BEAM_Z)
        .translate((0.0, 0.0, BRIDGE_BEAM_CENTER_Z))
    )
    uprights = (
        cq.Workplane("XY")
        .box(UPRIGHT_X, UPRIGHT_Y, UPRIGHT_Z)
        .translate((0.0, RAIL_SPAN / 2.0, UPRIGHT_CENTER_Z))
        .union(
            cq.Workplane("XY")
            .box(UPRIGHT_X, UPRIGHT_Y, UPRIGHT_Z)
            .translate((0.0, -RAIL_SPAN / 2.0, UPRIGHT_CENTER_Z))
        )
    )
    front_pocket = (
        cq.Workplane("XY")
        .box(0.030, BRIDGE_BEAM_Y - 0.080, BRIDGE_BEAM_Z - 0.028)
        .translate(((BRIDGE_BEAM_X / 2.0) - 0.015, 0.0, BRIDGE_BEAM_CENTER_Z))
    )
    underside_window = (
        cq.Workplane("XY")
        .box(BRIDGE_BEAM_X - 0.040, BRIDGE_BEAM_Y - 0.160, 0.060)
        .translate((0.0, 0.0, 0.112))
    )
    return main_beam.union(uprights).cut(front_pocket).cut(underside_window)


def _make_y_rail() -> cq.Workplane:
    rail = cq.Workplane("XY").box(Y_RAIL_X, Y_RAIL_Y, Y_RAIL_Z)
    return (
        rail.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.080, 0.0), (0.0, 0.0), (0.080, 0.0)])
        .circle(0.003)
        .cutBlind(-0.005)
    )


def _make_carriage_body() -> cq.Workplane:
    y_block = (
        cq.Workplane("XY")
        .box(CARRIAGE_MAIN_X, CARRIAGE_MAIN_Y, CARRIAGE_MAIN_Z)
        .translate((CARRIAGE_MAIN_CENTER_X, 0.0, 0.0))
    )
    z_housing = (
        cq.Workplane("XY")
        .box(CARRIAGE_GUIDE_X, CARRIAGE_GUIDE_Y, CARRIAGE_GUIDE_Z)
        .translate((CARRIAGE_GUIDE_CENTER_X, 0.0, CARRIAGE_GUIDE_CENTER_Z))
    )
    upper_cheek = (
        cq.Workplane("XY")
        .box(0.022, 0.074, 0.036)
        .translate((0.042, 0.0, 0.050))
    )
    lower_cheek = (
        cq.Workplane("XY")
        .box(0.022, 0.074, 0.036)
        .translate((0.042, 0.0, -0.046))
    )
    front_relief = (
        cq.Workplane("XY")
        .box(0.018, 0.050, 0.090)
        .translate((CARRIAGE_GUIDE_CENTER_X + 0.010, 0.0, CARRIAGE_GUIDE_CENTER_Z + 0.004))
    )
    bolt_relief = (
        cq.Workplane("XY")
        .box(0.010, 0.070, 0.070)
        .translate((0.030, 0.0, 0.0))
    )
    return y_block.union(z_housing).union(upper_cheek).union(lower_cheek).cut(front_relief).cut(bolt_relief)


def _make_ram_body() -> cq.Workplane:
    ram = cq.Workplane("XY").box(RAM_X, RAM_Y, RAM_Z).translate((RAM_X / 2.0, 0.0, -(RAM_Z / 2.0)))
    front_flat = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.120)
        .translate((RAM_X - 0.007, 0.0, -0.100))
    )
    return ram.cut(front_flat)


def _make_mounting_flange() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(FLANGE_R).extrude(FLANGE_T)
    flange = (
        flange.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_rect_points(0.032, 0.032))
        .circle(0.0028)
        .cutBlind(-FLANGE_T)
    )
    pilot = cq.Workplane("XY").circle(FLANGE_PILOT_R).extrude(FLANGE_PILOT_T).translate((0.0, 0.0, -FLANGE_PILOT_T))
    return flange.union(pilot).translate((RAM_X / 2.0, 0.0, -(RAM_Z + FLANGE_T)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_automation_stage")

    base_gray = model.material("base_gray", color=(0.32, 0.34, 0.37, 1.0))
    rail_black = model.material("rail_black", color=(0.17, 0.18, 0.19, 1.0))
    aluminum = model.material("aluminum", color=(0.76, 0.78, 0.80, 1.0))
    carriage_black = model.material("carriage_black", color=(0.12, 0.13, 0.15, 1.0))
    stainless = model.material("stainless", color=(0.82, 0.84, 0.86, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_frame(), "base_frame"), material=base_gray, name="base_frame")
    base.visual(
        mesh_from_cadquery(_make_linear_rail(), "left_rail"),
        origin=Origin(xyz=(0.0, RAIL_SPAN / 2.0, BASE_T + PEDESTAL_H + (RAIL_H / 2.0))),
        material=rail_black,
        name="left_rail",
    )
    base.visual(
        mesh_from_cadquery(_make_linear_rail(), "right_rail"),
        origin=Origin(xyz=(0.0, -(RAIL_SPAN / 2.0), BASE_T + PEDESTAL_H + (RAIL_H / 2.0))),
        material=rail_black,
        name="right_rail",
    )

    bridge = model.part("bridge_saddle")
    bridge.visual(
        mesh_from_cadquery(_make_truck(), "left_truck"),
        origin=Origin(xyz=(0.0, RAIL_SPAN / 2.0, 0.0)),
        material=carriage_black,
        name="left_truck",
    )
    bridge.visual(
        mesh_from_cadquery(_make_truck(), "right_truck"),
        origin=Origin(xyz=(0.0, -(RAIL_SPAN / 2.0), 0.0)),
        material=carriage_black,
        name="right_truck",
    )
    bridge.visual(mesh_from_cadquery(_make_bridge_beam(), "bridge_beam"), material=aluminum, name="bridge_beam")
    bridge.visual(
        mesh_from_cadquery(_make_y_rail(), "bridge_y_rail"),
        origin=Origin(xyz=(Y_RAIL_CENTER_X, 0.0, Y_RAIL_CENTER_Z)),
        material=rail_black,
        name="y_rail",
    )

    carriage = model.part("cross_carriage")
    carriage.visual(mesh_from_cadquery(_make_carriage_body(), "cross_carriage_body"), material=carriage_black, name="carriage_body")

    z_ram = model.part("z_ram")
    z_ram.visual(mesh_from_cadquery(_make_ram_body(), "z_ram_body"), material=aluminum, name="ram_body")
    z_ram.visual(mesh_from_cadquery(_make_mounting_flange(), "mounting_flange"), material=stainless, name="mounting_flange")

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40, lower=-BRIDGE_TRAVEL, upper=BRIDGE_TRAVEL),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(Y_RAIL_CENTER_X, 0.0, Y_RAIL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=-Y_TRAVEL, upper=Y_TRAVEL),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_ram,
        origin=Origin(xyz=(Z_JOINT_X, 0.0, Z_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=Z_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge_saddle")
    carriage = object_model.get_part("cross_carriage")
    z_ram = object_model.get_part("z_ram")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

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

    for part_name in ("base", "bridge_saddle", "cross_carriage", "z_ram"):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None, f"missing part {part_name}")

    for joint_name in ("x_axis", "y_axis", "z_axis"):
        ctx.check(
            f"{joint_name}_present",
            object_model.get_articulation(joint_name) is not None,
            f"missing articulation {joint_name}",
        )

    ctx.check("x_axis_direction", tuple(x_axis.axis) == (1.0, 0.0, 0.0), f"axis was {x_axis.axis}")
    ctx.check("y_axis_direction", tuple(y_axis.axis) == (0.0, 1.0, 0.0), f"axis was {y_axis.axis}")
    ctx.check("z_axis_direction", tuple(z_axis.axis) == (0.0, 0.0, -1.0), f"axis was {z_axis.axis}")

    ctx.expect_contact(base, bridge, elem_a="left_rail", elem_b="left_truck", name="left_truck_contacts_left_rail")
    ctx.expect_contact(base, bridge, elem_a="right_rail", elem_b="right_truck", name="right_truck_contacts_right_rail")
    ctx.expect_contact(bridge, carriage, elem_a="y_rail", elem_b="carriage_body", name="carriage_contacts_y_rail")
    ctx.expect_contact(carriage, z_ram, elem_a="carriage_body", elem_b="ram_body", name="ram_contacts_guide_housing")
    ctx.expect_overlap(
        z_ram,
        carriage,
        axes="y",
        elem_a="ram_body",
        elem_b="carriage_body",
        min_overlap=0.050,
        name="ram_aligned_with_carriage_in_y",
    )
    ctx.expect_gap(
        z_ram,
        carriage,
        axis="x",
        positive_elem="ram_body",
        negative_elem="carriage_body",
        max_gap=0.001,
        max_penetration=0.001,
        name="ram_front_face_stays_flush_to_carriage",
    )
    ctx.expect_gap(carriage, z_ram, axis="z", negative_elem="mounting_flange", min_gap=0.015, name="mounting_flange_hangs_below_carriage")

    def _check_axis_motion(name: str, moving_part, joint, commanded: float, expected: tuple[float, float, float]) -> None:
        rest = ctx.part_world_position(moving_part)
        with ctx.pose({joint: commanded}):
            moved = ctx.part_world_position(moving_part)
        if rest is None or moved is None:
            ctx.fail(name, "missing world position for motion check")
            return
        delta = tuple(moved[i] - rest[i] for i in range(3))
        ok = all(abs(delta[i] - expected[i]) <= 1e-6 for i in range(3))
        ctx.check(name, ok, f"expected delta {expected}, got {delta}")

    _check_axis_motion("bridge_moves_only_along_x", bridge, x_axis, 0.090, (0.090, 0.0, 0.0))
    _check_axis_motion("carriage_moves_only_along_y", carriage, y_axis, 0.050, (0.0, 0.050, 0.0))
    _check_axis_motion("ram_moves_only_down_z", z_ram, z_axis, 0.040, (0.0, 0.0, -0.040))

    with ctx.pose({x_axis: -BRIDGE_TRAVEL}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="left_truck",
            outer_elem="left_rail",
            margin=0.0,
            name="left_truck_supported_at_x_min",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="right_truck",
            outer_elem="right_rail",
            margin=0.0,
            name="right_truck_supported_at_x_min",
        )

    with ctx.pose({x_axis: BRIDGE_TRAVEL}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="left_truck",
            outer_elem="left_rail",
            margin=0.0,
            name="left_truck_supported_at_x_max",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="right_truck",
            outer_elem="right_rail",
            margin=0.0,
            name="right_truck_supported_at_x_max",
        )

    with ctx.pose({y_axis: -Y_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="y",
            elem_a="carriage_body",
            elem_b="y_rail",
            min_overlap=0.100,
            name="carriage_retains_y_overlap_at_min",
        )

    with ctx.pose({y_axis: Y_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="y",
            elem_a="carriage_body",
            elem_b="y_rail",
            min_overlap=0.100,
            name="carriage_retains_y_overlap_at_max",
        )

    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_overlap(
            z_ram,
            carriage,
            axes="y",
            elem_a="ram_body",
            elem_b="carriage_body",
            min_overlap=0.050,
            name="ram_stays_captured_at_z_max",
        )
        ctx.expect_gap(
            z_ram,
            carriage,
            axis="x",
            positive_elem="ram_body",
            negative_elem="carriage_body",
            max_gap=0.001,
            max_penetration=0.001,
            name="ram_front_face_stays_flush_at_z_max",
        )
        ctx.expect_gap(
            carriage,
            z_ram,
            axis="z",
            negative_elem="mounting_flange",
            min_gap=0.080,
            name="mounting_flange_extends_below_carriage_at_z_max",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
